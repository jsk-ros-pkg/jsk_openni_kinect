#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
//#include <kdl/frames.hpp>

//#include <OpenNI.h>
//#include <XnCodecIDs.h>
//#include <XnCppWrapper.h>
#include <NiTE.h>

using std::string;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}


void updateUserState(const nite::UserData& user, unsigned long long ts)
{
	if (user.isNew())
		USER_MESSAGE("New")
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		USER_MESSAGE("Visible")
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		USER_MESSAGE("Out of Scene")
	else if (user.isLost())
		USER_MESSAGE("Lost")

	g_visibleUsers[user.getId()] = user.isVisible();


	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh, nh_priv("~");

    nite::UserTracker userTracker;
    nite::Status niteRc;

    nite::NiTE::initialize();

	ros::Rate r(30);
    std::string cam_frame_name;
    nh_priv.getParam("camera_frame", cam_frame_name);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    const std::string jnames[] = {//This is converted from NiteEnums.h
    	"JOINT_HEAD",
    	"JOINT_NECK",
    	"JOINT_LEFT_SHOULDER",
    	"JOINT_RIGHT_SHOULDER",
    	"JOINT_LEFT_ELBOW",
    	"JOINT_RIGHT_ELBOW",
    	"JOINT_LEFT_HAND",
    	"JOINT_RIGHT_HAND",
    	"JOINT_TORSO",
    	"JOINT_LEFT_HIP",
    	"JOINT_RIGHT_HIP",
    	"JOINT_LEFT_KNEE",
    	"JOINT_RIGHT_KNEE",
    	"JOINT_LEFT_FOOT",
    	"JOINT_RIGHT_FOOT",
    };
    const std::vector<std::string> jnamelist(jnames,jnames+15);

	niteRc = userTracker.create();
	if (niteRc != nite::STATUS_OK)
	{
		printf("Couldn't create user tracker\n");
		return 3;
	}
	printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

	nite::UserTrackerFrameRef userTrackerFrame;

	while (ros::ok())
	{
		niteRc = userTracker.readFrame(&userTrackerFrame);
		if (niteRc == nite::STATUS_OK)
		{
			const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
			for (int i = 0; i < users.getSize(); ++i)
			{
				const nite::UserData& user = users[i];
				updateUserState(user,userTrackerFrame.getTimestamp());
				if (user.isNew())
				{
					userTracker.startSkeletonTracking(user.getId());
					ROS_INFO_STREAM("Found a new user.");
				}
				else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
				{
					ROS_INFO_STREAM("Now tracking user " << user.getId());
					for(int i=0;i<jnamelist.size();i++){
						const nite::SkeletonJoint& cur_joint = user.getSkeleton().getJoint(static_cast<nite::JointType>(i));
//						if (cur_joint.getPositionConfidence() > 0.5){
							transform.setOrigin( tf::Vector3(cur_joint.getPosition().z/1000.0, cur_joint.getPosition().x/1000.0, cur_joint.getPosition().y/1000.0) );//XtionData = Left:X, Up:Y, Forward:Z
							q.setRPY(0, 0, M_PI);// Assuming human is in front of and face to camera device (Human's forward:X Left:Y Up:Z)
							transform.setRotation(q);
						    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), cam_frame_name, jnamelist[i]));
//						}
					}
				}
			}
		}
		else
		{
			ROS_WARN_STREAM("Get next frame failed.");
		}

		r.sleep();
	}
	return 0;
}

