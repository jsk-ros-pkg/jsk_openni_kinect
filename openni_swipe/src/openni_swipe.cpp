// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

ros::Publisher swipe_pub;
//

//NITE
#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <XnVHandPointContext.h>
#include <XnVSessionManager.h>
#include <XnVFlowRouter.h>
#include <XnVSwipeDetector.h>
#include <XnVSteadyDetector.h>

#include "signal_catch.h"

XnVSessionManager* g_pSessionManager = NULL;
XnVFlowRouter* g_pMainFlowRouter;

XnVSwipeDetector* m_pSwipeDetector;
XnVSteadyDetector* m_pSteadyDetector;

XnBool g_bInSession = false;
XnFloat g_fValue = 0.5f;

xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;

XnBool g_bQuit = false;

// Change flow state between steady and swipe
void SetSteadyActive() {g_pMainFlowRouter->SetActive(m_pSteadyDetector);}
void SetSwipeActive() {g_pMainFlowRouter->SetActive(m_pSwipeDetector);}

void Publish_String(std::string str){
  std_msgs::String msg;
  msg.data = str;
  ROS_INFO((str + " published").c_str());
  swipe_pub.publish(msg);
}

// Swipe detector
void XN_CALLBACK_TYPE Swipe_SwipeUp(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  Publish_String("Up");

  SetSteadyActive();
}

void XN_CALLBACK_TYPE Swipe_SwipeDown(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  Publish_String("Down");

  SetSteadyActive();
}

void XN_CALLBACK_TYPE Swipe_SwipeLeft(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  Publish_String("Left");

  SetSteadyActive();
}

void XN_CALLBACK_TYPE Swipe_SwipeRight(XnFloat fVelocity, XnFloat fAngle, void* cxt)
{
  Publish_String("Right");

  SetSteadyActive();
}

// Steady detector
void XN_CALLBACK_TYPE Steady_OnSteady(XnUInt32 nId, XnFloat fVelocity, void* cxt)
{
  ROS_INFO("Steady [%d]", nId);

  SetSwipeActive();
}

void CleanupExit()
{
  if (NULL != g_pSessionManager) {
    delete g_pSessionManager;
    g_pSessionManager = NULL;
  }

  delete g_pMainFlowRouter;
  delete m_pSwipeDetector;
  delete m_pSteadyDetector;

  g_Context.Shutdown();

  exit (1);
}

void XN_CALLBACK_TYPE SessionStart(const XnPoint3D& pFocus, void* UserCxt)
{
  ROS_INFO("hand swipe detect start");
  g_bInSession = true;
  g_pMainFlowRouter->SetActive(m_pSteadyDetector);
}

void XN_CALLBACK_TYPE SessionEnd(void* UserCxt)
{
  g_bInSession = false;
  g_pMainFlowRouter->SetActive(NULL);
}

#define CHECK_RC(rc, what)                                      \
  if (rc != XN_STATUS_OK)                                       \
    {                                                           \
      printf("%s failed: %s\n", what, xnGetStatusString(rc));   \
      return rc;                                                \
    }

#define CHECK_ERRORS(rc, errors, what)          \
  if (rc == XN_STATUS_NO_NODE_PRESENT)          \
    {                                           \
      XnChar strError[1024];                    \
      errors.ToString(strError, 1024);          \
      printf("%s\n", strError);                 \
      return (rc);                              \
    }

int main(int argc, char **argv)
{
  ros::init (argc, argv, "openni_swipe");
  ros::NodeHandle nh;

  swipe_pub = nh.advertise<std_msgs::String>("swipe",10);

  XnStatus rc = XN_STATUS_OK;
  xn::EnumerationErrors errors;
  std::string configfile = ros::package::getPath ("openni_swipe") + "/openni_swipe.xml";

  rc = g_Context.InitFromXmlFile(configfile.c_str());
  CHECK_ERRORS(rc, errors, "InitFromXmlFile");
  CHECK_RC(rc, "InitFromXml");
  rc = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
  CHECK_RC(rc, "Find depth generator");

  // Create and initialize point tracker
  g_pSessionManager = new XnVSessionManager;
  rc = g_pSessionManager->Initialize(&g_Context, "Wave", "RaiseHand");
  if (rc != XN_STATUS_OK)
	{
      printf("Couldn't initialize the Session Manager: %s\n", xnGetStatusString(rc));
      delete g_pSessionManager;
      return rc;
	}
  g_pSessionManager->RegisterSession(NULL, &SessionStart, &SessionEnd);

  // Start catching signals for quit indications
  CatchSignals(&g_bQuit);

  m_pSwipeDetector = new XnVSwipeDetector;
  m_pSteadyDetector = new XnVSteadyDetector;

  // Swipe
  m_pSwipeDetector->RegisterSwipeUp(NULL, &Swipe_SwipeUp);
  m_pSwipeDetector->RegisterSwipeDown(NULL, &Swipe_SwipeDown);
  m_pSwipeDetector->RegisterSwipeLeft(NULL, &Swipe_SwipeLeft);
  m_pSwipeDetector->RegisterSwipeRight(NULL, &Swipe_SwipeRight);
  // Steady
  m_pSteadyDetector->RegisterSteady(NULL, &Steady_OnSteady);

  // Creat the flow manager
  g_pMainFlowRouter = new XnVFlowRouter;

  // Connect flow manager to the point tracker
  g_pSessionManager->AddListener(g_pMainFlowRouter);

  g_Context.StartGeneratingAll();
  ros::Rate r (30);
  while (ros::ok ()){
    // Read next available data
    g_Context.WaitOneUpdateAll(g_DepthGenerator);

    // Process the data
    g_pSessionManager->Update(&g_Context);
    
  }
  CleanupExit();
}
