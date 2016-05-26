#!/bin/bash -e
set -x
_THIS_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)

# run everythin within /tmp
cd /tmp
rm -fr openni-sensor-primesense* # make sure we start from clean environment
apt-src install libopenni-sensor-primesense-dev
patch -p0 < ${_THIS_DIR}/apt-get-intall-libopenni-sensor-primesense.diff
apt-src build libopenni-sensor-primesense-dev

# install 
ls -al *.deb
sudo dpkg -i libopenni-sensor-primesense*

echo "OK"




