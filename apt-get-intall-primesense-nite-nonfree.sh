#!/bin/bash -e

_THIS_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)

# run everythin within /tmp
cd /tmp
rm -fr primesense-nite-nonfree* # make sure we start from clean environment
if [ ! -e primesense-nite-nonfree-0.1 ]; then
    apt-src install primesense-nite-nonfree
fi
NITE_LIB=NITE-Bin-Linux-x64-v1.5.2.23.tar.bz2
if [ ! -e ${_THIS_DIR}/${NITE_LIB} ]; then
    echo "Please put $NITE_LIB"
    exit 1
fi
if [ ! -e $NITE_LIB.zip ]; then
    zip $NITE_LIB.zip ${_THIS_DIR}/$NITE_LIB
fi
set -x
# fix to use local .zip
sed -i "s@primesense-nite.tar.bz2.zip@$NITE_LIB.zip@" primesense-nite-nonfree-0.1/update-primesense-nite-nonfree
sed -i "s@^\s*wget@echo wget@"                        primesense-nite-nonfree-0.1/update-primesense-nite-nonfree
sed -i "s@\$checksum@$(sha256sum /tmp/$NITE_LIB.zip | cut -f1 -d\ )@" primesense-nite-nonfree-0.1/update-primesense-nite-nonfree
sed -i "s@/usr/lib/primesense-nite-nonfree/primesense-nite-nonfree-make-deb@/tmp/primesense-nite-nonfree-0.1/primesense-nite-nonfree-make-deb@" primesense-nite-nonfree-0.1/update-primesense-nite-nonfree

sed -i "s@/usr/share/primesense-nite-nonfree/nite_debian_dir@/tmp/primesense-nite-nonfree-0.1/nite_debian_dir@" primesense-nite-nonfree-0.1/primesense-nite-nonfree-make-deb

# create deb
apt-src build primesense-nite-nonfree

# install 
sudo dpkg -i primesense-nite-nonfree_0.1_amd64.deb
sudo dpkg -i /var/cache/primesense-nite-nonfree/openni-module-primesense-nite-nonfree_1.5.2.21-1_amd64.deb

echo "OK"




