#!/bin/bash 
# This script runs once the container is first created  
set -o pipefail

# Create the directory needed to the sshd daemon
mkdir /var/run/sshd
 
# Add docker user and generate a random password with 12 characters that includes at least one capital letter and number.
DOCKER_PASSWORD="saliency"
echo User: docker Password: $DOCKER_PASSWORD
DOCKER_ENCRYPYTED_PASSWORD=`perl -e 'print crypt('"$DOCKER_PASSWORD"', "aa"),"\n"'`
useradd -m -d /home/docker -p $DOCKER_ENCRYPYTED_PASSWORD docker
sed -Ei 's/adm:x:4:/docker:x:4:docker/' /etc/group
adduser docker sudo

# Make saliency code available to docker user
chown -Rf docker:docker /code/saliency

# Set the default shell as bash for docker user.
chsh -s /bin/bash docker

# Check out mbarivision code and build
mkdir /home/docker/code
cd /home/docker/code
git clone https://github.com/mbari-org/avedac-mbarivision.git
cd avedac-mbarivision
autoconf configure.ac > configure
chmod 0755 configure 
echo "export CPPFLAGS='-I/usr/include/libxml2'"  >> /home/docker/.bashrc
echo "export LDFLAGS='-lxml2'"  >> /home/docker/.bashrc
echo "export AVED_BIN=/home/docker/code/avedac-mbarivision/target/build/bin"  >> /home/docker/.bashrc
echo "export PATH=$PATH:$AVED_BIN:/home/docker/code/avedac-mbarivision/src/scripts" >> /home/docker/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib" >> /home/docker/.bashrc
source /home/docker/.bashrc
./configure --with-saliency=/code/saliency --with-xercesc=/usr/local/include/xercesc
chown -Rf docker:docker /home/docker/code
make 
exec "$@"
 

# restarts the xdm service
/etc/init.d/xdm restart

# Start the ssh service
/usr/sbin/sshd -D
