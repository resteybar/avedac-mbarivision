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
  

# Set the default shell as bash for docker user.
chsh -s /bin/bash docker
 
# Make all code accessible by docker user for testing/development
chown -Rf docker:docker /code
echo "export LD_LIBRARY_PATH=/usr/lib" >> /home/docker/.bashrc
echo "export CPPFLAGS='-I/usr/include/libxml2'"  >> /home/docker/.bashrc
echo "export LDFLAGS='-lxml2'"  >> /home/docker/.bashrc
echo "export AVED_BIN=/code/avedac-mbarivision/target/build/bin" >> /home/docker/.bashrc
echo "export PATH=/code/avedac-mbarivision/target/build/bin:$PATH" >> /home/docker/.bashrc 

# restarts the xdm service
/etc/init.d/xdm restart

# Start the ssh service
/usr/sbin/sshd -D
