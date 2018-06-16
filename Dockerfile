FROM ubuntu:16.04
MAINTAINER Danelle Cline <dcline@mbari.org>

# Install
RUN \
  apt-get update -y && \
  apt-get upgrade -y && \
  apt-get install -y subversion git cmake && \
  apt-get install -y gcc-4.7 g++-4.7 gcc-4.7-base && \
  apt-get install -y ffmpeg && \
  apt-get install -y build-essential python-pip && \
  apt-get install -y software-properties-common && \
  apt-get install -y openssh-server && \
  update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.7 100 && \
  update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.7 100 && \
  apt-get install -y byobu curl htop man unzip vim wget xterm fuse flex && \
  apt-get install -y zlib1g-dev libcurl4-openssl-dev libexpat1-dev dh-autoreconf liblapack-dev libxt-dev libpng-dev && \
  apt-get install -y libboost-all-dev qt5-default tclsh freeglut3-dev libjpeg-dev libx11-dev libxext-dev libxml2-dev libtiff-dev && \
  apt-get update

# Set /code as working directory
ENV APP_HOME /code
WORKDIR ${APP_HOME}

# Check out and build OpenCV
ENV CV_VERSION 2.4.11
RUN wget http://sourceforge.net/projects/opencvlibrary/files/opencv-unix/${CV_VERSION}/opencv-${CV_VERSION}.zip/download -O opencv-${CV_VERSION}.zip
RUN unzip opencv-${CV_VERSION}.zip
RUN mkdir /code/opencv-${CV_VERSION}/build
WORKDIR /code/opencv-${CV_VERSION}/build
RUN cmake -D CMAKE_INSTALL_PREFIX=/usr -D CMAKE_BUILD_TYPE=RELEASE -D WITH_TBB=ON -D WITH_V4L=OFF -D INSTALL_C_EXAMPLES=OFF -D INSTALL_PYTHON_EXAMPLES=OFF -D BUILD_EXAMPLES=OFF -D WITH_OPENGL=ON ..
RUN make
RUN make install

# Suppress warning about storing unencrypted password that prompts yes/no answer
RUN mkdir ~/.subversion
RUN echo "[global]" > ~/.subversion/config
RUN echo "store-plaintext-passwords=off" >> ~/.subversion/config

# Check out saliency
WORKDIR /code
ARG SALIENCY_SVN_PASSWORD
RUN svn checkout --username anonsvn --password ${SALIENCY_SVN_PASSWORD} svn://isvn.usc.edu/software/invt/trunk/saliency
ENV SALIENCYROOT=/code/saliency
WORKDIR /code/saliency
RUN autoconf configure.ac > configure
  
# Build Saliency
ENV CFLAGS='-L/usr/lib -I/usr/include'
ENV CXXFLAGS='-L/usr/lib -I/usr/include'
ENV CPPFLAGS='-L/usr/lib -I/usr/include'
ENV LDFLAGS='-L/usr/lib'
RUN ./configure 
RUN make clean
RUN make depoptions-all
RUN make bin/ezvision 
RUN make bin/test-JunctionHOG 
RUN make bin/test-KalmanFilter

# Check out and build xercesc
ENV X_VERSION 2_8_0
WORKDIR /code
RUN wget http://archive.apache.org/dist/xerces/c/2/sources/xerces-c-src_${X_VERSION}.tar.gz
RUN tar -zxf xerces-c-src_${X_VERSION}.tar.gz
ENV XERCESCROOT=/code/xerces-c-src_${X_VERSION}
WORKDIR /code/xerces-c-src_${X_VERSION}/src/xercesc
RUN ./runConfigure -p linux -b 64
RUN make
RUN make install

# For X11 Forwarding to display output
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get install -y xpra rox-filer openssh-server pwgen xserver-xephyr xdm fluxbox xvfb sudo
RUN sed -i 's/DisplayManager.requestPort/!DisplayManager.requestPort/g' /etc/X11/xdm/xdm-config
RUN sed -i '/#any host/c\*' /etc/X11/xdm/Xaccess
RUN echo X11Forwarding yes >> /etc/ssh/ssh_config
RUN sed -i 's/session    required     pam_loginuid.so/#session    required     pam_loginuid.so/g' /etc/pam.d/sshd
RUN dpkg-divert --local --rename --add /sbin/initctl && ln -sf /bin/true /sbin/initctl
RUN apt-get -y install fuse  || :
RUN rm -rf /var/lib/dpkg/info/fuse.postinst
RUN rm /code/saliency/bin/*
RUN rm -rf /var/lib/apt/lists/*
RUN localedef -v -c -i en_US -f UTF-8 en_US.UTF-8 || :
EXPOSE 22

# Miscellaneous dependencies for running scripts
ENV PERL_MM_USE_DEFAULT=1
RUN perl -MCPAN -e'install "XML::Simple";install "XML::Writer";install "Switch"'

# Add helper scripts
WORKDIR /usr/local/bin
ADD scripts .
RUN mv MBARItest.mp4 /tmp

# Add mbarivision code and build
WORKDIR /code/avedac-mbarivision/schema
ADD schema . 
WORKDIR /code/avedac-mbarivision
ADD src .
RUN autoconf configure.ac > configure
RUN chmod 0755 configure 
ENV CPPFLAGS '-I/usr/include/libxml2'
ENV LDFLAGS '-lxml2'
RUN ./configure --with-saliency=/code/saliency --with-xercesc=/usr/local/include/xercesc --prefix=/usr/local
RUN make
RUN make install
  
# Clean up and add startup script
WORKDIR /code
ADD start.sh .
RUN rm *.zip
RUN rm *.tar.gz
RUN rm -rf /code/opencv-${CV_VERSION}
RUN rm -rf /code/xerces-c-src_${X_VERSION}
   
# Start xdm and ssh services.
CMD ["/bin/bash", "/code/start.sh"]
