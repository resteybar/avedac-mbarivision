FROM base
MAINTAINER Danelle Cline <dcline@mbari.org>
 
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
   
# Remove leftover code used in debugging container   
RUN rm -rf /code/saliency
RUN rm -rf /code/avedac-mbarivision

# Start xdm and ssh services.
CMD ["/bin/bash", "/code/start.sh"]
