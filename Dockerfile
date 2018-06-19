FROM base
MAINTAINER Danelle Cline <dcline@mbari.org>
# remove left over code needed in debug/X images
RUN apt-get uninstall -y subversion git cmake build-essential python-pip byobu curl htop man unzip vim wget xterm fuse flex
RUN rm -rf /code/saliency
RUN rm -rf /code/avedac-mbarivision
CMD ["/usr/local/bin/mbarivision"]
