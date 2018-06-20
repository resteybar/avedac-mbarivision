FROM base
MAINTAINER Danelle Cline <dcline@mbari.org>
# remove left over code needed in debug/X containers
RUN apt-get purge -y --auto-remove subversion git cmake build-essential python-pip byobu curl htop man unzip vim wget xterm fuse flex
RUN rm -rf /code
CMD ["/usr/local/bin/mbarivision"]
