FROM base
MAINTAINER Danelle Cline <dcline@mbari.org>
# Install needed gdb components
RUN \
  apt-get update -y && \
  apt-get install -y gdbserver gdb 
CMD ["/bin/bash"]
