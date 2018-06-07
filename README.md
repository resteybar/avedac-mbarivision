## Requirements

- Install [Docker](https://docs.docker.com/installation/)
 
## Installation

Get the password from http://ilab.usc.edu/toolkit/downloads.shtml for the toolkit, 
then run the Docker build with that password set in SALIENCY_SVN_PASSWORD

    git clone git@github.org:mbari-org/avedac-mbarivision.git
    docker build -t avedac-mbarivision --build-arg SALIENCY_SVN_PASSWORD=********** . ```

## How to use the Docker image 

The software that runs the automated detection is simply called mbarivision. 
There is a script that simplifies its execution called runclip that can be run in the Docker container. 

If you don't have an example video clip, there is one in the Docker image at /examples/MBARItest.mp4.
This is a small video clip taken with a Remotely Operated Vehicle (ROV) transecting across the seafloor.

* Run docker in the background in a "detached" mode
> CID=$(docker run -d -v /Users/dcline/Downloads/:/tmp/Downloads/ -P avedac-mbarivision)

* Get the port number
> echo $(docker port $CID 22 | cut -d ':' -f 2)
> 32768

* Using the password "saliency", port, and IP address, ssh directly to the docker container
> ssh -Y docker@localhost -p 32768 

* Process clip
> cd /tmp/Downloads
> runclip -i /examples/MBARItest.mp4 -f benthic -x 
 
This will store the mbarivision data in the directory  /Users/dcline/Downloads/MBARItest on the host. 

To get other options with runclip, simply type runclip
> runclip


Alternatively, you can run mbarivision by hand. The options can all be found [here](doc/OPTIONS.md) 
