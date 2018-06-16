## Requirements

- Install [Docker](https://docs.docker.com/installation/)
 
## Installation

Get the password from http://ilab.usc.edu/toolkit/downloads.shtml for the toolkit, 
then run the Docker build with that password set in SALIENCY_SVN_PASSWORD

```bash
    git clone git@github.org:mbari-org/avedac-mbarivision.git
    docker build -t avedac-mbarivision --build-arg SALIENCY_SVN_PASSWORD=********** . 
```

## How to use the Docker image 

The software that runs the automated detection is simply called mbarivision. 
There is a script that simplifies its execution called runclip that can be run in the Docker container. 

If you don't have an example video clip, there is one in the Docker image at /examples/MBARItest.mp4.
This is a small video clip taken with a Remotely Operated Vehicle (ROV) transecting across the seafloor.
 
Ensure the host is allowing X forwarding 
> xhost + 127.0.0.1

in XQuarts click allow connections from network clients
 
* Run docker in the background in a "detached" mode (-d). Expose port 22 on the host machine (-P)  
> CID=$(docker run -d -P avedac-mbarivision)

* Get the port number
> echo $(docker port $CID 22 | cut -d ':' -f 2)
> 32768

* Using the password "saliency", port, and IP address, ssh directly to the docker container
> ssh -Y docker@localhost -p 32768 

* Set the display once in the container
> export DISPLAY=docker.for.mac.localhost:0

* Process clip /tmp/MBARItest.mp4 with benthic options (-f benthic) and display to XWindows (-x)
> runclip -i /tmp/MBARItest.mp4 -f benthic -x 
  
To get other options with runclip, simply type runclip
> runclip 

Alternatively, you can run mbarivision by hand. The options can all be found [here](doc/OPTIONS.md) 
