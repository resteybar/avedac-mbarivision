## Running Mbarivision

The software that runs the automated detection is simply called *mbarivision*. 

*mbarivision* is a command-line tool that can either take a series of sequential frames, or a single frame.

A script that simplifies its execution is called *runclip* . This script breaks aparts the video into 
frames and creates a command line script called mbarivis_command to simplify execution.

If you don't have an example video clip, there is one in the Docker image at */tmp/MBARItest.mp4*.
This is a small video clip taken with a Remotely Operated Vehicle (ROV) transecting across the seafloor.
 
## Running 
Launch XQuartz and open a terminal window with <kbd>&#8984;N</kbd>
 
Ensure the host is allowing X forwarding 
[![ Image link ](img/xquarts_allow.jpg)]

> xhost + 127.0.0.1

in XQuarts click allow connections from network clients
 
* Run docker in the background in a "detached" mode (-d). Expose port 22 on the host machine (-P)  

```bash 
CID=$(docker run -d -P avedac-mbarivision)
```

* Get the port number

```bash
echo $(docker port $CID 22 | cut -d ':' -f 2)
32768
```

* Using the password "saliency", port, and IP address, ssh directly to the docker container
```bash 
ssh -Y docker@localhost -p 32768 
```

* Set the display once in the container
```bash
export DISPLAY=docker.for.mac.localhost:0
```

* Process clip /tmp/MBARItest.mp4 with benthic options (-f benthic) and display to XWindows (-x)
```bash
runclip -i /tmp/MBARItest.mp4 -f benthic -x 
```
  
To get other options with runclip, simply type runclip
```bash
runclip 
```

Alternatively, you can run mbarivision by hand. 
Options are [here](OPTIONS.md) 
