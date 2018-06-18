## Running Mbarivision

The software that runs the automated detection is simply called *mbarivision*. 

*mbarivision* is a command-line tool that can either take a series of sequential frames, or a single frame.

There are currenlty two Docker images: one with the X11 display drivers (avedac-mbarivision-xdisplay), 
and one without (avedac-mbarivision). 

A script that simplifies its execution is called *runclip* in the xdisplay image. This script breaks aparts the video into 
frames and creates a command line script called mbarivis_command to simplify execution.

## Running with X11
Launch XQuartz and open a terminal window (Mac shortcut for this is <kbd>&#8984;N</kbd>)

In the terminal window, ensure the host is allowing X forwarding 

```bash 
xhost + 127.0.0.1
```

* In XQuarts preferences, click allow connections from network clients
[![ Image link ](img/xquarts_allow.jpg)]
 
* Run docker in the background in a "detached" mode (-d). Expose port 22 on the host machine (-p)  

```bash 
CID=$(docker run -d -p 22 avedac-mbarivision)
```

* If you have video to process, share your drive using the volume mount (-v) command. 

*Mac*
```bash 
CID=$(docker run -d -v /Users/dcline/Downloads:/tmp/Downloads -p 22 avedac-mbarivision-xdisplay)
```

*Windows*
```bash 
CID=$(docker run -d -v c:\\Users\dcline\Downloads/:/tmp/Downloads -p 22 avedac-mbarivision-xdisplay)
```

* If you don't have an example video clip, there is one in the Docker image at */tmp/MBARItest.mp4*.
This is a small video clip taken with a Remotely Operated Vehicle (ROV) transecting across the seafloor. 
 
* You should now see the image running in the background. Note that it takes 50-60 seconds to completely boot.
```bash
docker ps
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

## Running without X11
 
TODO: Add further detail here 
 
