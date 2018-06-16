## Requirements

- Install [Docker](https://docs.docker.com/installation/)
- (optional) [XQuartz](https://www.xquartz.org/) for display X windows
 
## Installation

Get the password from http://ilab.usc.edu/toolkit/downloads.shtml for the toolkit, 
then run the Docker build with that password set in SALIENCY_SVN_PASSWORD

```bash
    git clone git@github.org:mbari-org/avedac-mbarivision.git
    docker build -t avedac-mbarivision --build-arg SALIENCY_SVN_PASSWORD=********** . 
```

## Running

Details on running can be found  [here](doc/RUN.md) 

 

