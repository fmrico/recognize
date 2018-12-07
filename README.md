# Recognize

[![Build Status](https://travis-ci.com/fmrico/recognize.svg?branch=master)](https://travis-ci.com/fmrico/recognize)

- Author: Francisco Martín Rico fmrico@gmail.com (Intelligent Robotics Lab URJC)
- License: BSD

## Requirements

- Ubuntu 16.04 LTS or Ubuntu 18.04 LTS
- ROS Kinetic or Melodic
- GPU recommneded, but not mandatory

## Compilation

- Go to your ROS workspace and clone the repository. This repository contains submodules, so the clone must include them:

```
$ roscd
$ cd ../src
$ git clone --recurse-submodules https://github.com/fmrico/recognize.git
```

- Compile for non GPU systems:

```
$ roscd
$ cd ..
$ catkin_make
```

- Compile for GPU systems:

```
$ roscd
$ cd ..
$ catkin_make -DCUDA=true
```

## Usage

Download darknet weights:

```
$ cd ThirdParty/darknet/
$ wget https://pjreddie.com/media/files/yolov3.weights
```

## Install

Installing binaries is not ready by now.

## Contributing

To make any PR you should:

- Compile without warnings for non-ThirdParty software
- Pass both roslint and gtests without errors

```
$ roscd
$ cd ..
$ catkin_make roslint
$ catkin_make run_tests
```

- Any new functionality must include tests as far a possible
