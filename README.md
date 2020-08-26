# crtk-matlab-client

**This is work in progress, not ready to be used!!!**

## Introduction

The goal of the CRTK Matlab Client utilities is to facilitate the use of ROS CRTK compatible devices in Matlab.  The `crtk_utils` class hides all the ROS publishers and subscribers as well as data conversion from ROS messages to Matlab data types (and vice versa). It also provides methods to wait for state transitions such as waiting for a robot to home or wait for a motion to complete.

The following has been tested on Linux with Matlab 2020a.

## Custom messages and paths

Matlab's handling of custom messages is working but a bit frustrating to use.  Please read carefuly the content of this section, this might save you some time.

### Generate the code to handle CRTK custom ROS messages

Off the box, Matlab only supports ROS standard messages and can't handle custom messages.  There is an add-on available from Matlab to generate code (mix of Java and Matlab) to handle custom messages. The process is not fully automated and requires a one-time generation and configuration of your Matlab environment.  You can use the Matlab command `rosAddons` to install `rosgenmsg` (see https://www.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html).  For this step, make sure you run Matlab as "super user": `sudo matlab`.  This is the only time you should run Matlab with `sudo`.

To note, `rosgenmsg` has two rather annoying limitations:
* `rosgenmsg` expects a directory that contains directories of ROS packages.   So if you clone `crtk-msgs` in `catkin_ws/src` you will need to use `rosgenmsg` on all packages in `catkin_ws/src`.  To avoid this, you need to create a sub-directory in `catkin_ws/src` and then clone `crtk-msgs` in said sub-directory (for example `~/catkin_ws/src/crtk/crtk_msgs`).
* `rosgenmsg` expects that the folder containing the package is named after the package.  The current directory/repository name `crtk-msgs` prevents the message generation since it doesn't match the package name `crtk_msgs` (note the underscore vs dash).
If you're cloning `crtk-msgs` by hand, you can use:
```sh
cd ~/catkin_ws/src
# clone in sub-directory crtk and rename to crtk_msgs
git clone git clone https://github.com/collaborative-robotics/crtk-msgs crtk/crtk_msgs
```

At that point, you can finally generate the code (running Matlab without `sudo`):
```matlab
rosgenmsg(fullfile(userpath,'catkin_ws/src/crtk'))
```

Then follow instructions to edit `javaclasspath.txt` in `~/.matlab/R2020a`.  The tutorial recommends to use `savepath` but this would apply to all users on the workstation so you should probably skip that step and read the following section.

### Paths

Instead of setting the path for all users, you can use your startup script.  To edit the user's `startup.m`, use:
```matlab
edit(fullfile(userpath,'startup.m'))
```

In your `startup.m`, you can add the `addpath` commands that you want executed everytime your start Matlab:
```matlab
% to locate crtk_msgs
addpath(fullfile(userpath,'catkin_ws/src/crtk/matlab_gen/msggen'))
% to locate crtk client
addpath(fullfile(userpath,'catkin_ws/src/crtk-matlab-client'))
% to locate dvrk code
addpath(fullfile(userpath,'catkin_ws/src/dvrk-ros/dvrk_matlab'))
```

Then quit Matlab, restart it and test using:
```matlab
which startup
% show message properties
rosmsg('show', 'crtk_msgs/operating_state')
% create a message
m = rosmessage('crtk_msgs/operating_state')
```

## Usage

The first step is to create a Matlab class with dynamic properties.  For example, let's assume we want to create a simple force sensor client:
```matlab
classdef force_sensor < dynamicprops
```
The class should own an instance of `crtk_utils`:
```matlab
    properties (Access = protected)
        crtk_utils;
    end
```
Then in the constructor, create an instance of `crtk_utils` and add the CRTK features you need.  For example, if the device supports `measured_cf`, use the method `add_measured_cf()`.
```matlab
   methods
        function self = force_sensor(ros_namespace)
            self.crtk_utils = crtk_utils(self, ros_namespace);
            self.crtk_utils.add_measured_cf();
        end
    end
```
The method `add_measured_cf` will create the necessary ROS subscriber and add a function handle (`measured_cf`) to the force sensor class.  Once this is done, you can create an instance of the force sensor and call the method `measured_cf`:
```matlab
>> fs = force_sensor('optoforce/');
>> cf = fs.measured_cf()
cf =
   -0.0025   -0.0125    0.0775         0         0         0
```
If there are no messages on the CRTK topic subscribed to, you will get a warning similar to:
```matlab
>> cf = fs.measured_cf()
Warning: measured_cf has not received messages yet (topic /optoforce/measured_cf)
```
This can be used to make sure you're using the right ROS topic name and namespace.
