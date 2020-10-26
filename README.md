# crtk-matlab-client

**This is work in progress, not ready to be used!!!**

## Introduction

The goal of the CRTK Matlab Client utilities is to facilitate the use of ROS CRTK compatible devices in Matlab.  The `crtk_utils` class hides all the ROS publishers and subscribers as well as data conversion from ROS messages to Matlab data types (and vice versa). It also provides methods to wait for state transitions such as waiting for a robot to home or wait for a motion to complete.

The following has been tested on Linux with Matlab 2020a.

## Custom messages and paths

Matlab's handling of custom messages is working but a bit frustrating to use.  Please read carefuly the content of this section, this might save you some time.  Choose the instructions to follow based on the Matlab version you're using: 
* Matlab up to 2020a: https://github.com/collaborative-robotics/crtk_matlab_client/blob/master/custom_msgs_up_to_2020a.md
* Matlab 2020b and up: https://github.com/collaborative-robotics/crtk_matlab_client/blob/master/custom_msgs_2020b_and_up.md

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
