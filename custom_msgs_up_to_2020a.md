# Generate custom ROS messages for Matlab up to version 2020a 

Off the box, Matlab only supports ROS standard messages and can't handle custom messages.  There is an add-on available from Matlab to generate code (mix of Java and Matlab) to handle custom messages.  The process is not fully automated and requires a one-time generation and configuration of your Matlab environment.  

You can use the Matlab command `rosAddons` to install `rosgenmsg` (see https://www.mathworks.com/help/ros/ug/create-custom-messages-from-ros-package.html).
For this step, make sure you run Matlab as "super user": `sudo matlab`.  This is the only time you should run Matlab with `sudo`.  Successive users should all install the add-on without `sudo`.

To note, `rosgenmsg` has one rather annoying limitation, it expects a directory that contains directories of ROS packages.   So if you clone `crtk_msgs` in `catkin_ws/src` you will need to use `rosgenmsg` on all packages in `catkin_ws/src`.  To avoid this, you need to create a sub-directory in `catkin_ws/src` and then clone `crtk_msgs` in said sub-directory (for example `~/catkin_ws/src/crtk/crtk_msgs`).

If you're cloning `crtk_msgs` by hand, you can use:
```sh
cd ~/catkin_ws/src
# clone in sub-directory crtk and rename to crtk_msgs
git clone https://github.com/collaborative-robotics/crtk_msgs crtk/crtk_msgs
```

At that point, you can finally generate the code (running Matlab without `sudo`):
```matlab
rosgenmsg('~/catkin_ws/src/crtk')
```

Then follow instructions to edit `javaclasspath.txt` in `~/.matlab/R2020a` (step 1).  The tutorial recommends to use `savepath` (step 2) but this would apply to all users on the workstation so you should probably skip that step and read the following section.

# Paths

Instead of setting the path for all users, you can use your startup script.  To edit the user's `startup.m`, use:
```matlab
edit(fullfile(userpath,'startup.m'))
```
In your `startup.m`, you can add the `addpath` commands that you want executed everytime your start Matlab:
```matlab
% to locate crtk_msgs
addpath('~/catkin_ws/src/crtk/matlab_gen/msggen')
% to locate crtk client
addpath('~/catkin_ws/src/crtk/crtk_matlab_client')
% to locate dvrk code - only for dVRK users
addpath('~/catkin_ws/src/dvrk-ros/dvrk_matlab')
```

Then quit Matlab, restart it and test using:
```matlab
which startup
% show message properties
rosmsg('show', 'crtk_msgs/operating_state')
% create a message
m = rosmessage('crtk_msgs/operating_state')
```

# Very important debugging note

Matlab seems to create a cache of existing messages that is not being refreshed automatically.  This is apparently populated when you first call `rosmsg` or `rosmessage`.  So if you forgot to set your path and then called `rosmsg` (which will fail to find your new message types), adding the path with `addpath` is not enough to locate the new message types since the list of available messages will **not** be refreshed.  At that point, it seems the only solution is to quit Matlab, restart Matlab and remember to set your paths **before** any call to `rosmsg` or `rosmessage`.  The Matlab command `rehash` doesn't seem to fix the issue.  The script `startup.m` can ensure that `addpath` is called before any ROS command.
