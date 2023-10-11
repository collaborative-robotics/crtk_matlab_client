# Generate custom ROS messages for Matlab version 2020b and up

:warning: For Ubuntu 20.04, the default version for Python is 3.8.  Matlab dropped support for Python 3.8 after 2023a so don't use 2023b or higher.

Off the box, Matlab only supports ROS standard messages and can't handle custom messages.  The good news in 2020b is that you don't need an add-on to generate custom ROS messages, `rosgenmsg` is now part of the ROS toolbox.

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

If you're running into an error re. **_cmake_ not being found**, see fix below and then come back to this section.

Then follow instructions to use `savepath` but this would apply to all users on the workstation so you should probably skip that step and read the following section.

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
clear classes
rehash toolboxcache
which startup
% show message properties
rosmsg('show', 'crtk_msgs/operating_state')
% create a message
m = rosmessage('crtk_msgs/operating_state')
```

## Issues

### CMake not found

The build process for the custom messages requires CMake and you might get an error message indicating that CMake is not found.  This might happen even if CMake is running properly in a shell:
```sh
> which cmake
/usr/bin/cmake
```

The issue is that Matlab sets a different path to load dynamic libraries (`.so` files) and CMake is found but doesn't load properly.  To test this in Matlab, try:
```matlab
!cmake
cmake: /usr/local/MATLAB/R2020b/bin/glnxa64/libcurl.so.4: no version information available (required by cmake)
cmake: /usr/local/MATLAB/R2020b/sys/os/glnxa64/libstdc++.so.6: version `GLIBCXX_3.4.26' not found (required by cmake)
cmake: /usr/local/MATLAB/R2020b/sys/os/glnxa64/libstdc++.so.6: version `GLIBCXX_3.4.26' not found (required by /usr/lib/x86_64-linux-gnu/libjsoncpp.so.1)
```

To allow Matlab to load CMake properly and generate your messages, first find your system `stdc++.so.6` library using `locate libstdc++.so.6`.  Then define `LD_PRELOAD` and start Matlab:
```sh
export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
matlab
```

Note that you will need to define `LD_PRELOAD` only to generate the messages.  This is not needed afterwards.

### Could not find Python version 2.7

Python default version on Ubuntu 20.04 should be Python 3 but the ROS message generation seems to require Python 2.7.

Two possible fixes:
* Within Matlab, do `pyenv('Version', 'python2.7')`
* You can change the default Python for the whole system using:
  ```sh
  sudo apt install python-is-python2
  ```
  Then start Matlab and generate the messages.  Once you're done generating the custom messages, you might want to quit Matlab and restore Python 3 as your default Python using:
  ```
  sudo apt install python-is-python3
  ```
