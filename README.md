# ROSE ROS
This is a ROS wrapper for the ROSE code.

## Compilation and installation

Before starting working with this repository, please be sure that the following software is installed:
1) Python 3.7
2) catkin_virtualenv (http://wiki.ros.org/catkin_virtualenv)

After pulling the main repository, be sure that the submodule repository ROSE is also in place and modify the following lines in it:

fft_strucuture_extraction.py

l20 import helpers as he -> import rose.helpers as he
l21 from rose.wall_segment import WallSegment as ws -> from rose.wall_segment import WallSegment as ws

When in the ROSE directory, execute the following command
`catkin build --this --no-deps --catkin-make-args venv_lock`

After this, you are ready to build the workspace.

## Running

In the launch file, you will find a minimal working example of the ROSE.