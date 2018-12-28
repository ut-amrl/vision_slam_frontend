# vision-slam-frontend
Simple frontend for SLAM for a ground robot using vision and odometry

## Dependencies
Install [ROS](http://wiki.ros.org/ROS/Installation)

## Build Instructions
1. Clone this repo into `vision_slam_frontend`, and enter the directory.
   ```bash
   git clone git@github.com:umass-amrl/vision-slam-frontend.git vision_slam_frontend
   ```
   IMPORTANT: ROS does not like hyphens in package paths. Hence the change of name for the directory.
   All subsequent commands must be run from within the directory.
1. Add the project to the ROS packages path:
   ```bash
   export ROS_PACKAGE_PATH=/PATH/TO/YOUR/REPO:$ROS_PACKAGE_PATH
   ```
   Pro-tip: add this line to your `.bashrc` file.
1. Run `make`.
   DO NOT RUN `cmake` directly.
   DO NOT RUN `catkin_build` or anything catkin-related.
