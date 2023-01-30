# Husky Bringup
This bring-up package contains all the launch files and configurations to setup the Husky robot. 

To install the ROS service such that these lauch files are ran at start-up:
1. Connect all sensors
2. Setup all respective environment variables
3. Run: `rosrun husky_bringup install`

# Environment Variables
The Husky robot has pre-defined sensor accessories that can be easily added/removed using environment variables. These variables will add the sensor to the URDF and start-up the respective launch files. 

See the [Husky Description](https://github.com/husky/husky) package for all environment variables. 
