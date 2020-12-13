# WhereAmI_Jupiter
Localization project for Udacity's Robotics Engineering Nanodegree

This project uses ACML (Adaptive Monte Carlo Localization) to find the position of my robot, Jupiter, in his "home." ACML uses mapping and random visuals known as particles
to predict the probabilty/likelihood that my robot is at a specific spot in his area.

This was built using ROS Kinetic and Ubuntu Linux. 

The project requires the pgm_map_creator package to set up the map for the robot's environment. I also chose to use the teleop_twist_keyboard package to control the robot's
movement. If you clone this project, you will need to clone those packages and add them to your project separately, in the src folder within a catkin workspace. 

Enjoy! :D
