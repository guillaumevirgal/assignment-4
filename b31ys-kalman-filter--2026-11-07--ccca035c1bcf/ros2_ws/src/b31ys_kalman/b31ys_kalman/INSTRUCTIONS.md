Description:
We continue with the hospital robot that brings medicine to patients. The robot has just been delivered to a new ward and there is no map available. Before it can start delivering medicine, the robot must build a map of the ward. It must then use that map to navigate safely to the patients’ rooms.

Groups: groups of 3 students. Only one submission per group.

You can do this assignment in the real rosbot! You can check it out here: https://husarion.com/tutorials/howtostart/rosbot2r-quick-start/

Part 1: Mapping
Use an open-source ROS 2 SLAM package to build a map of the environment. We recommend slam_toolbox. You do not need to code the SLAM algorithm yourselves.
Drive the robot with teleop to build the map, then save it using the Nav2 map saver.
Part 2: PRM on the SLAM map
Adapt your PRM from Assignment 2 so it plans on the map you built. You can use the PRM of any group member, combine them, or use any other planning method. Your planner must:

Part 3: Localisation and path following
Localise the robot in your saved map, using AMCL or slam_toolbox in localisation mode.
Make the robot follow the planned path. You can write your own path follower (for example a proportional or pure pursuit controller publishing on /cmd_vel), or send your path to the Nav2 controller server through the FollowPath action.
The robot must reach the goal without collisions.
Part 4: Integration
Provide a single launch file that starts the full system (localisation, planner, path follower and RViz).
Demonstrate the robot reaching at least three different goals (three patient rooms), one after the other.

What to submit
One submission per group:

A group report (maximum 4 pages, PDF) including:
a diagram of your system architecture
the SLAM package you used and the parameters you changed
what you changed in your PRM to work on the SLAM map
results: success rate, path length, planning time and number of nodes
what failed and how you fixed it
a table with each member’s contribution
A video of the robot working! or a real live demo
Individually, each student must also complete the peer assessment form and submit a short statement (maximum 200 words) describing their own contribution.

You will be evaluated in terms of:
SLAM (15%): does the map represent the environment well?
PRM on the SLAM map (25%): does it find a solution? Is it PRM? Does it keep a safe distance from obstacles?
Localisation and path following (25%): does the robot reach the goals without collisions?
Integration and code quality (15%): does it run from the launch file? Is the code well structured and documented?
Report (20%): is the analysis clear, and does it explain the results and failures?
The group mark may be adjusted for individual students based on the peer assessment and contribution statements.