# smach_rooms_exploration_gazebo

This application illustrates how a drone explores a room. While the mission is executing, the drone starts mapping the room while generating paths in order to explore it.

In order to execute the mission, perform the following steps:

- Install Lidar components:

        $ ./lidar_installation.sh

- Execute the script that launches Gazebo for this project:

        $ ./launcher_gazebo.sh


- Wait until the following window is presented:



- Execute the script that launches the Aerostack components for this project:

        $ ./main_launcher.sh

-Move to /configs/mission directory

        $ cd configs/mission
        
- Execute the following command to run the mission:

        $ ./simple_mission.py

The following video illustrates how to launch the project:

The following video shows the complete execution:




