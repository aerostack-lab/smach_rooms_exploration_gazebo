# smach_rooms_exploration_gazebo

This application illustrates how a drone explores a room. While the mission is executing, the drone starts mapping the room while generating paths in order to explore it.

In order to execute the mission, perform the following steps:

- Install Lidar components:

        $ ./lidar_installation.sh

- Execute the script that launches Gazebo for this project:

        $ ./launcher_gazebo.sh


- Wait until the following window is presented:


<img src="https://github.com/aerostack/rooms_exploration_gazebo/blob/master/doc/gazeborooms.png" width=600>


- Execute the script that launches the Aerostack components for this project:

        $ ./main_launcher.sh

- Move to /configs/mission directory

        $ cd configs/mission
        
- Execute the following command to run the mission without using the Behavior Coordinator:

        $ ./move_mission.py
        
- If you want to use the Behavior Coordinator execute the following command instead:

        $ ./move_mission_with_behavior_coordinator.py 

The following video illustrates how to launch the project:

[ ![Launch](https://i.ibb.co/DVVTRb2/Captura-de-pantalla-de-2021-07-29-01-26-54.png)](https://www.youtube.com/watch?v=31e5CUVAzho)


The following video shows the complete execution of the mission without using the Behavior Coordinator:

[ ![Launch](https://i.ibb.co/61R1DtS/Captura-de-pantalla-de-2021-07-29-01-44-25.png)](https://www.youtube.com/watch?v=aPc0SeDHLSY)


The following video shows the complete execution of the mission using the Behavior Coordinator:

[ ![Launch](https://i.ibb.co/h736v4j/Captura-de-pantalla-de-2021-07-29-01-44-50.png)](https://www.youtube.com/watch?v=519rVCB-83M)


