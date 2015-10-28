# turtlesim_cleaner
This is the code from youtube ROS Tutorial 4 Series.

This is not my own work. This is the code written by following the tutorial from

https://www.youtube.com/playlist?list=PLSzYQGCXRW1HLWHdJ7ehZPA-nn7R9UKPa

Five C++ files. This tutorial demonstrates how to develop a simple cleaning application with turtlesim. It consists in making a coverage a full area like robot cleaners. 

1.robot_cleaner_move_rotate.cpp for the tutorial https://youtu.be/PGZMlzBlMmw?list=PLSzYQGCXRW1HLWHdJ7ehZPA-nn7R9UKPa

2.robot_cleaner_abs_orientation.cpp for the tutorial https://youtu.be/Ddqwq2WXFEk?list=PLSzYQGCXRW1HLWHdJ7ehZPA-nn7R9UKPa

3.robot_cleaner_move_to_goal.cpp for the tutorial https://youtu.be/Qh15Nol5htM?list=PLSzYQGCXRW1HLWHdJ7ehZPA-nn7R9UKPa
![alt tag](https://cloud.githubusercontent.com/assets/5270999/10780413/2861cd9a-7d63-11e5-99d0-351f45de5c56.png)


4.robot_cleaner_grid_clean.cpp
![alt tag](https://cloud.githubusercontent.com/assets/5270999/10780419/349f898a-7d63-11e5-88b9-770248e8cd24.png)
and
5.robot_cleaner_spiral_clean.cpp for the tutorial https://youtu.be/ehH8oLfsz-w?list=PLSzYQGCXRW1HLWHdJ7ehZPA-nn7R9UKPa
![alt tag](https://cloud.githubusercontent.com/assets/5270999/10780420/39414168-7d63-11e5-861c-c566d83e5a84.png)



Create a package by 
```
cd catkin_ws/src
catkin_create_pkg turtlesim_cleaner
cd catkin_ws
catkin_make
```

Now, go to 
```
cd ~/catkin_ws/src/turtlesim_cleaner
mkdir src
```

Go to this diectory in File Explorer and create a new file named   robot_cleaner.cpp
Copy the code of the tutorial you want from the repository and paste it in the robot_cleaner.cpp
Replace your CMakeList.txt and package.xml file with the files in the repo.


To run the Demo:

```
roscore                             //ROS Master
rosrun turtlesim turtlesim_node     //turtlesim
rosrun turtlseim_cleaner robot_cleaner_node
```
