# ros_file_all


----------------- STANDART-------------------

roslaunch jackal_car_gazebo jackal_world.launch


roslaunch jackal_car_viz view_robot.launch



----------------- NAVIGATION-------------------

roslaunch jackal_car_navigation odom_navigation_demo.launch


roslaunch jackal_car_viz view_robot.launch config:=navigation


----------------- MAPPING -------------------

roslaunch jackal_car_navigation gmapping_demo.launch


roslaunch jackal_car_viz view_robot.launch config:=gmapping

rosrun rqt_robot_steering rqt_robot_steering

rosrun map_server map_saver -f mymap

not: projenin dogru calismasi icin bazı paketlerin kurulması gerekmekte bunlar:

indigo için:

sudo apt-get install ros-indigo-jackal-simulator ros-indigo-jackal-desktop

Kinetic için

sudo apt-get install ros-kinetic-jackal-simulator 
sudo apt-get install ros-kinetic-jackal-desktop


