# turtle-twister

Group project. Moving a turtle bot using twist messages containing linear and angular velocity

**To run twist message in ros git clone the rosserial package **

$ sudo apt-get install ros-meledic-rosserial

$ mkdir -p ~/rosserial_ws/src

$ cd ~/rosserial_ws/src

$ git clone https://github.com/ros-drivers/rosserial.git

$cd ~/rosserial_ws

$ catkin_make

$ source ~/rosserial_ws/devel/setup.bash


**once you have sourced it  **

1.download and install Arduino IDE on your VMware 

2. setup ros_lib

$ roscore

$ rosrun rosserial_arduino make_library.py   

$ rosrun rosserial_python serial_node.py **/dev/ttyACM0** %make sure the correct USB prrt is selected 


**Note Build the code and upload it to the bot **

$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py %to move the bot 

if you dont have teleop_twist_keyboard.py from other projects , git clone it 

 else 
 
 use $ rostopic pub /cmd_vel geometry_msgs/Twist -r 10 -- '[0.3, 0.0, 0.0]' '[0.0, 0.0, -0.9]' 


 Change the value to the the linear and angular motions of the bot 
