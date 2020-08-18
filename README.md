sudo apt-get install ros-melodic-nao-meshes

git clone https://github.com/ros-naoqi/nao_virtual.git
git clone https://github.com/ros-naoqi/nao_robot.git
git clone https://github.com/OTL/rqt_ez_publisher.git

rosrun rqt_ez_publisher rqt_ez_publisher --force-discover

/gazebo_contact_info/Head
/gazebo_contact_info/base_link
/gazebo_contact_info/l_wrist
/gazebo_contact_info/r_wrist

