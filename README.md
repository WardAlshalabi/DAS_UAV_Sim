# DAS_UAV_Sim
Drone-based Autonomous Surveillance System


#run these commands on the up directory of your workspace

sudo apt-get install ros-kinetic-navigation

sudo apt-get install ros-kinetic-hector-*

sudo apt-get install libffi-dev libffi6 libzbar-dev

sudo apt-get install python-pip

pip install libzbar-cffi

sudo apt-get install python-rosdep

sudo rosdep init

rosdep update

rosdep install --from-paths src --ignore-src -r -y

catkin_make --pkg ros_qr_tracker

catkin_make
