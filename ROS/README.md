
## Installation 

To install all the dependencies run
```
mkdir ~/mpcc_ws
mkdir ~/mpcc_ws/src
cd ~/mpcc_ws/src
git clone -b mpcc_ros_node https://github.com/mschratter/MPCC.git

cd ~/mpcc_ws/src/MPCC/C++
./install.sh

cmake CMakeLists.txt
make

cd ~/mpcc_ws
catkin_make
```


## Run the MPCC 

```

source ~/mpcc_ws/devel/setup.sh


roslaunch mpcc rviz.launch 

roslaunch mpcc simulator.launch

roslaunch mpcc mpcc.launch 


```



