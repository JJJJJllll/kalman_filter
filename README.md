# kalman_filter
ROS implementation of kalman filter using the available package and Eigen

## Install Eigen
```
cd ~/ && mkdir Software
cd Software
git clone https://gitlab.com/libeigen/eigen.git
cd eigen && mkdir build && cd build
cmake ..
make
sudo make install
```
### Make sure Eigen can be found in system path
```
cd /usr/local/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported
```

## Initialize submodule kalman-cpp
```
roslaunch kalman_filter kalman_filter_inCallback.launch
```
## I/Os
sub: /vrpn_client_node/Quad/pose                                        
pub: /kf_vel (velocity from kalman_filter)                              
     /naive_vel (velocity from position difference, serves as benchmark)

## Results demonstration
/kf_vel is smooth & lags behind /naive_vel within 10 ms
/kf_vel frequency reaches 199HZ
![17c21d934e06c76442f3819c198381b](https://github.com/JJJJJllll/kalman_filter/assets/117176940/2d5f551c-1ff5-4066-9cd1-256e11e73aa5)
