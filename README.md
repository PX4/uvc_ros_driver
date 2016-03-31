`uvc_ros_driver` is a ROS package that publishes video streams from USB video devices. It is made for devices that send multiple grayscale images as well as IMU data encoded in a color image stream.

# Prerequisites
You will need [ROS indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) and the [`ait_ros_messages`](https://github.com/ethz-ait/ait_ros_messages) ROS package.

You need `libusb` installed:

```bash
sudo apt-get install libusb-1.0-0-dev
```

You also need [`libuvc`](https://github.com/ktossell/libuvc), which can be installed with the following commands:
```bash
git clone https://github.com/ktossell/libuvc
cd libuvc
mkdir build
cd build
cmake ..
make && sudo make install
```

# Installation
Clone this repository in your catkin workspace.

## MAVLink Submodule

This package depends on MAVLink. To initialize and check out the submodule, run this:

```bash
git submodule init --recursive
git submodule update --recursive
```

## Building

Build using `catkin_make` according to the ROS instructions on http://ros.org.

## Set up Permissions

To get proper permissions for the USB device, run the following script with the device attached:
```bash
sudo usermod -a -G dialout $USER
sudo ./fix_permissions.sh
```
Check the device vendor and product id from the list and enter them when prompted. The printed list has the format
`... {idDevice:idProduct} ...`.

Log out of Ubuntu and Log back in. Unplug the device and plug it back in to load it with the new permissions.

# Running the driver
```bash
roslaunch uvc_ros_driver uvc_ros_driver.launch
```
To flip the images horizontally, appropriately change the ros parameter defined in the launch file.

The stereo images and IMU messages will be published on the topic `/vio_sensor` as a `VioSensorMsg` defined in the `ait_ros_messages` package.

If you need the images and IMU messages seperately, you can run
```bash
rosrun ait_ros_messages vio_sensor_republisher.py
```
This will publish the images and IMUs on the topics `/left_image`, `/right_image`, `/imu`, respectively.
