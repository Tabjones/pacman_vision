# PaCMaN Vision ROS node #
![PaCMan Vision](https://cloud.githubusercontent.com/assets/1950251/12723299/22f2d844-c909-11e5-9621-142a1d49dcd4.png)
Collection of 3D Vision-Oriented utilities packed in a single ROS node.

PaCMan Vision (or just PACV) is a modular ROS node for robot Vision handling point cloud streams from various RGB-D sensors, like the Asus Xtion PRO or the Microsft Kinect One and republishing
the modified stream to the ROS network, making it available for other nodes.
The node is composed by a basic node (or __bare-bone__ node), providing some simple and fast point cloud filters, like a cropping filter, a voxel grid downsampling and a RANSAC plane
segmentation. All the filters can be modified at runtime using the built-in Qt Gui.

The node has other functionalities like the possibility to dynamically change point cloud stream subscription or to save a single processed point cloud from the stream that's being republished.

PaCMan Vision also introduces the concept of dynamic modules, a dynamic module is a part of the node, providing some additional functionality, you can dynamically load or kill at runtime.
Each module has its own ROS node handle with its topics and services you can exploit and when you don't need it anymore you can kill the module and its functionality will disappear.

As of now, PaCMan Vision has a few modules providing object recognition and pose estimation and 3D object tracking, but it is planned to release more modules in the near future.
Each module has its own section on the wiki, with install and usage instructions.

## Get PaCMaN Vision ##
Clone these two repositories into your ROS catkin workspace (you need and [SSH key setup on GitHub](https://help.github.com/articles/generating-an-ssh-key/)):
```
roscd && cd ../src
git clone git@github.com:Tabjones/pacman_vision.git pacv
git clone git@github.com:Tabjones/pacman_vision_communications.git pacv_com
```
Pacman_vision repository contains all the software needed to build the node, while vision_communication repository is just a bunch of defined messages and services PaCMan Vision uses
You will need both to build and run the node.
The reason why those two packages are separated is to provide the user the flexibility of having PaCMan Vision messages and services without actually building the
whole PaCMan Vision package, thus avoiding its dependencies, while still being able to call its services or subscribe to its messages from other packages.

## Build Instructions ##
Detailed install instructions are found on the [package wiki](https://github.com/Tabjones/pacman_vision/wiki)


### Step 2 - Choosing sensor (Optional)###
PaCMan-Vision provides a modular build system which enables/disables certain portion of code based on
other packages. For instance you have to choose if you want support for the Kinect v2 (Kinect One) or just
rely on the old Kinect v1 (Primesene, Asus Xtion, etc...).
Kinect v1 is supported by default, you just have to install `openni2_launch`, if you haven't already
(`openni2_launch` can be found on the standard ROS repository).

On the other hand if you also want support for Kinect v2 you'll need:
* [libfreenect2](https://github.com/OpenKinect/libfreenect2)
* And optionally [Kinect2 Bridge](https://github.com/code-iai/iai_kinect2)

Detailed install instructions are found on the packages pages. Here's a summary install for Ubuntu 14.04
with an Nvidia GPU:
  * Nvidia OpenCL
```
sudo add-apt-repository ppa:xorg-edgers/ppa
sudo apt-get update
sudo apt-get install nvidia-352-dev opencl-headers
```
  * Libfreenect2
```
sudo apt-get install build-essential libturbojpeg libjpeg-turbo8-dev libtool autoconf libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev automake
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2/depends
sh install_ubuntu.sh
sudo dpkg -i libglfw3*_3.0.4-1_*.deb
cd .. && mkdir build && cd build
cmake ..
make && sudo make install
sudo cp ../rules/90-kinect2.rules /etc/udev/rules.d/
```
  * Kinect2_Bridge
```
cd ~/catkin_ws/src/
git clone https://github.com/code-iai/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
```
However note that `Kinect2_Bridge` is purely optional, because PaCMan-Vision has a minimal built in Kinect v2 processor, built on top
of `libfreenect2`, so you can still use that sensor in ROS without `Kinect2_Bridge`, your choice entirely.
In fact, at runtime you can choose between `openni2_launch`, `kinect2_brdge`, or the internal kinectv2 processor, so it doesn't hurt to
install optional dependencies.

### Step 3 - Pose Estimation Library (Optional)###
PaCMan-Vision uses Pose Estimation Library (pel) to perform online pose estimation of objects and
tracking, these features are entirely optional, however if you want them you'll need to install [pel](https://github.com/Tabjones/Pose-Estimation-Library).

To do that you can either clone the library into your catkin source space directly and let it compile it
along with other pure catkin packages, or install it system wide via CMake.

* For a catkin install:
```
cd ~/catkin_ws/src/
git clone https://github.com/Tabjones/Pose-Estimation-Library
```
And let it compile when you execute `catkin build`.

* For a system-wide install:
```
git clone https://github.com/Tabjones/Pose-Estimation-Library pel
cd pel && mkdir build && cd build
cmake ..
make && sudo make install
```
Additional informations can be found on the package page.

### Step 4 - Other ROS Packages Needed at Exec-time###
Pacman-Vision expects to find the following packages at execution time:

* [Calibration](https://github.com/CentroEPiaggio/calibration)
* [Asus Scanner Models](https://github.com/pacman-project/pacman-object-database)

If you want to use the __Listener Module__ you will need also

* [Vito-Robot](https://github.com/CentroEPiaggio/vito-robot)

  To listen to its broadcasted transforms and perform precise filters to remove the robot hands or arms from
the scene.

You can build PaCMan-Vision without those packages, but you will most likely get runtime errors when
you launch it, so just clone them into `catkin` source space.

### Step 5 - Catkin Build ###
Everything is ready, you can start building PaCMan-Vision and other packages in your catkin workspace.
```
cd ~/catkin_ws
catkin build
```
Everything should go fine and you are finally able to use PaCMan-Vision.

## Usage ##
Launch the node with
`roslaunch pacman_vision pacman_vision.launch`
then configure it with `rqt-reconfigure` (already launched by default).

To check available parameters:
`roslaunch --ros-args pacman_vision pacman_vision.launch`

### Mirrors ###
This project is mirrored on:

  * [Github](https://github.com/Tabjones/pacman_vision).
  * [Bitbucket](https://bitbucket.org/Tabjones/pacman_vision).
  * [Gitlab](https://gitlab.com/fspinelli/pacman_vision).

