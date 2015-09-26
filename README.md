# PaCMaN Vision ROS node #
Collection of Vision-Oriented utilities packed in a single ROS node.

## Install Instructions step-by-step##
__NOTE: These install instructions are for Linux only, there's currently no support (and probably will never be) for other operating systems.__
### Step 0 ###
You need a `catkin` workspace possibly initialized and configured to work with [catkin tools](http://catkin-tools.readthedocs.org/en/latest/index.html).
`catkin tools` is to be preferred over `catkin_make` macro provided by ROS for a number of reasons.
Most important one is that it can handle `pure cmake` projects better, such as [pel](https://bitbucket.org/Tabjones/pose-estimation-library), which is optionally
needed by the package.

You will need also [CMake](http://cmake.org/)

Clone the repository in recursive mode (you'll need an [SSH key setup on Bitbucket](https://confluence.atlassian.com/bitbucket/set-up-ssh-for-git-728138079.html).)
```
git clone --recursive git@bitbucket.org:Tabjones/pacman_vision.git
```
Alternatively you can clone the repository in http mode without submodules, and then clone the submodule manually.
```
git clone https://Tabjones@bitbucket.org/Tabjones/pacman_vision.git
cd pacman_vision
git clone https://Tabjones@bitbucket.org/Tabjones/vision_communications.git communications
```
* [Calibration](https://github.com/CentroEPiaggio/calibration)
* [Asus Scanner Models](https://github.com/pacman-project/pacman-object-database)
* [Kinect2 Bridge](https://github.com/code-iai/iai_kinect2)

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

