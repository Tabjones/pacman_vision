# PaCMaN Vision ROS node #
## Dependencies ##
* [Calibration](https://github.com/CentroEPiaggio/calibration)
* [Asus Scanner Models](https://github.com/pacman-project/pacman-object-database)
* Openni2 Launch and Camera.

## Usage ##
Launch the node with
`roslaunch pacman_vision pacman_vision.launch`
then configure it with `rqt-reconfigure` (already launched by default).

To check available parameters:
`roslaunch --ros-args pacman_vision pacman_vision.launch`