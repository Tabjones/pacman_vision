#include <application.h>

/******************************************/
/*************** MAIN *********************/
/******************************************/
int main (int argc, char *argv[])
{
    ros::init(argc, argv, "pacman_vision");
    ros::Time::init();
    Application pacman_vision(argc,argv);
    pacman_vision.showGui();
    pacman_vision.startUpdating(500);
    return pacman_vision.exec();
}
