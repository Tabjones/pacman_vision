#include <application.h>

/******************************************/
/*************** MAIN *********************/
/******************************************/
int main (int argc, char *argv[])
{
    ros::init(argc, argv, "pacman_vision");
    ros::Time::init();
    Application pacman_vision(argc,argv);
    pacman_vision.show_gui();
    return pacman_vision.exec();
}
