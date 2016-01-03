#include <pacman_vision.h>

/*************** MAIN *********************/
int main (int argc, char *argv[])
{
    QApplication q_app (argc, argv);
    PacmanVision pacv_app;

    if (pacv_app.init(argc, argv))
        return q_app.exec();
    else
        return 1;
}
