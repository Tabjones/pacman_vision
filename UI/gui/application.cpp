#include <application.h>

Application::Application(int & argc, char **argv):
    QApplication(argc,argv),
    timer(new QTimer), gui(new MainWindow),
    storage(new Storage)
{
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(500);
    init();
}

Application::~Application()
{
    delete timer;
    delete gui;
}

void
Application::show_gui()
{
    if (gui)
        gui->show();
}

void
Application::init()
{
    node.reset(new BasicNode("pacman_vision", storage, 50));
    sensor.reset(new SensorProcessor(node->getNodeHandle(),"sensor_processor",storage,50));
    estimator.reset(new Estimator(node->getNodeHandle(),"estimator",storage,2));

    node->spawn();
    sensor->spawn();
}

void
Application::update()
{
    std::cout<<"prova\n"<<std::flush;
}
