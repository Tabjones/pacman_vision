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
    if(gui->isDisabled()){
        if(!node->isDisabled())
            node->disable();
        if(!sensor->isDisabled())
            sensor->disable();
        if(!estimator->isDisabled())
            estimator->disable();
    }
    else{
        if(node->isDisabled())
            node->enable();
        if(sensor->isDisabled())
            sensor->enable();
        if(estimator->isDisabled())
            estimator->enable();
        node->updateIfNeeded(gui->getBaseConfig());
        sensor->updateIfNeeded(gui->getSensorConfig());
        estimator->updateIfNeeded(gui->getEstimatorConfig());
    }
}
