#include <application.h>

Application::Application(int & argc, char **argv):
    QApplication(argc,argv),
    timer(new QTimer), gui(new MainWindow),
    storage(new Storage)
{
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    init();
}

Application::~Application()
{
    delete timer;
    delete gui;
}

void
Application::showGui()
{
    if (gui){
        gui->show();
        gui->configure(node->getConfig());
        gui->configure(sensor->getConfig());
        gui->configure(estimator->getConfig(), estimator->isRunning());
    }
}

void
Application::startUpdating(int msecs)
{
    timer->start(msecs);
    node->spawn();
    sensor->spawn();
}

void
Application::init()
{
    node.reset(new BasicNode("pacman_vision", storage, 50));
    sensor.reset(new SensorProcessor(node->getNodeHandle(),"sensor_processor",storage,50));
    estimator.reset(new Estimator(node->getNodeHandle(),"estimator",storage,2));
}

void
Application::update()
{
    if (gui->masterReset())
    {
        node->updateIfNeeded(BasicNode::ConfigPtr(), true);
        sensor->updateIfNeeded(SensorProcessor::ConfigPtr(), true);
        estimator->updateIfNeeded(Estimator::ConfigPtr(), true);
        gui->configure(node->getConfig());
        gui->configure(sensor->getConfig());
        gui->configure(estimator->getConfig(), estimator->isRunning());
        return;
    }
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
