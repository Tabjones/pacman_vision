//this dummy file is to test the gui in QtCreator without letting him know
//it is part of a bigger project !!
#include <estimator_gui.h>
#include <tracker_gui.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    pacv::EstimatorConfig::Ptr b=std::make_shared<pacv::EstimatorConfig>();//fake conf
    // pacv::SensorConfig::Ptr s=std::make_shared<pacv::SensorConfig>();//fake conf
    EstimatorGui w(b);
    w.show();
    return a.exec();
}
