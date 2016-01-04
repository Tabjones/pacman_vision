//this dummy file is to test the gui in QtCreator without letting him know
//it is part of a bigger project !!
#include <basic_node_gui.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    pacv::BasicConfig::Ptr b=std::make_shared<pacv::BasicConfig>();//fake conf
    pacv::SensorConfig::Ptr s=std::make_shared<pacv::SensorConfig>();//fake conf
    BasicNodeGui w(b,s);
    w.show();
    return a.exec();
}
