//this dummy file is to test the gui in QtCreator without letting him know
//it is part of a bigger project !!
#include <listener_gui.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    pacv::ListenerConfig::Ptr b=std::make_shared<pacv::ListenerConfig>();//fake conf
    ListenerGui w(b);
    w.show();
    return a.exec();
}
