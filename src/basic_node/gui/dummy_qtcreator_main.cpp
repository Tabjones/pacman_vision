//this dummy file is to test the gui in QtCreator without letting him know
//it is part of a bigger project !!
#include <mainwindow.h>
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
