#include "mainwindow.h"
#include <QApplication>
#include <QTimer>

class Main : public QApplication
{
public:
    Main(int & argc, char **argv):
        QApplication(argc,argv),
        timer(new QTimer), w(new MainWindow)
    {
        connect(timer, &QTimer::timeout, this, &Main::update);
        timer->start(5000);
    }
    QTimer *timer;
    MainWindow *w;

    void update()
    {
          w->resize(800,800);
    }
};

int main(int argc, char *argv[])
{
    Main a(argc,argv);
    a.w->show();
    //QApplication a(argc, argv);
    //MainWindow w;
    //w.show();

    return a.exec();
}
