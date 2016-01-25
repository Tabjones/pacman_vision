#ifndef _LISTENER_GUI_H_
#define _LISTENER_GUI_H_

#include <QMainWindow>
#include <QPushButton>
#include <listener/listener_config.hpp>

namespace Ui {
class ListenerGui;
}

class ListenerGui : public QMainWindow
{
    Q_OBJECT

public:
    explicit ListenerGui(const pacv::ListenerConfig::Ptr conf, QWidget *parent = 0);
    ~ListenerGui();
    QWidget* getWidget() const;
    void setRunning(const bool run);
    QPushButton* getRunButt() const;

private slots:
    void on_RunningButt_clicked();
    void on_SwitchHandButt_clicked();

    void on_LRAButt_clicked(bool checked);

private:
    void init();
    bool right;
    Ui::ListenerGui *ui;
    pacv::ListnerConfig::Ptr config;
    bool running;
};

#endif
