#ifndef _TRACKER_GUI_H_
#define _TRACKER_GUI_H_

#include <QMainWindow>
#include <QPushButton>
#include <recognition/tracker_config.hpp>

namespace Ui {
class TrackerGui;
}

class TrackerGui : public QMainWindow
{
    Q_OBJECT

public:
    explicit TrackerGui(const pacv::TrackerConfig::Ptr conf, QWidget *parent = 0);
    ~TrackerGui();
    QWidget* getWidget() const;
    void setRunning(const bool run);
    QPushButton* getRunButt() const;
    QPushButton* getTrackButt() const;
    QPushButton* getStopButt() const;

private slots:
    void on_RunningButt_clicked();

    void on_objects_itemSelectionChanged();

private:
    void init();
    Ui::TrackerGui *ui;
    pacv::TrackerConfig::Ptr config;
    bool running;
};
#endif
