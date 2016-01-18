#ifndef _TRACKER_GUI_H_
#define _TRACKER_GUI_H_

#include <QMainWindow>
#include <QPushButton>
#include <QListWidget>
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
    QListWidget* getObjList() const;
    QPushButton* getRefreshButt() const;

signals:
    void trackObject(std::string* obj);

private slots:
    void on_RunningButt_clicked();
    void on_objects_itemSelectionChanged();
    void on_TrackButt_clicked();
    void on_StopButt_clicked();
    void on_MarkButt_clicked(bool checked);
    void on_PubButt_clicked(bool checked);
    void on_TfButt_clicked(bool checked);

private:
    void init();
    Ui::TrackerGui *ui;
    pacv::TrackerConfig::Ptr config;
    bool running;
};
#endif
