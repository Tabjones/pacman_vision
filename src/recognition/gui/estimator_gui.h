#ifndef _ESTIMATOR_GUI_H_
#define _ESTIMATOR_GUI_H_

#include <QMainWindow>
#include <QPushButton>
#include <recognition/estimator_config.hpp>

namespace Ui {
class EstimatorGui;
}

class EstimatorGui : public QMainWindow
{
    Q_OBJECT

public:
    explicit EstimatorGui(const pacv::EstimatorConfig::Ptr conf, QWidget *parent = 0);
    ~EstimatorGui();
    QWidget* getWidget() const;
    void setRunning(const bool run);
    QPushButton* getRunButt() const;
    QPushButton* getEstButt() const;

private slots:
    void on_RunningButt_clicked();
    void on_CalibButt_clicked(bool checked);
    void on_Cluster_valueChanged(double arg1);
    void on_Iter_valueChanged(int arg1);
    void on_Neigh_valueChanged(int arg1);
    void on_EstimationButt_clicked();

private:
    void init();
    Ui::EstimatorGui *ui;
    pacv::EstimatorConfig::Ptr config;
    bool running;
};

#endif // BASIC_NODE_GUI_H
