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
    QPushButton* getRunButt() const;
    QPushButton* getEstButt() const;
    void enable(bool full=false);
    void disable(bool full=false);

private slots:
    void on_CalibButt_clicked(bool checked);
    void on_Cluster_valueChanged(double arg1);
    void on_Iter_valueChanged(int arg1);
    void on_Neigh_valueChanged(int arg1);
    void on_EstimationButt_clicked();
    void on_SuccessButt_clicked(bool checked);
    void on_rmse_valueChanged(double arg1);
    void on_MarksButt_clicked(bool checked);
    void on_TfButt_clicked(bool checked);

private:
    void init();
    Ui::EstimatorGui *ui;
    pacv::EstimatorConfig::Ptr config;
};

#endif
