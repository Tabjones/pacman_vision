#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_

#include <QMainWindow>
#include <pacman_vision/module_config.h>

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    //intial gui configuration
    void configure(const BasicNodeConfig::Ptr b_conf);
    void configure(const SensorProcessorConfig::Ptr s_conf);
    void configure(const EstimatorConfig::Ptr e_config);
    //get configs
    BasicNodeConfig::Ptr getBaseConfig() const;
    SensorProcessorConfig::Ptr getSensorConfig() const;
    EstimatorConfig::Ptr getEstimatorConfig() const;
    //get master disable
    bool isDisabled() const;
    //master reset issued
    bool masterReset();

private slots:
    void on_MasterDisable_clicked(bool checked);
    void on_MasterReset_pressed();
    void on_CroppingButt_clicked(bool checked);
    void on_Xmax_valueChanged(double arg1);
    void on_Xmin_valueChanged(double arg1);
    void on_Ymin_valueChanged(double arg1);
    void on_Ymax_valueChanged(double arg1);
    void on_Zmin_valueChanged(double arg1);
    void on_Zmax_valueChanged(double arg1);
    void on_PublishLimitsButt_clicked(bool checked);
    void on_DownsamplingButt_clicked(bool checked);
    void on_Leaf_valueChanged(double arg1);
    void on_SegmentingButt_clicked(bool checked);
    void on_Plane_valueChanged(double arg1);
    void on_OrganizedButt_clicked(bool checked);
    void on_Internal_toggled(bool checked);
    void on_Asus_toggled(bool checked);
    void on_Kinect2SD_toggled(bool checked);
    void on_Kinect2QHD_toggled(bool checked);
    void on_Kinect2HD_toggled(bool checked);
    void on_External_toggled(bool checked);
    void on_RefreshN_clicked();
    void on_RefreshT_clicked();
    void on_SpawnEstim_clicked(bool checked);

private:
    Ui::MainWindow *ui;
    bool disabled;
    bool reset;
    BasicNodeConfig::Ptr basic_conf;
    SensorProcessorConfig::Ptr sensor_conf;
    EstimatorConfig::Ptr estimator_conf;
};

#endif // MAINWINDOW_H
