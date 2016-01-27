#ifndef _BASIC_NODE_GUI_H_
#define _BASIC_NODE_GUI_H_

#include <QMainWindow>
#include <QPushButton>
#include <basic_node/basic_node_config.hpp>
#include <basic_node/sensor_config.hpp>

namespace Ui {
class BasicNodeGui;
}

class BasicNodeGui : public QMainWindow
{
    Q_OBJECT

public:
    explicit BasicNodeGui(const pacv::BasicConfig::Ptr conf,
                          const pacv::SensorConfig::Ptr s_conf, QWidget *parent = 0) ;
    ~BasicNodeGui();
    void addTab(QWidget* tab, const QString title);
    QPushButton* getSaveButt() const;

signals:
    void boxChanged();
    void sensorChanged();
    void saveCloud(std::string* n);
    void enableDisable(bool enable);

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
    void on_SaveButt_clicked();
    void on_BrdButt_clicked(bool checked);

private:
    void init();
    Ui::BasicNodeGui *ui;
    pacv::Box lim;
    pacv::BasicConfig::Ptr config;
    pacv::SensorConfig::Ptr s_config;
    QString last_save_location;
};

#endif // BASIC_NODE_GUI_H
