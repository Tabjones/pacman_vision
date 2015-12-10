#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

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

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
