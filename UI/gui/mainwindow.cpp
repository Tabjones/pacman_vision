#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_MasterDisable_clicked(bool checked)
{
    if (checked == false){
        ui->MasterDisable->setText("    Master Disable");
        ui->MasterReset->setDisabled(false);
        ui->BaseTab->setDisabled(false);
        ui->SensorTab->setDisabled(false);
        ui->LoggingConsole->appendPlainText("* Functionality is globally resumed.");
        // re-enable everything
    }
    else if (checked == true){
        ui->MasterDisable->setText("    Master Enable");
        ui->LoggingConsole->appendPlainText("* Functionality is globally disabled.");
        ui->MasterReset->setDisabled(true);
        ui->BaseTab->setDisabled(true);
        ui->SensorTab->setDisabled(true);
        // disable everything
    }
}

void MainWindow::on_MasterReset_pressed()
{
   ui->LoggingConsole->appendPlainText("* Issued a MASTER RESET!");
   //Reset Everything
}

void MainWindow::on_CroppingButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Cropping Filter");
        ui->CroppingG->setDisabled(false);
        //updateBaseConfig
        //cropping=true
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Cropping Filter");
        ui->CroppingG->setDisabled(true);
        //updateBaseConfig
        //cropping=false
    }
}

void MainWindow::on_Xmin_valueChanged(double arg1)
{
   //limx1 = arg1
}

void MainWindow::on_Xmax_valueChanged(double arg1)
{
   //limx2 = arg1
}

void MainWindow::on_Ymin_valueChanged(double arg1)
{
   //limy1 = arg1
}

void MainWindow::on_Ymax_valueChanged(double arg1)
{
   //limy2 = arg1
}

void MainWindow::on_Zmin_valueChanged(double arg1)
{
   //limz1 = arg1
}

void MainWindow::on_Zmax_valueChanged(double arg1)
{
   //limz2 = arg1
}

void MainWindow::on_PublishLimitsButt_clicked(bool checked)
{
   if(checked){
       //publishlimits = true
   }
   if(!checked){
       //publishlimits = false
   }
}

void MainWindow::on_DownsamplingButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Downsampling");
        ui->LeafF->setDisabled(false);
        //updateBaseConfig
        //downsampling=true
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Downsampling");
        ui->LeafF->setDisabled(true);
        //updateBaseConfig
        //downsampling=false
    }
}

void MainWindow::on_Leaf_valueChanged(double arg1)
{
   //leaf_size = arg1
}

void MainWindow::on_SegmentingButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Plane Segmentation");
        ui->PlaneF->setDisabled(false);
        //updateBaseConfig
        //segmenting=true
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Plane Segmentation");
        ui->LeafF->setDisabled(true);
        //updateBaseConfig
        //segmenting=false
    }
}

void MainWindow::on_Plane_valueChanged(double arg1)
{
  //tolerance=arg1
}

void MainWindow::on_OrganizedButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Keep PointCloud organized, if possible. Note that downsampling and plane segmentation, break organized cloud structure.");
        //updateBaseConfig
        //organized=true
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Keep PointCloud organized.");
        //updateBaseConfig
        //organized=false
    }

}

void MainWindow::on_Internal_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to internal kinect2 processor as main source of point clouds. Using the reference frame specified.");
        //internal=true
        std::string name = ui->Name->text().toStdString();
        ui->NameG->setDisabled(false);
        ui->RefreshN->setDisabled(false);
        //name = name
    }
    if(!checked){
        ui->NameG->setDisabled(true);
        ui->RefreshN->setDisabled(true);
    }
}

void MainWindow::on_Asus_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Asus Xtion subscriber as main source of point clouds. (/camera/depth_registered/points)");
        ui->Topic->setText("/camera/depth_registered/points");
        //internal=false
        //topic =
    }
}

void MainWindow::on_Kinect2SD_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Kinect2 Bridge subscriber as main source of point clouds. Using SD topic (512x424) (/kinect2/SD/points)");
        ui->Topic->setText("/kinect2/SD/points");
        //internal=false
        //topic =
    }
}

void MainWindow::on_Kinect2QHD_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Kinect2 Bridge subscriber as main source of point clouds. Using QHD topic (960x540) (/kinect2/QHD/points)");
        ui->Topic->setText("/kinect2/QHD/points");
        //internal=false
        //topic =
    }
}

void MainWindow::on_Kinect2HD_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Kinect2 Bridge subscriber as main source of point clouds. Using HD topic (1920x1080) (/kinect2/HD/points)");
        ui->Topic->setText("/kinect2/HD/points");
        //internal=false
        //topic =
    }
}

void MainWindow::on_External_toggled(bool checked)
{
    if(checked){
        QString msg = ui->Topic->text();
        ui->LoggingConsole->appendPlainText(msg.prepend("* Switching to a custom external subscriber, specified by topic name: "));
        std::string topic = msg.toStdString();
        ui->TopicG->setDisabled(false);
        ui->RefreshT->setDisabled(false);
        //internal=false
        //topic =
    }
    if(!checked){
        ui->TopicG->setDisabled(true);
        ui->RefreshT->setDisabled(true);
    }
}

void MainWindow::on_RefreshN_clicked()
{
    std::string name = ui->Name->text().toStdString();
    //name =
}

void MainWindow::on_RefreshT_clicked()
{
    std::string topic = ui->Topic->text().toStdString();
    //topic =
}
