#include <mainwindow.h>
#include <ui_mainwindow.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    disabled(false),
    basic_conf(new BasicNodeConfig),
    sensor_conf(new SensorProcessorConfig),
    estimator_conf(new EstimatorConfig)
{
    ui->setupUi(this);
}
MainWindow::~MainWindow()
{
    delete ui;
}
BasicNodeConfig::Ptr
MainWindow::getBaseConfig() const
{
    return basic_conf;
}
SensorProcessorConfig::Ptr
MainWindow::getSensorConfig() const
{
    return sensor_conf;
}
EstimatorConfig::Ptr
MainWindow::getEstimatorConfig() const
{
    return estimator_conf;
}
bool
MainWindow::isDisabled() const
{
    return disabled;
}
void MainWindow::configure(const BasicNodeConfig::Ptr b_conf)
{
    if (b_conf && basic_conf)
        *basic_conf = *b_conf; //TMP update gui also

}
void MainWindow::configure(const SensorProcessorConfig::Ptr s_conf)
{
    if (s_conf && sensor_conf)
        *sensor_conf = *s_conf;

}
void MainWindow::configure(const EstimatorConfig::Ptr e_config, const bool running)
{
    if (e_config && estimator_conf)
        *estimator_conf = *e_config;
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
        disabled = false;
    }
    else if (checked == true){
        ui->MasterDisable->setText("    Master Enable");
        ui->LoggingConsole->appendPlainText("* Functionality is globally disabled.");
        ui->MasterReset->setDisabled(true);
        ui->BaseTab->setDisabled(true);
        ui->SensorTab->setDisabled(true);
        // disable everything
        disabled = true;
    }
}

void MainWindow::on_MasterReset_pressed()
{
   ui->LoggingConsole->appendPlainText("* Issued a MASTER RESET!");
   //Reset Everything
   //TODO
}

void MainWindow::on_CroppingButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Cropping Filter");
        ui->CroppingG->setDisabled(false);
        //cropping=true
        basic_conf->cropping = true;
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Cropping Filter");
        ui->CroppingG->setDisabled(true);
        //cropping=false
        basic_conf->cropping = false;
    }
}

void MainWindow::on_Xmin_valueChanged(double arg1)
{
   //limx1 = arg1
   basic_conf->limits.x1 = arg1;
}

void MainWindow::on_Xmax_valueChanged(double arg1)
{
   //limx2 = arg1
   basic_conf->limits.x2 = arg1;
}

void MainWindow::on_Ymin_valueChanged(double arg1)
{
   //limy1 = arg1
   basic_conf->limits.y1 = arg1;
}

void MainWindow::on_Ymax_valueChanged(double arg1)
{
   //limy2 = arg1
   basic_conf->limits.y2 = arg1;
}

void MainWindow::on_Zmin_valueChanged(double arg1)
{
   //limz1 = arg1
   basic_conf->limits.z1 = arg1;
}

void MainWindow::on_Zmax_valueChanged(double arg1)
{
   //limz2 = arg1
   basic_conf->limits.z2 = arg1;
}

void MainWindow::on_PublishLimitsButt_clicked(bool checked)
{
   if(checked){
       //publishlimits = true
       basic_conf->publish_limits = true;
   }
   if(!checked){
       //publishlimits = false
       basic_conf->publish_limits = false;
   }
}

void MainWindow::on_DownsamplingButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Downsampling");
        ui->LeafF->setDisabled(false);
        //downsampling=true
        basic_conf->downsampling = true;
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Downsampling");
        ui->LeafF->setDisabled(true);
        //downsampling=false
        basic_conf->downsampling = false;
    }
}

void MainWindow::on_Leaf_valueChanged(double arg1)
{
   //leaf_size = arg1
   basic_conf->downsampling_leaf_size = arg1;
}

void MainWindow::on_SegmentingButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Plane Segmentation");
        ui->PlaneF->setDisabled(false);
        //segmenting=true
        basic_conf->segmenting = true;
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Plane Segmentation");
        ui->PlaneF->setDisabled(true);
        //segmenting=false
        basic_conf->segmenting = false;
    }
}

void MainWindow::on_Plane_valueChanged(double arg1)
{
    //tolerance=arg1
    basic_conf->plane_tolerance = arg1;
}

void MainWindow::on_OrganizedButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Keep PointCloud organized, if possible. Note that downsampling and plane segmentation, break organized cloud structure.");
        //organized=true
        basic_conf->keep_organized = true;
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Keep PointCloud organized.");
        //organized=false
        basic_conf->keep_organized = false;
    }
}

void MainWindow::on_Internal_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to internal kinect2 processor as main source of point clouds. Using the reference frame specified.");
        std::string name = ui->Name->text().toStdString();
        ui->NameG->setDisabled(false);
        ui->RefreshN->setDisabled(false);
        //internal=true
        //name = name
        sensor_conf->internal = true;
        sensor_conf->name = name;
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
        sensor_conf->internal = false;
        sensor_conf->topic = "/camera/depth_registered/points";
    }
}

void MainWindow::on_Kinect2SD_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Kinect2 Bridge subscriber as main source of point clouds. Using SD topic (512x424) (/kinect2/SD/points)");
        ui->Topic->setText("/kinect2/SD/points");
        //internal=false
        //topic =
        sensor_conf->internal = false;
        sensor_conf->topic = "/kinect2/SD/points";
    }
}

void MainWindow::on_Kinect2QHD_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Kinect2 Bridge subscriber as main source of point clouds. Using QHD topic (960x540) (/kinect2/QHD/points)");
        ui->Topic->setText("/kinect2/QHD/points");
        //internal=false
        //topic =
        sensor_conf->internal = false;
        sensor_conf->topic = "/kinect2/QHD/points";
    }
}

void MainWindow::on_Kinect2HD_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Kinect2 Bridge subscriber as main source of point clouds. Using HD topic (1920x1080) (/kinect2/HD/points)");
        ui->Topic->setText("/kinect2/HD/points");
        //internal=false
        //topic =
        sensor_conf->internal = false;
        sensor_conf->topic = "/kinect2/HD/points";
    }
}

void MainWindow::on_External_toggled(bool checked)
{
    if(checked){
        QString msg = ui->Topic->text();
        std::string topic = msg.toStdString();
        ui->LoggingConsole->appendPlainText(msg.prepend("* Switching to a custom external subscriber, specified by topic name: "));
        ui->TopicG->setDisabled(false);
        ui->RefreshT->setDisabled(false);
        //internal=false
        //topic =
        sensor_conf->internal = false;
        sensor_conf->topic = topic;
    }
    if(!checked){
        ui->TopicG->setDisabled(true);
        ui->RefreshT->setDisabled(true);
    }
}

void MainWindow::on_RefreshN_clicked()
{
    QString msg = ui->Name->text();
    std::string name = msg.toStdString();
    ui->LoggingConsole->appendPlainText(msg.prepend("* Internal Kinect2 reference frame updated: "));
    //name =
    sensor_conf->name = name;
}

void MainWindow::on_RefreshT_clicked()
{
    QString msg = ui->Topic->text();
    std::string topic = msg.toStdString();
    ui->LoggingConsole->appendPlainText(msg.prepend("* External subscriber topic updated: "));
    //topic =
    sensor_conf->topic = topic;
}
