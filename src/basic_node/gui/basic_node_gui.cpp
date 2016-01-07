#include <basic_node_gui.h>
#include <ui_basic_node_gui.h>
//For modular build macros
#include <pacv_config.h>

BasicNodeGui::BasicNodeGui(const pacv::BasicConfig::Ptr conf,
                           const pacv::SensorConfig::Ptr s_conf, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::BasicNodeGui)
{
    config = conf;
    s_config = s_conf;
    ui->setupUi(this);
    init();
}
BasicNodeGui::~BasicNodeGui()
{
    delete ui;
}

void
BasicNodeGui::addTab(QWidget* tab, const QString title)
{
    ui->Tabs->addItem(tab, title);
}

void BasicNodeGui::init()
{
    bool value;
    config->get("cropping", value);
    if (value != ui->CroppingButt->isChecked())
        ui->CroppingButt->click();
    config->get("downsampling", value);
    if (value != ui->DownsamplingButt->isChecked())
        ui->DownsamplingButt->click();
    config->get("segmenting", value);
    if (value != ui->SegmentingButt->isChecked())
        ui->SegmentingButt->click();
    config->get("publish_limits", value);
    if (value != ui->PublishLimitsButt->isChecked())
        ui->PublishLimitsButt->click();
    config->get("keep_organized", value);
    if (value != ui->OrganizedButt->isChecked())
        ui->OrganizedButt->click();
    double ls,pt;
    config->get("downsampling_leaf_size", ls);
    config->get("plane_tolerance", pt);
    ui->Leaf->setValue(ls);
    ui->Plane->setValue(pt);
    config->get("filter_limits", lim);
    ui->Xmin->setValue(lim.x1);
    ui->Xmax->setValue(lim.x2);
    ui->Ymin->setValue(lim.y1);
    ui->Ymax->setValue(lim.y2);
    ui->Zmin->setValue(lim.z1);
    ui->Zmax->setValue(lim.z2);
    std::string name, t;
    s_config->get("topic",t);
    ui->Topic->setText(QString(t.c_str()));
#ifndef PACV_KINECT2_SUPPORT
    value = false;
    ui->Internal->setDisabled(true);
#endif
#ifdef PACV_KINECT2_SUPPORT
    s_config->get("name", name);
    s_config->get("internal", value);
    ui->Name->setText(QString(name.c_str()));
    if (value)
        ui->Internal->click();
#endif
    if (!value){
        if (t.compare("/camera/depth_registered/points") == 0)
            ui->Asus->click();
        else if (t.compare("/kinect2/SD/points") == 0)
            ui->Kinect2SD->click();
        else if (t.compare("/kinect2/QHD/points") == 0)
            ui->Kinect2QHD->click();
        else if (t.compare("/kinect2/HD/points") == 0)
            ui->Kinect2HD->click();
        else
            ui->External->click();
    }
}

void BasicNodeGui::on_MasterDisable_clicked(bool checked)
{
    if (checked == false){
        ui->MasterDisable->setText("    Master Disable");
        ui->MasterReset->setDisabled(false);
        ui->BaseTab->setDisabled(false);
        ui->SensorTab->setDisabled(false);
        ui->LoggingConsole->appendPlainText("* Functionality is globally resumed.");
        // re-enable everything TODO
    }
    else if (checked == true){
        ui->MasterDisable->setText("    Master Enable");
        ui->LoggingConsole->appendPlainText("* Functionality is globally disabled.");
        ui->MasterReset->setDisabled(true);
        ui->BaseTab->setDisabled(true);
        ui->SensorTab->setDisabled(true);
        // disable everything TODO
    }
}

void BasicNodeGui::on_MasterReset_pressed()
{
   ui->LoggingConsole->appendPlainText("* Issued a MASTER RESET!");
   //Reset Everything TODO
}

void BasicNodeGui::on_CroppingButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Cropping Filter");
        ui->CroppingG->setDisabled(false);
        //cropping=true
        config->set("cropping", true);
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Cropping Filter");
        ui->CroppingG->setDisabled(true);
        //cropping=false
        config->set("cropping", false);
    }
}

void BasicNodeGui::on_Xmin_valueChanged(double arg1)
{
    //limx1 = arg1
    lim.x1 = arg1;
    config->set("filter_limits", lim);
    emit boxChanged();
}

void BasicNodeGui::on_Xmax_valueChanged(double arg1)
{
   //limx2 = arg1
    lim.x2 = arg1;
    config->set("filter_limits", lim);
    emit boxChanged();
}

void BasicNodeGui::on_Ymin_valueChanged(double arg1)
{
   //limy1 = arg1
    lim.y1 = arg1;
    config->set("filter_limits", lim);
    emit boxChanged();
}

void BasicNodeGui::on_Ymax_valueChanged(double arg1)
{
   //limy2 = arg1
    lim.y2 = arg1;
    config->set("filter_limits", lim);
    emit boxChanged();
}

void BasicNodeGui::on_Zmin_valueChanged(double arg1)
{
   //limz1 = arg1
    lim.z1 = arg1;
    config->set("filter_limits", lim);
    emit boxChanged();
}

void BasicNodeGui::on_Zmax_valueChanged(double arg1)
{
   //limz2 = arg1
    lim.z2 = arg1;
    config->set("filter_limits", lim);
    emit boxChanged();
}

void BasicNodeGui::on_PublishLimitsButt_clicked(bool checked)
{
   if(checked){
       //publishlimits = true
       config->set("publish_limits", true);
   }
   if(!checked){
       //publishlimits = false
       config->set("publish_limits", false);
   }
}

void BasicNodeGui::on_DownsamplingButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Downsampling");
        ui->LeafF->setDisabled(false);
        //downsampling=true
       config->set("downsampling", true);
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Downsampling");
        ui->LeafF->setDisabled(true);
        //downsampling=false
       config->set("downsampling", false);
    }
}

void BasicNodeGui::on_Leaf_valueChanged(double arg1)
{
   //leaf_size = arg1
   config->set("downsampling_leaf_size", arg1);
}

void BasicNodeGui::on_SegmentingButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Plane Segmentation");
        ui->PlaneF->setDisabled(false);
        //segmenting=true
        config->set("segmenting", true);
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Plane Segmentation");
        ui->PlaneF->setDisabled(true);
        //segmenting=false
        config->set("segmenting", false);
    }
}

void BasicNodeGui::on_Plane_valueChanged(double arg1)
{
    //tolerance=arg1
    config->set("plane_tolerance", arg1);
}

void BasicNodeGui::on_OrganizedButt_clicked(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Enabling Keep PointCloud organized, if possible. Note that downsampling and plane segmentation, break organized cloud structure.");
        //organized=true
        config->set("keep_organized", true);
    }
    if(!checked){
        ui->LoggingConsole->appendPlainText("* Disabling Keep PointCloud organized.");
        //organized=false
        config->set("keep_organized", false);
    }
}

void BasicNodeGui::on_Internal_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to internal kinect2 processor as main source of point clouds. Using the reference frame specified.");
        std::string name = ui->Name->text().toStdString();
        std::string c_name;
        s_config->get("name", c_name);
        if (c_name.compare(name) !=0){
            ui->LoggingConsole->appendPlainText("* Updating displayed kinect2 name...");
            ui->Name->setText(c_name.c_str());
        }
        ui->NameG->setDisabled(false);
        ui->RefreshN->setDisabled(false);
        //internal=true
        s_config->set("internal", true);
        emit sensorChanged();
    }
    if(!checked){
        ui->NameG->setDisabled(true);
        ui->RefreshN->setDisabled(true);
    }
}

void BasicNodeGui::on_Asus_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Asus Xtion subscriber as main source of point clouds. (/camera/depth_registered/points)");
        ui->Topic->setText("/camera/depth_registered/points");
        //internal=false
        //topic =
        s_config->set("internal", false);
        std::string t ("/camera/depth_registered/points");
        s_config->set("topic", t);
        emit sensorChanged();
    }
}

void BasicNodeGui::on_Kinect2SD_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Kinect2 Bridge subscriber as main source of point clouds. Using SD topic (512x424) (/kinect2/SD/points)");
        ui->Topic->setText("/kinect2/SD/points");
        //internal=false
        //topic =
        s_config->set("internal", false);
        std::string t("/kinect2/SD/points");
        s_config->set("topic", t);
        emit sensorChanged();
    }
}

void BasicNodeGui::on_Kinect2QHD_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Kinect2 Bridge subscriber as main source of point clouds. Using QHD topic (960x540) (/kinect2/QHD/points)");
        ui->Topic->setText("/kinect2/QHD/points");
        //internal=false
        //topic =
        s_config->set("internal", false);
        std::string t( "/kinect2/QHD/points");
        s_config->set("topic", t);
        emit sensorChanged();
    }
}

void BasicNodeGui::on_Kinect2HD_toggled(bool checked)
{
    if(checked){
        ui->LoggingConsole->appendPlainText("* Switching to external Kinect2 Bridge subscriber as main source of point clouds. Using HD topic (1920x1080) (/kinect2/HD/points)");
        ui->Topic->setText("/kinect2/HD/points");
        //internal=false
        //topic =
        s_config->set("internal", false);
        std::string t("/kinect2/HD/points");
        s_config->set("topic", t);
        emit sensorChanged();
    }
}

void BasicNodeGui::on_External_toggled(bool checked)
{
    if(checked){
        QString msg = ui->Topic->text();
        std::string topic = msg.toStdString();
        std::string c_topic;
        s_config->get("topic", c_topic);
        if (c_topic.compare(topic) !=0){
            ui->LoggingConsole->appendPlainText("* Updating displayed topic...");
            ui->Topic->setText(c_topic.c_str());
            msg = ui->Topic->text();
        }
        ui->LoggingConsole->appendPlainText(msg.prepend("* Switching to a custom external subscriber, specified by topic name: "));
        ui->TopicG->setDisabled(false);
        ui->RefreshT->setDisabled(false);
        //internal=false
        s_config->set("internal", false);
        emit sensorChanged();
    }
    if(!checked){
        ui->TopicG->setDisabled(true);
        ui->RefreshT->setDisabled(true);
    }
}

void BasicNodeGui::on_RefreshN_clicked()
{
    QString msg = ui->Name->text();
    std::string name = msg.toStdString();
    ui->LoggingConsole->appendPlainText(msg.prepend("* Internal Kinect2 reference frame updated: "));
    //name =
    s_config->set("name", name);
    emit sensorChanged();
}

void BasicNodeGui::on_RefreshT_clicked()
{
    QString msg = ui->Topic->text();
    std::string topic = msg.toStdString();
    ui->LoggingConsole->appendPlainText(msg.prepend("* External subscriber topic updated: "));
    //topic =
    s_config->set("topic", topic);
    emit sensorChanged();
}
