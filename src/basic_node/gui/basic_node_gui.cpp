#include <basic_node_gui.h>
#include <ui_basic_node_gui.h>
#include <QFileDialog>
//For modular build macros
#include <pacv_config.h>

#include <iostream>

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

QPushButton*
BasicNodeGui::getSaveButt() const
{
    return ui->SaveButt;
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
    ui->CroppingButt->setChecked(value);
    ui->CroppingG->setDisabled(!value);
    config->get("downsampling", value);
    ui->DownsamplingButt->setChecked(value);
    ui->LeafF->setDisabled(!value);
    config->get("segmenting", value);
    ui->SegmentingButt->setChecked(value);
    ui->PlaneF->setDisabled(!value);
    config->get("publish_limits", value);
    ui->PublishLimitsButt->setChecked(value);
    config->get("keep_organized", value);
    ui->OrganizedButt->setChecked(value);
    s_config->get("broadcast_identity_tf", value);
    ui->BrdButt->setChecked(value);
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
        else if (t.compare("/kinect2/sd/points") == 0)
            ui->Kinect2SD->click();
        else if (t.compare("/kinect2/qhd/points") == 0)
            ui->Kinect2QHD->click();
        else if (t.compare("/kinect2/hd/points") == 0)
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
        // re-enable everything
        emit enableDisable(true);
    }
    else if (checked == true){
        ui->MasterDisable->setText("    Master Enable");
        ui->MasterReset->setDisabled(true);
        ui->BaseTab->setDisabled(true);
        ui->SensorTab->setDisabled(true);
        // disable everything
        emit enableDisable(false);
    }
}

void BasicNodeGui::on_CroppingButt_clicked(bool checked)
{
    if(checked){
        ui->CroppingG->setDisabled(false);
        //cropping=true
        config->set("cropping", true);
    }
    if(!checked){
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
        ui->LeafF->setDisabled(false);
        //downsampling=true
       config->set("downsampling", true);
    }
    if(!checked){
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
        ui->PlaneF->setDisabled(false);
        //segmenting=true
        config->set("segmenting", true);
    }
    if(!checked){
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
        //organized=true
        config->set("keep_organized", true);
    }
    if(!checked){
        //organized=false
        config->set("keep_organized", false);
    }
}

void BasicNodeGui::on_Internal_toggled(bool checked)
{
    if(checked){
        std::string name = ui->Name->text().toStdString();
        std::string c_name;
        s_config->get("name", c_name);
        if (c_name.compare(name) !=0){
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
        ui->Topic->setText("/kinect2/sd/points");
        //internal=false
        //topic =
        s_config->set("internal", false);
        std::string t("/kinect2/sd/points");
        s_config->set("topic", t);
        emit sensorChanged();
    }
}

void BasicNodeGui::on_Kinect2QHD_toggled(bool checked)
{
    if(checked){
        ui->Topic->setText("/kinect2/qhd/points");
        //internal=false
        //topic =
        s_config->set("internal", false);
        std::string t( "/kinect2/qhd/points");
        s_config->set("topic", t);
        emit sensorChanged();
    }
}

void BasicNodeGui::on_Kinect2HD_toggled(bool checked)
{
    if(checked){
        ui->Topic->setText("/kinect2/hd/points");
        //internal=false
        //topic =
        s_config->set("internal", false);
        std::string t("/kinect2/hd/points");
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
            ui->Topic->setText(c_topic.c_str());
            msg = ui->Topic->text();
        }
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
    //name =
    s_config->set("name", name);
    emit sensorChanged();
}

void BasicNodeGui::on_RefreshT_clicked()
{
    QString msg = ui->Topic->text();
    std::string topic = msg.toStdString();
    //topic =
    s_config->set("topic", topic);
    emit sensorChanged();
}

void BasicNodeGui::on_SaveButt_clicked()
{
    ui->SaveButt->setDisabled(true);
    if (last_save_location.isEmpty())
        last_save_location = std::getenv("HOME");
    QString filename = QFileDialog::getSaveFileName(this, tr("Save Processed Scene"),
                               last_save_location, tr("Point Clouds (*.pcd)"));
    std::string * fn = new std::string();
    *fn = filename.toStdString();
    if( fn->empty() ){
        ui->SaveButt->setDisabled(false);
        last_save_location.clear();
    }
    else{
        last_save_location = filename;
        emit saveCloud(fn);
    }
}

void BasicNodeGui::on_BrdButt_clicked(bool checked)
{
   s_config->set("broadcast_identity_tf", checked);
}

void BasicNodeGui::on_MasterReset_clicked()
{
    emit reset();
}

void BasicNodeGui::on_SaveConfB_clicked()
{
    ui->SaveConfB->setDisabled(true);
    if (save_conf_loc.isEmpty())
        save_conf_loc = std::getenv("HOME");
    QString filename = QFileDialog::getSaveFileName(this, tr("Save Current Configuration"),
                               save_conf_loc, tr("Configuration Files (*.yaml)"));
    std::string * fn = new std::string();
    *fn = filename.toStdString();
    if ( fn->empty())
        save_conf_loc.clear();
    else{
        save_conf_loc = filename;
        emit saveConf(fn);
    }
    ui->SaveConfB->setDisabled(false);
}

void BasicNodeGui::on_LoadConfB_clicked()
{
    ui->LoadConfB->setDisabled(true);
    if (save_conf_loc.isEmpty())
        save_conf_loc = std::getenv("HOME");
    QString filename = QFileDialog::getOpenFileName(this, tr("Load Configuration"),
                               save_conf_loc, tr("Configuration Files (*.yaml)"));
    std::string * fn = new std::string();
    *fn = filename.toStdString();
    if ( fn->empty())
        save_conf_loc.clear();
    else{
        save_conf_loc = filename;
        emit loadConf(fn);
    }
    ui->LoadConfB->setDisabled(false);
}
