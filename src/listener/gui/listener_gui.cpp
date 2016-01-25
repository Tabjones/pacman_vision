#include <listener_gui.h>
#include <ui_listener_gui.h>
//For modular build macros

ListenerGui::ListenerGui(const pacv::ListenerConfig::Ptr conf, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ListenerGui), running(false), right(true)
{
    config = conf;
    ui->setupUi(this);
    init();
}
ListenerGui::~ListenerGui()
{
    delete ui;
}

QPushButton*
ListenerGui::getRunButt() const
{
    return ui->RunningButt;
}
QWidget*
ListenerGui::getWidget() const
{
    return ui->ListenerW;
}
void
ListenerGui::setRunning(const bool run)
{
    running=run;
}

void ListenerGui::init()
{
/*
    bool value;
    config->get("object_calibration", value);
    ui->CalibButt->setChecked(value);
    config->get("broadcast_tf", value);
    ui->TfButt->setChecked(value);
    config->get("publish_markers", value);
    ui->MarksButt->setChecked(value);
    double val;
    config->get("cluster_tol", val);
    ui->Cluster->setValue(val);
    int v;
    config->get("iterations", v);
    ui->Iter->setValue(v);
    config->get("neighbors", v);
    ui->Neigh->setValue(v);
    config->get("always_success", value);
    ui->SuccessButt->setChecked(value);
    config->get("rmse_thresh", val);
    ui->rmse->setValue(val);
    ui->status->setStyleSheet("QLabel {color : red}");
    */
}

void ListenerGui::on_RunningButt_clicked()
{
    if (running){
        //kill him
        ui->RunningButt->setText("Spawn me");
        ui->status->setText("Not Running");
        ui->status->setStyleSheet("QLabel {color : red}");
        ui->ListenerF->setDisabled(true);
        return;
    }
    else{
        //spawn him
        ui->RunningButt->setText("Kill me");
        ui->status->setText("Running");
        ui->status->setStyleSheet("QLabel {color : green}");
        ui->ListenerF->setDisabled(false);
        return;
    }
}

void ListenerGui::on_SwitchHandButt_clicked()
{
    if (right){
        right = false;
        ui->GetInHandButt->setText("Save Object in Left Hand ...");
        return;
    }
    else{
        right = true;
        ui->GetInHandButt->setText("Save Object in Right Hand ...");
        return;
    }
}

void ListenerGui::on_LRAButt_clicked(bool checked)
{
    config->set("listen_right_arm", checked);
    if (!checked && ui->RRAButt->cli)
    ui->RRAButt->setDisabled(!checked);
}

void ListenerGui::on_LLAButt_clicked(bool checked)
{
    config->set("listen_left_arm", checked);
    ui->RLAButt->setDisabled(!checked);
}

void ListenerGui::on_LRHButt_clicked(bool checked)
{
    config->set("listen_right_hand", checked);
    ui->RRHButt->setDisabled(!checked);
}

void ListenerGui::on_LLHButt_clicked(bool checked)
{
    config->set("listen_left_hand", checked);
    ui->RLHButt->setDisabled(!checked);
}

void ListenerGui::on_RRAButt_clicked(bool checked)
{
   config->set("remove_right_arm", checked);
}

void ListenerGui::on_RLAButt_clicked(bool checked)
{
   config->set("remove_left_arm", checked);
}

void ListenerGui::on_RRHButt_clicked(bool checked)
{
   config->set("remove_right_hand", checked);
}

void ListenerGui::on_RLHButt_clicked(bool checked)
{
   config->set("remove_left_hand", checked);
}
