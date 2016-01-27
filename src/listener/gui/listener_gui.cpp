#include <listener_gui.h>
#include <ui_listener_gui.h>
#include <QFileDialog>
//For modular build macros

ListenerGui::ListenerGui(const pacv::ListenerConfig::Ptr conf, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ListenerGui), running(false), right(true)
{
    config = conf;
    ui->setupUi(this);
    ui->status->setStyleSheet("QLabel {color : red}");
}
ListenerGui::~ListenerGui()
{
    delete ui;
}

void
ListenerGui::enableDisable(bool enable)
{
    ui->RunningButt->setDisabled(!enable);
    ui->ListenerF->setDisabled(!enable);
}

QPushButton*
ListenerGui::getRunButt() const
{
    return ui->RunningButt;
}
QPushButton*
ListenerGui::getInHandButt() const
{
    return ui->GetInHandButt;
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
    bool value;
    config->get("listen_right_arm", value);
    ui->LRAButt->setChecked(value);
    config->get("listen_left_arm", value);
    ui->LLAButt->setChecked(value);
    config->get("listen_right_hand", value);
    ui->LRHButt->setChecked(value);
    config->get("listen_left_hand", value);
    ui->LLHButt->setChecked(value);
    config->get("remove_right_arm", value);
    ui->RRAButt->setChecked(value);
    config->get("remove_left_arm", value);
    ui->RLAButt->setChecked(value);
    config->get("remove_right_hand", value);
    ui->RRHButt->setChecked(value);
    config->get("remove_left_hand", value);
    ui->RLHButt->setChecked(value);
    config->get("publish_markers", value);
    ui->MarksButt->setChecked(value);
    double val;
    config->get("geometry_scale", val);
    ui->Scale->setValue(val);
}

void ListenerGui::on_RunningButt_clicked()
{
    if (running){
        //kill him
        ui->RunningButt->setText("Spawn me");
        ui->status->setText("Not Running");
        ui->status->setStyleSheet("QLabel {color : red}");
        ui->ListenerF->setDisabled(true);
        ui->ScaleF->setDisabled(true);
        return;
    }
    else{
        //spawn him
        ui->RunningButt->setText("Kill me");
        ui->status->setText("Running");
        ui->status->setStyleSheet("QLabel {color : green}");
        ui->ListenerF->setDisabled(false);
        ui->ScaleF->setDisabled(false);
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
    if (!checked && ui->RRAButt->isChecked())
        ui->RRAButt->click();
    ui->RRAButt->setDisabled(!checked);
}

void ListenerGui::on_LLAButt_clicked(bool checked)
{
    config->set("listen_left_arm", checked);
    if (!checked && ui->RLAButt->isChecked())
        ui->RLAButt->click();
    ui->RLAButt->setDisabled(!checked);
}

void ListenerGui::on_LRHButt_clicked(bool checked)
{
    config->set("listen_right_hand", checked);
    if (!checked && ui->RRHButt->isChecked())
        ui->RRHButt->click();
    ui->RRHButt->setDisabled(!checked);
}

void ListenerGui::on_LLHButt_clicked(bool checked)
{
    config->set("listen_left_hand", checked);
    if (!checked && ui->RLHButt->isChecked())
        ui->RLHButt->click();
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

void ListenerGui::on_Scale_valueChanged(double arg1)
{
    config->set("geometry_scale", arg1);
}

void ListenerGui::on_MarksButt_clicked(bool checked)
{
    config->set("publish_markers",checked);
}

void ListenerGui::on_GetInHandButt_clicked()
{
    ui->GetInHandButt->setDisabled(true);
    if (last_save_location.isEmpty())
        last_save_location = std::getenv("HOME");
    QString filename = QFileDialog::getSaveFileName(this, "Save 'In Hand' Point Cloud",
                               last_save_location, "Point Clouds (*.pcd)");
    std::string * fn = new std::string();
    std::string * hfn = new std::string();
    *fn = filename.toStdString();
    last_save_location = filename;
    if( fn->empty() ){
        ui->GetInHandButt->setDisabled(false);
        last_save_location.clear();
        return;
    }
    if (ui->saveHand->isChecked()){
        if (last_save_location.isEmpty())
            last_save_location = std::getenv("HOME");
        filename = QFileDialog::getSaveFileName(this, "Save Soft Hand Point Cloud",
                                last_save_location, "Point Clouds (*.pcd)");
        *hfn = filename.toStdString();
        if( hfn->empty() ){
            ui->GetInHandButt->setDisabled(false);
            last_save_location.clear();
            return;
        }
    }
    emit saveInHand(right, fn, hfn);
}
