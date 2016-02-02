// Software License Agreement (BSD License)
//
//   PaCMan Vision (PaCV) - https://github.com/Tabjones/pacman_vision
//   Copyright (c) 2015-2016, Federico Spinelli (fspinelli@gmail.com)
//   All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder(s) nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include <listener_gui.h>
#include <ui_listener_gui.h>
#include <QFileDialog>
//For modular build macros

ListenerGui::ListenerGui(const pacv::ListenerConfig::Ptr conf, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ListenerGui), right(true)
{
    config = conf;
    ui->setupUi(this);
    ui->status->setStyleSheet("QLabel {color : red}");
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

void ListenerGui::disable(bool full)
{
    if(full){
        ui->ListenerW->setDisabled(true);
        return;
    }
    ui->RunningButt->setText("Spawn it");
    ui->status->setText("Not Running");
    ui->status->setStyleSheet("QLabel {color : red}");
    ui->ListenerF->setDisabled(true);
    ui->ScaleF->setDisabled(true);
    config->set("running", false);
}

void ListenerGui::enable(bool full)
{
    if(full){
        ui->ListenerW->setDisabled(false);
        return;
    }
    ui->RunningButt->setText("Kill it");
    ui->status->setText("Running");
    ui->status->setStyleSheet("QLabel {color : green}");
    ui->ListenerF->setDisabled(false);
    ui->ScaleF->setDisabled(false);
    config->set("running", true);
    init();
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
