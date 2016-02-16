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
#include <tracker_gui.h>
#include <ui_tracker_gui.h>

TrackerGui::TrackerGui(const pacv::TrackerConfig::Ptr conf, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TrackerGui)
{
    config = conf;
    ui->setupUi(this);
    ui->status->setStyleSheet("QLabel {color : red}");
}
TrackerGui::~TrackerGui()
{
    delete ui;
}


QListWidget*
TrackerGui::getObjList() const
{
    return ui->objects;
}

QPushButton*
TrackerGui::getRunButt() const
{
    return ui->RunningButt;
}
QPushButton*
TrackerGui::getTrackButt() const
{
    return ui->TrackButt;
}
QPushButton*
TrackerGui::getStopButt() const
{
    return ui->StopButt;
}

QPushButton*
TrackerGui::getRefreshButt() const
{
    return ui->RefreshO;
}
QWidget*
TrackerGui::getWidget() const
{
    return ui->TrackerW;
}

void TrackerGui::init()
{
    bool value;
    config->get("publish_bounding_box", value);
    ui->PubButt->setChecked(value);
    config->get("publish_markers", value);
    ui->MarkButt->setChecked(value);
    ui->PubButt->setDisabled(!value);
    config->get("broadcast_tf", value);
    ui->TfButt->setChecked(value);
    ui->StopButt->setDisabled(true);
    ui->TrackButt->setDisabled(true);
    ui->objects->clear();
}

void TrackerGui::disable(bool full)
{
    if (full){
        ui->TrackerW->setDisabled(true);
        return;
    }
    ui->RunningButt->setText("Spawn it");
    ui->status->setText("Not Running");
    ui->status->setStyleSheet("QLabel {color : red}");
    ui->TrackerF->setDisabled(true);
    config->set("spawn", false);
}

void TrackerGui::enable(bool full)
{
    if (full){
        ui->TrackerW->setDisabled(false);
        return;
    }
    ui->RunningButt->setText("Kill it");
    ui->status->setText("Running");
    ui->status->setStyleSheet("QLabel {color : green}");
    ui->TrackerF->setDisabled(false);
    config->set("spawn", true);
    init();
}

void TrackerGui::on_objects_itemSelectionChanged()
{
    ui->TrackButt->setDisabled(false);
}

void TrackerGui::on_TrackButt_clicked()
{
    ui->TrackButt->setDisabled(true);
    ui->StopButt->setDisabled(false);
    ui->objects->setDisabled(true);
    QListWidgetItem *item = ui->objects->currentItem();
    std::string obj = item->text().toStdString();
    emit trackObject(&obj);
}

void TrackerGui::on_StopButt_clicked()
{
    ui->objects->setDisabled(false);
    ui->StopButt->setDisabled(true);
}

void TrackerGui::on_MarkButt_clicked(bool checked)
{
   config->set("publish_markers", checked);
   ui->PubButt->setDisabled(!checked);
}

void TrackerGui::on_PubButt_clicked(bool checked)
{
   config->set("publish_bounding_box", checked);
}

void TrackerGui::on_TfButt_clicked(bool checked)
{
   config->set("broadcast_tf", checked);
}
