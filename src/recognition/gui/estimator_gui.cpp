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
#include <estimator_gui.h>
#include <ui_estimator_gui.h>
//For modular build macros

EstimatorGui::EstimatorGui(const pacv::EstimatorConfig::Ptr conf, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::EstimatorGui)
{
    config = conf;
    ui->setupUi(this);
    ui->status->setStyleSheet("QLabel {color : red}");
}
EstimatorGui::~EstimatorGui()
{
    delete ui;
}

QPushButton*
EstimatorGui::getRunButt() const
{
    return ui->RunningButt;
}
QPushButton*
EstimatorGui::getEstButt() const
{
    return ui->EstimationButt;
}

QWidget*
EstimatorGui::getWidget() const
{
    return ui->EstimatorW;
}

void EstimatorGui::init()
{
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
}
void
EstimatorGui::disable(bool full){
    if (full){
        ui->EstimatorW->setDisabled(true);
        return;
    }
    ui->RunningButt->setText("Spawn it");
    ui->status->setText("Not Running");
    ui->status->setStyleSheet("QLabel {color : red}");
    ui->EstimatorF->setDisabled(true);
    ui->ClusterF->setDisabled(true);
    ui->IterF->setDisabled(true);
    ui->NeighF->setDisabled(true);
    ui->rmseF->setDisabled(true);
    config->set("running", false);
}
void EstimatorGui::enable(bool full)
{
    if (full){
        ui->EstimatorW->setDisabled(false);
        return;
    }
    ui->RunningButt->setText("Kill it");
    ui->status->setText("Running");
    ui->status->setStyleSheet("QLabel {color : green}");
    ui->EstimatorF->setDisabled(false);
    ui->ClusterF->setDisabled(false);
    ui->IterF->setDisabled(false);
    ui->NeighF->setDisabled(false);
    ui->rmseF->setDisabled(false);
    config->set("running", true);
    init();
}

void EstimatorGui::on_CalibButt_clicked(bool checked)
{
    config->set("object_calibration", checked);
}

void EstimatorGui::on_Cluster_valueChanged(double arg1)
{
    config->set("cluster_tol", arg1);
}

void EstimatorGui::on_Iter_valueChanged(int arg1)
{
    config->set("iterations", arg1);
}

void EstimatorGui::on_Neigh_valueChanged(int arg1)
{
    config->set("neighbors", arg1);
}

void EstimatorGui::on_EstimationButt_clicked()
{
    ui->EstimationButt->setDisabled(true);
    ui->EstimationButt->setText("Pose Estimation (Running...)");
}

void EstimatorGui::on_SuccessButt_clicked(bool checked)
{
   config->set("always_success", checked);
}

void EstimatorGui::on_rmse_valueChanged(double arg1)
{
   config->set("rmse_thresh", arg1);
}

void EstimatorGui::on_MarksButt_clicked(bool checked)
{
    config->set("publish_markers", checked);
}

void EstimatorGui::on_TfButt_clicked(bool checked)
{
   config->set("broadcast_tf", checked);
}
