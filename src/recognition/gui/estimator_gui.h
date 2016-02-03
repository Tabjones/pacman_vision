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
#ifndef _ESTIMATOR_GUI_H_
#define _ESTIMATOR_GUI_H_

#include <QMainWindow>
#include <QPushButton>
#include <recognition/estimator_config.hpp>

namespace Ui {
class EstimatorGui;
}

class EstimatorGui : public QMainWindow
{
    Q_OBJECT

public:
    explicit EstimatorGui(const pacv::EstimatorConfig::Ptr conf, QWidget *parent = 0);
    ~EstimatorGui();
    QWidget* getWidget() const;
    QPushButton* getRunButt() const;
    QPushButton* getEstButt() const;
    void enable(bool full=false);
    void disable(bool full=false);

private slots:
    void on_CalibButt_clicked(bool checked);
    void on_Cluster_valueChanged(double arg1);
    void on_Iter_valueChanged(int arg1);
    void on_Neigh_valueChanged(int arg1);
    void on_EstimationButt_clicked();
    void on_SuccessButt_clicked(bool checked);
    void on_rmse_valueChanged(double arg1);
    void on_MarksButt_clicked(bool checked);
    void on_TfButt_clicked(bool checked);

private:
    void init();
    Ui::EstimatorGui *ui;
    pacv::EstimatorConfig::Ptr config;
};

#endif
