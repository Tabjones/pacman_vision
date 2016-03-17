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
#ifndef _BASIC_NODE_GUI_H_
#define _BASIC_NODE_GUI_H_

#include <QMainWindow>
#include <QPushButton>
#include <basic_node/basic_node_config.hpp>
#include <basic_node/sensor_config.hpp>

namespace Ui {
class BasicNodeGui;
}

class BasicNodeGui : public QMainWindow
{
    Q_OBJECT

public:
    explicit BasicNodeGui(const pacv::BasicConfig::Ptr conf,
                          const pacv::SensorConfig::Ptr s_conf, QWidget *parent = 0) ;
    ~BasicNodeGui();
    void addTab(QWidget* tab, const QString title);
    QPushButton* getSaveButt() const;
    void init();

signals:
    void boxChanged();
    void sensorChanged();
    void saveCloud(std::string* );
    void enableDisable(bool enable);
    void reset();
    void saveConf(std::string* );
    void loadConf(std::string* );

private slots:
    void on_MasterDisable_clicked(bool checked);
    void on_CroppingButt_clicked(bool checked);
    void on_Xmax_valueChanged(double arg1);
    void on_Xmin_valueChanged(double arg1);
    void on_Ymin_valueChanged(double arg1);
    void on_Ymax_valueChanged(double arg1);
    void on_Zmin_valueChanged(double arg1);
    void on_Zmax_valueChanged(double arg1);
    void on_PublishLimitsButt_clicked(bool checked);
    void on_DownsamplingButt_clicked(bool checked);
    void on_Leaf_valueChanged(double arg1);
    void on_SegmentingButt_clicked(bool checked);
    void on_Plane_valueChanged(double arg1);
    void on_OrganizedButt_clicked(bool checked);
    void on_Internal_toggled(bool checked);
    void on_Asus_toggled(bool checked);
    void on_Kinect2SD_toggled(bool checked);
    void on_Kinect2QHD_toggled(bool checked);
    void on_Kinect2HD_toggled(bool checked);
    void on_External_toggled(bool checked);
    void on_RefreshN_clicked();
    void on_RefreshT_clicked();
    void on_SaveButt_clicked();
    void on_BrdButt_clicked(bool checked);
    void on_MasterReset_clicked();
    void on_SaveConfB_clicked();
    void on_LoadConfB_clicked();

    void on_ColorButt_clicked(bool checked);

    void on_OutiliersButt_clicked(bool checked);

    void on_OutliersK_valueChanged(int arg1);

    void on_OutliersS_valueChanged(double arg1);

    void on_Color_valueChanged(double arg1);

private:
    Ui::BasicNodeGui *ui;
    pacv::Box lim;
    pacv::BasicConfig::Ptr config;
    pacv::SensorConfig::Ptr s_config;
    QString last_save_location, save_conf_loc;
};

#endif // BASIC_NODE_GUI_H
