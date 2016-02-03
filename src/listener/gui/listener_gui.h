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
#ifndef _LISTENER_GUI_H_
#define _LISTENER_GUI_H_

#include <QMainWindow>
#include <QPushButton>
#include <listener/listener_config.hpp>

namespace Ui {
class ListenerGui;
}

class ListenerGui : public QMainWindow
{
    Q_OBJECT

public:
    explicit ListenerGui(const pacv::ListenerConfig::Ptr conf, QWidget *parent = 0);
    ~ListenerGui();
    QWidget* getWidget() const;
    QPushButton* getRunButt() const;
    QPushButton* getInHandButt() const;
    void enable(bool full=false);
    void disable(bool full=false);

signals:
    void saveInHand(bool r, std::string *obj, std::string *hand);

private slots:
    void on_SwitchHandButt_clicked();
    void on_LRAButt_clicked(bool checked);
    void on_LLAButt_clicked(bool checked);
    void on_LRHButt_clicked(bool checked);
    void on_LLHButt_clicked(bool checked);
    void on_RRAButt_clicked(bool checked);
    void on_RLAButt_clicked(bool checked);
    void on_RRHButt_clicked(bool checked);
    void on_RLHButt_clicked(bool checked);
    void on_Scale_valueChanged(double arg1);
    void on_MarksButt_clicked(bool checked);
    void on_GetInHandButt_clicked();

private:
    void init();
    bool right;
    Ui::ListenerGui *ui;
    pacv::ListenerConfig::Ptr config;
    QString last_save_location;
};

#endif
