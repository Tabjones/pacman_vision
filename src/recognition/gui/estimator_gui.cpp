#include <estimator_gui.h>
#include <ui_estimator_gui.h>
//For modular build macros

EstimatorGui::EstimatorGui(const pacv::EstimatorConfig::Ptr conf, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::EstimatorGui), running(false)
{
    config = conf;
    ui->setupUi(this);
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

void
EstimatorGui::setRunning(const bool run)
{
    running=run;
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

void EstimatorGui::on_RunningButt_clicked()
{
    if (running){
        //kill him
        ui->RunningButt->setText("Spawn me");
        ui->status->setText("Not Running");
        ui->status->setStyleSheet("QLabel {color : red}");
        ui->EstimatorF->setDisabled(true);
        ui->ClusterF->setDisabled(true);
        ui->IterF->setDisabled(true);
        ui->NeighF->setDisabled(true);
        ui->rmseF->setDisabled(true);
        return;
    }
    else{
        //spawn him
        ui->RunningButt->setText("Kill me");
        ui->status->setText("Running");
        ui->status->setStyleSheet("QLabel {color : green}");
        ui->EstimatorF->setDisabled(false);
        ui->ClusterF->setDisabled(false);
        ui->IterF->setDisabled(false);
        ui->NeighF->setDisabled(false);
        ui->rmseF->setDisabled(false);
        return;
    }
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
