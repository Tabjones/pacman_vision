#include <tracker_gui.h>
#include <ui_tracker_gui.h>
//For modular build macros
#include <pacv_config.h>

TrackerGui::TrackerGui(const pacv::TrackerConfig::Ptr conf, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::TrackerGui), running(false)
{
    config = conf;
    ui->setupUi(this);
    init();
}
TrackerGui::~TrackerGui()
{
    delete ui;
}

QPushButton*
TrackerGui::getRunButt() const
{
    return ui->RunningButt;
}
QPushButton*
TrackerGui::getTrackButt() const
{
    //return ui->EstimationButt;
}
QPushButton*
TrackerGui::getStopButt() const
{
    //return ui->EstimationButt;
}
QWidget*
TrackerGui::getWidget() const
{
    return ui->TrackerW;
}

void
TrackerGui::setRunning(const bool run)
{
    running=run;
}

void TrackerGui::init()
{
    bool value;
    config->get("publish_bounding_box", value);
    ui->CalibButt->setChecked(value);

//    double val;
//    config->get("cluster_tol", val);
//    ui->Cluster->setValue(val);
//    int v;
//    config->get("iterations", v);
//    ui->Iter->setValue(v);
//    config->get("neighbors", v);
//    ui->Neigh->setValue(v);
//    config->get("always_success", value);
//    ui->SuccessButt->setChecked(value);
//    config->get("rmse_thresh", val);
//    ui->rmse->setValue(val);
    ui->status->setStyleSheet("QLabel {color : red}");
}

void TrackerGui::on_RunningButt_clicked()
{
    if (running){
        //kill him
        ui->RunningButt->setText("Spawn me");
        ui->status->setText("Not Running");
        ui->status->setStyleSheet("QLabel {color : red}");
        ui->TrackerF->setDisabled(true);
        return;
    }
    else{
        //spawn him
        ui->RunningButt->setText("Kill me");
        ui->status->setText("Running");
        ui->status->setStyleSheet("QLabel {color : green}");
        ui->TrackerF->setDisabled(false);
        return;
    }
}
