#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_MasterDisable_clicked(bool checked)
{
    if (checked == false){
        ui->MasterDisable->setText("    Master Disable");
        ui->MasterReset->setDisabled(false);
        ui->LoggingConsole->appendPlainText("Functionality is globally resumed.");
        // re-enable everything
    }
    else if (checked == true){
        ui->MasterDisable->setText("    Master Enable");
        ui->LoggingConsole->appendPlainText("Functionality is globally disabled.");
        ui->MasterReset->setDisabled(true);
        // disable everything
    }
}

void MainWindow::on_MasterReset_pressed()
{
   ui->LoggingConsole->appendPlainText("Issued a MASTER RESET!");
   //Reset Everything
}
