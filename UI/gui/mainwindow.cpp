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

void MainWindow::on_DisableCheckBox_stateChanged(int state)
{
    if (state == Qt::Unchecked){
        // re-enable everything
    }
    else if (state == Qt::Checked){
        // disable everything
    }
    else{
        //partial state not-defined
    }
}

void MainWindow::on_MasterReset_pressed()
{
   //Reset Everything
}
