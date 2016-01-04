#include "basic_node_gui.h"
#include "ui_basic_node_gui.h"

BasicNodeGui::BasicNodeGui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::BasicNodeGui)
{
    ui->setupUi(this);
}

BasicNodeGui::~BasicNodeGui()
{
    delete ui;
}
