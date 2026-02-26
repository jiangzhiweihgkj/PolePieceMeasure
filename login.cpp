#include "login.h"
#include "ui_login.h"

LogIn::LogIn(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LogIn)
{
    ui->setupUi(this);

    //
//    connect(ui->btnChangePassword,&QPushButton::clicked,[=](){
//        ui->stackedWidget->setCurrentIndex(1);
//    });
}

LogIn::~LogIn()
{
    delete ui;
}
