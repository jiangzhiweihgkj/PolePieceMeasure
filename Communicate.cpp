#include "Communicate.h"
#include "ui_Communicate.h"

Communicate::Communicate(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::CommunicateClass())
{
	ui->setupUi(this);
}

Communicate::~Communicate()
{
	delete ui;
}

