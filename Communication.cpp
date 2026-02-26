#include "Communication.h"
#include "ui_Communication.h"

Communication::Communication(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::CommunicationClass())
{
	ui->setupUi(this);
}

Communication::~Communication()
{
	delete ui;
}

