#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <QMainWindow> 

QT_BEGIN_NAMESPACE
namespace Ui { class CommunicationClass; };
QT_END_NAMESPACE

class Communication : public QMainWindow
{
	Q_OBJECT

public:
	Communication(QWidget *parent = nullptr);
	~Communication();

private:
	Ui::CommunicationClass *ui;
};

#endif // COMMUNICATION_H
