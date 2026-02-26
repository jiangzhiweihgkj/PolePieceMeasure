#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include <QWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class CommunicateClass; };
QT_END_NAMESPACE

class Communicate : public QWidget
{
	Q_OBJECT

public:
	Communicate(QWidget *parent = nullptr);
	~Communicate();

private:
	Ui::CommunicateClass *ui;
};
#endif//COMMUNICATE_H
