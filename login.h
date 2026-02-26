#ifndef LOGIN_H
#define LOGIN_H

#include <QWidget>

 QT_BEGIN_NAMESPACE 
namespace Ui {
class LogIn;
}
QT_END_NAMESPACE

class LogIn : public QWidget
{
    Q_OBJECT

public:
    explicit LogIn(QWidget *parent = nullptr);
    ~LogIn();

private:
    Ui::LogIn *ui;
};

#endif // LOGIN_H
