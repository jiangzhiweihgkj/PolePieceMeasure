#ifndef SUBWINDOW_H
#define SUBWINDOW_H
#include <QWidget>
  
 QT_BEGIN_NAMESPACE
namespace Ui { class SubWindowClass; };
QT_END_NAMESPACE
 
class SubWindow : public QWidget
{
	Q_OBJECT

public:
	SubWindow(QWidget *parent = nullptr);
	~SubWindow(); 
 
private:
	Ui::SubWindowClass *ui;
  };
#endif // SUBWINDOWCLASS_H 
