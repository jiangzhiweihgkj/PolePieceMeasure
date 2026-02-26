
#include <QWidget>
#include "ui_DataExcel.h"

QT_BEGIN_NAMESPACE
namespace Ui { class DataExcelClass; };
QT_END_NAMESPACE

class DataExcel : public QWidget
{
	Q_OBJECT

public:
	DataExcel(QWidget *parent = nullptr);
	~DataExcel();

private:
	Ui::DataExcelClass *ui;
};

