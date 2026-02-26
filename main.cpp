#include "PolePieceMeasureWin.h"
#include <QtWidgets/QApplication>
#include <QSharedMemory>
#include <QSystemSemaphore>
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);  app.setWindowIcon(QIcon(":/icon/HGTECH.ico"));

    /* 1. 先抢一个“系统信号量”，防止 QSharedMemory 段异常时死锁 */
    QSystemSemaphore sema("myAppSema", 1);   // 名字随便起，保持唯一
    sema.acquire();                            // 拿到通行证

    /* 2. 尝试创建共享内存段 */
    QSharedMemory mem("myAppSharedMem");       // 关键：名字全局唯一
    bool alreadyRunning = true;
    if (mem.create(1)) {                       // 创建成功 → 第一个实例
        alreadyRunning = false;
    }
    else if (mem.error() == QSharedMemory::AlreadyExists) {
        /* 段已存在，再 attach 一下，看是不是真的有人在用 */
        if (mem.attach() && mem.lock() && mem.unlock() && mem.detach()) {
            alreadyRunning = true;             // 能 attach 说明真有人在用
        }
        else {
            // 段残留但无主，强行删掉再建
            mem.attach();
            mem.detach();
            alreadyRunning = !mem.create(1);
        }
    }

    sema.release();                            // 释放通行证

    /* 3. 如果已存在实例，弹提示并退出 */
    if (alreadyRunning) {
        QMessageBox::warning(nullptr, QStringLiteral("提示"),
            QStringLiteral("程序已经运行，请勿重复启动！"));
        return 0;
    }


    PolePieceMeasureWin window; 
   // window.show();
    window.showMaximized();
    return app.exec();
}
