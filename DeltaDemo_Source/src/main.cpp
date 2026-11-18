#include "MainWindow.h"
#include <QApplication>
#include <QTimer>
#include <QPixmap>
#include <QDebug>
#include <QTabWidget>
#include <QFont>
#include <QIcon>
int main(int argc,char** argv) {
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication app(argc,argv);app.setApplicationName("DeltaDemo");app.setStyle("Fusion");
    app.setWindowIcon(QIcon(QStringLiteral(":/icons/app.png")));
#ifdef _WIN32
    app.setFont(QFont(QStringLiteral("Microsoft YaHei UI"),9));
#endif
    demo::MainWindow window(!app.arguments().contains("--smoke-test"));window.show();
    // UI smoke test never connects, loads a customer configuration, or calls motion.
    if(app.arguments().contains("--smoke-test"))QTimer::singleShot(350,&window,[&]{
        const auto error=window.checkForm();qInfo()<<"Default form validation:"<<error;
        bool saved=true;
        auto* tabs=window.findChild<QTabWidget*>();
        for(int i=0;i<tabs->count();++i){tabs->setCurrentIndex(i);app.processEvents();saved=window.grab().save(QCoreApplication::applicationDirPath()+QString("/preview-%1.png").arg(i))&&saved;}
        tabs->setCurrentIndex(0);app.processEvents();saved=window.grab().save(QCoreApplication::applicationDirPath()+"/preview.png")&&saved;
        app.exit(error.isEmpty()&&saved?0:1);
    });
    return app.exec();
}
