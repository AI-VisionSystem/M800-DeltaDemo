#pragma once
#include "RobotController.h"
#include <QMainWindow>
#include <QVector>
#include <functional>
class QDoubleSpinBox;class QSpinBox;class QComboBox;class QTableWidget;
class QLineEdit;class QLabel;class QPlainTextEdit;class QTimer;class QPushButton;
namespace demo {
class MainWindow : public QMainWindow {
public:
    explicit MainWindow(bool autoOpen=false);
    QString checkForm() const;
protected:
    void closeEvent(QCloseEvent* event) override;
    bool eventFilter(QObject* object,QEvent* event) override;
private:
    friend struct WindowTestAccess;
    RobotController m_robot;
    bool m_loading=false,m_jogHeld=false,m_fault=false;
    bool m_closing=false;
    QString m_pollError;
    QString m_lastFault;
    QLineEdit* m_host=nullptr;
    QSpinBox* m_address=nullptr;
    QTableWidget *m_axes=nullptr,*m_points=nullptr,*m_statusTable=nullptr;
    std::array<QDoubleSpinBox*,4> m_geometry{},m_target{},m_speed{};
    QVector<std::array<QWidget*,4>> m_axisEditors;
    bool m_homeZPending=false;
    QSpinBox* m_estopInput=nullptr;
    QComboBox* m_estopLevel=nullptr;
    QSpinBox* m_ioBit=nullptr;
    QLabel *m_state=nullptr,*m_position=nullptr,*m_ioState=nullptr,*m_connectionState=nullptr;
    QPlainTextEdit* m_log=nullptr;
    QPushButton *m_connect=nullptr,*m_disconnect=nullptr,*m_load=nullptr;
    QPushButton *m_enableServos=nullptr,*m_disableServos=nullptr;
    QVector<QWidget*> m_settingsPanels;
    QTimer* m_timer=nullptr;
    QWidget* controlPage();
    QWidget* parametersPage();
    QWidget* homingPage();
    QWidget* axisEditor(int field,int axis);
    QTableWidget* parameterTable(const QStringList& keys);
    QString applyManualSpeeds();
    QString startHome();
    QString setAllServos(bool enable);
    void synchronizeHomeZ(const Status& status);
    QWidget* diagnosticPage();
    QPushButton* action(const QString& title,QWidget* parent,std::function<QString()> fn);
    QDoubleSpinBox* number(double min,double max,int decimals=3);
    void addPoint(const Point& p);
    Settings settings() const;
    void setSettings(const Settings& s);
    void changed();
    void axisChanged(int field,int axis);
    void poll();
    void logResult(const QString& action,const QString& error);
    void stopJog();
    void save();
    void load();
};
}
