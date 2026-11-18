#pragma once
#include "Settings.h"
#include <QElapsedTimer>

namespace demo {
struct AxisStatus {
    long pulses=0, encoder=0;
    double angle=0, speed=0;
    bool enabled=false, homed=false, running=false, homing=false;
    bool alarm=false, servoAlarm=false, positiveLimit=false, negativeLimit=false, origin=false;
};
struct Status {
    unsigned long ticks=0;
    std::array<AxisStatus,4> axes;
    Pose position{{0,0,0,0}};
    bool estop=false, positionValid=false, pulsePose=false;
    bool running() const { for(const auto& a:axes)if(a.running||a.homing)return true;return false; }
};
// Customer entry point. No UI dependency; SDK calls are in RobotController.cpp.
// Use from one thread only. Never send a second command until the first finishes.
class RobotController {
public:
    ~RobotController();
    QString connectCard(const Settings& settings);
    QString disconnectCard();
    QString closeForExit();
    QString applyParameters(const Settings& settings);
    QString applyAxisParameter(int axis,int field,const AxisSettings& values);
    QString initialize();
    QString servo(int axis,bool enable);
    QString resetAlarm(int axis);
    QString home();
    QString move(const Point& point);
    QString runBuffer(const QVector<Point>& points);
    QString jogAxis(int axis,bool positive);
    QString jogCartesian(int axis,bool positive);
    QString setManualSpeeds(double axisSpeed,double axisAcc,double cartSpeed,double cartAcc);
    QString stop(bool emergency=false);
    QString readStatus(Status& result);
    QString readIo(int bit,bool& input,bool& output);
    QString writeOutput(int bit,bool on);
    void invalidate(); // settings changed / communication fault: require apply/init/home again
    bool connected() const { return m_connected; }
    bool configured() const { return m_configured; }
    bool initialized() const { return m_initialized; }
    bool busy() const { return m_active; }
    bool homed() const { return m_homed; }
    bool homing() const { return m_homing; }
private:
    friend struct ControllerTestAccess;
    QString guard(bool needHome,bool allowAlarm=false);
    QString sendPoint(const Point& point);
    Settings m_settings;
    bool m_connected=false, m_configured=false, m_initialized=false;
    bool m_homed=false, m_homing=false, m_active=false, m_fifo=false;
    bool m_nativeFault=false;
    std::array<bool,4> m_homeObserved{{false,false,false,false}};
    std::array<long,4> m_homeStartPulses{{0,0,0,0}};
    double m_axisDec=0.1, m_cartDec=100;
    QElapsedTimer m_commandTimer;
};
}
