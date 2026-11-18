#pragma once
#include <QString>
#include <QVector>
#include <array>

namespace demo {
// All methods return an empty QString on success, otherwise a readable error.
// Axis values use doubles so the form/JSON share one definition; integer fields
// are checked before any SDK conversion. Keep names aligned with SDK parameters.
struct AxisSettings {
    double pulseMode=0, pulsePol=0, dirPol=1, servoOnPol=0;
    double alarmEnable=1, alarmPol=0, resetPol=0;
    double positiveEnable=1, positivePol=0, negativeEnable=1, negativePol=0;
    double originEnable=1, originPol=1;
    double encoderEnable=0, encoderPol=0, encoderMult=4, encoderZPol=0, encoderFilter=0;
    double angle=90, pulses=7500, deltaPol=0;
    double homeMode=3, homeDirection=1, homeLow=1, homeHigh=1, homeAcc=0.01, homeOffset=2000, homeEncoder=0;
};
struct AxisField {
    const char* key;
    const char* label;
    double AxisSettings::* member;
    double minimum, maximum;
    int decimals;
    const char* choices; // optional | separated labels; item index = SDK value
};
const QVector<AxisField>& axisFields();
using Pose = std::array<double,4>; // XYZ mm, R degrees (absolute, no angle wrapping)
struct Point {
    Pose position{{0,0,-173,0}};
    double start=0, speed=20, end=0, acc=100, dec=100;
    int curve=1;
};
struct Settings {
    QString hostIp="192.168.100.101";
    int cardAddress=1;
    std::array<AxisSettings,4> axes;
    std::array<double,4> geometry{{100,30,150,280}};
    double axisSpeed=1, axisAcc=0.1, cartSpeed=20, cartAcc=100;
    int estopInput=0, estopLevel=1;
    QVector<Point> points;
    Settings();
    QString validate() const;
    QString load(const QString& path); // atomic: failure leaves *this unchanged
    QString save(const QString& path) const;
};
QString validatePoint(const Point& point, const Settings& settings);
}
