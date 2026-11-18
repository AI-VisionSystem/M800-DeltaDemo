#include "RobotController.h"
namespace demo {
// No simulated hardware success: preview can edit/save settings but never connect/move.
static QString unavailable(){return QStringLiteral("当前为离线界面预览版，未编入控制卡SDK");}
RobotController::~RobotController()=default;
void RobotController::invalidate(){}
QString RobotController::connectCard(const Settings&){return unavailable();}
QString RobotController::disconnectCard(){return {};}
QString RobotController::closeForExit(){return {};}
QString RobotController::applyParameters(const Settings&){return unavailable();}
QString RobotController::applyAxisParameter(int,int,const AxisSettings&){return unavailable();}
QString RobotController::initialize(){return unavailable();}
QString RobotController::servo(int,bool){return unavailable();}
QString RobotController::resetAlarm(int){return unavailable();}
QString RobotController::home(){return unavailable();}
QString RobotController::move(const Point&){return unavailable();}
QString RobotController::runBuffer(const QVector<Point>&){return unavailable();}
QString RobotController::jogAxis(int,bool){return unavailable();}
QString RobotController::jogCartesian(int,bool){return unavailable();}
QString RobotController::setManualSpeeds(double,double,double,double){return unavailable();}
QString RobotController::stop(bool){return {};}
QString RobotController::readStatus(Status&){return unavailable();}
QString RobotController::readIo(int,bool&,bool&){return unavailable();}
QString RobotController::writeOutput(int,bool){return unavailable();}
}
