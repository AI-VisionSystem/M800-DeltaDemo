#include "RobotController.h"
#include "DeltaFeedback.h"
#include "sldmv.h"
#include <QCoreApplication>
#include <QDebug>
namespace demo {
struct ControllerTestAccess {
    static void ready(RobotController& c){c.m_connected=c.m_configured=c.m_initialized=c.m_homed=true;}
    static void release(RobotController& c){c.m_connected=false;}
};
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);int failures=0,checks=0;
    auto expect=[&](bool ok,const char* name){++checks;if(!ok){++failures;qCritical()<<name;}};
    demo::Settings settings;demo::Pose pose;std::array<double,4> angles;
    std::array<long,4> pulses{{0,0,0,0}};
    expect(demo::deltaPulsePose(settings,pulses,angles,pose),"origin solves");
    expect(std::abs(pose[0])<1e-8&&std::abs(pose[1])<1e-8&&std::abs(pose[2]+std::sqrt(30000.0))<1e-8,"known default origin");
    pulses={100,250,-150,750};
    expect(demo::deltaPulsePose(settings,pulses,angles,pose),"asymmetric pose solves");
    for(int i=0;i<3;++i){const double az=i*2*3.141592653589793/3,theta=angles[i]*3.141592653589793/180;
        const double radius=70+150*std::cos(theta);
        const double dx=pose[0]-radius*std::cos(az),dy=pose[1]-radius*std::sin(az),dz=pose[2]+150*std::sin(theta);
        expect(std::abs(dx*dx+dy*dy+dz*dz-280*280)<1e-6,"independent rod length constraint");}
    expect(std::abs(pose[3]-3.6)<1e-8,"R pulse conversion");
    settings.axes[3].deltaPol=1;demo::Pose opposite;
    expect(demo::deltaPulsePose(settings,pulses,angles,opposite)&&opposite[3]==-pose[3]&&opposite[0]==pose[0],"R polarity only changes R");
    settings.geometry[3]=1;expect(!demo::deltaPulsePose(settings,pulses,angles,pose),"impossible geometry rejected");
    fake::reset();demo::RobotController robot;demo::ControllerTestAccess::ready(robot);demo::Status before,after;
    expect(robot.readStatus(before).isEmpty()&&before.positionValid&&before.pulsePose,"controller selects pulse feedback");
    fake::pulses[0]=100;fake::axes[0].running=true;
    expect(robot.readStatus(after).isEmpty()&&after.positionValid&&std::abs(after.position[0]-before.position[0])>0.01,"XYZ updates while moving despite frozen SDK cache");
    fake::axes[0].running=false;before=after;fake::pulses[3]=750;
    expect(robot.readStatus(after).isEmpty()&&after.position[0]==before.position[0]&&std::abs(after.position[3]-3.6)<1e-8,"R updates without changing XYZ");
    expect(!fake::calls.contains("SLDM_Get_Delta_XYZR_Pos")&&!fake::calls.contains("SLDM_Get_Delta_Angle"),"stale caches never read");
    expect(robot.jogCartesian(1,true).isEmpty()&&std::abs(fake::lastPosition[0]-after.position[0])<0.001&&std::abs(fake::lastPosition[1]-after.position[1]-1000)<0.001,"JOG starts from pulse feedback");
    robot.stop();
    for(bool positive:{false,true}) {
        expect(robot.jogCartesian(3,positive).isEmpty(),"pulse-based R JOG accepted");
        for(int i=0;i<4;++i)expect(std::abs(fake::lastPosition[i]-after.position[i]-(i==3?(positive?999:-999):0))<0.001,"R command preserves live XYZ and offsets live R");
        expect(robot.stop().isEmpty(),"R release stops Delta interpolation");
    }
    fake::axes[0].homed=false;
    expect(robot.readStatus(after).isEmpty()&&!after.positionValid,"unhomed coordinates unavailable");
    fake::axes[0].homed=true;fake::failure="SLDM_Get_Pos";
    expect(!robot.readStatus(after).isEmpty(),"pulse read error propagates");
    demo::ControllerTestAccess::release(robot);qInfo()<<checks<<"checks,"<<failures<<"failures";return failures?1:0;
}
