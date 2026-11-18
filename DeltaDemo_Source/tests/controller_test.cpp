#include "RobotController.h"
#include "sldmv.h"
#include <QCoreApplication>
#include <QDebug>
#include <cstdio>
#include <limits>
#include <cmath>
namespace demo {
struct ControllerTestAccess {
    static void ready(RobotController& c){c.m_connected=true;c.m_configured=true;c.m_initialized=true;c.m_homed=true;c.m_active=false;c.m_homing=false;c.m_fifo=false;}
    static void unhome(RobotController& c){c.m_homed=false;}
    static void release(RobotController& c){c.m_connected=false;}
    static void inPlaceHome(RobotController& c){for(auto& a:c.m_settings.axes)a.homeMode=0;}
};
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);int failures=0,checks=0;
    auto expect=[&](bool ok,const char* name){++checks;if(!ok){++failures;qCritical()<<"FAIL"<<name;}};
    demo::RobotController robot;demo::Point p;
    auto prepare=[&]{fake::reset();demo::ControllerTestAccess::ready(robot);};
    prepare();robot.invalidate();fake::axes[1].servon=false;
    expect(robot.jogAxis(0,true).isEmpty(),"joint JOG needs only selected axis enable without setup or home");
    expect(fake::calls.contains("SLDM_JogP"),"uninitialized joint JOG reaches SDK");
    prepare();robot.invalidate();fake::axes[0].servon=false;
    expect(!robot.jogAxis(0,true).isEmpty()&&!fake::calls.contains("SLDM_JogP"),"disabled joint cannot jog");
    prepare();robot.invalidate();fake::estop=1;
    expect(!robot.jogAxis(0,true).isEmpty(),"uninitialized JOG still checks estop");
    prepare();expect(robot.applyParameters(demo::Settings{}).isEmpty(),"apply allowed with four axes enabled");
    expect(robot.initialize().isEmpty(),"initialize allowed with four axes enabled");
    expect(!fake::calls.contains("SLDM_Set_ServoOn"),"apply and initialize do not switch enables");
    prepare();fake::axes[2].servon=false;
    expect(!robot.home().isEmpty(),"home requires all four enabled");
    expect(!robot.move(p).isEmpty(),"point move requires all four enabled");
    expect(!robot.jogCartesian(0,true).isEmpty(),"space JOG requires all four enabled");
    expect(!robot.runBuffer({p}).isEmpty(),"buffer requires all four enabled");
    prepare();expect(robot.runBuffer({p,p}).isEmpty(),"buffer enables FIFO before querying space (disabled returns -36)");
    expect(fake::fifoEnabled&&fake::calls.count("SLDM_Delta_PTPn_DPoint")==2,"valid FIFO submits all points");
    prepare();fake::failure="SLDM_Get_MoveFifo_Space";
    expect(!robot.runBuffer({p}).isEmpty()&&!fake::fifoEnabled,"space read failure disables FIFO");
    expect(!fake::calls.contains("SLDM_Delta_PTPn_DPoint"),"space read failure submits no points");
    prepare();fake::spaces={0,0,95,96};
    expect(robot.runBuffer({p,p}).isEmpty(),"delayed FIFO readback recovers in one click");
    expect(fake::spaceReads==4&&fake::calls.count("SLDM_Delta_PTPn_DPoint")==2,"retry reads without duplicating moves");
    expect(fake::calls.count("SLDM_Set_MoveFifo_En")==2,"FIFO enabled once during retries");
    prepare();fake::spaces={95};QElapsedTimer capacityWait;capacityWait.start();
    const auto capacityError=robot.runBuffer({p,p});
    expect(capacityError.contains("95")&&capacityError.contains("96"),"capacity error reports parameter units and required budget");
    expect(capacityWait.elapsed()>=450&&capacityWait.elapsed()<2000,"capacity wait is bounded");
    expect(!fake::fifoEnabled&&!fake::calls.contains("SLDM_Delta_PTPn_DPoint")&&!robot.busy(),"insufficient capacity sends nothing and disables FIFO");
    prepare();fake::spaces={0,1024};fake::estopOnSpaceRead=2;
    expect(!robot.runBuffer({p}).isEmpty()&&!fake::calls.contains("SLDM_Delta_PTPn_DPoint"),"estop during wait cancels before enqueue");
    prepare();fake::spaces={0,1024};fake::failure="SLDM_Get_MoveFifo_Space";fake::failOccurrence=2;
    expect(!robot.runBuffer({p}).isEmpty()&&fake::spaceReads==2&&!fake::calls.contains("SLDM_Delta_PTPn_DPoint"),"SDK read error is not blindly retried");
    prepare();demo::ControllerTestAccess::unhome(robot);
    expect(!robot.move(p).isEmpty(),"motion requires session homing");
    expect(!fake::calls.contains("SLDM_Delta_PTPn_DPoint"),"no movement before home");
    prepare();fake::estop=1;expect(!robot.move(p).isEmpty(),"estop blocks motion");
    expect(!fake::calls.contains("SLDM_Delta_PTPn_DPoint"),"no command after estop");
    prepare();fake::failure="SLDM_Get_Axis_Status";
    const auto readError=robot.move(p);expect(readError.contains("SLDM_Get_Axis_Status")&&readError.contains("77"),"read failure retains API and code");
    expect(!fake::calls.contains("SLDM_Delta_PTPn_DPoint"),"read failure blocks command");
    prepare();fake::axes[0].running=true;expect(!robot.move(p).isEmpty(),"busy hardware blocks another command");
    prepare();demo::Point invalid=p;invalid.position[0]=std::numeric_limits<double>::quiet_NaN();
    expect(!robot.runBuffer({p,invalid}).isEmpty(),"all buffer targets prevalidated");
    expect(!fake::calls.contains("SLDM_Set_MoveFifo_En")&&!fake::calls.contains("SLDM_Delta_PTPn_DPoint"),"bad last point causes no FIFO writes");
    for(bool positive:{false,true}){
        prepare();robot.applyParameters(demo::Settings{});demo::ControllerTestAccess::ready(robot);fake::calls.clear();fake::arguments.clear();
        expect(robot.jogCartesian(3,positive).isEmpty(),"R JOG accepted");
        const auto calls=fake::arguments.value("SLDM_Delta_PTPn_DPoint");
        expect(calls.size()==1&&calls[0][1]==4&&calls[0][2]==0&&calls[0][3]==0&&calls[0][4]==-173&&calls[0][5]==(positive?999:-999),"R Delta target preserves XYZ");
        expect(calls.size()==1&&calls[0][6]==0&&calls[0][7]==20&&calls[0][8]==0&&calls[0][9]==100&&calls[0][10]==100&&calls[0][11]==4,"R Delta speed and curve match C#");
        expect(!fake::calls.contains("SLDM_JogP")&&!fake::calls.contains("SLDM_JogM"),"R JOG avoids joint commands");
        expect(robot.homed(),"R JOG retains spatial home");robot.stop();
    }
    prepare();fake::failure="SLDM_Delta_PTPn_DPoint";fake::failOccurrence=2;
    expect(!robot.runBuffer({p,p}).isEmpty(),"partial FIFO write failure reported");
    expect(fake::calls.contains("SLDM_PTPn_Stop_Plus")&&fake::calls.count("SLDM_Stop")==4,"failed enqueue stops all motion paths");
    prepare();fake::failure="SLDM_PTPn_Stop_Plus";
    expect(!robot.stop().isEmpty(),"stop failure reported");
    expect(fake::calls.contains("SLDM_Set_MoveFifo_En")&&fake::calls.contains("SLDM_PTPn_Stop")&&fake::calls.count("SLDM_Stop")==4,"remaining stops attempted after first failure");
    expect(!robot.configured(),"stop failure invalidates setup");
    prepare();expect(robot.jogAxis(0,true).isEmpty(),"joint jog before rehome");expect(!robot.homed(),"joint jog invalidates Cartesian home");
    expect(robot.stop().isEmpty(),"jog release stop");expect(fake::calls.count("SLDM_Stop")==4,"jog stop covers all axes");
    prepare();fake::axes[0].Plimit_valid=true;expect(!robot.jogAxis(0,true).isEmpty(),"jog toward active limit rejected");
    expect(robot.jogAxis(0,false).isEmpty(),"jog away from limit allowed");
    prepare();fake::axes[1].servo_alarm_valid=true;expect(!robot.home().isEmpty(),"servo alarm blocks homing");
    prepare();fake::axes[1].alarm_status=0x10;
    expect(!robot.home().isEmpty(),"latched servo alarm blocks homing even after input clears");
    expect(!robot.jogAxis(1,true).isEmpty(),"latched servo alarm blocks joint JOG");
    expect(!fake::calls.contains("SLDM_Delta_Home_Move")&&!fake::calls.contains("SLDM_JogP"),"latched alarm sends no motion command");
    prepare();fake::failure="SLDM_Set_Org_Config";expect(!robot.applyParameters(demo::Settings{}).isEmpty(),"partial setup failure reported");
    expect(!robot.configured()&&!robot.initialized(),"partial setup cannot initialize");
    expect(!fake::calls.contains("SLDM_Delta_Init"),"apply never initializes implicitly");
    prepare();expect(robot.stop(true).isEmpty(),"software estop succeeds");
    expect(fake::calls.count("SLDM_Estop")==4,"software estop invokes immediate stop on all axes");
    expect(!robot.configured()&&!robot.homed(),"software estop requires new initialization and home");
    prepare();fake::failure="SLDM_Estop";expect(!robot.stop(true).isEmpty(),"immediate stop failure reported");
    expect(fake::calls.count("SLDM_Estop")==4&&fake::calls.count("SLDM_Stop")==4&&fake::calls.contains("SLDM_PTPn_Stop_Plus"),"estop error does not skip remaining stop interfaces");
    prepare();expect(robot.setManualSpeeds(2,0.2,30,200).isEmpty(),"manual speed updates accepted at rest");
    expect(robot.homed()&&robot.initialized(),"speed-only edits preserve home and initialization");
    expect(robot.jogCartesian(2,true).isEmpty(),"finite Cartesian jog succeeds");
    expect(fake::lastPosition[2]==827.0f&&fake::lastPosition[0]==0,"jog adds fixed distant target to current position");
    expect(!robot.setManualSpeeds(3,1,30,200).isEmpty(),"speed updates blocked during motion");
    for(int axis=0;axis<3;++axis)for(bool positive:{false,true}) {
        prepare();
        expect(robot.jogCartesian(axis,positive).isEmpty(),"distant Cartesian jog accepted");
        for(int i=0;i<4;++i)expect(fake::lastPosition[i]==(i==2?-173.0f:0.0f)+(i==axis?(positive?1000.0f:-1000.0f):0.0f),"only selected coordinate offset");
        expect(fake::calls.count("SLDM_Delta_PTPn_DPoint")==1,"one target per press");
        expect(robot.stop().isEmpty(),"release stops distant target");
        expect(fake::calls.contains("SLDM_PTPn_Stop_Plus")&&fake::calls.count("SLDM_Stop")==4,"release stops Cartesian and joint motion");
    }
    for(bool positive:{false,true}){
        prepare();robot.applyParameters(demo::Settings{});demo::ControllerTestAccess::ready(robot);fake::calls.clear();fake::arguments.clear();
        expect(robot.jogCartesian(3,positive).isEmpty(),"R JOG accepted");
        const auto calls=fake::arguments.value("SLDM_Delta_PTPn_DPoint");
        expect(calls.size()==1&&calls[0][1]==4&&calls[0][2]==0&&calls[0][3]==0&&calls[0][4]==-173&&calls[0][5]==(positive?999:-999),"R Delta target preserves XYZ");
        expect(calls.size()==1&&calls[0][6]==0&&calls[0][7]==20&&calls[0][8]==0&&calls[0][9]==100&&calls[0][10]==100&&calls[0][11]==4,"R Delta speed and curve match C#");
        expect(!fake::calls.contains("SLDM_JogP")&&!fake::calls.contains("SLDM_JogM"),"R JOG avoids joint commands");
        expect(robot.homed(),"R JOG retains spatial home");robot.stop();
    }
    prepare();fake::failure="SLDM_Delta_PTPn_DPoint";
    expect(!robot.jogCartesian(0,true).isEmpty()&&!robot.busy(),"rejected distant target reports error without retry");
    prepare();p.position[0]=999;expect(robot.move(p).isEmpty(),"retired spatial limits no longer reject finite targets");
    prepare();robot.invalidate();
    for(int axis=0;axis<4;++axis)expect(robot.servo(axis,true).isEmpty(),"enable allowed before Delta setup");
    expect(fake::calls.count("SLDM_Set_ServoOn")==4,"four enable commands sent");
    expect(!robot.move(p).isEmpty(),"enable alone does not authorize motion");
    prepare();robot.invalidate();fake::estop=1;
    expect(!robot.servo(0,true).isEmpty(),"estop blocks startup enable");
    expect(!fake::calls.contains("SLDM_Set_ServoOn"),"estop sends no enable command");
    prepare();expect(robot.disconnectCard().isEmpty(),"disconnect succeeds at rest");
    expect(!fake::calls.contains("SLDM_Set_ServoOn"),"disconnect preserves servo outputs");
    expect(fake::calls.contains("SLDM_Close"),"disconnect closes SDK session");
    prepare();expect(robot.home().isEmpty(),"home command accepted");
    expect(fake::calls.indexOf("SLDM_Set_MoveFifo_En")<fake::calls.indexOf("SLDM_Delta_Home_Move")&&!fake::fifoEnabled,"home disables FIFO before command");
    QThread::msleep(210);demo::Status homeStatus;
    expect(robot.readStatus(homeStatus).isEmpty()&&!robot.homed()&&robot.homing(),"stale homed flags cannot complete new home");
    QThread::msleep(1810);
    expect(robot.readStatus(homeStatus).contains("未确认本次回零启动")&&!robot.homed(),"unobserved home reports diagnostic instead of success");
    robot.stop();
    prepare();expect(robot.home().isEmpty(),"new home accepted after stop");
    for(int i=0;i<3;++i){fake::axes[i].homing=true;fake::axes[i].homed=false;}
    expect(robot.readStatus(homeStatus).isEmpty()&&!robot.homed(),"observe this home while moving");
    for(int i=0;i<3;++i){fake::axes[i].homing=false;fake::axes[i].homed=true;}
    QThread::msleep(210);
    expect(robot.readStatus(homeStatus).isEmpty()&&robot.homed()&&!robot.busy(),"observed three-axis home plus in-place R completes");
    prepare();fake::failure="SLDM_Set_MoveFifo_En";
    expect(!robot.home().isEmpty()&&!fake::calls.contains("SLDM_Delta_Home_Move"),"FIFO disable failure prevents home command");
    prepare();demo::ControllerTestAccess::inPlaceHome(robot);expect(robot.home().isEmpty(),"intentional all-in-place home accepted");QThread::msleep(210);
    expect(robot.readStatus(homeStatus).isEmpty()&&robot.homed(),"in-place modes need no physical movement");
    prepare();demo::ControllerTestAccess::release(robot);
    std::printf("%d checks, %d failures\n",checks,failures);return failures?1:0;
}
