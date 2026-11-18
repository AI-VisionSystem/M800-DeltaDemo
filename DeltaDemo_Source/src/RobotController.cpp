#include "RobotController.h"
#include "DeltaFeedback.h"
#include <QNetworkInterface>
#include <QHostAddress>
#include <QThread>
#include <QStringList>
#include <cmath>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include "sldmv.h"
#else
#include "sldml.h"
#endif

namespace demo {
namespace {
constexpr unsigned long card=0;
QString sdkError(long rc,const char* api) {
    return rc==0 ? QString{} : QStringLiteral("%1 返回码 %2").arg(QString::fromLatin1(api)).arg(rc);
}
// Keep SEH in a POD-only function; a faulty Windows SDK session cannot be retried.
long openE450(int host,int address,bool* fault) {
#ifdef _WIN32
#ifdef _MSC_VER
    __try {
#endif
        unsigned long types[1]={450};
        long rc=SLDM_Set_Card_List(static_cast<unsigned long>(host),1,types);
        return rc ? rc : SLDM_Open_IP(192,168,100,static_cast<unsigned long>(host),static_cast<unsigned long>(address),0);
#ifdef _MSC_VER
    } __except((GetExceptionCode()==EXCEPTION_ACCESS_VIOLATION || GetExceptionCode()==EXCEPTION_IN_PAGE_ERROR)
                 ? EXCEPTION_EXECUTE_HANDLER : EXCEPTION_CONTINUE_SEARCH) {
        *fault=true; return -10001;
    }
#endif
#else
    Q_UNUSED(host) Q_UNUSED(address) Q_UNUSED(fault)
    return -10002;
#endif
}
}
// CHECK preserves the failing function name; a multi-call operation stops at its first failure.
#define CHECK(call) do { const auto sdkCallError=sdkError((call),#call); if(!sdkCallError.isEmpty())return sdkCallError; } while(false)
RobotController::~RobotController() { disconnectCard(); }
void RobotController::invalidate() { m_configured=false;m_initialized=false;m_homed=false;m_homing=false; }
QString RobotController::connectCard(const Settings& s) {
    if(m_connected)return {};
    if(m_nativeFault)return QStringLiteral("SDK发生原生异常，请关闭其他控制软件并重启Demo后检查连接");
    const auto error=s.validate(); if(!error.isEmpty())return error;
#ifdef _WIN32
    CHECK(SLDM_Set_Net_Timeout(100,3));
    bool found=false;
    for(const auto& iface:QNetworkInterface::allInterfaces()) {
        if(!(iface.flags()&QNetworkInterface::IsUp)||!(iface.flags()&QNetworkInterface::IsRunning)
           ||(iface.flags()&QNetworkInterface::IsLoopBack))continue;
        for(const auto& entry:iface.addressEntries())if(entry.ip()==QHostAddress(s.hostIp))found=true;
    }
    if(!found)return QStringLiteral("未找到活动网卡地址 %1；请设置网卡IP并连接网线").arg(s.hostIp);
    const long rc=openE450(s.hostIp.section('.',3).toInt(),s.cardAddress,&m_nativeFault);
    if(m_nativeFault)return QStringLiteral("E450 SDK原生异常，连接已中止；请重启Demo后检查驱动和网卡");
    const auto openError=sdkError(rc,"SLDM_Set_Card_List / SLDM_Open_IP");
    if(!openError.isEmpty())return openError;
#else
    // M800 onboard path from the existing M800 project. Verify against the
    // matching ARM64 SDK before shipping; never substitute an x86 Linux library.
    unsigned long count=8,types[8]={};
    CHECK(SLDM_Get_Card_List(101,&count,types));
    if(count!=1)return QStringLiteral("M800 Demo要求恰好一个板载控制器，检测到%1个").arg(count);
    CHECK(SLDM_Set_Card_List(101,count,types));
    CHECK(SLDM_Open(101,0));
#endif
    m_connected=true; m_settings=s; invalidate();
    // A previous application may have left a queue running. Connect never starts motion.
    const auto stopped=stop(true);
    if(!stopped.isEmpty())return stopped;
    return {};
}
QString RobotController::disconnectCard() {
    if(!m_connected)return {};
    QString error=stop();
    // Wait at most 1s for actual standstill. On failure retain the connection so
    // the operator can retry STOP; never silently close a moving controller.
    QElapsedTimer wait;wait.start();bool stationary=false;
    while(wait.elapsed()<1000) {
        Status s; const auto e=readStatus(s);
        if(!e.isEmpty()){if(error.isEmpty())error=e;break;}
        if(!s.running()){stationary=true;break;}
        QThread::msleep(10);
    }
    if(!stationary)return error.isEmpty()?QStringLiteral("停机未确认，连接保持；请检查设备并重试停止"):error;
    const auto closed=sdkError(SLDM_Close(card),"SLDM_Close");
    if(!closed.isEmpty())return closed;
    m_connected=false;invalidate();m_active=false;m_fifo=false;
    return error;
}
QString RobotController::readStatus(Status& result) {
    if(!m_connected)return QStringLiteral("未连接控制器");
    Status next;
    CHECK(SLDM_Get_SysTicks(card,&next.ticks));
    unsigned long input=0;
    CHECK(SLDM_Get_InBit(card,0,static_cast<unsigned long>(m_settings.estopInput),&input));
    next.estop=(input!=0)==(m_settings.estopLevel!=0);
    for(unsigned long i=0;i<4;++i) {
        Axis_Status state{}; auto& a=next.axes[i];
        CHECK(SLDM_Get_Axis_Status(card,i,&state));
        a.enabled=state.servon;a.homed=state.homed;a.running=state.running;a.homing=state.homing;
        a.alarm=state.servo_alarm_valid||state.alarm_status!=0;
        a.servoAlarm=state.servo_alarm_valid||(state.alarm_status&0x10)!=0;
        a.positiveLimit=state.Plimit_valid||state.soft_Plimit_valid;
        a.negativeLimit=state.Nlimit_valid||state.soft_Nlimit_valid;a.origin=state.origin_valid;
        CHECK(SLDM_Get_Pos(card,i,&a.pulses));CHECK(SLDM_Get_Enc(card,i,&a.encoder));
        float value=0;
        CHECK(SLDM_Get_Vel(card,i,&value));a.speed=value;
#if defined(_WIN32) && !defined(DEMO_PULSE_FEEDBACK_TEST)
        // Before Delta_Init there is no meaningful Cartesian coordinate system.
        if(m_initialized) {
            CHECK(SLDM_Get_Delta_Angle(card,i,&value));a.angle=value;
            CHECK(SLDM_Get_Delta_XYZR_Pos(card,i,&value));next.position[i]=value;
            if(!std::isfinite(a.angle)||!std::isfinite(next.position[i]))return QStringLiteral("控制卡返回无效坐标");
            next.positionValid=true;
        }
#endif
    }
    bool homeObserved=true;
    if(m_homing) {
        QStringList missing;
        for(int i=0;i<4;++i) {
            const auto& a=next.axes[i];
            m_homeObserved[i]=m_homeObserved[i]||a.homing||a.running||!a.homed||a.pulses!=m_homeStartPulses[i];
            if(m_settings.axes[i].homeMode!=0&&!m_homeObserved[i])missing.append(QString::number(i));
        }
        homeObserved=missing.isEmpty();
        if(!homeObserved&&!next.running()&&m_commandTimer.elapsed()>=2000)
            return QStringLiteral("未确认本次回零启动：轴%1未观察到回零状态或脉冲变化，不能使用旧的已回零标志；请检查回零模式和控制卡执行状态") .arg(missing.join(','));
    }
    if(m_active && m_commandTimer.elapsed()>=200 && !next.running() && (!m_homing||homeObserved)) {
        unsigned long remaining=0,running=0;
        if(m_fifo) {
            CHECK(SLDM_Get_MoveFifo_FuncNum(card,&remaining));
            CHECK(SLDM_Get_MoveFifo_Running(card,&running));
        }
        if(remaining==0 && running==0) {
            if(m_fifo){CHECK(SLDM_Set_MoveFifo_En(card,0));m_fifo=false;}
            m_active=false;
            if(m_homing) {
                m_homed=true;
                for(const auto& a:next.axes)if(!a.homed||a.alarm)m_homed=false;
                m_homing=false;
            }
        }
    }
#if !defined(_WIN32) || defined(DEMO_PULSE_FEEDBACK_TEST)
    // M800's Delta cache can stay stale. Use live command pulses after homing.
    bool axesHomed=true;for(const auto& a:next.axes)axesHomed=axesHomed&&a.homed;
    if(m_initialized&&!m_homing&&axesHomed) {
        std::array<long,4> pulses;std::array<double,4> angles;
        for(int i=0;i<4;++i)pulses[i]=next.axes[i].pulses;
        next.positionValid=deltaPulsePose(m_settings,pulses,angles,next.position);
        if(next.positionValid){for(int i=0;i<4;++i)next.axes[i].angle=angles[i];next.pulsePose=true;}
    }
#endif
    result=next;return {};
}
QString RobotController::closeForExit() {
    const auto error=disconnectCard();
    // Process exit releases a failed SDK session; never retry in the destructor.
    m_connected=false;invalidate();m_active=false;m_fifo=false;
    return error;
}
QString RobotController::guard(bool needHome,bool allowAlarm) {
    if(!m_connected)return QStringLiteral("请先连接控制器");
    if(!m_configured||!m_initialized)return QStringLiteral("请先下发参数并初始化机械手");
    Status s;const auto error=readStatus(s);if(!error.isEmpty())return error;
    if(s.estop)return QStringLiteral("急停输入有效，禁止运动");
    if(m_active||s.running())return QStringLiteral("已有运动正在执行，请先停止");
    for(const auto& a:s.axes)if(!a.enabled)return QStringLiteral("请先开启四轴使能");
    for(const auto& a:s.axes)if(a.servoAlarm)return QStringLiteral("伺服报警有效，禁止运动");
    for(const auto& a:s.axes)if(a.alarm&&!allowAlarm)return QStringLiteral("存在轴报警/限位，请检查状态并清除报警");
    if(needHome) {
        if(!m_homed)return QStringLiteral("请先完成本次初始化后的回零");
        for(const auto& a:s.axes)if(!a.homed)return QStringLiteral("控制卡回零状态失效，请重新回零");
    }
    return {};
}
QString RobotController::applyParameters(const Settings& s) {
    if(!m_connected)return QStringLiteral("未连接控制器");
    auto error=s.validate();if(!error.isEmpty())return error;
    Status state;error=readStatus(state);if(!error.isEmpty())return error;
    if(m_active||state.running())return QStringLiteral("运动中不能下发参数");
    unsigned long estop=0;
    CHECK(SLDM_Get_InBit(card,0,static_cast<unsigned long>(s.estopInput),&estop));
    if((estop!=0)==(s.estopLevel!=0))return QStringLiteral("所配置的急停输入有效，请先检查接线及有效电平");
    invalidate();
    for(unsigned long i=0;i<4;++i) {
        const auto& a=s.axes[i];
        CHECK(SLDM_Set_Pulse_Mode(card,i,static_cast<unsigned long>(a.pulseMode)));
        CHECK(SLDM_Set_Pulse_Pol(card,i,static_cast<unsigned long>(a.pulsePol)));
        CHECK(SLDM_Set_Dir_Pol(card,i,static_cast<unsigned long>(a.dirPol)));
        CHECK(SLDM_Set_ServoOn_Pol(card,i,static_cast<unsigned long>(a.servoOnPol)));
        CHECK(SLDM_Set_ServoAlm_Config(card,i,static_cast<unsigned long>(a.alarmEnable),static_cast<unsigned long>(a.alarmPol)));
        CHECK(SLDM_Set_RstAlm_Pol(card,i,static_cast<unsigned long>(a.resetPol)));
        CHECK(SLDM_Set_Plimit_Config(card,i,static_cast<unsigned long>(a.positiveEnable),static_cast<unsigned long>(a.positivePol)));
        CHECK(SLDM_Set_Nlimit_Config(card,i,static_cast<unsigned long>(a.negativeEnable),static_cast<unsigned long>(a.negativePol)));
        CHECK(SLDM_Set_Org_Config(card,i,static_cast<unsigned long>(a.originEnable),static_cast<unsigned long>(a.originPol)));
        CHECK(SLDM_Set_Soft_Limit(card,i,0,0,0)); // Retired option: clear any controller-retained soft limit.
        CHECK(SLDM_Set_Enc_Enable(card,i,static_cast<unsigned long>(a.encoderEnable)));
        CHECK(SLDM_Set_Enc_Pol(card,i,static_cast<unsigned long>(a.encoderPol)));
        CHECK(SLDM_Set_Enc_Mult(card,i,float(a.encoderMult)));
        CHECK(SLDM_Set_EncZ_Pol(card,i,static_cast<unsigned long>(a.encoderZPol)));
        CHECK(SLDM_Set_Enc_Filter(card,i,float(a.encoderFilter)));
        CHECK(SLDM_Set_Delta_Step_Angle(card,i,float(a.angle),long(a.pulses),static_cast<unsigned long>(a.deltaPol)));
        CHECK(SLDM_Set_Home_Oft(card,i,long(a.homeOffset)));
        // Homing encoder selection is independent of position-feedback enable.
        CHECK(SLDM_Set_Home_Para(card,i,static_cast<unsigned long>(a.homeDirection),float(a.homeLow),
              float(a.homeHigh),float(a.homeAcc),static_cast<unsigned long>(a.homeMode),static_cast<unsigned long>(a.homeEncoder)));
    }
    CHECK(SLDM_Set_Delta_Para(card,float(s.geometry[0]),float(s.geometry[1]),float(s.geometry[2]),float(s.geometry[3]),0));
    m_settings=s;m_axisDec=s.axisAcc;m_cartDec=s.cartAcc;m_configured=true;return {};
}
QString RobotController::applyAxisParameter(int axis,int field,const AxisSettings& values) {
    if(!m_connected)return QStringLiteral("未连接控制器，参数尚未写入");
    if(axis<0||axis>3||field<0||field>=axisFields().size())return QStringLiteral("轴或参数无效");
    Status state;auto error=readStatus(state);if(!error.isEmpty())return error;
    if(m_active||state.running())return QStringLiteral("请先停止运动再修改参数");
    const QString key=axisFields()[field].key;
    Settings next=m_settings;auto& a=next.axes[axis];
    a.*axisFields()[field].member=values.*axisFields()[field].member;
    error=next.validate();if(!error.isEmpty())return error;
    // Grouped SDK setters use other values already accepted by the controller.
    long rc=0;const char* api="";
#define SET_AXIS(name, ...) do {api=#name;rc=name(card,axis,__VA_ARGS__);} while(false)
    if(key=="pulseMode")SET_AXIS(SLDM_Set_Pulse_Mode,(unsigned long)a.pulseMode);
    else if(key=="pulsePol")SET_AXIS(SLDM_Set_Pulse_Pol,(unsigned long)a.pulsePol);
    else if(key=="dirPol")SET_AXIS(SLDM_Set_Dir_Pol,(unsigned long)a.dirPol);
    else if(key=="servoOnPol")SET_AXIS(SLDM_Set_ServoOn_Pol,(unsigned long)a.servoOnPol);
    else if(key=="alarmEnable"||key=="alarmPol")SET_AXIS(SLDM_Set_ServoAlm_Config,(unsigned long)a.alarmEnable,(unsigned long)a.alarmPol);
    else if(key=="resetPol")SET_AXIS(SLDM_Set_RstAlm_Pol,(unsigned long)a.resetPol);
    else if(key=="positiveEnable"||key=="positivePol")SET_AXIS(SLDM_Set_Plimit_Config,(unsigned long)a.positiveEnable,(unsigned long)a.positivePol);
    else if(key=="negativeEnable"||key=="negativePol")SET_AXIS(SLDM_Set_Nlimit_Config,(unsigned long)a.negativeEnable,(unsigned long)a.negativePol);
    else if(key=="originEnable"||key=="originPol")SET_AXIS(SLDM_Set_Org_Config,(unsigned long)a.originEnable,(unsigned long)a.originPol);
    else if(key=="encoderEnable")SET_AXIS(SLDM_Set_Enc_Enable,(unsigned long)a.encoderEnable);
    else if(key=="encoderPol")SET_AXIS(SLDM_Set_Enc_Pol,(unsigned long)a.encoderPol);
    else if(key=="encoderMult")SET_AXIS(SLDM_Set_Enc_Mult,float(a.encoderMult));
    else if(key=="encoderZPol")SET_AXIS(SLDM_Set_EncZ_Pol,(unsigned long)a.encoderZPol);
    else if(key=="encoderFilter")SET_AXIS(SLDM_Set_Enc_Filter,float(a.encoderFilter));
    else if(key=="deltaPol"||key=="angle"||key=="pulses")SET_AXIS(SLDM_Set_Delta_Step_Angle,float(a.angle),long(a.pulses),(unsigned long)a.deltaPol);
    else if(key=="homeOffset")SET_AXIS(SLDM_Set_Home_Oft,long(a.homeOffset));
    else if(key.startsWith("home"))SET_AXIS(SLDM_Set_Home_Para,(unsigned long)a.homeDirection,float(a.homeLow),float(a.homeHigh),float(a.homeAcc),(unsigned long)a.homeMode,(unsigned long)a.homeEncoder);
    else return QStringLiteral("未知轴参数");
#undef SET_AXIS
    error=sdkError(rc,api);
    if(!error.isEmpty()){invalidate();return error;}
    m_settings.axes[axis]=a;m_homed=false;
    if(key=="deltaPol"||key=="angle"||key=="pulses")m_initialized=false;
    return {};
}
QString RobotController::initialize() {
    if(!m_connected||!m_configured)return QStringLiteral("请先下发参数");
    Status s;auto error=readStatus(s);if(!error.isEmpty())return error;
    if(m_active||s.running()||s.estop)return QStringLiteral("运动中或急停有效，不能初始化");
    m_initialized=false;m_homed=false;
    CHECK(SLDM_Delta_Init(card));m_initialized=true;return {};
}
QString RobotController::servo(int axis,bool enable) {
    if(!m_connected||axis<0||axis>3)return QStringLiteral("未连接或轴号无效");
    if(enable){
        // Servo power does not require Delta initialization; motion still does.
        Status s;const auto e=readStatus(s);if(!e.isEmpty())return e;
        if(s.estop)return QStringLiteral("急停输入有效，不能开启使能");
        if(m_active||s.running())return QStringLiteral("请先停止运动再开启使能");
        for(const auto& a:s.axes)if(a.servoAlarm)return QStringLiteral("伺服报警有效，不能开启使能");
    }
    else {const auto e=stop();if(!e.isEmpty())return e;m_homed=false;}
    CHECK(SLDM_Set_ServoOn(card,static_cast<unsigned long>(axis),enable?1:0));return {};
}
QString RobotController::resetAlarm(int axis) {
    if(!m_connected||axis<0||axis>3)return QStringLiteral("未连接或轴号无效");
    Status s;auto e=readStatus(s);if(!e.isEmpty())return e;
    if(m_active||s.running())return QStringLiteral("请先停止运动再清报警");
    CHECK(SLDM_Reset(card,static_cast<unsigned long>(axis)));return {};
}
QString RobotController::home() {
    const auto e=guard(false,true);if(!e.isEmpty())return e;
    Status before;const auto readError=readStatus(before);if(!readError.isEmpty())return readError;
    m_homed=false;
    CHECK(SLDM_Set_MoveFifo_En(card,0));m_fifo=false;
    for(int i=0;i<4;++i){m_homeStartPulses[i]=before.axes[i].pulses;m_homeObserved[i]=false;}
    CHECK(SLDM_Delta_Home_Move(card));m_homing=true;m_active=true;m_commandTimer.start();return {};
}
QString RobotController::sendPoint(const Point& p) {
    unsigned long axes[4]={0,1,2,3};float pos[4];for(int i=0;i<4;++i)pos[i]=float(p.position[i]);
    CHECK(SLDM_Delta_PTPn_DPoint(card,4,axes,pos,float(p.start),float(p.speed),float(p.end),
                               float(p.acc),float(p.dec),static_cast<unsigned long>(p.curve)));
    return {};
}
QString RobotController::move(const Point& p) {
    auto e=guard(true);if(!e.isEmpty())return e;
    e=validatePoint(p,m_settings);if(!e.isEmpty())return e;
    CHECK(SLDM_Set_MoveFifo_En(card,0));
    e=sendPoint(p);if(!e.isEmpty())return e;
    m_cartDec=p.dec;m_active=true;m_commandTimer.start();return {};
}
QString RobotController::runBuffer(const QVector<Point>& points) {
    auto e=guard(true);if(!e.isEmpty())return e;
    if(points.isEmpty()||points.size()>100)return QStringLiteral("缓冲点数须为1~100");
    // Validate the WHOLE list before enabling FIFO: this SDK executes while enqueueing.
    for(const auto& p:points){e=validatePoint(p,m_settings);if(!e.isEmpty())return e;}
    CHECK(SLDM_Set_MoveFifo_En(card,0));
    CHECK(SLDM_Set_MoveFifo_En(card,1));m_fifo=true;
    // Space counts parameter words, not points. The bundled M800 four-axis
    // packet has 2*n+9 words including checksum; budget 32/point plus 32 spare.
    // Readback can lag FIFO reset/enable. Retry reads only, never motion writes.
    const unsigned long required=32UL*(static_cast<unsigned long>(points.size())+1);
    unsigned long space=0;QElapsedTimer ready;ready.start();
    do {
        e=sdkError(SLDM_Get_MoveFifo_Space(card,&space),"SLDM_Get_MoveFifo_Space");
        if(!e.isEmpty()||space>=required||ready.elapsed()>=500)break;
        QThread::msleep(20);
    } while(ready.elapsed()<500);
    if(e.isEmpty()&&space<required)e=QStringLiteral("缓冲空间未就绪或不足：剩余%1，需%2参数单位（%3点，含余量）；等待%4ms，未下发运动")
        .arg(space).arg(required).arg(points.size()).arg(ready.elapsed());
    // A status change while waiting must cancel before the first point is sent.
    if(e.isEmpty())e=guard(true);
    if(!e.isEmpty()){const auto stopped=stop();return e+(stopped.isEmpty()?QString{}:"\n停止失败："+stopped);}
    for(const auto& p:points) {
        e=sendPoint(p);
        if(!e.isEmpty()){const auto stopped=stop(true);return e+(stopped.isEmpty()?QString{}:"\n停止失败："+stopped);}
    }
    m_active=true;m_commandTimer.start();return {};
}
QString RobotController::jogAxis(int axis,bool positive) {
    if(axis<0||axis>3)return QStringLiteral("轴号无效");
    if(!m_connected)return QStringLiteral("请先连接控制器");
    Status s;auto error=readStatus(s);if(!error.isEmpty())return error;
    if(s.estop)return QStringLiteral("急停输入有效，禁止运动");
    if(m_active||s.running())return QStringLiteral("已有运动正在执行，请先停止");
    const auto& a=s.axes[axis];
    if(!a.enabled)return QStringLiteral("请先开启该轴使能");
    if(a.servoAlarm)return QStringLiteral("伺服报警有效");
    if((positive&&a.positiveLimit)||(!positive&&a.negativeLimit))return QStringLiteral("该方向限位有效");
    // Permit leaving a limit, but never jog through a servo alarm.
    CHECK(SLDM_Set_MoveFifo_En(card,0));
    const long rc=positive?SLDM_JogP(card,axis,0,float(m_settings.axisSpeed),float(m_settings.axisAcc))
                          :SLDM_JogM(card,axis,0,float(m_settings.axisSpeed),float(m_settings.axisAcc));
    const auto jogError=sdkError(rc,positive?"SLDM_JogP":"SLDM_JogM");if(!jogError.isEmpty())return jogError;
    m_homed=false;m_active=true;m_commandTimer.start();return {};
}
QString RobotController::jogCartesian(int axis,bool positive) {
    if(axis<0||axis>3)return QStringLiteral("坐标轴无效");
    const auto e=guard(true);if(!e.isEmpty())return e;
    Status s;auto error=readStatus(s);if(!error.isEmpty())return error;
    if(!s.positionValid)return QStringLiteral("当前空间坐标不可用，请完成回零并检查机械参数");
    Point p;p.position=s.position;p.position[axis]+=(positive?1:-1)*(axis==3?999.0:1000.0);
    p.speed=m_settings.cartSpeed;p.acc=p.dec=m_settings.cartAcc;
    if(axis==3)p.curve=4; // Match the C# Delta R JOG command.
    return move(p); // One distant target (XYZ mm / R degrees); release stops, no automatic repeat.
}
QString RobotController::setManualSpeeds(double axisSpeed,double axisAcc,double cartSpeed,double cartAcc) {
    if(m_active)return QStringLiteral("请停止运动后调整手动速度");
    Settings next=m_settings;next.axisSpeed=axisSpeed;next.axisAcc=axisAcc;
    next.cartSpeed=cartSpeed;next.cartAcc=cartAcc;
    const auto e=next.validate();if(!e.isEmpty())return e;
    if(m_connected){Status s;const auto error=readStatus(s);if(!error.isEmpty())return error;if(s.running())return QStringLiteral("轴尚未停稳");}
    m_settings=next;m_axisDec=axisAcc;m_cartDec=cartAcc;return {};
}
QString RobotController::stop(bool emergency) {
    if(!m_connected)return {};
    QString error;
    auto record=[&](long rc,const char* name){const auto e=sdkError(rc,name);if(!e.isEmpty()){if(!error.isEmpty())error+='\n';error+=e;}};
    // Try EVERY stop even if one fails; FIFO, Cartesian interpolation and single-axis JOG are separate.
    record(SLDM_Set_MoveFifo_En(card,0),"SLDM_Set_MoveFifo_En(0)");
    if(emergency)for(unsigned long a=0;a<4;++a)record(SLDM_Estop(card,a),"SLDM_Estop");
    record(SLDM_PTPn_Stop_Plus(card,emergency?0:float(m_cartDec)),"SLDM_PTPn_Stop_Plus");
    record(SLDM_PTPn_Stop(card,emergency?0:float(m_axisDec)),"SLDM_PTPn_Stop");
    for(unsigned long a=0;a<4;++a)record(SLDM_Stop(card,a,emergency?0:float(m_axisDec)),"SLDM_Stop");
    if(emergency)invalidate();
    if(m_homing||emergency)m_homed=false;
    m_homing=false;
    if(error.isEmpty()){m_active=false;m_fifo=false;}else invalidate();
    return error;
}
QString RobotController::readIo(int bit,bool& input,bool& output) {
    if(!m_connected||bit<0||bit>31)return QStringLiteral("未连接或IO号无效（0~31）");
    unsigned long di=0,dout=0;
    CHECK(SLDM_Get_InBit(card,0,bit,&di));CHECK(SLDM_Get_OutBit(card,0,bit,&dout));
    input=di!=0;output=dout!=0;return {};
}
QString RobotController::writeOutput(int bit,bool on) {
    if(bit<0||bit>31)return QStringLiteral("IO号无效");
    const auto e=guard(false);if(!e.isEmpty())return e;
    CHECK(SLDM_Set_OutBit(card,0,bit,on?1:0));return {};
}
#undef CHECK
}
