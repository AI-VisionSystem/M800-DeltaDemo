#pragma once
// Deliberately separate from the vendor header/DLL. Test-only SDK implementation.
#include <QStringList>
#include <QThread>
#include <cstring>
#include <QMap>
#include <QVector>
#include <type_traits>
struct Axis_Status {
    bool servon=true,homed=true,running=false,homing=false,servo_alarm_valid=false;
    bool Plimit_valid=false,Nlimit_valid=false,soft_Plimit_valid=false,soft_Nlimit_valid=false,origin_valid=false;
    unsigned char alarm_status=0;
};
namespace fake {
inline unsigned long closeDelay=0;
inline QStringList calls;
inline QMap<QString,QVector<QVector<double>>> arguments;
inline float lastPosition[4]={};
inline QString failure;
inline int failOccurrence=1,occurrence=0;
inline Axis_Status axes[4];
inline unsigned long estop=0;
inline bool fifoEnabled=false;
inline float currentZ=-173.0f;
inline long pulses[4]={};
inline QVector<unsigned long> spaces;
inline int spaceReads=0,estopOnSpaceRead=0;
inline void reset(){spaces.clear();spaceReads=0;estopOnSpaceRead=0;closeDelay=0;fifoEnabled=false;calls.clear();arguments.clear();currentZ=-173.0f;failure.clear();failOccurrence=1;occurrence=0;estop=0;for(auto& a:axes)a=Axis_Status{};}
inline long call(const char* name){calls.append(name);return failure==name&&++occurrence==failOccurrence?77:0;}
}
namespace fake {
template<class T> double numeric(T value){if constexpr(std::is_arithmetic_v<T>)return double(value);else return 0;}
template<class... Args> long record(const char* name,Args... args){arguments[name].append(QVector<double>{numeric(args)...});return call(name);}
}
#define MOCK(name) template<class... Args> inline long name(Args... args){return fake::record(#name,args...);}
MOCK(SLDM_Set_Net_Timeout) MOCK(SLDM_Set_Card_List) MOCK(SLDM_Open_IP) MOCK(SLDM_Open) MOCK(SLDM_Get_Card_List)
MOCK(SLDM_Set_Pulse_Mode) MOCK(SLDM_Set_Pulse_Pol) MOCK(SLDM_Set_Dir_Pol)
MOCK(SLDM_Set_ServoOn_Pol) MOCK(SLDM_Set_ServoAlm_Config) MOCK(SLDM_Set_RstAlm_Pol)
MOCK(SLDM_Set_Plimit_Config) MOCK(SLDM_Set_Nlimit_Config) MOCK(SLDM_Set_Org_Config)
MOCK(SLDM_Set_Soft_Limit) MOCK(SLDM_Set_Enc_Enable) MOCK(SLDM_Set_Enc_Pol) MOCK(SLDM_Set_Enc_Mult)
MOCK(SLDM_Set_EncZ_Pol) MOCK(SLDM_Set_Enc_Filter) MOCK(SLDM_Set_Delta_Step_Angle)
MOCK(SLDM_Set_Home_Oft) MOCK(SLDM_Set_Home_Para) MOCK(SLDM_Set_Delta_Para) MOCK(SLDM_Delta_Init)
MOCK(SLDM_Set_ServoOn) MOCK(SLDM_Reset) MOCK(SLDM_Delta_Home_Move) 
MOCK(SLDM_JogP) MOCK(SLDM_JogM) MOCK(SLDM_PTPn_Stop_Plus)
MOCK(SLDM_PTPn_Stop) MOCK(SLDM_Stop) MOCK(SLDM_Estop) MOCK(SLDM_Set_OutBit)
#undef MOCK
inline long SLDM_Close(unsigned long card){if(fake::closeDelay)QThread::msleep(fake::closeDelay);return fake::record("SLDM_Close",card);}
inline long SLDM_Get_SysTicks(unsigned long,unsigned long* v){*v=123;return fake::call("SLDM_Get_SysTicks");}
inline long SLDM_Get_InBit(unsigned long,unsigned long,unsigned long,unsigned long* v){*v=fake::estop;return fake::call("SLDM_Get_InBit");}
inline long SLDM_Get_OutBit(unsigned long,unsigned long,unsigned long,unsigned long* v){*v=0;return fake::call("SLDM_Get_OutBit");}
inline long SLDM_Get_Axis_Status(unsigned long,unsigned long axis,Axis_Status* v){*v=fake::axes[axis];return fake::call("SLDM_Get_Axis_Status");}
inline long SLDM_Get_Pos(unsigned long,unsigned long axis,long* v){*v=fake::pulses[axis];return fake::call("SLDM_Get_Pos");}
inline long SLDM_Get_Enc(unsigned long,unsigned long,long* v){*v=0;return fake::call("SLDM_Get_Enc");}
inline long SLDM_Get_Vel(unsigned long,unsigned long,float* v){*v=0;return fake::call("SLDM_Get_Vel");}
inline long SLDM_Get_Delta_Angle(unsigned long,unsigned long,float* v){*v=0;return fake::call("SLDM_Get_Delta_Angle");}
inline long SLDM_Get_Delta_XYZR_Pos(unsigned long,unsigned long axis,float* v){*v=axis==2?fake::currentZ:0;return fake::call("SLDM_Get_Delta_XYZR_Pos");}
inline long SLDM_Get_MoveFifo_FuncNum(unsigned long,unsigned long* v){*v=0;return fake::call("SLDM_Get_MoveFifo_FuncNum");}
inline long SLDM_Get_MoveFifo_Running(unsigned long,unsigned long* v){*v=0;return fake::call("SLDM_Get_MoveFifo_Running");}
inline long SLDM_Set_MoveFifo_En(unsigned long card,unsigned long en){const auto rc=fake::record("SLDM_Set_MoveFifo_En",card,en);if(!rc)fake::fifoEnabled=en!=0;return rc;}
inline long SLDM_Get_MoveFifo_Space(unsigned long,unsigned long* v){*v=fake::spaces.isEmpty()?1024:fake::spaces[qMin(fake::spaceReads,fake::spaces.size()-1)];++fake::spaceReads;if(fake::estopOnSpaceRead==fake::spaceReads)fake::estop=1;const auto rc=fake::call("SLDM_Get_MoveFifo_Space");return rc?rc:fake::fifoEnabled?0:-36;}

inline long SLDM_Delta_PTPn_DPoint(unsigned long card,unsigned long n,unsigned long*,float* pos,float start,float speed,float end,float acc,float dec,unsigned long curve){for(int i=0;i<4;++i)fake::lastPosition[i]=pos[i];return fake::record("SLDM_Delta_PTPn_DPoint",card,n,pos[0],pos[1],pos[2],pos[3],start,speed,end,acc,dec,curve);}
