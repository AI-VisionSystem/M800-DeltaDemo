#include "Settings.h"
#include <QFile>
#include <QSaveFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <cmath>

namespace demo {
const QVector<AxisField>& axisFields() {
    static const QVector<AxisField> fields = {
        {"pulseMode","脉冲方式", &AxisSettings::pulseMode,0,2,0,"脉冲＋方向|AB相|双脉冲"},
        {"pulsePol","脉冲极性", &AxisSettings::pulsePol,0,1,0,"正|负"},
        {"dirPol","方向极性", &AxisSettings::dirPol,0,1,0,"正|负"},
        {"servoOnPol","使能极性", &AxisSettings::servoOnPol,0,1,0,"正|负"},
        {"alarmEnable","伺服报警检测", &AxisSettings::alarmEnable,0,1,0,"关闭|启用"},
        {"alarmPol","伺服报警极性", &AxisSettings::alarmPol,0,1,0,"正|负"},
        {"resetPol","清报警输出极性", &AxisSettings::resetPol,0,1,0,"正|负"},
        {"positiveEnable","正限位检测", &AxisSettings::positiveEnable,0,1,0,"关闭|启用"},
        {"positivePol","正限位极性", &AxisSettings::positivePol,0,1,0,"正|负"},
        {"negativeEnable","负限位检测", &AxisSettings::negativeEnable,0,1,0,"关闭|启用"},
        {"negativePol","负限位极性", &AxisSettings::negativePol,0,1,0,"正|负"},
        {"originEnable","原点检测", &AxisSettings::originEnable,0,1,0,"关闭|启用"},
        {"originPol","原点有效状态", &AxisSettings::originPol,0,1,0,"输入导通有效|输入不导通有效"},
        {"encoderEnable","编码器使能", &AxisSettings::encoderEnable,0,1,0,"关闭|启用"},
        {"encoderPol","编码器极性", &AxisSettings::encoderPol,0,1,0,"正|负"},
        {"encoderMult","编码器倍率", &AxisSettings::encoderMult,0.001,1000,3,nullptr},
        {"encoderZPol","编码器Z极性", &AxisSettings::encoderZPol,0,1,0,"正|负"},
        {"encoderFilter","编码器滤波（SDK单位）", &AxisSettings::encoderFilter,0,10000,3,nullptr},
        {"angle","当量角度（°）", &AxisSettings::angle,0.001,36000,3,nullptr},
        {"pulses","对应脉冲数（p）", &AxisSettings::pulses,1,2000000000,0,nullptr},
        {"deltaPol","Delta角度极性", &AxisSettings::deltaPol,0,1,0,"正|负"},
        {"homeMode","回零模式（SDK编号）", &AxisSettings::homeMode,0,14,0,nullptr},
        {"homeEncoder","回零使用编码器", &AxisSettings::homeEncoder,0,1,0,"关闭|启用"},
        {"homeDirection","回零方向", &AxisSettings::homeDirection,0,1,0,"正向|反向"},
        {"homeLow","回零低速（p/ms）", &AxisSettings::homeLow,0.001,8000,3,nullptr},
        {"homeHigh","回零高速（p/ms）", &AxisSettings::homeHigh,0.001,8000,3,nullptr},
        {"homeAcc","回零加速度（p/ms²）", &AxisSettings::homeAcc,0.001,8000,3,nullptr},
        {"homeOffset","回零偏置（p）", &AxisSettings::homeOffset,-2000000000,2000000000,0,nullptr}
    };
    return fields;
}
Settings::Settings() {
    axes[3].angle=36;
    axes[3].homeMode=0;
    for(const auto& xy: {std::array<double,2>{-10,-10}, {-10,10}, {10,-10}, {10,10}}) {
        Point p;p.position[0]=xy[0];p.position[1]=xy[1];points.append(p);
    }
}
static bool inRange(double v,double lo,double hi) { return std::isfinite(v)&&v>=lo&&v<=hi; }
QString validatePoint(const Point& p,const Settings& s) {
    Q_UNUSED(s)
    for(int i=0;i<4;++i)
        if(!inRange(p.position[i],-100000,100000)) return QStringLiteral("坐标不是有限数值或超出数值输入范围±100000（非机械软限位）");
    if(!inRange(p.speed,0.001,8000)||!inRange(p.start,0,p.speed)||!inRange(p.end,0,p.speed)
       ||!inRange(p.acc,0.001,8000)||!inRange(p.dec,0.001,8000)||p.curve<0||p.curve>4)
        return QStringLiteral("速度/加减速度/曲线无效；起止速度不得大于最大速度");
    return {};
}
QString Settings::validate() const {
    const auto octets=hostIp.split('.');
    if(octets.size()!=4 || octets[0]!="192" || octets[1]!="168" || octets[2]!="100")
        return QStringLiteral("E450主机IP须为192.168.100.x");
    bool ok=false; const int last=octets[3].toInt(&ok);
    if(!ok||last<1||last>254||cardAddress<1||cardAddress>254||last==cardAddress)
        return QStringLiteral("主机/控制卡IP末段须为1~254，且不能相同");
    for(int a=0;a<4;++a) {
        for(const auto& f:axisFields()) {
            const double v=axes[a].*f.member;
            if(!inRange(v,f.minimum,f.maximum)||(f.decimals==0&&v!=std::floor(v)))
                return QStringLiteral("轴%1：%2无效").arg(a).arg(QString::fromUtf8(f.label));
        }
        if(axes[a].homeLow>axes[a].homeHigh) return QStringLiteral("回零低速不能大于高速");
        if(!QVector<int>{0,1,3,4,5,7,8,9,10,12,13,14}.contains(int(axes[a].homeMode)))return QStringLiteral("回零模式编号不受SDK支持");
        if(!inRange(geometry[a],0.001,10000)) return QStringLiteral("机械尺寸必须为正数");
    }
    if(!inRange(axisSpeed,1,5)||!inRange(axisAcc,0.001,8000)
       ||!inRange(cartSpeed,0.001,8000)||!inRange(cartAcc,0.001,8000)) return QStringLiteral("手动速度参数无效");
    if(estopInput<0||estopInput>31||estopLevel<0||estopLevel>1) return QStringLiteral("急停输入设置无效");
    if(points.size()>100) return QStringLiteral("Demo最多支持100个缓冲点位");
    for(int i=0;i<points.size();++i) {
        const auto error=validatePoint(points[i],*this);
        if(!error.isEmpty()) return QStringLiteral("点表第%1行：%2").arg(i+1).arg(error);
    }
    return {};
}
static QJsonArray array(const Pose& p) { QJsonArray a; for(double v:p)a.append(v); return a; }
QString Settings::save(const QString& path) const {
    const auto error=validate(); if(!error.isEmpty()) return error;
    QJsonObject o{{"schema",2},{"hostIp",hostIp},{"cardAddress",cardAddress},
        {"geometry",array(geometry)},
        {"axisSpeed",axisSpeed},{"axisAcc",axisAcc},{"cartSpeed",cartSpeed},{"cartAcc",cartAcc},
        {"estopInput",estopInput},{"estopLevel",estopLevel}};
    QJsonArray aa,pp;
    for(const auto& a:axes) { QJsonObject x; for(const auto& f:axisFields())x[f.key]=a.*f.member; aa.append(x); }
    for(const auto& p:points) pp.append(QJsonObject{{"position",array(p.position)},
        {"start",p.start},{"speed",p.speed},{"end",p.end},{"acc",p.acc},{"dec",p.dec},{"curve",p.curve}});
    o["axes"]=aa; o["points"]=pp;
    QSaveFile file(path); if(!file.open(QIODevice::WriteOnly))return file.errorString();
    const auto data=QJsonDocument(o).toJson(QJsonDocument::Indented);
    if(file.write(data)!=data.size()||!file.commit()) return file.errorString();
    return {};
}
QString Settings::load(const QString& path) {
    QFile file(path); if(!file.open(QIODevice::ReadOnly))return file.errorString();
    if(file.size()>1024*1024) return QStringLiteral("参数文件超过1MB");
    QJsonParseError parse; const auto doc=QJsonDocument::fromJson(file.readAll(),&parse);
    if(parse.error!=QJsonParseError::NoError||!doc.isObject())return QStringLiteral("无效JSON参数文件");
    Settings next; const auto o=doc.object(); bool valid=true;
    auto number=[&](const QJsonObject& obj,const char* key) {
        const auto v=obj.value(key); if(!v.isDouble()||!std::isfinite(v.toDouble()))valid=false; return v.toDouble();
    };
    auto integer=[&](const QJsonObject& obj,const char* key) {
        const double v=number(obj,key); if(v!=std::floor(v)||v< -2147483647.0||v>2147483647.0){valid=false;return 0;} return int(v);
    };
    auto readArray=[&](const QJsonValue& value,Pose& dest) {
        if(!value.isArray()||value.toArray().size()!=4){valid=false;return;}
        const auto a=value.toArray(); for(int i=0;i<4;++i){if(!a[i].isDouble())valid=false;dest[i]=a[i].toDouble();}
    };
    const int schema=integer(o,"schema");
    if(schema!=1&&schema!=2)return QStringLiteral("不支持的参数文件版本");
    next.hostIp=o["hostIp"].toString(); next.cardAddress=integer(o,"cardAddress");
    readArray(o["geometry"],next.geometry); // v1 lower/upper intentionally retired
    next.axisSpeed=number(o,"axisSpeed");next.axisAcc=number(o,"axisAcc");
    next.cartSpeed=number(o,"cartSpeed");next.cartAcc=number(o,"cartAcc");
    next.estopInput=integer(o,"estopInput");next.estopLevel=integer(o,"estopLevel");
    const auto aa=o["axes"].toArray(); if(aa.size()!=4)valid=false;
    else for(int i=0;i<4;++i)for(const auto& f:axisFields())next.axes[i].*f.member=number(aa[i].toObject(),f.key);
    next.points.clear();
    if(!o["points"].isArray()||o["points"].toArray().size()>100)valid=false;
    else for(const auto& v:o["points"].toArray()) {
        Point p; const auto x=v.toObject(); readArray(x["position"],p.position);
        p.start=number(x,"start");p.speed=number(x,"speed");p.end=number(x,"end");
        p.acc=number(x,"acc");p.dec=number(x,"dec");p.curve=integer(x,"curve");next.points.append(p);
    }
    if(!valid)return QStringLiteral("参数文件缺少字段或字段类型不正确");
    const auto error=next.validate(); if(!error.isEmpty())return error;
    *this=next; return {};
}
}
