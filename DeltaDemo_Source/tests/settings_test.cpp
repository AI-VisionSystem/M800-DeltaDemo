#include "Settings.h"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <cstdio>
#include <limits>
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);int failures=0,checks=0;
    auto expect=[&](bool ok,const char* name){++checks;if(!ok){++failures;qCritical()<<"FAIL:"<<name;}};
    QTemporaryDir dir(QCoreApplication::applicationDirPath()+"/settings-test-XXXXXX");
    expect(dir.isValid(),"temporary directory inside runtime");
    const QString path=dir.path()+"/settings.json";
    demo::Settings s;expect(s.validate().isEmpty(),"defaults validate");expect(s.axes[3].homeMode==0,"rotary axis defaults to home in place");for(const auto& axis:s.axes)expect(axis.dirPol==1,"all direction polarities default negative");
    s.axes[0].pulseMode=2;s.axes[3].homeOffset=-314;s.axes[1].encoderEnable=1;
    s.axes[3].homeMode=5;s.estopLevel=0;
    s.points[0].position[3]=80;s.points[0].curve=4;
    expect(s.save(path).isEmpty(),"save");demo::Settings restored;
    expect(restored.load(path).isEmpty(),"load");
    for(int a=0;a<4;++a)for(const auto& f:demo::axisFields())expect(restored.axes[a].*f.member==s.axes[a].*f.member,f.key);
    expect(restored.estopLevel==0 && restored.points[0].position[3]==80 && restored.points[0].curve==4,"point and estop round trip");
    demo::Point p=s.points.first();p.position[3]=360;expect(demo::validatePoint(p,s).isEmpty() && p.position[3]==360,"R stays absolute without retired workspace limits");
    p=s.points.first();p.position[0]=std::numeric_limits<double>::quiet_NaN();expect(!demo::validatePoint(p,s).isEmpty(),"NaN target rejected");
    p=s.points.first();p.start=p.speed+1;expect(!demo::validatePoint(p,s).isEmpty(),"start speed above steady rejected");
    auto bad=s;bad.axes[1].pulseMode=0.5;expect(!bad.validate().isEmpty(),"fractional enum rejected");
    bad=s;bad.axes[0].homeMode=2;expect(!bad.validate().isEmpty(),"unsupported home mode rejected");
    bad=s;bad.axisSpeed=6;expect(!bad.validate().isEmpty(),"axis speed above five rejected");
    QFile file(path);file.open(QIODevice::ReadOnly);auto json=QJsonDocument::fromJson(file.readAll()).object();file.close();
    auto write=[&](const QJsonObject& o){file.open(QIODevice::WriteOnly|QIODevice::Truncate);file.write(QJsonDocument(o).toJson());file.close();};
    expect(!json.contains("lower")&&!json.contains("upper"),"retired workspace limits not saved");
    auto legacy=json;legacy["schema"]=1;legacy.remove("cartJogStep");legacy["lower"]=QJsonArray{0,0,0,0};legacy["upper"]=QJsonArray{0,0,0,0};write(legacy);
    expect(restored.load(path).isEmpty(),"legacy limits ignored");
    auto invalid=json;invalid.remove("axes");write(invalid);
    expect(!restored.load(path).isEmpty(),"missing axes rejected");
    expect(restored.axes[3].homeOffset==-314,"failed load is atomic");
    invalid=json;invalid["estopInput"]=0.5;write(invalid);expect(!restored.load(path).isEmpty(),"fractional integer JSON rejected");
    invalid=json;invalid["axisSpeed"]="20";write(invalid);expect(!restored.load(path).isEmpty(),"wrong JSON type rejected");
    invalid=json;invalid["schema"]=3;write(invalid);expect(!restored.load(path).isEmpty(),"future schema rejected");
    invalid=json;invalid["geometry"]=QJsonArray{1,2};write(invalid);expect(!restored.load(path).isEmpty(),"wrong array length rejected");
    std::printf("%d checks, %d failures\n",checks,failures);return failures?1:0;
}
