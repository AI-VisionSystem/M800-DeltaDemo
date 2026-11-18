#include "MainWindow.h"
#include "sldmv.h"
#include <QtWidgets>
#include <cstdio>
#include <cmath>



namespace demo {
struct ControllerTestAccess {
    static void ready(RobotController& c){c.m_connected=true;c.m_configured=true;c.m_initialized=true;c.m_homed=false;}
    static void release(RobotController& c){c.m_connected=false;}
    static void motionReady(RobotController& c){ready(c);c.m_homed=true;c.m_active=false;c.m_homing=false;c.m_fifo=false;}
};
struct WindowTestAccess {
    static int blockedClose(MainWindow& w){
        fake::reset();ControllerTestAccess::motionReady(w.m_robot);w.m_timer->stop();fake::closeDelay=10000;
        QTimer::singleShot(0,[]{if(auto* box=qobject_cast<QMessageBox*>(QApplication::activeModalWidget()))box->button(QMessageBox::Yes)->click();});
        QCloseEvent event;QApplication::sendEvent(&w,&event);return 1; // Deadline should terminate first.
    }
    static int run(MainWindow& w) {
        int checks=0,failures=0;
        auto expect=[&](bool ok,const char* name){++checks;if(!ok){++failures;std::fprintf(stderr,"FAIL: %s\n",name);}};
        w.m_timer->stop();fake::reset();
        const auto initial=w.settings();
        QString speedText="8";int cursor=1;
        expect(w.m_speed[0]->validate(speedText,cursor)==QValidator::Acceptable&&speedText=="5.000","typed speed above maximum becomes five");
        w.m_speed[0]->setValue(0);expect(w.m_speed[0]->value()==1,"axis speed minimum one");
        w.m_speed[0]->setValue(9);expect(w.m_speed[0]->value()==5,"axis speed maximum five");
        w.m_speed[0]->setValue(initial.axisSpeed);
        expect(initial.points.size()==4,"four default points");
        const double xy[4][2]={{-10,-10},{-10,10},{10,-10},{10,10}};
        for(int a=0;a<4;++a)expect(initial.points[a].position[0]==xy[a][0]&&initial.points[a].position[1]==xy[a][1],"default XY order");
        Settings edited=initial;
        for(int field=0;field<axisFields().size();++field)for(int axis=0;axis<4;++axis) {
            const auto& f=axisFields()[field];double v=edited.axes[axis].*f.member;
            const QString key=f.key;
            if(f.choices)v=(axis+1)%(int(f.maximum)+1);
            else if(key=="homeMode")v=13;
            else if(key=="homeOffset")v=-123-axis;
            else if(key=="pulses")v=12345+axis;
            else if(key=="homeLow")v=0.1+axis;
            else if(key=="homeHigh")v=0.5+axis;
            else v=2+axis;
            edited.axes[axis].*f.member=v;
            auto* widget=w.m_axisEditors[field][axis];
            if(auto* combo=qobject_cast<QComboBox*>(widget))combo->setCurrentIndex(combo->findData(int(v)));
            else qobject_cast<QDoubleSpinBox*>(widget)->setValue(v);
        }
        const auto actual=w.settings();expect(actual.validate().isEmpty(),"edited UI values valid");
        for(int a=0;a<4;++a)for(const auto& f:axisFields())expect(std::abs(actual.axes[a].*f.member-edited.axes[a].*f.member)<1e-6,f.key);
        ControllerTestAccess::ready(w.m_robot);
        expect(w.m_robot.applyParameters(actual).isEmpty(),"apply edited UI values to SDK");
        for(const auto& call:fake::arguments.value("SLDM_Set_Soft_Limit"))expect(call[2]==0,"retired joint soft limits disabled");
        struct Binding {const char* field;const char* api;int argument;};
        const Binding bindings[]={
            {"pulseMode","SLDM_Set_Pulse_Mode",2},{"pulsePol","SLDM_Set_Pulse_Pol",2},{"dirPol","SLDM_Set_Dir_Pol",2},
            {"servoOnPol","SLDM_Set_ServoOn_Pol",2},{"alarmEnable","SLDM_Set_ServoAlm_Config",2},{"alarmPol","SLDM_Set_ServoAlm_Config",3},
            {"resetPol","SLDM_Set_RstAlm_Pol",2},{"positiveEnable","SLDM_Set_Plimit_Config",2},{"positivePol","SLDM_Set_Plimit_Config",3},
            {"negativeEnable","SLDM_Set_Nlimit_Config",2},{"negativePol","SLDM_Set_Nlimit_Config",3},{"originEnable","SLDM_Set_Org_Config",2},{"originPol","SLDM_Set_Org_Config",3},
            {"encoderEnable","SLDM_Set_Enc_Enable",2},{"encoderPol","SLDM_Set_Enc_Pol",2},{"encoderMult","SLDM_Set_Enc_Mult",2},{"encoderZPol","SLDM_Set_EncZ_Pol",2},{"encoderFilter","SLDM_Set_Enc_Filter",2},
            {"angle","SLDM_Set_Delta_Step_Angle",2},{"pulses","SLDM_Set_Delta_Step_Angle",3},{"deltaPol","SLDM_Set_Delta_Step_Angle",4},
            {"homeOffset","SLDM_Set_Home_Oft",2},{"homeDirection","SLDM_Set_Home_Para",2},{"homeLow","SLDM_Set_Home_Para",3},
            {"homeHigh","SLDM_Set_Home_Para",4},{"homeAcc","SLDM_Set_Home_Para",5},{"homeMode","SLDM_Set_Home_Para",6},{"homeEncoder","SLDM_Set_Home_Para",7}
        };
        for(const auto& b:bindings)for(const auto& f:axisFields())if(QString(f.key)==b.field) {
            const auto calls=fake::arguments.value(b.api);expect(calls.size()==4,b.api);
            for(int a=0;a<4&&a<calls.size();++a)expect(calls[a].size()>b.argument&&calls[a][1]==a&&std::abs(calls[a][b.argument]-edited.axes[a].*f.member)<0.0001,b.field);
        }
        expect(w.m_robot.initialize().isEmpty(),"initialize after edited parameters");
        const int callsBefore=fake::calls.size();
        auto* origin=w.findChild<QComboBox*>("axis0_originPol");origin->setCurrentIndex(1-origin->currentIndex());
        expect(w.m_robot.configured()&&w.m_robot.initialized()&&!w.m_robot.homed(),"live origin edit preserves setup and requires rehome");
        expect(fake::calls.size()>callsBefore&&fake::calls.last()=="SLDM_Set_Org_Config","origin edit immediately writes grouped SDK setting");
        expect(w.m_robot.applyParameters(w.settings()).isEmpty()&&w.m_robot.initialize().isEmpty(),"explicit reapply accepts changed origin parameter");
        for(const auto& b:bindings)for(int field=0;field<axisFields().size();++field)if(QString(axisFields()[field].key)==b.field) {
            for(int axis=0;axis<4;++axis) {
                expect(w.m_robot.applyAxisParameter(axis,field,actual.axes[axis]).isEmpty(),"single field auto-apply succeeds");
                const auto args=fake::arguments.value(b.api).last();
                expect(args[1]==axis&&std::abs(args[b.argument]-actual.axes[axis].*axisFields()[field].member)<0.0001,"auto-apply SDK field binding");
            }
        }
        fake::failure="SLDM_Set_Org_Config";origin->setCurrentIndex(1-origin->currentIndex());
        expect(!w.m_robot.configured()&&!origin->styleSheet().isEmpty(),"failed auto-write invalidates and marks pending field");
        fake::failure.clear();fake::occurrence=0;origin->setCurrentIndex(1-origin->currentIndex());
        expect(origin->styleSheet().isEmpty(),"successful retry clears pending mark");
        expect(w.m_robot.applyParameters(w.settings()).isEmpty()&&w.m_robot.initialize().isEmpty(),"restore configuration after live-write checks");
        // Readback Z is deliberately not zero and not the old demonstration -173.
        fake::currentZ=-246.375f;
        for(auto& a:fake::axes){a.homed=false;a.homing=true;}
        // Start home requires idle, then the controller reports homing.
        for(auto& a:fake::axes)a.homing=false;
        expect(w.startHome().isEmpty(),"start home");
        expect(!w.startHome().isEmpty()&&w.m_homeZPending,"repeated home click keeps pending Z synchronization");
        for(auto& a:fake::axes)a.homing=true;
        w.poll();expect(w.m_target[2]->value()==initial.points[0].position[2],"do not sync during homing");
        for(auto& a:fake::axes){a.homing=false;a.homed=true;}
        QThread::msleep(210);w.poll();
        expect(w.m_target[2]->value()==-246.375,"point Z uses measured home Z");
        const auto homed=w.settings();
        for(int row=0;row<homed.points.size();++row)expect(homed.points[row].position[2]==-246.375&&homed.points[row].position[0]==xy[row][0]&&homed.points[row].position[1]==xy[row][1],"buffer Z synced and XY preserved");
        w.m_target[2]->setValue(-230);qobject_cast<QDoubleSpinBox*>(w.m_points->cellWidget(0,2))->setValue(-229);w.poll();
        expect(w.m_target[2]->value()==-230&&w.settings().points[0].position[2]==-229,"later edits not overwritten by polling");
        expect(w.startHome().isEmpty(),"second home");fake::currentZ=-255;
        fake::failure="SLDM_Get_Delta_XYZR_Pos";QThread::msleep(210);w.poll();
        expect(w.m_target[2]->value()==-230,"failed position read never synchronizes home Z");
        fake::reset();w.m_fault=false;w.m_pollError.clear();ControllerTestAccess::ready(w.m_robot);
        expect(w.startHome().isEmpty(),"home before cancellation");w.m_robot.stop();w.poll();
        expect(w.m_target[2]->value()==-230&&!w.m_homeZPending,"cancelled home never synchronizes Z");
        // Exercise actual button signals with a fake SDK; no hardware connection.
        auto prepare=[&]{w.m_robot.stop();w.setSettings(initial);fake::reset();ControllerTestAccess::motionReady(w.m_robot);w.m_fault=false;w.m_pollError.clear();w.m_pollError.clear();w.m_jogHeld=false;w.poll();fake::calls.clear();fake::arguments.clear();};
        auto button=[&](const QString& label)->QPushButton*{for(auto* b:w.findChildren<QPushButton*>())if(b->text()==label)return b;return nullptr;};
        auto click=[&](const QString& label){auto* b=button(label);expect(b!=nullptr,"button exists");if(b)b->click();};
        prepare();expect(button("减速停止")==nullptr,"top deceleration button removed");expect(w.m_connectionState->text().contains("已连接")&&w.m_connectionState->text().contains("心跳："),"controller heartbeat shown in permanent status");fake::failure="SLDM_Get_SysTicks";w.poll();expect(w.m_connectionState->text().contains("未连接")&&w.m_connectionState->text().contains("—"),"read failure never displays connected or stale heartbeat");prepare();expect(button("四轴回零")==nullptr&&button("停止回零")!=nullptr,"main stop homing present, duplicate home absent");expect(!w.m_enableServos->isEnabled()&&w.m_disableServos->isEnabled(),"all enabled only allows disable");fake::axes[0].servon=false;w.poll();expect(w.m_enableServos->isEnabled()&&w.m_disableServos->isEnabled(),"partial enable permits completing or clearing enables");for(auto& axis:fake::axes)axis.servon=false;w.poll();expect(w.m_enableServos->isEnabled()&&!w.m_disableServos->isEnabled(),"all disabled only allows enable");prepare();click("1. 下发参数");expect(fake::calls.count("SLDM_Set_Home_Para")==4&&fake::calls.count("SLDM_Set_Delta_Para")==1,"apply button writes axes and geometry");
        click("2. 初始化");expect(fake::calls.count("SLDM_Delta_Init")==1,"initialize button calls SDK");
        auto confirm=[&](QMessageBox::StandardButton answer){QTimer::singleShot(0,[answer]{if(auto* box=qobject_cast<QMessageBox*>(QApplication::activeModalWidget()))if(auto* b=box->button(answer))b->click();});};
        prepare();w.show();QApplication::processEvents();confirm(QMessageBox::No);click("3. 四轴回零");expect(!fake::calls.contains("SLDM_Delta_Home_Move"),"cancel homing sends no home command");confirm(QMessageBox::Yes);click("3. 四轴回零");expect(fake::calls.count("SLDM_Delta_Home_Move")==1,"confirmed homing sends one home command");expect(w.m_position->text().contains("当前空间坐标"),"current position explicitly labeled");
        for(const auto& label:{"停止运动","停止回零","软件急停"}){prepare();click(label);expect(fake::calls.count("SLDM_Stop")==4&&fake::calls.contains("SLDM_PTPn_Stop_Plus"),"all stop buttons stop joint and Cartesian paths");expect(fake::calls.count("SLDM_Estop")== (QString(label)=="软件急停"?4:0),"immediate stop only for emergency");}
        for(bool on:{false,true}){prepare();for(auto& axis:fake::axes)axis.servon=!on;w.poll();click(on?"打开四轴使能":"关闭四轴使能");const auto calls=fake::arguments.value("SLDM_Set_ServoOn");expect(calls.size()==4,"servo button addresses all axes");for(int a=0;a<calls.size();++a)expect(calls[a][1]==a&&calls[a][2]==(on?1:0),"servo direction and axis correct");}
        prepare();click("四轴清报警");expect(fake::calls.count("SLDM_Reset")==4,"reset alarms on all four axes");
        for(bool on:{false,true}){prepare();w.m_ioBit->setValue(7);click(on?"DO置1":"DO置0");const auto calls=fake::arguments.value("SLDM_Set_OutBit");expect(calls.size()==1&&calls[0][2]==7&&calls[0][3]==(on?1:0),"DO uses selected bit and level");}
        prepare();click("读取IO");expect(w.m_ioState->text()=="DI 0 / DO 0","read IO button displays readback");w.m_ioBit->setValue(8);expect(w.m_ioState->text()=="DI — / DO —","changing bit clears stale IO");
        prepare();w.m_target[0]->setValue(12);w.m_target[1]->setValue(-8);w.m_target[2]->setValue(-220);w.m_target[3]->setValue(25);w.m_speed[2]->setValue(32);w.m_speed[3]->setValue(240);confirm(QMessageBox::No);click("移动到目标点");expect(!fake::calls.contains("SLDM_Delta_PTPn_DPoint"),"cancel point confirmation sends no motion");confirm(QMessageBox::Yes);click("移动到目标点");
        auto commands=fake::arguments.value("SLDM_Delta_PTPn_DPoint");expect(commands.size()==1&&commands[0][2]==12&&commands[0][3]==-8&&commands[0][4]==-220&&commands[0][5]==25&&commands[0][7]==32&&commands[0][9]==240,"point button sends target and current manual speeds");
        for(int kind=0;kind<2;++kind){QGroupBox* group=nullptr;for(auto* g:w.findChildren<QGroupBox*>())if(g->title().startsWith(kind==0?"单轴JOG":"空间 JOG"))group=g;expect(group!=nullptr,"JOG group exists");if(!group)continue;
            auto* grid=qobject_cast<QGridLayout*>(group->layout());
            for(int a=0;a<4;++a)for(int dir=0;dir<2;++dir){prepare();auto* b=qobject_cast<QPushButton*>(grid->itemAtPosition(a,dir+1)->widget());QMetaObject::invokeMethod(b,"pressed",Qt::DirectConnection);
                if(kind==0){const auto calls=fake::arguments.value(dir==0?"SLDM_JogP":"SLDM_JogM");expect(calls.size()==1&&calls[0][1]==a&&calls[0][3]==1,"joint JOG button axis direction speed");}
                else if(a==3){const auto calls=fake::arguments.value("SLDM_Delta_PTPn_DPoint");expect(calls.size()==1&&calls[0][1]==4&&calls[0][5]==(dir==0?999:-999)&&calls[0][11]==4,"R button sends Delta R target");}
                else {expect(fake::calls.count("SLDM_Delta_PTPn_DPoint")==1,"Cartesian press sends one target");for(int i=0;i<4;++i)expect(fake::lastPosition[i]==(i==2?-173.0f:0.0f)+(i==a?(dir==0?1000.0f:-1000.0f):0.0f),"Cartesian button selects correct coordinate and sign");}
                QMetaObject::invokeMethod(b,"released",Qt::DirectConnection);expect(!w.m_jogHeld&&fake::calls.contains("SLDM_PTPn_Stop_Plus"),"JOG release stops");
            }
        }
        prepare();qobject_cast<QDoubleSpinBox*>(w.m_points->cellWidget(1,5))->setValue(37);qobject_cast<QComboBox*>(w.m_points->cellWidget(1,9))->setCurrentIndex(4);confirm(QMessageBox::Cancel);click("开始缓冲运动");expect(!fake::calls.contains("SLDM_Set_MoveFifo_En")&&!fake::calls.contains("SLDM_Delta_PTPn_DPoint"),"cancel buffer confirmation neither enables FIFO nor sends motion");confirm(QMessageBox::Yes);click("开始缓冲运动");commands=fake::arguments.value("SLDM_Delta_PTPn_DPoint");expect(commands.size()==4,"buffer button sends all four rows");expect(commands.size()==4&&commands[1][7]==37&&commands[1][11]==4,"buffer sends edited speed and curve");
        prepare();w.show();QApplication::processEvents();
        confirm(QMessageBox::No);click("添加目标点");expect(w.m_points->rowCount()==4,"cancel add leaves table intact");
        confirm(QMessageBox::Yes);click("添加目标点");expect(w.m_points->rowCount()==5,"confirmed add appends point");
        w.m_points->setCurrentCell(2,0);
        confirm(QMessageBox::No);click("删除选中点");expect(w.m_points->rowCount()==5,"cancel delete leaves table intact");
        confirm(QMessageBox::Yes);click("删除选中点");expect(w.m_points->rowCount()==4&&w.settings().points[2].position[1]==10,"confirmed delete removes selected row");
        prepare();click("关闭控制器");expect(!w.m_robot.connected()&&fake::calls.contains("SLDM_Close")&&!fake::calls.contains("SLDM_Set_ServoOn"),"disconnect preserves enables and closes session");
        prepare();confirm(QMessageBox::No);QCloseEvent cancelled;QApplication::sendEvent(&w,&cancelled);expect(!cancelled.isAccepted()&&w.m_robot.connected()&&!fake::calls.contains("SLDM_Close"),"cancel close retains controller session");
        confirm(QMessageBox::Yes);QCloseEvent accepted;QApplication::sendEvent(&w,&accepted);expect(accepted.isAccepted()&&!w.m_robot.connected()&&fake::calls.contains("SLDM_Close"),"confirmed close disconnects controller");
        w.m_closing=false;prepare();fake::failure="SLDM_Get_SysTicks";w.poll();const int readCount=fake::calls.count("SLDM_Get_SysTicks");w.poll();
        expect(fake::calls.count("SLDM_Get_SysTicks")==readCount,"failed polling does not repeat SDK reads");
        fake::occurrence=0;confirm(QMessageBox::Yes);QCloseEvent failedClose;QApplication::sendEvent(&w,&failedClose);
        expect(failedClose.isAccepted()&&!w.m_robot.connected(),"read failure cannot veto application exit");
        QProcess blocked;blocked.start(QCoreApplication::applicationFilePath(),{"--blocked-close-test"});
        expect(blocked.waitForFinished(6000)&&blocked.exitCode()==0,"blocked SDK close exits within deadline in isolated process");
        if(blocked.state()!=QProcess::NotRunning){blocked.kill();blocked.waitForFinished();}
        ControllerTestAccess::release(w.m_robot);
        std::printf("%d UI/SDK checks, %d failures\n",checks,failures);return failures?1:0;
    }
};
}
int main(int argc,char** argv){
    QApplication app(argc,argv);app.setStyle("Fusion");demo::MainWindow window;
    if(app.arguments().contains("--blocked-close-test"))return demo::WindowTestAccess::blockedClose(window);
    return demo::WindowTestAccess::run(window);
}
