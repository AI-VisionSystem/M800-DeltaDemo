#include "MainWindow.h"
#include <QtWidgets>
#include <cmath>
#include <cstdlib>
namespace demo {
namespace {
// Clamp typed values above the requested maximum instead of rejecting the keystroke.
class AxisSpeedBox : public QDoubleSpinBox {
public:
    AxisSpeedBox(){setDecimals(3);setRange(1,5);setKeyboardTracking(false);setCorrectionMode(QAbstractSpinBox::CorrectToNearestValue);}
    QValidator::State validate(QString& text,int& pos) const override {
        bool ok=false;const double v=locale().toDouble(text,&ok);
        if(ok&&v>maximum()){text=locale().toString(maximum(),'f',decimals());pos=text.size();}
        return QDoubleSpinBox::validate(text,pos);
    }
};
QString platformName() {
#ifdef _WIN32
    return QStringLiteral("Windows · E450");
#else
    return QStringLiteral("M800");
#endif
}
QWidget* wrapScroll(QWidget* content) {
    auto* area=new QScrollArea;area->setWidgetResizable(true);area->setFrameShape(QFrame::NoFrame);area->setWidget(content);return area;
}
QString stateText(bool v){return v?QStringLiteral("是"):QStringLiteral("否");}
}
MainWindow::MainWindow(bool autoOpen) {
    setWindowTitle(QStringLiteral("Delta 蜘蛛手控制 Demo — ")+platformName());resize(1240,900);
    m_axisEditors.resize(axisFields().size());
    auto* root=new QWidget;auto* layout=new QVBoxLayout(root);layout->setContentsMargins(8,4,8,4);layout->setSpacing(0);setCentralWidget(root);
    auto* top=new QHBoxLayout;
    auto* controllerButtons=new QHBoxLayout;
    m_connect=action("打开控制器",root,[this]{
        auto e=m_robot.connectCard(settings());
        if(e.isEmpty()){m_fault=false;m_lastFault.clear();m_pollError.clear();e=setAllServos(true);}
        return e;
    });
    m_disconnect=action("关闭控制器",root,[this]{m_jogHeld=false;return m_robot.disconnectCard();});
    m_connect->setFixedHeight(60);m_disconnect->setFixedHeight(60);
    controllerButtons->addWidget(m_connect,0,Qt::AlignTop);controllerButtons->addWidget(m_disconnect,0,Qt::AlignTop);top->addLayout(controllerButtons);top->addStretch();
    auto* commands=new QGridLayout;
    commands->setVerticalSpacing(2);
    m_load=new QPushButton(QStringLiteral("读取参数文件"));commands->addWidget(m_load,0,0);connect(m_load,&QPushButton::clicked,this,&MainWindow::load);
    auto* saveButton=new QPushButton(QStringLiteral("保存参数文件"));commands->addWidget(saveButton,1,0);connect(saveButton,&QPushButton::clicked,this,&MainWindow::save);
    m_enableServos=action("打开四轴使能",root,[this]{return setAllServos(true);});commands->addWidget(m_enableServos,0,1);
    m_disableServos=action("关闭四轴使能",root,[this]{return setAllServos(false);});commands->addWidget(m_disableServos,1,1);
    auto* emergency=action(QStringLiteral("软件急停"),root,[this]{m_jogHeld=false;m_robot.invalidate();return m_robot.stop(true);});
    emergency->setToolTip(QStringLiteral("清队列、四轴立即停止并停止插补；之后需重新初始化和回零。依赖通讯，不代替硬件急停。"));
    emergency->setStyleSheet("background:#b52b32;color:white;font-weight:bold;padding:8px 20px;");emergency->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Expanding);commands->addWidget(emergency,0,2,2,1);top->addLayout(commands);layout->addLayout(top);
    auto* tabs=new QTabWidget;layout->addWidget(tabs);
    tabs->addTab(wrapScroll(controlPage()),QStringLiteral("蜘蛛手控制"));
    tabs->addTab(wrapScroll(parametersPage()),QStringLiteral("轴参数"));
    tabs->addTab(wrapScroll(homingPage()),QStringLiteral("回零参数"));
    tabs->addTab(diagnosticPage(),QStringLiteral("连接与诊断"));
    m_state=new QLabel;statusBar()->addWidget(m_state,1);
    m_connectionState=new QLabel;statusBar()->addPermanentWidget(m_connectionState);
#ifndef DEMO_WITH_SDK
    statusBar()->addPermanentWidget(new QLabel(QStringLiteral("离线界面预览 · 不连接设备")));
#endif
    setSettings(Settings{});
    m_timer=new QTimer(this);m_timer->setInterval(100);connect(m_timer,&QTimer::timeout,this,&MainWindow::poll);m_timer->start();
    qApp->installEventFilter(this);
    logResult(QStringLiteral("启动"),{});
    m_log->appendPlainText(QStringLiteral("示例参数尚未与客户机械核对。连接成功自动开启四轴使能。轴参数及回零参数结束编辑自动下发，保存仅写文件。空间运动顺序：下发 → 初始化 → 四轴使能 → 回零 → 运动；单轴JOG无需初始化。退出保留使能状态。"));
    poll();
    if(autoOpen)QTimer::singleShot(0,this,[this]{
        const QString path=QCoreApplication::applicationDirPath()+"/DeltaDemo.json";
        if(QFile::exists(path)) {
            Settings s;const auto error=s.load(path);logResult("启动读取参数",error);
            if(!error.isEmpty())return;
            setSettings(s);
        }
        m_connect->click();
    });
}
QDoubleSpinBox* MainWindow::number(double min,double max,int decimals) {
    auto* box=new QDoubleSpinBox;box->setDecimals(decimals);box->setRange(min,max);box->setKeyboardTracking(false);
    box->setMinimumWidth(85);box->setButtonSymbols(QAbstractSpinBox::PlusMinus);return box;
}
QPushButton* MainWindow::action(const QString& title,QWidget* parent,std::function<QString()> fn) {
    auto* button=new QPushButton(title,parent);button->setMinimumHeight(30);
    connect(button,&QPushButton::clicked,this,[this,title,fn]{logResult(title,fn());poll();});return button;
}
QWidget* MainWindow::axisEditor(int field,int axis) {
    const auto& f=axisFields()[field];QWidget* editor=nullptr;
    if(QString::fromLatin1(f.key)=="homeMode") {
        auto* combo=new QComboBox;
        const int codes[]={0,1,3,4,5,7,8,9,10,12,13,14};
        const QStringList labels={"原地回零","一次原点","原点加回找","限位加回找","两次原点","EZ回零","原点后EZ","限位后EZ","原点锁存","EZ锁存","原点反向EZ锁存","限位反向EZ锁存"};
        for(int i=0;i<labels.size();++i)combo->addItem(QString::number(codes[i])+" · "+labels[i],codes[i]);
        editor=combo;
    } else if(f.choices) {
        auto* combo=new QComboBox;const auto labels=QString::fromUtf8(f.choices).split('|');
        for(int i=0;i<labels.size();++i)combo->addItem(labels[i],i);editor=combo;
    } else editor=number(f.minimum,f.maximum,f.decimals);
    if(auto* combo=qobject_cast<QComboBox*>(editor))connect(combo,qOverload<int>(&QComboBox::currentIndexChanged),this,[this,field,axis]{axisChanged(field,axis);});
    else connect(qobject_cast<QDoubleSpinBox*>(editor),qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this,field,axis]{axisChanged(field,axis);});
    editor->setObjectName(QString("axis%1_%2").arg(axis).arg(f.key));
    m_axisEditors[field][axis]=editor;return editor;
}
QTableWidget* MainWindow::parameterTable(const QStringList& keys) {
    auto* table=new QTableWidget(0,5);table->setHorizontalHeaderLabels({"参数","轴0","轴1","轴2","轴3 / R"});
    table->verticalHeader()->hide();table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->verticalHeader()->setDefaultSectionSize(36);
    for(int field=0;field<axisFields().size();++field) {
        if(!keys.contains(QString::fromLatin1(axisFields()[field].key)))continue;
        const int row=table->rowCount();table->insertRow(row);
        auto* label=new QTableWidgetItem(QString::fromUtf8(axisFields()[field].label));label->setFlags(Qt::ItemIsEnabled);table->setItem(row,0,label);
        for(int axis=0;axis<4;++axis)table->setCellWidget(row,axis+1,axisEditor(field,axis));
    }
    return table;
}
QWidget* MainWindow::controlPage() {
    auto* page=new QWidget;auto* layout=new QVBoxLayout(page);layout->setContentsMargins(6,6,6,6);layout->setSpacing(5);
    auto* top=new QHBoxLayout;
    auto* mechanics=new QGroupBox(QStringLiteral("机械参数（mm）"));auto* form=new QGridLayout(mechanics);
    form->setContentsMargins(8,8,8,8);form->setVerticalSpacing(4);
    form->addWidget(new QLabel("尺寸"),0,0);form->addWidget(new QLabel("数值（mm）"),0,1);
    const QStringList labels={"基座半径","动平台半径","主动臂长","从动臂长"};
    for(int i=0;i<4;++i){m_geometry[i]=number(0.001,10000);m_geometry[i]->setFixedHeight(27);m_geometry[i]->setMaximumWidth(110);form->addWidget(new QLabel(labels[i]),i+1,0);form->addWidget(m_geometry[i],i+1,1);connect(m_geometry[i],qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this]{changed();});}
    top->addWidget(mechanics);m_settingsPanels.append(mechanics);
    auto* conversion=new QGroupBox(QStringLiteral("角度与脉冲当量"));auto* conversionGrid=new QGridLayout(conversion);conversionGrid->setContentsMargins(8,8,8,8);conversionGrid->setVerticalSpacing(4);
    conversionGrid->addWidget(new QLabel("轴"),0,0);conversionGrid->addWidget(new QLabel("角度（°）"),0,1);conversionGrid->addWidget(new QLabel("对应脉冲（p）"),0,2);
    for(int a=0;a<4;++a) {
        conversionGrid->addWidget(new QLabel(QString::number(a)),a+1,0);
        for(int f=0;f<axisFields().size();++f){const QString key=axisFields()[f].key;if(key=="angle"||key=="pulses"){auto* editor=axisEditor(f,a);editor->setFixedHeight(27);editor->setMaximumWidth(110);conversionGrid->addWidget(editor,a+1,key=="angle"?1:2);}}
    }
    top->addWidget(conversion);m_settingsPanels.append(conversion);
    auto* information=new QGroupBox(QStringLiteral("实时状态"));auto* info=new QVBoxLayout(information);
    m_position=new QLabel("XYZR：—");m_position->setWordWrap(false);m_position->setMinimumWidth(0);m_position->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);info->addWidget(m_position);
    m_statusTable=new QTableWidget(4,10);m_statusTable->setHorizontalHeaderLabels({"轴","脉冲/编码器","角度°","速度p/ms","使能","回零","运行","原点","正/负限位","报警"});
    m_statusTable->verticalHeader()->hide();m_statusTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_statusTable->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    m_statusTable->horizontalHeader()->setSectionResizeMode(1,QHeaderView::Interactive);m_statusTable->setColumnWidth(1,125);
    for(int column:{2,3,8})m_statusTable->horizontalHeader()->setSectionResizeMode(column,QHeaderView::Stretch);
    m_statusTable->horizontalHeader()->setSectionResizeMode(9,QHeaderView::Fixed);m_statusTable->setColumnWidth(9,44);
    m_statusTable->setFixedHeight(116);m_statusTable->verticalHeader()->setMinimumSectionSize(20);m_statusTable->verticalHeader()->setDefaultSectionSize(22);
    for(int r=0;r<4;++r)for(int c=0;c<10;++c)m_statusTable->setItem(r,c,new QTableWidgetItem(c==0?QString::number(r):QStringLiteral("—")));
    info->addWidget(m_statusTable);top->addWidget(information,2);layout->addLayout(top);
    for(auto* group:{mechanics,conversion,information})group->setFixedHeight(185);
    auto* sequence=new QHBoxLayout;
    auto* apply=action("1. 下发参数",page,[this]{const auto s=settings();const auto e=s.validate();if(!e.isEmpty())return e;const auto result=m_robot.applyParameters(s);if(result.isEmpty()){m_fault=false;m_pollError.clear();for(const auto& editors:m_axisEditors)for(auto* editor:editors){editor->setStyleSheet({});editor->setToolTip("已写入控制卡");}}return result;});
    apply->setToolTip("将机械参数、轴参数及回零参数写入控制卡；保存参数文件不会下发。");sequence->addWidget(apply);
    sequence->addWidget(action("2. 初始化",page,[this]{return m_robot.initialize();}));
    sequence->addWidget(action("3. 四轴回零",page,[this]{
        if(QMessageBox::question(this,"四轴回零","是否进行四轴回零？",QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return QString{};
        return startHome();
    }));
    auto* stopHome=action("停止回零",page,[this]{return m_robot.stop();});stopHome->setFixedWidth(90);sequence->addWidget(stopHome);
    layout->addLayout(sequence);
    auto* speeds=new QHBoxLayout;
    for(int kind=0;kind<2;++kind) {
        auto* group=new QGroupBox(kind==0?"单轴手动速度":"空间手动速度");auto* row=new QHBoxLayout(group);row->setContentsMargins(8,6,8,6);
        for(int j=0;j<2;++j){const int i=kind*2+j;row->addWidget(new QLabel(QStringList{"速度 p/ms","加减速度 p/ms²","速度 mm/s、°/s","加减速度 mm/s²、°/s²"}[i]));m_speed[i]=i==0?new AxisSpeedBox:number(0.001,8000);row->addWidget(m_speed[i]);}
        speeds->addWidget(group);m_settingsPanels.append(group);
    }
    layout->addLayout(speeds);
    auto* motionRow=new QHBoxLayout;
    for(int kind=0;kind<2;++kind) {
        auto* group=new QGroupBox(kind==0?"单轴JOG(默认JOG+向下移动)":"空间 JOG（按住运动，松开停止）");auto* grid=new QGridLayout(group);grid->setVerticalSpacing(4);grid->setContentsMargins(8,6,8,6);
        for(int axis=0;axis<4;++axis) {
            grid->addWidget(new QLabel(kind==0?QString("轴%1").arg(axis):QStringList{"X","Y","Z","R"}[axis]),axis,0);
            for(int direction=0;direction<2;++direction) {
                auto* button=new QPushButton(direction==0?"JOG +":"JOG −");button->setFixedHeight(27);button->setAutoRepeat(false);grid->addWidget(button,axis,1+direction);
                connect(button,&QPushButton::pressed,this,[this,kind,axis,direction]{
                    if(m_jogHeld)return;
                    auto e=applyManualSpeeds();if(e.isEmpty())e=kind==0?m_robot.jogAxis(axis,direction==0):m_robot.jogCartesian(axis,direction==0);
                    m_jogHeld=e.isEmpty();logResult("JOG",e);
                });
                connect(button,&QPushButton::released,this,&MainWindow::stopJog);
            }
        }
        if(kind==0){auto* hint=new QLabel("按住运动，松开停止。\n用于确认方向/调平，之后重新回零。");hint->setWordWrap(true);grid->addWidget(hint,4,0,1,3);}
        else {auto* hint=new QLabel("按住运动，松开减速停止。\nXYZ到达目标后停止；R仅控制轴3。");hint->setWordWrap(true);grid->addWidget(hint,4,0,1,3);}
        motionRow->addWidget(group);
    }
    auto* pointGroup=new QGroupBox("空间点位运动（绝对目标）");auto* pointGrid=new QGridLayout(pointGroup);pointGrid->setVerticalSpacing(4);pointGrid->setContentsMargins(8,6,8,6);
    for(int i=0;i<4;++i){pointGrid->addWidget(new QLabel(QStringList{"X (mm)","Y (mm)","Z (mm)","R (°)"}[i]),i,0);m_target[i]=number(-100000,100000);m_target[i]->setFixedHeight(27);pointGrid->addWidget(m_target[i],i,1);}
    pointGrid->addWidget(action("移动到目标点",pointGroup,[this]{if(QMessageBox::question(this,"确认运动","是否移动到目标点？",QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return QString{};auto e=applyManualSpeeds();if(!e.isEmpty())return e;Point p;for(int i=0;i<4;++i)p.position[i]=m_target[i]->value();p.speed=m_speed[2]->value();p.acc=p.dec=m_speed[3]->value();return m_robot.move(p);}),4,0,1,2);
    motionRow->addWidget(pointGroup);layout->addLayout(motionRow);
    auto* buffer=new QGroupBox("缓冲点表（绝对坐标，每次执行一遍）");auto* bufferLayout=new QVBoxLayout(buffer);
    auto* row=new QHBoxLayout;
    row->addWidget(action("添加目标点",buffer,[this]{if(m_robot.busy()||m_jogHeld)return QStringLiteral("请先停止运动");if(m_points->rowCount()>=100)return QStringLiteral("最多100行");if(QMessageBox::question(this,"添加目标点","确认将当前空间目标位置添加到缓冲点表？",QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return QString{};Point p;for(int i=0;i<4;++i)p.position[i]=m_target[i]->value();p.speed=m_speed[2]->value();p.acc=p.dec=m_speed[3]->value();addPoint(p);return QString{};}));
    row->addWidget(action("删除选中点",buffer,[this]{if(m_robot.busy())return QStringLiteral("请先停止");const int r=m_points->currentRow();if(r<0)return QStringLiteral("请先选中要删除的点");if(QMessageBox::question(this,"删除选中点",QString("确认删除第 %1 行目标点？").arg(r+1),QMessageBox::Yes|QMessageBox::No,QMessageBox::No)==QMessageBox::Yes)m_points->removeRow(r);return QString{};}));
    row->addStretch();row->addWidget(action("开始缓冲运动",buffer,[this]{
        if(QMessageBox::question(this,"确认缓冲运动","是否开始执行缓冲点表运动？",QMessageBox::Yes|QMessageBox::Cancel,QMessageBox::Cancel)!=QMessageBox::Yes)return QString{};
        return m_robot.runBuffer(settings().points);
    }));row->addWidget(action("停止运动",buffer,[this]{m_jogHeld=false;return m_robot.stop();}));bufferLayout->addLayout(row);
    m_points=new QTableWidget(0,10);m_points->setHorizontalHeaderLabels({"X mm","Y mm","Z mm","R °","起速 mm/s","最大速度 mm/s","末速 mm/s","加速度 mm/s²","减速度 mm/s²","曲线"});
    m_points->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);m_points->verticalHeader()->setDefaultSectionSize(29);m_points->setMinimumHeight(155);m_points->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);bufferLayout->addWidget(m_points);layout->addWidget(buffer,1);
    return page;
}
QWidget* MainWindow::parametersPage() {
    auto* page=new QWidget;auto* layout=new QVBoxLayout(page);layout->addWidget(new QLabel("驱动器接口与轴限位参数。连接且停稳时，结束编辑自动写入控制卡；黄色表示尚未生效。保存仅写文件。"));
    QStringList keys;for(const auto& f:axisFields()){const QString key=f.key;if(key!="angle"&&key!="pulses"&&!key.startsWith("home"))keys.append(key);}
    m_axes=parameterTable(keys);m_axes->setMinimumHeight(400);layout->addWidget(m_axes,1);m_settingsPanels.append(page);return page;
}
QWidget* MainWindow::homingPage() {
    auto* page=new QWidget;auto* layout=new QVBoxLayout(page);
    auto* note=new QLabel("原点检测在“轴参数”页设置。常开/常闭原点均可配置：按接线选择输入导通有效或不导通有效，并观察实时原点状态核对。\n回零速度：p/ms；原地回零只适用于已校准姿态。连接且停稳时，结束编辑自动写入控制卡；黄色表示尚未生效，保存仅写文件。");note->setWordWrap(true);layout->addWidget(note);
    QStringList keys;for(const auto& f:axisFields()){const QString key=f.key;if(key.startsWith("home"))keys.append(key);}
    auto* table=parameterTable(keys);table->setMinimumHeight(380);layout->addWidget(table,1);m_settingsPanels.append(table);
    return page;
}
QString MainWindow::setAllServos(bool enable) {
    QString errors;
    for(int i=0;i<4;++i) {
        const auto e=m_robot.servo(i,enable);
        if(e.isEmpty())continue;
        errors+=QString("轴%1：%2\n").arg(i).arg(e);
        if(enable){for(int a=0;a<4;++a){const auto off=m_robot.servo(a,false);if(!off.isEmpty())errors+=off+'\n';}break;}
    }
    return errors;
}
QString MainWindow::startHome() {
    const auto error=m_robot.home();if(error.isEmpty())m_homeZPending=true;return error;
}
void MainWindow::synchronizeHomeZ(const Status& status) {
    if(!status.positionValid||!m_homeZPending||!m_robot.homed()||m_robot.busy()||status.running()||status.estop)return;
    for(const auto& axis:status.axes)if(!axis.homed||axis.alarm)return;
    const double z=status.position[2];
    // Never replace a measured home coordinate by 0 or clamp it into the editor.
    if(!std::isfinite(z)||z<m_target[2]->minimum()||z>m_target[2]->maximum()) {
        m_homeZPending=false;m_robot.invalidate();logResult("回零Z同步",QStringLiteral("控制卡Z无效或超出数值编辑范围，禁止继续运动"));return;
    }
    m_target[2]->setValue(z);
    for(int row=0;row<m_points->rowCount();++row)qobject_cast<QDoubleSpinBox*>(m_points->cellWidget(row,2))->setValue(z);
    m_homeZPending=false;
    logResult(QString("回零成功，点位目标和全部缓冲点Z同步为 %1 mm").arg(z,0,'f',3),{});
}
QString MainWindow::applyManualSpeeds() {
    return m_robot.setManualSpeeds(m_speed[0]->value(),m_speed[1]->value(),m_speed[2]->value(),m_speed[3]->value());
}
QWidget* MainWindow::diagnosticPage() {
    auto* page=new QWidget;auto* layout=new QVBoxLayout(page);
    auto* connection=new QGroupBox(QStringLiteral("控制卡连接"));auto* row=new QHBoxLayout(connection);
    row->addWidget(new QLabel(QStringLiteral("E450本机IP")));m_host=new QLineEdit;m_host->setMaximumWidth(170);row->addWidget(m_host);
    row->addWidget(new QLabel(QStringLiteral("卡IP末段")));m_address=new QSpinBox;m_address->setRange(1,254);row->addWidget(m_address);
    row->addWidget(new QLabel("使用左上角按钮打开或关闭控制器"));layout->addWidget(connection);
#ifndef _WIN32
    m_host->setEnabled(false);m_address->setEnabled(false);
    layout->addWidget(new QLabel(QStringLiteral("M800使用板载枚举＋SLDM_Open，以上E450网口参数不参与板载连接。")));
#endif
    connect(m_host,&QLineEdit::textChanged,this,[this]{changed();});connect(m_address,qOverload<int>(&QSpinBox::valueChanged),this,[this]{changed();});
    auto* safety=new QGroupBox("急停输入监视");auto* safetyRow=new QHBoxLayout(safety);
    safetyRow->addWidget(new QLabel("急停 DI"));m_estopInput=new QSpinBox;m_estopInput->setRange(0,31);safetyRow->addWidget(m_estopInput);
    safetyRow->addWidget(new QLabel("有效电平"));m_estopLevel=new QComboBox;m_estopLevel->addItems({"低电平（0）","高电平（1）"});safetyRow->addWidget(m_estopLevel);
    auto* safetyNote=new QLabel("100ms软件轮询；不代替硬件急停回路。");safetyNote->setWordWrap(true);safetyRow->addWidget(safetyNote);layout->addWidget(safety);m_settingsPanels.append(safety);
    connect(m_estopInput,qOverload<int>(&QSpinBox::valueChanged),this,[this]{changed();});connect(m_estopLevel,qOverload<int>(&QComboBox::currentIndexChanged),this,[this]{changed();});
    auto* io=new QGroupBox(QStringLiteral("基础IO / 轴诊断"));auto* ioRow=new QHBoxLayout(io);
    ioRow->addWidget(new QLabel(QStringLiteral("IO编号")));m_ioBit=new QSpinBox;m_ioBit->setRange(0,31);ioRow->addWidget(m_ioBit);m_ioState=new QLabel("DI — / DO —");ioRow->addWidget(m_ioState);connect(m_ioBit,qOverload<int>(&QSpinBox::valueChanged),this,[this]{m_ioState->setText("DI — / DO —");});
    ioRow->addWidget(action(QStringLiteral("读取IO"),io,[this]{bool di=false,dout=false;const auto e=m_robot.readIo(m_ioBit->value(),di,dout);m_ioState->setText(e.isEmpty()?QString("DI %1 / DO %2").arg(di?1:0).arg(dout?1:0):"DI — / DO —");return e;}));
    ioRow->addWidget(action(QStringLiteral("DO置1"),io,[this]{return m_robot.writeOutput(m_ioBit->value(),true);}));
    ioRow->addWidget(action(QStringLiteral("DO置0"),io,[this]{return m_robot.writeOutput(m_ioBit->value(),false);}));
    ioRow->addWidget(action(QStringLiteral("四轴清报警"),io,[this]{QString errors;for(int i=0;i<4;++i){const auto e=m_robot.resetAlarm(i);if(!e.isEmpty())errors+=e+'\n';}return errors;}));layout->addWidget(io);
    layout->addWidget(new QLabel(QStringLiteral("最近操作与错误（显示SDK函数名及返回码）")));m_log=new QPlainTextEdit;m_log->setReadOnly(true);m_log->setMaximumBlockCount(1000);layout->addWidget(m_log,1);return page;
}
void MainWindow::addPoint(const Point& p) {
    const int r=m_points->rowCount();m_points->insertRow(r);
    const double values[]={p.position[0],p.position[1],p.position[2],p.position[3],p.start,p.speed,p.end,p.acc,p.dec};
    for(int c=0;c<9;++c){auto* b=number(c<4?-100000:((c==5||c>=7)?0.001:0),c<4?100000:8000);b->setValue(values[c]);m_points->setCellWidget(r,c,b);}
    auto* curve=new QComboBox;curve->addItems({"T型","S型","五次多项式","五次多项式S","正弦"});curve->setCurrentIndex(p.curve);m_points->setCellWidget(r,9,curve);
}
Settings MainWindow::settings() const {
    Settings s;s.hostIp=m_host->text().trimmed();s.cardAddress=m_address->value();
    for(int r=0;r<axisFields().size();++r)for(int a=0;a<4;++a){auto* w=m_axisEditors[r][a];auto* combo=qobject_cast<QComboBox*>(w);s.axes[a].*axisFields()[r].member=combo?combo->currentData().toInt():qobject_cast<QDoubleSpinBox*>(w)->value();}
    for(int i=0;i<4;++i){s.geometry[i]=m_geometry[i]->value();}
    s.axisSpeed=m_speed[0]->value();s.axisAcc=m_speed[1]->value();s.cartSpeed=m_speed[2]->value();s.cartAcc=m_speed[3]->value();
    s.estopInput=m_estopInput->value();s.estopLevel=m_estopLevel->currentIndex();s.points.clear();
    for(int r=0;r<m_points->rowCount();++r){Point p;double v[9];for(int c=0;c<9;++c)v[c]=qobject_cast<QDoubleSpinBox*>(m_points->cellWidget(r,c))->value();for(int i=0;i<4;++i)p.position[i]=v[i];p.start=v[4];p.speed=v[5];p.end=v[6];p.acc=v[7];p.dec=v[8];p.curve=qobject_cast<QComboBox*>(m_points->cellWidget(r,9))->currentIndex();s.points.append(p);}
    return s;
}
void MainWindow::setSettings(const Settings& s) {
    m_loading=true;m_host->setText(s.hostIp);m_address->setValue(s.cardAddress);
    for(int r=0;r<axisFields().size();++r)for(int a=0;a<4;++a){auto* w=m_axisEditors[r][a];const double v=s.axes[a].*axisFields()[r].member;if(auto* combo=qobject_cast<QComboBox*>(w))combo->setCurrentIndex(combo->findData(int(v)));else qobject_cast<QDoubleSpinBox*>(w)->setValue(v);}
    for(int i=0;i<4;++i){m_geometry[i]->setValue(s.geometry[i]);m_target[i]->setValue(s.points.isEmpty()?Point{}.position[i]:s.points.first().position[i]);}
    m_speed[0]->setValue(s.axisSpeed);m_speed[1]->setValue(s.axisAcc);m_speed[2]->setValue(s.cartSpeed);m_speed[3]->setValue(s.cartAcc);
    m_estopInput->setValue(s.estopInput);m_estopLevel->setCurrentIndex(s.estopLevel);m_points->setRowCount(0);for(const auto& p:s.points)addPoint(p);
    m_loading=false;m_homeZPending=false;m_robot.invalidate();
}
QString MainWindow::checkForm() const {
    for(const auto& axis:m_axisEditors)for(auto* editor:axis)if(!editor)return QStringLiteral("轴参数编辑器缺失");
    return settings().validate();
}
void MainWindow::changed() {
    if(m_loading)return;
    if(m_robot.busy()) {m_jogHeld=false;logResult(QStringLiteral("参数变更停机"),m_robot.stop(true));}
    m_homeZPending=false;m_robot.invalidate();
}
void MainWindow::logResult(const QString& name,const QString& error) {
    if(!m_log)return;
    const auto text=QStringLiteral("[%1] %2：%3").arg(QTime::currentTime().toString("HH:mm:ss"),name,error.isEmpty()?QStringLiteral("完成"):error);
    m_log->appendPlainText(text);if(!error.isEmpty())statusBar()->showMessage(error,10000);
}
void MainWindow::stopJog() {
    if(!m_jogHeld)return;m_jogHeld=false;logResult(QStringLiteral("JOG停止"),m_robot.stop());
}
void MainWindow::poll() {
    if(m_closing)return;
    // Covers lost mouse capture as well as the normal QPushButton::released path.
    if(m_jogHeld && !(QApplication::mouseButtons()&Qt::LeftButton))stopJog();
    const bool connected=m_robot.connected();m_connect->setEnabled(!connected);m_disconnect->setEnabled(connected);m_load->setEnabled(!connected);
#ifdef _WIN32
    m_host->setEnabled(!connected);m_address->setEnabled(!connected);
#endif
    Status s;QString error;if(!connected)m_ioState->setText("DI — / DO —");
    if(!connected)m_pollError.clear();
    if(connected){error=m_pollError.isEmpty()?m_robot.readStatus(s):m_pollError;if(!error.isEmpty())m_pollError=error;}
    if(connected && (!error.isEmpty()||s.estop)) {
        const QString fault=error.isEmpty()?QStringLiteral("急停输入有效"):error;
        if(!m_fault) {
            const auto stopped=m_robot.stop(true);m_robot.invalidate();m_jogHeld=false;m_fault=true;
            logResult(QStringLiteral("故障停机"),fault+(stopped.isEmpty()?QString{}:QStringLiteral("；停止失败：")+stopped));
        }
        if(m_lastFault!=fault){statusBar()->showMessage(fault,10000);m_lastFault=fault;}
    }
    if(connected && error.isEmpty() && !m_fault) {
        bool servoAlarm=false,alarm=false;
        for(const auto& a:s.axes){servoAlarm=servoAlarm||a.servoAlarm;alarm=alarm||a.alarm;}
        if(servoAlarm || (alarm && m_robot.busy() && !m_robot.homing() && !m_jogHeld)) {
            const auto stopped=m_robot.stop(true);m_robot.invalidate();m_jogHeld=false;m_fault=true;
            logResult(QStringLiteral("轴报警停机"),stopped.isEmpty()?QStringLiteral("请检查报警/限位并清除后重新下发参数"):stopped);
        }
    }
    if(connected&&error.isEmpty()&&!m_fault)synchronizeHomeZ(s);
    if(!connected||m_fault||(!m_robot.homing()&&!m_robot.homed()))m_homeZPending=false;
    for(auto* panel:m_settingsPanels)panel->setEnabled(!m_robot.busy()&&!s.running()&&!m_jogHeld);
    m_points->setEnabled(!m_robot.busy()&&!m_jogHeld);
    m_connectionState->setText(QString("控制器：%1  |  心跳：%2").arg(connected&&error.isEmpty()?"已连接":"未连接",connected&&error.isEmpty()?QString::number(s.ticks):QStringLiteral("—")));
    m_state->setText(!connected?QStringLiteral("请点击左上角“打开控制器”"):
        QStringLiteral("%1").arg(
        m_fault?QStringLiteral("故障锁定，请检查后重新下发参数"):!m_robot.configured()?QStringLiteral("待下发参数"):
        !m_robot.initialized()?QStringLiteral("待初始化"):m_robot.homing()?QStringLiteral("回零指令已发送，等待本次回零确认"):m_robot.busy()?QStringLiteral("运动中"):
        m_robot.homed()?QStringLiteral("已回零，就绪"):QStringLiteral("待回零")));
    const bool valid=connected&&error.isEmpty();
    int enabledAxes=0;for(const auto& axis:s.axes)if(axis.enabled)++enabledAxes;
    m_enableServos->setEnabled(valid&&enabledAxes<4);
    m_disableServos->setEnabled(valid&&enabledAxes>0);
    const auto servoState=valid?QString("当前 %1/4 轴已使能").arg(enabledAxes):QStringLiteral("使能状态不可用");
    m_enableServos->setToolTip(servoState);m_disableServos->setToolTip(servoState);
    m_position->setText(valid&&s.positionValid?QStringLiteral("当前空间坐标（XYZR）：X %1 mm  Y %2 mm  Z %3 mm  R %4°").arg(s.position[0],0,'f',3).arg(s.position[1],0,'f',3).arg(s.position[2],0,'f',3).arg(s.position[3],0,'f',3):QStringLiteral("当前空间坐标（XYZR）：—（未初始化、未回零或坐标不可用）"));
    m_position->setToolTip(m_position->text()+(s.pulsePose?QStringLiteral("\nM800：由实时指令脉冲正解计算，非编码器实测位置"):QString{}));
    for(int i=0;i<4;++i){const auto& a=s.axes[i];const QStringList values={QString::number(i),QString("%1 / %2").arg(a.pulses).arg(a.encoder),s.positionValid?QString::number(a.angle,'f',3):QStringLiteral("—"),QString::number(a.speed,'f',3),stateText(a.enabled),stateText(a.homed),stateText(a.running||a.homing),stateText(a.origin),QString("%1 / %2").arg(stateText(a.positiveLimit),stateText(a.negativeLimit)),stateText(a.alarm)};for(int c=1;c<10;++c)m_statusTable->item(i,c)->setText(valid?values[c]:QStringLiteral("—"));}
}
bool MainWindow::eventFilter(QObject* obj,QEvent* event) {
    if(m_closing)return QMainWindow::eventFilter(obj,event);
    if(event->type()==QEvent::Wheel&&(qobject_cast<QAbstractSpinBox*>(obj)||qobject_cast<QComboBox*>(obj)))return true;
    if(event->type()==QEvent::ApplicationDeactivate || (event->type()==QEvent::WindowDeactivate&&obj==this))stopJog();
    if(event->type()==QEvent::MouseButtonRelease && static_cast<QMouseEvent*>(event)->button()==Qt::LeftButton)stopJog();
    if(event->type()==QEvent::KeyPress&&static_cast<QKeyEvent*>(event)->key()==Qt::Key_Escape){m_jogHeld=false;logResult(QStringLiteral("Esc停止"),m_robot.stop());return true;}
    return QMainWindow::eventFilter(obj,event);
}
void MainWindow::save() {
    logResult(QStringLiteral("保存参数"),settings().save(QCoreApplication::applicationDirPath()+"/DeltaDemo.json"));
}
void MainWindow::axisChanged(int field,int axis) {
    if(m_loading)return;
    const QString key=axisFields()[field].key;
    if(key=="angle"||key=="pulses"){changed();return;}
    m_homeZPending=false;
    auto* editor=m_axisEditors[field][axis];
    const auto error=m_robot.applyAxisParameter(axis,field,settings().axes[axis]);
    editor->setStyleSheet(error.isEmpty()?QString{}:QStringLiteral("background:#fff0cf;"));
    editor->setToolTip(error.isEmpty()?QStringLiteral("已写入控制卡；保存参数文件可保留到下次启动"):error+QStringLiteral("；此值尚未生效，可重新编辑或点击下发参数重试"));
    if(!error.isEmpty())m_robot.invalidate();
    logResult(QString("轴%1 %2 自动下发").arg(axis).arg(QString::fromUtf8(axisFields()[field].label)),error);
}
void MainWindow::load() {
    if(m_robot.connected())return;
    if(QMessageBox::question(this,"读取参数文件","是否读取参数文件",QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;
    Settings s;const auto e=s.load(QCoreApplication::applicationDirPath()+"/DeltaDemo.json");if(e.isEmpty())setSettings(s);logResult(QStringLiteral("读取参数"),e);
}
void MainWindow::closeEvent(QCloseEvent* event) {
    if(m_closing){event->ignore();return;}
    // Pause polling even while the confirmation is open, so disconnect errors do
    // not queue more blocking reads before the operator can confirm exit.
    m_timer->stop();
    if(QMessageBox::question(this,"关闭软件","是否关闭软件？\n退出时尝试停止并关闭控制器，最多等待3秒。\n通信中断时无法确认机械已停稳，请使用设备硬件停止。",QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes){m_timer->start();event->ignore();return;}
    m_closing=true;m_jogHeld=false;centralWidget()->setEnabled(false);
    statusBar()->showMessage("正在停止并关闭控制器…");
    QEventLoop wait;QString error;
    auto* worker=QThread::create([this,&error]{error=m_robot.closeForExit();});
    connect(worker,&QThread::finished,&wait,&QEventLoop::quit);
    QTimer deadline;deadline.setSingleShot(true);
    // Only after confirmed application exit. Terminate the whole process rather
    // than killing an SDK thread and then accessing its possibly corrupted state.
    connect(&deadline,&QTimer::timeout,&wait,[]{std::_Exit(0);});
    deadline.start(3000);worker->start();wait.exec();deadline.stop();worker->wait();delete worker;
    if(!error.isEmpty())logResult("退出时控制器未确认停机",error);
    event->accept();
}
}
