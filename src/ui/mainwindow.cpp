#include "mainwindow.h"
#include <QApplication>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QKeySequence>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <limits>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent) {
    
    // 设置窗口标题和大小
    setWindowTitle("AGV调度控制界面");
    resize(1200, 800);
    
    // 创建菜单栏
    setupMenuBar();
    
    // 创建中央部件
    centralWidget_ = new QWidget(this);
    setCentralWidget(centralWidget_);
    
    // 创建主布局
    mainLayout_ = new QVBoxLayout(centralWidget_);
    
    // 创建地图绘制区域
    mapWidget_ = new MapWidget(this);
    mapWidget_->setVehicle(&vehicle_);
    mapWidget_->setMap(&map_);
    mapWidget_->setController(&controller_);
    mapWidget_->setMapData(&mapData_);
    connect(mapWidget_, &MapWidget::mouseClicked, this, [this](const QPointF& worldPos, Qt::MouseButton button) {
        QPointF clampedPos = map_.clampToMap(worldPos);
        if (button == Qt::LeftButton) {
            double goalTheta = std::numeric_limits<double>::quiet_NaN();
            double thetaValue = goalThetaSpin_->value();
            if (thetaValue > -900.0) {
                goalTheta = thetaValue * M_PI / 180.0;
            }
            controller_.setGoal(clampedPos, goalTheta);
            updateStatus();
        } else if (button == Qt::RightButton) {
            Vehicle::Pose pose = vehicle_.getPose();
            double dx = clampedPos.x() - pose.x;
            double dy = clampedPos.y() - pose.y;
            double angle = std::atan2(dy, dx) * 180.0 / M_PI;
            goalThetaSpin_->setValue(angle);
            if (controller_.hasGoal()) {
                controller_.setGoal(controller_.getGoal(), angle * M_PI / 180.0);
            } else {
                controller_.setGoal(clampedPos, angle * M_PI / 180.0);
            }
            updateStatus();
        }
    });
    mainLayout_->addWidget(mapWidget_, 1);
    
    // 创建控制面板
    controlPanel_ = new QGroupBox("控制面板", this);
    controlLayout_ = new QHBoxLayout(controlPanel_);
    
    // 清除轨迹按钮
    clearTrajectoryBtn_ = new QPushButton("清除轨迹", this);
    connect(clearTrajectoryBtn_, &QPushButton::clicked, this, &MainWindow::onClearTrajectory);
    controlLayout_->addWidget(clearTrajectoryBtn_);
    
    // 重置车辆按钮
    resetVehicleBtn_ = new QPushButton("重置车辆", this);
    connect(resetVehicleBtn_, &QPushButton::clicked, this, &MainWindow::onResetVehicle);
    controlLayout_->addWidget(resetVehicleBtn_);
    
    // 地图大小设置
    controlLayout_->addWidget(new QLabel("地图宽度:", this));
    mapWidthSpin_ = new QDoubleSpinBox(this);
    mapWidthSpin_->setRange(10.0, 200.0);
    mapWidthSpin_->setValue(50.0);
    mapWidthSpin_->setSuffix(" m");
    controlLayout_->addWidget(mapWidthSpin_);
    
    controlLayout_->addWidget(new QLabel("地图高度:", this));
    mapHeightSpin_ = new QDoubleSpinBox(this);
    mapHeightSpin_->setRange(10.0, 200.0);
    mapHeightSpin_->setValue(50.0);
    mapHeightSpin_->setSuffix(" m");
    controlLayout_->addWidget(mapHeightSpin_);
    
    setMapSizeBtn_ = new QPushButton("设置地图大小", this);
    connect(setMapSizeBtn_, &QPushButton::clicked, this, &MainWindow::onSetMapSize);
    controlLayout_->addWidget(setMapSizeBtn_);
    
    // 算法选择
    controlLayout_->addWidget(new QLabel("算法:", this));
    algorithmCombo_ = new QComboBox(this);
    algorithmCombo_->addItem("先旋转再前进", Controller::ALGORITHM_ROTATE_THEN_MOVE);
    algorithmCombo_->addItem("边旋转边前进", Controller::ALGORITHM_MOVE_WHILE_ROTATE);
    algorithmCombo_->addItem("PID控制", Controller::ALGORITHM_PID_CONTROL);
    algorithmCombo_->addItem("Pure Pursuit", Controller::ALGORITHM_PURE_PURSUIT);
    algorithmCombo_->setCurrentIndex(2);  // 默认使用PID控制
    connect(algorithmCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onAlgorithmChanged);
    controlLayout_->addWidget(algorithmCombo_);
    
    // 目标角度设置（使用-999作为"未设置"的特殊值）
    controlLayout_->addWidget(new QLabel("目标角度:", this));
    goalThetaSpin_ = new QDoubleSpinBox(this);
    goalThetaSpin_->setRange(-999.0, 180.0);
    goalThetaSpin_->setValue(-999.0);  // -999表示未设置
    goalThetaSpin_->setSuffix("°");
    goalThetaSpin_->setWrapping(false);
    goalThetaSpin_->setSpecialValueText("自动");
    controlLayout_->addWidget(goalThetaSpin_);
    
    controlLayout_->addStretch();
    
    // 状态标签
    statusLabel_ = new QLabel("状态: 就绪", this);
    controlLayout_->addWidget(statusLabel_);
    
    mainLayout_->addWidget(controlPanel_);
    
    // 初始化车辆（在地图中心）
    vehicle_.setPose(25.0, 25.0, 0.0);
    vehicle_.setColor(Qt::blue);
    
    // 初始化地图
    map_.setSize(50.0, 50.0);
    
    // 初始化控制器算法模式
    controller_.setAlgorithmMode(Controller::ALGORITHM_PID_CONTROL);
    
    // 创建地图编辑器
    mapEditor_ = new MapEditor(this);
    connect(mapEditor_, &MapEditor::mapDataChanged, this, &MainWindow::onMapDataChanged);
    
    // 创建定时器
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::updateSimulation);
    timer_->start(50);  // 20 FPS
    
    // 更新状态
    updateStatus();
}

MainWindow::~MainWindow() {}

void MainWindow::paintEvent(QPaintEvent* event) {
    QMainWindow::paintEvent(event);
    // 绘制由 MapWidget 处理
}

void MainWindow::mousePressEvent(QMouseEvent* event) {
    QMainWindow::mousePressEvent(event);
    // 鼠标事件由 MapWidget 处理
}

void MainWindow::mouseMoveEvent(QMouseEvent* event) {
    QMainWindow::mouseMoveEvent(event);
    // 鼠标事件由 MapWidget 处理
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    // 重绘由 MapWidget 处理
}

void MainWindow::updateSimulation() {
    if (controller_.hasGoal() && !controller_.isGoalReached(vehicle_)) {
        // 计算控制命令
        Vehicle::Velocity vel = controller_.computeControl(vehicle_, 0.05);
        
        // 更新车辆状态
        vehicle_.update(0.05, vel.v, vel.omega);
        
        // 检查是否到达目标
        if (controller_.isGoalReached(vehicle_)) {
            controller_.clearGoal();
        }
    }
    
    updateStatus();
    mapWidget_->update();  // 触发地图重绘
}

void MainWindow::onClearTrajectory() {
    vehicle_.clearTrajectory();
    vehicle_.addTrajectoryPoint();  // 保留当前位置
    mapWidget_->update();
}

void MainWindow::onResetVehicle() {
    Map::MapProperties mapProps = map_.getProperties();
    vehicle_.setPose(mapProps.width / 2.0, mapProps.height / 2.0, 0.0);
    vehicle_.clearTrajectory();
    vehicle_.addTrajectoryPoint();
    controller_.clearGoal();
    mapWidget_->update();
}

void MainWindow::onSetMapSize() {
    double width = mapWidthSpin_->value();
    double height = mapHeightSpin_->value();
    map_.setSize(width, height);
    
    // 调整车辆位置到新地图中心
    vehicle_.setPose(width / 2.0, height / 2.0, vehicle_.getPose().theta);
    mapWidget_->update();
}

void MainWindow::onAlgorithmChanged(int index) {
    Controller::AlgorithmMode mode = static_cast<Controller::AlgorithmMode>(
        algorithmCombo_->itemData(index).toInt());
    controller_.setAlgorithmMode(mode);
    updateStatus();
}

void MainWindow::onOpenMapEditor() {
    mapEditor_->setMapData(mapData_);
    if (mapEditor_->exec() == QDialog::Accepted) {
        mapData_ = mapEditor_->getMapData();
        // 更新地图显示
        mapWidget_->setMapData(&mapData_);
    }
}

void MainWindow::onMapDataChanged(const MapData& data) {
    mapData_ = data;
    // 更新地图显示
    mapWidget_->setMapData(&mapData_);
}

void MainWindow::updateStatus() {
    Vehicle::Pose pose = vehicle_.getPose();
    QString status = QString("状态: X=%1 m, Y=%2 m, θ=%3°")
                        .arg(pose.x, 0, 'f', 2)
                        .arg(pose.y, 0, 'f', 2)
                        .arg(pose.theta * 180.0 / M_PI, 0, 'f', 2);
    
    // 显示当前算法
    QString algorithmName = algorithmCombo_->currentText();
    status += QString(" | 算法: %1").arg(algorithmName);
    
    if (controller_.hasGoal()) {
        QPointF goal = controller_.getGoal();
        status += QString(" | 目标: (%1, %2)")
                    .arg(goal.x(), 0, 'f', 2)
                    .arg(goal.y(), 0, 'f', 2);
        if (controller_.hasGoalTheta()) {
            status += QString(" θ=%1°")
                        .arg(controller_.getGoalTheta() * 180.0 / M_PI, 0, 'f', 1);
        }
    } else {
        status += " | 无目标";
    }
    
    statusLabel_->setText(status);
}

void MainWindow::setupMenuBar() {
    // 创建菜单栏
    menuBar_ = menuBar();
    
    // 文件菜单
    fileMenu_ = menuBar_->addMenu("文件(&F)");
    
    openMapAction_ = new QAction("打开地图(&O)", this);
    openMapAction_->setShortcut(QKeySequence::Open);
    openMapAction_->setStatusTip("打开地图文件");
    connect(openMapAction_, &QAction::triggered, this, [this]() {
        QString filePath = QFileDialog::getOpenFileName(this, "打开地图", 
                                                         mapEditor_->getCurrentFilePath().isEmpty() ? "." : mapEditor_->getCurrentFilePath(),
                                                         "地图文件 (*.json *.xml *.map);;JSON文件 (*.json);;XML文件 (*.xml);;二进制文件 (*.map);;所有文件 (*.*)");
        if (!filePath.isEmpty()) {
            bool success = false;
            QString ext = QFileInfo(filePath).suffix().toLower();
            if (ext == "json") {
                success = mapData_.loadFromJson(filePath);
            } else if (ext == "xml") {
                success = mapData_.loadFromXml(filePath);
            } else if (ext == "map") {
                success = mapData_.loadFromBinary(filePath);
            } else {
                // 尝试自动检测
                success = mapData_.loadFromJson(filePath) || 
                         mapData_.loadFromXml(filePath) || 
                         mapData_.loadFromBinary(filePath);
            }
            if (success) {
                mapEditor_->setCurrentFilePath(filePath);
                mapEditor_->setMapData(mapData_);
                mapWidget_->setMapData(&mapData_);
                QMessageBox::information(this, "成功", "地图加载成功！");
            } else {
                QMessageBox::warning(this, "错误", "地图加载失败！");
            }
        }
    });
    fileMenu_->addAction(openMapAction_);
    
    saveMapAction_ = new QAction("保存地图(&S)", this);
    saveMapAction_->setShortcut(QKeySequence::Save);
    saveMapAction_->setStatusTip("保存当前地图");
    connect(saveMapAction_, &QAction::triggered, this, [this]() {
        QString filePath = mapEditor_->getCurrentFilePath();
        if (filePath.isEmpty()) {
            filePath = QFileDialog::getSaveFileName(this, "保存地图", "map.json",
                                                     "地图文件 (*.json *.xml *.map);;JSON文件 (*.json);;XML文件 (*.xml);;二进制文件 (*.map)");
        }
        if (!filePath.isEmpty()) {
            bool success = false;
            QString ext = QFileInfo(filePath).suffix().toLower();
            if (ext == "json") {
                success = mapData_.saveToJson(filePath);
            } else if (ext == "xml") {
                success = mapData_.saveToXml(filePath);
            } else if (ext == "map") {
                success = mapData_.saveToBinary(filePath);
            } else {
                success = mapData_.saveToJson(filePath + ".json");
                filePath += ".json";
            }
            if (success) {
                mapEditor_->setCurrentFilePath(filePath);
                QMessageBox::information(this, "成功", "地图保存成功！");
            } else {
                QMessageBox::warning(this, "错误", "地图保存失败！");
            }
        }
    });
    fileMenu_->addAction(saveMapAction_);
    
    QAction* saveAsAction = new QAction("另存为(&A)...", this);
    saveAsAction->setShortcut(QKeySequence::SaveAs);
    saveAsAction->setStatusTip("另存为地图文件");
    connect(saveAsAction, &QAction::triggered, this, [this]() {
        QString filePath = QFileDialog::getSaveFileName(this, "另存为地图", "map.json",
                                                         "地图文件 (*.json *.xml *.map);;JSON文件 (*.json);;XML文件 (*.xml);;二进制文件 (*.map)");
        if (!filePath.isEmpty()) {
            bool success = false;
            QString ext = QFileInfo(filePath).suffix().toLower();
            if (ext == "json") {
                success = mapData_.saveToJson(filePath);
            } else if (ext == "xml") {
                success = mapData_.saveToXml(filePath);
            } else if (ext == "map") {
                success = mapData_.saveToBinary(filePath);
            } else {
                success = mapData_.saveToJson(filePath + ".json");
                filePath += ".json";
            }
            if (success) {
                mapEditor_->setCurrentFilePath(filePath);
                QMessageBox::information(this, "成功", "地图保存成功！");
            } else {
                QMessageBox::warning(this, "错误", "地图保存失败！");
            }
        }
    });
    fileMenu_->addAction(saveAsAction);
    
    fileMenu_->addSeparator();
    
    QAction* exitAction = new QAction("退出(&X)", this);
    exitAction->setShortcut(QKeySequence::Quit);
    exitAction->setStatusTip("退出程序");
    connect(exitAction, &QAction::triggered, this, &QMainWindow::close);
    fileMenu_->addAction(exitAction);
    
    // 编辑菜单
    editMenu_ = menuBar_->addMenu("编辑(&E)");
    
    mapEditorAction_ = new QAction("地图编辑器(&M)", this);
    mapEditorAction_->setShortcut(QKeySequence("Ctrl+M"));
    mapEditorAction_->setStatusTip("打开地图编辑器");
    connect(mapEditorAction_, &QAction::triggered, this, &MainWindow::onOpenMapEditor);
    editMenu_->addAction(mapEditorAction_);
}

