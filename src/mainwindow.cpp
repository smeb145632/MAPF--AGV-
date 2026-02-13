#include "mainwindow.h"
#include <QApplication>
#include <QMessageBox>
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
    algorithmCombo_->setCurrentIndex(1);  // 默认使用边旋转边前进
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
    controller_.setAlgorithmMode(Controller::ALGORITHM_MOVE_WHILE_ROTATE);
    
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

