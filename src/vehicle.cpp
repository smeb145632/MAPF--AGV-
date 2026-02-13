#include "vehicle.h"
#include <algorithm>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

Vehicle::Vehicle() : pose_({0.0, 0.0, 0.0}), color_(Qt::blue) {
    addTrajectoryPoint();
}

Vehicle::Vehicle(double x, double y, double theta) 
    : pose_({x, y, theta}), color_(Qt::blue) {
    addTrajectoryPoint();
}

void Vehicle::setPose(double x, double y, double theta) {
    pose_.x = x;
    pose_.y = y;
    pose_.theta = theta;
    addTrajectoryPoint();
}

void Vehicle::update(double dt, double v, double omega) {
    // 限制速度
    limitVelocity(v, omega);
    
    // 运动学模型：差分驱动模型（适用于4轮车）
    // 对于4轮车，可以使用类似差分驱动的模型
    if (std::abs(omega) < 1e-6) {
        // 直线运动
        pose_.x += v * std::cos(pose_.theta) * dt;
        pose_.y += v * std::sin(pose_.theta) * dt;
    } else {
        // 圆弧运动
        double radius = v / omega;
        double dtheta = omega * dt;
        pose_.x += radius * (std::sin(pose_.theta + dtheta) - std::sin(pose_.theta));
        pose_.y += radius * (-std::cos(pose_.theta + dtheta) + std::cos(pose_.theta));
        pose_.theta += dtheta;
    }
    
    // 归一化角度到 [-π, π]
    while (pose_.theta > M_PI) pose_.theta -= 2 * M_PI;
    while (pose_.theta < -M_PI) pose_.theta += 2 * M_PI;
    
    // 添加轨迹点
    addTrajectoryPoint();
}

void Vehicle::limitVelocity(double& v, double& omega) {
    // 限制线速度
    v = std::max(-properties_.maxSpeed, std::min(properties_.maxSpeed, v));
    
    // 限制角速度
    omega = std::max(-properties_.maxAngularVelocity, 
                     std::min(properties_.maxAngularVelocity, omega));
}

QVector<QPointF> Vehicle::getCorners() const {
    QVector<QPointF> corners(4);
    double halfLength = properties_.length / 2.0;
    double halfWidth = properties_.width / 2.0;
    
    // 车辆四个角的相对坐标（以车辆中心为原点，朝向为0度）
    QVector<QPointF> localCorners = {
        QPointF(halfLength, halfWidth),   // 右前
        QPointF(halfLength, -halfWidth),  // 右后
        QPointF(-halfLength, -halfWidth), // 左后
        QPointF(-halfLength, halfWidth)   // 左前
    };
    
    // 旋转和平移到世界坐标系
    double cosTheta = std::cos(pose_.theta);
    double sinTheta = std::sin(pose_.theta);
    
    for (int i = 0; i < 4; ++i) {
        double x_local = localCorners[i].x();
        double y_local = localCorners[i].y();
        
        // 旋转
        double x_rot = x_local * cosTheta - y_local * sinTheta;
        double y_rot = x_local * sinTheta + y_local * cosTheta;
        
        // 平移
        corners[i] = QPointF(pose_.x + x_rot, pose_.y + y_rot);
    }
    
    return corners;
}

void Vehicle::addTrajectoryPoint() {
    trajectory_.append(QPointF(pose_.x, pose_.y));
}


