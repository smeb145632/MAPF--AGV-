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
    
    // Ackermann转向模型（4轮车物理模型）
    // 将角速度转换为转向角：ω = (v * tan(δ)) / L
    // 因此：δ = atan(ω * L / v)
    double delta = 0.0;
    if (std::abs(v) > 1e-6) {
        delta = std::atan(omega * properties_.wheelbase / v);
        // 限制转向角
        delta = std::max(-properties_.maxSteeringAngle, 
                        std::min(properties_.maxSteeringAngle, delta));
    } else {
        // 速度为0时，只能原地旋转（特殊情况）
        if (std::abs(omega) > 1e-6) {
            pose_.theta += omega * dt;
            // 归一化角度
            while (pose_.theta > M_PI) pose_.theta -= 2 * M_PI;
            while (pose_.theta < -M_PI) pose_.theta += 2 * M_PI;
            addTrajectoryPoint();
            return;
        }
    }
    
    // 使用Ackermann运动学方程更新位置
    updateWithSteering(dt, v, delta);
}

void Vehicle::updateWithSteering(double dt, double v, double delta) {
    // Ackermann转向模型运动学方程（自行车模型）
    // 这是4轮车的标准物理模型
    //
    // 运动学方程：
    // ẋ = v * cos(θ)
    // ẏ = v * sin(θ)
    // θ̇ = (v * tan(δ)) / L
    //
    // 其中：
    // - v: 后轮中心速度（线速度）
    // - δ: 前轮转向角
    // - L: 轴距（wheelbase）
    // - θ: 车辆朝向角
    
    // 限制转向角
    delta = std::max(-properties_.maxSteeringAngle, 
                    std::min(properties_.maxSteeringAngle, delta));
    
    // 限制速度
    v = std::max(-properties_.maxSpeed, std::min(properties_.maxSpeed, v));
    
    // 计算角速度（从转向角推导）
    double omega = 0.0;
    if (std::abs(properties_.wheelbase) > 1e-6) {
        omega = (v * std::tan(delta)) / properties_.wheelbase;
    }
    
    // 限制角速度
    omega = std::max(-properties_.maxAngularVelocity, 
                    std::min(properties_.maxAngularVelocity, omega));
    
    // 数值积分更新位置（使用精确的圆弧运动模型）
    if (std::abs(omega) < 1e-6) {
        // 直线运动（转向角为0或速度很小）
        pose_.x += v * std::cos(pose_.theta) * dt;
        pose_.y += v * std::sin(pose_.theta) * dt;
    } else {
        // 圆弧运动（Ackermann转向）
        // 瞬时转向半径：R = L / tan(δ) = v / ω
        double R = v / omega;
        
        // 瞬时转向中心（ICR - Instantaneous Center of Rotation）
        // 在车辆坐标系中，ICR位于后轴中心的正后方
        double icr_x_local = 0.0;
        double icr_y_local = R;
        
        // 转换到世界坐标系
        double icr_x_world = pose_.x - icr_y_local * std::sin(pose_.theta);
        double icr_y_world = pose_.y + icr_y_local * std::cos(pose_.theta);
        
        // 计算角度变化
        double dtheta = omega * dt;
        
        // 绕ICR旋转
        double cos_dtheta = std::cos(dtheta);
        double sin_dtheta = std::sin(dtheta);
        
        double dx_local = pose_.x - icr_x_world;
        double dy_local = pose_.y - icr_y_world;
        
        pose_.x = icr_x_world + dx_local * cos_dtheta - dy_local * sin_dtheta;
        pose_.y = icr_y_world + dx_local * sin_dtheta + dy_local * cos_dtheta;
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
    
    // Ackermann约束：角速度与线速度的关系
    // ω_max = (v * tan(δ_max)) / L
    // 因此，给定v时，ω不能超过物理限制
    if (std::abs(v) > 1e-6 && std::abs(properties_.wheelbase) > 1e-6) {
        double maxOmegaFromSteering = (std::abs(v) * std::tan(properties_.maxSteeringAngle)) 
                                      / properties_.wheelbase;
        omega = std::max(-maxOmegaFromSteering, std::min(maxOmegaFromSteering, omega));
    }
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


