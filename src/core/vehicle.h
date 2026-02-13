#ifndef VEHICLE_H
#define VEHICLE_H

#include <QPointF>
#include <QVector>
#include <QColor>
#include <cmath>

/**
 * @brief 车辆类 - 表示4轮AGV
 */
class Vehicle {
public:
    Vehicle();
    Vehicle(double x, double y, double theta);
    
    // 车辆属性
    struct VehicleProperties {
        double length = 2.0;        // 车身长度 (m)
        double width = 1.0;         // 车身宽度 (m)
        double wheelbase = 1.5;     // 轴距 (m) - Ackermann模型关键参数
        double maxSpeed = 2.0;      // 最大速度 (m/s)
        double maxAngularVelocity = 1.0; // 最大角速度 (rad/s)
        double maxSteeringAngle = 0.5236; // 最大转向角 (rad) = 30度
        double wheelRadius = 0.3;   // 轮子半径 (m)
    };
    
    // 位置和姿态
    struct Pose {
        double x;      // X坐标 (m)
        double y;      // Y坐标 (m)
        double theta;  // 朝向角度 (rad)
    };
    
    // 速度控制
    struct Velocity {
        double v;      // 线速度 (m/s)
        double omega;  // 角速度 (rad/s)
    };
    
    // Getters
    Pose getPose() const { return pose_; }
    VehicleProperties getProperties() const { return properties_; }
    QVector<QPointF> getTrajectory() const { return trajectory_; }
    QColor getColor() const { return color_; }
    
    // Setters
    void setPose(double x, double y, double theta);
    void setColor(const QColor& color) { color_ = color; }
    
    // 运动学更新（基于速度模型）
    // 使用Ackermann转向模型（4轮车物理模型）
    void update(double dt, double v, double omega);
    
    // 运动学更新（基于转向角模型，更符合物理）
    // delta: 前轮转向角 (rad)
    void updateWithSteering(double dt, double v, double delta);
    
    // 获取车辆四个角的坐标（用于绘制）
    QVector<QPointF> getCorners() const;
    
    // 添加轨迹点
    void addTrajectoryPoint();
    
    // 清除轨迹
    void clearTrajectory() { trajectory_.clear(); }
    
private:
    Pose pose_;
    VehicleProperties properties_;
    QVector<QPointF> trajectory_;  // 轨迹点
    QColor color_;
    
    // 限制速度在合理范围内
    void limitVelocity(double& v, double& omega);
};

#endif // VEHICLE_H


