#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "../core/vehicle.h"
#include "../core/map.h"
#include <QPointF>
#include <cmath>
#include <limits>

/**
 * @brief 控制器类 - 实现4轮车控制算法
 */
class Controller {
public:
    // 算法模式枚举
    enum AlgorithmMode {
        ALGORITHM_ROTATE_THEN_MOVE = 0,  // 算法1：先旋转再前进
        ALGORITHM_MOVE_WHILE_ROTATE = 1, // 算法2：边旋转边前进
        ALGORITHM_PID_CONTROL = 2,       // 算法3：PID控制（成熟算法）
        ALGORITHM_PURE_PURSUIT = 3       // 算法4：Pure Pursuit纯跟踪算法
    };
    
    Controller();
    
    // 控制参数
    struct ControlParams {
        // 基础参数
        double kp = 2.0;           // 位置比例增益
        double ktheta = 3.0;       // 角度比例增益
        double maxLinearVel = 2.0; // 最大线速度
        double maxAngularVel = 1.0; // 最大角速度
        double goalTolerance = 0.1; // 目标点容差 (m)
        double angleTolerance = 0.05; // 角度容差 (rad)
        
        // PID控制参数
        double kp_v = 1.5;         // 线速度P增益
        double ki_v = 0.0;         // 线速度I增益
        double kd_v = 0.3;         // 线速度D增益
        double kp_omega = 2.5;     // 角速度P增益
        double ki_omega = 0.0;     // 角速度I增益
        double kd_omega = 0.5;     // 角速度D增益
        
        // Pure Pursuit参数
        double lookaheadDistance = 1.5; // 前瞻距离 (m)
        double minLookahead = 0.5;      // 最小前瞻距离
        double maxLookahead = 3.0;      // 最大前瞻距离
        
        // 运动学参数
        double wheelbase = 1.5;    // 轴距 (m)，用于Ackermann模型
    };
    
    // 设置目标点（位置和角度）
    void setGoal(double x, double y, double theta = std::numeric_limits<double>::quiet_NaN());
    void setGoal(const QPointF& goal, double theta = std::numeric_limits<double>::quiet_NaN());
    
    // 计算控制命令（返回速度和角速度）
    Vehicle::Velocity computeControl(const Vehicle& vehicle, double dt);
    
    // 检查是否到达目标
    bool isGoalReached(const Vehicle& vehicle) const;
    
    // 获取目标点
    QPointF getGoal() const { return goal_; }
    double getGoalTheta() const { return goalTheta_; }
    bool hasGoal() const { return hasGoal_; }
    bool hasGoalTheta() const { return hasGoalTheta_; }
    
    // 清除目标
    void clearGoal() { hasGoal_ = false; hasGoalTheta_ = false; }
    
    // 设置控制参数
    void setControlParams(const ControlParams& params) { params_ = params; }
    ControlParams getControlParams() const { return params_; }
    
    // 设置算法模式
    void setAlgorithmMode(AlgorithmMode mode) { algorithmMode_ = mode; }
    AlgorithmMode getAlgorithmMode() const { return algorithmMode_; }
    
private:
    QPointF goal_;
    double goalTheta_;  // 目标角度
    bool hasGoal_;
    bool hasGoalTheta_;  // 是否指定了目标角度
    ControlParams params_;
    AlgorithmMode algorithmMode_;
    
    // 计算到目标点的距离和角度
    double computeDistance(const Vehicle& vehicle) const;
    double computeAngleToGoal(const Vehicle& vehicle) const;
    
    // 归一化角度到 [-π, π]
    double normalizeAngle(double angle) const;
    
    // 算法1：先旋转再前进（改进版，考虑目标角度）
    Vehicle::Velocity computeRotateThenMove(const Vehicle& vehicle, double dt);
    
    // 算法2：边旋转边前进（考虑前进/后退选择）
    Vehicle::Velocity computeMoveWhileRotate(const Vehicle& vehicle, double dt);
    
    // 算法3：PID控制算法（成熟的双PID控制器）
    Vehicle::Velocity computePIDControl(const Vehicle& vehicle, double dt);
    
    // 算法4：Pure Pursuit纯跟踪算法
    Vehicle::Velocity computePurePursuit(const Vehicle& vehicle, double dt);
    
    // 判断应该前进还是后退
    bool shouldMoveForward(const Vehicle& vehicle) const;
    
    // PID控制器状态（用于积分和微分项）
    struct PIDState {
        double lastError_v = 0.0;      // 上次线速度误差
        double integral_v = 0.0;       // 线速度积分项
        double lastError_omega = 0.0;  // 上次角速度误差
        double integral_omega = 0.0;   // 角速度积分项
    };
    PIDState pidState_;
};

#endif // CONTROLLER_H

