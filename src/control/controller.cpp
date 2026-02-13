#include "controller.h"
#include <algorithm>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <limits>

Controller::Controller() 
    : hasGoal_(false), hasGoalTheta_(false), goal_(0, 0), goalTheta_(0.0),
      algorithmMode_(ALGORITHM_PID_CONTROL), pidState_() {}

void Controller::setGoal(double x, double y, double theta) {
    goal_ = QPointF(x, y);
    hasGoal_ = true;
    if (!std::isnan(theta) && theta == theta) {  // Check for NaN
        goalTheta_ = normalizeAngle(theta);
        hasGoalTheta_ = true;
    } else {
        hasGoalTheta_ = false;
    }
}

void Controller::setGoal(const QPointF& goal, double theta) {
    goal_ = goal;
    hasGoal_ = true;
    if (!std::isnan(theta) && theta == theta) {  // Check for NaN
        goalTheta_ = normalizeAngle(theta);
        hasGoalTheta_ = true;
    } else {
        hasGoalTheta_ = false;
    }
}

double Controller::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double Controller::computeDistance(const Vehicle& vehicle) const {
    Vehicle::Pose pose = vehicle.getPose();
    double dx = goal_.x() - pose.x;
    double dy = goal_.y() - pose.y;
    return std::sqrt(dx * dx + dy * dy);
}

double Controller::computeAngleToGoal(const Vehicle& vehicle) const {
    Vehicle::Pose pose = vehicle.getPose();
    double dx = goal_.x() - pose.x;
    double dy = goal_.y() - pose.y;
    return std::atan2(dy, dx);
}

Vehicle::Velocity Controller::computeControl(const Vehicle& vehicle, double dt) {
    if (!hasGoal_) {
        return {0.0, 0.0};
    }
    
    // 根据算法模式选择不同的控制算法
    switch (algorithmMode_) {
        case ALGORITHM_ROTATE_THEN_MOVE:
            return computeRotateThenMove(vehicle, dt);
        case ALGORITHM_MOVE_WHILE_ROTATE:
            return computeMoveWhileRotate(vehicle, dt);
        case ALGORITHM_PID_CONTROL:
            return computePIDControl(vehicle, dt);
        case ALGORITHM_PURE_PURSUIT:
            return computePurePursuit(vehicle, dt);
        default:
            return computePIDControl(vehicle, dt);
    }
}

Vehicle::Velocity Controller::computeRotateThenMove(const Vehicle& vehicle, double dt) {
    Vehicle::Pose pose = vehicle.getPose();
    double distance = computeDistance(vehicle);
    
    Vehicle::Velocity vel;
    
    // 如果距离目标很近
    if (distance < params_.goalTolerance) {
        // 如果指定了目标角度，调整到目标角度
        if (hasGoalTheta_) {
            double angleError = normalizeAngle(goalTheta_ - pose.theta);
            if (std::abs(angleError) > params_.angleTolerance) {
                vel.v = 0.0;
                vel.omega = params_.ktheta * angleError;
                vel.omega = std::max(-params_.maxAngularVel, 
                                    std::min(params_.maxAngularVel, vel.omega));
                return vel;
            }
        }
        // 到达目标
        vel.v = 0.0;
        vel.omega = 0.0;
        return vel;
    }
    
    // 计算到目标点的角度
    double angleToGoal = computeAngleToGoal(vehicle);
    double angleError = normalizeAngle(angleToGoal - pose.theta);
    
    // 如果指定了目标角度，在接近目标时需要考虑最终角度
    if (hasGoalTheta_ && distance < 2.0) {
        // 接近目标时，同时考虑位置和角度
        double finalAngleError = normalizeAngle(goalTheta_ - pose.theta);
        double positionAngleError = angleError;
        
        // 如果角度误差较大，优先调整角度
        if (std::abs(finalAngleError) > params_.angleTolerance * 2) {
            vel.v = 0.0;
            vel.omega = params_.ktheta * finalAngleError;
            vel.omega = std::max(-params_.maxAngularVel, 
                                std::min(params_.maxAngularVel, vel.omega));
            return vel;
        }
    }
    
    // 控制算法：先调整角度，再前进
    if (std::abs(angleError) > params_.angleTolerance) {
        // 角度误差较大，先调整角度
        vel.v = 0.0;
        vel.omega = params_.ktheta * angleError;
        vel.omega = std::max(-params_.maxAngularVel, 
                            std::min(params_.maxAngularVel, vel.omega));
    } else {
        // 角度正确，前进并微调
        vel.v = params_.kp * distance;
        vel.v = std::min(params_.maxLinearVel, vel.v);
        
        // 角速度用于微调方向
        vel.omega = params_.ktheta * angleError;
        vel.omega = std::max(-params_.maxAngularVel, 
                            std::min(params_.maxAngularVel, vel.omega));
    }
    
    return vel;
}

Vehicle::Velocity Controller::computeMoveWhileRotate(const Vehicle& vehicle, double dt) {
    Vehicle::Pose pose = vehicle.getPose();
    double distance = computeDistance(vehicle);
    
    Vehicle::Velocity vel;
    
    // 如果距离目标很近
    if (distance < params_.goalTolerance) {
        // 如果指定了目标角度，调整到目标角度
        if (hasGoalTheta_) {
            double angleError = normalizeAngle(goalTheta_ - pose.theta);
            if (std::abs(angleError) > params_.angleTolerance) {
                vel.v = 0.0;
                vel.omega = params_.ktheta * angleError;
                vel.omega = std::max(-params_.maxAngularVel, 
                                    std::min(params_.maxAngularVel, vel.omega));
                return vel;
            }
        }
        // 到达目标
        vel.v = 0.0;
        vel.omega = 0.0;
        return vel;
    }
    
    // 计算到目标点的角度
    double angleToGoal = computeAngleToGoal(vehicle);
    double angleError = normalizeAngle(angleToGoal - pose.theta);
    
    // 如果指定了目标角度，计算最终角度误差
    double finalAngleError = 0.0;
    if (hasGoalTheta_) {
        finalAngleError = normalizeAngle(goalTheta_ - pose.theta);
    }
    
    // 判断应该前进还是后退
    bool moveForward = shouldMoveForward(vehicle);
    
    // 边旋转边前进算法
    // 计算期望的线速度和角速度
    double desiredV = params_.kp * distance;
    desiredV = std::min(params_.maxLinearVel, desiredV);
    
    // 如果角度误差很大，降低线速度
    double angleErrorAbs = std::abs(angleError);
    if (angleErrorAbs > M_PI / 3) {  // 60度以上
        desiredV *= (1.0 - angleErrorAbs / M_PI);
    }
    
    // 如果指定了目标角度，在接近目标时考虑最终角度
    if (hasGoalTheta_ && distance < 3.0) {
        // 混合位置角度误差和最终角度误差
        double weight = distance / 3.0;  // 距离越近，越重视最终角度
        angleError = weight * angleError + (1.0 - weight) * finalAngleError;
    }
    
    // 计算角速度
    double desiredOmega = params_.ktheta * angleError;
    desiredOmega = std::max(-params_.maxAngularVel, 
                           std::min(params_.maxAngularVel, desiredOmega));
    
    // 根据前进/后退选择设置速度方向
    vel.v = moveForward ? desiredV : -desiredV;
    vel.omega = desiredOmega;
    
    return vel;
}

bool Controller::shouldMoveForward(const Vehicle& vehicle) const {
    Vehicle::Pose pose = vehicle.getPose();
    double angleToGoal = computeAngleToGoal(vehicle);
    double angleError = normalizeAngle(angleToGoal - pose.theta);
    
    // 如果角度误差在90度以内，前进；否则考虑后退
    if (std::abs(angleError) < M_PI / 2) {
        return true;
    }
    
    // 如果指定了目标角度，考虑最终角度
    if (hasGoalTheta_) {
        double finalAngleError = normalizeAngle(goalTheta_ - pose.theta);
        // 如果最终角度与当前位置角度接近，可能后退更合适
        double backwardAngleError = normalizeAngle(angleToGoal - (pose.theta + M_PI));
        if (std::abs(backwardAngleError) < std::abs(angleError)) {
            return false;
        }
    }
    
    // 默认前进
    return true;
}

Vehicle::Velocity Controller::computePIDControl(const Vehicle& vehicle, double dt) {
    Vehicle::Pose pose = vehicle.getPose();
    double distance = computeDistance(vehicle);
    
    Vehicle::Velocity vel;
    
    // 如果距离目标很近
    if (distance < params_.goalTolerance) {
        // 如果指定了目标角度，调整到目标角度
        if (hasGoalTheta_) {
            double angleError = normalizeAngle(goalTheta_ - pose.theta);
            if (std::abs(angleError) > params_.angleTolerance) {
                // 使用PID控制角度
                double error_omega = angleError;
                pidState_.integral_omega += error_omega * dt;
                double derivative_omega = (error_omega - pidState_.lastError_omega) / dt;
                
                vel.v = 0.0;
                vel.omega = params_.kp_omega * error_omega + 
                           params_.ki_omega * pidState_.integral_omega +
                           params_.kd_omega * derivative_omega;
                vel.omega = std::max(-params_.maxAngularVel, 
                                    std::min(params_.maxAngularVel, vel.omega));
                
                pidState_.lastError_omega = error_omega;
                return vel;
            }
        }
        // 到达目标，重置PID状态
        pidState_ = PIDState();
        vel.v = 0.0;
        vel.omega = 0.0;
        return vel;
    }
    
    // PID控制算法公式：
    // 1. 计算位置误差和目标速度
    double error_v = distance;  // 位置误差作为速度误差
    pidState_.integral_v += error_v * dt;
    double derivative_v = (error_v - pidState_.lastError_v) / dt;
    
    // 线速度PID控制：v = Kp*e + Ki*∫e*dt + Kd*de/dt
    double desiredV = params_.kp_v * error_v + 
                      params_.ki_v * pidState_.integral_v +
                      params_.kd_v * derivative_v;
    desiredV = std::max(0.0, std::min(params_.maxLinearVel, desiredV));
    
    // 2. 计算角度误差
    double angleToGoal = computeAngleToGoal(vehicle);
    double angleError = normalizeAngle(angleToGoal - pose.theta);
    
    // 如果指定了目标角度，在接近目标时混合位置角度和最终角度
    if (hasGoalTheta_ && distance < 3.0) {
        double finalAngleError = normalizeAngle(goalTheta_ - pose.theta);
        double weight = distance / 3.0;
        angleError = weight * angleError + (1.0 - weight) * finalAngleError;
    }
    
    // 角速度PID控制：ω = Kp*e + Ki*∫e*dt + Kd*de/dt
    double error_omega = angleError;
    pidState_.integral_omega += error_omega * dt;
    double derivative_omega = (error_omega - pidState_.lastError_omega) / dt;
    
    double desiredOmega = params_.kp_omega * error_omega + 
                         params_.ki_omega * pidState_.integral_omega +
                         params_.kd_omega * derivative_omega;
    desiredOmega = std::max(-params_.maxAngularVel, 
                           std::min(params_.maxAngularVel, desiredOmega));
    
    // 3. 根据角度误差调整线速度（角度误差大时减速）
    double angleErrorAbs = std::abs(angleError);
    if (angleErrorAbs > M_PI / 4) {  // 45度以上
        desiredV *= std::cos(angleErrorAbs);  // 使用cos函数平滑减速
    }
    
    // 4. 判断前进/后退
    bool moveForward = shouldMoveForward(vehicle);
    if (!moveForward) {
        desiredV = -desiredV;
    }
    
    vel.v = desiredV;
    vel.omega = desiredOmega;
    
    // 更新PID状态
    pidState_.lastError_v = error_v;
    pidState_.lastError_omega = error_omega;
    
    return vel;
}

Vehicle::Velocity Controller::computePurePursuit(const Vehicle& vehicle, double dt) {
    Vehicle::Pose pose = vehicle.getPose();
    double distance = computeDistance(vehicle);
    
    Vehicle::Velocity vel;
    
    // 如果距离目标很近
    if (distance < params_.goalTolerance) {
        // 如果指定了目标角度，调整到目标角度
        if (hasGoalTheta_) {
            double angleError = normalizeAngle(goalTheta_ - pose.theta);
            if (std::abs(angleError) > params_.angleTolerance) {
                vel.v = 0.0;
                vel.omega = params_.ktheta * angleError;
                vel.omega = std::max(-params_.maxAngularVel, 
                                    std::min(params_.maxAngularVel, vel.omega));
                return vel;
            }
        }
        vel.v = 0.0;
        vel.omega = 0.0;
        return vel;
    }
    
    // Pure Pursuit算法公式：
    // 1. 计算前瞻点（lookahead point）
    //    前瞻距离根据速度动态调整：ld = k * v + ld_min
    double currentV = std::min(params_.maxLinearVel, params_.kp * distance);
    double lookaheadDist = params_.lookaheadDistance;
    if (currentV > 0.1) {
        lookaheadDist = std::max(params_.minLookahead, 
                                 std::min(params_.maxLookahead, 
                                         lookaheadDist * (1.0 + currentV / params_.maxLinearVel)));
    }
    
    // 2. 计算到目标的向量
    double dx = goal_.x() - pose.x;
    double dy = goal_.y() - pose.y;
    double distToGoal = std::sqrt(dx * dx + dy * dy);
    
    // 3. 如果目标在前瞻距离内，直接指向目标
    QPointF lookaheadPoint;
    if (distToGoal <= lookaheadDist) {
        lookaheadPoint = goal_;
    } else {
        // 前瞻点在从当前位置指向目标方向的直线上
        double ratio = lookaheadDist / distToGoal;
        lookaheadPoint = QPointF(pose.x + dx * ratio, pose.y + dy * ratio);
    }
    
    // 4. 计算到前瞻点的横向误差（lateral error）
    double dx_look = lookaheadPoint.x() - pose.x;
    double dy_look = lookaheadPoint.y() - pose.y;
    double alpha = normalizeAngle(std::atan2(dy_look, dx_look) - pose.theta);
    
    // 5. Pure Pursuit控制律：
    //    v = 常数（或根据距离调整）
    //    ω = (2*v*sin(α)) / ld
    //    其中：α是前瞻点与车辆朝向的夹角，ld是前瞻距离
    
    double desiredV = params_.kp * distance;
    desiredV = std::min(params_.maxLinearVel, desiredV);
    
    // 如果角度误差很大，降低速度
    if (std::abs(alpha) > M_PI / 3) {
        desiredV *= (1.0 - std::abs(alpha) / M_PI);
    }
    
    // Pure Pursuit角速度公式
    double desiredOmega = 0.0;
    if (lookaheadDist > 0.01 && desiredV > 0.01) {
        desiredOmega = (2.0 * desiredV * std::sin(alpha)) / lookaheadDist;
    } else {
        // 如果前瞻距离太小或速度为0，使用简单的角度控制
        desiredOmega = params_.ktheta * alpha;
    }
    
    desiredOmega = std::max(-params_.maxAngularVel, 
                           std::min(params_.maxAngularVel, desiredOmega));
    
    // 6. 判断前进/后退
    bool moveForward = shouldMoveForward(vehicle);
    if (!moveForward) {
        desiredV = -desiredV;
        // 后退时角速度也需要反向
        desiredOmega = -desiredOmega;
    }
    
    vel.v = desiredV;
    vel.omega = desiredOmega;
    
    return vel;
}

bool Controller::isGoalReached(const Vehicle& vehicle) const {
    if (!hasGoal_) return false;
    
    Vehicle::Pose pose = vehicle.getPose();
    double distance = computeDistance(vehicle);
    
    // 检查位置是否到达
    if (distance >= params_.goalTolerance) {
        return false;
    }
    
    // 如果指定了目标角度，检查角度是否到达
    if (hasGoalTheta_) {
        double angleError = std::abs(normalizeAngle(goalTheta_ - pose.theta));
        return angleError < params_.angleTolerance;
    }
    
    return true;
}

