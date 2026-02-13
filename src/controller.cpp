#include "controller.h"
#include <algorithm>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <limits>

Controller::Controller() 
    : hasGoal_(false), hasGoalTheta_(false), goal_(0, 0), goalTheta_(0.0),
      algorithmMode_(ALGORITHM_MOVE_WHILE_ROTATE) {}

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
        default:
            return computeMoveWhileRotate(vehicle, dt);
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

