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
    inAngleOnlyPhase_ = false;
    slidePhase_ = SLIDE_APPROACH;
    if (!std::isnan(theta) && theta == theta) {
        goalTheta_ = normalizeAngle(theta);
        hasGoalTheta_ = true;
    } else {
        hasGoalTheta_ = false;
    }
}

void Controller::setGoal(const QPointF& goal, double theta) {
    goal_ = goal;
    hasGoal_ = true;
    inAngleOnlyPhase_ = false;
    slidePhase_ = SLIDE_APPROACH;
    if (!std::isnan(theta) && theta == theta) {
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
    if (!hasGoal_) return {0.0, 0.0};
    
    double dist = computeDistance(vehicle);
    if (!hasGoalTheta_ || dist > params_.goalToleranceOut) {
        inAngleOnlyPhase_ = false;
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
        case ALGORITHM_STANLEY:
            return computeStanley(vehicle, dt);
        case ALGORITHM_SLIDING_MODE:
            return computeSlidingMode(vehicle, dt);
        default:
            return computePIDControl(vehicle, dt);
    }
}

Vehicle::Velocity Controller::computeRotateThenMove(const Vehicle& vehicle, double dt) {
    Vehicle::Pose pose = vehicle.getPose();
    double distance = computeDistance(vehicle);
    
    Vehicle::Velocity vel;
    
    if (distance < params_.goalTolerance) {
        if (hasGoalTheta_) {
            double angleError = normalizeAngle(goalTheta_ - pose.theta);
            if (std::abs(angleError) > params_.angleTolerance) {
                return computeAngleAdjustmentNearGoal(vehicle, dt);
            }
        }
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
    
    if (distance < params_.goalTolerance) {
        if (hasGoalTheta_) {
            double angleError = normalizeAngle(goalTheta_ - pose.theta);
            if (std::abs(angleError) > params_.angleTolerance) {
                return computeAngleAdjustmentNearGoal(vehicle, dt);
            }
        }
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
    
    double desiredV = params_.kp * distance;
    if (params_.nearGoalDistance > 1e-6 && distance < params_.nearGoalDistance) {
        double gainScale = 0.6 + 0.4 * (distance / params_.nearGoalDistance);
        desiredV *= gainScale;
    }
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
    if (params_.nearGoalDistance > 1e-6 && distance < params_.nearGoalDistance) {
        double gainScale = 0.6 + 0.4 * (distance / params_.nearGoalDistance);
        desiredOmega *= gainScale;
    }
    desiredOmega = std::max(-params_.maxAngularVel, 
                           std::min(params_.maxAngularVel, desiredOmega));
    
    // 根据前进/后退选择设置速度方向
    vel.v = moveForward ? desiredV : -desiredV;
    vel.omega = desiredOmega;
    
    return vel;
}

Vehicle::Velocity Controller::computeAngleAdjustmentNearGoal(const Vehicle& vehicle, double dt) {
    Vehicle::Pose pose = vehicle.getPose();
    Vehicle::VehicleProperties props = vehicle.getProperties();
    double distance = computeDistance(vehicle);
    double angleError = normalizeAngle(goalTheta_ - pose.theta);
    double angleErrorAbs = std::abs(angleError);
    
    // 死区：角度已足够接近，停止输出
    double deadZone = params_.angleTolerance * 1.3;  // 略大于判定到达的阈值
    if (angleErrorAbs < deadZone) {
        inAngleOnlyPhase_ = false;
        return {0.0, 0.0};
    }
    
    // 迟滞：若已漂出位置容差，退出“仅调角”阶段，交由主算法重新接近
    if (inAngleOnlyPhase_ && distance > params_.goalToleranceOut) {
        inAngleOnlyPhase_ = false;
        return {0.0, 0.0};  // 返回零，下一帧主算法会按位置误差输出
    }
    
    inAngleOnlyPhase_ = true;
    
    // 使用蠕行速度（4轮车无法原地转向）
    double creep = (props.minCreepVelocity > 1e-6) ? props.minCreepVelocity : 0.08;
    double v = (angleError > 0) ? creep : -creep;
    
    // 增益随角度误差衰减，接近目标时减弱，减少抖振
    double gainScale = 0.6 + 0.4 * std::min(angleErrorAbs / params_.anglePhaseAngleThreshold, 1.0);
    double omega = params_.kp_omega * angleError * gainScale;
    omega = std::max(-params_.maxAngularVel, std::min(params_.maxAngularVel, omega));
    
    // 速度限制：角速度与线速度需满足 Ackermann 约束
    double maxOmega = (std::abs(v) * std::tan(props.maxSteeringAngle)) / props.wheelbase;
    omega = std::max(-maxOmega, std::min(maxOmega, omega));
    
    return {v, omega};
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
    
    if (distance < params_.goalTolerance) {
        if (hasGoalTheta_) {
            double angleError = normalizeAngle(goalTheta_ - pose.theta);
            if (std::abs(angleError) > params_.angleTolerance) {
                return computeAngleAdjustmentNearGoal(vehicle, dt);
            }
        }
        pidState_ = PIDState();
        vel.v = 0.0;
        vel.omega = 0.0;
        return vel;
    }
    
    // PID控制算法（含积分抗饱和）
    double error_v = distance;
    pidState_.integral_v += error_v * dt;
    pidState_.integral_v = std::max(-PIDState::maxIntegral_v,
                                    std::min(PIDState::maxIntegral_v, pidState_.integral_v));
    double derivative_v = (error_v - pidState_.lastError_v) / std::max(dt, 1e-6);
    
    double desiredV = params_.kp_v * error_v + 
                      params_.ki_v * pidState_.integral_v +
                      params_.kd_v * derivative_v;
    if (params_.nearGoalDistance > 1e-6 && distance < params_.nearGoalDistance) {
        double gainScale = 0.6 + 0.4 * (distance / params_.nearGoalDistance);
        desiredV *= gainScale;
    }
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
    
    double error_omega = angleError;
    pidState_.integral_omega += error_omega * dt;
    pidState_.integral_omega = std::max(-PIDState::maxIntegral_omega,
                                        std::min(PIDState::maxIntegral_omega, pidState_.integral_omega));
    double derivative_omega = (error_omega - pidState_.lastError_omega) / std::max(dt, 1e-6);
    
    double desiredOmega = params_.kp_omega * error_omega + 
                         params_.ki_omega * pidState_.integral_omega +
                         params_.kd_omega * derivative_omega;
    if (params_.nearGoalDistance > 1e-6 && distance < params_.nearGoalDistance) {
        double gainScale = 0.6 + 0.4 * (distance / params_.nearGoalDistance);
        desiredOmega *= gainScale;
    }
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
    
    if (distance < params_.goalTolerance) {
        if (hasGoalTheta_) {
            double angleError = normalizeAngle(goalTheta_ - pose.theta);
            if (std::abs(angleError) > params_.angleTolerance) {
                return computeAngleAdjustmentNearGoal(vehicle, dt);
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
    if (params_.nearGoalDistance > 1e-6 && distance < params_.nearGoalDistance) {
        double gainScale = 0.6 + 0.4 * (distance / params_.nearGoalDistance);
        desiredV *= gainScale;
    }
    desiredV = std::min(params_.maxLinearVel, desiredV);
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

Vehicle::Velocity Controller::computeStanley(const Vehicle& vehicle, double dt) {
    Vehicle::Pose pose = vehicle.getPose();
    Vehicle::VehicleProperties props = vehicle.getProperties();
    double distance = computeDistance(vehicle);
    
    Vehicle::Velocity vel;
    
    if (distance < params_.goalTolerance) {
        if (hasGoalTheta_) {
            double angleError = normalizeAngle(goalTheta_ - pose.theta);
            if (std::abs(angleError) > params_.angleTolerance) {
                return computeAngleAdjustmentNearGoal(vehicle, dt);
            }
        }
        vel.v = 0.0;
        vel.omega = 0.0;
        return vel;
    }
    
    double angleToGoal = computeAngleToGoal(vehicle);
    double angleError = normalizeAngle(angleToGoal - pose.theta);
    
    if (hasGoalTheta_ && distance < 3.0) {
        double finalAngleError = normalizeAngle(goalTheta_ - pose.theta);
        double w = distance / 3.0;
        angleError = w * angleError + (1.0 - w) * finalAngleError;
    }
    
    // Stanley：δ = θ_e + atan(k·e/v)，e为横向误差
    // 接近目标时降低增益，平滑过渡，减少抖振
    double gainScale = 1.0;
    if (params_.nearGoalDistance > 1e-6 && distance < params_.nearGoalDistance) {
        gainScale = 0.6 + 0.4 * (distance / params_.nearGoalDistance);
    }
    double desiredV = params_.kp * distance * gainScale;
    desiredV = std::min(params_.maxLinearVel, desiredV);
    double v = std::max(desiredV, 0.15);
    
    double e = distance * std::sin(angleError);
    double delta = angleError + std::atan(params_.stanleyK * e / v);
    double maxDelta = props.maxSteeringAngle;
    delta = std::max(-maxDelta, std::min(maxDelta, delta));
    double wheelbase = props.wheelbase > 1e-6 ? props.wheelbase : params_.wheelbase;
    double desiredOmega = (desiredV * std::tan(delta)) / wheelbase;
    
    desiredOmega = std::max(-params_.maxAngularVel,
                           std::min(params_.maxAngularVel, desiredOmega));
    
    bool moveForward = shouldMoveForward(vehicle);
    vel.v = moveForward ? desiredV : -desiredV;
    vel.omega = moveForward ? desiredOmega : -desiredOmega;
    
    return vel;
}

Vehicle::Velocity Controller::computeSlidingMode(const Vehicle& vehicle, double dt) {
    Vehicle::Pose pose = vehicle.getPose();
    Vehicle::VehicleProperties props = vehicle.getProperties();
    double rho = computeDistance(vehicle);
    double alpha = normalizeAngle(computeAngleToGoal(vehicle) - pose.theta);
    double thetaE = hasGoalTheta_ ? normalizeAngle(goalTheta_ - pose.theta) : 0.0;
    
    const double posTol = params_.slidePosTol;
    const double angTol = params_.slideAngTol;
    const double backOffDist = params_.slideBackOffDist;
    const double driftTol = params_.slideDriftTol;
    const double turnThresh = params_.slideTurnAngleThresh;
    const double creep = (props.minCreepVelocity > 1e-6) ? props.minCreepVelocity : 0.1;
    const double L = props.wheelbase > 1e-6 ? props.wheelbase : params_.wheelbase;
    
    Vehicle::Velocity vel = {0.0, 0.0};
    
    if (!hasGoalTheta_) {
        slidePhase_ = SLIDE_APPROACH;
        return computeStanley(vehicle, dt);
    }
    
    if (rho < posTol && std::abs(thetaE) < angTol) {
        slidePhase_ = SLIDE_APPROACH;
        return {0.0, 0.0};
    }
    
    switch (slidePhase_) {
    case SLIDE_APPROACH: {
        if (rho < posTol && std::abs(thetaE) > angTol) {
            if (std::abs(thetaE) > turnThresh) {
                slidePhase_ = SLIDE_BACK_OFF;
                slideBackOffStartX_ = pose.x;
                slideBackOffStartY_ = pose.y;
            } else {
                slidePhase_ = SLIDE_ANGLE_ADJUST;
            }
            return computeSlidingMode(vehicle, dt);
        }
        vel = computeStanley(vehicle, dt);
        break;
    }
    
    case SLIDE_ANGLE_ADJUST: {
        if (rho > driftTol) {
            slidePhase_ = SLIDE_BACK_OFF;
            slideBackOffStartX_ = pose.x;
            slideBackOffStartY_ = pose.y;
            return computeSlidingMode(vehicle, dt);
        }
        if (std::abs(thetaE) < angTol) {
            slidePhase_ = SLIDE_APPROACH;
            return {0.0, 0.0};
        }
        double v = (thetaE > 0) ? creep : -creep;
        double omega = params_.kp_omega * thetaE;
        double maxOmega = (std::abs(v) * std::tan(props.maxSteeringAngle)) / L;
        omega = std::max(-maxOmega, std::min(maxOmega, omega));
        vel.v = v;
        vel.omega = omega;
        break;
    }
    
    case SLIDE_BACK_OFF: {
        double dx = pose.x - goal_.x();
        double dy = pose.y - goal_.y();
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < 0.05) {
            dx = std::cos(pose.theta + M_PI);
            dy = std::sin(pose.theta + M_PI);
        } else {
            dx /= d;
            dy /= d;
        }
        double distFromStart = std::sqrt((pose.x - slideBackOffStartX_) * (pose.x - slideBackOffStartX_) +
                                        (pose.y - slideBackOffStartY_) * (pose.y - slideBackOffStartY_));
        if (distFromStart >= backOffDist) {
            slidePhase_ = SLIDE_TURN;
            return computeSlidingMode(vehicle, dt);
        }
        double vBack = 0.5 * params_.maxLinearVel;
        double dir = dx * std::cos(pose.theta) + dy * std::sin(pose.theta);
        vel.v = (dir > 0) ? vBack : -vBack;
        vel.omega = 0.0;  // 直线后退，转向在 TURN 阶段完成
        break;
    }
    
    case SLIDE_TURN: {
        if (std::abs(thetaE) < angTol) {
            slidePhase_ = SLIDE_RE_APPROACH;
            return computeSlidingMode(vehicle, dt);
        }
        double v = (thetaE > 0) ? creep : -creep;
        double omega = params_.kp_omega * thetaE;
        double maxOmega = (std::abs(v) * std::tan(props.maxSteeringAngle)) / L;
        omega = std::max(-maxOmega, std::min(maxOmega, omega));
        vel.v = v;
        vel.omega = omega;
        break;
    }
    
    case SLIDE_RE_APPROACH: {
        if (rho < posTol && std::abs(thetaE) < angTol) {
            slidePhase_ = SLIDE_APPROACH;
            return {0.0, 0.0};
        }
        if (rho < posTol && std::abs(thetaE) > angTol) {
            if (std::abs(thetaE) > turnThresh) {
                slidePhase_ = SLIDE_BACK_OFF;
                slideBackOffStartX_ = pose.x;
                slideBackOffStartY_ = pose.y;
                return computeSlidingMode(vehicle, dt);
            }
            slidePhase_ = SLIDE_ANGLE_ADJUST;
            return computeSlidingMode(vehicle, dt);
        }
        vel = computeStanley(vehicle, dt);
        break;
    }
    }
    
    return vel;
}

bool Controller::isGoalReached(const Vehicle& vehicle) const {
    if (!hasGoal_) return false;
    
    Vehicle::Pose pose = vehicle.getPose();
    double distance = computeDistance(vehicle);
    double posTol = (algorithmMode_ == ALGORITHM_SLIDING_MODE) ? params_.slidePosTol : params_.goalTolerance;
    double angTol = (algorithmMode_ == ALGORITHM_SLIDING_MODE) ? params_.slideAngTol : params_.angleTolerance;
    
    if (distance >= posTol) return false;
    if (hasGoalTheta_) {
        double angleError = std::abs(normalizeAngle(goalTheta_ - pose.theta));
        return angleError < angTol;
    }
    return true;
}

