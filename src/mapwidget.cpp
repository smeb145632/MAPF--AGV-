#include "mapwidget.h"
#include <cmath>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

MapWidget::MapWidget(QWidget* parent)
    : QWidget(parent), vehicle_(nullptr), map_(nullptr), controller_(nullptr) {
    setMinimumSize(800, 600);
    setStyleSheet("background-color: #f0f0f0; border: 2px solid #333;");
}

void MapWidget::paintEvent(QPaintEvent* event) {
    QWidget::paintEvent(event);
    
    if (!vehicle_ || !map_) return;
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 计算地图绘制区域
    double margin = 20;
    mapRect_ = QRectF(margin, margin,
                     width() - 2 * margin,
                     height() - 2 * margin);
    
    // 绘制地图
    drawMap(painter);
    drawGrid(painter);
    drawTrajectory(painter);
    drawGoal(painter);
    drawVehicle(painter);
}

void MapWidget::drawMap(QPainter& painter) {
    // 绘制地图边界
    painter.setPen(QPen(Qt::black, 2));
    painter.setBrush(QBrush(Qt::white));
    painter.drawRect(mapRect_);
}

void MapWidget::drawGrid(QPainter& painter) {
    if (!map_) return;
    
    // 绘制网格
    painter.setPen(QPen(Qt::lightGray, 1, Qt::DashLine));
    
    Map::MapProperties mapProps = map_->getProperties();
    double gridSize = 5.0;  // 网格大小（米）
    
    // 计算网格线数量
    int numLinesX = static_cast<int>(mapProps.width / gridSize) + 1;
    int numLinesY = static_cast<int>(mapProps.height / gridSize) + 1;
    
    // 绘制垂直线
    for (int i = 0; i < numLinesX; ++i) {
        double x = i * gridSize;
        QPointF p1 = worldToScreen(QPointF(x, 0));
        QPointF p2 = worldToScreen(QPointF(x, mapProps.height));
        painter.drawLine(p1, p2);
    }
    
    // 绘制水平线
    for (int i = 0; i < numLinesY; ++i) {
        double y = i * gridSize;
        QPointF p1 = worldToScreen(QPointF(0, y));
        QPointF p2 = worldToScreen(QPointF(mapProps.width, y));
        painter.drawLine(p1, p2);
    }
}

void MapWidget::drawTrajectory(QPainter& painter) {
    if (!vehicle_ || vehicle_->getTrajectory().isEmpty()) return;
    
    // 绘制轨迹
    painter.setPen(QPen(Qt::green, 2));
    QVector<QPointF> trajectory = vehicle_->getTrajectory();
    
    for (int i = 1; i < trajectory.size(); ++i) {
        QPointF p1 = worldToScreen(trajectory[i-1]);
        QPointF p2 = worldToScreen(trajectory[i]);
        painter.drawLine(p1, p2);
    }
}

void MapWidget::drawGoal(QPainter& painter) {
    if (!controller_ || !controller_->hasGoal()) return;
    
    QPointF goal = controller_->getGoal();
    QPointF screenGoal = worldToScreen(goal);
    
    // 绘制目标点
    painter.setPen(QPen(Qt::red, 2));
    painter.setBrush(QBrush(Qt::red));
    double radius = 8;
    painter.drawEllipse(screenGoal, radius, radius);
    
    // 绘制十字标记
    painter.drawLine(screenGoal.x() - radius, screenGoal.y(),
                     screenGoal.x() + radius, screenGoal.y());
    painter.drawLine(screenGoal.x(), screenGoal.y() - radius,
                     screenGoal.x(), screenGoal.y() + radius);
    
    // 如果指定了目标角度，绘制角度方向箭头（屏幕坐标系，Y轴向下）
    if (controller_->hasGoalTheta()) {
        double goalTheta = controller_->getGoalTheta();
        double arrowLength = worldToScreenLength(2.0);
        double screenDx = arrowLength * std::cos(goalTheta);
        double screenDy = -arrowLength * std::sin(goalTheta);  // 翻转Y轴
        QPointF arrowEnd = screenGoal + QPointF(screenDx, screenDy);
        
        painter.setPen(QPen(Qt::darkRed, 3, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawLine(screenGoal, arrowEnd);
        
        // 绘制箭头头部
        double arrowHeadSize = 8;
        QPointF arrowDir = QPointF(screenDx, screenDy);
        double dirLength = std::sqrt(arrowDir.x() * arrowDir.x() + arrowDir.y() * arrowDir.y());
        if (dirLength > 0.1) {
            arrowDir /= dirLength;  // 归一化
            QPointF perp = QPointF(-arrowDir.y(), arrowDir.x());
            QPointF head1 = arrowEnd - arrowHeadSize * arrowDir + arrowHeadSize * 0.5 * perp;
            QPointF head2 = arrowEnd - arrowHeadSize * arrowDir - arrowHeadSize * 0.5 * perp;
            painter.drawLine(arrowEnd, head1);
            painter.drawLine(arrowEnd, head2);
        }
    }
}

void MapWidget::drawVehicle(QPainter& painter) {
    if (!vehicle_) return;
    
    Vehicle::Pose pose = vehicle_->getPose();
    Vehicle::VehicleProperties props = vehicle_->getProperties();
    
    // 车辆中心在屏幕坐标系中的位置
    QPointF center = worldToScreen(QPointF(pose.x, pose.y));
    
    // 车辆尺寸（屏幕坐标系中的长度）
    double screenLength = worldToScreenLength(props.length);
    double screenWidth = worldToScreenLength(props.width);
    double halfLength = screenLength / 2.0;
    double halfWidth = screenWidth / 2.0;
    
    // 车辆四个角的局部坐标（以车辆中心为原点，朝向为0度时）
    // 注意：在屏幕坐标系中，Y轴向下，所以需要调整
    // 世界坐标系：theta=0指向X正方向（右），theta=π/2指向Y正方向（上）
    // 屏幕坐标系：theta=0指向X正方向（右），theta=π/2指向Y正方向（下）
    // 因此需要将世界坐标系的theta转换为屏幕坐标系的theta：screenTheta = -theta
    double screenTheta = -pose.theta;  // 屏幕坐标系中Y轴向下，所以角度取反
    
    double cosTheta = std::cos(screenTheta);
    double sinTheta = std::sin(screenTheta);
    
    // 车辆四个角的局部坐标（车辆前方为length方向，宽度为width方向）
    QVector<QPointF> localCorners = {
        QPointF(halfLength, halfWidth),   // 右前（车辆前方右侧）
        QPointF(halfLength, -halfWidth),  // 右后（车辆后方右侧）
        QPointF(-halfLength, -halfWidth), // 左后（车辆后方左侧）
        QPointF(-halfLength, halfWidth)   // 左前（车辆前方左侧）
    };
    
    // 在屏幕坐标系中旋转和平移
    QPolygonF polygon;
    for (const QPointF& local : localCorners) {
        // 旋转（在屏幕坐标系中）
        double x_rot = local.x() * cosTheta - local.y() * sinTheta;
        double y_rot = local.x() * sinTheta + local.y() * cosTheta;
        
        // 平移到屏幕位置
        polygon << (center + QPointF(x_rot, y_rot));
    }
    
    // 绘制车辆（俯视图：矩形）
    painter.setPen(QPen(vehicle_->getColor(), 2));
    painter.setBrush(QBrush(vehicle_->getColor().lighter(150)));
    painter.drawPolygon(polygon);
    
    // 绘制车辆朝向箭头（在屏幕坐标系中）
    // 车辆前方方向：在屏幕坐标系中，theta=0时指向右，所以箭头方向就是(cos, sin)
    double arrowLength = halfLength + 5;  // 箭头稍微超出车辆前端
    QPointF arrowEnd = center + QPointF(
        arrowLength * cosTheta,
        arrowLength * sinTheta
    );
    
    painter.setPen(QPen(Qt::yellow, 3, Qt::SolidLine, Qt::RoundCap));
    painter.drawLine(center, arrowEnd);
    
    // 绘制箭头头部
    double arrowHeadSize = 8;
    QPointF arrowDir = arrowEnd - center;
    double dirLength = std::sqrt(arrowDir.x() * arrowDir.x() + arrowDir.y() * arrowDir.y());
    if (dirLength > 0.1) {
        arrowDir /= dirLength;  // 归一化
        QPointF perp = QPointF(-arrowDir.y(), arrowDir.x());  // 垂直方向
        QPointF head1 = arrowEnd - arrowHeadSize * arrowDir + arrowHeadSize * 0.5 * perp;
        QPointF head2 = arrowEnd - arrowHeadSize * arrowDir - arrowHeadSize * 0.5 * perp;
        painter.drawLine(arrowEnd, head1);
        painter.drawLine(arrowEnd, head2);
    }
    
    // 绘制车辆中心点
    painter.setPen(QPen(Qt::black, 1));
    painter.setBrush(QBrush(Qt::black));
    painter.drawEllipse(center, 3, 3);
}

QPointF MapWidget::worldToScreen(const QPointF& worldPos) const {
    if (!map_) return QPointF();
    
    Map::MapProperties mapProps = map_->getProperties();
    
    // 归一化坐标 [0, 1]
    double nx = (worldPos.x() - mapProps.originX) / mapProps.width;
    double ny = 1.0 - (worldPos.y() - mapProps.originY) / mapProps.height; // Y轴翻转
    
    // 转换到屏幕坐标
    double sx = mapRect_.left() + nx * mapRect_.width();
    double sy = mapRect_.top() + ny * mapRect_.height();
    
    return QPointF(sx, sy);
}

QPointF MapWidget::screenToWorld(const QPointF& screenPos) const {
    if (!map_) return QPointF();
    
    Map::MapProperties mapProps = map_->getProperties();
    
    // 归一化坐标 [0, 1]
    double nx = (screenPos.x() - mapRect_.left()) / mapRect_.width();
    double ny = 1.0 - (screenPos.y() - mapRect_.top()) / mapRect_.height(); // Y轴翻转
    
    // 转换到世界坐标
    double wx = mapProps.originX + nx * mapProps.width;
    double wy = mapProps.originY + ny * mapProps.height;
    
    return QPointF(wx, wy);
}

double MapWidget::worldToScreenLength(double worldLength) const {
    if (!map_) return 0;
    
    Map::MapProperties mapProps = map_->getProperties();
    return worldLength * mapRect_.width() / mapProps.width;
}

void MapWidget::mousePressEvent(QMouseEvent* event) {
    if (mapRect_.contains(event->pos())) {
        QPointF worldPos = screenToWorld(event->pos());
        emit mouseClicked(worldPos, event->button());
    }
    QWidget::mousePressEvent(event);
}

void MapWidget::mouseMoveEvent(QMouseEvent* event) {
    QWidget::mouseMoveEvent(event);
}

void MapWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    update();  // 触发重绘
}

