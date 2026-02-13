#include "mapwidget.h"
#include <cmath>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

MapWidget::MapWidget(QWidget* parent)
    : QWidget(parent), vehicle_(nullptr), map_(nullptr), controller_(nullptr), mapData_(nullptr) {
    setMinimumSize(800, 600);
    setStyleSheet("background-color: #f0f0f0; border: 2px solid #333;");
}

void MapWidget::paintEvent(QPaintEvent* event) {
    QWidget::paintEvent(event);
    
    if (!vehicle_ || !map_) return;
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 计算地图绘制区域（考虑坐标轴标注空间）
    double leftMargin = 60;   // 左侧留出空间给Y轴标注
    double bottomMargin = 50;  // 底部留出空间给X轴标注
    double topMargin = 20;
    double rightMargin = 20;
    
    mapRect_ = QRectF(leftMargin, topMargin,
                     width() - leftMargin - rightMargin,
                     height() - topMargin - bottomMargin);
    
    // 确保x和y使用相同的缩放比例（保持纵横比）
    Map::MapProperties mapProps = map_->getProperties();
    double scaleX = mapRect_.width() / mapProps.width;
    double scaleY = mapRect_.height() / mapProps.height;
    double scale = std::min(scaleX, scaleY);  // 使用较小的缩放比例
    
    // 调整mapRect_使其保持纵横比
    double mapAspect = mapProps.width / mapProps.height;
    double rectAspect = mapRect_.width() / mapRect_.height();
    
    if (mapAspect > rectAspect) {
        // 地图更宽，调整高度
        double newHeight = mapRect_.width() / mapAspect;
        double offsetY = (mapRect_.height() - newHeight) / 2.0;
        mapRect_.adjust(0, offsetY, 0, -offsetY);
    } else {
        // 地图更高，调整宽度
        double newWidth = mapRect_.height() * mapAspect;
        double offsetX = (mapRect_.width() - newWidth) / 2.0;
        mapRect_.adjust(offsetX, 0, -offsetX, 0);
    }
    
    // 绘制地图
    drawMap(painter);
    drawGrid(painter);
    drawAxes(painter);  // 绘制坐标轴和标尺
    drawMapEdges(painter);  // 先绘制边（在点位下方）
    drawMapPoints(painter);  // 绘制点位
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
    
    // 计算统一的缩放比例（x和y相同，保持纵横比）
    double scaleX = mapRect_.width() / mapProps.width;
    double scaleY = mapRect_.height() / mapProps.height;
    double scale = std::min(scaleX, scaleY);  // 使用较小的缩放比例
    
    // 计算地图中心在屏幕上的位置（考虑保持纵横比后的实际地图区域）
    double actualMapWidth = mapProps.width * scale;
    double actualMapHeight = mapProps.height * scale;
    double centerX = mapRect_.left() + mapRect_.width() / 2.0;
    double centerY = mapRect_.top() + mapRect_.height() / 2.0;
    
    // 转换到屏幕坐标（使用统一的缩放比例）
    double sx = centerX + (worldPos.x() - mapProps.originX - mapProps.width / 2.0) * scale;
    double sy = centerY - (worldPos.y() - mapProps.originY - mapProps.height / 2.0) * scale; // Y轴翻转
    
    return QPointF(sx, sy);
}

QPointF MapWidget::screenToWorld(const QPointF& screenPos) const {
    if (!map_) return QPointF();
    
    Map::MapProperties mapProps = map_->getProperties();
    
    // 计算统一的缩放比例（x和y相同，保持纵横比）
    double scaleX = mapRect_.width() / mapProps.width;
    double scaleY = mapRect_.height() / mapProps.height;
    double scale = std::min(scaleX, scaleY);  // 使用较小的缩放比例
    
    // 计算地图中心在屏幕上的位置
    double centerX = mapRect_.left() + mapRect_.width() / 2.0;
    double centerY = mapRect_.top() + mapRect_.height() / 2.0;
    
    // 转换到世界坐标（使用统一的缩放比例）
    double wx = mapProps.originX + mapProps.width / 2.0 + (screenPos.x() - centerX) / scale;
    double wy = mapProps.originY + mapProps.height / 2.0 - (screenPos.y() - centerY) / scale; // Y轴翻转
    
    return QPointF(wx, wy);
}

double MapWidget::worldToScreenLength(double worldLength) const {
    if (!map_) return 0;
    
    Map::MapProperties mapProps = map_->getProperties();
    // 使用统一的缩放比例（x和y相同）
    double scaleX = mapRect_.width() / mapProps.width;
    double scaleY = mapRect_.height() / mapProps.height;
    double scale = std::min(scaleX, scaleY);  // 使用较小的缩放比例，保持纵横比
    
    return worldLength * scale;
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

void MapWidget::drawAxes(QPainter& painter) {
    if (!map_) return;
    
    Map::MapProperties mapProps = map_->getProperties();
    
    // 设置字体
    QFont font = painter.font();
    font.setPointSize(9);
    painter.setFont(font);
    
    // 计算缩放比例（x和y相同）
    double scaleX = mapRect_.width() / mapProps.width;
    double scaleY = mapRect_.height() / mapProps.height;
    double scale = std::min(scaleX, scaleY);
    
    // 绘制X轴（底部）
    painter.setPen(QPen(Qt::black, 2));
    QPointF xAxisStart = QPointF(mapRect_.left(), mapRect_.bottom());
    QPointF xAxisEnd = QPointF(mapRect_.right(), mapRect_.bottom());
    painter.drawLine(xAxisStart, xAxisEnd);
    
    // 绘制X轴箭头
    double arrowSize = 8;
    painter.drawLine(xAxisEnd, xAxisEnd + QPointF(-arrowSize, -arrowSize * 0.5));
    painter.drawLine(xAxisEnd, xAxisEnd + QPointF(-arrowSize, arrowSize * 0.5));
    
    // 绘制Y轴（左侧）
    QPointF yAxisStart = QPointF(mapRect_.left(), mapRect_.bottom());
    QPointF yAxisEnd = QPointF(mapRect_.left(), mapRect_.top());
    painter.drawLine(yAxisStart, yAxisEnd);
    
    // 绘制Y轴箭头
    painter.drawLine(yAxisEnd, yAxisEnd + QPointF(-arrowSize * 0.5, arrowSize));
    painter.drawLine(yAxisEnd, yAxisEnd + QPointF(arrowSize * 0.5, arrowSize));
    
    // X轴标尺刻度
    double tickInterval = 10.0;  // 每10米一个刻度
    int numTicksX = static_cast<int>(mapProps.width / tickInterval) + 1;
    
    for (int i = 0; i <= numTicksX; ++i) {
        double worldX = i * tickInterval;
        if (worldX > mapProps.width) break;
        
        QPointF tickPos = worldToScreen(QPointF(worldX, 0));
        tickPos.setY(mapRect_.bottom());  // 确保在X轴上
        
        // 绘制刻度线
        painter.setPen(QPen(Qt::black, 1));
        painter.drawLine(tickPos.x(), mapRect_.bottom(), 
                        tickPos.x(), mapRect_.bottom() + 5);
        
        // 绘制刻度值
        painter.setPen(QPen(Qt::black, 1));
        QString label = QString::number(worldX, 'f', 1);
        QRectF textRect(tickPos.x() - 20, mapRect_.bottom() + 8, 40, 15);
        painter.drawText(textRect, Qt::AlignCenter, label);
    }
    
    // Y轴标尺刻度
    int numTicksY = static_cast<int>(mapProps.height / tickInterval) + 1;
    
    for (int i = 0; i <= numTicksY; ++i) {
        double worldY = i * tickInterval;
        if (worldY > mapProps.height) break;
        
        QPointF tickPos = worldToScreen(QPointF(0, worldY));
        tickPos.setX(mapRect_.left());  // 确保在Y轴上
        
        // 绘制刻度线
        painter.setPen(QPen(Qt::black, 1));
        painter.drawLine(mapRect_.left(), tickPos.y(), 
                        mapRect_.left() - 5, tickPos.y());
        
        // 绘制刻度值
        painter.setPen(QPen(Qt::black, 1));
        QString label = QString::number(worldY, 'f', 1);
        QRectF textRect(mapRect_.left() - 55, tickPos.y() - 7, 50, 15);
        painter.drawText(textRect, Qt::AlignRight | Qt::AlignVCenter, label);
    }
    
    // 绘制坐标轴标签
    painter.setPen(QPen(Qt::black, 1));
    QFont labelFont = painter.font();
    labelFont.setPointSize(10);
    labelFont.setBold(true);
    painter.setFont(labelFont);
    
    // X轴标签
    QRectF xLabelRect(mapRect_.right() - 30, mapRect_.bottom() + 25, 20, 15);
    painter.drawText(xLabelRect, Qt::AlignCenter, "X (m)");
    
    // Y轴标签（旋转90度）
    painter.save();
    QRectF yLabelRect(mapRect_.left() - 45, mapRect_.top() + 10, 40, 15);
    painter.translate(yLabelRect.center());
    painter.rotate(-90);
    painter.drawText(QRectF(-20, -7, 40, 15), Qt::AlignCenter, "Y (m)");
    painter.restore();
    
    // 绘制原点标记
    QPointF origin = worldToScreen(QPointF(0, 0));
    painter.setPen(QPen(Qt::black, 2));
    painter.setBrush(QBrush(Qt::black));
    painter.drawEllipse(origin, 3, 3);
    
    // 原点标签
    painter.setFont(font);
    painter.setPen(QPen(Qt::black, 1));
    QRectF originLabelRect(mapRect_.left() - 25, mapRect_.bottom() + 8, 20, 15);
    painter.drawText(originLabelRect, Qt::AlignCenter, "O");
}

void MapWidget::drawMapPoints(QPainter& painter) {
    if (!mapData_) return;
    
    QVector<MapData::Point> points = mapData_->getAllPoints();
    
    for (const MapData::Point& point : points) {
        QPointF screenPos = worldToScreen(QPointF(point.x, point.y));
        
        // 根据点位类型选择颜色和大小
        QColor pointColor;
        double pointSize = 8.0;
        QString pointLabel;
        
        switch (point.type) {
            case MapData::POINT_WORKSTATION:
                pointColor = Qt::blue;
                pointSize = 10.0;
                pointLabel = "工";
                break;
            case MapData::POINT_PATH:
                pointColor = Qt::gray;
                pointSize = 6.0;
                pointLabel = "路";
                break;
            case MapData::POINT_REST:
                pointColor = Qt::green;
                pointSize = 8.0;
                pointLabel = "休";
                break;
        }
        
        // 绘制点位圆圈
        painter.setPen(QPen(pointColor.darker(), 2));
        painter.setBrush(QBrush(pointColor.lighter(150)));
        painter.drawEllipse(screenPos, pointSize, pointSize);
        
        // 绘制点位ID和名称
        painter.setPen(QPen(Qt::black, 1));
        QFont font = painter.font();
        font.setPointSize(8);
        font.setBold(true);
        painter.setFont(font);
        
        // 在点位上方显示ID和名称
        QString text = QString("%1\n%2").arg(point.id).arg(point.name);
        QRectF textRect(screenPos.x() - 30, screenPos.y() - pointSize - 25, 60, 20);
        painter.drawText(textRect, Qt::AlignCenter, text);
        
        // 绘制点位朝向箭头
        if (std::abs(point.theta) > 0.01) {
            double arrowLength = worldToScreenLength(1.0);
            QPointF arrowEnd = screenPos + QPointF(
                arrowLength * std::cos(point.theta),
                -arrowLength * std::sin(point.theta)  // Y轴翻转
            );
            
            painter.setPen(QPen(pointColor.darker(), 2));
            painter.drawLine(screenPos, arrowEnd);
            
            // 箭头头部
            double arrowHeadSize = 5;
            QPointF arrowDir = QPointF(std::cos(point.theta), -std::sin(point.theta));
            QPointF perp = QPointF(-arrowDir.y(), arrowDir.x());
            QPointF head1 = arrowEnd - arrowHeadSize * arrowDir + arrowHeadSize * 0.5 * perp;
            QPointF head2 = arrowEnd - arrowHeadSize * arrowDir - arrowHeadSize * 0.5 * perp;
            painter.drawLine(arrowEnd, head1);
            painter.drawLine(arrowEnd, head2);
        }
    }
}

void MapWidget::drawMapEdges(QPainter& painter) {
    if (!mapData_) return;
    
    QVector<MapData::Edge> edges = mapData_->getAllEdges();
    
    for (const MapData::Edge& edge : edges) {
        const MapData::Point* startPoint = mapData_->getPoint(edge.startId);
        const MapData::Point* endPoint = mapData_->getPoint(edge.endId);
        
        if (!startPoint || !endPoint) continue;
        
        QPointF startScreen = worldToScreen(QPointF(startPoint->x, startPoint->y));
        QPointF endScreen = worldToScreen(QPointF(endPoint->x, endPoint->y));
        
        // 根据导航模式和行驶模式选择颜色和样式
        QColor edgeColor;
        Qt::PenStyle penStyle;
        
        if (edge.navMode == MapData::NAV_FORWARD) {
            edgeColor = Qt::darkGreen;
        } else {
            edgeColor = Qt::darkRed;  // 后退用红色
        }
        
        if (edge.driveMode == MapData::DRIVE_STRAIGHT) {
            penStyle = Qt::SolidLine;
        } else {
            penStyle = Qt::DashLine;  // 曲线用虚线
        }
        
        // 绘制边（直线）
        painter.setPen(QPen(edgeColor, 2, penStyle));
        painter.drawLine(startScreen, endScreen);
        
        // 绘制箭头（表示方向）
        QPointF direction = endScreen - startScreen;
        double length = std::sqrt(direction.x() * direction.x() + direction.y() * direction.y());
        if (length > 0.1) {
            direction /= length;  // 归一化
            
            // 箭头位置（距离终点一定距离）
            double arrowOffset = worldToScreenLength(1.5);
            QPointF arrowPos = endScreen - direction * arrowOffset;
            
            // 箭头大小
            double arrowSize = 8;
            QPointF perp = QPointF(-direction.y(), direction.x());
            
            QPointF head1 = arrowPos - arrowSize * direction + arrowSize * 0.5 * perp;
            QPointF head2 = arrowPos - arrowSize * direction - arrowSize * 0.5 * perp;
            
            painter.setPen(QPen(edgeColor, 2));
            painter.drawLine(arrowPos, head1);
            painter.drawLine(arrowPos, head2);
            
            // 在边的中点显示导航模式标签
            QPointF midPoint = (startScreen + endScreen) / 2.0;
            QFont font = painter.font();
            font.setPointSize(7);
            painter.setFont(font);
            
            QString label = edge.navMode == MapData::NAV_FORWARD ? "前进" : "后退";
            if (edge.driveMode == MapData::DRIVE_CURVE) {
                label += " (曲线)";
            }
            
            QRectF labelRect(midPoint.x() - 25, midPoint.y() - 8, 50, 16);
            painter.setPen(QPen(Qt::white, 1));
            painter.setBrush(QBrush(edgeColor));
            painter.drawRect(labelRect);
            painter.setPen(QPen(Qt::white, 1));
            painter.drawText(labelRect, Qt::AlignCenter, label);
        }
    }
}

void MapWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    update();  // 触发重绘
}

