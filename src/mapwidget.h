#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include "vehicle.h"
#include "map.h"
#include "controller.h"

/**
 * @brief 地图绘制组件
 */
class MapWidget : public QWidget {
    Q_OBJECT

public:
    explicit MapWidget(QWidget* parent = nullptr);
    
    // 设置数据
    void setVehicle(Vehicle* vehicle) { vehicle_ = vehicle; }
    void setMap(Map* map) { map_ = map; }
    void setController(Controller* controller) { controller_ = controller; }

signals:
    void mouseClicked(const QPointF& worldPos, Qt::MouseButton button);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    Vehicle* vehicle_;
    Map* map_;
    Controller* controller_;
    
    QRectF mapRect_;  // 地图绘制区域
    
    // 坐标转换
    QPointF worldToScreen(const QPointF& worldPos) const;
    QPointF screenToWorld(const QPointF& screenPos) const;
    double worldToScreenLength(double worldLength) const;
    
    // 绘制函数
    void drawMap(QPainter& painter);
    void drawGrid(QPainter& painter);
    void drawTrajectory(QPainter& painter);
    void drawGoal(QPainter& painter);
    void drawVehicle(QPainter& painter);
};

#endif // MAPWIDGET_H

