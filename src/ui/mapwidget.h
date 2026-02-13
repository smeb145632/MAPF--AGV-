#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include "../core/vehicle.h"
#include "../core/map.h"
#include "../core/mapdata.h"
#include "../control/controller.h"

/**
 * @brief 地图绘制组件
 */
class MapWidget : public QWidget {
    Q_OBJECT

public:
    explicit MapWidget(QWidget* parent = nullptr);
    
    // 设置数据
    void setVehicle(Vehicle* vehicle) { vehicle_ = vehicle; update(); }
    void setMap(Map* map) { map_ = map; update(); }
    void setController(Controller* controller) { controller_ = controller; update(); }
    void setMapData(const MapData* mapData) { mapData_ = mapData; update(); }

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
    const MapData* mapData_;  // 地图数据（点位和边）
    
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
    void drawAxes(QPainter& painter);  // 绘制坐标轴和标尺
    void drawMapPoints(QPainter& painter);  // 绘制地图点位
    void drawMapEdges(QPainter& painter);    // 绘制地图有向边
};

#endif // MAPWIDGET_H

