#ifndef MAPEDITORCANVAS_H
#define MAPEDITORCANVAS_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QMouseEvent>
#include "../core/mapdata.h"

/**
 * @brief 地图编辑画布 - 支持通过点击添加点位和边
 */
class MapEditorCanvas : public QWidget {
    Q_OBJECT

public:
    enum EditMode {
        MODE_SELECT,      // 选择模式（无操作）
        MODE_ADD_POINT,   // 添加点位：点击地图任意位置添加
        MODE_ADD_EDGE,    // 添加边：先点击起点，再点击终点
        MODE_REMOVE_POINT,// 删除点位：点击点位删除
        MODE_REMOVE_EDGE  // 删除边：点击边删除
    };

    explicit MapEditorCanvas(QWidget* parent = nullptr);

    // 数据
    void setMapData(MapData* mapData) { mapData_ = mapData; update(); }
    MapData* mapData() { return mapData_; }
    const MapData* mapData() const { return mapData_; }

    // 地图尺寸（米）
    void setMapSize(double width, double height);
    double mapWidth() const { return mapWidth_; }
    double mapHeight() const { return mapHeight_; }

    // 编辑模式
    void setEditMode(EditMode mode);
    EditMode editMode() const { return editMode_; }

signals:
    void mapDataChanged();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

private:
    MapData* mapData_;
    double mapWidth_;
    double mapHeight_;
    EditMode editMode_;

    // 添加边时的临时状态
    int pendingEdgeStartId_;  // -1 表示未选择起点

    QRectF mapRect_;  // 地图绘制区域（屏幕坐标）

    // 坐标转换
    QPointF worldToScreen(const QPointF& worldPos) const;
    QPointF screenToWorld(const QPointF& screenPos) const;
    double worldToScreenLength(double worldLength) const;

    // 命中检测（屏幕坐标，像素容差）
    int findPointAt(const QPointF& screenPos, double tolerance = 15.0) const;
    bool findEdgeAt(const QPointF& screenPos, int& outStartId, int& outEndId, double tolerance = 8.0) const;

    // 绘制
    void drawMap(QPainter& painter);
    void drawGrid(QPainter& painter);
    void drawMapPoints(QPainter& painter);
    void drawMapEdges(QPainter& painter);
    void drawPendingEdge(QPainter& painter);
};

#endif // MAPEDITORCANVAS_H
