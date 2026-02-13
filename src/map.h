#ifndef MAP_H
#define MAP_H

#include <QPointF>
#include <QRectF>
#include <QVector>

/**
 * @brief 地图类 - 管理方形空间地图
 */
class Map {
public:
    Map();
    Map(double width, double height);
    
    // 地图属性
    struct MapProperties {
        double width = 50.0;   // 地图宽度 (m)
        double height = 50.0;  // 地图高度 (m)
        double originX = 0.0;  // 原点X坐标
        double originY = 0.0;  // 原点Y坐标
    };
    
    // Getters
    MapProperties getProperties() const { return properties_; }
    QRectF getBounds() const;
    
    // 检查点是否在地图范围内
    bool isPointInMap(double x, double y) const;
    bool isPointInMap(const QPointF& point) const;
    
    // 将点限制在地图范围内
    QPointF clampToMap(double x, double y) const;
    QPointF clampToMap(const QPointF& point) const;
    
    // 设置地图大小
    void setSize(double width, double height);
    
private:
    MapProperties properties_;
};

#endif // MAP_H


