#include "map.h"

Map::Map() : properties_({50.0, 50.0, 0.0, 0.0}) {}

Map::Map(double width, double height) 
    : properties_({width, height, 0.0, 0.0}) {}

QRectF Map::getBounds() const {
    return QRectF(properties_.originX, properties_.originY,
                  properties_.width, properties_.height);
}

bool Map::isPointInMap(double x, double y) const {
    return (x >= properties_.originX && 
            x <= properties_.originX + properties_.width &&
            y >= properties_.originY && 
            y <= properties_.originY + properties_.height);
}

bool Map::isPointInMap(const QPointF& point) const {
    return isPointInMap(point.x(), point.y());
}

QPointF Map::clampToMap(double x, double y) const {
    double clampedX = std::max(properties_.originX, 
                               std::min(properties_.originX + properties_.width, x));
    double clampedY = std::max(properties_.originY, 
                               std::min(properties_.originY + properties_.height, y));
    return QPointF(clampedX, clampedY);
}

QPointF Map::clampToMap(const QPointF& point) const {
    return clampToMap(point.x(), point.y());
}

void Map::setSize(double width, double height) {
    properties_.width = width;
    properties_.height = height;
}


