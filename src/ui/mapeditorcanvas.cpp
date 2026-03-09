#include "mapeditorcanvas.h"
#include <QPainter>
#include <QMouseEvent>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MapEditorCanvas::MapEditorCanvas(QWidget* parent)
    : QWidget(parent), mapData_(nullptr), mapWidth_(50.0), mapHeight_(50.0),
      editMode_(MODE_SELECT), pendingEdgeStartId_(-1) {
    setMinimumSize(600, 500);
    setStyleSheet("background-color: #f5f5f5; border: 1px solid #ccc;");
    setMouseTracking(true);
}

void MapEditorCanvas::setMapSize(double width, double height) {
    mapWidth_ = qMax(10.0, width);
    mapHeight_ = qMax(10.0, height);
    update();
}

void MapEditorCanvas::setEditMode(EditMode mode) {
    editMode_ = mode;
    pendingEdgeStartId_ = -1;  // 切换模式时清除边的临时状态
    update();
}

void MapEditorCanvas::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QWidget::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    double leftMargin = 50;
    double bottomMargin = 40;
    double topMargin = 20;
    double rightMargin = 20;

    mapRect_ = QRectF(leftMargin, topMargin,
                      width() - leftMargin - rightMargin,
                      height() - topMargin - bottomMargin);

    double scaleX = mapRect_.width() / mapWidth_;
    double scaleY = mapRect_.height() / mapHeight_;
    double scale = qMin(scaleX, scaleY);

    double mapAspect = mapWidth_ / mapHeight_;
    double rectAspect = mapRect_.width() / mapRect_.height();

    if (mapAspect > rectAspect) {
        double newHeight = mapRect_.width() / mapAspect;
        double offsetY = (mapRect_.height() - newHeight) / 2.0;
        mapRect_.adjust(0, offsetY, 0, -offsetY);
    } else {
        double newWidth = mapRect_.height() * mapAspect;
        double offsetX = (mapRect_.width() - newWidth) / 2.0;
        mapRect_.adjust(offsetX, 0, -offsetX, 0);
    }

    drawMap(painter);
    drawGrid(painter);
    drawMapEdges(painter);
    drawMapPoints(painter);
    drawPendingEdge(painter);
}

void MapEditorCanvas::drawMap(QPainter& painter) {
    painter.setPen(QPen(Qt::black, 2));
    painter.setBrush(QBrush(Qt::white));
    painter.drawRect(mapRect_);
}

void MapEditorCanvas::drawGrid(QPainter& painter) {
    painter.setPen(QPen(Qt::lightGray, 1, Qt::DashLine));
    double gridSize = 5.0;
    int numLinesX = static_cast<int>(mapWidth_ / gridSize) + 1;
    int numLinesY = static_cast<int>(mapHeight_ / gridSize) + 1;

    for (int i = 0; i < numLinesX; ++i) {
        double x = i * gridSize;
        QPointF p1 = worldToScreen(QPointF(x, 0));
        QPointF p2 = worldToScreen(QPointF(x, mapHeight_));
        painter.drawLine(p1, p2);
    }
    for (int i = 0; i < numLinesY; ++i) {
        double y = i * gridSize;
        QPointF p1 = worldToScreen(QPointF(0, y));
        QPointF p2 = worldToScreen(QPointF(mapWidth_, y));
        painter.drawLine(p1, p2);
    }
}

void MapEditorCanvas::drawMapPoints(QPainter& painter) {
    if (!mapData_) return;

    QVector<MapData::Point> points = mapData_->getAllPoints();
    for (const MapData::Point& point : points) {
        QPointF screenPos = worldToScreen(QPointF(point.x, point.y));

        QColor pointColor;
        double pointSize = 8.0;
        switch (point.type) {
            case MapData::POINT_WORKSTATION:
                pointColor = Qt::blue;
                pointSize = 10.0;
                break;
            case MapData::POINT_PATH:
                pointColor = Qt::gray;
                pointSize = 6.0;
                break;
            case MapData::POINT_REST:
                pointColor = Qt::green;
                pointSize = 8.0;
                break;
        }

        bool isPending = (editMode_ == MODE_ADD_EDGE && pendingEdgeStartId_ == point.id);
        if (isPending) {
            painter.setPen(QPen(Qt::darkBlue, 3));
            painter.setBrush(QBrush(Qt::cyan));
        } else {
            painter.setPen(QPen(pointColor.darker(), 2));
            painter.setBrush(QBrush(pointColor.lighter(150)));
        }
        painter.drawEllipse(screenPos, pointSize, pointSize);

        painter.setPen(QPen(Qt::black, 1));
        QFont font = painter.font();
        font.setPointSize(8);
        painter.setFont(font);
        QString text = QString("%1\n%2").arg(point.id).arg(point.name);
        QRectF textRect(screenPos.x() - 30, screenPos.y() - pointSize - 25, 60, 20);
        painter.drawText(textRect, Qt::AlignCenter, text);
    }
}

void MapEditorCanvas::drawMapEdges(QPainter& painter) {
    if (!mapData_) return;

    QVector<MapData::Edge> edges = mapData_->getAllEdges();
    for (const MapData::Edge& edge : edges) {
        const MapData::Point* startPoint = mapData_->getPoint(edge.startId);
        const MapData::Point* endPoint = mapData_->getPoint(edge.endId);
        if (!startPoint || !endPoint) continue;

        QPointF startScreen = worldToScreen(QPointF(startPoint->x, startPoint->y));
        QPointF endScreen = worldToScreen(QPointF(endPoint->x, endPoint->y));

        QColor edgeColor = (edge.navMode == MapData::NAV_FORWARD) ? Qt::darkGreen : Qt::darkRed;
        Qt::PenStyle penStyle = (edge.driveMode == MapData::DRIVE_STRAIGHT) ? Qt::SolidLine : Qt::DashLine;
        painter.setPen(QPen(edgeColor, 2, penStyle));
        painter.drawLine(startScreen, endScreen);

        QPointF direction = endScreen - startScreen;
        double length = std::sqrt(direction.x() * direction.x() + direction.y() * direction.y());
        if (length > 0.1) {
            direction /= length;
            double arrowOffset = worldToScreenLength(1.5);
            QPointF arrowPos = endScreen - direction * arrowOffset;
            double arrowSize = 8;
            QPointF perp(-direction.y(), direction.x());
            QPointF head1 = arrowPos - arrowSize * direction + arrowSize * 0.5 * perp;
            QPointF head2 = arrowPos - arrowSize * direction - arrowSize * 0.5 * perp;
            painter.drawLine(arrowPos, head1);
            painter.drawLine(arrowPos, head2);
        }
    }
}

void MapEditorCanvas::drawPendingEdge(QPainter& painter) {
    if (editMode_ != MODE_ADD_EDGE || pendingEdgeStartId_ < 0 || !mapData_) return;

    const MapData::Point* startPoint = mapData_->getPoint(pendingEdgeStartId_);
    if (!startPoint) return;

    QPointF startScreen = worldToScreen(QPointF(startPoint->x, startPoint->y));
    QPointF cursorScreen = mapFromGlobal(QCursor::pos());

    if (mapRect_.contains(cursorScreen)) {
        painter.setPen(QPen(Qt::darkBlue, 2, Qt::DashLine));
        painter.drawLine(startScreen, cursorScreen);
    }
}

QPointF MapEditorCanvas::worldToScreen(const QPointF& worldPos) const {
    double scaleX = mapRect_.width() / mapWidth_;
    double scaleY = mapRect_.height() / mapHeight_;
    double scale = qMin(scaleX, scaleY);
    double centerX = mapRect_.left() + mapRect_.width() / 2.0;
    double centerY = mapRect_.top() + mapRect_.height() / 2.0;

    double sx = centerX + (worldPos.x() - mapWidth_ / 2.0) * scale;
    double sy = centerY - (worldPos.y() - mapHeight_ / 2.0) * scale;
    return QPointF(sx, sy);
}

QPointF MapEditorCanvas::screenToWorld(const QPointF& screenPos) const {
    double scaleX = mapRect_.width() / mapWidth_;
    double scaleY = mapRect_.height() / mapHeight_;
    double scale = qMin(scaleX, scaleY);
    double centerX = mapRect_.left() + mapRect_.width() / 2.0;
    double centerY = mapRect_.top() + mapRect_.height() / 2.0;

    double wx = mapWidth_ / 2.0 + (screenPos.x() - centerX) / scale;
    double wy = mapHeight_ / 2.0 - (screenPos.y() - centerY) / scale;
    return QPointF(wx, wy);
}

double MapEditorCanvas::worldToScreenLength(double worldLength) const {
    double scaleX = mapRect_.width() / mapWidth_;
    double scaleY = mapRect_.height() / mapHeight_;
    double scale = qMin(scaleX, scaleY);
    return worldLength * scale;
}

int MapEditorCanvas::findPointAt(const QPointF& screenPos, double tolerance) const {
    if (!mapData_) return -1;

    QVector<MapData::Point> points = mapData_->getAllPoints();
    for (const MapData::Point& point : points) {
        QPointF pScreen = worldToScreen(QPointF(point.x, point.y));
        double dx = screenPos.x() - pScreen.x();
        double dy = screenPos.y() - pScreen.y();
        if (dx * dx + dy * dy <= tolerance * tolerance) {
            return point.id;
        }
    }
    return -1;
}

bool MapEditorCanvas::findEdgeAt(const QPointF& screenPos, int& outStartId, int& outEndId, double tolerance) const {
    if (!mapData_) return false;

    QVector<MapData::Edge> edges = mapData_->getAllEdges();
    for (const MapData::Edge& edge : edges) {
        const MapData::Point* startPoint = mapData_->getPoint(edge.startId);
        const MapData::Point* endPoint = mapData_->getPoint(edge.endId);
        if (!startPoint || !endPoint) continue;

        QPointF startScreen = worldToScreen(QPointF(startPoint->x, startPoint->y));
        QPointF endScreen = worldToScreen(QPointF(endPoint->x, endPoint->y));

        QPointF lineVec = endScreen - startScreen;
        QPointF toPoint = screenPos - startScreen;
        double lineLen = std::sqrt(lineVec.x() * lineVec.x() + lineVec.y() * lineVec.y());
        if (lineLen < 0.1) continue;

        double t = (toPoint.x() * lineVec.x() + toPoint.y() * lineVec.y()) / (lineLen * lineLen);
        t = qBound(0.0, t, 1.0);

        QPointF closest = startScreen + t * lineVec;
        double dx = screenPos.x() - closest.x();
        double dy = screenPos.y() - closest.y();
        if (dx * dx + dy * dy <= tolerance * tolerance) {
            outStartId = edge.startId;
            outEndId = edge.endId;
            return true;
        }
    }
    return false;
}

void MapEditorCanvas::mousePressEvent(QMouseEvent* event) {
    if (!mapRect_.contains(event->pos()) || !mapData_) {
        QWidget::mousePressEvent(event);
        return;
    }

    QPointF worldPos = screenToWorld(event->pos());
    int pointId = findPointAt(event->pos());
    int edgeStartId, edgeEndId;
    bool hitEdge = findEdgeAt(event->pos(), edgeStartId, edgeEndId);

    if (event->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(event);
        return;
    }

    switch (editMode_) {
    case MODE_ADD_POINT: {
        double x = qBound(0.0, worldPos.x(), mapWidth_);
        double y = qBound(0.0, worldPos.y(), mapHeight_);
        int newId = mapData_->getNextPointId();
        MapData::Point newPoint("点位" + QString::number(newId), newId,
                               x, y, 0.0, MapData::POINT_PATH);
        mapData_->addPoint(newPoint);
        emit mapDataChanged();
        break;
    }
    case MODE_ADD_EDGE: {
        if (pointId >= 0) {
            if (pendingEdgeStartId_ < 0) {
                pendingEdgeStartId_ = pointId;
            } else {
                if (pendingEdgeStartId_ != pointId) {
                    const MapData::Point* startPt = mapData_->getPoint(pendingEdgeStartId_);
                    const MapData::Point* endPt = mapData_->getPoint(pointId);
                    if (startPt && endPt) {
                        MapData::Edge newEdge(pendingEdgeStartId_, startPt->name,
                                              pointId, endPt->name,
                                              MapData::NAV_FORWARD, MapData::DRIVE_STRAIGHT);
                        mapData_->addEdge(newEdge);
                        emit mapDataChanged();
                    }
                }
                pendingEdgeStartId_ = -1;
            }
        }
        break;
    }
    case MODE_REMOVE_POINT: {
        if (pointId >= 0) {
            mapData_->removePoint(pointId);
            emit mapDataChanged();
        }
        break;
    }
    case MODE_REMOVE_EDGE: {
        if (hitEdge) {
            mapData_->removeEdge(edgeStartId, edgeEndId);
            emit mapDataChanged();
        }
        break;
    }
    default:
        break;
    }

    update();
    QWidget::mousePressEvent(event);
}

void MapEditorCanvas::mouseMoveEvent(QMouseEvent* event) {
    if (editMode_ == MODE_ADD_EDGE && pendingEdgeStartId_ >= 0) {
        update();
    }
    QWidget::mouseMoveEvent(event);
}
