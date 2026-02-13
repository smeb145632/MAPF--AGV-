#include "mapdata.h"
#include <QDebug>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QFileInfo>
#include <algorithm>

// Point 序列化
QJsonObject MapData::Point::toJson() const {
    QJsonObject obj;
    obj["name"] = name;
    obj["id"] = id;
    obj["x"] = x;
    obj["y"] = y;
    obj["theta"] = theta;
    obj["type"] = static_cast<int>(type);
    return obj;
}

void MapData::Point::fromJson(const QJsonObject& json) {
    name = json["name"].toString();
    id = json["id"].toInt();
    x = json["x"].toDouble();
    y = json["y"].toDouble();
    theta = json["theta"].toDouble();
    type = static_cast<PointType>(json["type"].toInt());
}

// Edge 序列化
QJsonObject MapData::Edge::toJson() const {
    QJsonObject obj;
    obj["startId"] = startId;
    obj["startName"] = startName;
    obj["endId"] = endId;
    obj["endName"] = endName;
    obj["navMode"] = static_cast<int>(navMode);
    obj["driveMode"] = static_cast<int>(driveMode);
    return obj;
}

void MapData::Edge::fromJson(const QJsonObject& json) {
    startId = json["startId"].toInt();
    startName = json["startName"].toString();
    endId = json["endId"].toInt();
    endName = json["endName"].toString();
    navMode = static_cast<NavigationMode>(json["navMode"].toInt());
    driveMode = static_cast<DriveMode>(json["driveMode"].toInt());
}

// MapData 实现
MapData::MapData() {}

MapData::~MapData() {}

void MapData::addPoint(const Point& point) {
    // 检查ID是否已存在
    if (getPoint(point.id)) {
        qWarning() << "Point with ID" << point.id << "already exists";
        return;
    }
    points_.append(point);
}

void MapData::removePoint(int id) {
    int index = findPointIndex(id);
    if (index >= 0) {
        // 删除相关的边
        edges_.erase(std::remove_if(edges_.begin(), edges_.end(),
            [id](const Edge& e) { return e.startId == id || e.endId == id; }),
            edges_.end());
        points_.removeAt(index);
    }
}

void MapData::updatePoint(int id, const Point& point) {
    int index = findPointIndex(id);
    if (index >= 0) {
        points_[index] = point;
        // 更新相关边的名称
        for (Edge& edge : edges_) {
            if (edge.startId == id) {
                edge.startName = point.name;
            }
            if (edge.endId == id) {
                edge.endName = point.name;
            }
        }
    }
}

MapData::Point* MapData::getPoint(int id) {
    int index = findPointIndex(id);
    return index >= 0 ? &points_[index] : nullptr;
}

const MapData::Point* MapData::getPoint(int id) const {
    int index = findPointIndex(id);
    return index >= 0 ? &points_[index] : nullptr;
}

int MapData::getNextPointId() const {
    int maxId = 0;
    for (const Point& p : points_) {
        if (p.id > maxId) {
            maxId = p.id;
        }
    }
    return maxId + 1;
}

void MapData::addEdge(const Edge& edge) {
    // 检查起点和终点是否存在
    if (!getPoint(edge.startId) || !getPoint(edge.endId)) {
        qWarning() << "Cannot add edge: start or end point not found";
        return;
    }
    // 检查边是否已存在
    if (findEdgeIndex(edge.startId, edge.endId) >= 0) {
        qWarning() << "Edge from" << edge.startId << "to" << edge.endId << "already exists";
        return;
    }
    edges_.append(edge);
}

void MapData::removeEdge(int startId, int endId) {
    int index = findEdgeIndex(startId, endId);
    if (index >= 0) {
        edges_.removeAt(index);
    }
}

void MapData::updateEdge(int startId, int endId, const Edge& edge) {
    int index = findEdgeIndex(startId, endId);
    if (index >= 0) {
        edges_[index] = edge;
    }
}

QVector<MapData::Edge> MapData::getEdgesFrom(int startId) const {
    QVector<Edge> result;
    for (const Edge& edge : edges_) {
        if (edge.startId == startId) {
            result.append(edge);
        }
    }
    return result;
}

QVector<MapData::Edge> MapData::getEdgesTo(int endId) const {
    QVector<Edge> result;
    for (const Edge& edge : edges_) {
        if (edge.endId == endId) {
            result.append(edge);
        }
    }
    return result;
}

// JSON格式保存和加载
bool MapData::saveToJson(const QString& filePath) const {
    QJsonObject root;
    root["version"] = "1.0";
    root["pointCount"] = points_.size();
    root["edgeCount"] = edges_.size();
    
    // 保存点位
    QJsonArray pointsArray;
    for (const Point& point : points_) {
        pointsArray.append(point.toJson());
    }
    root["points"] = pointsArray;
    
    // 保存边
    QJsonArray edgesArray;
    for (const Edge& edge : edges_) {
        edgesArray.append(edge.toJson());
    }
    root["edges"] = edgesArray;
    
    QJsonDocument doc(root);
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "Cannot open file for writing:" << filePath;
        return false;
    }
    file.write(doc.toJson());
    file.close();
    return true;
}

bool MapData::loadFromJson(const QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Cannot open file for reading:" << filePath;
        return false;
    }
    
    QByteArray data = file.readAll();
    file.close();
    
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    if (error.error != QJsonParseError::NoError) {
        qWarning() << "JSON parse error:" << error.errorString();
        return false;
    }
    
    QJsonObject root = doc.object();
    clear();
    
    // 加载点位
    QJsonArray pointsArray = root["points"].toArray();
    for (const QJsonValue& value : pointsArray) {
        Point point;
        point.fromJson(value.toObject());
        points_.append(point);
    }
    
    // 加载边
    QJsonArray edgesArray = root["edges"].toArray();
    for (const QJsonValue& value : edgesArray) {
        Edge edge;
        edge.fromJson(value.toObject());
        edges_.append(edge);
    }
    
    return validate();
}

// XML格式保存和加载
bool MapData::saveToXml(const QString& filePath) const {
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "Cannot open file for writing:" << filePath;
        return false;
    }
    
    QXmlStreamWriter writer(&file);
    writer.setAutoFormatting(true);
    writer.writeStartDocument();
    writer.writeStartElement("MapData");
    writer.writeAttribute("version", "1.0");
    
    // 保存点位
    writer.writeStartElement("Points");
    for (const Point& point : points_) {
        writer.writeStartElement("Point");
        writer.writeAttribute("id", QString::number(point.id));
        writer.writeAttribute("name", point.name);
        writer.writeAttribute("x", QString::number(point.x));
        writer.writeAttribute("y", QString::number(point.y));
        writer.writeAttribute("theta", QString::number(point.theta));
        writer.writeAttribute("type", QString::number(static_cast<int>(point.type)));
        writer.writeEndElement();
    }
    writer.writeEndElement();
    
    // 保存边
    writer.writeStartElement("Edges");
    for (const Edge& edge : edges_) {
        writer.writeStartElement("Edge");
        writer.writeAttribute("startId", QString::number(edge.startId));
        writer.writeAttribute("startName", edge.startName);
        writer.writeAttribute("endId", QString::number(edge.endId));
        writer.writeAttribute("endName", edge.endName);
        writer.writeAttribute("navMode", QString::number(static_cast<int>(edge.navMode)));
        writer.writeAttribute("driveMode", QString::number(static_cast<int>(edge.driveMode)));
        writer.writeEndElement();
    }
    writer.writeEndElement();
    
    writer.writeEndElement();
    writer.writeEndDocument();
    file.close();
    return true;
}

bool MapData::loadFromXml(const QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Cannot open file for reading:" << filePath;
        return false;
    }
    
    QXmlStreamReader reader(&file);
    clear();
    
    Point currentPoint;
    Edge currentEdge;
    QString currentElement;
    
    while (!reader.atEnd()) {
        reader.readNext();
        
        if (reader.isStartElement()) {
            currentElement = reader.name().toString();
            
            if (currentElement == "Point") {
                QXmlStreamAttributes attrs = reader.attributes();
                currentPoint.id = attrs.value("id").toInt();
                currentPoint.name = attrs.value("name").toString();
                currentPoint.x = attrs.value("x").toDouble();
                currentPoint.y = attrs.value("y").toDouble();
                currentPoint.theta = attrs.value("theta").toDouble();
                currentPoint.type = static_cast<PointType>(attrs.value("type").toInt());
            } else if (currentElement == "Edge") {
                QXmlStreamAttributes attrs = reader.attributes();
                currentEdge.startId = attrs.value("startId").toInt();
                currentEdge.startName = attrs.value("startName").toString();
                currentEdge.endId = attrs.value("endId").toInt();
                currentEdge.endName = attrs.value("endName").toString();
                currentEdge.navMode = static_cast<NavigationMode>(attrs.value("navMode").toInt());
                currentEdge.driveMode = static_cast<DriveMode>(attrs.value("driveMode").toInt());
            }
        } else if (reader.isEndElement()) {
            QString elementName = reader.name().toString();
            if (elementName == "Point") {
                points_.append(currentPoint);
            } else if (elementName == "Edge") {
                edges_.append(currentEdge);
            }
        }
    }
    
    file.close();
    return !reader.hasError() && validate();
}

// 二进制格式保存和加载
bool MapData::saveToBinary(const QString& filePath) const {
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "Cannot open file for writing:" << filePath;
        return false;
    }
    
    QDataStream out(&file);
    out.setVersion(QDataStream::Qt_5_9);
    
    // 写入版本号
    out << QString("MAPDATA_V1.0");
    
    // 写入点位数量
    out << static_cast<quint32>(points_.size());
    for (const Point& point : points_) {
        out << point.name << point.id << point.x << point.y << point.theta 
            << static_cast<quint32>(point.type);
    }
    
    // 写入边数量
    out << static_cast<quint32>(edges_.size());
    for (const Edge& edge : edges_) {
        out << edge.startId << edge.startName << edge.endId << edge.endName
            << static_cast<quint32>(edge.navMode) << static_cast<quint32>(edge.driveMode);
    }
    
    file.close();
    return true;
}

bool MapData::loadFromBinary(const QString& filePath) {
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Cannot open file for reading:" << filePath;
        return false;
    }
    
    QDataStream in(&file);
    in.setVersion(QDataStream::Qt_5_9);
    
    clear();
    
    // 读取版本号
    QString version;
    in >> version;
    if (version != "MAPDATA_V1.0") {
        qWarning() << "Unsupported file version:" << version;
        file.close();
        return false;
    }
    
    // 读取点位
    quint32 pointCount;
    in >> pointCount;
    for (quint32 i = 0; i < pointCount; ++i) {
        Point point;
        quint32 type;
        in >> point.name >> point.id >> point.x >> point.y >> point.theta >> type;
        point.type = static_cast<PointType>(type);
        points_.append(point);
    }
    
    // 读取边
    quint32 edgeCount;
    in >> edgeCount;
    for (quint32 i = 0; i < edgeCount; ++i) {
        Edge edge;
        quint32 navMode, driveMode;
        in >> edge.startId >> edge.startName >> edge.endId >> edge.endName 
           >> navMode >> driveMode;
        edge.navMode = static_cast<NavigationMode>(navMode);
        edge.driveMode = static_cast<DriveMode>(driveMode);
        edges_.append(edge);
    }
    
    file.close();
    return validate();
}

void MapData::clear() {
    points_.clear();
    edges_.clear();
}

bool MapData::validate() const {
    // 检查所有边的起点和终点是否存在
    for (const Edge& edge : edges_) {
        if (!getPoint(edge.startId) || !getPoint(edge.endId)) {
            qWarning() << "Invalid edge: point not found";
            return false;
        }
    }
    return true;
}

int MapData::findPointIndex(int id) const {
    for (int i = 0; i < points_.size(); ++i) {
        if (points_[i].id == id) {
            return i;
        }
    }
    return -1;
}

int MapData::findEdgeIndex(int startId, int endId) const {
    for (int i = 0; i < edges_.size(); ++i) {
        if (edges_[i].startId == startId && edges_[i].endId == endId) {
            return i;
        }
    }
    return -1;
}

