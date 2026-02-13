#ifndef MAPDATA_H
#define MAPDATA_H

#include <QString>
#include <QPointF>
#include <QVector>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <QDataStream>

/**
 * @brief 地图数据类 - 管理地图的点位和边信息
 */
class MapData {
public:
    // 点位类型枚举
    enum PointType {
        POINT_WORKSTATION = 0,  // 工位点
        POINT_PATH = 1,         // 路径点
        POINT_REST = 2          // 休息点
    };
    
    // 导航模式枚举
    enum NavigationMode {
        NAV_FORWARD = 0,        // 前进
        NAV_BACKWARD = 1        // 后退
    };
    
    // 行驶模式枚举
    enum DriveMode {
        DRIVE_STRAIGHT = 0,     // 直线
        DRIVE_CURVE = 1         // 曲线
    };
    
    // 点位信息结构
    struct Point {
        QString name;           // 点位名称
        int id;                 // 点位ID
        double x;               // X坐标 (m)
        double y;               // Y坐标 (m)
        double theta;           // 朝向角度 (rad)
        PointType type;          // 点位类型
        
        Point() : id(0), x(0.0), y(0.0), theta(0.0), type(POINT_PATH) {}
        Point(const QString& n, int i, double px, double py, double t, PointType pt)
            : name(n), id(i), x(px), y(py), theta(t), type(pt) {}
        
        // JSON序列化
        QJsonObject toJson() const;
        void fromJson(const QJsonObject& json);
    };
    
    // 有向边信息结构
    struct Edge {
        int startId;            // 起点ID
        QString startName;      // 起点名称
        int endId;              // 终点ID
        QString endName;        // 终点名称
        NavigationMode navMode; // 导航模式（前进/后退）
        DriveMode driveMode;    // 行驶模式（直线/曲线）
        
        Edge() : startId(0), endId(0), navMode(NAV_FORWARD), driveMode(DRIVE_STRAIGHT) {}
        Edge(int sid, const QString& sname, int eid, const QString& ename, 
             NavigationMode nav, DriveMode drive)
            : startId(sid), startName(sname), endId(eid), endName(ename),
              navMode(nav), driveMode(drive) {}
        
        // JSON序列化
        QJsonObject toJson() const;
        void fromJson(const QJsonObject& json);
    };
    
    MapData();
    ~MapData();
    
    // 点位管理
    void addPoint(const Point& point);
    void removePoint(int id);
    void updatePoint(int id, const Point& point);
    Point* getPoint(int id);
    const Point* getPoint(int id) const;
    QVector<Point> getAllPoints() const { return points_; }
    int getNextPointId() const;
    
    // 边管理
    void addEdge(const Edge& edge);
    void removeEdge(int startId, int endId);
    void updateEdge(int startId, int endId, const Edge& edge);
    QVector<Edge> getEdgesFrom(int startId) const;
    QVector<Edge> getEdgesTo(int endId) const;
    QVector<Edge> getAllEdges() const { return edges_; }
    
    // 文件操作
    bool loadFromJson(const QString& filePath);
    bool saveToJson(const QString& filePath) const;
    bool loadFromXml(const QString& filePath);
    bool saveToXml(const QString& filePath) const;
    bool loadFromBinary(const QString& filePath);
    bool saveToBinary(const QString& filePath) const;
    
    // 清空数据
    void clear();
    
    // 验证数据
    bool validate() const;
    
    // 获取统计信息
    int getPointCount() const { return points_.size(); }
    int getEdgeCount() const { return edges_.size(); }
    
private:
    QVector<Point> points_;     // 点位列表
    QVector<Edge> edges_;       // 边列表
    
    // 辅助函数
    int findPointIndex(int id) const;
    int findEdgeIndex(int startId, int endId) const;
};

#endif // MAPDATA_H

