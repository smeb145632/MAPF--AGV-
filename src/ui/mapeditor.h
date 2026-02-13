#ifndef MAPEDITOR_H
#define MAPEDITOR_H

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QHeaderView>
#include "../core/mapdata.h"

/**
 * @brief 地图编辑界面 - 管理地图点位和边的编辑
 */
class MapEditor : public QDialog {
    Q_OBJECT

public:
    explicit MapEditor(QWidget* parent = nullptr);
    ~MapEditor();
    
    // 加载和保存地图数据
    void setMapData(const MapData& data);
    MapData getMapData() const { return mapData_; }
    
    // 设置当前地图文件路径
    void setCurrentFilePath(const QString& path) { currentFilePath_ = path; }
    QString getCurrentFilePath() const { return currentFilePath_; }

signals:
    void mapDataChanged(const MapData& data);

private slots:
    // 文件操作
    void onLoadMap();
    void onSaveMap();
    void onSaveAsMap();
    
    // 点位操作
    void onAddPoint();
    void onRemovePoint();
    void onPointSelectionChanged();
    void onPointDataChanged(int row, int column);
    
    // 边操作
    void onAddEdge();
    void onRemoveEdge();
    void onEdgeSelectionChanged();
    void onEdgeDataChanged(int row, int column);
    
    // 格式选择
    void onFormatChanged(int index);

private:
    // UI组件
    QVBoxLayout* mainLayout_;
    QHBoxLayout* buttonLayout_;
    QPushButton* loadBtn_;
    QPushButton* saveBtn_;
    QPushButton* saveAsBtn_;
    QPushButton* closeBtn_;
    
    // 点位编辑区域
    QGroupBox* pointGroup_;
    QTableWidget* pointTable_;
    QPushButton* addPointBtn_;
    QPushButton* removePointBtn_;
    
    // 边编辑区域
    QGroupBox* edgeGroup_;
    QTableWidget* edgeTable_;
    QPushButton* addEdgeBtn_;
    QPushButton* removeEdgeBtn_;
    
    // 格式选择
    QComboBox* formatCombo_;
    QLabel* formatLabel_;
    
    // 数据
    MapData mapData_;
    QString currentFilePath_;
    bool isUpdating_;  // 防止递归更新
    
    // 辅助函数
    void setupUI();
    void updatePointTable();
    void updateEdgeTable();
    void updatePointRow(int row, const MapData::Point& point);
    void updateEdgeRow(int row, const MapData::Edge& edge);
    QString getFileFilter() const;
    QString getFileExtension() const;
    bool loadMapFile(const QString& filePath);
    bool saveMapFile(const QString& filePath);
};

#endif // MAPEDITOR_H

