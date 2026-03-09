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
#include <QButtonGroup>
#include <QScrollArea>
#include "../core/mapdata.h"
#include "mapeditorcanvas.h"

/**
 * @brief 地图编辑界面 - 左侧工具栏 + 中央画布，通过点击地图添加点位和边
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

    // 设置地图尺寸（从主窗口传入）
    void setMapSize(double width, double height);

signals:
    void mapDataChanged(const MapData& data);

private slots:
    // 文件操作
    void onLoadMap();
    void onSaveMap();
    void onSaveAsMap();

    // 工具栏模式切换
    void onEditModeChanged(int mode);

    // 画布数据变化
    void onCanvasMapDataChanged();

    // 点位操作（表格）
    void onRemovePointFromTable();
    void onPointSelectionChanged();
    void onPointDataChanged(int row, int column);

    // 边操作（表格）
    void onRemoveEdgeFromTable();
    void onEdgeSelectionChanged();
    void onEdgeDataChanged(int row, int column);

    // 格式选择
    void onFormatChanged(int index);

private:
    // 主布局：左侧工具栏 + 中央画布
    QHBoxLayout* mainLayout_;
    QWidget* leftPanel_;
    MapEditorCanvas* canvas_;

    // 顶部文件操作
    QHBoxLayout* buttonLayout_;
    QPushButton* loadBtn_;
    QPushButton* saveBtn_;
    QPushButton* saveAsBtn_;
    QPushButton* closeBtn_;
    QComboBox* formatCombo_;
    QLabel* formatLabel_;

    // 左侧工具栏
    QButtonGroup* toolButtonGroup_;
    QPushButton* addPointBtn_;
    QPushButton* addEdgeBtn_;
    QPushButton* removePointBtn_;
    QPushButton* removeEdgeBtn_;
    QPushButton* selectModeBtn_;

    // 点位列表（可折叠，用于查看/编辑属性）
    QGroupBox* pointGroup_;
    QTableWidget* pointTable_;
    QPushButton* removePointTableBtn_;

    // 边列表
    QGroupBox* edgeGroup_;
    QTableWidget* edgeTable_;
    QPushButton* removeEdgeTableBtn_;

    // 数据
    MapData mapData_;
    QString currentFilePath_;
    bool isUpdating_;

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
