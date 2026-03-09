#include "mapeditor.h"
#include <QHeaderView>
#include <QSpinBox>
#include <QAbstractButton>
#include <QDebug>
#include <QFileInfo>
#include <QSplitter>

MapEditor::MapEditor(QWidget* parent)
    : QDialog(parent), isUpdating_(false) {
    setWindowTitle("地图编辑器");
    setMinimumSize(1200, 750);
    resize(1300, 800);
    setupUI();
    updatePointTable();
    updateEdgeTable();
}

MapEditor::~MapEditor() {}

void MapEditor::setupUI() {
    QVBoxLayout* topLayout = new QVBoxLayout(this);

    // 顶部：文件操作
    buttonLayout_ = new QHBoxLayout();
    formatLabel_ = new QLabel("保存格式:", this);
    formatCombo_ = new QComboBox(this);
    formatCombo_->addItem("JSON (*.json)", "json");
    formatCombo_->addItem("XML (*.xml)", "xml");
    formatCombo_->addItem("二进制 (*.map)", "map");
    formatCombo_->setCurrentIndex(0);

    loadBtn_ = new QPushButton("读取地图", this);
    saveBtn_ = new QPushButton("保存地图", this);
    saveAsBtn_ = new QPushButton("另存为", this);
    closeBtn_ = new QPushButton("关闭", this);

    buttonLayout_->addWidget(formatLabel_);
    buttonLayout_->addWidget(formatCombo_);
    buttonLayout_->addStretch();
    buttonLayout_->addWidget(loadBtn_);
    buttonLayout_->addWidget(saveBtn_);
    buttonLayout_->addWidget(saveAsBtn_);
    buttonLayout_->addWidget(closeBtn_);
    topLayout->addLayout(buttonLayout_);

    // 中央区域：左侧工具栏 + 画布
    QSplitter* splitter = new QSplitter(Qt::Horizontal, this);

    leftPanel_ = new QWidget(this);
    QVBoxLayout* leftLayout = new QVBoxLayout(leftPanel_);
    leftLayout->setContentsMargins(0, 0, 0, 0);

    // 左侧工具栏
    QGroupBox* toolGroup = new QGroupBox("编辑工具", this);
    QVBoxLayout* toolLayout = new QVBoxLayout();
    toolButtonGroup_ = new QButtonGroup(this);

    selectModeBtn_ = new QPushButton("选择模式", this);
    selectModeBtn_->setCheckable(true);
    selectModeBtn_->setChecked(true);
    toolButtonGroup_->addButton(selectModeBtn_, MapEditorCanvas::MODE_SELECT);

    addPointBtn_ = new QPushButton("添加点位", this);
    addPointBtn_->setCheckable(true);
    addPointBtn_->setToolTip("点击地图任意位置添加点位");
    toolButtonGroup_->addButton(addPointBtn_, MapEditorCanvas::MODE_ADD_POINT);

    addEdgeBtn_ = new QPushButton("添加边", this);
    addEdgeBtn_->setCheckable(true);
    addEdgeBtn_->setToolTip("先点击起点，再点击终点添加边");
    toolButtonGroup_->addButton(addEdgeBtn_, MapEditorCanvas::MODE_ADD_EDGE);

    removePointBtn_ = new QPushButton("删除点位", this);
    removePointBtn_->setCheckable(true);
    removePointBtn_->setToolTip("点击地图上的点位删除");
    toolButtonGroup_->addButton(removePointBtn_, MapEditorCanvas::MODE_REMOVE_POINT);

    removeEdgeBtn_ = new QPushButton("删除边", this);
    removeEdgeBtn_->setCheckable(true);
    removeEdgeBtn_->setToolTip("点击地图上的边删除");
    toolButtonGroup_->addButton(removeEdgeBtn_, MapEditorCanvas::MODE_REMOVE_EDGE);

    toolLayout->addWidget(selectModeBtn_);
    toolLayout->addWidget(addPointBtn_);
    toolLayout->addWidget(addEdgeBtn_);
    toolLayout->addWidget(removePointBtn_);
    toolLayout->addWidget(removeEdgeBtn_);
    toolLayout->addStretch();
    toolGroup->setLayout(toolLayout);
    leftLayout->addWidget(toolGroup);

    // 点位列表
    pointGroup_ = new QGroupBox("点位列表", this);
    QVBoxLayout* pointLayout = new QVBoxLayout();
    removePointTableBtn_ = new QPushButton("删除选中", this);
    removePointTableBtn_->setEnabled(false);
    pointLayout->addWidget(removePointTableBtn_);

    pointTable_ = new QTableWidget(this);
    pointTable_->setColumnCount(6);
    pointTable_->setHorizontalHeaderLabels(QStringList() << "ID" << "名称" << "X (m)" << "Y (m)" << "角度 (rad)" << "类型");
    pointTable_->horizontalHeader()->setStretchLastSection(true);
    pointTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    pointTable_->setSelectionMode(QAbstractItemView::SingleSelection);
    pointTable_->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked);
    pointTable_->setMaximumHeight(180);
    pointLayout->addWidget(pointTable_);
    pointGroup_->setLayout(pointLayout);
    leftLayout->addWidget(pointGroup_);

    // 边列表
    edgeGroup_ = new QGroupBox("边列表", this);
    QVBoxLayout* edgeLayout = new QVBoxLayout();
    removeEdgeTableBtn_ = new QPushButton("删除选中", this);
    removeEdgeTableBtn_->setEnabled(false);
    edgeLayout->addWidget(removeEdgeTableBtn_);

    edgeTable_ = new QTableWidget(this);
    edgeTable_->setColumnCount(6);
    edgeTable_->setHorizontalHeaderLabels(QStringList() << "起点ID" << "起点名称" << "终点ID" << "终点名称" << "导航模式" << "行驶模式");
    edgeTable_->horizontalHeader()->setStretchLastSection(true);
    edgeTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    edgeTable_->setSelectionMode(QAbstractItemView::SingleSelection);
    edgeTable_->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked);
    edgeTable_->setMaximumHeight(180);
    edgeLayout->addWidget(edgeTable_);
    edgeGroup_->setLayout(edgeLayout);
    leftLayout->addWidget(edgeGroup_);

    leftLayout->addStretch();
    leftPanel_->setMaximumWidth(320);
    splitter->addWidget(leftPanel_);

    // 中央画布
    canvas_ = new MapEditorCanvas(this);
    canvas_->setMapData(&mapData_);
    connect(canvas_, &MapEditorCanvas::mapDataChanged, this, &MapEditor::onCanvasMapDataChanged);
    splitter->addWidget(canvas_);

    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    topLayout->addWidget(splitter);

    // 连接信号
    connect(loadBtn_, &QPushButton::clicked, this, &MapEditor::onLoadMap);
    connect(saveBtn_, &QPushButton::clicked, this, &MapEditor::onSaveMap);
    connect(saveAsBtn_, &QPushButton::clicked, this, &MapEditor::onSaveAsMap);
    connect(closeBtn_, &QPushButton::clicked, this, &QDialog::accept);

    connect(toolButtonGroup_, QOverload<QAbstractButton*>::of(&QButtonGroup::buttonClicked),
            this, [this](QAbstractButton* btn) {
        int id = toolButtonGroup_->id(btn);
        onEditModeChanged(id);
    });

    connect(removePointTableBtn_, &QPushButton::clicked, this, &MapEditor::onRemovePointFromTable);
    connect(pointTable_, &QTableWidget::itemSelectionChanged, this, &MapEditor::onPointSelectionChanged);
    connect(pointTable_, &QTableWidget::cellChanged, this, &MapEditor::onPointDataChanged);

    connect(removeEdgeTableBtn_, &QPushButton::clicked, this, &MapEditor::onRemoveEdgeFromTable);
    connect(edgeTable_, &QTableWidget::itemSelectionChanged, this, &MapEditor::onEdgeSelectionChanged);
    connect(edgeTable_, &QTableWidget::cellChanged, this, &MapEditor::onEdgeDataChanged);

    connect(formatCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MapEditor::onFormatChanged);
}

void MapEditor::setMapSize(double width, double height) {
    canvas_->setMapSize(width, height);
}

void MapEditor::onEditModeChanged(int mode) {
    canvas_->setEditMode(static_cast<MapEditorCanvas::EditMode>(mode));
}

void MapEditor::onCanvasMapDataChanged() {
    updatePointTable();
    updateEdgeTable();
    emit mapDataChanged(mapData_);
}

void MapEditor::onFormatChanged(int index) {
    Q_UNUSED(index);
}

void MapEditor::setMapData(const MapData& data) {
    mapData_ = data;
    canvas_->setMapData(&mapData_);
    updatePointTable();
    updateEdgeTable();
}

void MapEditor::updatePointTable() {
    isUpdating_ = true;
    pointTable_->setRowCount(0);

    QVector<MapData::Point> points = mapData_.getAllPoints();
    pointTable_->setRowCount(points.size());

    for (int i = 0; i < points.size(); ++i) {
        const MapData::Point& point = points[i];
        updatePointRow(i, point);
    }

    isUpdating_ = false;
}

void MapEditor::updatePointRow(int row, const MapData::Point& point) {
    QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(point.id));
    idItem->setFlags(idItem->flags() & ~Qt::ItemIsEditable);
    pointTable_->setItem(row, 0, idItem);

    pointTable_->setItem(row, 1, new QTableWidgetItem(point.name));
    pointTable_->setItem(row, 2, new QTableWidgetItem(QString::number(point.x, 'f', 2)));
    pointTable_->setItem(row, 3, new QTableWidgetItem(QString::number(point.y, 'f', 2)));
    pointTable_->setItem(row, 4, new QTableWidgetItem(QString::number(point.theta, 'f', 3)));

    QComboBox* typeCombo = new QComboBox();
    typeCombo->addItem("工位点", MapData::POINT_WORKSTATION);
    typeCombo->addItem("路径点", MapData::POINT_PATH);
    typeCombo->addItem("休息点", MapData::POINT_REST);
    typeCombo->setCurrentIndex(static_cast<int>(point.type));
    pointTable_->setCellWidget(row, 5, typeCombo);
    connect(typeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, row]() { onPointDataChanged(row, 5); });
}

void MapEditor::updateEdgeTable() {
    isUpdating_ = true;
    edgeTable_->setRowCount(0);

    QVector<MapData::Edge> edges = mapData_.getAllEdges();
    edgeTable_->setRowCount(edges.size());

    for (int i = 0; i < edges.size(); ++i) {
        const MapData::Edge& edge = edges[i];
        updateEdgeRow(i, edge);
    }

    isUpdating_ = false;
}

void MapEditor::updateEdgeRow(int row, const MapData::Edge& edge) {
    QVector<MapData::Point> points = mapData_.getAllPoints();

    QComboBox* startIdCombo = new QComboBox();
    for (const MapData::Point& point : points) {
        QString itemText = QString("%1 - %2").arg(point.id).arg(point.name);
        startIdCombo->addItem(itemText, point.id);
        if (point.id == edge.startId) {
            startIdCombo->setCurrentIndex(startIdCombo->count() - 1);
        }
    }
    edgeTable_->setCellWidget(row, 0, startIdCombo);
    connect(startIdCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, row]() { onEdgeDataChanged(row, 0); });

    QTableWidgetItem* startNameItem = new QTableWidgetItem(edge.startName);
    startNameItem->setFlags(startNameItem->flags() & ~Qt::ItemIsEditable);
    edgeTable_->setItem(row, 1, startNameItem);

    QComboBox* endIdCombo = new QComboBox();
    for (const MapData::Point& point : points) {
        QString itemText = QString("%1 - %2").arg(point.id).arg(point.name);
        endIdCombo->addItem(itemText, point.id);
        if (point.id == edge.endId) {
            endIdCombo->setCurrentIndex(endIdCombo->count() - 1);
        }
    }
    edgeTable_->setCellWidget(row, 2, endIdCombo);
    connect(endIdCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, row]() { onEdgeDataChanged(row, 2); });

    QTableWidgetItem* endNameItem = new QTableWidgetItem(edge.endName);
    endNameItem->setFlags(endNameItem->flags() & ~Qt::ItemIsEditable);
    edgeTable_->setItem(row, 3, endNameItem);

    QComboBox* navCombo = new QComboBox();
    navCombo->addItem("前进", MapData::NAV_FORWARD);
    navCombo->addItem("后退", MapData::NAV_BACKWARD);
    navCombo->setCurrentIndex(static_cast<int>(edge.navMode));
    edgeTable_->setCellWidget(row, 4, navCombo);
    connect(navCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, row]() { onEdgeDataChanged(row, 4); });

    QComboBox* driveCombo = new QComboBox();
    driveCombo->addItem("直线", MapData::DRIVE_STRAIGHT);
    driveCombo->addItem("曲线", MapData::DRIVE_CURVE);
    driveCombo->setCurrentIndex(static_cast<int>(edge.driveMode));
    edgeTable_->setCellWidget(row, 5, driveCombo);
    connect(driveCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, row]() { onEdgeDataChanged(row, 5); });
}

void MapEditor::onLoadMap() {
    QString filePath = QFileDialog::getOpenFileName(this, "读取地图",
                                                     currentFilePath_.isEmpty() ? "." : currentFilePath_,
                                                     getFileFilter());
    if (!filePath.isEmpty()) {
        if (loadMapFile(filePath)) {
            currentFilePath_ = filePath;
            updatePointTable();
            updateEdgeTable();
            QMessageBox::information(this, "成功", "地图加载成功！");
        } else {
            QMessageBox::warning(this, "错误", "地图加载失败！");
        }
    }
}

void MapEditor::onSaveMap() {
    if (currentFilePath_.isEmpty()) {
        onSaveAsMap();
    } else {
        if (saveMapFile(currentFilePath_)) {
            QMessageBox::information(this, "成功", "地图保存成功！");
            emit mapDataChanged(mapData_);
        } else {
            QMessageBox::warning(this, "错误", "地图保存失败！");
        }
    }
}

void MapEditor::onSaveAsMap() {
    QString filePath = QFileDialog::getSaveFileName(this, "保存地图",
                                                    currentFilePath_.isEmpty() ? "map." + getFileExtension() : currentFilePath_,
                                                    getFileFilter());
    if (!filePath.isEmpty()) {
        if (saveMapFile(filePath)) {
            currentFilePath_ = filePath;
            QMessageBox::information(this, "成功", "地图保存成功！");
            emit mapDataChanged(mapData_);
        } else {
            QMessageBox::warning(this, "错误", "地图保存失败！");
        }
    }
}

void MapEditor::onRemovePointFromTable() {
    int row = pointTable_->currentRow();
    if (row >= 0) {
        QTableWidgetItem* idItem = pointTable_->item(row, 0);
        if (idItem) {
            int id = idItem->text().toInt();
            mapData_.removePoint(id);
            updatePointTable();
            updateEdgeTable();
            canvas_->update();
            emit mapDataChanged(mapData_);
        }
    }
}

void MapEditor::onPointSelectionChanged() {
    bool hasSelection = pointTable_->currentRow() >= 0;
    removePointTableBtn_->setEnabled(hasSelection);
}

void MapEditor::onPointDataChanged(int row, int column) {
    if (isUpdating_) return;

    QTableWidgetItem* idItem = pointTable_->item(row, 0);
    if (!idItem) return;

    int id = idItem->text().toInt();
    MapData::Point* point = mapData_.getPoint(id);
    if (!point) return;

    if (column == 1) {
        point->name = pointTable_->item(row, column)->text();
    } else if (column == 2) {
        point->x = pointTable_->item(row, column)->text().toDouble();
    } else if (column == 3) {
        point->y = pointTable_->item(row, column)->text().toDouble();
    } else if (column == 4) {
        point->theta = pointTable_->item(row, column)->text().toDouble();
    } else if (column == 5) {
        QComboBox* combo = qobject_cast<QComboBox*>(pointTable_->cellWidget(row, column));
        if (combo) {
            point->type = static_cast<MapData::PointType>(combo->currentIndex());
        }
    }

    mapData_.updatePoint(id, *point);
    canvas_->update();
    emit mapDataChanged(mapData_);
}

void MapEditor::onRemoveEdgeFromTable() {
    int row = edgeTable_->currentRow();
    if (row >= 0) {
        int startId = 0;
        int endId = 0;

        QComboBox* startIdCombo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, 0));
        if (startIdCombo) {
            startId = startIdCombo->currentData().toInt();
        } else {
            QTableWidgetItem* item = edgeTable_->item(row, 0);
            if (item) startId = item->text().toInt();
        }

        QComboBox* endIdCombo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, 2));
        if (endIdCombo) {
            endId = endIdCombo->currentData().toInt();
        } else {
            QTableWidgetItem* item = edgeTable_->item(row, 2);
            if (item) endId = item->text().toInt();
        }

        if (startId != 0 && endId != 0) {
            mapData_.removeEdge(startId, endId);
            updateEdgeTable();
            canvas_->update();
            emit mapDataChanged(mapData_);
        }
    }
}

void MapEditor::onEdgeSelectionChanged() {
    bool hasSelection = edgeTable_->currentRow() >= 0;
    removeEdgeTableBtn_->setEnabled(hasSelection);
}

void MapEditor::onEdgeDataChanged(int row, int column) {
    if (isUpdating_) return;

    int startId = 0;
    int endId = 0;

    QComboBox* startIdCombo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, 0));
    if (startIdCombo) {
        startId = startIdCombo->currentData().toInt();
    } else {
        QTableWidgetItem* item = edgeTable_->item(row, 0);
        if (item) startId = item->text().toInt();
    }

    QComboBox* endIdCombo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, 2));
    if (endIdCombo) {
        endId = endIdCombo->currentData().toInt();
    } else {
        QTableWidgetItem* item = edgeTable_->item(row, 2);
        if (item) endId = item->text().toInt();
    }

    if (startId == 0 || endId == 0) return;

    const MapData::Point* startPoint = mapData_.getPoint(startId);
    const MapData::Point* endPoint = mapData_.getPoint(endId);

    if (!startPoint || !endPoint) {
        QMessageBox::warning(this, "错误", "起点或终点不存在！");
        updateEdgeTable();
        return;
    }

    QVector<MapData::Edge> edges = mapData_.getAllEdges();
    int oldStartId = 0;
    int oldEndId = 0;
    if (row < edges.size()) {
        oldStartId = edges[row].startId;
        oldEndId = edges[row].endId;
    }

    MapData::Edge edge(startId, startPoint->name, endId, endPoint->name,
                      MapData::NAV_FORWARD, MapData::DRIVE_STRAIGHT);

    if (oldStartId != 0 && oldEndId != 0) {
        for (const MapData::Edge& oldEdge : edges) {
            if (oldEdge.startId == oldStartId && oldEdge.endId == oldEndId) {
                edge.navMode = oldEdge.navMode;
                edge.driveMode = oldEdge.driveMode;
                break;
            }
        }
    }

    if (column == 4) {
        QComboBox* combo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, column));
        if (combo) edge.navMode = static_cast<MapData::NavigationMode>(combo->currentIndex());
    } else if (column == 5) {
        QComboBox* combo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, column));
        if (combo) edge.driveMode = static_cast<MapData::DriveMode>(combo->currentIndex());
    }

    if (oldStartId != 0 && oldEndId != 0 && (startId != oldStartId || endId != oldEndId)) {
        mapData_.removeEdge(oldStartId, oldEndId);
        mapData_.addEdge(edge);
    } else if (oldStartId != 0 && oldEndId != 0) {
        mapData_.updateEdge(startId, endId, edge);
    } else {
        mapData_.addEdge(edge);
    }

    updateEdgeTable();
    canvas_->update();
    emit mapDataChanged(mapData_);
}

QString MapEditor::getFileFilter() const {
    int index = formatCombo_->currentIndex();
    if (index == 0) return "JSON文件 (*.json);;所有文件 (*.*)";
    if (index == 1) return "XML文件 (*.xml);;所有文件 (*.*)";
    return "二进制文件 (*.map);;所有文件 (*.*)";
}

QString MapEditor::getFileExtension() const {
    int index = formatCombo_->currentIndex();
    if (index == 0) return "json";
    if (index == 1) return "xml";
    return "map";
}

bool MapEditor::loadMapFile(const QString& filePath) {
    QString ext = QFileInfo(filePath).suffix().toLower();

    if (ext == "json") return mapData_.loadFromJson(filePath);
    if (ext == "xml") return mapData_.loadFromXml(filePath);
    if (ext == "map") return mapData_.loadFromBinary(filePath);

    if (mapData_.loadFromJson(filePath)) return true;
    if (mapData_.loadFromXml(filePath)) return true;
    if (mapData_.loadFromBinary(filePath)) return true;
    return false;
}

bool MapEditor::saveMapFile(const QString& filePath) {
    QString ext = QFileInfo(filePath).suffix().toLower();

    if (ext == "json") return mapData_.saveToJson(filePath);
    if (ext == "xml") return mapData_.saveToXml(filePath);
    if (ext == "map") return mapData_.saveToBinary(filePath);

    int index = formatCombo_->currentIndex();
    QString newPath = filePath;
    if (!newPath.endsWith("." + getFileExtension())) {
        newPath += "." + getFileExtension();
    }

    if (index == 0) return mapData_.saveToJson(newPath);
    if (index == 1) return mapData_.saveToXml(newPath);
    return mapData_.saveToBinary(newPath);
}
