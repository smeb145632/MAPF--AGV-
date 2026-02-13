#include "mapeditor.h"
#include <QHeaderView>
#include <QSpinBox>
#include <QDebug>

MapEditor::MapEditor(QWidget* parent)
    : QDialog(parent), isUpdating_(false) {
    setWindowTitle("地图编辑器");
    setMinimumSize(1000, 700);
    setupUI();
    updatePointTable();
    updateEdgeTable();
}

MapEditor::~MapEditor() {}

void MapEditor::setupUI() {
    mainLayout_ = new QVBoxLayout(this);
    
    // 文件操作按钮
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
    
    mainLayout_->addLayout(buttonLayout_);
    
    // 点位编辑区域
    pointGroup_ = new QGroupBox("点位管理", this);
    QVBoxLayout* pointLayout = new QVBoxLayout();
    
    QHBoxLayout* pointButtonLayout = new QHBoxLayout();
    addPointBtn_ = new QPushButton("添加点位", this);
    removePointBtn_ = new QPushButton("删除点位", this);
    removePointBtn_->setEnabled(false);
    pointButtonLayout->addWidget(addPointBtn_);
    pointButtonLayout->addWidget(removePointBtn_);
    pointButtonLayout->addStretch();
    
    pointTable_ = new QTableWidget(this);
    pointTable_->setColumnCount(6);
    pointTable_->setHorizontalHeaderLabels(QStringList() << "ID" << "名称" << "X (m)" << "Y (m)" << "角度 (rad)" << "类型");
    pointTable_->horizontalHeader()->setStretchLastSection(true);
    pointTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    pointTable_->setSelectionMode(QAbstractItemView::SingleSelection);
    pointTable_->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked);
    
    pointLayout->addLayout(pointButtonLayout);
    pointLayout->addWidget(pointTable_);
    pointGroup_->setLayout(pointLayout);
    
    mainLayout_->addWidget(pointGroup_);
    
    // 边编辑区域
    edgeGroup_ = new QGroupBox("边管理", this);
    QVBoxLayout* edgeLayout = new QVBoxLayout();
    
    QHBoxLayout* edgeButtonLayout = new QHBoxLayout();
    addEdgeBtn_ = new QPushButton("添加边", this);
    removeEdgeBtn_ = new QPushButton("删除边", this);
    removeEdgeBtn_->setEnabled(false);
    edgeButtonLayout->addWidget(addEdgeBtn_);
    edgeButtonLayout->addWidget(removeEdgeBtn_);
    edgeButtonLayout->addStretch();
    
    edgeTable_ = new QTableWidget(this);
    edgeTable_->setColumnCount(6);
    edgeTable_->setHorizontalHeaderLabels(QStringList() << "起点ID" << "起点名称" << "终点ID" << "终点名称" << "导航模式" << "行驶模式");
    edgeTable_->horizontalHeader()->setStretchLastSection(true);
    edgeTable_->setSelectionBehavior(QAbstractItemView::SelectRows);
    edgeTable_->setSelectionMode(QAbstractItemView::SingleSelection);
    edgeTable_->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::SelectedClicked);
    
    edgeLayout->addLayout(edgeButtonLayout);
    edgeLayout->addWidget(edgeTable_);
    edgeGroup_->setLayout(edgeLayout);
    
    mainLayout_->addWidget(edgeGroup_);
    
    // 连接信号
    connect(loadBtn_, &QPushButton::clicked, this, &MapEditor::onLoadMap);
    connect(saveBtn_, &QPushButton::clicked, this, &MapEditor::onSaveMap);
    connect(saveAsBtn_, &QPushButton::clicked, this, &MapEditor::onSaveAsMap);
    connect(closeBtn_, &QPushButton::clicked, this, &QDialog::accept);
    
    connect(addPointBtn_, &QPushButton::clicked, this, &MapEditor::onAddPoint);
    connect(removePointBtn_, &QPushButton::clicked, this, &MapEditor::onRemovePoint);
    connect(pointTable_, &QTableWidget::itemSelectionChanged, this, &MapEditor::onPointSelectionChanged);
    connect(pointTable_, &QTableWidget::cellChanged, this, &MapEditor::onPointDataChanged);
    
    connect(addEdgeBtn_, &QPushButton::clicked, this, &MapEditor::onAddEdge);
    connect(removeEdgeBtn_, &QPushButton::clicked, this, &MapEditor::onRemoveEdge);
    connect(edgeTable_, &QTableWidget::itemSelectionChanged, this, &MapEditor::onEdgeSelectionChanged);
    connect(edgeTable_, &QTableWidget::cellChanged, this, &MapEditor::onEdgeDataChanged);
    
    connect(formatCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &MapEditor::onFormatChanged);
}

void MapEditor::onFormatChanged(int index) {
    // 格式改变时的处理（可以在这里更新UI提示等）
    Q_UNUSED(index);
    // 目前不需要特殊处理，保存时会根据选择的格式保存
}

void MapEditor::setMapData(const MapData& data) {
    mapData_ = data;
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
    // ID
    QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(point.id));
    idItem->setFlags(idItem->flags() & ~Qt::ItemIsEditable);
    pointTable_->setItem(row, 0, idItem);
    
    // 名称
    pointTable_->setItem(row, 1, new QTableWidgetItem(point.name));
    
    // X坐标
    QTableWidgetItem* xItem = new QTableWidgetItem(QString::number(point.x, 'f', 2));
    pointTable_->setItem(row, 2, xItem);
    
    // Y坐标
    QTableWidgetItem* yItem = new QTableWidgetItem(QString::number(point.y, 'f', 2));
    pointTable_->setItem(row, 3, yItem);
    
    // 角度
    QTableWidgetItem* thetaItem = new QTableWidgetItem(QString::number(point.theta, 'f', 3));
    pointTable_->setItem(row, 4, thetaItem);
    
    // 类型
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
    // 获取所有点位用于下拉框
    QVector<MapData::Point> points = mapData_.getAllPoints();
    
    // 起点ID（使用下拉框选择）
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
    
    // 起点名称（自动更新，不可编辑）
    QTableWidgetItem* startNameItem = new QTableWidgetItem(edge.startName);
    startNameItem->setFlags(startNameItem->flags() & ~Qt::ItemIsEditable);
    edgeTable_->setItem(row, 1, startNameItem);
    
    // 终点ID（使用下拉框选择）
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
    
    // 终点名称（自动更新，不可编辑）
    QTableWidgetItem* endNameItem = new QTableWidgetItem(edge.endName);
    endNameItem->setFlags(endNameItem->flags() & ~Qt::ItemIsEditable);
    edgeTable_->setItem(row, 3, endNameItem);
    
    // 导航模式
    QComboBox* navCombo = new QComboBox();
    navCombo->addItem("前进", MapData::NAV_FORWARD);
    navCombo->addItem("后退", MapData::NAV_BACKWARD);
    navCombo->setCurrentIndex(static_cast<int>(edge.navMode));
    edgeTable_->setCellWidget(row, 4, navCombo);
    connect(navCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, row]() { onEdgeDataChanged(row, 4); });
    
    // 行驶模式
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
            // 刷新显示
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

void MapEditor::onAddPoint() {
    int newId = mapData_.getNextPointId();
    MapData::Point newPoint("新点位", newId, 0.0, 0.0, 0.0, MapData::POINT_PATH);
    mapData_.addPoint(newPoint);
    updatePointTable();
    emit mapDataChanged(mapData_);
}

void MapEditor::onRemovePoint() {
    int row = pointTable_->currentRow();
    if (row >= 0) {
        QTableWidgetItem* idItem = pointTable_->item(row, 0);
        if (idItem) {
            int id = idItem->text().toInt();
            mapData_.removePoint(id);
            updatePointTable();
            updateEdgeTable();  // 更新边表（可能删除了相关边）
            emit mapDataChanged(mapData_);
        }
    }
}

void MapEditor::onPointSelectionChanged() {
    bool hasSelection = pointTable_->currentRow() >= 0;
    removePointBtn_->setEnabled(hasSelection);
}

void MapEditor::onPointDataChanged(int row, int column) {
    if (isUpdating_) return;
    
    QTableWidgetItem* idItem = pointTable_->item(row, 0);
    if (!idItem) return;
    
    int id = idItem->text().toInt();
    MapData::Point* point = mapData_.getPoint(id);
    if (!point) return;
    
    // 更新点位数据
    if (column == 1) {  // 名称
        point->name = pointTable_->item(row, column)->text();
    } else if (column == 2) {  // X坐标
        point->x = pointTable_->item(row, column)->text().toDouble();
    } else if (column == 3) {  // Y坐标
        point->y = pointTable_->item(row, column)->text().toDouble();
    } else if (column == 4) {  // 角度
        point->theta = pointTable_->item(row, column)->text().toDouble();
    } else if (column == 5) {  // 类型
        QComboBox* combo = qobject_cast<QComboBox*>(pointTable_->cellWidget(row, column));
        if (combo) {
            point->type = static_cast<MapData::PointType>(combo->currentIndex());
        }
    }
    
    mapData_.updatePoint(id, *point);
    emit mapDataChanged(mapData_);
}

void MapEditor::onAddEdge() {
    QVector<MapData::Point> points = mapData_.getAllPoints();
    if (points.size() < 2) {
        QMessageBox::warning(this, "错误", "至少需要2个点位才能添加边！");
        return;
    }
    
    // 使用第一个和第二个点位创建边
    int startId = points[0].id;
    int endId = points[1].id;
    
    MapData::Edge newEdge(startId, points[0].name, endId, points[1].name,
                         MapData::NAV_FORWARD, MapData::DRIVE_STRAIGHT);
    mapData_.addEdge(newEdge);
    updateEdgeTable();
    emit mapDataChanged(mapData_);
}

void MapEditor::onRemoveEdge() {
    int row = edgeTable_->currentRow();
    if (row >= 0) {
        // 从下拉框或表格项获取ID
        int startId = 0;
        int endId = 0;
        
        QComboBox* startIdCombo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, 0));
        if (startIdCombo) {
            startId = startIdCombo->currentData().toInt();
        } else {
            QTableWidgetItem* item = edgeTable_->item(row, 0);
            if (item) {
                startId = item->text().toInt();
            }
        }
        
        QComboBox* endIdCombo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, 2));
        if (endIdCombo) {
            endId = endIdCombo->currentData().toInt();
        } else {
            QTableWidgetItem* item = edgeTable_->item(row, 2);
            if (item) {
                endId = item->text().toInt();
            }
        }
        
        if (startId != 0 && endId != 0) {
            mapData_.removeEdge(startId, endId);
            updateEdgeTable();
            emit mapDataChanged(mapData_);
        }
    }
}

void MapEditor::onEdgeSelectionChanged() {
    bool hasSelection = edgeTable_->currentRow() >= 0;
    removeEdgeBtn_->setEnabled(hasSelection);
}

void MapEditor::onEdgeDataChanged(int row, int column) {
    if (isUpdating_) return;
    
    // 获取起点和终点ID（从下拉框或旧数据）
    int startId = 0;
    int endId = 0;
    
    // 从下拉框获取起点ID
    QComboBox* startIdCombo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, 0));
    if (startIdCombo) {
        startId = startIdCombo->currentData().toInt();
    } else {
        QTableWidgetItem* item = edgeTable_->item(row, 0);
        if (item) {
            startId = item->text().toInt();
        }
    }
    
    // 从下拉框获取终点ID
    QComboBox* endIdCombo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, 2));
    if (endIdCombo) {
        endId = endIdCombo->currentData().toInt();
    } else {
        QTableWidgetItem* item = edgeTable_->item(row, 2);
        if (item) {
            endId = item->text().toInt();
        }
    }
    
    if (startId == 0 || endId == 0) return;
    
    // 获取起点和终点名称
    const MapData::Point* startPoint = mapData_.getPoint(startId);
    const MapData::Point* endPoint = mapData_.getPoint(endId);
    
    if (!startPoint || !endPoint) {
        QMessageBox::warning(this, "错误", "起点或终点不存在！");
        updateEdgeTable();
        return;
    }
    
    // 获取旧的起点和终点ID（用于删除旧边）
    int oldStartId = 0;
    int oldEndId = 0;
    
    // 尝试从当前边数据获取旧ID
    QVector<MapData::Edge> edges = mapData_.getAllEdges();
    if (row < edges.size()) {
        oldStartId = edges[row].startId;
        oldEndId = edges[row].endId;
    }
    
    // 创建新的边对象，先使用默认值
    MapData::Edge edge(startId, startPoint->name, endId, endPoint->name,
                      MapData::NAV_FORWARD, MapData::DRIVE_STRAIGHT);
    
    // 如果存在旧边，保留其导航模式和行驶模式
    if (oldStartId != 0 && oldEndId != 0) {
        for (const MapData::Edge& oldEdge : edges) {
            if (oldEdge.startId == oldStartId && oldEdge.endId == oldEndId) {
                edge.navMode = oldEdge.navMode;
                edge.driveMode = oldEdge.driveMode;
                break;
            }
        }
    }
    
    // 更新导航模式和行驶模式（如果列是4或5）
    if (column == 4) {
        QComboBox* combo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, column));
        if (combo) {
            edge.navMode = static_cast<MapData::NavigationMode>(combo->currentIndex());
        }
    } else if (column == 5) {
        QComboBox* combo = qobject_cast<QComboBox*>(edgeTable_->cellWidget(row, column));
        if (combo) {
            edge.driveMode = static_cast<MapData::DriveMode>(combo->currentIndex());
        }
    }
    
    // 如果起点或终点ID改变了，需要删除旧边并添加新边
    if (oldStartId != 0 && oldEndId != 0 && (startId != oldStartId || endId != oldEndId)) {
        mapData_.removeEdge(oldStartId, oldEndId);
        mapData_.addEdge(edge);
    } else if (oldStartId != 0 && oldEndId != 0) {
        // ID没变，只更新边
        mapData_.updateEdge(startId, endId, edge);
    } else {
        // 新边，直接添加（这种情况不应该发生，因为边应该已经存在）
        mapData_.addEdge(edge);
    }
    
    // 更新表格显示（刷新名称等）
    updateEdgeTable();
    emit mapDataChanged(mapData_);
}

QString MapEditor::getFileFilter() const {
    int index = formatCombo_->currentIndex();
    if (index == 0) {
        return "JSON文件 (*.json);;所有文件 (*.*)";
    } else if (index == 1) {
        return "XML文件 (*.xml);;所有文件 (*.*)";
    } else {
        return "二进制文件 (*.map);;所有文件 (*.*)";
    }
}

QString MapEditor::getFileExtension() const {
    int index = formatCombo_->currentIndex();
    if (index == 0) {
        return "json";
    } else if (index == 1) {
        return "xml";
    } else {
        return "map";
    }
}

bool MapEditor::loadMapFile(const QString& filePath) {
    QString ext = QFileInfo(filePath).suffix().toLower();
    
    if (ext == "json") {
        return mapData_.loadFromJson(filePath);
    } else if (ext == "xml") {
        return mapData_.loadFromXml(filePath);
    } else if (ext == "map") {
        return mapData_.loadFromBinary(filePath);
    } else {
        // 尝试自动检测格式
        if (mapData_.loadFromJson(filePath)) return true;
        if (mapData_.loadFromXml(filePath)) return true;
        if (mapData_.loadFromBinary(filePath)) return true;
        return false;
    }
}

bool MapEditor::saveMapFile(const QString& filePath) {
    QString ext = QFileInfo(filePath).suffix().toLower();
    
    if (ext == "json") {
        return mapData_.saveToJson(filePath);
    } else if (ext == "xml") {
        return mapData_.saveToXml(filePath);
    } else if (ext == "map") {
        return mapData_.saveToBinary(filePath);
    } else {
        // 根据格式选择保存
        int index = formatCombo_->currentIndex();
        QString newPath = filePath;
        if (!newPath.endsWith("." + getFileExtension())) {
            newPath += "." + getFileExtension();
        }
        
        if (index == 0) {
            return mapData_.saveToJson(newPath);
        } else if (index == 1) {
            return mapData_.saveToXml(newPath);
        } else {
            return mapData_.saveToBinary(newPath);
        }
    }
}

