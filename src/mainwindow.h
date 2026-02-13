#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QTimer>
#include <QPainter>
#include <QMouseEvent>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QPolygonF>
#include <QResizeEvent>
#include <QPaintEvent>
#include "mapwidget.h"
#include "vehicle.h"
#include "map.h"
#include "controller.h"

/**
 * @brief 主窗口类 - AGV调度界面
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void updateSimulation();
    void onClearTrajectory();
    void onResetVehicle();
    void onSetMapSize();
    void onAlgorithmChanged(int index);

private:
    // UI组件
    QWidget* centralWidget_;
    MapWidget* mapWidget_;
    QVBoxLayout* mainLayout_;
    QHBoxLayout* controlLayout_;
    QGroupBox* controlPanel_;
    QPushButton* clearTrajectoryBtn_;
    QPushButton* resetVehicleBtn_;
    QDoubleSpinBox* mapWidthSpin_;
    QDoubleSpinBox* mapHeightSpin_;
    QPushButton* setMapSizeBtn_;
    QComboBox* algorithmCombo_;
    QDoubleSpinBox* goalThetaSpin_;
    QLabel* statusLabel_;
    
    // 仿真对象
    Vehicle vehicle_;
    Map map_;
    Controller controller_;
    
    // 定时器
    QTimer* timer_;
    
    
    // 更新状态显示
    void updateStatus();
};

#endif // MAINWINDOW_H

