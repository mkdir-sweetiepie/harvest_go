/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the integrated Vision & Manipulator Qt GUI.
 *
 * @date January 2025
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/harvest_go/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  ui->setupUi(this);
  move(3000, 0);
  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);
  this->setWindowTitle("🤖 Vision & Manipulator Control System");
}

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

MainWindow::~MainWindow() { delete ui; }

// ========== 비전 시스템 ==========
void MainWindow::on_Vision_All_On_clicked() {
  on_RealSense_clicked();
  std::this_thread::sleep_for(std::chrono::seconds(2));
  on_Calibration_clicked();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  on_YOLO_clicked();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  on_FoundationPose_clicked();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  on_TSP_clicked();
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void MainWindow::on_Vision_All_Off_clicked() {
  system("pkill -f 'realsense2_camera'");
  system("pkill -f 'calibration_node'");
  system("pkill -f 'yolo12_bbox_coordinate'");
  system("pkill -f 'foundationpose_ros'");
  system("pkill -f 'tsp'");
}

void MainWindow::on_RealSense_clicked() {
  system(
      "gnome-terminal --geometry=65x12+0+50 -- bash -c '"
      "ros2 launch realsense2_camera rs_launch.py \
      enable_color:=true \
      enable_depth:=true \
      color_width:=640 \
      color_height:=480 \
      align_depth.enable:=true \
      enable_rgbd:=true \
      enable_sync:=true \
      color_qos:=SENSOR_DATA \
      depth_qos:=SENSOR_DATA; exec bash'");
}

void MainWindow::on_Calibration_clicked() {
  system(
      "gnome-terminal --geometry=65x12+0+250 -- bash -c '"
      "ros2 run calibration_node crop_transformer_node; exec bash'");
}

void MainWindow::on_YOLO_clicked() {
  system(
      "gnome-terminal --geometry=65x12+0+450 -- bash -c '"
      "ros2 run yolo12_bbox_coordinate yolo_XYZ_pub_comp; exec bash'");
}

void MainWindow::on_FoundationPose_clicked() {
  system(
      "gnome-terminal --geometry=65x12+0+650 -- bash -c '"
      "source ~/miniconda3/etc/profile.d/conda.sh && \
      conda activate foundationpose_ros && \
      export PYTHONNOUSERSITE=True && \
      export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 && \
      cd ~/FoundationPoseROS2 && \
      python foundationpose_ros_multi.py; exec bash'");
}

void MainWindow::on_TSP_clicked() {
  system(
      "gnome-terminal --geometry=65x12+0+850 -- bash -c '"
      "ros2 run tsp tsp; exec bash'");
}

// ========== 매니퓰레이터 시스템 ==========
void MainWindow::on_Mani_All_On_clicked() {
  // Motor 노드 먼저 실행 (항상 첫번째)
  on_Motor_clicked();
  std::this_thread::sleep_for(std::chrono::seconds(2));
  on_Map_clicked();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  on_Path_clicked();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  on_Inverse_Sim_clicked();
  std::this_thread::sleep_for(std::chrono::seconds(1));
}

void MainWindow::on_Mani_All_Off_clicked() { system("pkill -f 'sad_pkg'"); }

void MainWindow::on_Motor_clicked() {
  system(
      "gnome-terminal --geometry=65x12+800+50 -- bash -c '"
      "ros2 run sad_pkg motor_node; exec bash'");
}

void MainWindow::on_Map_clicked() {
  system(
      "gnome-terminal --geometry=65x12+800+250 -- bash -c '"
      "ros2 run sad_pkg map_node; exec bash'");
}

void MainWindow::on_Path_clicked() {
  system(
      "gnome-terminal --geometry=65x12+800+450 -- bash -c '"
      "ros2 run sad_pkg path_node; exec bash'");
}

void MainWindow::on_Inverse_Sim_clicked() {
  system(
      "gnome-terminal --geometry=65x12+800+650 -- bash -c '"
      "ros2 run sad_pkg Inverse_node; exec bash'");
}

void MainWindow::on_Real_Inverse_clicked() {
  system(
      "gnome-terminal --geometry=65x12+800+850 -- bash -c '"
      "ros2 run sad_pkg real_inverse_node; exec bash'");
}

// ========== 전체 시스템 ==========
void MainWindow::on_System_All_On_clicked() {
  on_Vision_All_On_clicked();
  on_Mani_All_On_clicked();
}

void MainWindow::on_System_All_Off_clicked() {
  on_Vision_All_Off_clicked();
  on_Mani_All_Off_clicked();
}