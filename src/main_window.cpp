/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include "../include/turtle_go/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign) {
  ui->setupUi(this);
  move(0, 0);
  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);
}

void MainWindow::closeEvent(QCloseEvent* event) { QMainWindow::closeEvent(event); }

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_On_clicked() {
  // 1. RealSense 카메라 실행
  system(
      "gnome-terminal --geometry=80x18+0+0 -- bash -c '"
      "ros2 launch realsense2_camera rs_launch.py "
      "enable_color:=true "
      "enable_depth:=true "
      "color_width:=640 "
      "color_height:=480 "
      "align_depth.enable:=true "
      "enable_rgbd:=true "
      "enable_sync:=true "
      "color_qos:=SENSOR_DATA "
      "depth_qos:=SENSOR_DATA; exec bash'");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // 2. YOLO 실행 (yolo12_bbox_coordinate)
  system(
      "gnome-terminal --geometry=80x18+800+0 -- bash -c '"
      "ros2 run yolo12_bbox_coordinate yolo_XYZ_pub_comp; exec bash'");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 3. FoundationPose 실행
  system(
      "gnome-terminal --geometry=80x18+0+300 -- bash -c '"
      "source ~/miniconda3/etc/profile.d/conda.sh && "
      "conda activate foundationpose_ros && "
      "export PYTHONNOUSERSITE=True && "
      "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 && "
      "cd ~/FoundationPoseROS2 && "
      "python foundationpose_ros_multi.py; exec bash'");
  std::this_thread::sleep_for(std::chrono::seconds(2));
}

void MainWindow::on_Off_clicked() {
  // 모든 관련 프로세스 종료
  system("pkill -f 'ros2 launch'");
  system("pkill -f 'ros2 run'");
  system("pkill -f 'realsense2_camera'");
  system("pkill -f 'yolo12_bbox_coordinate'");
  system("pkill -f 'foundationpose_ros_multi.py'");
  system("pkill -f 'python foundationpose_ros_multi.py'");
  system("pkill -f 'conda activate foundationpose_ros'");
}