#ifndef TRON_OPEN_SPACE_NAV_OPEN_NAV_MISSION_PANEL_H_
#define TRON_OPEN_SPACE_NAV_OPEN_NAV_MISSION_PANEL_H_

#include <mapviz/mapviz_plugin.h>

#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QWidget>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <tron_open_space_nav/LocalizationStatus.h>
#include <tron_open_space_nav/MissionStatus.h>
#include <tron_open_space_nav/WaypointInfoArray.h>
#include <tron_open_space_nav/DeleteWaypoint.h>
#include <tron_open_space_nav/MissionPatrolStart.h>
#include <tron_open_space_nav/MoveWaypoint.h>

#include <mutex>
#include <vector>

namespace tron_open_space_nav
{

struct WaypointRow
{
  uint32_t index;
  std::string name;
  double x;
  double y;
  double latitude;
  double longitude;
  std::string status;
  double distance;
};

class OpenNavMissionPanel : public mapviz::MapvizPlugin
{
  Q_OBJECT

public:
  OpenNavMissionPanel();
  ~OpenNavMissionPanel() override;

  bool Initialize(QGLWidget* canvas) override;
  void Shutdown() override;
  void Draw(double x, double y, double scale) override;
  void Transform() override;
  void LoadConfig(const YAML::Node& node, const std::string& path) override;
  void SaveConfig(YAML::Emitter& emitter, const std::string& path) override;
  QWidget* GetConfigWidget(QWidget* parent) override;
  void PrintError(const std::string& message) override;
  void PrintInfo(const std::string& message) override;
  void PrintWarning(const std::string& message) override;

Q_SIGNALS:
  void waypointInfoUpdated();
  void missionStatusUpdated();
  void odomUpdated();
  void localizationUpdated();

private Q_SLOTS:
  void onWaypointsUpdated();
  void onMissionStatusUpdated();
  void onOdomUpdated();
  void onLocalizationUpdated();
  void onTableSelectionChanged();
  void onDeleteSelected();
  void onMoveUp();
  void onMoveDown();
  void onClearAll();
  void onSendMission();
  void onStartPatrol();
  void onPause();
  void onResume();
  void onStop();
  void onSave();
  void onLoad();

private:
  static const int COL_INDEX = 0;
  static const int COL_NAME = 1;
  static const int COL_X = 2;
  static const int COL_Y = 3;
  static const int COL_LAT = 4;
  static const int COL_LON = 5;
  static const int COL_STATUS = 6;
  static const int COL_DIST = 7;

  void setupUi();
  void updateButtonStates();
  bool missionEditable() const;
  int selectedRow() const;
  void publishSelectedWaypoint(int row);
  void syncRowsFromWaypointInfo(const tron_open_space_nav::WaypointInfoArray& msg);
  void applyMissionStatusToRows();
  bool waypointStructureChanged(
      const tron_open_space_nav::WaypointInfoArray& a,
      const tron_open_space_nav::WaypointInfoArray& b) const;
  void captureSelectionAnchor();
  void restoreSelectionFromAnchor();
  void clearSelectionAnchor();
  int rowForAnchor(double x, double y) const;
  void rebuildWaypointTable(const char* source);
  void updateDynamicWaypointFields(const char* source);

  void waypointInfoCallback(const tron_open_space_nav::WaypointInfoArrayConstPtr& msg);
  void missionStatusCallback(const tron_open_space_nav::MissionStatusConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void localizationCallback(const tron_open_space_nav::LocalizationStatusConstPtr& msg);

  QWidget* config_widget_;
  QLabel* status_label_;
  QLabel* mission_state_label_;
  QLabel* current_label_;
  QLabel* distance_label_;
  QLabel* mode_label_;
  QLabel* loop_label_;
  QLabel* robot_label_;
  QLabel* gps_label_;
  QComboBox* patrol_mode_combo_;
  QSpinBox* loop_count_spin_;
  QTableWidget* table_;
  QPushButton* delete_btn_;
  QPushButton* move_up_btn_;
  QPushButton* move_down_btn_;
  QPushButton* clear_btn_;
  QPushButton* send_btn_;
  QPushButton* start_btn_;
  QPushButton* pause_btn_;
  QPushButton* resume_btn_;
  QPushButton* stop_btn_;
  QPushButton* save_btn_;
  QPushButton* load_btn_;

  ros::Subscriber waypoint_info_sub_;
  ros::Subscriber mission_status_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber localization_sub_;
  ros::Publisher selected_pub_;

  ros::ServiceClient send_client_;
  ros::ServiceClient start_client_;
  ros::ServiceClient pause_client_;
  ros::ServiceClient resume_client_;
  ros::ServiceClient stop_client_;
  ros::ServiceClient clear_client_;
  ros::ServiceClient save_client_;
  ros::ServiceClient load_client_;
  ros::ServiceClient delete_client_;
  ros::ServiceClient move_client_;

  std::vector<WaypointRow> rows_;
  tron_open_space_nav::WaypointInfoArray latest_waypoint_info_;
  tron_open_space_nav::WaypointInfoArray cached_structure_;
  tron_open_space_nav::MissionStatus mission_status_;
  nav_msgs::Odometry odom_;
  tron_open_space_nav::LocalizationStatus localization_;
  std::mutex waypoint_info_mutex_;
  mutable std::mutex mission_status_mutex_;
  std::mutex odom_mutex_;
  bool have_odom_;
  bool have_localization_;
  bool have_selection_anchor_;
  double selection_x_;
  double selection_y_;
};

}  // namespace tron_open_space_nav

#endif  // TRON_OPEN_SPACE_NAV_OPEN_NAV_MISSION_PANEL_H_
