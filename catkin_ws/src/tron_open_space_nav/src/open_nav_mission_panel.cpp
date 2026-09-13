#include <tron_open_space_nav/open_nav_mission_panel.h>

#include <QHeaderView>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <cmath>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(tron_open_space_nav::OpenNavMissionPanel, mapviz::MapvizPlugin)

namespace tron_open_space_nav
{

namespace
{
const double kSelectionAnchorEps = 0.05;
}

OpenNavMissionPanel::OpenNavMissionPanel()
  : config_widget_(new QWidget())
  , have_odom_(false)
  , have_localization_(false)
  , have_selection_anchor_(false)
  , selection_x_(0.0)
  , selection_y_(0.0)
{
  setupUi();
}

OpenNavMissionPanel::~OpenNavMissionPanel()
{
}

void OpenNavMissionPanel::setupUi()
{
  QVBoxLayout* layout = new QVBoxLayout(config_widget_);
  layout->setContentsMargins(4, 4, 4, 4);

  status_label_ = new QLabel("Initializing...");
  mission_state_label_ = new QLabel("Mission: IDLE");
  current_label_ = new QLabel("Current: -");
  distance_label_ = new QLabel("Distance: -");
  mode_label_ = new QLabel("Mode: once");
  loop_label_ = new QLabel("Loop: 0 / 0");
  robot_label_ = new QLabel("Robot: -");
  gps_label_ = new QLabel("GPS: -");

  layout->addWidget(status_label_);
  layout->addWidget(mission_state_label_);
  layout->addWidget(current_label_);
  layout->addWidget(distance_label_);
  layout->addWidget(mode_label_);
  layout->addWidget(loop_label_);
  layout->addWidget(robot_label_);
  layout->addWidget(gps_label_);

  patrol_mode_combo_ = new QComboBox();
  patrol_mode_combo_->addItems(QStringList() << "once" << "loop" << "pingpong");
  loop_count_spin_ = new QSpinBox();
  loop_count_spin_->setRange(0, 9999);
  loop_count_spin_->setToolTip("0 = infinite loops");
  layout->addWidget(new QLabel("Patrol mode:"));
  layout->addWidget(patrol_mode_combo_);
  layout->addWidget(new QLabel("Loop count (0=infinite):"));
  layout->addWidget(loop_count_spin_);

  table_ = new QTableWidget(0, 8);
  table_->setHorizontalHeaderLabels(
      QStringList() << "Index" << "Name" << "X" << "Y" << "Lat" << "Lon" << "Status" << "Dist(m)");
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  layout->addWidget(table_);

  delete_btn_ = new QPushButton("Delete Selected");
  move_up_btn_ = new QPushButton("Move Up");
  move_down_btn_ = new QPushButton("Move Down");
  clear_btn_ = new QPushButton("Clear All");
  send_btn_ = new QPushButton("Send Mission");
  start_btn_ = new QPushButton("Start Patrol");
  pause_btn_ = new QPushButton("Pause");
  resume_btn_ = new QPushButton("Resume");
  stop_btn_ = new QPushButton("Stop");
  save_btn_ = new QPushButton("Save");
  load_btn_ = new QPushButton("Load");

  layout->addWidget(delete_btn_);
  layout->addWidget(move_up_btn_);
  layout->addWidget(move_down_btn_);
  layout->addWidget(clear_btn_);
  layout->addWidget(send_btn_);
  layout->addWidget(start_btn_);
  layout->addWidget(pause_btn_);
  layout->addWidget(resume_btn_);
  layout->addWidget(stop_btn_);
  layout->addWidget(save_btn_);
  layout->addWidget(load_btn_);

  QPalette p(config_widget_->palette());
  p.setColor(QPalette::Background, Qt::white);
  config_widget_->setPalette(p);

  connect(this, SIGNAL(waypointInfoUpdated()), this, SLOT(onWaypointsUpdated()),
          Qt::QueuedConnection);
  connect(this, SIGNAL(missionStatusUpdated()), this, SLOT(onMissionStatusUpdated()),
          Qt::QueuedConnection);
  connect(this, SIGNAL(odomUpdated()), this, SLOT(onOdomUpdated()), Qt::QueuedConnection);
  connect(this, SIGNAL(localizationUpdated()), this, SLOT(onLocalizationUpdated()),
          Qt::QueuedConnection);

  connect(table_, SIGNAL(itemSelectionChanged()), this, SLOT(onTableSelectionChanged()));
  connect(delete_btn_, SIGNAL(clicked()), this, SLOT(onDeleteSelected()));
  connect(move_up_btn_, SIGNAL(clicked()), this, SLOT(onMoveUp()));
  connect(move_down_btn_, SIGNAL(clicked()), this, SLOT(onMoveDown()));
  connect(clear_btn_, SIGNAL(clicked()), this, SLOT(onClearAll()));
  connect(send_btn_, SIGNAL(clicked()), this, SLOT(onSendMission()));
  connect(start_btn_, SIGNAL(clicked()), this, SLOT(onStartPatrol()));
  connect(pause_btn_, SIGNAL(clicked()), this, SLOT(onPause()));
  connect(resume_btn_, SIGNAL(clicked()), this, SLOT(onResume()));
  connect(stop_btn_, SIGNAL(clicked()), this, SLOT(onStop()));
  connect(save_btn_, SIGNAL(clicked()), this, SLOT(onSave()));
  connect(load_btn_, SIGNAL(clicked()), this, SLOT(onLoad()));
}

bool OpenNavMissionPanel::Initialize(QGLWidget* canvas)
{
  canvas_ = canvas;

  waypoint_info_sub_ = node_.subscribe(
      "/open_nav/waypoint_info", 1, &OpenNavMissionPanel::waypointInfoCallback, this);
  mission_status_sub_ = node_.subscribe(
      "/open_nav/mission_status", 1, &OpenNavMissionPanel::missionStatusCallback, this);
  odom_sub_ = node_.subscribe(
      "/open_nav/odom", 1, &OpenNavMissionPanel::odomCallback, this);
  localization_sub_ = node_.subscribe(
      "/open_nav/localization_status", 1, &OpenNavMissionPanel::localizationCallback, this);
  selected_pub_ = node_.advertise<std_msgs::Int32>("/open_nav/selected_waypoint", 1, true);

  send_client_ = node_.serviceClient<std_srvs::Trigger>("/open_nav/mission/send");
  start_client_ = node_.serviceClient<tron_open_space_nav::MissionPatrolStart>(
      "/open_nav/mission/start_patrol");
  pause_client_ = node_.serviceClient<std_srvs::Empty>("/open_nav/mission/pause");
  resume_client_ = node_.serviceClient<std_srvs::Empty>("/open_nav/mission/resume");
  stop_client_ = node_.serviceClient<std_srvs::Empty>("/open_nav/mission/stop");
  clear_client_ = node_.serviceClient<std_srvs::Empty>("/open_nav/mission/clear");
  save_client_ = node_.serviceClient<std_srvs::Trigger>("/open_nav/mission/save");
  load_client_ = node_.serviceClient<std_srvs::Trigger>("/open_nav/mission/load");
  delete_client_ = node_.serviceClient<tron_open_space_nav::DeleteWaypoint>(
      "/open_nav/mission/delete_waypoint");
  move_client_ = node_.serviceClient<tron_open_space_nav::MoveWaypoint>(
      "/open_nav/mission/move_waypoint");

  initialized_ = true;
  PrintInfo("OpenNav Mission Panel ready");
  updateButtonStates();
  return true;
}

void OpenNavMissionPanel::Shutdown()
{
  waypoint_info_sub_.shutdown();
  mission_status_sub_.shutdown();
  odom_sub_.shutdown();
  localization_sub_.shutdown();
}

void OpenNavMissionPanel::Draw(double /*x*/, double /*y*/, double /*scale*/)
{
}

void OpenNavMissionPanel::Transform()
{
}

void OpenNavMissionPanel::LoadConfig(const YAML::Node& /*node*/, const std::string& /*path*/)
{
}

void OpenNavMissionPanel::SaveConfig(YAML::Emitter& /*emitter*/, const std::string& /*path*/)
{
}

QWidget* OpenNavMissionPanel::GetConfigWidget(QWidget* parent)
{
  config_widget_->setParent(parent);
  return config_widget_;
}

void OpenNavMissionPanel::PrintError(const std::string& message)
{
  PrintErrorHelper(status_label_, message);
}

void OpenNavMissionPanel::PrintInfo(const std::string& message)
{
  PrintInfoHelper(status_label_, message);
}

void OpenNavMissionPanel::PrintWarning(const std::string& message)
{
  PrintWarningHelper(status_label_, message);
}

void OpenNavMissionPanel::waypointInfoCallback(
    const tron_open_space_nav::WaypointInfoArrayConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(waypoint_info_mutex_);
    latest_waypoint_info_ = *msg;
  }
  Q_EMIT waypointInfoUpdated();
}

void OpenNavMissionPanel::missionStatusCallback(
    const tron_open_space_nav::MissionStatusConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mission_status_mutex_);
    mission_status_ = *msg;
  }
  Q_EMIT missionStatusUpdated();
}

void OpenNavMissionPanel::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_ = *msg;
    have_odom_ = true;
  }
  Q_EMIT odomUpdated();
}

void OpenNavMissionPanel::localizationCallback(
    const tron_open_space_nav::LocalizationStatusConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  localization_ = *msg;
  have_localization_ = true;
  Q_EMIT localizationUpdated();
}

bool OpenNavMissionPanel::waypointStructureChanged(
    const tron_open_space_nav::WaypointInfoArray& a,
    const tron_open_space_nav::WaypointInfoArray& b) const
{
  if (a.waypoints.size() != b.waypoints.size())
  {
    return true;
  }
  for (size_t i = 0; i < a.waypoints.size(); ++i)
  {
    const auto& wa = a.waypoints[i];
    const auto& wb = b.waypoints[i];
    if (wa.index != wb.index)
    {
      return true;
    }
    if (wa.name != wb.name)
    {
      return true;
    }
    if (std::abs(wa.x - wb.x) > 1e-4)
    {
      return true;
    }
    if (std::abs(wa.y - wb.y) > 1e-4)
    {
      return true;
    }
    if (std::abs(wa.latitude - wb.latitude) > 1e-7)
    {
      return true;
    }
    if (std::abs(wa.longitude - wb.longitude) > 1e-7)
    {
      return true;
    }
  }
  return false;
}

void OpenNavMissionPanel::syncRowsFromWaypointInfo(
    const tron_open_space_nav::WaypointInfoArray& msg)
{
  rows_.clear();
  rows_.reserve(msg.waypoints.size());
  for (const auto& wp : msg.waypoints)
  {
    WaypointRow row;
    row.index = wp.index;
    row.name = wp.name;
    row.x = wp.x;
    row.y = wp.y;
    row.latitude = wp.latitude;
    row.longitude = wp.longitude;
    row.status = wp.status.empty() ? "PENDING" : wp.status;
    row.distance = 0.0;
    rows_.push_back(row);
  }
}

void OpenNavMissionPanel::applyMissionStatusToRows()
{
  tron_open_space_nav::MissionStatus status;
  {
    std::lock_guard<std::mutex> lock(mission_status_mutex_);
    status = mission_status_;
  }
  if (rows_.empty())
  {
    return;
  }
  for (size_t i = 0; i < rows_.size(); ++i)
  {
    std::string st = "PENDING";
    if (status.state == "COMPLETED")
    {
      st = "DONE";
    }
    else if (status.state == "RUNNING" || status.state == "PAUSED")
    {
      if (i < status.current_index)
      {
        st = "DONE";
      }
      else if (i == status.current_index)
      {
        st = "ACTIVE";
      }
    }
    rows_[i].status = st;
  }
}

void OpenNavMissionPanel::captureSelectionAnchor()
{
  const int row = table_->currentRow();
  if (row < 0 || row >= static_cast<int>(rows_.size()))
  {
    return;
  }
  selection_x_ = rows_[static_cast<size_t>(row)].x;
  selection_y_ = rows_[static_cast<size_t>(row)].y;
  have_selection_anchor_ = true;
}

void OpenNavMissionPanel::clearSelectionAnchor()
{
  have_selection_anchor_ = false;
  selection_x_ = 0.0;
  selection_y_ = 0.0;
}

int OpenNavMissionPanel::rowForAnchor(double x, double y) const
{
  for (size_t i = 0; i < rows_.size(); ++i)
  {
    if (std::hypot(rows_[i].x - x, rows_[i].y - y) <= kSelectionAnchorEps)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void OpenNavMissionPanel::restoreSelectionFromAnchor()
{
  if (!have_selection_anchor_)
  {
    return;
  }
  const int row = rowForAnchor(selection_x_, selection_y_);
  if (row < 0)
  {
    clearSelectionAnchor();
    return;
  }

  QSignalBlocker blocker(table_);
  table_->selectRow(row);
  table_->setCurrentCell(row, COL_INDEX);
}

void OpenNavMissionPanel::rebuildWaypointTable(const char* source)
{
  captureSelectionAnchor();

  nav_msgs::Odometry odom;
  bool have_odom = false;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom = odom_;
    have_odom = have_odom_;
  }

  QSignalBlocker blocker(table_);
  table_->setUpdatesEnabled(false);
  table_->clearContents();
  table_->setRowCount(static_cast<int>(rows_.size()));

  for (size_t i = 0; i < rows_.size(); ++i)
  {
    if (have_odom)
    {
      const double dx = rows_[i].x - odom.pose.pose.position.x;
      const double dy = rows_[i].y - odom.pose.pose.position.y;
      rows_[i].distance = std::hypot(dx, dy);
    }

    const WaypointRow& r = rows_[i];
    QTableWidgetItem* index_item = new QTableWidgetItem(QString::number(r.index + 1));
    index_item->setData(Qt::UserRole, static_cast<int>(r.index));
    table_->setItem(static_cast<int>(i), COL_INDEX, index_item);
    table_->setItem(static_cast<int>(i), COL_NAME, new QTableWidgetItem(QString::fromStdString(r.name)));
    table_->setItem(static_cast<int>(i), COL_X, new QTableWidgetItem(QString::number(r.x, 'f', 2)));
    table_->setItem(static_cast<int>(i), COL_Y, new QTableWidgetItem(QString::number(r.y, 'f', 2)));
    table_->setItem(
        static_cast<int>(i), COL_LAT, new QTableWidgetItem(QString::number(r.latitude, 'f', 6)));
    table_->setItem(
        static_cast<int>(i), COL_LON, new QTableWidgetItem(QString::number(r.longitude, 'f', 6)));
    table_->setItem(
        static_cast<int>(i), COL_STATUS, new QTableWidgetItem(QString::fromStdString(r.status)));
    table_->setItem(
        static_cast<int>(i), COL_DIST, new QTableWidgetItem(QString::number(r.distance, 'f', 1)));
  }

  table_->setUpdatesEnabled(true);
  table_->viewport()->update();

  restoreSelectionFromAnchor();

  const int selected_row = table_->currentRow();
  QString selected_name = "-";
  if (selected_row >= 0 && table_->item(selected_row, COL_NAME))
  {
    selected_name = table_->item(selected_row, COL_NAME)->text();
  }
  ROS_INFO(
      "[OpenNavMissionPanel] FULL REBUILD source=%s N=%zu selected=%s",
      source,
      rows_.size(),
      selected_name.toStdString().c_str());
}

void OpenNavMissionPanel::updateDynamicWaypointFields(const char* source)
{
  if (rows_.empty())
  {
    return;
  }
  if (table_->rowCount() != static_cast<int>(rows_.size()))
  {
    rebuildWaypointTable("row_mismatch");
    return;
  }

  nav_msgs::Odometry odom;
  bool have_odom = false;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom = odom_;
    have_odom = have_odom_;
  }

  QSignalBlocker blocker(table_);
  for (size_t i = 0; i < rows_.size(); ++i)
  {
    if (have_odom)
    {
      const double dx = rows_[i].x - odom.pose.pose.position.x;
      const double dy = rows_[i].y - odom.pose.pose.position.y;
      rows_[i].distance = std::hypot(dx, dy);
    }

    QTableWidgetItem* status_item = table_->item(static_cast<int>(i), COL_STATUS);
    if (status_item)
    {
      status_item->setText(QString::fromStdString(rows_[i].status));
    }
    QTableWidgetItem* dist_item = table_->item(static_cast<int>(i), COL_DIST);
    if (dist_item)
    {
      dist_item->setText(QString::number(rows_[i].distance, 'f', 1));
    }
  }

  ROS_DEBUG("[OpenNavMissionPanel] dynamic update source=%s rows=%zu", source, rows_.size());
}

void OpenNavMissionPanel::onWaypointsUpdated()
{
  tron_open_space_nav::WaypointInfoArray msg;
  {
    std::lock_guard<std::mutex> lock(waypoint_info_mutex_);
    msg = latest_waypoint_info_;
  }

  const bool structure_changed = waypointStructureChanged(cached_structure_, msg)
      || table_->rowCount() != static_cast<int>(msg.waypoints.size());

  syncRowsFromWaypointInfo(msg);
  applyMissionStatusToRows();

  if (structure_changed)
  {
    rebuildWaypointTable("waypoint_info");
    cached_structure_ = msg;
  }
  else
  {
    updateDynamicWaypointFields("waypoint_info");
  }
  updateButtonStates();
}

void OpenNavMissionPanel::onMissionStatusUpdated()
{
  tron_open_space_nav::MissionStatus msg;
  {
    std::lock_guard<std::mutex> lock(mission_status_mutex_);
    msg = mission_status_;
  }

  mission_state_label_->setText(QString("Mission: %1").arg(QString::fromStdString(msg.state)));
  const uint32_t cur = msg.current_index + 1;
  current_label_->setText(QString("Current: P%1 / %2").arg(cur).arg(msg.total));
  distance_label_->setText(QString("Distance: %1 m").arg(msg.distance_to_goal, 0, 'f', 2));
  mode_label_->setText(QString("Mode: %1").arg(QString::fromStdString(msg.patrol_mode)));
  loop_label_->setText(QString("Loop: %1 / %2").arg(msg.current_loop).arg(msg.loop_count));

  applyMissionStatusToRows();
  if (!rows_.empty())
  {
    updateDynamicWaypointFields("mission_status");
  }
  updateButtonStates();
}

void OpenNavMissionPanel::onOdomUpdated()
{
  nav_msgs::Odometry msg;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    msg = odom_;
  }
  robot_label_->setText(
      QString("Robot: x=%1 y=%2")
          .arg(msg.pose.pose.position.x, 0, 'f', 2)
          .arg(msg.pose.pose.position.y, 0, 'f', 2));
  if (!rows_.empty())
  {
    updateDynamicWaypointFields("odom");
  }
}

void OpenNavMissionPanel::onLocalizationUpdated()
{
  gps_label_->setText(
      QString("GPS: %1 sigma=%2 m")
          .arg(localization_.gps_fix_valid ? "VALID" : "INVALID")
          .arg(localization_.gps_sigma, 0, 'f', 1));
}

bool OpenNavMissionPanel::missionEditable() const
{
  tron_open_space_nav::MissionStatus status;
  {
    std::lock_guard<std::mutex> lock(mission_status_mutex_);
    status = mission_status_;
  }
  return status.state != "RUNNING" && status.state != "PAUSED";
}

void OpenNavMissionPanel::updateButtonStates()
{
  const bool editable = missionEditable();
  tron_open_space_nav::MissionStatus status;
  {
    std::lock_guard<std::mutex> lock(mission_status_mutex_);
    status = mission_status_;
  }
  const std::string st = status.state;
  const bool has_waypoints = !rows_.empty();
  const int row = selectedRow();
  const int last_row = static_cast<int>(rows_.size()) - 1;

  delete_btn_->setEnabled(editable && row >= 0);
  move_up_btn_->setEnabled(editable && row > 0);
  move_down_btn_->setEnabled(editable && row >= 0 && row < last_row);
  clear_btn_->setEnabled(editable);
  load_btn_->setEnabled(editable);
  send_btn_->setEnabled(editable && has_waypoints);

  start_btn_->setEnabled(st == "READY" || st == "STOPPED");
  pause_btn_->setEnabled(st == "RUNNING");
  resume_btn_->setEnabled(st == "PAUSED");
  stop_btn_->setEnabled(st == "RUNNING" || st == "PAUSED");
  save_btn_->setEnabled(has_waypoints);
}

int OpenNavMissionPanel::selectedRow() const
{
  const int row = table_->currentRow();
  if (row >= 0 && row < table_->rowCount())
  {
    return row;
  }
  return -1;
}

void OpenNavMissionPanel::onTableSelectionChanged()
{
  const int row = selectedRow();
  if (row >= 0 && row < static_cast<int>(rows_.size()))
  {
    selection_x_ = rows_[static_cast<size_t>(row)].x;
    selection_y_ = rows_[static_cast<size_t>(row)].y;
    have_selection_anchor_ = true;
    ROS_INFO(
        "[OpenNavMissionPanel] selected row=%d waypoint=%s",
        row,
        rows_[static_cast<size_t>(row)].name.c_str());
  }
  publishSelectedWaypoint(row);
  updateButtonStates();
}

void OpenNavMissionPanel::publishSelectedWaypoint(int row)
{
  std_msgs::Int32 msg;
  msg.data = row;
  selected_pub_.publish(msg);
}

void OpenNavMissionPanel::onDeleteSelected()
{
  const int row = selectedRow();
  if (row < 0)
  {
    return;
  }
  captureSelectionAnchor();
  const QString name = table_->item(row, COL_NAME)->text();
  if (QMessageBox::question(config_widget_, "Delete waypoint",
                            QString("Delete waypoint %1?").arg(name)) != QMessageBox::Yes)
  {
    return;
  }
  tron_open_space_nav::DeleteWaypoint srv;
  srv.request.index = static_cast<uint32_t>(row);
  if (delete_client_.call(srv) && srv.response.success)
  {
    PrintInfo("Deleted " + name.toStdString());
    clearSelectionAnchor();
  }
  else
  {
    PrintError(srv.response.message.empty() ? "delete failed" : srv.response.message);
  }
}

void OpenNavMissionPanel::onMoveUp()
{
  const int row = selectedRow();
  if (row <= 0)
  {
    return;
  }
  captureSelectionAnchor();
  const QString name = table_->item(row, COL_NAME)->text();
  ROS_INFO("[OpenNavMissionPanel] Move Up row=%d waypoint=%s", row, name.toStdString().c_str());

  tron_open_space_nav::MoveWaypoint srv;
  srv.request.from_index = static_cast<uint32_t>(row);
  srv.request.to_index = static_cast<uint32_t>(row - 1);
  if (!move_client_.call(srv) || !srv.response.success)
  {
    PrintError(srv.response.message.empty() ? "move failed" : srv.response.message);
  }
}

void OpenNavMissionPanel::onMoveDown()
{
  const int row = selectedRow();
  if (row < 0 || row >= static_cast<int>(rows_.size()) - 1)
  {
    return;
  }
  captureSelectionAnchor();
  const QString name = table_->item(row, COL_NAME)->text();
  ROS_INFO("[OpenNavMissionPanel] Move Down row=%d waypoint=%s", row, name.toStdString().c_str());

  tron_open_space_nav::MoveWaypoint srv;
  srv.request.from_index = static_cast<uint32_t>(row);
  srv.request.to_index = static_cast<uint32_t>(row + 1);
  if (!move_client_.call(srv) || !srv.response.success)
  {
    PrintError(srv.response.message.empty() ? "move failed" : srv.response.message);
  }
}

void OpenNavMissionPanel::onClearAll()
{
  if (QMessageBox::question(config_widget_, "Clear all waypoints",
                            "Clear all waypoints?") != QMessageBox::Yes)
  {
    return;
  }
  std_srvs::Empty srv;
  if (clear_client_.call(srv))
  {
    PrintInfo("Cleared all waypoints");
    clearSelectionAnchor();
    publishSelectedWaypoint(-1);
  }
  else
  {
    PrintError("clear service failed");
  }
}

void OpenNavMissionPanel::onSendMission()
{
  std_srvs::Trigger srv;
  if (send_client_.call(srv) && srv.response.success)
  {
    PrintInfo("Mission sent: " + srv.response.message);
  }
  else
  {
    PrintError(srv.response.message.empty() ? "send failed" : srv.response.message);
  }
}

void OpenNavMissionPanel::onStartPatrol()
{
  tron_open_space_nav::MissionPatrolStart srv;
  srv.request.patrol_mode = patrol_mode_combo_->currentText().toStdString();
  srv.request.loop_count = static_cast<uint32_t>(loop_count_spin_->value());
  if (start_client_.call(srv) && srv.response.success)
  {
    PrintInfo("Patrol started");
  }
  else
  {
    PrintError(srv.response.message.empty() ? "start failed" : srv.response.message);
  }
}

void OpenNavMissionPanel::onPause()
{
  std_srvs::Empty srv;
  if (pause_client_.call(srv))
    PrintInfo("Paused");
  else
    PrintError("pause failed");
}

void OpenNavMissionPanel::onResume()
{
  std_srvs::Empty srv;
  if (resume_client_.call(srv))
    PrintInfo("Resumed");
  else
    PrintError("resume failed");
}

void OpenNavMissionPanel::onStop()
{
  std_srvs::Empty srv;
  if (stop_client_.call(srv))
    PrintInfo("Stopped");
  else
    PrintError("stop failed");
}

void OpenNavMissionPanel::onSave()
{
  std_srvs::Trigger srv;
  if (save_client_.call(srv) && srv.response.success)
    PrintInfo(srv.response.message);
  else
    PrintError(srv.response.message.empty() ? "save failed" : srv.response.message);
}

void OpenNavMissionPanel::onLoad()
{
  if (QMessageBox::question(config_widget_, "Load waypoints",
                            "Load waypoints from file?") != QMessageBox::Yes)
  {
    return;
  }
  std_srvs::Trigger srv;
  if (load_client_.call(srv) && srv.response.success)
    PrintInfo(srv.response.message);
  else
    PrintError(srv.response.message.empty() ? "load failed" : srv.response.message);
}

}  // namespace tron_open_space_nav
