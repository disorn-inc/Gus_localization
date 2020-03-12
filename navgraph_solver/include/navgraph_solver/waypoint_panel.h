#ifndef WAYPOINT_PANEL_H
#define WAYPOINT_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

class QLineEdit;
class QPushButton;
class QVBoxLayout;

namespace navgraph_rviz_panel
{
class WaypointPanel: public rviz::Panel
{
Q_OBJECT
public:
  WaypointPanel( QWidget* parent = 0 );

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:

protected Q_SLOTS:
  void updatePoints();
  void updateAdjacency();
  void movebaseButton();
  void freezeButton();
  void loadButton();
  void addButton();
  void missionButton();
  void abortButton();
  void deletePoints();
  void deleteAdjacency();

protected:

  QLineEdit* point_file_editor_;
  QLineEdit* adjacency_file_editor_;
  QString point_file_;
  QString adjacency_file_;
  QPushButton *move_base_button;
  QPushButton *freeze_button;
  QPushButton *load_button;
  QPushButton *add_button;
  QPushButton *mission_button;
  QPushButton *abort_button;
  QPushButton *delete_points;
  QPushButton *delete_adjacency;
  QVBoxLayout* stations_layout;
  int station_count;
  ros::Publisher commands;
  ros::Publisher points_pub;
  ros::Publisher mission_pub;
  ros::NodeHandle nh_;

};

} // end namespace rviz_plugin_tutorials

#endif // WAYPOINT_PANEL_H