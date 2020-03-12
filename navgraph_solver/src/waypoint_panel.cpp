#include <stdio.h>
#include <iostream>

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QCheckBox>
#include <QSpinBox>
#include <QComboBox>
#include <QMenu>
#include <QLabel>
#include <QListWidget>

#include "navgraph_solver/waypoint_panel.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <string>

namespace navgraph_rviz_panel
{

WaypointPanel::WaypointPanel( QWidget* parent )
  : rviz::Panel( parent ), station_count(1)
{
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Points File:" ));
  point_file_editor_ = new QLineEdit;
  topic_layout->addWidget( point_file_editor_ );

  QHBoxLayout* topic_layout2 = new QHBoxLayout;
  topic_layout2->addWidget( new QLabel( "Adjacency File:" ));
  adjacency_file_editor_ = new QLineEdit;
  topic_layout2->addWidget( adjacency_file_editor_ );

  QHBoxLayout* points_layout = new QHBoxLayout;
  freeze_button = new QPushButton("Freeze OnScreen Points");
  points_layout->addWidget(freeze_button);
  load_button = new QPushButton("Load Files");
  points_layout->addWidget(load_button);

  QHBoxLayout* delete_layout = new QHBoxLayout;
  delete_points = new QPushButton("Delete Points");
  delete_layout->addWidget(delete_points);
  delete_adjacency = new QPushButton("Delete Adjacency");
  delete_layout->addWidget(delete_adjacency);

  QHBoxLayout* move_base_layout = new QHBoxLayout;
  move_base_button = new QPushButton("Launch move_base");
  move_base_layout->addWidget(move_base_button);
  // move_base_layout->addWidget( new QLabel( "Using files: " ));

  QHBoxLayout* station_layout = new QHBoxLayout;
  station_layout->addWidget( new QLabel( "Station:" ));
  QSpinBox* station_id = new QSpinBox;
  station_layout->addWidget(station_id);
  QComboBox *comboBox = new QComboBox;
  comboBox->addItem("Pick Up Object");
  comboBox->addItem("Drop Object");
  comboBox->addItem("Charge Bot");
  station_layout->addWidget(comboBox);
  QCheckBox *checkbox = new QCheckBox("Stay at Station");
  station_layout->addWidget(checkbox);

  stations_layout = new QVBoxLayout;
  QCheckBox *loop = new QCheckBox("Loop Stations");
  stations_layout->addWidget(loop);
  stations_layout->addLayout( station_layout );

  QHBoxLayout* mission_layout = new QHBoxLayout;
  add_button = new QPushButton("Add Station");
  mission_layout->addWidget(add_button);
  mission_button = new QPushButton("Publish Mission");
  mission_layout->addWidget(mission_button);
  abort_button = new QPushButton("Abort Mission");
  mission_layout->addWidget(abort_button);
  
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout( topic_layout );
  layout->addLayout( topic_layout2 );
  layout->addLayout( points_layout );
  layout->addLayout( delete_layout );
  layout->addLayout(move_base_layout);
  layout->addLayout(stations_layout);
  layout->addLayout(mission_layout);
  setLayout( layout );

  connect( point_file_editor_, SIGNAL( editingFinished() ), this, SLOT( updatePoints() ));
  connect( adjacency_file_editor_, SIGNAL( editingFinished() ), this, SLOT( updateAdjacency() ));
  connect(move_base_button, SIGNAL (released()), this, SLOT (movebaseButton()));
  connect(freeze_button, SIGNAL (released()), this, SLOT (freezeButton()));
  connect(load_button, SIGNAL (released()), this, SLOT (loadButton()));
  connect(add_button, SIGNAL (released()), this, SLOT (addButton()));
  connect(mission_button, SIGNAL (released()), this, SLOT (missionButton()));
  connect(abort_button, SIGNAL (released()), this, SLOT (abortButton()));
  connect(delete_points, SIGNAL (released()), this, SLOT (deletePoints()));
  connect(delete_adjacency, SIGNAL (released()), this, SLOT (deleteAdjacency()));

  commands = nh_.advertise<std_msgs::Int32>("/panel_msgs", 5);
  mission_pub = nh_.advertise<std_msgs::String>("/mission", 5);
  points_pub = nh_.advertise<std_msgs::String>("/points_files", 5);

  move_base_button->setEnabled(false);
  loop->setEnabled(false);

}

void WaypointPanel::updatePoints()
{
  point_file_ = point_file_editor_->text();
  Q_EMIT configChanged();
  // move_base_button->setEnabled( point_file_ != "" && adjacency_file_ != "");
  load_button->setEnabled( point_file_ != "");
}

void WaypointPanel::updateAdjacency()
{
  adjacency_file_ = adjacency_file_editor_->text();
  Q_EMIT configChanged();
  // move_base_button->setEnabled( point_file_ != "" && adjacency_file_ != "");
  load_button->setEnabled( point_file_ != "");
}

void WaypointPanel::freezeButton(){
  std_msgs::Int32 msg;
  msg.data = 1;
  commands.publish(msg);

  std::stringstream filename, adjacency;
  filename << "/home/" << getenv("USER") << "/catkin_ws/src/navgraph_solver/navgraphs/points_rviz.csv";
  adjacency << "/home/" << getenv("USER") << "/catkin_ws/src/navgraph_solver/navgraphs/adjacency_rviz.csv";

  point_file_editor_->setText(QString::fromStdString(filename.str()));
  updatePoints();
  adjacency_file_editor_->setText(QString::fromStdString(adjacency.str()));
  updateAdjacency();
}

void WaypointPanel::loadButton(){
  std_msgs::Int32 msg;
  msg.data = 2;
  commands.publish(msg);
  std_msgs::String files;
  std::stringstream s;
  s << point_file_editor_->text().toStdString() << "+" << adjacency_file_editor_->text().toStdString();
  files.data = s.str();
  points_pub.publish(files);
}

void WaypointPanel::movebaseButton(){
  std_msgs::Int32 msg;
  msg.data = 3;
  commands.publish(msg);
}

void WaypointPanel::addButton(){
  QHBoxLayout* station_layout = new QHBoxLayout;
  station_layout->addWidget( new QLabel( "Station:" ));
  QSpinBox* station_id = new QSpinBox;
  station_layout->addWidget(station_id);
  QComboBox *comboBox = new QComboBox;
  comboBox->addItem("Pick Up Object");
  comboBox->addItem("Drop Object");
  comboBox->addItem("Charge Bot");
  station_layout->addWidget(comboBox);
  QCheckBox *checkbox = new QCheckBox("Stay at Station");
  station_layout->addWidget(checkbox);

  stations_layout->addLayout( station_layout );
}

void WaypointPanel::missionButton(){
  std_msgs::String msg;
  std::stringstream s;
  for (int i = 1; i < stations_layout->count(); i++){
    QWidget *station_widget = stations_layout->itemAt(i)->layout()->itemAt(1)->widget();
    QWidget *task_widget = stations_layout->itemAt(i)->layout()->itemAt(2)->widget();
    QWidget *stay_widget = stations_layout->itemAt(i)->layout()->itemAt(3)->widget();
    QSpinBox *station_widget_cast;
    QComboBox *task_widget_cast;
    QCheckBox *stay_widget_cast;
    station_widget_cast = qobject_cast<QSpinBox*>(station_widget);
    task_widget_cast = qobject_cast<QComboBox*>(task_widget);
    stay_widget_cast = qobject_cast<QCheckBox*>(stay_widget);
    int station = station_widget_cast->value();
    int task = task_widget_cast->currentIndex();
    bool stay = stay_widget_cast->isChecked();

    s << station << " " << task << " " << stay << "/";

  }
  msg.data = s.str();
  mission_pub.publish(msg);
  mission_button->setEnabled(false);
}

void WaypointPanel::abortButton(){
  std_msgs::Int32 msg;
  msg.data = 4;
  commands.publish(msg);
  mission_button->setEnabled(true);
}

void WaypointPanel::deletePoints(){
  std_msgs::Int32 msg;
  msg.data = 5;
  commands.publish(msg);
}

void WaypointPanel::deleteAdjacency(){
  std_msgs::Int32 msg;
  msg.data = 6;
  commands.publish(msg);
}

void WaypointPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Points", point_file_ );
  config.mapSetValue( "Adjacency", adjacency_file_ );
}

void WaypointPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  if( config.mapGetString( "Points", &point_file_ ))
  {
    point_file_editor_->setText( point_file_ );
    updatePoints();
  }
  if( config.mapGetString( "Adjacency", &adjacency_file_ ))
  {
    adjacency_file_editor_->setText( adjacency_file_ );
    updateAdjacency();
  }
}

} // end namespace rviz_plugin_tutorials

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navgraph_rviz_panel::WaypointPanel,rviz::Panel )
