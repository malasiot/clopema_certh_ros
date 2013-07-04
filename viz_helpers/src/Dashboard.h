#ifndef __DASHBOARD_H__
#define __DASHBOARD_H__

#include "ui_Dashboard.h"

#define slots Q_SLOTS
#include <rqt_gui_cpp/plugin.h>

#include <QWidget>
#include <ros/ros.h>

namespace viz_helpers {

class RobotDashboard: public QWidget
{
    Q_OBJECT

public:

    RobotDashboard(ros::NodeHandle nh, QWidget *parent = 0): QWidget(parent), handle(nh)
    {
        ui.setupUi(this);
    }

private Q_SLOTS:
    void toggleLeftGripper() {}

private:
    Ui::DashboardWidget ui;
    ros::NodeHandle handle ;
};


class RobotDashboardPlugin: public rqt_gui_cpp::Plugin
{

public:
  RobotDashboardPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();

   RobotDashboard *widget_ ;


};

} // namespace

#endif
