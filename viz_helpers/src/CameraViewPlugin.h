#ifndef __INTERACTIVE_IMAGE_VIEW_PLUGIN_H__
#define __INTERACTIVE_IMAGE_VIEW_PLUGIN_H__

#include <rqt_gui_cpp/plugin.h>

#include <QWidget>
#include <ros/ros.h>
#include "CameraView.h"

namespace viz_helpers {



class CameraViewPlugin
  : public rqt_gui_cpp::Plugin
{

public:
  CameraViewPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();

   QCameraView *widget_ ;
   ros::NodeHandle handle ;
};
} // namespace
#endif 
