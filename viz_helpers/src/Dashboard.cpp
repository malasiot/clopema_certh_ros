#include "Dashboard.h"
#include <pluginlib/class_list_macros.h>
#include <robot_helpers/Robot.h>
#include <robot_helpers/Utils.h>

using namespace robot_helpers ;
using namespace std ;

namespace viz_helpers {


RobotDashboardPlugin::RobotDashboardPlugin()
  :  rqt_gui_cpp::Plugin()
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("CloPeMa Robot Dashboard Plugin");

}

void RobotDashboardPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
    QStringList argv = context.argv() ;

    widget_ = new RobotDashboard(getNodeHandle(), 0) ;

    if (context.serialNumber() > 1)
    {
         widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }

    context.addWidget(widget_);
}

void RobotDashboardPlugin::shutdownPlugin()
{

}

void RobotDashboardPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    /*
  QString topic = ui_.topics_combo_box->currentText();
  //qDebug("ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
  instance_settings.setValue("topic", topic);
  instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
  instance_settings.setValue("dynamic_range", ui_.dynamic_range_check_box->isChecked());
  instance_settings.setValue("max_range", ui_.max_range_double_spin_box->value());
  */
}

void RobotDashboardPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    /*
  bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
  ui_.zoom_1_push_button->setChecked(zoom1_checked);

  bool dynamic_range_checked = instance_settings.value("dynamic_range", false).toBool();
  ui_.dynamic_range_check_box->setChecked(dynamic_range_checked);

  double max_range = instance_settings.value("max_range", ui_.max_range_double_spin_box->value()).toDouble();
  ui_.max_range_double_spin_box->setValue(max_range);

  QString topic = instance_settings.value("topic", "").toString();
  //qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
  selectTopic(topic);
  */
}


//////////////////////////////////////////////////////////////////////////////////////////

void RobotDashboard::openLeftGripper()
{
    ui.openLeftGripperButton->setEnabled(false);
    ros::Duration(ui.gripperDelay->value()).sleep() ;
    if ( !setGripperState("r2", true) )
        ui.messageBox->setText("Failed to open left hand gripper.") ;
    updateState() ;
}

void RobotDashboard::openRightGripper()
{
    ui.openRightGripperButton->setEnabled(false);
    ros::Duration(ui.gripperDelay->value()).sleep() ;
    if ( !setGripperState("r1", true) )
        ui.messageBox->setText("Failed to open right hand gripper.") ;
    updateState() ;
}


void RobotDashboard::closeLeftGripper()
{
    ui.closeLeftGripperButton->setEnabled(false);
    ros::Duration(ui.gripperDelay->value()).sleep() ;
    if ( !setGripperState("r2", false) )
        ui.messageBox->setText("Failed to close left hand gripper.") ;
    updateState() ;
}

void RobotDashboard::closeRightGripper()
{
    ui.closeRightGripperButton->setEnabled(false);
    ros::Duration(ui.gripperDelay->value()).sleep() ;
    if ( !setGripperState("r1", false) )
        ui.messageBox->setText("Failed to close right hand gripper.") ;
    updateState() ;
}

void RobotDashboard::setServoPowerOff()
{
       robot_helpers::setServoPowerOff();
}

void RobotDashboard::updateState()
{
    bool gripper_state ;

    if ( getGripperState("r1", gripper_state) )
    {
        ui.openRightGripperButton->setEnabled(!gripper_state) ;
        ui.closeRightGripperButton->setEnabled(gripper_state) ;
    }

    if ( getGripperState("r2", gripper_state) )
    {
        ui.openLeftGripperButton->setEnabled(!gripper_state) ;
        ui.closeLeftGripperButton->setEnabled(gripper_state) ;
    }
}

void RobotDashboard::moveHome()
{
 //   robot_helpers::setRobotSpeed(0.1) ;
    MoveRobot mv ;
    robot_helpers::moveHome(mv) ;
}

} // namespace

PLUGINLIB_DECLARE_CLASS(viz_helpers, RobotDashboardPlugin, viz_helpers::RobotDashboardPlugin, rqt_gui_cpp::Plugin)
