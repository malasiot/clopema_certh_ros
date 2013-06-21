#include "CameraViewPlugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include "CameraView.h"

namespace viz_helpers {

CameraViewPlugin::CameraViewPlugin()
  :  rqt_gui_cpp::Plugin()
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("Interactive Image Viewer Plugin");

}

void CameraViewPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
    QStringList argv = context.argv() ;

    widget_ = new QCameraView(0, getNodeHandle()) ;

    if (context.serialNumber() > 1)
    {
         widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }

    context.addWidget(widget_);
}
void CameraViewPlugin::shutdownPlugin()
{
    widget_->cleanup()  ;
}

void CameraViewPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
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

void CameraViewPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
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

} // namespace
PLUGINLIB_DECLARE_CLASS(viz_helpers, CameraViewPlugin, viz_helpers::CameraViewPlugin, rqt_gui_cpp::Plugin)
