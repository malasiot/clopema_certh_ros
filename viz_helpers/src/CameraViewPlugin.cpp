#include "CameraViewPlugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include "CameraView.h"
#include <set>

using namespace std ;

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

    widget_ = new QCameraView(0, getNodeHandle(), getTopicList()) ;

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

std::vector<std::string> CameraViewPlugin::getTopicList()
{
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);

    set<string> message_types;
    message_types.insert("sensor_msgs/Image");

    // get declared transports
    QList<QString> transports;

    transports.push_back("raw") ;


    set<string> all_topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    {
        all_topics.insert(it->name);
    }

    vector<string> topics;

    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    {
        if ( it->datatype == "sensor_msgs/Image")
        {
            string topic = it->name ;

            // add raw topic
            topics.push_back(topic);
            //qDebug("ImageView::getTopicList() raw topic '%s'", topic.toStdString().c_str());

            if (all_topics.count(topic + "/raw"))
            {
                string sub = topic + " raw";
                topics.push_back(sub);
                    //qDebug("ImageView::getTopicList() transport specific sub-topic '%s'", sub.toStdString().c_str());

            }
        }
    }
    return topics;



}



} // namespace
PLUGINLIB_DECLARE_CLASS(viz_helpers, CameraViewPlugin, viz_helpers::CameraViewPlugin, rqt_gui_cpp::Plugin)
