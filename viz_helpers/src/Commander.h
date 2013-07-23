#ifndef __COMMANDER_H__
#define __COMMANDER_H__

#define slots Q_SLOTS
#include <rqt_gui_cpp/plugin.h>

#include <ros/ros.h>

#include <QWidget>
#include <QToolBar>
#include <QMainWindow>

#include <set>

#include <viz_helpers/CommandPanelFeedback.h>

namespace viz_helpers {

class ActionBundle ;

class CommandPanel: public QWidget
{
    Q_OBJECT

public:

    CommandPanel(ros::NodeHandle &handle_, const std::string &topic_ns, QWidget *parent = 0) ;

public Q_SLOTS:

    QAction *addCommand(const QString &cmd) ;

private Q_SLOTS:

    void onCommand() ;

Q_SIGNALS:
    void addCommandRequested(QString) ;

private:

    QAction *createActionOnExisting(QMenu *m, QStringList &menus) ;
    QAction *addSubMenuAction(QAction *act,  QStringList &menus ) ;

    QToolBar *toolBar ;

    QList<QMenu *> mbarMenus;
    ros::NodeHandle nh ;

    ros::Publisher feedback_pub_ ;
    ros::Subscriber command_sub_ ;

    void addCommandCallback(const viz_helpers::CommandPanelFeedback::ConstPtr& msg) ;
};


class CommandPanelPlugin: public rqt_gui_cpp::Plugin
{

public:
    CommandPanelPlugin();

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();

    CommandPanel *widget_ ;
};

} // namespace

#endif
