#include "Commander.h"
#include <pluginlib/class_list_macros.h>

#include <viz_helpers/CommandPanelFeedback.h>

#include <qglobal.h>
#include <QAction>
#include <QMenu>
#include <QMetaMethod>
#include <QMenuBar>
#include <QToolButton>
#include <QDebug>
#include <QHBoxLayout>

using namespace std ;

namespace viz_helpers {


QAction *CommandPanel::addCommand(const QString &command)
{
    QStringList menus = command.split("/", QString::SkipEmptyParts) ;

    QAction *res = NULL ;

    // Check if is to be inserted in an existing menu

    QMenu *m ;
    Q_FOREACH(m, mbarMenus)
    {
        if ( m->title() == menus[0] ) {
            res = createActionOnExisting(m, menus) ;

        }

    }

    if ( !res )
    {
        if ( menus.size() == 1 )
        {
            res = toolBar->addAction( menus[0] ) ;
        }
        else
        {
            m = new QMenu(menus[0], toolBar) ;

            QToolButton* toolButton = new QToolButton(toolBar);
            toolButton->setText(menus[0]) ;
            toolButton->setMenu(m);
            toolButton->setPopupMode(QToolButton::InstantPopup);
            toolBar->addWidget(toolButton);

            res = createActionOnExisting(m, menus) ;

            mbarMenus.push_back(m) ;
        }

    }

    res->setData(command) ;
    connect(res, SIGNAL(triggered()), this, SLOT(onCommand())) ;

    return res ;



}


QAction *CommandPanel::createActionOnExisting(QMenu *m, QStringList &menus)
{
    QList<QAction *> olist = m->actions() ;

    menus.removeAt(0) ;

    // Check if item already inserted

    QAction *a ;
    Q_FOREACH(a, olist)
    {
        if ( a->text() == menus[0] )
            return createActionOnExisting(a->menu(), menus) ;
    }

    if ( menus.size() == 1 ) // This is an action
    {
        QAction *newAction ;

        if ( menus[0] == "--" )
        {
            newAction = m->addSeparator() ;
            return newAction ;
        }
        else
            newAction = m->addAction(menus[0]) ;

        return newAction ;
    }
    else // This is a sub-menu
    {
        QAction *subMenu ;

        subMenu = m->addMenu(new QMenu(menus[0])) ;

        return createActionOnExisting(subMenu->menu(), menus) ;
    }

}

QAction * CommandPanel::addSubMenuAction(QAction *act,  QStringList &menus )
{
    QMenu *menu = act->menu() ;

    if ( menu )
    {
        if ( menus.size() == 1 ) // action
        {
            if ( menus[0] == "--" )
            {
                menu->addSeparator() ;
                return NULL ;
            }
            else
            {
                QAction *action = menu->addAction(menus[0]) ;
                return action ;
            }
        }
        else
        {
            QAction *subMenu = menu->addMenu(new QMenu(menus[0])) ;
            menus.removeAt(0) ;
            return addSubMenuAction(subMenu, menus) ;
        }
    }
}

void CommandPanel::onCommand()
{
    QAction *action = qobject_cast<QAction *>(sender()) ;

    CommandPanelFeedback fb ;
    fb.command = action->data().toString().toStdString() ;

    feedback_pub_.publish(fb) ;
}

CommandPanel::CommandPanel(ros::NodeHandle &handle_, const string &topic_ns, QWidget *parent): nh(handle_), QWidget(parent) {

    QHBoxLayout *layout = new QHBoxLayout(this) ;
    layout->setContentsMargins(QMargins());

    toolBar = new QToolBar(this) ;

    layout->addWidget(toolBar) ;

    setLayout(layout) ;

    string feedback_topic = topic_ns + "/command_panel/feedback" ;
    string command_topic = topic_ns + "/command_panel/command" ;

    feedback_pub_ = nh.advertise<viz_helpers::CommandPanelFeedback>(feedback_topic, 1) ;
    command_sub_ = nh.subscribe<viz_helpers::CommandPanelFeedback>(command_topic, 1, boost::bind(&CommandPanel::addCommandCallback, this, _1)) ;

    connect(this, SIGNAL(addCommandRequested(QString)), this, SLOT(addCommand(QString))) ;

}

void CommandPanel::addCommandCallback(const viz_helpers::CommandPanelFeedback::ConstPtr& msg)
{
    Q_EMIT addCommandRequested(QString().fromStdString(msg->command)) ;
}

////////////////////////////////////////////////////////////////////////////

CommandPanelPlugin::CommandPanelPlugin()
  :  rqt_gui_cpp::Plugin()
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("CloPeMa Command Panel plugin");

}

void CommandPanelPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
    QStringList argv = context.argv() ;

    widget_ = new CommandPanel(this->getNodeHandle(), "", 0) ;

    if (context.serialNumber() > 1)
    {
         widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }

    context.addWidget(widget_);
}

void CommandPanelPlugin::shutdownPlugin()
{

}

void CommandPanelPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
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

void CommandPanelPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
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

PLUGINLIB_DECLARE_CLASS(viz_helpers, CommandPanelPlugin, viz_helpers::CommandPanelPlugin, rqt_gui_cpp::Plugin)
