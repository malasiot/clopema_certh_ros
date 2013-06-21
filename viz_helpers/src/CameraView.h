#ifndef _CAMERA_VIEW_H_
#define _CAMERA_VIEW_H_

#include "CameraWidget.h"

#include <QtGui/QScrollArea>
#include <QtGui/QGridLayout>
#include <QtGui/QComboBox>

#include <image_transport/subscriber.h>

#include <ros/callback_queue.h>



#include "viz_helpers/CameraViewFeedback.h"

namespace viz_helpers {

class QCameraView: public QScrollArea
{
	Q_OBJECT

public:

    QCameraView(QWidget *parent, ros::NodeHandle handle, const std::string &topic_ns = std::string()) ;
    ~QCameraView() ;

     void cleanup() ;

public Q_SLOTS:

    void updateFrame(const QImage &im) ;
    void onTopicChanged(int index) ;

 Q_SIGNALS:

    void frameChanged(const QImage &im) ;
protected:

	bool eventFilter(QObject *o, QEvent *e) ;

    void updateTopicList() ;
    QList<QString> getTopicList(const QSet<QString>& message_types, const QList<QString>& transports) ;
    void selectTopic(const QString& topic) ;

private:
    QCameraWidget *imageWidget ;
    QComboBox *sourceCombo ;

	QScrollArea *container ;
	QGridLayout *vbl ;

    image_transport::Subscriber subscriber_;
    ros::Publisher feedback_pub_ ;

    QMutex image_ ;
    ros::NodeHandle handle ;

    boost::mutex connect_mutex_ ;
    bool needMouseFeedback ;

    virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);

    void feedbackCb(const ros::SingleSubscriberPublisher &pub) ;
} ;




}

#endif
