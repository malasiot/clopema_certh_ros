#include "CameraView.h"

#include <QtGui/QKeyEvent>

#include <ros/master.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <QMessageBox>
#include <QDebug>

using namespace std ;
namespace viz_helpers {

QCameraView::QCameraView(QWidget *parent, ros::NodeHandle handle_, const std::string &topic_ns):
    QScrollArea(parent), handle(handle_)
{

    setAlignment(Qt::AlignHCenter | Qt::AlignCenter ) ;

    vbl = new QGridLayout(this);

    setLayout(vbl) ;

    imageWidget = new QCameraWidget(this, "camera") ;
    sourceCombo = new QComboBox(this) ;

    connect(sourceCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int))) ;
    connect(this, SIGNAL(frameChanged(QImage)), imageWidget, SLOT(updateFrame(QImage)));

    vbl->addWidget(sourceCombo, 0, 0 ) ;
    vbl->addWidget(imageWidget, 1, 0, Qt::AlignHCenter | Qt::AlignCenter ) ;

    installEventFilter(this);

    setFocus() ;

    updateTopicList();

    string feedback_topic = topic_ns + "/camera_view/feedback" ;

    ros::SubscriberStatusCallback fcb = boost::bind(&QCameraView::feedbackCb, this, _1) ;
    feedback_pub_ = handle.advertise<viz_helpers::CameraViewFeedback>(feedback_topic, 10, fcb, fcb) ;
    needMouseFeedback = false ;
}

void QCameraView::feedbackCb(const ros::SingleSubscriberPublisher &pub)
{
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    needMouseFeedback = feedback_pub_.getNumSubscribers() > 0 ;

}

QCameraView::~QCameraView()
{
    cleanup() ;
}


void  QCameraView::cleanup()
{
    subscriber_.shutdown() ;
}

void QCameraView::updateFrame(const QImage &im)
{
    imageWidget->updateFrame(im) ;
}


bool QCameraView::eventFilter(QObject *o, QEvent *e)
{

    if ( e->type() == QEvent::KeyRelease )
    {
        QKeyEvent *kpe = (QKeyEvent *)e ;

    }
    else if (e->type() == QEvent::MouseButtonRelease && needMouseFeedback)
    {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(e);

        CameraViewFeedback feedback ;

        QPoint p = imageWidget->mapFromParent(mouseEvent->pos()) ;
        feedback.mouse_point.x = p.x() ;
        feedback.mouse_point.y = p.y() ;


        feedback_pub_.publish(feedback) ;
    }
    return false;


}


void QCameraView::updateTopicList()
{
    QSet<QString> message_types;
    message_types.insert("sensor_msgs/Image");

    // get declared transports
    QList<QString> transports;

    transports.push_back("raw") ;

    QString selected = sourceCombo->currentText();

    // fill combo box
    QList<QString> topics = getTopicList(message_types, transports);
    topics.append("");

    qSort(topics);

    sourceCombo->clear();

    for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
    {
        QString label(*it);
        label.replace(" ", "/");
        sourceCombo->addItem(label, QVariant(*it));
    }

  // restore previous selection
    selectTopic(selected);
}

QList<QString> QCameraView::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports)
{
    ros::master::V_TopicInfo topic_info;
    ros::master::getTopics(topic_info);

    QSet<QString> all_topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    {
        all_topics.insert(it->name.c_str());
    }

    QList<QString> topics;
    for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
    {
        if (message_types.contains(it->datatype.c_str()))
        {
            QString topic = it->name.c_str();

            // add raw topic
            topics.append(topic);
            //qDebug("ImageView::getTopicList() raw topic '%s'", topic.toStdString().c_str());

            // add transport specific sub-topics
            for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
            {
                if (all_topics.contains(topic + "/" + *jt))
                {
                    QString sub = topic + " " + *jt;
                    topics.append(sub);
                    //qDebug("ImageView::getTopicList() transport specific sub-topic '%s'", sub.toStdString().c_str());
                }
            }
        }
    }
    return topics;
}

void QCameraView::selectTopic(const QString& topic)
{
    int index = sourceCombo->findText(topic);
    if (index == -1)
    {
        index = sourceCombo->findText("");
    }
    sourceCombo->setCurrentIndex(index);
}

void QCameraView::onTopicChanged(int index)
{
    subscriber_.shutdown();

     QStringList parts = sourceCombo->itemData(index).toString().split(" ");
    QString topic = parts.first();
    QString transport = parts.length() == 2 ? parts.last() : "raw";

    if (!topic.isEmpty())
    {
        image_transport::ImageTransport it(handle);
        image_transport::TransportHints hints(transport.toStdString());

        try {
            subscriber_ = it.subscribe(topic.toStdString(), 1, &QCameraView::callbackImage, this, hints);
            qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
        } catch (image_transport::TransportLoadException& e) {
        QMessageBox::warning(this, tr("Loading image transport plugin failed"), e.what());
    }
  }
}

void QCameraView::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{

    cv::Mat conversion_mat_ ;

    try
    {

        // First check if we have a color format. If so, use the current tools for color conversion
        if (sensor_msgs::image_encodings::isColor(msg->encoding) || sensor_msgs::image_encodings::isMono(msg->encoding))
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
            conversion_mat_ = cv_ptr->image;
        }
        else
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
            if (msg->encoding == "CV_8UC3")
            {
                // assuming it is rgb
                conversion_mat_ = cv_ptr->image;
            } else if (msg->encoding == "8UC1") {
                // convert gray to rgb
                cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
            } else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
                // scale / quantify
                double min = 0;
                double max = 1 ; //ui_.max_range_double_spin_box->value();
                if (msg->encoding == "16UC1") max *= 1000;
                if (1)
                {
                    // dynamically adjust range based on min/max in image
                    cv::minMaxLoc(cv_ptr->image, &min, &max);
                    if (min == max) {
                        // completely homogeneous images are displayed in gray
                        min = 0;
                        max = 2;
                    }
                }
                cv::Mat img_scaled_8u;
                cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
                cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
            } else {
                qWarning("ImageView.callback_image() unhandled image encoding '%s'", msg->encoding.c_str());
                return ;

            }
        }
    }
    catch (cv_bridge::Exception& e)
    {
        qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        return ;

    }

  // copy temporary image as it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation

  QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, QImage::Format_RGB888);
  QImage res ;

  QMutexLocker lock(&image_) ;


  Q_EMIT frameChanged(image.copy()) ;

}


}
