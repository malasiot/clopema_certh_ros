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

QCameraView::QCameraView(QWidget *parent, ros::NodeHandle handle_, const vector<string> &imageTopics, const std::string &topic_ns):
    QScrollArea(parent), handle(handle_), topics(imageTopics)
{

    setAlignment(Qt::AlignHCenter | Qt::AlignCenter ) ;

    vbl = new QGridLayout(this);

    setLayout(vbl) ;

    imageWidget = new QCameraWidget(this, "camera") ;

    sourceCombo = NULL ;

    if ( imageTopics.size() > 1 )
    {
        sourceCombo = new QComboBox(this) ;
        connect(sourceCombo, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int))) ;
        vbl->addWidget(sourceCombo, 0, 0 ) ;

        for (vector<string>::const_iterator it = topics.begin(); it != topics.end(); it++)
        {
            QString label((*it).c_str());
            label.replace(" ", "/");
            sourceCombo->addItem(label, QVariant((*it).c_str()));
        }
    }

    connect(this, SIGNAL(frameChanged(QImage)), imageWidget, SLOT(updateFrame(QImage)));

    vbl->addWidget(imageWidget, 1, 0, Qt::AlignHCenter | Qt::AlignCenter ) ;

    installEventFilter(this);

    if ( !sourceCombo )
        setTopic(imageTopics[0]) ;

    setFocus() ;

    string feedback_topic = topic_ns + "/camera_viewer/feedback" ;

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

        if ( kpe->key() == Qt::Key_Plus )
            imageWidget->zoomUp() ;
        else if ( kpe->key() == Qt::Key_Minus )
            imageWidget->zoomDown() ;

    }
    else if (e->type() == QEvent::MouseButtonRelease && needMouseFeedback)
    {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(e);

        CameraViewFeedback feedback ;

        QPoint p ;

        if ( imageWidget->mapPoint(mouseEvent->pos(), p) )
        {
            feedback.mouse_point.x = p.x() ;
            feedback.mouse_point.y = p.y() ;

            feedback_pub_.publish(feedback) ;
        }

        return false;
    }


}

void QCameraView::setTopic(const std::string &topic)
{

    subscriber_.shutdown();

    if ( !topic.empty() )
    {
        image_transport::ImageTransport it(handle);
        image_transport::TransportHints hints("raw");

        try {
            subscriber_ = it.subscribe(topic, 1, &QCameraView::callbackImage, this, hints);

        } catch (image_transport::TransportLoadException& e) {
        QMessageBox::warning(this, tr("Loading image transport plugin failed"), e.what());
        }
    }
}


void QCameraView::onTopicChanged(int index)
{

    QStringList parts = sourceCombo->itemData(index).toString().split(" ");
    QString topic = parts.first();
    QString transport = parts.length() == 2 ? parts.last() : "raw";

    setTopic(topic.toStdString()) ;
}



static void hsv2rgb(float h, QRgb &rgb)
{
    int i ;
    float f, p, q, t, r, g, b ;

    if ( h == 0.0 ) return ;

            // h = 360.0-h ;

    h /= 60.0 ;

    i = (int)h ; f = h - i ; p = 0  ; q = 1-f ;	t = f ;

    switch (i)
    {
        case 0:
            r = 1 ;
            g = t ;
            b = p ;
            break ;
        case 1:
            r = q ;
            g = 1 ;
            b = p ;
            break ;
        case 2:
            r = p ;
            g = 1 ;
            b = t ;
            break ;
        case 3:
            r = p ;
            g = q ;
            b = 1 ;
            break ;
        case 4:
            r = t ;
            g = p ;
            b = 1 ;
            break ;
        case 5:
            r = 1 ;
            g = p ;
            b = q ;
            break ;
    }

    rgb = qRgb((int)(255.0*r), (int)(255.0*g), (int)(255.0*b)) ;
}

const int nColors = 2 << 12 ;
static QRgb *hsvlut = NULL ;

void QCameraView::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{

    cv::Mat conversion_mat_ ;

    try
    {

        if (sensor_msgs::image_encodings::isColor(msg->encoding) || sensor_msgs::image_encodings::isMono(msg->encoding))
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
            conversion_mat_ = cv_ptr->image;
        }
        else
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
            conversion_mat_ = cv_ptr->image ;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        return ;
    }


    int w = conversion_mat_.cols, h = conversion_mat_.rows, lw = conversion_mat_.step[0] ;


    if ( conversion_mat_.type() == CV_8UC1 )
    {
        QImage image((uchar *)conversion_mat_.data, w, h, lw, QImage::Format_Indexed8) ;

        QVector<QRgb> colors ;
        for( int i=0 ; i<256 ; i++ ) colors.append(QColor(i, i, i).rgba()) ;

        image.setColorTable(colors) ;

        QMutexLocker lock(&image_) ;
        Q_EMIT frameChanged(image.copy()) ;

    }
    else if ( conversion_mat_.type() == CV_8UC3 )
    {

        QImage image(w, h,QImage::Format_RGB32) ;

        for( int i=0 ; i<h ; i++ )
        {
            uchar *dst = image.scanLine(i), *src = (uchar *)conversion_mat_.ptr<uchar>(i) ;

            for( int j=0 ; j<w ; j++ )
            {
                uchar R = *src++ ;
                uchar G = *src++ ;
                uchar B = *src++ ;

                *(QRgb *)dst = qRgb(R, G, B) ;
                dst += 4 ;
            }
        }

        Q_EMIT frameChanged(image) ;

    }
    else if ( conversion_mat_.type() == CV_16UC1 )
    {
        int nc = nColors ;

        if ( !hsvlut )
        {
            int c ;
            float h, hmax, hstep ;

            hsvlut = new QRgb [nColors] ;

            hmax = 180 ;
            hstep = hmax/nc ;

            for ( c=0, h=hstep ; c<nc ; c++, h += hstep) hsv2rgb(h, hsvlut[c]) ;
        }

        unsigned short minv, maxv ;
        int i, j ;

        minv = 0xffff ;
        maxv = 0 ;

        uchar *ppl = conversion_mat_.data ;
        unsigned short *pp = (unsigned short *)ppl ;

        for ( i=0 ; i<h ; i++, ppl += lw )
            for ( j=0, pp = (unsigned short *)ppl ; j<w ; j++, pp++ )
            {
                if ( *pp == 0 ) continue ;
                maxv = qMax(*pp, maxv) ;
                minv = qMin(*pp, minv) ;
            }

        QImage image(w, h, QImage::Format_RGB32) ;

        for( i=0 ; i<h ; i++ )
        {
            uchar *dst = image.scanLine(i) ;
            unsigned short *src = (unsigned short *)conversion_mat_.ptr<ushort>(i) ;

            for( j=0 ; j<w ; j++ )
            {
                unsigned short val = *src++ ;

                if ( val == 0 )
                {
                    *(QRgb *)dst = Qt::black ;
                    dst += 3 ;
                    *dst++ = 255 ;

                    continue ;
                }
                else val = (nc-1)*float((val - minv)/float(maxv - minv)) ;

                const QRgb &clr = hsvlut[val] ;

                *(QRgb *)dst = clr ;
                dst += 3 ;
                *dst++ = 255 ;
            }
        }

        Q_EMIT frameChanged(image) ;

    }
}

}
