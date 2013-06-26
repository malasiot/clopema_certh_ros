#include "CameraWidget.h"

#include <QDebug>
#include <QPainter>
#include "QSingleItemSquareLayout.h"

using namespace std ;

static float zoomFactors[] = { 0.25, 0.33, 0.5, 0.75, 1.0, 1.5, 2.0  } ;

QCameraWidget::QCameraWidget(QWidget *parent, const QString &label): QWidget(parent), zoom_factor_(4)
{
    setFocusPolicy(Qt::StrongFocus) ;

    pLabel = new QLabel(this) ;

    QSingleItemSquareLayout *vbl = new QSingleItemSquareLayout(this) ;

    vbl->addWidget(pLabel) ;

    vbl->setContentsMargins(0, 0, 0, 0);

    setLayout(vbl) ;

    QImage frame(640, 480, QImage::Format_RGB32) ;

    QPainter painter ;

    painter.begin(&frame) ;

    painter.fillRect(0, 0, 640, 480, "black") ;

    painter.setPen(QColor("red")) ;
    painter.drawText( 0, 0, 640, 480, Qt::AlignCenter | Qt::AlignHCenter, tr("No signal") ) ;

    painter.end() ;

    pLabel->setScaledContents(true) ;

    pLabel->setPixmap(QPixmap::fromImage(frame));
}



QCameraWidget::~QCameraWidget()	{

}


void QCameraWidget::zoomUp()
{
    QMutexLocker lock_(&mutex) ;

    int nf = sizeof(zoomFactors)/sizeof(float) ;
    zoom_factor_ = std::min(zoom_factor_ + 1, nf - 1) ;
}

void QCameraWidget::zoomDown()
{
    QMutexLocker lock_(&mutex) ;

    zoom_factor_ = std::max(zoom_factor_ - 1, 0) ;
}


void QCameraWidget::updateFrame(const QImage &frame)
{
    QMutexLocker lock_(&mutex) ;

    w_ = frame.width();
    h_ = frame.height();

    float scale = zoomFactors[zoom_factor_] ;

    pLabel->setPixmap(QPixmap::fromImage(frame.scaled(scale*w_,scale*h_,Qt::KeepAspectRatio)));
}

 bool QCameraWidget::mapPoint(const QPoint &parent, QPoint &child)
 {
     child = mapFromParent(parent) ;

     QRect rect = pLabel->geometry() ;

     int _x = child.x() ;
     int _y = child.y() ;

     unsigned int ox = rect.topLeft().x() ;
     unsigned int oy = rect.topLeft().y() ;
     unsigned int rw = rect.width() ;
     unsigned int rh = rect.height() ;

     _x -= ox ;
     _y -= oy ;

     if ( _x < 0 || _x >= rw || _y < 0 || _y >= rh ) return false ;

     child = QPoint(_x * (float)w_ /rw + 0.5, _y * (float)h_/rh + 0.5) ;

     return true ;

 }
