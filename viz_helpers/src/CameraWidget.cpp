#include "CameraWidget.h"

#include <QDebug>
#include <QPainter>
#include "QSingleItemSquareLayout.h"

using namespace std ;

QCameraWidget::QCameraWidget(QWidget *parent, const QString &label): QWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus) ;

    pLabel = new QLabel(this) ;

    vLabel = new QLabel(this) ;
    vLabel->setMargin(5) ;

    QPalette palette;

    //white text
    QBrush brush(QColor(255, 0, 0, 255));
    brush.setStyle(Qt::SolidPattern);

    //set white text
    palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
    palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);

        //set palette
    vLabel->setPalette(palette);

    QSingleItemSquareLayout *vbl = new QSingleItemSquareLayout(this) ;

    vbl->addWidget(pLabel) ;

    vbl->setContentsMargins(0, 0, 0, 0);

    setLayout(vbl) ;

    if ( !label.isEmpty() )
        vLabel->setText(label) ;

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



void QCameraWidget::updateFrame(const QImage &frame)
{
	int w = 640;
	int h = 480;


    pLabel->setPixmap(QPixmap::fromImage(frame.scaled(w,h,Qt::KeepAspectRatio)));
    // pLabel->setPixmap(QPixmap::fromImage(frame));
    //resize(frame.size()) ;
}


