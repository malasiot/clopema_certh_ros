#ifndef _CAMERA_WIDGET_H_
#define _CAMERA_WIDGET_H_

#include <QWidget>
#include <QLabel>
#include <QMutex>

class QCameraWidget: public QWidget
{
     Q_OBJECT

public:

    QCameraWidget(QWidget *parent, const QString &label) ;
    ~QCameraWidget() ;

private:

    QLabel *pLabel, *vLabel ;
    int camView ;
     QMutex mutex ;

public Q_SLOTS:


    void updateFrame(const QImage &im) ;
};


#endif
