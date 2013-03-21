#ifndef MUSICAPPLET_H
#define MUSICAPPLET_H

#include "applet.h"

#include <QApplication>
#include <QSlider>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include <QTime>
#include <QCloseEvent>
#include <phonon/audiooutput.h>
#include <phonon/seekslider.h>
#include <phonon/mediaobject.h>
#include <phonon/backendcapabilities.h>

#include "filters/calibrationFilter.h"
#include "filters/meanFilter.h"
#include "filters/objectDetectionFilter.h"

#define MUSIC_NAME "music"
#define MUSIC_TITLE "Music"

// Delay in # of frames
#define MUSIC_RESURECT_LENGTH 15

#define MUSIC_SOUND_FILE "/data/sound/shakira.mp3"
#define MUSIC_COVER ":image/music/shakira.jpg"

/**
 *
 */
class MusicApplet : public Applet
{
    Q_OBJECT

public:

    static const QString TAG;

    MusicApplet(QWidget *parent = 0);

protected:

    void closeEvent(QCloseEvent * event);

    void hideEvent(QHideEvent *e) {
        if( m_mediaObject ) {
            m_mediaObject->pause();
            setPlayingButton();
        }
        Applet::hideEvent(e);
    }

    void showEvent(QShowEvent *e) {
        if( m_mediaObject ) {
            m_mediaObject->play();
            setStoppingButton();
        }
        Applet::showEvent(e);
    }

public slots:

    void setFrame( cv::Mat& frame );

    void reset();

    // resurect last valid input from user
    void resurect();

    void loopMusic();

    void changeVolume( int );

    void tick(qint64 time);

    void setPlayingButton()
    {
        playButton->setEnabled(true);
        pauseButton->setEnabled(false);
        stopButton->setEnabled(false);
    }

    void setStoppingButton()
    {
        playButton->setEnabled(false);
        pauseButton->setEnabled(true);
        stopButton->setEnabled(true);
    }

private:
    CalibrationFilter *mCalibrationFilter;
    MeanFilter *mMeanFilter;
    ObjectDetectionFilter *mObjectDetectionFilter;

    QList<double> mList;
    double mLastValue;

    // UI
    QSlider *m_slider;
    QPicture *m_image;
    QLabel *m_time;

    QPushButton *playButton;
    QPushButton *pauseButton;
    QPushButton *stopButton;

    Phonon::SeekSlider *m_seekSlider;
    Phonon::MediaObject *m_mediaObject;
    Phonon::AudioOutput *m_audioOutput;
    Phonon::Path m_path;
};

#endif // MUSICAPPLET_H
