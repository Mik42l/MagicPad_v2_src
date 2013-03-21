#include "vumeterWidget.h"

const QString VuMeterWidget::TAG = QString("VuMeterWidget");

/**
 *
 */
VuMeterWidget::VuMeterWidget(QWidget *parent) :
    QWidget(parent)
{
    setLevelNumber(9);
    setRange(1, 100);

    QPalette p(palette());
    p.setColor(QPalette::Background, Qt::transparent);
    this->setPalette(p);
}

/**
 *
 */
void VuMeterWidget::draw(QPainter *painter)
{
    // 1 - draw rectangle in the right place X
    // 2 - color rectangle X
    // 3 - style, halo, ON/OFF
    // 4 - shapes, ...

    // Vertical step between levels (in pixels)
    const double vstep = 100.0 / (m_levels + 2);
    // Value interval width
    const double w = (m_max-m_min)/m_levels;
    // minimum brightness
    const int bmin = 20;

    // translate 1 step downward
    painter->translate(0, vstep);

    // draw m_levels rectangles
    for(int i=m_levels-1; i>=0; --i)
    {
        /// compute brigthness
        int brigthness;
        if(m_value >= m_min + (i+1)*w)
        {
            // Full ON
            brigthness = 255;
        }
        else if(m_value <= m_min + i*w)
        {
            // Full OFF
            brigthness = bmin;
        }
        else
        {
            // Proportionnal brigthness
            brigthness = bmin + (255-bmin)*(m_value - i*w)/w;
        }

        // Draw level
        QColor h1, h2;
        h1.setHsv(m_colormap[i], 255, brigthness);
        h2.setHsv(m_colormap[i], 128, brigthness);

        QRect rectangle(-15, 0, 30, 0.8*vstep);

        // TODO: Draw shadow

        // Draw color
        QLinearGradient haloGradient(-1, 0, +0, 0.8*vstep);
        haloGradient.setColorAt(0.0, h2);
        haloGradient.setColorAt(0.4, h1);

        painter->setBrush(haloGradient);
        painter->setPen(Qt::NoPen);

        painter->drawRoundRect(rectangle, 10, 50);
        painter->translate(0, vstep);
    }
}

/**
 *
 */
void VuMeterWidget::setRange(int _min, int _max)
{
    if(_min == _max)
    {
        _min = 0;
        _max= 100;
    }

    m_min = _min;
    m_max = _max;
}

/**
 *
 */
void VuMeterWidget::setLevelNumber(int n)
{
    if(n > 2 && n < 20)
    {
        m_levels = n;

        // Build colormap:
        // Green (Hue= 120deg) to Red (Hue= 0deg)
        m_colormap.resize(m_levels);
        for(int i=0; i<m_levels; i++)
        {
            m_colormap[i] = (m_levels-i-1) * 120 / (m_levels-1);
        }
    }
}

/**
 *
 */
void VuMeterWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    /*painter.setBrush(Qt::NoBrush);
    painter.setPen(Qt::red);
    painter.drawRect(rect());*/

    int side = qMin(width(), height());
    int sideV = qMin(1.5*side, (double)height());
    painter.setViewport((width() - side) / 2, (height() - sideV) / 2,
                        side, sideV);
    painter.setWindow(-50, 0, 100, 100);
    draw(&painter);
}

/**
 *
 */
void VuMeterWidget::setValue(int v)
{
    // clamp 'v' in [m_min m_max]
    m_value = std::min(m_max, std::max(m_min, v));
    repaint();
}
