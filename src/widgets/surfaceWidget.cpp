#include "surfaceWidget.h"

const QString SurfaceWidget::TAG = QString("SurfaceWidget");

/**
 *
 */
SurfaceWidget::SurfaceWidget(QWidget *parent) : Qwt3D::SurfacePlot(parent)
{
    // Initialisation de l'état initial
    m_tabla = NULL;
    m_frameDouble = cv::Mat( 10, 10, CV_64F, cv::Scalar::all(0) );
    m_frameFiltered = cv::Mat( 10, 10, CV_64F, cv::Scalar::all(0) );

    setShift( 0.35, 0, 0 );
    setRotation( 60, 0, 80 );
    setUpscale( 5 );
    setCoordinateStyle( Qwt3D::NOCOORD );
    enableMouse( true );
    enableKeyboard( true );
    setPlotStyle( Qwt3D::FILLEDMESH );
    setSmoothMesh( true );

    // Lightning
    enableLighting( true );
    illuminate( 0 );
    setShading( Qwt3D::GOURAUD );
    setLightComponent( GL_DIFFUSE, 0.5 );
    setMaterialComponent( GL_SPECULAR, 0.1 );
    setShininess( 0.1 );

    // Create color vector
    m_cv.resize( 255 );
    for(int i=0; i<m_cv.size(); i++)
    {
        QColor c;
        c.setHsv(((255-i)*255.0)/255.0, 255.0, 255.0);
        m_cv[i] = Qwt3D::RGBA(c.red()/255., c.green()/255., c.blue()/255.);
    }
    Qwt3D::StandardColor *sc = new Qwt3D::StandardColor( this, 255 );
    sc->setColorVector( m_cv );
    setDataColor( sc );
    showColorLegend( true );
    legend()->setLimits( 0, 255 );
}


/**
 * Allocate memory to store 'double' version of high resolution frame
 */
void SurfaceWidget::allocateData()
{
    // TODO: fix desallocation
    if(m_tabla != NULL) delete [] m_tabla;

    m_hrWidth = 10 * m_resolution + 2;
    m_hrHeight = 10 * m_resolution + 2;

    m_tabla = new double*[m_hrWidth];
    for(int i=0; i<m_hrWidth; ++i)
        m_tabla[i] = new double[m_hrHeight];

    // Reset values
    for(int i=0; i<m_hrWidth; i++)
    {
        for(int j=0; j<m_hrHeight; j++)
        {
            m_tabla[i][j] = 0;
        }
    }

    // Set display scale
    this->setScale(1.0, 1.0, (m_resolution+1)/2.0);
}

/**
 *
 */
void SurfaceWidget::setUpscale(int newRes)
{
    if(newRes < 0 || newRes > 10) return;

    m_resolution = newRes;

    this->setMeshLineWidth(1.0/(2*m_resolution+1));

    this->allocateData();

}

/**
 *
 */
void SurfaceWidget::setFrame(const cv::Mat &frame)
{
    double maxSensor = 0.0;

    // convert to 'double' data type
    frame.convertTo(m_frameDouble, CV_64F);

    // Low pass temporal filter to smooth down movement
    m_frameFiltered += 0.5*(m_frameDouble-m_frameFiltered);

    // Increase image resolution
    cv::resize(m_frameFiltered,
               m_highResFrame,
               cv::Size(),
               m_resolution,
               m_resolution,
               cv::INTER_CUBIC);

    for(int i=0; i<m_highResFrame.rows; i++)
    {
        for(int j=0; j<m_highResFrame.cols; j++)
        {
            double value = (m_highResFrame.at<double>(m_highResFrame.rows-i-1, m_highResFrame.cols-j-1))/SURFACE_PLOTFACTOR;
            m_tabla[i+1][j+1] = value;
            if(value > maxSensor) maxSensor = value;
        }
    }

    // The maximum value across the whole matrix is used to compute the color legend;
    int maxScale = std::max(2, std::min(255, (int)(255.0 * maxSensor / (255.0/SURFACE_PLOTFACTOR))));
    //int maxScale = 255;

    Qwt3D::ColorVector::iterator last = m_cv.begin() + (maxScale-1);
    Qwt3D::ColorVector cv_(m_cv.begin(), last);

    Qwt3D::StandardColor *sc = new Qwt3D::StandardColor(this, maxScale);
    sc->setColorVector( cv_ );
    setDataColor( sc );

    loadFromData(
        m_tabla,        // data array
        m_hrWidth,      // number of rows
        m_hrHeight,     // number of columns
        1.0,            // min row
        m_hrWidth,      // max row
        1.0,            // min col
        m_hrHeight);    // max col

    updateData();
    updateGL();
}
