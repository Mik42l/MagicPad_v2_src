#include "doubleClickFilter.h"

const QString DoubleClickFilter::TAG = QString("DoubleClickFilter");

//
// doubleClickPattern_X are double-click gesture pattern.
// The pattern are the average value of the pixels during a double-clck gestures.
// Multiples patterns are recorded to match slightly different gestures
//
float doubleClickPattern_1[] = {
    227.5280, 225.8890, 219.3890, 184.7780, 144.6110, 138.3330, 172.9440, 222.9170, 186.0280, 143.3060,
    129.6940, 173.5830, 213.0830, 222.6670, 228.2780, 232.9170 };

float doubleClickPattern_2[] = {
    238.8060, 236.7220, 235.2500, 211.3330, 141.3890, 116.4170, 151.2220, 214.0280, 239.0280, 234.4720,
    174.5560,  95.0000, 122.0280, 179.8610, 197.3610, 209.8060, 214.6670, 220.5830, 227.1940, 229.4170,
    234.5000, 237.1940, 240.4720, 242.5830 };

float doubleClickPattern_3[] = {
    250.8060, 250.3610, 245.1390, 214.9440, 132.1670, 100.4170, 158.7220, 219.0830, 198.3060, 106.6940,
    103.1940, 149.4720, 196.1940, 219.5280, 237.6110, 251.5000, 255.0000, 255.0000 };

float doubleClickPattern_4[] = {
    197.7220, 202.1390, 207.1670, 196.7220, 158.0560,  81.2778,  88.3056, 156.6110, 184.3610, 154.7220,
     84.1389,  59.1944,  92.3056, 164.3060, 195.8060, 212.4170, 223.1670, 231.1390, 230.8890 };

float doubleClickPattern_5[] = {
    194.4440, 198.9440, 204.8890, 207.6390, 194.8610, 145.0280,  90.8889,  96.6389, 167.5830, 203.3330,
    181.0280,  83.1944,  80.3333, 127.8060, 154.8330, 167.0560, 180.1390, 188.3060, 196.3330, 212.1940,
    217.4440, 221.1110, 228.2500 };

float doubleClickPattern_6[] = {
    127.6110, 127.0830, 125.8890, 126.4440, 126.0000, 123.3890, 115.5280,  87.0000,  83.8889, 106.8060,
    127.5560, 121.1390,  91.4444,  60.6111,  90.7778, 110.2500, 120.4720, 122.3060, 124.5280, 126.0560,
    125.7500, 127.6670, 129.3330, 129.3330 };

float doubleClickPattern_7[] = {
    148.1110, 149.5000, 153.1390, 157.0560, 170.4720, 177.8610, 185.7500, 193.6110, 197.8890, 196.8890,
    182.4170, 140.8890,  90.2500,  95.1944, 138.1940, 164.3890, 194.8610, 206.6110, 213.6390, 210.8610,
    195.5000, 153.6110, 148.3890, 162.4440, 171.2220, 173.9440 };

float doubleClickPattern_8[] = {
    174.9170, 181.7220, 189.2500, 157.5280, 106.1670, 141.7500, 156.3890, 122.2500, 119.6670, 142.1940,
    159.2500, 166.1110, 170.5000, 173.9440, 177.6940, 179.3890, 179.2500, 179.2220, 178.9170, 178.0560,
    177.3890 };

/**
 *
 */
DoubleClickFilter::DoubleClickFilter(QObject *parent)
    : Filter(parent)
{
    reset();

    loadDoubleClickPatterns();
}

/**
 * Add a pattern to the pattern list.
 * 'pattern' must be a 1-by-N matrice of type CV_32F
 */
void DoubleClickFilter::addPattern( const cv::Mat& pattern )
{
    if( (pattern.cols != 1) || (pattern.type() != CV_32F) ) {
        QLOG_WARN() << TAG << "pattern should be 1-by-N matrice of type CV_32F";
        return;
    }

    // Normalize the pattern: zero-mean and Norm of 1
    float mean = cv::mean( pattern )[0];
    float norm = cv::norm( pattern - mean , cv::NORM_L2 );

    if( norm < 10e-6 ) {
        QLOG_ERROR() << "TAG" << "Pattern has a 0-norm";
    }

    mPatterns.push_back( (pattern - mean) / norm );

    // resize input stream
    // one side effect is that input stream can be erased
    mPatternsMaxLenght = qMax( mPatternsMaxLenght, pattern.rows );
    if( mInputStream.rows != mPatternsMaxLenght ) {
        mInputStream = cv::Mat::zeros( mPatternsMaxLenght, 1, CV_32F );
    }
}

/**
 * Load a predefined set of double-click gestures patterns.
 * Theses patterns are manualy recorded @ 20FPS
 * If the framerate changes the patterns wont be relevant anymore.
 */
void DoubleClickFilter::loadDoubleClickPatterns()
{
    cv::Mat p1( sizeof(doubleClickPattern_1)/sizeof(float), 1, CV_32F, doubleClickPattern_1 );
    cv::Mat p2( sizeof(doubleClickPattern_2)/sizeof(float), 1, CV_32F, doubleClickPattern_2 );
    cv::Mat p3( sizeof(doubleClickPattern_3)/sizeof(float), 1, CV_32F, doubleClickPattern_3 );
    cv::Mat p4( sizeof(doubleClickPattern_4)/sizeof(float), 1, CV_32F, doubleClickPattern_4 );
    cv::Mat p5( sizeof(doubleClickPattern_5)/sizeof(float), 1, CV_32F, doubleClickPattern_5 );
    cv::Mat p6( sizeof(doubleClickPattern_6)/sizeof(float), 1, CV_32F, doubleClickPattern_6 );
    cv::Mat p7( sizeof(doubleClickPattern_7)/sizeof(float), 1, CV_32F, doubleClickPattern_7 );
    cv::Mat p8( sizeof(doubleClickPattern_8)/sizeof(float), 1, CV_32F, doubleClickPattern_8 );

    addPattern( p1 );
    addPattern( p2 );
    addPattern( p3 );
    addPattern( p4 );
    addPattern( p5 );
    addPattern( p6 );
    addPattern( p7 );
    addPattern( p8 );
}

/**
 * Process input data stream
 */
void DoubleClickFilter::process()
{
    // Store latest input (shift the buffer
    for( int k=0; k<mInputStream.rows; k++ ) {
        mInputStream.at< float >(k, 0) = mInputStream.at< float >(k+1, 0);
    }
    mInputStream.at< float >( mInputStream.rows-1, 0) = Input;

    // Find maximum cross correlation with a pattern
    double max = 0.0;
    for( int i=0; i<mPatterns.size(); i++ )
    {
        double x = xcorr( mPatterns.at(i), mInputStream );

        max = qMax( max, x );
    }
    XCorr = max;

    // If the pattern matches the input stream the xcorr is > Threshold,
    // in this case trigger the detected() signal.
    if( max >= Threshold )
    {
        emit detected();
        Detected = true;
    }
    else
    {
        Detected = false;
    }
}

/**
 * Reset filter to its original state
 */
void DoubleClickFilter::reset()
{
    XCorr = 0.0;
    Detected = 0;
    mInputStream = cv::Mat();
    mPatterns.clear();
    mPatternsMaxLenght = 0;
    Threshold = DOUBLECLICK_DEFAULT_THRESHOLD;
}

/**
 *
 */
double DoubleClickFilter::xcorr( const cv::Mat& pattern, const cv::Mat& stream )
{
    // Extract the end-portion of the input stream to match the size of the pattern
    int offset = stream.rows - pattern.rows;
    cv::Mat input( stream.rowRange( cv::Range( offset, stream.rows ) ) );

    // Compute normalizer xcorr between pattern and input
    // The 'pattern' is already centered (0-mean) and has a norm of 1.0
    // which simplify computation.
    //
    // In our case, the normalized xcorr formula is:
    //
    // I = Input, P = pattern
    //
    //      Sum[ (I-mean(I) * P ]
    // r = -----------------------
    //         norm(I-mean(I))

    cv::Mat Im = input - cv::mean( input )[0];

    double r = Im.dot( pattern ) / cv::norm( Im );

    return r;
}
