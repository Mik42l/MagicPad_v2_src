#ifndef FILTER_H
#define FILTER_H

#include <QObject>
#include "QsLog.h"

#include <opencv2/opencv.hpp>

//
// Set/Get built-in type
//
#define FILTER_SET_GET_MACRO(name,type) \
    FILTER_SET_MACRO(name,type) \
    FILTER_GET_MACRO(name,type)

//
// Set built-in type. Creates member Set"name"() (e.g., setVisibility());
//
#define FILTER_SET_MACRO(name,type) \
    public slots:\
    void set##name (type _arg) \
    { \
        this->name = _arg; \
        emit modified(); \
    } \
    public:

//
// Get built-in type.  Creates member Get"name"() (e.g., setVisibility());
//
#define FILTER_GET_MACRO(name,type) \
    virtual type get##name () \
    { \
        return this->name; \
    } \
    signals: \
    void name##Changed (type _arg); \
    public:

/**
 * Filter: Base class for all algorithm
 */
class Filter : public QObject
{
    Q_OBJECT	
public:
    static const QString TAG;

    Filter(QObject *parent = 0);
	
public slots:
    virtual void process() = 0;
    virtual void reset() = 0;

signals:
    void modified();

public slots:

private:

};

#endif // FILTER_H
