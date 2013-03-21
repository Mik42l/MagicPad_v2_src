#ifndef LOGGERDIALOG_H
#define LOGGERDIALOG_H

#include <QApplication>
#include <QDialog>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollBar>
#include <QSyntaxHighlighter>
#include <QTimer>
#include <QVBoxLayout>


// Debug
#include "QsLog.h"

/**
 *
 */
class LoggerDialog : public QDialog
{
    Q_OBJECT    
public:
    LoggerDialog(QWidget *parent = 0);

signals:
    void refresh();

public slots:
    int exec();
    void refreshText();

private:
    QPlainTextEdit  *mText;
    QTimer *mTimer;
};

/**
 *
 */
class Highlighter : public QSyntaxHighlighter
 {
     Q_OBJECT
 public:
     Highlighter(QTextDocument *parent = 0)
         : QSyntaxHighlighter(parent)
      {
          HighlightingRule rule;

          traceFormat.setForeground(Qt::black);
          rule.pattern = QRegExp("\\bTRACE\\b.*\\n");
          rule.format = traceFormat;
          highlightingRules.append(rule);

          debugFormat.setForeground(Qt::black);
          rule.pattern = QRegExp("\\bDEBUG\\b.*\\n");
          rule.format = debugFormat;
          highlightingRules.append(rule);

          infoFormat.setForeground(Qt::darkGreen);
          rule.pattern = QRegExp("\\bINFO\\b.*");
          rule.format = infoFormat;
          highlightingRules.append(rule);

          warnFormat.setForeground(Qt::red);
          rule.pattern = QRegExp("\\bWARN\\b.*");
          rule.format = warnFormat;
          highlightingRules.append(rule);

          errorFormat.setForeground(Qt::red);
          rule.pattern = QRegExp("\\bERROR\\b.*");
          rule.format = errorFormat;
          highlightingRules.append(rule);

          fatalFormat.setForeground(Qt::red);
          rule.pattern = QRegExp("\\bFATAL\\b.*");
          rule.format = fatalFormat;
          highlightingRules.append(rule);
      }


 protected:
     void highlightBlock(const QString &text)
     {
         foreach (const HighlightingRule &rule, highlightingRules) {
             QRegExp expression(rule.pattern);
             int index = expression.indexIn(text);
             while (index >= 0) {
                 int length = expression.matchedLength();
                 setFormat(index, length, rule.format);
                 index = expression.indexIn(text, index + length);
             }
         }
     }

 private:
     struct HighlightingRule
     {
         QRegExp pattern;
         QTextCharFormat format;
     };
     QVector<HighlightingRule> highlightingRules;

     QTextCharFormat traceFormat;
     QTextCharFormat debugFormat;
     QTextCharFormat infoFormat;
     QTextCharFormat warnFormat;
     QTextCharFormat errorFormat;
     QTextCharFormat fatalFormat;
 };


#endif // LOGGERDIALOG_H
