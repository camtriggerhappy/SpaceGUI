#pragma once

#include <QWidget>
#include <QString>
#include <QPoint>

class QLabel;

class RosDataWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RosDataWidget(const QString &dataText = QString(), QWidget *parent = nullptr);

    QString dataText() const { return m_dataText; }
    void setDataText(const QString &text);     // updates the visible label
    void setFixedWidgetSize(const QSize &s);   // optional helper to change size

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

private:
    QString m_dataText;
    QPoint m_dragStartPos;
    QLabel *m_label = nullptr;
};
