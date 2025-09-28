#ifndef AUTONOMYWINDOW_H
#define AUTONOMYWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QList>
#include <QPushButton>
#include <QGridLayout>
#include <QResizeEvent>

namespace Ui {
class AUTONOMYWindow;
}


class AUTONOMYWindow : public QWidget
{
    Q_OBJECT

public:
    explicit AUTONOMYWindow(QWidget *parent = nullptr);
    ~AUTONOMYWindow();

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    QGridLayout *gridLayout;       
    QList<QPushButton*> buttons_;  
};


#endif // MAINWINDOW_H
