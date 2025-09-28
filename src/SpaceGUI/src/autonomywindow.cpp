#include <cstdio>
#include <QtWidgets>
#include <QGridLayout>
#include "autonomywindow.h"
#include <QMainWindow>
#include <QWidget>
#include <QList>
#include <QPushButton>
#include <QGridLayout>
#include <QResizeEvent>

AUTONOMYWindow::AUTONOMYWindow(QWidget *parent) : QWidget(parent) {
    gridLayout = new QGridLayout(this);
    gridLayout->setSpacing(5);
    gridLayout->setContentsMargins(10, 10, 10, 10);

    setLayout(gridLayout);
    // initial size
    resize(800, 600); 
}

void AUTONOMYWindow::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    //Can adjust this
    int minButtonSize = 300; 
    int cols = std::max(1, width() / minButtonSize);
    int rows = std::max(1, height() / minButtonSize);
    int needed = rows * cols;

    // Add buttons if needed
    while (buttons_.size() < needed) {
        int idx = buttons_.size();
        QPushButton *button = new QPushButton(QString("Button %1").arg(idx), this);
        button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        button->setMinimumSize(minButtonSize, minButtonSize);
        buttons_.append(button);
    }

    // Remove buttons if too many
    while (buttons_.size() > needed) {
        QPushButton *btn = buttons_.takeLast();
        gridLayout->removeWidget(btn);
        btn->deleteLater();
    }

    // Clear and re-layout
    // Does not delete the button, just removes from layout
    QLayoutItem *item;
    while ((item = gridLayout->takeAt(0)) != nullptr) {
        
    }

    for (int i = 0; i < buttons_.size(); i++) {
        gridLayout->addWidget(buttons_[i], i / cols, i % cols);
    }
}

AUTONOMYWindow::~AUTONOMYWindow() {}