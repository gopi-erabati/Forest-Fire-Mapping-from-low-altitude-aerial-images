#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "datareader.h"
#include "segment.h"
#include "projection.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private slots:
    void on_uploadImagePush_clicked();

    void on_segmentImgPush_clicked();

    void on_updateFireMapPush_clicked();

    void on_uploadImagePush_2_clicked();

private:
    Ui::MainWindow *ui;

    //objects
    DataReader datareader;
    Segment segment;

    QStringList fileNames, fileNamesSeg;

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

};

#endif // MAINWINDOW_H
