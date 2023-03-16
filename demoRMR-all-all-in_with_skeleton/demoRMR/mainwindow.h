#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "qlineedit.h"
#include <QMainWindow>
#include <QTimer>
#ifdef _WIN32
#include<windows.h>
#endif
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
//#include "ckobuki.h"
//#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "robot.h"
#include <QJoysticks.h>

enum customGestures
{
    NONE = 0,
    LEFT_LIKE = 1,
    LEFT_DISLIKE = 2,
    RIGHT_LIKE = 3,
    RIGHT_DISLIKE = 4,
    TELEPHONE_LEFT = 5,
    TELEPHONE_RIGHT = 6
};

enum settingSpeedOption
{
    SET = 0,
    ADD_TO_EXISTED = 1
};

enum speedType
{
    TRANSLATED = 0,
    ANGULAR = 1
};

namespace Ui {
class MainWindow;
}

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera1;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];

    cv::Mat frame[3];
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);

int processThisCamera(cv::Mat cameraData);
int processThisSkeleton(skeleton skeledata);

private slots:
    void on_pushButton_9_clicked();

    void getNewFrame();

    void on_pushButton_8_clicked();

    void on_pushButton_7_clicked();

private:

    //--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     Robot robot;
     TKobukiData robotdata;
     int updateSkeletonPicture;
     skeleton skeleJoints;
     int datacounter;
     QTimer *timer;

     QJoysticks *instance;

     double forwardspeed;//mm/s
     double rotationspeed;//omega/s
     double maxForwardspeed = 400;//mm/s
     double maxRotationspeed = 3.14159;//omega/s

     double f = 681.743;
     float Z_d = -14.5;
     float Z = 21.0;
     float Y_d = 11.5;
     float cameraWidth = 853.0;
     float cameraHeight = 480.0;
     float widthRatio;
     float heightRatio;
     bool frameState = true;
     float graphicConstant = 15.9723;

     double getX(float dist, double angle);
     double getY(float dist, double angle);

     int minVal = 0; // Minimalna hodnota
     int maxVal = 10; // Maximalna hodnota
     int getLidarPointColor(int distance);
     QColor getWidgetWarningColor(double distance);
     int maxForWidgetWarningColor = 100;
     int minForWidgetWarningColor = 20;
     int angleSpaces = 15;
     int skippingPoints = 10;
//     QColor getLidarPointColor(float distance);

     void drawCamera(QPainter* painter, QRect rect);
     void drawLidar(QPainter* painter, QRect rect, QPen pero);
     void drawMinimap(QPainter* painter, QRect rect, QPen pero);
     void drawWarningWidged(QPainter* painter, QRect rect, QPen pero);

     customGestures detectGesture();
     bool connected = false;

     void speedSetting(double speed, settingSpeedOption option, speedType type);
     bool speedCheck(double speed, speedType type);
     void getRatio(int frameWidth, int frameHeight);


};

#endif // MAINWINDOW_H
