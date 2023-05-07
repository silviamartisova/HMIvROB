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
#include <list>
#include <QAbstractTableModel>
#include <queue>
#include <sstream>
#include <QtMath>
#include <QDateTime>
#include <QMessageBox>

struct MyRectangle {
    float x1, y1, x2, y2, x3, y3, x4, y4;
};

enum Direction {
    LEFT,
    RIGHT,
    UP,
    DOWN
};

struct Node {
    int row, col, val;
    Node(int r, int c, int v) : row(r), col(c), val(v) {}
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

struct Point {
    float x;
    float y;
};

class PointTableModel : public QAbstractTableModel
{
public:
    PointTableModel(QObject *parent = nullptr)
            : QAbstractTableModel(parent)
    {
    }

    int rowCount(const QModelIndex &parent = QModelIndex()) const override
    {
        return m_points.size();
    }

    int columnCount(const QModelIndex &parent = QModelIndex()) const override
    {
        return 2; // Assuming Point has x and y coordinates
    }

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override
    {
        if (!index.isValid())
            return QVariant();

        if (index.row() >= m_points.size())
            return QVariant();

        const Point &point = *std::next(m_points.begin(), index.row());

        if (role == Qt::DisplayRole)
        {
            if (index.column() == 0)
                return QVariant(point.x);
            else if (index.column() == 1)
                return QVariant(point.y);
        }

        return QVariant();
    }

    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override
    {
        if (role == Qt::DisplayRole && orientation == Qt::Horizontal)
        {
            if (section == 0)
                return QVariant("X");
            else if (section == 1)
                return QVariant("Y");
        }

        return QVariant();
    }

    void push_back(Point point){
        m_points.push_back(point);
        newItemInserted();
    }

    void pop_front(){
        m_points.pop_front();
        newItemInserted();
    }

    Point front(){
        return m_points.front();
    }

    bool empty(){
        return m_points.empty();
    }

    std::list<Point>::iterator begin(){
        return m_points.begin();
    }

    std::list<Point>::iterator end(){
        return m_points.end();
    }



private:
    std::list<Point> m_points;

    void newItemInserted(){
        beginInsertRows(QModelIndex(), rowCount(), rowCount());
        endInsertRows();
    }
};

struct RobotState {
    double x; // position in meters
    double y; // position in meters
    double angle; // orientation in radians
    double forwardSpeed; //mm/s
    double angularSpeed; //omega/s
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

    void on_pushButton_clicked();

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

     float circleAngle;

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
     bool startDetected = false;

     void drawCamera(QPainter* painter, QRect rect);
     void drawLidar(QPainter* painter, QRect rect, QPen pero);
     void drawMinimap(QPainter* painter, QRect rect, QPen pero);
     void drawRobotToMinimap(QPainter* painter, QRect rect, QPen pero);
     void drawWarningWidged(QPainter* painter, QRect rect, QPen pero);
     void drawWarningText(QPainter* painter, QRect rect);

     bool connected = false;

     void getRatio(int frameWidth, int frameHeight);

     void loadMap();
     vector<vector<int>> map;  // initialize with zeros
     vector<vector<int>> defaultMap;  // initialize with zeros
     int gridSizeBlockInMM = 100;
     void printMap();
     RobotState state = {0, 0, 0}; // starting at (0, 0)
     void mainLogicOfU2(int x, int y);
     void floodFill(vector<vector<int>>& map, int row, int col);
     Direction checkDirection(int workingRow, int workingCollumn);
     void expandWalls();
     PointTableModel *pointsModel;
     void regulate();
     bool regulating = false;
     bool risingEdgeOfRegulating = false;
     std::chrono::steady_clock::time_point regulation_start_time;
     void toogleRegulation();
     void stopRobot();

     long double tickToMeter = 0.000085292090497737556558; // [m/tick]
     long double wheelbase = 0.23;
     const unsigned short ENCODER_MAX = 65535;  // define maximum encoder value
     float regulatorTranslateProportionalElement = 2000;
     float regulatorAngularProportionalElement = 3.141592*2;
     int rampTranslateConstant = 10; // mm/s
     float rampAngularConstant = 0.1; //omega/s
     int translateSaturationValue = 350;//mm/s;
     float angularSaturationValue = 3.14159/2;//omega/s
     long double oldEncoderLeft;
     long double oldEncoderRight;
     void evaluateAngleRamp(float targetangle);
     void evaluateSaturation();
     void updateRobotState(long double encoderLeft, long double encoderRight);
     void updateRobotPositionInMap();
     void removeOldPositionInMap(int serchingIndex);
     bool eventFilter(QObject *obj, QEvent *event);
     bool detectCircle(QImage& inputImage);

     // for map from drive
     std::vector<MyRectangle> parseFile(const std::string& filename);
     void createMap(const std::vector<MyRectangle>& rects);
     vector<float> getNumbersFromLine(std::istringstream& iss);
     std::list<Point> pointWherRobotWas;
     int findLidarPoint();
     bool circleDetection = false;
     bool safetyStopActivated = false;
     bool done = false;
     QString messageBText;
     bool circleFound = false;
     QString fileName;

public slots:
     void showMessageB();
signals:
     void uiShowMessageB(); ///toto nema telo
};


#endif // MAINWINDOW_H
