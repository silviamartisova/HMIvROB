#include "mainwindow.h"
#include "qevent.h"
#include "qlabel.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <opencv2/opencv.hpp>


///Tomas Lizak, Siliva Martisova


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";//192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;

    pointsModel = new PointTableModel();



    datacounter=0;

    ui->frame->setMouseTracking(true);
    ui->frame->installEventFilter(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);    

    auto battery = robotdata.Battery;
    double batteryPercentage = static_cast<double>(battery) / 255.0 * 100.0;
    QString batteryString = QString::number(batteryPercentage, 'f', 1) + "%";
    ui->lblStatus->setText("Battery: " + batteryString);

    /// prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect mainRect, secondaryRect, thirdRect;
    mainRect = ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit

//    //set ratio of main frame
//    int width = mainRect.size().width(); // get current width
//    int height = mainRect.size().height(); // get current height
//    if (!(width * 16 == height * 9))
//     // check if aspect ratio is 4:3
//    {
////        int newHeight = width * 16 / 9; // calculate new height for 4:3 ratio
//        int newWidth = height * 16 / 9; // calculate new height for 4:3 ratio

//        mainRect.setWidth(newWidth);
////        mainRect.setHeight(newHeight); // set new size
//    }

////////this was making the ratio of window
//    //set ratio of main frame
//    int width = size().width();
//    int height = size().height();
//    if (!(width * 16 == height * 9))
//    {
//        int newWidth = height * 16 / 9;
//        resize(newWidth, height);
//    }
////////////////////////////////////////////////


    secondaryRect = ui->frame_2->geometry();
    thirdRect = ui->frame_3->geometry();
    mainRect.translate(0,15);


    secondaryRect.translate(mainRect.topLeft());
    secondaryRect.setHeight(mainRect.height()/3);
    secondaryRect.setWidth(mainRect.width()/3);

    thirdRect.setHeight(mainRect.height()/3);
    thirdRect.setWidth(mainRect.width()/3);
    thirdRect.translate(mainRect.bottomLeft().x(), mainRect.bottomLeft().y() - thirdRect.height());

    painter.drawRect(mainRect);

    if(frameState){
        drawCamera(&painter, frameState ? mainRect : secondaryRect);
        drawLidar(&painter, frameState ? mainRect : secondaryRect, pero);
    //    QColor fillColor(255, 255, 255, 128);
    //    QBrush brush(fillColor);
    //    painter.fillRect(secondaryRect, brush);

        if(connected){
            updateRobotPositionInMap();
            drawMinimap(&painter, frameState ? secondaryRect : mainRect, pero);
        }
        drawWarningWidged(&painter, thirdRect, pero);
        drawWarningText(&painter, frameState ? mainRect : secondaryRect);
    }else{
    //    QColor fillColor(255, 255, 255, 128);
    //    QBrush brush(fillColor);
    //    painter.fillRect(secondaryRect, brush);
        if(connected){
            updateRobotPositionInMap();
            drawMinimap(&painter, frameState ? secondaryRect : mainRect, pero);
        }
//        drawCamera(&painter, frameState ? mainRect : secondaryRect);
//        drawLidar(&painter, frameState ? mainRect : secondaryRect, pero);
    }


}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{

//    cout << battery << endl;
//    ui->lblStatus->setText();
    updateRobotState(robotdata.EncoderLeft, robotdata.EncoderRight);
    if(regulating)regulate();
    if(circleFound){
        messageBText = "Treasuere found - evidence will be saved in " + fileName;
        emit uiShowMessageB();
        circleFound = false;
    }
    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky... kazdopadne, aktualne to blokuje gombiky cize tak
//    if(forwardspeed==0 && rotationspeed!=0)
//        robot.setRotationSpeed(rotationspeed);
//    else if(forwardspeed!=0 && rotationspeed==0)
//        robot.setTranslationSpeed(forwardspeed);
//    else if((forwardspeed!=0 && rotationspeed!=0))
//        robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
//    else
//        robot.setTranslationSpeed(0);

///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    if(datacounter%5)
    {

        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
                // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
                //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
                //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
                /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
                /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
                /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        ///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }

    datacounter++;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{


    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej struktury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka
    updateLaserPicture=1;

    QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
//        cout << frame->rows << endl;
    if(!frameState && circleDetection && detectCircle(image)){
        cout << "circle detected" << endl;

        int bod = findLidarPoint();

        float x = state.x*1000 + copyOfLaserData.Data[bod].scanDistance*cos(state.angle+(360-copyOfLaserData.Data[bod].scanAngle)*3.14159/180.0);
        float y = state.y*1000 + copyOfLaserData.Data[bod].scanDistance*sin(state.angle+(360-copyOfLaserData.Data[bod].scanAngle)*3.14159/180.0);

        x /= gridSizeBlockInMM;
        y /= gridSizeBlockInMM;


        if ((y < map.size() && x < map[0].size()) && x >= 0 && y >= 0) {
            defaultMap[y][x] = 9;
        }

    }

    return 0;
}

///toto je calback na data zo skeleton trackera, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z trackera
int MainWindow::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));
    return 0;
}

void MainWindow::on_pushButton_9_clicked() //start button
{
    if(connected) return;
    forwardspeed=0;
    rotationspeed=0;


    state.forwardSpeed=0;
    state.angularSpeed=0;
    state.angle = 0;
    state.x = 0.5;
    state.y = 0.5;

    loadMap();
    printMap();
    expandWalls();
    printMap();
//    mainLogicOfU2();
    printMap();

//    createMap(parseFile("priestor.txt"));
//    printMap();


    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiShowMessageB()),this,SLOT(showMessageB()));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    //---simulator ma port 8889, realny robot 8000
    robot.setCameraParameters("http://"+ipaddress+":8000/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();



    //ziskanie joystickov
    instance = QJoysticks::getInstance();


    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );

    oldEncoderLeft = robotdata.EncoderLeft;
    oldEncoderRight = robotdata.EncoderRight;
    connected = true;
    ui->pushButton_9->hide();

    state.forwardSpeed=0;
    state.angularSpeed=0;
    state.angle = 0;
    state.x = 0.5;
    state.y = 0.5;
    regulating = false;

}

void  MainWindow::showMessageB()
{
    QMessageBox::information(nullptr, "Information", messageBText);
}


void MainWindow::getNewFrame()
{

}

double MainWindow::getX(float dist, double angle){
    return dist * cos((360-angle)*(3.14159265358979/180));
}

double MainWindow::getY(float dist, double angle){
    return dist * sin((360-angle)*(3.14159265358979/180));
}

int MainWindow::getLidarPointColor(int distance){

    // Kontrola prekročenia maximálnej hodnoty
    if (distance > maxVal) {
        maxVal = distance;
    }

    // Vypočítanie množstva červenej a zelenej zložky farby na základe hodnoty
    int blue = (255 * (distance - minVal)) / (maxVal - minVal);
    int red = (255 * (maxVal - distance)) / (maxVal - minVal);

    // Vytvorenie farby v hexadecimálnom tvare
    int color = (red << 16) | (blue);

    // Vypísanie farby
    // cout << "Farba: #" << hex << color << endl;
    return color;
}

QColor  MainWindow::getWidgetWarningColor(double distance){
    // Calculate the color based on the input value
        int red = static_cast<int>(255.0 * (1.0 - static_cast<double>(distance - minForWidgetWarningColor) / (maxForWidgetWarningColor - minForWidgetWarningColor)));
        int blue = static_cast<int>(255.0 * static_cast<double>(distance - minForWidgetWarningColor) / (maxForWidgetWarningColor - minForWidgetWarningColor));
        return QColor(red, 0, blue);

}


void MainWindow::drawCamera(QPainter* painter, QRect rect){
    if(actIndex>-1)/// ak zobrazujem data z kamery ak aspon niektory frame vo vectore je naplneny
    {
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
    //        cout << frame->rows << endl;
        if(frameState && circleDetection && detectCircle(image)){
            cout << "circle detected" << endl;

            int bod = findLidarPoint();

            float x = state.x*1000 + copyOfLaserData.Data[bod].scanDistance*cos(state.angle+(360-copyOfLaserData.Data[bod].scanAngle)*3.14159/180.0);
            float y = state.y*1000 + copyOfLaserData.Data[bod].scanDistance*sin(state.angle+(360-copyOfLaserData.Data[bod].scanAngle)*3.14159/180.0);

            x /= gridSizeBlockInMM;
            y /= gridSizeBlockInMM;


            if ((y < map.size() && x < map[0].size()) && x >= 0 && y >= 0) {
                defaultMap[y][x] = 9;
            }

        }
        painter->drawImage(rect,image.rgbSwapped());
    }
}

void MainWindow::drawWarningText(QPainter* painter, QRect rect){

    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
//        cout << copyOfLaserData.Data[k].scanDistance << endl;
        if(copyOfLaserData.Data[k].scanDistance < 300 && copyOfLaserData.Data[k].scanDistance > 130){

            QFont font("Benshrift", rect.width()/10);
            painter->setFont(font);
             // vytvorenie QPainter objektu a nastavenie farby a priehľadnosti
            QColor color(255, 0, 0, 100); // nastavenie farby na červenú s priehľadnosťou 50%
            painter->setPen(color);
            painter->drawText(rect, Qt::AlignCenter, "Warning!");
            return;
        }

    }
}

void MainWindow::drawLidar(QPainter* painter, QRect rect, QPen pero){

    getRatio(rect.width(), rect.height());

    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {

        if(copyOfLaserData.Data[k].scanAngle > 30 && copyOfLaserData.Data[k].scanAngle < 330) continue;
        float dist=copyOfLaserData.Data[k].scanDistance/10.0;
        float xp=(cameraWidth/2.0) - (f * getY(dist, copyOfLaserData.Data[k].scanAngle)) / (getX(dist, copyOfLaserData.Data[k].scanAngle) + Z_d);
        float yp=(cameraHeight/2.0) + (f * (-Z + Y_d))/(getX(dist, copyOfLaserData.Data[k].scanAngle) + Z_d);

        yp *= heightRatio;
        xp *= widthRatio;

        yp += rect.topLeft().y();
        xp += rect.topLeft().x();




        if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
        {
            pero.setColor(getLidarPointColor(dist));
            painter->setPen(pero);

            painter->drawEllipse(QPoint(xp, yp),2,2);
        }
    }


}

void MainWindow::drawMinimap(QPainter* painter, QRect rect, QPen pero){

    float yp = rect.topLeft().y();
    float xp = rect.topLeft().x();
    int mapWidth = map[0].size();
    int mapHeight = map.size();
    int cellWidth = rect.width() / mapWidth;
    int cellHeight = rect.height() / mapHeight;
    int cellSize = qMin(cellWidth, cellHeight);
    QRect robotRect;
    for (int i = 0; i < mapHeight; i++) {
        for (int j = 0; j < mapWidth; j++) {
            QRect rect(j * cellSize + xp, i * cellSize + yp, cellSize, cellSize);
            switch (defaultMap[i][j]) {
                case 1:
                    painter->fillRect(rect, Qt::black);
                    break;
                case -1:
                    painter->fillRect(rect, Qt::white);
                    robotRect = rect;
                    break;
                case 2:
                    painter->fillRect(rect, Qt::green);
                    break;
                case 9:
                    painter->fillRect(rect, Qt::white);
                    pero.setColor(Qt::magenta);
                    painter->setPen(pero);
                    painter->drawLine(rect.topLeft(), rect.bottomRight());
                    painter->drawLine(rect.topRight(), rect.bottomLeft());
//                    painter->fillRect(rect, Qt::magenta);
                    break;
                default:
                    painter->fillRect(rect, Qt::white);
                    break;
            }
        }
    }
    if(!pointWherRobotWas.empty()){
        for (std::list<Point>::iterator it = pointWherRobotWas.begin(); it != pointWherRobotWas.end(); ++it) {
            // Do something with the current object pointed to by the iterator
            Point& obj = *it;
            int x = obj.x*1000/gridSizeBlockInMM;
            int y = obj.y*1000/gridSizeBlockInMM;
            QRect rect(x * cellSize + xp, y * cellSize + yp, cellSize, cellSize);
            painter->fillRect(rect, Qt::blue);
        }
    }

    if(!pointsModel->empty()){
        for (std::list<Point>::iterator it = pointsModel->begin(); it != pointsModel->end(); ++it) {
            // Do something with the current object pointed to by the iterator
            Point& obj = *it;
            int x = obj.x*1000/gridSizeBlockInMM;
            int y = obj.y*1000/gridSizeBlockInMM;
            QRect rect(x * cellSize + xp, y * cellSize + yp, cellSize, cellSize);
            painter->fillRect(rect, Qt::green);
        }
    }

    drawRobotToMinimap(painter, robotRect, pero);
}

void MainWindow::drawRobotToMinimap(QPainter* painter, QRect rect, QPen pero){
    pero.setBrush(Qt::red);
    pero.setColor(Qt::red);
    painter->setPen(pero);
    painter->setBrush(Qt::red);
    QRect centeredRect(rect.x() - rect.width() / 2, rect.y() - rect.height() / 2, rect.width() * 2, rect.height() * 2);
    painter->drawEllipse(centeredRect);

    QPoint center = centeredRect.center();

    // Calculate the end point of the line using the angle and the radius of the circle
    qreal radius = centeredRect.width() / 2.0;
    qreal x = center.x() + radius * qCos(state.angle);
    qreal y = center.y() + radius * qSin(state.angle);
    QPoint endPoint(qRound(x), qRound(y));

    pero.setColor(QColor(0, 0, 0));
    qreal lineWidth = centeredRect.width() * 0.1;
    pero.setWidth(lineWidth);
    painter->setPen(pero);
    painter->drawLine(center, endPoint);
}

void MainWindow::drawWarningWidged(QPainter* painter, QRect rect, QPen pero){

    int screenMiddleX = rect.width()/2;
    int screenMiddleY = rect.height()/2;
    int radius = rect.width() > rect.height() ? rect.height() : rect.width();


   for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k+=skippingPoints){
       if(copyOfLaserData.Data[k].scanDistance/10 < maxForWidgetWarningColor){
           auto color = getWidgetWarningColor(copyOfLaserData.Data[k].scanDistance/10);
           painter->setPen(color);
           painter->setBrush(color);
           painter->drawPie(rect.topLeft().x(), rect.topLeft().y(), radius, radius, graphicConstant*(360-copyOfLaserData.Data[k].scanAngle-angleSpaces/2+90), graphicConstant*angleSpaces);

       }
    }

    pero.setWidth(2);
    pero.setBrush(Qt::white);
    pero.setColor(Qt::black);
    painter->setPen(pero);
    painter->setBrush(Qt::white);
    painter->drawEllipse(rect.topLeft().x()+radius/4, rect.topLeft().y()+radius/4, radius/2, radius/2);
    painter->drawLine(rect.topLeft().x() + radius/2,rect.topLeft().y() + screenMiddleY - radius/4, rect.topLeft().x() + radius/2, rect.topLeft().y() + screenMiddleY);

}


void MainWindow::getRatio(int frameWidth, int frameHeight){
    heightRatio =  frameHeight/cameraHeight;
    widthRatio =  frameWidth/cameraWidth;
}






void MainWindow::on_pushButton_8_clicked()
{
    frameState = !frameState;
}


void MainWindow::on_pushButton_7_clicked()
{
// safety stop
    if(safetyStopActivated){
         ui->pushButton_7->setStyleSheet("");
    }else{
        ui->pushButton_7->setStyleSheet("background-color: red;");
        robot.setTranslationSpeed(0);
        state.forwardSpeed = 0;
        state.angularSpeed = 0;
    }
    safetyStopActivated = !safetyStopActivated;
}

void MainWindow::loadMap(){

    ifstream file("mapa.txt");

    if (file.is_open()) {
        string line;
        while (getline(file, line)) {
            vector<int> row;
            for (char c : line) {
                if (c == '0') {
                    row.push_back(0);
                } else if (c == '1') {
                    row.push_back(1);
                }
            }
            map.push_back(row);
        }
        file.close();
    } else {
        cout << "Unable to open file";
        return;
    }
    defaultMap = map;
}

void MainWindow::printMap(){
    cout << endl << "MAP:" << endl;
    for (int i = 0; i < map.size(); i++) {
        for (int j = 0; j < map[i].size(); j++) {
            cout << map[i][j] << " ";
        }
        cout << endl;
    }
}

void MainWindow::mainLogicOfU2(int x, int y){

    // Set -1 to tile where robot is located
    int row = state.y*1000 / gridSizeBlockInMM;
    int col = state.x*1000 / gridSizeBlockInMM;
    map[row][col] = -1;

//    // Set a random destination
//        srand(time(NULL));
//        int rand_row = rand() % map.size();
//        int rand_col = rand() % map[0].size();

//        while (map[rand_row][rand_col] != 0) {
//            rand_row = rand() % map.size();
//            rand_col = rand() % map[0].size();
//        }

//        map[rand_row][rand_col] = 2;
//        defaultMap[rand_row][rand_col] = 2;

//    printMap();
//    // Run flood fill from the destination to the robot
//    floodFill(map, rand_row, rand_col);

        map[y][x] = 2;
        defaultMap[y][x] = 2;

        printMap();
        // Run flood fill from the destination to the robot
        floodFill(map, y, x);

    printMap();


    Direction direction;
    int oldValue = 999;
    while(true){

        direction = checkDirection(row, col);

        while(true){
            switch (direction){
            case LEFT:
                col--;
                break;
            case RIGHT:
                col++;
                break;
            case UP:
                row--;
                break;
            case DOWN:
                row++;
                break;
            }
            if(map[row][col] < oldValue && map[row][col] != 1 && map[row][col] != 0){
                oldValue = map[row][col];
                cout << oldValue << endl;
            }else break;
        }

        switch (direction){
        case LEFT:
            col++;
            break;
        case RIGHT:
            col--;
            break;
        case UP:
            row++;
            break;
        case DOWN:
            row--;
            break;
        }

        pointsModel->push_back(Point{col*gridSizeBlockInMM/1000.0f, row*gridSizeBlockInMM/1000.0f});


        if(oldValue == 2){
            break;
        }
    }
    toogleRegulation();
}

void MainWindow::floodFill(vector<vector<int>>& map, int row, int col) {

    int val = 2;
    queue<Node> q;
    q.push(Node(row, col, val));

    while (!q.empty()) {
        Node node = q.front();
        q.pop();

        row = node.row;
        col = node.col;
        val = node.val;

        if (row < 0 || row >= map.size() || col < 0 || col >= map[0].size()) {
            continue;
        }

        if (map[row][col] == -1) {
            break;
        }

        if (map[row][col] != 0 && map[row][col] != 2) {
            continue;
        }
        if (map[row][col] != 2) {
            map[row][col] = val;
        }
//        cout << "inserting to: " << row  << " " << col << " with value: " << val << endl;
        q.push(Node(row-1, col, val+1));
        q.push(Node(row+1, col, val+1));
        q.push(Node(row, col-1, val+1));
        q.push(Node(row, col+1, val+1));
    }
}

Direction MainWindow::checkDirection(int workingRow, int workingCollumn){

    bool left = true;
    bool right = true;
    bool up = true;
    bool down = true;

    int leftValue = map[workingRow][workingCollumn-1];
    int rightValue = map[workingRow][workingCollumn+1];
    int upValue = map[workingRow-1][workingCollumn];
    int downValue = map[workingRow+1][workingCollumn];

    if(downValue == 1 || downValue == 0 || downValue == -1)down = false;
    if(upValue == 1 || upValue == 0 || upValue == -1)up = false;
    if(rightValue == 1 || rightValue == 0 || rightValue == -1)right = false;
    if(leftValue == 1 || leftValue == 0 || leftValue == -1)left = false;

    int smallest;
    if(left)smallest = leftValue;
    else if(right)smallest = rightValue;
    else if(up)smallest = upValue;
    else smallest = downValue;

    if (right && rightValue < smallest) {
        smallest = rightValue;
    }
    if (up && upValue < smallest) {
        smallest = upValue;
    }
    if (down && downValue < smallest) {
        smallest = downValue;
    }

    if (smallest == leftValue) {
        return LEFT;
    } else if (smallest == rightValue) {
        return RIGHT;
    } else if (smallest == upValue) {
        return UP;
    } else {
        return DOWN;
    }
}

void MainWindow::expandWalls(){
    // Define the expansion factor
    int expansionFactor = 3;

    // Create a new 2D vector to store the expanded map
    vector<vector<int>> expandedMap(map.size(), vector<int>(map[0].size(), 0));

    // Iterate over all cells in the map
    for (int i = 0; i < map.size(); i++) {
        for (int j = 0; j < map[0].size(); j++) {
            // Check if the cell represents a wall
            if (map[i][j] == 1) {
                // Replace the wall cell with a larger block of wall cells
                for (int k = i-expansionFactor; k <= i+expansionFactor; k++) {
                    for (int l = j-expansionFactor; l <= j+expansionFactor; l++) {
                        // Check if the cell is within the bounds of the map
                        if (k >= 0 && k < map.size() && l >= 0 && l < map[0].size()) {
                            // Replace the cell with a wall
                            expandedMap[k][l] = 1;
                        }
                    }
                }
            } else {
                // Copy the cell from the original map
                expandedMap[i][j] = map[i][j];
            }
        }
    }

    // Use the expanded map for further processing
    map = expandedMap;

}

void MainWindow::regulate(){
    if(safetyStopActivated)return;
    cout << "forwarSpeed: " << state.forwardSpeed << " angular speed: " << state.angularSpeed << endl;

    if(pointsModel->rowCount() == 0){
        // There are no more points to go
        toogleRegulation();
        cout << "No points to go!" << endl;
        if(!done){
            messageBText = "Final point reached, but treasue was not found, feel free to set new final point";
            emit uiShowMessageB();
        }

        return;
    }
    Point destinationPoint = pointsModel->front();

    // Destination reached
    if((abs(state.x - destinationPoint.x) < 0.01) && (abs(state.y - destinationPoint.y) < 0.01)){
        pointsModel->pop_front();
        cout << "point x: " << destinationPoint.x << " y: " << destinationPoint.y << " reached!!" << endl;
        pointWherRobotWas.push_front(destinationPoint);
        return;
    }

    // Calculate the distance and angle between the robot and the destination
    float dx = destinationPoint.x - state.x;
    float dy = destinationPoint.y - state.y;
    float distance = std::sqrt(dx*dx + dy*dy);
    float angle = std::atan2(dy, dx) - state.angle;
//    cout << "angle before:  " << angle << endl;
    // difference between -π and π.
    angle = fmod(angle + 3.14159265358979, 3.14159265358979*2) - 3.14159265358979;
//    cout << "angle after:  " << angle << endl;

    // Check if angle isnt too big
    if(abs(angle) > 0.785398){ // 45 degrees
        state.forwardSpeed = 0;
        evaluateAngleRamp(angle);
        evaluateSaturation();
        robot.setRotationSpeed(state.angularSpeed);
        cout << "angle is too big!  " << angle << endl;
        return;
    }

    // Create ramp effect, if needed
    double acceleration = distance * regulatorTranslateProportionalElement - state.forwardSpeed;
    if(acceleration > rampTranslateConstant){
        state.forwardSpeed += rampTranslateConstant;
    }else{
        state.forwardSpeed = distance * regulatorTranslateProportionalElement;
    }
    evaluateAngleRamp(angle);
    evaluateSaturation();

    if(state.angularSpeed == 0){
        robot.setTranslationSpeed(state.forwardSpeed);
    }else
    robot.setArcSpeed(state.forwardSpeed, state.forwardSpeed/state.angularSpeed);

}

void MainWindow::evaluateSaturation(){
    // Saturation
    if(state.forwardSpeed > translateSaturationValue){
        state.forwardSpeed = translateSaturationValue;
    }
    else if(state.forwardSpeed < -translateSaturationValue){
        state.forwardSpeed = -translateSaturationValue;
    }

    if(state.angularSpeed > angularSaturationValue){
        state.angularSpeed = angularSaturationValue;
    }
    else if(state.angularSpeed < -angularSaturationValue){
        state.angularSpeed = -angularSaturationValue;
    }
}

void MainWindow::evaluateAngleRamp(float targetangle){

    float diff = targetangle * regulatorAngularProportionalElement - state.angularSpeed;
    int dir = (diff > 0) ? 1 : -1;

    float increment = rampAngularConstant < std::fabs(diff) ? rampAngularConstant : std::fabs(diff);
    state.angularSpeed += dir * increment;
}

void MainWindow::toogleRegulation(){
    if(!regulating){
        regulating = true;
        risingEdgeOfRegulating = true;

        regulation_start_time = std::chrono::steady_clock::now();
        cout << "Starting regulation" << endl;
    }else{
        regulating = false;
        stopRobot();

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - regulation_start_time);
        cout << "StopingRegulation" << endl;
        std::cout << "Time taken: " << duration.count() << " milliseconds" << std::endl;

        //clearing
        removeOldPositionInMap(2);
        map = defaultMap;
        expandWalls();
    }
}

void MainWindow::stopRobot(){
    robot.setTranslationSpeed(0);
    state.angularSpeed = 0;
    state.forwardSpeed = 0;
}

void MainWindow::updateRobotState(long double encoderLeft, long double encoderRight) {

    // calculate the difference in encoder counts for each wheel
    long double deltaEncoderLeft = encoderLeft - oldEncoderLeft;
    long double deltaEncoderRight = encoderRight - oldEncoderRight;

    // handle encoder overflow by adding or subtracting the maximum encoder value
    if (deltaEncoderLeft > ENCODER_MAX / 2) {
        deltaEncoderLeft -= ENCODER_MAX;
        cout << "!!!!!!!!!!!!!!!!!!!!!" << endl;
    } else if (deltaEncoderLeft < -ENCODER_MAX / 2) {
        deltaEncoderLeft += ENCODER_MAX;
        cout << "!!!!!!!!!!!!!!!!!!!!!" << endl;
    }
    if (deltaEncoderRight > ENCODER_MAX / 2) {
        deltaEncoderRight -= ENCODER_MAX;
        cout << "!!!!!!!!!!!!!!!!!!!!!" << endl;
    } else if (deltaEncoderRight < -ENCODER_MAX / 2) {
        deltaEncoderRight += ENCODER_MAX;
        cout << "!!!!!!!!!!!!!!!!!!!!!" << endl;
    }

    // calculate the distance traveled by each wheel
    long double distanceLeft = deltaEncoderLeft * tickToMeter;
    long double distanceRight = deltaEncoderRight * tickToMeter;
//    cout << "DistanceLeft: " << distanceLeft;
//    cout << " DistanceRight: " << distanceRight;

    // calculate the average distance traveled by the robot
    long double distance = (distanceLeft + distanceRight) / 2;

//    // calculate the change in angle of the robot
    long double deltaAngle = (distanceRight - distanceLeft) / wheelbase;

    if(distanceRight == distanceLeft){

    //    cout << " DeltaAngle: " << deltaAngle << endl;

        // update the robot's position and orientation
        state.x += distance * cos(state.angle + deltaAngle / 2);
        state.y += distance * sin(state.angle + deltaAngle / 2);
        state.angle += deltaAngle;

    }
    else{

        state.x += ((wheelbase*(distanceRight + distanceLeft))/(2*(distanceRight - distanceLeft)))*(sin(state.angle+deltaAngle)-sin(state.angle));
        state.y -= ((wheelbase*(distanceRight + distanceLeft))/(2*(distanceRight - distanceLeft)))*(cos(state.angle+deltaAngle)-cos(state.angle));
        state.angle += deltaAngle;


    }

    oldEncoderLeft = encoderLeft;
    oldEncoderRight = encoderRight;

//    cout << "x: " << state.x << "; y: " << state.y << "; angle: " << state.angle << endl;
}

void MainWindow::updateRobotPositionInMap(){
    // lets first find old robot position and remove it
    removeOldPositionInMap(-1);
    // now lets draw him on new position by his x and y cords
    int row = state.y*1000 / gridSizeBlockInMM;
    int col = state.x*1000 / gridSizeBlockInMM;
    map[row][col] = -1;
    defaultMap[row][col] = -1;
}

void MainWindow::removeOldPositionInMap(int serchingIndex){
    // loop throught map unfortunately
    for(int i = 0; i < map.size(); i++){
        for(int j = 0; j < map[0].size(); j++){
            if(map[i][j] == serchingIndex){
                map[i][j] = 0;
                defaultMap[i][j] = 0;
                return;
            }
        }
    }
}

std::vector<MyRectangle> MainWindow::parseFile(const std::string& filename) {
    std::vector<MyRectangle> rects;

    std::ifstream infile(filename);
    std::string line;
    while (std::getline(infile, line)) {
        if(line.empty())return rects;
        std::istringstream iss(line);
        int n;
        iss >> n;

        std::vector<float> numbers = getNumbersFromLine(iss);
        if (n == 4) {
            MyRectangle rect;
            rect.x1 = numbers[0];
            rect.y1 = numbers[1];
            rect.x2 = numbers[2];
            rect.y2 = numbers[3];
            rect.x3 = numbers[4];
            rect.y3 = numbers[5];
            rect.x4 = numbers[6];
            rect.y4 = numbers[7];
            rects.push_back(rect);
        }
    }

    return rects;
}

void MainWindow::createMap(const std::vector<MyRectangle>& rects){

    map.resize(10);
    for (auto& row : map) {
        row.resize(10, 0);  // initialize the new columns with zeros
    }

    for (const auto& rect : rects) {
        for (int y = rect.y1/gridSizeBlockInMM; y <= rect.y2/gridSizeBlockInMM; ++y) {
            for (int x = rect.x1/gridSizeBlockInMM; x <= rect.x2/gridSizeBlockInMM; ++x) {

                if (y >= map.size() || x >= map[0].size()) {
                    int rows = fmax(y + 1, (int)map.size());
                    int cols = fmax(x + 1, (int)map[0].size());
                    map.resize(rows);
                    for (auto& row : map) {
                        row.resize(cols, 0);  // initialize the new columns with zeros
                    }
                }

                map[y][x] = 1;
            }
        }
    }

    defaultMap = map;
}

vector<float> MainWindow::getNumbersFromLine(std::istringstream& iss){
    std::vector<float> numbers;
    std::string temp;

    char c;
    while (iss.get(c)) {
        if (isdigit(c) || c == '.') {
            temp += c;
        } else if (!temp.empty()) {
            double number = std::stod(temp);
            numbers.push_back(number);
            temp.clear();
        }
    }

    if (!temp.empty()) {
        double number = std::stod(temp);
        numbers.push_back(number);
    }
    return numbers;
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == ui->frame && event->type() == QEvent::MouseButtonPress && !frameState) {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        if (mouseEvent->button() == Qt::LeftButton && !regulating) {
            int x = mouseEvent->pos().x();
            int y = mouseEvent->pos().y();

            int mapWidth = map[0].size();
            int mapHeight = map.size();
            int cellWidth = ui->frame->width() / mapWidth;
            int cellHeight = ui->frame->height() / mapHeight;
            int cellSize = qMin(cellWidth, cellHeight);

            x /= cellSize;
            y /= cellSize;

            cout << "Left mouse button clicked at " << x << " " << y << endl;
            if (((y < map.size() && x < map[0].size()) && x >= 0 && y >= 0) && map[y][x] != 1) {
                defaultMap[y][x] = 2;
                mainLogicOfU2(x, y);
                return false;
            }

        }
        return true; // event is handled, stop processing
    }
    return QObject::eventFilter(obj, event);
}

bool MainWindow::detectCircle(QImage& inputImage){
    // Convert QImage to cv::Mat
        cv::Mat inputMat(inputImage.height(), inputImage.width(), CV_8UC3, (cv::Scalar*)inputImage.scanLine(0));

        // Convert to grayscale
        cv::Mat grayMat;
        if (inputMat.channels() == 1) {
            // Input image is already grayscale, no need to convert
            grayMat = inputMat.clone();
        } else if (inputMat.channels() == 3) {
            // Convert 3-channel BGR image to grayscale using cv::cvtColor
            cv::cvtColor(inputMat, grayMat, cv::COLOR_BGR2GRAY);
        } else if (inputMat.channels() == 4) {
            // Convert 4-channel BGRA image to grayscale using cv::cvtColor
            cv::cvtColor(inputMat, grayMat, cv::COLOR_BGRA2GRAY);
        } else {
            // Unsupported input image format
            // Handle error here
        }

        // Blur the image to reduce noise
        cv::GaussianBlur(grayMat, grayMat, cv::Size(5, 5), 0);

        // Apply Hough transform to detect circles
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(grayMat, circles, cv::HOUGH_GRADIENT, 1, grayMat.rows/8, 100.0, 80.0);

//        // Draw circles on output image
//        outputImage = QImage(inputImage.width(), inputImage.height(), QImage::Format_ARGB32);
//        outputImage.fill(Qt::transparent);
        QPainter painter(&inputImage);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setPen(Qt::red);
        for (const auto& circle : circles)
        {
            painter.drawEllipse(circle[0] - circle[2], circle[1] - circle[2], circle[2] * 2, circle[2] * 2);
        }

        if(!circles.empty() && !done){
            regulating = false;
            circleDetection = false;
            robot.setTranslationSpeed(0);
            state.angularSpeed = 0;
            state.forwardSpeed = 0;
            auto x = circles[0][0];
            double distanceFromCenter = x - inputMat.cols / 2.0;
            circleAngle = distanceFromCenter / (cameraWidth / 2.0) * 32.0;
            cout << "angle: " << circleAngle << endl;

            QDateTime currentDateTime = QDateTime::currentDateTime();

            // Format the date and time as a string
            fileName = currentDateTime.toString("yyyyMMdd_hhmmss");
            fileName += ".png";
//            auto tmpimg = inputImage.copy();
            inputImage.rgbSwapped().save(fileName, "PNG");
            done = true;
            circleFound = true;
        }

        return !circles.empty();
}

int MainWindow::findLidarPoint(){

    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++){
        float tmp = circleAngle - copyOfLaserData.Data[k].scanAngle;
        if(fabs(tmp) < 1){
            cout << "lidarPointFounded: " << k << endl;
            return k;
        }
    }
}

void MainWindow::on_pushButton_clicked()
{
    state.angle = 0;
    state.x = 0.5;
    state.y = 0.5;
    ui->pushButton->setVisible(false);
    circleDetection = true;
}

