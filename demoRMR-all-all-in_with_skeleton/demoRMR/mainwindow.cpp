#include "mainwindow.h"
#include "qlabel.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
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




    datacounter=0;


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);    
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


    //set ratio of main frame
    int width = size().width();
    int height = size().height();
    if (!(width * 16 == height * 9))
    {
        int newWidth = height * 16 / 9;
        resize(newWidth, height);
    }



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

    drawCamera(&painter, frameState ? mainRect : secondaryRect);
    drawLidar(&painter, frameState ? mainRect : secondaryRect, pero);
    QColor fillColor(255, 255, 255, 128);
    QBrush brush(fillColor);
    painter.fillRect(secondaryRect, brush);

    drawMinimap(&painter, frameState ? secondaryRect : mainRect, pero);
    drawWarningWidged(&painter, thirdRect, pero);
    drawWarningText(&painter, frameState ? mainRect : secondaryRect);
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{


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
    return 0;
}

///toto je calback na data zo skeleton trackera, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z trackera
int MainWindow::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    updateSkeletonPicture=1;

    switch (detectGesture()){
    case LEFT_LIKE:
        break;
        speedSetting(maxForwardspeed, SET, TRANSLATED);
        cout << "New speed is: " << forwardspeed << endl;
        break;
    case LEFT_DISLIKE:
        speedSetting(0, SET, TRANSLATED);
        rotationspeed = 0;
        cout << "New speed is: " << forwardspeed << endl;
        break;
    case RIGHT_LIKE:
        speedSetting(7, ADD_TO_EXISTED, TRANSLATED);
        cout << "New speed is: " << forwardspeed << endl;
        break;
    case RIGHT_DISLIKE:
        speedSetting(-7, ADD_TO_EXISTED, TRANSLATED);
        cout << "New speed is: " << forwardspeed << endl;
        break;
    case TELEPHONE_LEFT:
        speedSetting(-0.03, ADD_TO_EXISTED, ANGULAR);
        cout << "New speed is: " << rotationspeed << endl;
        break;
    case TELEPHONE_RIGHT:
        speedSetting(0.03, ADD_TO_EXISTED, ANGULAR);
        cout << "New speed is: " << rotationspeed << endl;
        break;
    default:
        break;
    }

    return 0;
}
void MainWindow::on_pushButton_9_clicked() //start button
{
    if(connected) return;
    forwardspeed=0;
    rotationspeed=0;

    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    //---simulator ma port 8889, realny robot 8000
    robot.setCameraParameters("http://"+ipaddress+":8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
    robot.setSkeletonParameters("127.0.0.1",23432,23432,std::bind(&MainWindow::processThisSkeleton,this,std::placeholders::_1));
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

    connected = true;
    ui->pushButton_9->hide();
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

//QColor MainWindow::getLidarPointColor(float distance) {
//    // Aktualizacia maximalnej hodnoty
//    if (distance > maxVal) {
//        maxVal = distance;
//    }
//    float ratio = distance / maxVal;
//    int r = static_cast<int>(255 * (1-ratio));
//    int b = static_cast<int>(255 * ratio);
//    int g = 0;
//    return QColor(r, g, b);
//}

void MainWindow::drawCamera(QPainter* painter, QRect rect){
    if(actIndex>-1)/// ak zobrazujem data z kamery ak aspon niektory frame vo vectore je naplneny
    {
//        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
//        cout << frame->rows << endl;
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

    pero.setColor(Qt::blue);
    painter->setPen(pero);

    for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
    {
        int dist=copyOfLaserData.Data[k].scanDistance/(12000/(rect.height()-60)-5);
        int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
        int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
        if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
        {
//            pero.setColor(getLidarPointColor(dist));
            painter->drawEllipse(QPoint(xp, yp),1,1);
        }
    }
    pero.setColor(Qt::magenta);
    painter->setBrush(Qt::magenta);
    painter->setPen(pero);
    painter->drawEllipse(QPoint(rect.x() + rect.width()/2, rect.y() + rect.height()/2),5,5);
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


customGestures MainWindow::detectGesture() {

    float threshold;
    if(skeleJoints.joints[left_thumb_tip].x != 0){
        float dx = skeleJoints.joints[left_thumb_tip].x - skeleJoints.joints[left_thumb_mcp].x;
        float dy = skeleJoints.joints[left_thumb_tip].y - skeleJoints.joints[left_thumb_mcp].y;
        threshold = std::sqrt(dx*dx + dy*dy);
    }else {
        float dx = skeleJoints.joints[right_thumb_tip].x - skeleJoints.joints[right_thumb_mcp].x;
        float dy = skeleJoints.joints[right_thumb_tip].y - skeleJoints.joints[right_thumb_mcp].y;
        threshold = std::sqrt(dx*dx + dy*dy);
    }
    threshold *= 1.25;


    float threshold2;
    if(skeleJoints.joints[left_wrist].x != 0){
        float dx = skeleJoints.joints[left_wrist].x - skeleJoints.joints[left_index_mcp].x;
        float dy = skeleJoints.joints[left_wrist].y - skeleJoints.joints[left_index_mcp].y;
        threshold2 = std::sqrt(dx*dx + dy*dy);
    }else {
        float dx = skeleJoints.joints[right_wrist].x - skeleJoints.joints[right_index_mcp].x;
        float dy = skeleJoints.joints[right_wrist].y - skeleJoints.joints[right_index_mcp].y;
        threshold2 = std::sqrt(dx*dx + dy*dy);
    }
//    cout << "threshold2: " << threshold2 << endl;
    threshold2 /= 2;


    if(abs(skeleJoints.joints[right_thumb_tip].y - skeleJoints.joints[right_wrist].y) > threshold2){
        if(abs(skeleJoints.joints[right_index_tip].y - skeleJoints.joints[right_wrist].y) > threshold2){
            if(abs(skeleJoints.joints[right_middle_tip].y - skeleJoints.joints[right_wrist].y) > threshold2){
                if(abs(skeleJoints.joints[right_ringy_tip].y - skeleJoints.joints[right_wrist].y) > threshold2){
                    if(abs(skeleJoints.joints[right_pink_tip].y - skeleJoints.joints[right_wrist].y) > threshold2){
                        startDetected = true;
                        ui->lblStatus->setText("Detecting gestures");
                    }}}}
    }

    if(abs(skeleJoints.joints[left_thumb_tip].y - skeleJoints.joints[left_wrist].y) > threshold2)
        if(abs(skeleJoints.joints[left_index_tip].y - skeleJoints.joints[left_wrist].y) > threshold2)
            if(abs(skeleJoints.joints[left_middle_tip].y - skeleJoints.joints[left_wrist].y) > threshold2)
                if(abs(skeleJoints.joints[left_ringy_tip].y - skeleJoints.joints[left_wrist].y) > threshold2)
                    if(abs(skeleJoints.joints[left_pink_tip].y - skeleJoints.joints[left_wrist].y) > threshold2){
                        startDetected = false;
                        ui->lblStatus->setText("Not detecting gestures");
    }

    if(!startDetected) return LEFT_DISLIKE;

    // Minimalny ozdel medzi koncom palca a ukazovaka
    if(abs(skeleJoints.joints[left_thumb_tip].y - skeleJoints.joints[left_index_tip].y) > threshold){

        // Poradie ostatnych prstov
        if(skeleJoints.joints[left_index_ip].y > skeleJoints.joints[left_middle_ip].y){
            if (skeleJoints.joints[left_middle_ip].y > skeleJoints.joints[left_ring_ip].y){
                if(skeleJoints.joints[left_ring_ip].y > skeleJoints.joints[left_pink_ip].y){

                    cout << "Gesture: LEFT_DISLIKE" << endl;
                    return LEFT_DISLIKE;
                }
            }
        }

        // Poradie ostatnych prstov
        if(skeleJoints.joints[left_index_ip].y < skeleJoints.joints[left_middle_ip].y){
            if (skeleJoints.joints[left_middle_ip].y < skeleJoints.joints[left_ring_ip].y){
                if(skeleJoints.joints[left_ring_ip].y < skeleJoints.joints[left_pink_ip].y){

                    cout << "Gesture: LEFT_LIKE" << endl;
                    return LEFT_LIKE;
                }
            }
        }
    }

    // Minimalny rozdiel medzi koncom palca a ukazovaka
    if(abs(skeleJoints.joints[right_thumb_tip].y - skeleJoints.joints[right_index_tip].y) > threshold){

        // Poradie ostatnych prstov
        if(skeleJoints.joints[right_index_ip].y > skeleJoints.joints[right_middle_ip].y){
            if (skeleJoints.joints[right_middle_ip].y > skeleJoints.joints[right_ring_ip].y){
                if(skeleJoints.joints[right_ring_ip].y > skeleJoints.joints[right_pink_ip].y){

                    cout << "Gesture: RIGHT_DISLIKE" << endl;
                    return RIGHT_DISLIKE;
                }
            }
        }

        // Poradie ostatnych prstov
        if(skeleJoints.joints[right_index_ip].y < skeleJoints.joints[right_middle_ip].y){
            if (skeleJoints.joints[right_middle_ip].y < skeleJoints.joints[right_ring_ip].y){
                if(skeleJoints.joints[right_ring_ip].y < skeleJoints.joints[right_pink_ip].y){

                    cout << "Gesture: RIGHT_LIKE" << endl;
                    return RIGHT_LIKE;
                }
            }
        }
    }

//    cout << endl << endl << threshold << endl << abs(skeleJoints.joints[left_wrist].x - skeleJoints.joints[left_pink_tip].x) << abs(skeleJoints.joints[left_middle_tip].x - skeleJoints.joints[left_wrist].x);

    if(abs(skeleJoints.joints[left_wrist].x - skeleJoints.joints[left_pink_tip].x) > threshold*1.5){
        if(abs(skeleJoints.joints[left_middle_tip].x - skeleJoints.joints[left_wrist].x) < threshold*1.5){
            cout << "Gesture: TELEPHONE_LEFT" << endl;
            return TELEPHONE_LEFT;
        }
    }

    if(abs(skeleJoints.joints[right_wrist].x - skeleJoints.joints[right_pink_tip].x) > threshold*1.5){
        if(abs(skeleJoints.joints[right_middle_tip].x - skeleJoints.joints[right_wrist].x) < threshold*1.5){
            cout << "Gesture: TELEPHONE_RIGHT" << endl;
            return TELEPHONE_RIGHT;
        }
    }

    return NONE;
}

void MainWindow::speedSetting(double speed, settingSpeedOption option, speedType type){

    switch (option) {
    case ADD_TO_EXISTED:
        switch (type) {
        case TRANSLATED:
            if(speedCheck(forwardspeed + speed, type)){
                forwardspeed += speed;
                robot.setTranslationSpeed(forwardspeed);
            }
            break;
        case ANGULAR:
            if(speedCheck(rotationspeed + speed, type)){
                rotationspeed += speed;
                robot.setRotationSpeed(rotationspeed);
            }
            break;
        }
        break;
    case SET:
        switch (type) {
        case TRANSLATED:
            if(speedCheck(speed, type)){
                forwardspeed = speed;
                robot.setTranslationSpeed(forwardspeed);
            }
            break;
        case ANGULAR:
            if(speedCheck(speed, type)){
                rotationspeed = speed;
                robot.setRotationSpeed(rotationspeed);
            }
            break;
        }
        break;
    }
}

bool MainWindow::speedCheck(double speed, speedType type){
    switch (type) {
    case TRANSLATED:
        if(speed < -maxForwardspeed || maxForwardspeed > 400){
            return false;
        }
        break;
    case ANGULAR:
        if(speed < -maxRotationspeed || speed > maxRotationspeed){
            return false;
        }
        break;
    }
    return true;
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
    speedSetting(0, SET, TRANSLATED);
    rotationspeed = 0;
}

