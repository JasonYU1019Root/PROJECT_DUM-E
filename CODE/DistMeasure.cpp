// ********************************************
// Source code for DUM-E Line Follower Robot. *
// CopyRight @ JasonYU. All Rights Reserved.  *
// ********************************************

#include<cstdio>
#include<wiringPi.h>
#include<wiringSerial.h>
#include<time.h>
#include<cstdlib>
#include<unistd.h>
#include<lcd.h>
#include<softPwm.h>
#include<iostream>
#include"opencv2/opencv.hpp"
#include<cmath>
#include<algorithm>
#include<sys/time.h>
#include<cstring>

using namespace std;
using namespace cv;

//define code words for the GPIO pins
#define robotReset 7
//lcd
#define lcd_rs 3//register select pin
#define lcd_e  0//enable pin
#define lcd_d4 6//data pin 4
#define lcd_d5 1//data pin 5
#define lcd_d6 5//data pin 6
#define lcd_d7 4//data pin 7
//servo motor
#define servo 2
//ultrasound sensor
#define TRIG 27
#define ECHO 25
//default speed of the motors, modified by the pid output
#define leftMotorBaseSpeed 20
#define rightMotorBaseSpeed 20
//speed limits of the motors
#define min_speed -30
#define max_speed 50

//global variables
int robot,lcd;//variables to store numeric identifiers for wiringPi
VideoCapture cap(0);//variable to store the camera device
int cameraFlag=0;
int flag_run=0;
//variables to store the pid constants
double kp=0.4;
double ki=0;
double kd=0.8;
//variables to hold the current motor speed (+-100%)
double leftMotorSpeed=0;//initialise speed variables
double rightMotorSpeed=0;
//variables for the pid loop
double pid_error=0;//initialise error variables
double errorSum=0;
double errorOld=0;
//if the black line not detected, run backward
bool is_inline=0;//a flag indicated whether the line was detected
int loc;//for storing the current loc
int flag_direction=0;//for large degree angles
//a flag indicated the camera (servo) loc
bool flag_servoLoc=0;
//a flag indicated whether a task was to be done
bool is_pink=0;
//a flag & a var to temp disable the symbol rec
bool is_task=0;
int count_task=0;
//a flag to refresh the camera if pink block detected
bool is_taskCap=0;

//function headers
void setup(void);//function to configure the GPIO and devices for operation
void setupCamera(int,int);//set up the camera for image capture
void errorTrap(void);//lock up the programme if an error occurs
void closeCV(void);//disable the camera and close any window
int getLoc(Mat&);//get loc (error)
void lineFollow_pid(double);//basic line following using pid ctrl
double pid(int);//calc the pid output
double constrain(double,double,double);//constrain var a between val b & c
void setServoLoc(int);//set camera loc via servo
void getDist(void);//distance measurement using ultrasound sensor
void pink(Mat&);//detect whether a task was to be done
void taskCheck(void);//check tasks if pink block detected
void taskCap(void);//to refresh the camera if pink block detected

int main(int argc,char** argv)
{
    setup();//call a setup function to prepare IO and devices

    //display greeting on lcd
    lcdPosition(lcd,0,0);//cursor: first line, first column
    lcdPuts(lcd,"Bonjour! Jason");//print txt msg at current cursor pos
    lcdPosition(lcd,0,1);//cursor: second line, first column
    lcdPuts(lcd,"Press any Key...");//print txt msg at current cursor pos
    getchar();//wait for key press
    lcdClear(lcd);//clear the display
    lcdPosition(lcd,0,0);//cursor: first line, first column
    lcdPuts(lcd,"DUM-E Ready :-)");//print txt msg at current cursor pos

    while(1)//main loop to perform image processing
    {
        Mat frame;
        cap>>frame;//capture a frame from the camera and store in a new matrix variable

        Mat frameHSV;//convert the frame to HSV and apply the limits
        cvtColor(frame,frameHSV,COLOR_BGR2HSV);
        int lowH=0,highH=179,lowS=0,highS=255,lowV=0,highV=66;
        inRange(frameHSV,Scalar(lowH,lowS,lowV),Scalar(highH,highS,highV),frameHSV);

        //set the camera to init loc via servo motor
        if(flag_servoLoc)setServoLoc(0);

        //for pink block & symbol recognition
        Mat framePink;
        cvtColor(frame,framePink,COLOR_BGR2HSV);
        inRange(framePink,Scalar(161,104,0),Scalar(166,255,255),framePink);

        if(!is_task)
        {
            if(count_task<8)count_task++;
            else pink(framePink);//detect whether a task was to be done
        }

        if(is_pink&&!is_task){taskCheck();count_task=0;continue;}//check tasks if pink block detected
        
        if(is_task&&!is_taskCap)taskCap();

        loc=getLoc(frameHSV);//get loc (error)
        cout<<"loc: "<<loc<<endl;

        double output=pid(loc);//calc the pid output

        if(flag_run&&is_inline)lineFollow_pid(output);//basic line following using pid output
        else if(flag_run&&!is_inline)
        {
            if(flag_direction==1){serialPrintf(robot,"#Baffrr030,030,060,060");delay(50);}//left backward
            else if(flag_direction==2){serialPrintf(robot,"#Barrff060,060,030,030");delay(50);}//right backward
            else {serialPrintf(robot,"#Barrrr020,020,020,020");delay(50);}//backward
        }

        Rect rect(0,120,320,1),rect1(0,160,320,1),rect2(0,80,320,1),rect_middle(160,0,1,240);
        rectangle(frameHSV,rect,Scalar(255,255,255),1,1,0);
        rectangle(frameHSV,rect1,Scalar(255,255,255),1,1,0);
        rectangle(frameHSV,rect2,Scalar(255,255,255),1,1,0);
        rectangle(frameHSV,rect_middle,Scalar(255,255,255),1,1,0);
        imshow("Thresholded",frameHSV);
        
        int key=waitKey(1);//wait 1ms for a key press (required to update windows)
        key=(key==255)?-1:key;//check if the esc key has been pressed
        if(key==32){flag_run=!flag_run;if(!flag_run){serialPrintf(robot,"#Ha");delay(50);}}
        if(key==27){serialPrintf(robot,"#Ha");delay(50);break;}
    }

    closeCV();//disable the camera and close any window
    //disable motors & robot
    serialPrintf(robot,"#Ha");delay(50);
    serialClose(robot);

    return 0;
}

void setup(void)
{
    wiringPiSetup();//initialise the wiringPi lib

    pinMode(robotReset,OUTPUT);//set up the robotRest pin
    robot=serialOpen("/dev/ttyAMA0",57600);//configure the serial port for the robot and bounce the rest pin
    digitalWrite(robotReset,LOW);
    delay(50);
    digitalWrite(robotReset,HIGH);

    //initialise camera
    setupCamera(320,240);//enable the camera for OpenCV

    //initialise servo motor
    pinMode(servo,OUTPUT);
    setServoLoc(0);

    //initialise lcd
    if(lcd=lcdInit(2,16,4,lcd_rs,lcd_e,lcd_d4,lcd_d5,lcd_d6,lcd_d7,0,0,0,0))
    {
        cout<<"ERROR! LCD not ready!\n";
    }

    //initialise ultrasound sensor
    pinMode(TRIG,OUTPUT);
    pinMode(ECHO,INPUT);

    return;
}

void setupCamera(int width,int height)
{
    //open the camera device for image capture, set the dimensions of the image
    if(!cap.isOpened())
    {
        cout<<"ERROR! Camera not ready!\n";
        errorTrap();
    }
    cap.set(CAP_PROP_FRAME_WIDTH,width);
    cap.set(CAP_PROP_FRAME_HEIGHT,height);
    cameraFlag=1;
    return;
}

void errorTrap(void)
{
    while(1);
    return;
}

void closeCV(void)
{
    //clean up the open camera and windows
    if(cameraFlag)
        cap.release();
    destroyAllWindows();
}

int getLoc(Mat& img)//get loc (error)
{
    int i=(int)img.rows/2;//check for middle row pixels only
    int i1=(int)img.rows*2/3;//also check for row at 2/3 pixels
    int i2=(int)img.rows*1/3;//also check for row at 1/3 pixels
    int count=0;
    int sum_x;

    for(int x=0;x<img.cols;x++)
    {
        uchar p=img.at<uchar>(i,x);
        uchar p1=img.at<uchar>(i1,x);
        uchar p2=img.at<uchar>(i2,x);
        int pixel=p;
        if(pixel>200){
            count++;
            sum_x+=x;
        }
        int pixel1=p1;
        if(pixel1>200){
            count++;
            sum_x+=x;
        }
        int pixel2=p2;
        if(pixel2>200){
            count++;
            sum_x+=x;
        }
    }
    if(count>0){is_inline=1;flag_direction=0;}else is_inline=0;
    if(!count)return 0;
    int x_avg=(int)sum_x/count;
    if(count>160)
    {
        if(x_avg>=(int)(img.cols/2))flag_direction=1;
        else flag_direction=2;
    }

    //return centre-160;
    return x_avg-(int)(img.cols/2);
}

void lineFollow_pid(double output)//basic line following using pid ctrl
{
    leftMotorSpeed=leftMotorBaseSpeed+output;//calc the modified motor speed
    rightMotorSpeed=rightMotorBaseSpeed-output;

    //apply new speed and direction to each motor
    if(leftMotorSpeed>=0&&rightMotorSpeed>=0)
    {
        leftMotorSpeed=constrain(leftMotorSpeed,0,max_speed);
        rightMotorSpeed=constrain(rightMotorSpeed,0,max_speed);
        serialPrintf(robot,"#Baffff0%d,0%d,0%d,0%d",(int)leftMotorSpeed,(int)leftMotorSpeed,(int)rightMotorSpeed,(int)rightMotorSpeed);
        delay(50);
    }
    else if(leftMotorSpeed<0&&rightMotorSpeed>=0)
    {
        leftMotorSpeed=constrain(leftMotorSpeed,min_speed,0);
        rightMotorSpeed=constrain(rightMotorSpeed,0,max_speed);
        serialPrintf(robot,"#Barrff0%d,0%d,0%d,0%d",-(int)leftMotorSpeed,-(int)leftMotorSpeed,(int)rightMotorSpeed,(int)rightMotorSpeed);
        delay(50);
    }
    else if(leftMotorSpeed>=0&&rightMotorSpeed<0)
    {
        leftMotorSpeed=constrain(leftMotorSpeed,0,max_speed);
        rightMotorSpeed=constrain(rightMotorSpeed,min_speed,0);
        serialPrintf(robot,"#Baffrr0%d,0%d,0%d,0%d",(int)leftMotorSpeed,(int)leftMotorSpeed,-(int)rightMotorSpeed,-(int)rightMotorSpeed);
        delay(50);
    }
    else if(leftMotorSpeed<0&&rightMotorSpeed<0)
    {
        leftMotorSpeed=constrain(leftMotorSpeed,min_speed,0);
        rightMotorSpeed=constrain(rightMotorSpeed,min_speed,0);
        serialPrintf(robot,"#Barrrr0%d,0%d,0%d,0%d",-(int)leftMotorSpeed,-(int)leftMotorSpeed,-(int)rightMotorSpeed,-(int)rightMotorSpeed);
        delay(50);
    }

    return;
}

double pid(int pos)//calc the pid output
{
    //pid loop
    errorOld=pid_error;//save the old error for differential component
    pid_error=pos;//the error in pos
    errorSum+=pid_error;

    //calc the components of pid
    double proportional=pid_error*kp;
    double integral=errorSum*ki;
    double differential=(pid_error-errorOld)*kd;

    double output=proportional+integral+differential;//calc the result

    return output;
}

double constrain(double a,double b,double c)//constrain var a between val b & c
{
    if(a<b)return b;
    if(a>c)return c;
    return a;
}

void setServoLoc(int mode)//set camera loc via servo
{
    switch(mode)
    {
        case(0)://default for line following
        {
            int angle=80;
            int i=0,k=180;
            float x=0;
            while(k--)
            {
                x=11.11*i;
                digitalWrite(servo,HIGH);
                delayMicroseconds(500+x);
                digitalWrite(servo,LOW);
                delayMicroseconds(19500-x);
                if(i==angle)break;
                i++;
            }
            break;
        }
        case(1)://only when pink blocks were detected
        {
            int angle=20;
            int i=0,k=180;
            float x=0;
            while(k--)
            {
                x=11.11*i;
                digitalWrite(servo,HIGH);
                delayMicroseconds(500+x);
                digitalWrite(servo,LOW);
                delayMicroseconds(19500-x);
                if(i==angle)break;
                i++;
            }
            break;
        }
        default:break;
    }
    return;
}

void getDist(void)//distance measurement using ultrasound sensor
{
    struct timeval start_time;
    struct timeval stop_time;

    digitalWrite(TRIG,LOW);
    delay(10);
    digitalWrite(TRIG,HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG,LOW);

    while(!(digitalRead(ECHO)==1))gettimeofday(&start_time,NULL);
    while(!(digitalRead(ECHO)==0))gettimeofday(&stop_time,NULL);

    double start,stop;
    start=start_time.tv_sec*1000000+start_time.tv_usec;
    stop=stop_time.tv_sec*1000000+stop_time.tv_usec;
    double dist=(stop-start)/1000000*34000/2;

    lcdPosition(lcd,0,1);//row 2, col 1
    lcdPuts(lcd,"                ");//clear the row
    char msg[16];
    sprintf(msg,"Dist: %.2f cm",dist);
    lcdPosition(lcd,0,1);//row 2, col 1
    lcdPuts(lcd,msg);

    return;
}

void pink(Mat& img)//detect whether a task was to be done
{
    int i=(int)img.rows/2;//check for middle row pixels only
    int i1=(int)img.rows*2/3;//also check for row at 2/3 pixels
    int i2=(int)img.rows*1/3;//also check for row at 1/3 pixels
    int count=0;

    for(int x=0;x<img.cols;x++)
    {
        uchar p=img.at<uchar>(i,x);
        uchar p1=img.at<uchar>(i1,x);
        uchar p2=img.at<uchar>(i2,x);
        int pixel=p;
        if(pixel>200)count++;
        int pixel1=p1;
        if(pixel1>200)count++;
        int pixel2=p2;
        if(pixel2>200)count++;
    }
    if(count>0)is_pink=1;
    else is_pink=0;
    is_taskCap=0;
    return;
}

void taskCheck(void)//check tasks if pink block detected
{
    serialPrintf(robot,"#Ha");
    setServoLoc(1);
    delay(500);
    is_task=1;

    return;
}

void taskCap(void)//to refresh the camera if pink block detected
{
    Mat symbol;
    cap.read(symbol);
    imshow("Symbol Detected",symbol);
    delay(500);
    is_taskCap=1;
    is_task=0;
    is_pink=0;
    setServoLoc(0);

    lcdPosition(lcd,0,0);//row 1, col 1
    lcdPuts(lcd,"Dist Measurement");
    getDist();

    return;
}
