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

//function headers
void setup(void);//function to configure the GPIO and devices for operation
void setupCamera(int,int);//set up the camera for image capture
void errorTrap(void);//lock up the programme if an error occurs
void closeCV(void);//disable the camera and close any window
int getLoc(Mat&,Mat&,Mat&,Mat&,Mat&);//get loc (error)
void lineFollow_pid(double);//basic line following using pid ctrl
double pid(int);//calc the pid output
double constrain(double,double,double);//constrain var a between val b & c

int main(int argc,char** argv)
{
    setup();//call a setup function to prepare IO and devices

    while(1)//main loop to perform image processing
    {
        Mat frame;
        cap>>frame;//capture a frame from the camera and store in a new matrix variable

        Mat frameHSV;//convert the frame to HSV and apply the limits
        cvtColor(frame,frameHSV,COLOR_BGR2HSV);
        int lowH=0,highH=179,lowS=0,highS=255,lowV=0,highV=66;
        inRange(frameHSV,Scalar(lowH,lowS,lowV),Scalar(highH,highS,highV),frameHSV);

        //for short cut task
        Mat frameYellow;//normal speed
        cvtColor(frame,frameYellow,COLOR_BGR2HSV);
        inRange(frameYellow,Scalar(17,35,35),Scalar(28,255,255),frameYellow);
        Mat frameBlue;//normal speed, run between split tracks
        cvtColor(frame,frameBlue,COLOR_BGR2HSV);
        inRange(frameBlue,Scalar(105,40,35),Scalar(122,255,255),frameBlue);
        Mat frameGreen;//higher speed
        cvtColor(frame,frameGreen,COLOR_BGR2HSV);
        inRange(frameGreen,Scalar(58,25,35),Scalar(92,255,255),frameGreen);
        Mat frameRed;//lower speed
        cvtColor(frame,frameRed,COLOR_BGR2HSV);
        inRange(frameRed,Scalar(169,64,67),Scalar(179,255,255),frameRed);

        loc=getLoc(frameHSV,frameYellow,frameBlue,frameGreen,frameRed);//get loc (error)
        cout<<"loc: "<<loc<<endl;

        double output=pid(loc);//calc the pid output

        if(flag_run&&is_inline)lineFollow_pid(output);//basic line following using pid output
        else if(flag_run&&!is_inline)
        {
            if(flag_direction==1){serialPrintf(robot,"#Baffrr030,030,060,060");delay(50);}//left backward
            else if(flag_direction==2){serialPrintf(robot,"#Barrff060,060,030,030");delay(50);}//right backward
            else {serialPrintf(robot,"#Barrrr020,020,020,020");delay(50);}//backward
        }

        Rect rect(0,120,320,1),rect1(0,160,320,1),rect2(0,80,320,1),rect_middle(160,0,1,240),rect3(0,200,320,1);
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
    wiringPiSetupGpio();
    robot=serialOpen("/dev/ttyAMA0",57600);//configure the serial port for the robot and bounce the rest pin
    digitalWrite(robotReset,LOW);
    delay(50);
    digitalWrite(robotReset,HIGH);

    //initialise camera
    setupCamera(320,240);//enable the camera for OpenCV

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

int getLoc(Mat& img,Mat& imgYellow,Mat& imgBlue,Mat& imgGreen,Mat& imgRed)//get loc (error)
{
    int i=(int)img.rows/2;//check for middle row pixels only
    int i1=(int)img.rows*2/3;//also check for row at 2/3 pixels
    int i2=(int)img.rows*1/3;//also check for row at 1/3 pixels
    int count=0,countYellow=0,countBlue=0,countGreen=0,countRed=0;
    int sum_x=0,sum_xYellow=0,sum_xBlue=0,sum_xGreen=0,sum_xRed=0;

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
    for(int x=0;x<imgYellow.cols;x++)
    {
        uchar pYellow=imgYellow.at<uchar>(i,x);
        uchar p1Yellow=imgYellow.at<uchar>(i1,x);
        uchar p2Yellow=imgYellow.at<uchar>(i2,x);
        int pixelYellow=pYellow;
        if(pixelYellow>200){
            countYellow++;
            sum_xYellow+=x;
        }
        int pixel1Yellow=p1Yellow;
        if(pixel1Yellow>200){
            countYellow++;
            sum_xYellow+=x;
        }
        int pixel2Yellow=p2Yellow;
        if(pixel2Yellow>200){
            countYellow++;
            sum_xYellow+=x;
        }
    }
    for(int x=0;x<imgBlue.cols;x++)
    {
        uchar pBlue=imgBlue.at<uchar>(i,x);
        uchar p1Blue=imgBlue.at<uchar>(i1,x);
        uchar p2Blue=imgBlue.at<uchar>(i2,x);
        int pixelBlue=pBlue;
        if(pixelBlue>200){
            countBlue++;
            sum_xBlue+=x;
        }
        int pixel1Blue=p1Blue;
        if(pixel1Blue>200){
            countBlue++;
            sum_xBlue+=x;
        }
        int pixel2Blue=p2Blue;
        if(pixel2Blue>200){
            countBlue++;
            sum_xBlue+=x;
        }
    }
    for(int x=0;x<imgGreen.cols;x++)
    {
        uchar pGreen=imgGreen.at<uchar>(i,x);
        uchar p1Green=imgGreen.at<uchar>(i1,x);
        uchar p2Green=imgGreen.at<uchar>(i2,x);
        int pixelGreen=pGreen;
        if(pixelGreen>200){
            countGreen++;
            sum_xGreen+=x;
        }
        int pixel1Green=p1Green;
        if(pixel1Green>200){
            countGreen++;
            sum_xGreen+=x;
        }
        int pixel2Green=p2Green;
        if(pixel2Green>200){
            countGreen++;
            sum_xGreen+=x;
        }
    }
    for(int x=0;x<imgRed.cols;x++)
    {
        uchar pRed=imgRed.at<uchar>(i,x);
        uchar p1Red=imgRed.at<uchar>(i1,x);
        uchar p2Red=imgRed.at<uchar>(i2,x);
        int pixelRed=pRed;
        if(pixelRed>200){
            countRed++;
            sum_xRed+=x;
        }
        int pixel1Red=p1Red;
        if(pixel1Red>200){
            countRed++;
            sum_xRed+=x;
        }
        int pixel2Red=p2Red;
        if(pixel2Red>200){
            countRed++;
            sum_xRed+=x;
        }
    }
    if(count>0||countYellow>0||countBlue>0||countGreen||countRed){is_inline=1;flag_direction=0;}else is_inline=0;
    if(!count&&!countYellow&&!countBlue&&!countGreen&&!countRed)return 0;
    int x_avg=count==0?0:(int)sum_x/count;
    int x_avgYellow=countYellow==0?0:(int)sum_xYellow/countYellow;
    int x_avgBlue=countBlue==0?0:(int)sum_xBlue/countBlue;
    int x_avgGreen=countGreen==0?0:(int)sum_xGreen/countGreen;
    int x_avgRed=countRed==0?0:(int)sum_xRed/countRed;
    if(countYellow==0&&countBlue==0&&countGreen==0&&countRed==0&&count>160)
    {
        if(x_avg>=(int)(img.cols/2))flag_direction=1;
        else flag_direction=2;
    }
    else if(countYellow>160)
    {
        if(x_avgYellow>=(int)(imgYellow.cols/2))flag_direction=1;
        else flag_direction=2;
    }
    else if(countBlue>160)
    {
        if(x_avgBlue>=(int)(imgBlue.cols/2))flag_direction=1;
        else flag_direction=2;
    }
    else if(countGreen>160)
    {
        if(x_avgBlue>=(int)(imgGreen.cols/2))flag_direction=1;
        else flag_direction=2;
    }
    else if(countRed>160)
    {
        if(x_avgBlue>=(int)(imgRed.cols/2))flag_direction=1;
        else flag_direction=2;
    }

    //return centre-160;
    if(countYellow>0)return x_avgYellow-(int)(imgYellow.cols/2);
    else if(countBlue>0)return x_avgBlue-(int)(imgBlue.cols/2);
    else if(countGreen>0)return x_avgGreen-(int)(imgGreen.cols/2);
    else if(countRed>0)return x_avgRed-(int)(imgRed.cols/2);
    else return x_avg-(int)(img.cols/2);
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
