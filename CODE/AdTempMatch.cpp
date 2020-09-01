// *******************************************
// Advanced Template Matching Algorithm.     *
// CopyRight @ JasonYU. All Rights Reserved. *
// *******************************************

#include<cstdio>
#include<iostream>
#include"opencv2/opencv.hpp"

using namespace std;
using namespace cv;

Mat processing(Mat);

int main(){
    Mat cap=imread("cap.png");
    Mat result=processing(cap);
    waitKey(0);
    return 0;
}

Mat processing(Mat frame){
    imshow("Raw",frame);

    Mat symbol_HSV;
    Mat symbol_Bina;
    Mat symbol_contours;
    Mat image_transformed;
    Mat symbol_transformed;
    Mat symbol_final;
    bool hull_sort;
    int maxArea;
    vector<vector<Point>>contours;
    vector<Vec4i>hierarchy;
    vector<int>hull;
    Point2f symbol_conners[4],screen_conners[4];
    screen_conners[0]=Point2f(0,0);
    screen_conners[1]=Point2f(640,0);
    screen_conners[2]=Point2f(640,480);
    screen_conners[3]=Point2f(0,480);

    cvtColor(frame,symbol_HSV,COLOR_BGR2HSV);
    inRange(symbol_HSV,Scalar(135,86,45),Scalar(170,255,255),symbol_Bina);//apply pink filter
    Mat spotFilter=getStructuringElement(MORPH_ELLIPSE,Size(5,5));
    erode(symbol_Bina,symbol_Bina,spotFilter);
    Mat maskMorph=getStructuringElement(MORPH_ELLIPSE,Size(4,4));
    dilate(symbol_Bina,symbol_Bina,maskMorph);
    imshow("Thresholded",symbol_Bina);
    findContours(symbol_Bina,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());
    symbol_contours=Mat::zeros(frame.size(),CV_8UC3);
    vector<vector<Point>>polyContours(contours.size());
    maxArea=0;
    for(size_t index=0;index<contours.size();index++){
        if(contourArea(contours[index])>contourArea(contours[maxArea])){maxArea=index;}
        approxPolyDP(contours[maxArea],polyContours[maxArea],12,true);
        //drawContours(symbol_contours,polyContours,-1,Scalar(0,0,255),2);
        convexHull(polyContours[maxArea],hull,false);
        //imshow("Contour",symbol_contours);
        if (hull.size()==4){
            hull_sort=false;
            int n=4;
            while(!hull_sort){
                for(int i=1;i<n;i++){
                    hull_sort=true;
                    if(polyContours[maxArea][i-1].x>polyContours[maxArea][i].x){
                        swap(polyContours[maxArea][i-1],polyContours[maxArea][i]);
                        hull_sort=false;
                    }
                }
                n--;
            }
            if(polyContours[maxArea][0].y<polyContours[maxArea][1].y){
                symbol_conners[0]=polyContours[maxArea][0];
                symbol_conners[3]=polyContours[maxArea][1];
            }
            else{
                symbol_conners[0]=polyContours[maxArea][1];
                symbol_conners[3]=polyContours[maxArea][0];
            }
            if(polyContours[maxArea][2].y<polyContours[maxArea][3].y){
                symbol_conners[1]=polyContours[maxArea][2];
                symbol_conners[2]=polyContours[maxArea][3];
            }
            else{
                symbol_conners[1]=polyContours[maxArea][3];
                symbol_conners[2]=polyContours[maxArea][2];
            }
        }
        image_transformed=getPerspectiveTransform(symbol_conners,screen_conners);
        warpPerspective(frame,symbol_transformed,image_transformed,frame.size());
        imshow("Transformed",symbol_transformed);
        cvtColor(symbol_transformed,symbol_transformed,COLOR_BGR2HSV);
        inRange(symbol_transformed,Scalar(135,86,45),Scalar(170,255,255),symbol_final);
        imshow("Processed",symbol_final);
    return symbol_final;
    }
}
