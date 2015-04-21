#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include<iostream>
#include <stdio.h>
#include<vector>
#include <time.h>
#include <fstream>
#include <sstream>
#include <iostream> // for standard I/O
#include <string>
#define _CRT_SECURE_NO_DEPRECATE

using namespace cv;
using namespace std;

struct coordinates{
int x;
int y;
int height;
int width;
};
char sequence[500];
int seq = 1;

vector<coordinates> Points;

int main()
{
   int frameWidth =0,frameHeight = 0,obsNo = 50;
    //cout<<"Enter the Width of frame : ";
    //cin>>frameWidth;
    //cout<<"Enter the Height of frame : ";
    //cin>>frameHeight;

    frameHeight = 100;
    frameWidth = 100;

    //cout<<"Enter the no of obstacle : ";
    //cin>>obsNo;


    int maxHeight = 4 * frameHeight/obsNo;
    int maxWidth =  4 * frameWidth/obsNo;




    srand(time(NULL));

	//Mat img(100,100,CV_8UC1);

	Mat img = Mat::zeros(frameWidth,frameHeight,CV_8UC1);

    for(int z = 1 ; z <= 100;z++)
    {
        Points.clear();
        snprintf(sequence, 500, "/home/pbsujit/catkin_ws/src/path_planning/extras/50_obstacle/data_%d.txt", z);
        //cout<<sequence<<endl;
        ofstream fout(sequence);
        int i = 0;
        while(i<obsNo)
        {

            coordinates tempPoint;
            tempPoint.x = frameWidth * ((float)rand()/(float)RAND_MAX);
            tempPoint.y = frameHeight * ((float)rand()/(float)RAND_MAX);

            tempPoint.height = 6 + maxHeight * ((float)rand()/(float)RAND_MAX);
            tempPoint.width = 6 + maxWidth * ((float)rand()/(float)RAND_MAX);

            if((tempPoint.y + tempPoint.height) > frameHeight ||
               (tempPoint.x + tempPoint.width) > frameWidth);

            else
            {
                int check = 0;
                for(int j = 0;j<i;j++)
                {
                    coordinates checkPoint = Points[j];

                    Rect tempRect(checkPoint.x-1,checkPoint.y-1,checkPoint.width+2,checkPoint.height+2);
                    Rect tempRect2(tempPoint.x,tempPoint.y,tempPoint.width,tempPoint.height);

                    Rect intersect = tempRect & tempRect2;
                   Rect orArea = tempRect | tempRect2;

                    if((intersect.area() > 0) || (orArea.area() < (tempRect2.area() + tempRect.area()) ))
                    {
                        check = 1;
                        break;
                    }
                }

                if(check == 1)
                    check = 0;
                else
                {
                    fout<<tempPoint.x<<","<<tempPoint.y<<","<<tempPoint.width<<","<<tempPoint.height<<"\n";
                    i++;
                    Points.push_back(tempPoint);

//                    for(int x = tempPoint.x ; x < (tempPoint.x+tempPoint.width) ;x++)
//                    {
//
//                        for(int y = tempPoint.y ;y < (tempPoint.y+tempPoint.height);y++)
//                        {
//                            img.at<uchar>(y,x) = 255;
//
//                        }
//                    }
                }
            }
        //cout<<"\nSuccess"<<i;
        //fout.close();
        }
    fout.close();
}


/*
	for(int j = 1;j<i;j++)
	{
		for(int x = Points[i].x ; x < (Points[i].x+Points[i].width) ;x++)
		{
			cout<<Points[i].x;
			for(int y = Points[i].y ;y < (Points[i].y+Points[i].height);y++)
			{
				//img.at<uchar>(y,x) = 0;

			}
		}
	}
*/

//	namedWindow("Res",0);
//	imshow("Res",img);
//	waitKey(0);
	return 0;
}

