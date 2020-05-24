/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/mat.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

// gobal constant
const int MinWidth = 5;
const int MinHeight = 10;
const double MaxRatio = 0.2;
const int MinArea = 50;
const int MaxArea = 4500;
int GetOnTrack = 0;

// define lambda function 
bool isTrafficCone(std::vector<cv::Point> convexHull);
//void drawAtConeCenter(std::vector<cv::Point> trafficCone, cv::Mat &img);

//lambda function
bool isTrafficCone(std::vector<cv::Point> convexHull){ 
      cv::Rect bounding = cv::boundingRect(convexHull);
      int area = bounding.area();
      double widthHeightRatio= bounding.width / bounding.height;
      if (area < MinArea || area > MaxArea || bounding.width < MinWidth || bounding.height < MinHeight || widthHeightRatio > MaxRatio) {
         return false;
         }
      return true;
      }


//bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
//{
//      //Number of key points
//      int N = key_point.size();
// 
//      //构造矩阵X
//      cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
//      for (int i = 0; i < n + 1; i++)
//      {
//            for (int j = 0; j < n + 1; j++)
//            {
//                  for (int k = 0; k < N; k++)
//                  {
//                        X.at<double>(i, j) = X.at<double>(i, j) + std::pow(key_point[k].y, i + j);
//                  }
//            }
//      }
// 
//      //构造矩阵Y
//      cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
//      for (int i = 0; i < n + 1; i++)
//      {
//            for (int k = 0; k < N; k++)
//            {
//                  Y.at<double>(i, 0) = Y.at<double>(i, 0) + std::pow(key_point[k].y, i) * key_point[k].x;
//            }
//      }
// 
//      A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
//      //求解矩阵A
//      cv::solve(X, Y, A, cv::DECOMP_LU);
//      return true;
//}

//void drawAtCone(std::vector<cv::Point> trafficCone, cv::Mat &img) {
//    cv::Moments moments = cv::moments(trafficCone);
//    int xCenter = (int)(moments.m10 / moments.m00);
//    int yCenter = (int)(moments.m01 / moments.m00);
//    cv::circle(img, cv::Point(xCenter, yCenter), 3, cv::Scalar(0,255,255), -1);
//}



int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.argb --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Handler to receive distance readings (realized as C++ lambda).
//            std::mutex distancesMutex;
//            float front{0};
//            float rear{0};
//            float left{0};
//            float right{0};
//            auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env){
//                auto senderStamp = env.senderStamp();
//                // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
//                opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));

//                // Store distance readings.
//                std::lock_guard<std::mutex> lck(distancesMutex);
//                switch (senderStamp) {
//                    case 0: front = dr.distance(); break;
//                    case 2: rear = dr.distance(); break;
//                    case 1: left = dr.distance(); break;
//                    case 3: right = dr.distance(); break;
//                }
//            };
//            // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
//            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);
            

            // LOAD CASCADE CLASSIFIER
            cv::CascadeClassifier kiwi_detector;
            kiwi_detector.load("/usr/cascade.xml");
            	if (kiwi_detector.empty()&VERBOSE)
	{
		std::cout << "分类器加载失败!!!" << std::endl;
		return -1;
	}
            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy image into cvMat structure.
                    // Be aware of that any code between lock/unlock is blocking
                    // the camera to provide the next frame. Thus, any
                    // computationally heavy algorithms should be placed outside
                    // lock/unlock
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                // TODO: Do something with the frame.
                cv::Mat hsv;
                //cv::Mat Ones;
                //Ones = cv::Mat::zeros(cv::Size(320, 1280), CV_32F);
                // BLUE CONES DETECTION DECLARE 
                cv::Mat BlueColourThreshod;
                cv::Mat Bluedilate;
                cv::Mat Blueerode;
                cv::Mat Bluegaussian;
                cv::Mat Bluecanny;
                cv::Mat BlueContour_img;
                cv::Mat BlueConvexHulls_img;
                cv::Mat blueCones_img;
                std::vector<cv::Point> blueCones_coordiate;               

                std::vector<std::vector<cv::Point> > Bluecontours;
                std::vector<std::vector<cv::Point> > blueConesConvex;

                // YELLOW CONES DETECTION DECLARE 
                cv::Mat YellowColourThreshod;
                cv::Mat Yellowdilate;
                cv::Mat Yellowerode;
                cv::Mat Yellowgaussian;
                cv::Mat Yellowcanny;
                cv::Mat YellowContour_img;
                cv::Mat YellowConvexHulls_img;
                cv::Mat YellowCones_img;
                std::vector<cv::Point> YellowCones_coordiate; 


                std::vector<std::vector<cv::Point> > Yellowcontours;
                std::vector<std::vector<cv::Point> > YellowConesConvex;
                
                // RED CONES DETECTION DECLARE 
                cv::Mat RedColourThreshod;
                cv::Mat Reddilate;
                cv::Mat Rederode;
                cv::Mat Redgaussian;
                cv::Mat Redcanny;
                cv::Mat RedContour_img;
                cv::Mat RedConvexHulls_img;
                cv::Mat RedCones_img;
                std::vector<cv::Point> RedCones_coordiate; 


                std::vector<std::vector<cv::Point> > Redcontours;
                std::vector<std::vector<cv::Point> > RedConesConvex;


                cv::Mat imgWithCones;
                imgWithCones = img.clone();

                cv::cvtColor(img,hsv,cv::COLOR_BGR2HSV);

                cv::Scalar hsvBlueLow(100,70,40);
                cv::Scalar hsvBlueHi(124,255,255);
                cv::inRange(hsv,hsvBlueLow,hsvBlueHi,BlueColourThreshod);

                cv::Scalar hsvYellowLow(5,60,100);
                cv::Scalar hsvYellowHi(34,255,255);
                cv::inRange(hsv,hsvYellowLow,hsvYellowHi,YellowColourThreshod);

                cv::Scalar hsvRedLow(156,60,100);
                cv::Scalar hsvRedHi(180,255,255);
                //cv::Scalar hsvRedLow1(0,60,100);
                //cv::Scalar hsvRedHi1(10,255,255);
                cv::inRange(hsv,hsvRedLow,hsvRedHi,RedColourThreshod);
                //cv::inRange(hsv,hsvRedLow1,hsvRedHi1,RedColourThreshod);
                //Ones.copyTo(BlueColourThreshod.rowRange(0, 319));
                //Ones.row(0, 69).copyTo(BlueColourThreshod.rowRange(650, 719));



                




                //Dilate
                uint32_t iterations{12};
                cv::dilate(BlueColourThreshod.rowRange(400, 650),Bluedilate,cv::Mat(),cv::Point(-1,1),iterations,1,1);

                cv::dilate(YellowColourThreshod.rowRange(400, 650),Yellowdilate,cv::Mat(),cv::Point(-1,1),iterations,1,1);

                cv::dilate(RedColourThreshod.rowRange(400, 650),Reddilate,cv::Mat(),cv::Point(-1,1),iterations,1,1);
                //Erode
                cv::erode(Bluedilate,Blueerode,cv::Mat(),cv::Point(-1,1),iterations,1,1);

                cv::erode(Yellowdilate,Yellowerode,cv::Mat(),cv::Point(-1,1),iterations,1,1);

                cv::erode(Reddilate,Rederode,cv::Mat(),cv::Point(-1,1),iterations,1,1);
                //Apply Gaussian filter
                cv::GaussianBlur(Blueerode, Bluegaussian, cv::Size(5, 5), 0);

                cv::GaussianBlur(Yellowerode, Yellowgaussian, cv::Size(5, 5), 0);

                cv::GaussianBlur(Rederode, Redgaussian, cv::Size(5, 5), 0);
                //Edge dectection
                cv::Canny(Bluegaussian,Bluecanny,30,90,3);  

                cv::Canny(Yellowgaussian,Yellowcanny,30,90,3); 
                
                cv::Canny(Redgaussian,Redcanny,30,90,3);
                // find and draw contours
                cv::findContours(Bluecanny.clone(), Bluecontours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                BlueContour_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
                cv::drawContours(BlueContour_img, Bluecontours, -1, cv::Scalar(0,0,128)); //must be diff. color 

                cv::findContours(Yellowcanny.clone(), Yellowcontours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                YellowContour_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
                cv::drawContours(YellowContour_img, Yellowcontours, -1, cv::Scalar(255,255,0)); //must be diff. color 

                cv::findContours(Redcanny.clone(), Redcontours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                RedContour_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
                cv::drawContours(RedContour_img, Redcontours, -1, cv::Scalar(0,255,0)); //must be diff. color 
                //find and draw convex hulls
                std::vector<std::vector<cv::Point> > BlueConvexHulls(Bluecontours.size());
                for (unsigned int i = 0; i < Bluecontours.size(); i++) {
                    cv::convexHull(Bluecontours[i], BlueConvexHulls[i]);
                }
                BlueConvexHulls_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
                cv::drawContours(BlueConvexHulls_img, BlueConvexHulls, -1, cv::Scalar(0,0,128));

                std::vector<std::vector<cv::Point> > YellowConvexHulls(Yellowcontours.size());

                for (unsigned int i = 0; i < Yellowcontours.size(); i++) {
                    cv::convexHull(Yellowcontours[i], YellowConvexHulls[i]);
                }
                YellowConvexHulls_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
                cv::drawContours(YellowConvexHulls_img, YellowConvexHulls, -1, cv::Scalar(255,255,0));
                
                std::vector<std::vector<cv::Point> > RedConvexHulls(Redcontours.size());

                for (unsigned int i = 0; i < Redcontours.size(); i++) {
                    cv::convexHull(Redcontours[i], RedConvexHulls[i]);
                }
                RedConvexHulls_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
                cv::drawContours(RedConvexHulls_img, RedConvexHulls, -1, cv::Scalar(0,255,0));

      //elimnate not-cone objects *****************************************************************************
                for (auto &BlueconvexHull_each : BlueConvexHulls) {
                   if (isTrafficCone(BlueconvexHull_each)) {
                      blueConesConvex.push_back(BlueconvexHull_each);
                   }
                }
               blueCones_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
               cv::drawContours(blueCones_img, blueConesConvex, -1, cv::Scalar(0,0,128));

                for (auto &YellowconvexHull_each : YellowConvexHulls) {
                   if (isTrafficCone(YellowconvexHull_each)) {
                      YellowConesConvex.push_back(YellowconvexHull_each);
                   }
                }
               YellowCones_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
               cv::drawContours(YellowCones_img, YellowConesConvex, -1, cv::Scalar(255,255,0));

                for (auto &RedconvexHull_each : RedConvexHulls) {
                   if (isTrafficCone(RedconvexHull_each)) {
                      RedConesConvex.push_back(RedconvexHull_each);
                   }
                }
               RedCones_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
               cv::drawContours(RedCones_img, RedConesConvex, -1, cv::Scalar(0,255,0));
     //Find the coordinate of cones and Draw a rectangle on each dectected cone**********************************
                for (unsigned int i = 0; i < blueConesConvex.size(); i++){
                     auto blueConesConvex_each = blueConesConvex[i];
                     cv::Moments bluemoments = cv::moments(blueConesConvex_each);
                     int xCenter = (int)(bluemoments.m10 / bluemoments.m00);
                     int yCenter = (int)(bluemoments.m01 / bluemoments.m00);
                    //change cordinate to kiwi car
                     int xi = xCenter;
                     int yi = yCenter+400;
                     cv::Point blueCones_coordiatei(xi,yi);
                     blueCones_coordiate.push_back(blueCones_coordiatei); 
                     cv::rectangle(imgWithCones, cv::Point(xCenter-25, yCenter+400-50),cv::Point(xCenter+25, yCenter+400+40), cv::Scalar(255,255,0),2);
                }

 
                for (unsigned int i = 0; i < YellowConesConvex.size(); i++){
                     auto YellowConesConvex_each=YellowConesConvex[i];
                     cv::Moments Yellowmoments = cv::moments(YellowConesConvex_each);
                     int xCenter = (int)(Yellowmoments.m10 / Yellowmoments.m00);
                     int yCenter = (int)(Yellowmoments.m01 / Yellowmoments.m00);                     
                     int xi = xCenter;
                     int yi = yCenter+400;
                     cv::Point YellowCones_coordiatei(xi,yi);
                     YellowCones_coordiate.push_back(YellowCones_coordiatei);
                     cv::rectangle(imgWithCones, cv::Point(xCenter-25, yCenter+400-50),cv::Point(xCenter+25, yCenter+400+40), cv::Scalar(255,255,255),2);
                }
                     

                for (unsigned int i = 0; i < RedConesConvex.size(); i++){
                     auto RedConesConvex_each=RedConesConvex[i];
                     cv::Moments Redmoments = cv::moments(RedConesConvex_each);
                     int xCenter = (int)(Redmoments.m10 / Redmoments.m00);
                     int yCenter = (int)(Redmoments.m01 / Redmoments.m00);                     
                     int xi = xCenter;
                     int yi = yCenter+400;
                     cv::Point RedCones_coordiatei(xi,yi);
                     RedCones_coordiate.push_back(RedCones_coordiatei);
                     cv::rectangle(imgWithCones, cv::Point(xCenter-25, yCenter+400-50),cv::Point(xCenter+25, yCenter+400+40), cv::Scalar(0,0,255),2);
                }
                     
               std::vector<cv::Point> YellowCones_coordiate_copy = YellowCones_coordiate; 
               std::vector<cv::Point> blueCones_coordiate_copy = blueCones_coordiate; 
               
     // fill the corordinate if the dectected cones are not enough*******************************************************
         double Aimx;
         double Aimy;
         //double Aimfarx;
         //double Aimfary;
         std::vector<cv::Point> MiddlePoints; 
         if ((YellowCones_coordiate_copy.size()==0) &(blueCones_coordiate_copy.size()==0)){
               Aimx=640;//640
               Aimy=200;
               //Aimfarx=640;
               //Aimfary=400;
         }else{
               if (YellowCones_coordiate_copy.size()<1&&blueCones_coordiate_copy.size()>=1){
                   for (unsigned int i = 0;i <blueCones_coordiate_copy.size();i++){
                       cv::Point YellowOnEdge(blueCones_coordiate_copy[i].x-500,blueCones_coordiate_copy[i].y);
                      //cv::Point YellowOnEdge(0,blueCones_coordiate_copy[i].y);
                       YellowCones_coordiate_copy.push_back(YellowOnEdge);
                   }
               }

               if (blueCones_coordiate_copy.size()<1&&YellowCones_coordiate_copy.size()>=1){
                   for (unsigned int i = 0;i<YellowCones_coordiate_copy.size();i++){
                       cv::Point blueOnEdge(YellowCones_coordiate_copy[i].x+500,YellowCones_coordiate_copy[i].y);
                       //cv::Point blueOnEdge(1280,YellowCones_coordiate_copy[i].y);
                       blueCones_coordiate_copy.push_back(blueOnEdge);
                   }
               }


              if (YellowCones_coordiate_copy.size()==1){
                  YellowCones_coordiate_copy.push_back(YellowCones_coordiate_copy[0]);
                  YellowCones_coordiate_copy.push_back(YellowCones_coordiate_copy[0]);
                         
              }else if (YellowCones_coordiate_copy.size()==2){
                  YellowCones_coordiate_copy.push_back(YellowCones_coordiate_copy[1]);
              }

              if (blueCones_coordiate_copy.size()==1){
                  blueCones_coordiate_copy.push_back(blueCones_coordiate_copy[0]);
                  blueCones_coordiate_copy.push_back(blueCones_coordiate_copy[0]);
              }else if (blueCones_coordiate_copy.size()==2){
                  blueCones_coordiate_copy.push_back(blueCones_coordiate_copy[1]);
              }
  //compute middle point of each connect line**********************************************************************
             
              for  (unsigned int i = 0; i < 3; i++){
                       //check if the yellow cone is on the left side 
                       if  (YellowCones_coordiate_copy[i].x < blueCones_coordiate_copy[i].x){       
                          cv::line(imgWithCones,  YellowCones_coordiate_copy[i],blueCones_coordiate_copy[i], cv::Scalar(0,255,255), 1);
                
                          double x1 = (YellowCones_coordiate_copy[i].x+blueCones_coordiate_copy[i].x)/2;
                          double y1 = (YellowCones_coordiate_copy[i].y+blueCones_coordiate_copy[i].y)/2;
                          cv::Point MiddlePoint(static_cast<int>(x1),static_cast<int>(y1));
                          MiddlePoints.push_back(MiddlePoint);
                          cv::circle(imgWithCones, MiddlePoint, 1, cv::Scalar(0,0,255), -1);
                       }         
              }
                    
             if (MiddlePoints.size()==3){
                       GetOnTrack = 1;
                  
                       cv::line(imgWithCones,  MiddlePoints[1],MiddlePoints[2], cv::Scalar(0,255,255), 1);
                       cv::line(imgWithCones,  MiddlePoints[1],MiddlePoints[0], cv::Scalar(0,255,255), 1);
   // connect the middle point and compute the aiming point************************************************************

                       if (YellowCones_coordiate_copy[0].x==0){
                            Aimx=(MiddlePoints[0].x+MiddlePoints[1].x)/2;
                           //Aimx=50;
                           double Ay1 = (MiddlePoints[0].y+MiddlePoints[1].y)/2;
                           double Ay2 = (MiddlePoints[1].y+MiddlePoints[2].y)/2;
                           Aimy=(Ay1+Ay2)/2;
                      }else if (blueCones_coordiate_copy[0].x==12800){
                           Aimx=(MiddlePoints[0].x+MiddlePoints[1].x)/2;
                           //Aimx=1230;
                           double Ay1 = (MiddlePoints[0].y+MiddlePoints[1].y)/2;
                           double Ay2 = (MiddlePoints[1].y+MiddlePoints[2].y)/2;
                           Aimy=(Ay1+Ay2)/2;
                      }else{

                           double Ax1 = (MiddlePoints[0].x+MiddlePoints[1].x)/2;
                           double Ay1 = (MiddlePoints[0].y+MiddlePoints[1].y)/2;
                           double Ax2 = (MiddlePoints[1].x+MiddlePoints[2].x)/2;
                           double Ay2 = (MiddlePoints[1].y+MiddlePoints[2].y)/2;
                           cv::line(imgWithCones, cv::Point(static_cast<int>(Ax1), static_cast<int>(Ay1)),cv::Point(static_cast<int>(Ax2), static_cast<int>(Ay2)), cv::Scalar(0,255,255), 1);
                           Aimx=Ax1;
                           Aimy=Ay1;
                           //Aimfarx=Ax2;
                           //Aimfary=Ay2;
                       }
            }else if (MiddlePoints.size()==2){
                  cv::line(imgWithCones,  MiddlePoints[1],MiddlePoints[0], cv::Scalar(0,255,255), 1);
                       if (YellowCones_coordiate_copy[0].x==0){
                           Aimx=(MiddlePoints[0].x+MiddlePoints[1].x)/2;
                           //Aimx=50;
                           Aimy = (MiddlePoints[0].y+MiddlePoints[1].y)/2;
                      }else if (blueCones_coordiate_copy[0].x==12800){
                           Aimx=(MiddlePoints[0].x+MiddlePoints[1].x)/2;
                           //Aimx=1230;
                           Aimy= (MiddlePoints[0].y+MiddlePoints[1].y)/2;
                      }else{
                           Aimx = (MiddlePoints[0].x+MiddlePoints[1].x)/2;
                           Aimy = (MiddlePoints[0].y+MiddlePoints[1].y)/2;
                      } 


            }else if(MiddlePoints.size()==1){
                       Aimx=MiddlePoints[0].x;
                       Aimy=MiddlePoints[0].y;
            }else{
                       Aimx=640;//640
                       Aimy=200;               
            }

                       
            }
       //Calculate the ground steering angle      
            cv::Point AimPoint(static_cast<int>(Aimx),static_cast<int>(Aimy));
            cv::circle(imgWithCones, AimPoint, 8, cv::Scalar(0,255,0), -1);
            
            double imageAngleRad=atan((640-Aimx)/(720-Aimy)); 
            //double imageAngleDeg=imageAngleRad/3.14*180; 
            double ScaleFactor=0.15;
            double ScaledAngle;
            if (GetOnTrack == 1){
               ScaledAngle=imageAngleRad*ScaleFactor;
            }else{
               ScaledAngle=0.6;
            }
            //std::cout <<"ScaledAngle= "<< ScaledAngle <<std::endl;


                //kiwi detection
                cv::Mat img_gray;
                cv::cvtColor(img.rowRange(300, 650),img_gray, cv::COLOR_BGR2GRAY);
                int area = 0;
                std::vector<cv::Rect> kiwi;
		kiwi_detector.detectMultiScale(img_gray, kiwi, 1.1, 3, 0,cv::Size(72,72), cv::Size(500, 300));
		for (size_t i = 0; i < kiwi.size(); i++)
		{
                        cv::Point center(kiwi[i].x + kiwi[i].width / 2,kiwi[i].y + kiwi[i].height / 2 + 300);
			cv::rectangle(imgWithCones,cv::Point(kiwi[i].x, kiwi[i].y + 300), cv::Point(kiwi[i].x + kiwi[i].width, kiwi[i].y + kiwi[i].height+ 300),cv::Scalar(0, 0, 255), 1, 8, 0);
                        int detarea = kiwi[i].width * kiwi[i].height;
                        if (area < detarea){
                               area = detarea;
                        }
                        
		}
		//cv::imshow("detect result", imgWithCones);

  // Pedel Position control based on Kiwi detection.
           float pedalPositionFactor=1.0f;
                if (area >= 19000 && area < 57600){
                        //pedalPositionFactor=0.7f;
                        pedalPositionFactor=((19000.0f-static_cast<float>(area))/38600.0f)+1.0f;
                        }
                else if (area >= 57600){
                        pedalPositionFactor=0.0f;
                        }
                else{
                        if (ScaledAngle>0.08||ScaledAngle<-0.08){
                        pedalPositionFactor=0.8f;
                        }else{
                        pedalPositionFactor=1.0f;
                        }
                    }

            float pedalPositioninit=0.3f;
            float pedalPosition=pedalPositioninit * pedalPositionFactor;
                 

                // Display image.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    //cv::imshow("erode", Blueerode);
                    //cv::imshow("gaussian", Bluegaussian);
                    //cv::imshow("bluecanny", Bluecanny);
                    //cv::imshow("yellowcanny", Yellowcanny);
                    //cv::imshow("Contours", BlueContour_img);
                    //cv::imshow("ConvexHulls", BlueConvexHulls_img);
                    //cv::imshow("blueCones_img", blueCones_img);
                    cv::imshow("imgWithCones", imgWithCones);
	//cv::imshow("image3", image3);
                    cv::waitKey(1);
                }

                ////////////////////////////////////////////////////////////////
                // Do something with the distance readings if wanted.
//                {

//                    std::lock_guard<std::mutex> lck(distancesMutex);
//                    if (VERBOSE){
//                    std::cout << "front = " << front << ", "
//                              << "rear = " << rear << ", "
//                              << "left = " << left << ", "
//                              << "right = " << right << "." << std::endl;
//                    }
//                }

                ////////////////////////////////////////////////////////////////
                // Example for creating and sending a message to other microservices; can
                // be removed when not needed.

         opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest;
         groundSteeringAngleRequest.groundSteering(ScaledAngle);
         opendlv::proxy::PedalPositionRequest pedalPositionRequest;
         pedalPositionRequest.position(pedalPosition);


        cluon::data::TimeStamp sampleTime = cluon::time::now();
        od4.send(groundSteeringAngleRequest, sampleTime, 0);
        od4.send(pedalPositionRequest, sampleTime, 0);
        if (VERBOSE) {
          std::cout << "Ground steering angle is " << groundSteeringAngleRequest.groundSteering()
            << " and pedal position is " << pedalPositionRequest.position() << std::endl;
        }
//                opendlv::proxy::AngleReading ar;
//                ar.angle(123.45f);
//                od4.send(ar);

                ////////////////////////////////////////////////////////////////
                // Steering and acceleration/decelration.
                //
                // Uncomment the following lines to steer; range: +38deg (left) .. -38deg (right).
                // Value groundSteeringRequest.groundSteering must be given in radians (DEG/180. * PI).
                //opendlv::proxy::GroundSteeringRequest gsr;
                //gsr.groundSteering(0);
                //od4.send(gsr);

                // Uncomment the following lines to accelerate/decelerate; range: +0.25 (forward) .. -1.0 (backwards).
                // Be careful!
                //opendlv::proxy::PedalPositionRequest ppr;
                //ppr.position(0);
                //od4.send(ppr);
              
        }

      }
        
        retCode = 0;
    }
    return retCode;
}
