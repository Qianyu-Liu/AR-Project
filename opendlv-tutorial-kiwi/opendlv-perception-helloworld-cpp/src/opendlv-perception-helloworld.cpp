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


bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A)
{
      //Number of key points
      int N = key_point.size();
 
      //构造矩阵X
      cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
      for (int i = 0; i < n + 1; i++)
      {
            for (int j = 0; j < n + 1; j++)
            {
                  for (int k = 0; k < N; k++)
                  {
                        X.at<double>(i, j) = X.at<double>(i, j) + std::pow(key_point[k].y, i + j);
                  }
            }
      }
 
      //构造矩阵Y
      cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
      for (int i = 0; i < n + 1; i++)
      {
            for (int k = 0; k < N; k++)
            {
                  Y.at<double>(i, 0) = Y.at<double>(i, 0) + std::pow(key_point[k].y, i) * key_point[k].x;
            }
      }
 
      A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
      //求解矩阵A
      cv::solve(X, Y, A, cv::DECOMP_LU);
      return true;
}

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
            std::mutex distancesMutex;
            float front{0};
            float rear{0};
            float left{0};
            float right{0};
            auto onDistance = [&distancesMutex, &front, &rear, &left, &right](cluon::data::Envelope &&env){
                auto senderStamp = env.senderStamp();
                // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
                opendlv::proxy::DistanceReading dr = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));

                // Store distance readings.
                std::lock_guard<std::mutex> lck(distancesMutex);
                switch (senderStamp) {
                    case 0: front = dr.distance(); break;
                    case 2: rear = dr.distance(); break;
                    case 1: left = dr.distance(); break;
                    case 3: right = dr.distance(); break;
                }
            };
            // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
            od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistance);


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
                cv::Mat imgWithCones;
                imgWithCones = img.clone();

                std::vector<std::vector<cv::Point> > Yellowcontours;
                std::vector<std::vector<cv::Point> > YellowConesConvex;
                

                cv::cvtColor(img,hsv,cv::COLOR_BGR2HSV);

                cv::Scalar hsvBlueLow(100,70,40);
                cv::Scalar hsvBlueHi(124,255,255);
                cv::inRange(hsv,hsvBlueLow,hsvBlueHi,BlueColourThreshod);

                cv::Scalar hsvYellowLow(11,60,100);
                cv::Scalar hsvYellowHi(34,255,255);
                cv::inRange(hsv,hsvYellowLow,hsvYellowHi,YellowColourThreshod);

                //Ones.copyTo(BlueColourThreshod.rowRange(0, 319));
                //Ones.row(0, 69).copyTo(BlueColourThreshod.rowRange(650, 719));
                //Dilate
                uint32_t iterations{12};
                cv::dilate(BlueColourThreshod.rowRange(400, 650),Bluedilate,cv::Mat(),cv::Point(-1,1),iterations,1,1);

                cv::dilate(YellowColourThreshod.rowRange(400, 650),Yellowdilate,cv::Mat(),cv::Point(-1,1),iterations,1,1);

                //Erode
                cv::erode(Bluedilate,Blueerode,cv::Mat(),cv::Point(-1,1),iterations,1,1);

                cv::erode(Yellowdilate,Yellowerode,cv::Mat(),cv::Point(-1,1),iterations,1,1);

                //Apply Gaussian filter
                cv::GaussianBlur(Blueerode, Bluegaussian, cv::Size(5, 5), 0);

                cv::GaussianBlur(Yellowerode, Yellowgaussian, cv::Size(5, 5), 0);

                //Edge dectection
                cv::Canny(Bluegaussian,Bluecanny,30,90,3);  

                cv::Canny(Yellowgaussian,Yellowcanny,30,90,3); 
                

                // find and draw contours
                cv::findContours(Bluecanny.clone(), Bluecontours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                BlueContour_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
                cv::drawContours(BlueContour_img, Bluecontours, -1, cv::Scalar(0,0,128)); //must be diff. color 

                cv::findContours(Yellowcanny.clone(), Yellowcontours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                YellowContour_img = cv::Mat(img.size(), CV_8UC3, cv::Scalar(0, 0, 0));
                cv::drawContours(YellowContour_img, Yellowcontours, -1, cv::Scalar(255,255,0)); //must be diff. color 

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
                


                //elimnate not-cone objects 
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


                //Draw a rectangle on each dectect blue cone
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
                     //std::cout << "xi=" << xi <<"yi" << yi << std::endl;
                     cv::rectangle(imgWithCones, cv::Point(xCenter-25, yCenter+400-50),cv::Point(xCenter+25, yCenter+400+40), cv::Scalar(255,255,0),2);
                }
                //cv::Point right_init(1280,700);
                //blueCones_coordiate.push_back(right_init); 

                for (unsigned int i = 0; i < YellowConesConvex.size(); i++){
                     auto YellowConesConvex_each=YellowConesConvex[i];
                     cv::Moments Yellowmoments = cv::moments(YellowConesConvex_each);
                     int xCenter = (int)(Yellowmoments.m10 / Yellowmoments.m00);
                     int yCenter = (int)(Yellowmoments.m01 / Yellowmoments.m00);                     
                //change cordinate to kiwi car
                     int xi = xCenter;
                     int yi = yCenter+400;
                     cv::Point YellowCones_coordiatei(xi,yi);
                     YellowCones_coordiate.push_back(YellowCones_coordiatei);
                     cv::rectangle(imgWithCones, cv::Point(xCenter-25, yCenter+400-50),cv::Point(xCenter+25, yCenter+400+40), cv::Scalar(255,255,255),2);
                }
                //cv::Point left_init(0,700);
                //YellowCones_coordiate.push_back(left_init); 

       //cv::Mat image = cv::Mat::zeros(720, 1280, CV_8UC3);
	//image.setTo(cv::Scalar(100, 0, 0));
 

 
//将拟合点绘制到空白图上  

	//cv::circle(image, blueCones_coordiate, 5, cv::Scalar(0, 0, 255), 2, 8, 0);
	
 
//绘制折线
                //cv::polylines(imgWithCones, blueCones_coordiate, false, cv::Scalar(255, 255, 0), 1, 8, 0);
                cv::Mat A;
                polynomial_curve_fit(blueCones_coordiate, 2, A);
                std::vector<cv::Point> blue_points_fitted;

                for (int y = 0; y < 720; y++)
                {
                    double x = A.at<double>(0, 0) + A.at<double>(1, 0) * y + A.at<double>(2, 0)*std::pow(y, 2) + A.at<double>(3, 0)*std::pow(y, 3);
                    blue_points_fitted.push_back(cv::Point(static_cast<int>(x), y ));
                }
                cv::polylines(imgWithCones, blue_points_fitted, false, cv::Scalar(255, 255, 0), 1, 8, 0);



                //cv::polylines(imgWithCones, YellowCones_coordiate, false, cv::Scalar(255, 255, 255), 1, 8, 0);
                cv::Mat B;
                polynomial_curve_fit(YellowCones_coordiate, 2, B);
                std::vector<cv::Point> yellow_points_fitted;

                for (int y = 0; y < 720; y++)
                {
                    double x = B.at<double>(0, 0) + B.at<double>(1, 0) * y + B.at<double>(2, 0)*std::pow(y, 2) + B.at<double>(3, 0)*std::pow(y, 3);
                    yellow_points_fitted.push_back(cv::Point(static_cast<int>(x), y));
                }
                cv::polylines(imgWithCones, yellow_points_fitted, false, cv::Scalar(255, 255, 255), 1, 8, 0);
                

 




                // Display image.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    //cv::imshow("erode", Blueerode);
                    cv::imshow("gaussian", Bluegaussian);
                    cv::imshow("bluecanny", Bluecanny);
                    cv::imshow("yellowcanny", Yellowcanny);
                    //cv::imshow("Contours", BlueContour_img);
                    //cv::imshow("ConvexHulls", BlueConvexHulls_img);
                    cv::imshow("blueCones_img", blueCones_img);
                    cv::imshow("imgWithCones", imgWithCones);
	//cv::imshow("image", image);
                    cv::waitKey(1);
                }

                ////////////////////////////////////////////////////////////////
                // Do something with the distance readings if wanted.
                {

                    std::lock_guard<std::mutex> lck(distancesMutex);
                    std::cout << "front = " << front << ", "
                              << "rear = " << rear << ", "
                              << "left = " << left << ", "
                              << "right = " << right << "." << std::endl;
                }

                ////////////////////////////////////////////////////////////////
                // Example for creating and sending a message to other microservices; can
                // be removed when not needed.
                opendlv::proxy::AngleReading ar;
                ar.angle(123.45f);
                od4.send(ar);

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
