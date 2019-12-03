/**
	g++ $(pkg-config --cflags --libs /usr/local/Cellar/opencv/4.1.1_2/lib/pkgconfig/opencv4.pc) -std=c++11  local.cpp -o local
**/
#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <list>
#include <array>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include<ros/ros.h>
#include "std_msgs/String.h"
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include<stdio.h>

using namespace std;
using namespace cv; 

static void help()
{
    cout
        << "------------------------------------------------------------------------------" << endl
        << "This program shows how to read a video file with OpenCV."                       << endl
        << "Usage:"                                                                         << endl
        << "./local <video>"                                                                << endl
        << "--------------------------------------------------------------------------"     << endl
        << endl;
}

static Mat unwarp(Mat input,Point2f *inputQuad)
{
	Mat output;

	// Input Quadilateral or Image plane coordinates
    // Output Quadilateral or World plane coordinates
    Point2f outputQuad[4];

    // Lambda Matrix
    Mat lambda( 2, 4, CV_32FC1 );

    // Set the lambda matrix the same type and size as input
    lambda = Mat::zeros( input.rows, input.cols, input.type() );

    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input


    // inputQuad[0] = Point2f( 113, 505);
    // inputQuad[1] = Point2f( 795, 175);
    // inputQuad[2] = Point2f(1515, 315);
    // inputQuad[3] = Point2f(1210, 1040);  



	cout << inputQuad[0];
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    outputQuad[0] = Point2f( 0,0 );
    outputQuad[1] = Point2f( input.rows-1, 0);
    outputQuad[2] = Point2f( input.rows-1, input.rows-1);
    outputQuad[3] = Point2f( 0, input.rows-1);

	// Get the Perspective Transform Matrix i.e. lambda 
    lambda = getPerspectiveTransform( inputQuad, outputQuad );
    // Apply the Perspective Transform just found to the src image
    warpPerspective(input,output,lambda,output.size() );

    /*
	//Display input and output
    imshow("Input", input);
    imshow("Output", output);
    waitKey(0);
    */

    return output;
}

float dist(Point p1, Point p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)*1.0);
}

struct DistanceFunc
{
    DistanceFunc(const Point& _p) : p(_p) {}

    bool operator()(const Point& lhs, const Point& rhs) const
    {
        return dist(p, lhs) < dist(p, rhs);
    }

private:
    Point p;
};


Point get_closest(Point p, vector<Point> list)
{
	sort(list.begin(), list.end(), DistanceFunc(p));

	return list[0];
}

int main(int argc, char *argv[])
{
    std::string param;
    ros::init(argc, argv, "axis_publisher");

    ros::NodeHandle nh;
    nh.getParam("param", param);
    ROS_INFO("Got parameter : %s", param.c_str());
    ros::Time timeros = ros::Time::now();
    ros::Rate loop_rate(10);
	ros::Publisher axis_pub = nh.advertise<std_msgs::String>("axis", 1000);
    //  // 定义节点句柄   
    // image_transport::ImageTransport it(n);
    // image_transport::Publisher pub = it.advertise("video_image", 1);
    // sensor_msgs::ImagePtr msg;
	// cout<<param<<endl;
	const int N_CARS = atoi(argv[1]);
	vector<vector<Point>> array_vec(N_CARS);
	Scalar color_map[14] = {
		Scalar(0, 0, 0), Scalar(0, 0, 127), Scalar(0, 147, 0), Scalar(255, 0, 0), Scalar(127, 0, 0), Scalar(156, 0, 156), 
		Scalar(252, 127, 0), Scalar(255, 255, 0), Scalar(0, 252, 0), Scalar(0, 147, 147), Scalar(0, 255, 255),
		Scalar(0, 0, 252), Scalar(255, 0, 255), Scalar(147, 147, 147) 
	};

	help();
	VideoCapture cap;
	cap.open(atoi(argv[2]));//atoi(param);
  	if(!cap.isOpened()){
    	cout << "Error opening video stream or file" << endl;
    	return -1;
  	}
	cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
  	int frame_width = int(cap.get(3)); 
  	int frame_height = int(cap.get(4)); 
	cout << "!!!!!!!" <<frame_width << frame_height << endl;
	VideoWriter video("outcpp.avi",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height));
	int time = 0;
  	// Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file. 
  	// VideoWriter video("outcpp.mov", CAP_OPENCV_MJPEG, 10, Size(frame_width,frame_height)); 
  	while(1){

    	Mat frame;    	// Capture frame-by-frame
    	cap >> frame;
		time++;
		Point2f inputQuad[4]; 
		video.write(frame);
		//atoi(param);
		inputQuad[0] = Point2f(206, 253);
		inputQuad[1] = Point2f(648, 110);
		inputQuad[2] = Point2f(1122, 264);
		inputQuad[3] = Point2f(757, 698);

		// inputQuad[0] = Point2f( 123, 346);
    	// inputQuad[1] = Point2f( 554, 146);
    	// inputQuad[2] = Point2f(1016, 227);
    	// inputQuad[3] = Point2f(811, 664); 
      	// Draw circles to find 4 points
 		circle( frame, inputQuad[2], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
 		circle( frame, inputQuad[1], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
 		circle( frame, inputQuad[0], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
 		circle( frame, inputQuad[3], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
		  
    	// If the frame is empty, break immediately
    	if (frame.empty())
      		break;

      	// Calibrate the image
    	frame = unwarp(frame,inputQuad);
    	// imshow( "Frame", frame );

    	// Apply threshold
    	Mat frame_HSV, mask_BGR, result_BGR;
		Mat erode_frame, dilate_frame, gray_frame, b_frame;

    	// Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);

        // Detect the object based on HSV Range Value
    	// inRange(frame_HSV, Scalar(36, 105, 25), Scalar(86, 220, 220), mask_BGR);
		inRange(frame_HSV, Scalar(34, 105, 25), Scalar(55, 220, 220), mask_BGR);

    	bitwise_and(frame_HSV, frame_HSV, result_BGR, mask_BGR);

		Mat kernel = getStructuringElement( MORPH_RECT,
                       Size( 2*1 + 1, 2*1+1 ),
                       Point( 1, 1 ) );

		erode(result_BGR, erode_frame, kernel);
		dilate(erode_frame, dilate_frame, kernel);
		cvtColor(dilate_frame, gray_frame, COLOR_BGR2GRAY);
		threshold(gray_frame, b_frame, 0, 255.0, THRESH_BINARY);

        Mat locations;// output, locations of non-zero pixels
        findNonZero(b_frame, locations);

        Mat locations_mat(locations.rows, 2, CV_32F);
          for( int i = 0; i < locations.rows; i++ ){
              locations_mat.at<float>(i, 0) = locations.at<Point>(i).x;
              locations_mat.at<float>(i, 1) = locations.at<Point>(i).y;
          }

        int clusterCount = N_CARS;
        Mat labels;
        int attempts = 5;

        Mat centers;
		// cout << "before kmeans(1)"<< endl;
		locations_mat.convertTo(locations_mat, CV_32F);
		// cout << locations_mat.size();
		try{	
		kmeans(locations_mat, clusterCount, labels, TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 10000, 0.0001), attempts, KMEANS_PP_CENTERS, centers);
      	vector<Point> vector_centers;
		//   cout << centers.rows;
      	for(int i = 0; i < centers.rows; i++){
            const float* Mi = centers.ptr<float>(i);
			// cout<< "yikaishi" << i << "---" << Mi[0] << "!"<< Mi[1];
            Point center = Point(Mi[0], Mi[1]);
            vector_centers.push_back(center);
			// circle(frame, vector_centers.back(), 20, color_map[i], 5);
        }
		if(array_vec[0].size() == 0){
			for(int i = 0; i < centers.rows; i++){
			const float* Mi_ = centers.ptr<float>(i);
			Point center = Point(Mi_[0], Mi_[1]);
			// cout<< "!!!" << i << "---" << Mi_[0] << "!"<< Mi_[1]<<endl;
			array_vec[i].push_back(center);
			}
      	}
		else{
			// cout<<"ROWS!!!!!!!!!!"<< centers.rows<< endl;
			for(int i = 0; i < centers.rows; i++){
				const float* Mi_ = centers.ptr<float>(i);
				// Point center = Point(Mi_[0], Mi_[1]);
				// cout<<"vector_centers"<<array_vec[i]<<endl;
				Point closet = get_closest(array_vec[i].back(), vector_centers);
				vector_centers.erase(std::find(vector_centers.begin(),vector_centers.end(),closet));
				array_vec[i].push_back(closet);
				circle(frame, array_vec[i].back(), 20, color_map[i], 5);
				std_msgs::String msg;
				std::stringstream ss;
				ss << i<<"at,X" <<  array_vec[i].back().x <<"Y"<<array_vec[i].back().y;
				// cout << ss<<endl;
				msg.data = ss.str();
				ROS_INFO("%s", msg.data.c_str());

				axis_pub.publish(msg);
				ros::spinOnce();
			}
		}
		}
		catch(...){

		}
    	// imshow( "Frame", frame );
    	// imshow("New Frame", b_frame);
    	char c=(char)waitKey(25);
    	if(c==27)
      		break;
  	}
  
  	// When everything done, release the video capture object
  	cap.release();
  	// video.release();
  	// Closes all the frames
  	destroyAllWindows();   
	return 0;
}
