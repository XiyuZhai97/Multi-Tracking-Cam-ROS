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

#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

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
        << "This program shows how to locate cars with OpenCV."                       << endl
        << "Usage:"                                                                         << endl
        << "./local N_CARS camera_index refresh_rate"                                                                << endl
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
void onMouse(int evt, int x, int y, int flags, void*param){
    if(evt == CV_EVENT_LBUTTONDOWN){
        cv::Point2f* ptPtr = (cv::Point2f*)param;
        ptPtr->x = x;
        ptPtr->y = y;
    }
}

float orientation(Point p1, Point p2, float &v)
{   
    v = dist(p1, p2) ;
    if ((p1.y - p2.y) != 0)
        return ((float)(p1.y- p2.y) / (float)(p1.x - p2.x));
    else
    {
        return INFINITY;
    }
    
}

const int max_value_H = 360/2;
const int max_value = 255;
const String window_detection_name = "Object Detection";
int low_H = 36, low_S = 105, low_V = 25;
int high_H = 86, high_S = 220, high_V = 220;

// inRange(frame_HSV, Scalar(36, 105, 25), Scalar(86, 220, 220), mask_BGR);
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

int main(int argc, char *argv[])
{
    int radius = 1;
    std::string param;
    ros::init(argc, argv, "axis_publisher");

    ros::NodeHandle nh;
    nh.getParam("param", param);
    ROS_INFO("Got parameter : %s", param.c_str());
    ros::Time timeros = ros::Time::now();
    ros::Rate loop_rate(atoi(argv[3]));
	ros::Publisher axis_pub = nh.advertise<std_msgs::String>("axis_position", 1000);
     // 定义节点句柄   
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("video_image_complete", 1);
    sensor_msgs::ImagePtr msgPicture;

    image_transport::Publisher pub_unwrap = it.advertise("video_image_unwrap", 1);
    sensor_msgs::ImagePtr msgPicture_unwrap;
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
  	// Define the codec and create VideoWriter object.The output is stored in 'outcpp.avi' file. 
  	// VideoWriter video("outcpp.mov", CAP_OPENCV_MJPEG, 10, Size(frame_width,frame_height)); 

  	vector<Point2f> point_get(5);
    Vec3b color;
  	int numf = 0;
  	Mat frame_temp;

    Mat hsv;
    namedWindow(window_detection_name);
    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);

  	while(1){
        ++numf;
    	Mat frame;    	// Capture frame-by-frame
    	cap >> frame;

        if(numf == 5){
            frame_temp = frame;
            int i = 0;
            cout<<"??????"<<endl;
            while(1)
            {
                imshow("Temp_frame", frame_temp);

                cv::setMouseCallback("Temp_frame", onMouse, (void*)&point_get[0]);
                circle( frame_temp, point_get[0], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
                imshow("Temp_frame", frame_temp);
                i++;
                char c_=(char)waitKey(25);
                if(c_==27)
                    break;
            }
            i = 0;
            while(1)
            {
                imshow("Temp_frame", frame_temp);
                cv::setMouseCallback("Temp_frame", onMouse, (void*)&point_get[1]);
                circle( frame_temp, point_get[1], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
                imshow("Temp_frame", frame_temp);

                i++;

                char c_=(char)waitKey(25);
                if(c_==27)
                    break;
            }

            i = 0;

            while(1)
            {
                imshow("Temp_frame", frame_temp);
                cv::setMouseCallback("Temp_frame", onMouse, (void*)&point_get[2]);
                circle( frame_temp, point_get[2], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
                imshow("Temp_frame", frame_temp);
                i++;

                char c_=(char)waitKey(25);
                if(c_==27)
                    break;
            }
            i = 0;
            while(1)
            {
                imshow("Temp_frame", frame_temp);
                cv::setMouseCallback("Temp_frame", onMouse, (void*)&point_get[3]);
                circle( frame_temp, point_get[3], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
                imshow("Temp_frame", frame_temp);
                i++;

                char c_=(char)waitKey(25);
                if(c_==27)
                    break;
            }

            // while(1)
            // {
            //     imshow("Temp_frame", frame_temp);
            //     cv::setMouseCallback("Temp_frame", onMouse, (void*)&point_get[4]);
            //     // circle( frame_temp, point_get[4], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
            //     cvtColor(frame_temp, hsv, CV_RGB2HSV);
            //     color = frame_temp.at<Vec3b>(point_get[4]);


            //     cout<<"0"<<static_cast<unsigned>(color[0])<<endl;
            //     cout<<"1"<<static_cast<unsigned>(color[1])<<endl;
            //     cout<<"2"<<static_cast<unsigned>(color[2])<<endl;
            //     circle( frame_temp, point_get[4], 200/32, color, 10, LINE_8 );

            //     imshow("Temp_frame", frame_temp);
            //     i++;

            //     char c_=(char)waitKey(25);
            //     if(c_==27)
            //         break;
            // }


            cout<< "wait" << endl;
        }
		msgPicture_unwrap = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		pub_unwrap.publish(msgPicture_unwrap);

		Point2f inputQuad[4];
		inputQuad[0] = Point2f(206, 253);
		inputQuad[1] = Point2f(648, 110);
		inputQuad[2] = Point2f(1122, 264);
		inputQuad[3] = Point2f(757, 698);
		int maxX,maxY = 0;

		inputQuad[0] = point_get[0];
		inputQuad[1] = point_get[1];
		inputQuad[2] = point_get[2];
		inputQuad[3] = point_get[3];
	cout<<"================================================================================="<<endl;
        // cout<<"1"<<inputQuad[0]<<endl<<"2"<<inputQuad[1]<<endl<<"3"<<inputQuad[2]<<endl<<"4"<<inputQuad[3]<<endl;

        circle( frame, inputQuad[0], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
        circle( frame, inputQuad[1], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
        circle( frame, inputQuad[2], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
        circle( frame, inputQuad[3], 200/32, Scalar( 255, 0, 0 ), 4, LINE_8 );
    	if (frame.empty())
      		break;
      	// Calibrate the image
    	frame = unwarp(frame,inputQuad);
        Mat ROI(frame, Rect(1, 1, 770, 719));
        ROI.copyTo(frame);
    	// imshow( "Frame", frame );
    	// Apply threshold
    	Mat frame_HSV, mask_BGR, result_BGR;
		Mat erode_frame, dilate_frame, gray_frame, b_frame;
    	// Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Value
    	// inRange(frame_HSV, Scalar(36, 105, 25), Scalar(86, 220, 220), mask_BGR);

    	// inRange(frame_HSV, Scalar(35, 34, 0), Scalar(55, 134, 220), mask_BGR);
        
        // stable

	    // inRange(frame_HSV, Scalar(34, 105, 25), Scalar(55, 220, 220), mask_BGR);
        int c_1 = color.val[0];
        int c_2 = color.val[1];
        int c_3 = color.val[2];
        cout<<"1,"<<c_1<<",2,"<<c_2<<",3,"<<c_3;
        // inRange(frame_HSV, Scalar(c_1 - 5, c_2-5, c_3 - 70), Scalar(c_1 + 20, c_2 + 100, c_3 + 100), mask_BGR);
        // inRange(frame_HSV, Scalar(c_1-10, 50, 30), Scalar(c_1+10, 200, 230), mask_BGR);
                inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), mask_BGR);


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

            // now, we have label and location mat. 
            // This is only for one car.
            // b_frame
            // for(int i = 0; i < 20; i++)
            // {
            //     circle(b_frame, Point(200+i,200+i*o), 5, Scalar(255, 0, 0), 5);

            // }

            
    
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
                std::stringstream ss;

				// cout<<"ROWS!!!!!!!!!!"<< centers.rows<< endl;
				for(int i = 0; i < centers.rows; i++){
					const float* Mi_ = centers.ptr<float>(i);
					// Point center = Point(Mi_[0], Mi_[1]);
					// cout<<"vector_centers"<<array_vec[i]<<endl;
					Point closet = get_closest(array_vec[i].back(), vector_centers);
                    float v = 0.0;
                    float o = orientation(array_vec[i].back(), closet, v);
                    float theta = atan(o);
                    // float o = 0.0;
//                    cout<< o << "velocity" <<v<< "theta"<<theta;
					vector_centers.erase(std::find(vector_centers.begin(),vector_centers.end(),closet));
					array_vec[i].push_back(closet);
					circle(frame, array_vec[i].back(), 20, color_map[i], 5);

                    
                    float x_out,y_out,s_pass;
                    x_out =  (244 + array_vec[i].back().x - 300)/304.0 - 1;
                    y_out =  (365 + 320 - array_vec[i].back().y)/323.0 - 1;

                    s_pass = atan2(abs(y_out),x_out) * radius;
                    if (y_out >= 0){;}
                    else{s_pass = 2 * 3.14159265 - s_pass;}

                    s_pass = round(s_pass * 100.0)/100;
                    string s(to_string(s_pass));
                    
                std::stringstream ss_pass;
                    
                    ss_pass << std::setprecision(2)<<s_pass;
                    s = ss_pass.str();
                    putText(frame, s, array_vec[i].back(), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 0), 2);    

					// ss << i+1 <<"," <<  x_out <<","<< y_out <<","<<theta<<","<< v<<";";
					ss << i+1 <<"," <<  x_out <<","<< y_out <<",";

					// cout << ss<<endl;

				}
                std_msgs::String msg;

                msg.data = ss.str();
                ROS_INFO("%s", msg.data.c_str());

                axis_pub.publish(msg);
                ros::spinOnce();
			}
		}
		catch(...){

		}
		msgPicture = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		pub.publish(msgPicture);
            
            //ROS_INFO( "read the %dth frame successfully!", count );
            // loop_rate.sleep();
            ros::spinOnce();
    	// imshow( "Frame", frame );
    	// imshow(window_detection_name, b_frame);
            imshow(window_detection_name, b_frame);
    	char c_=(char)waitKey(25);
    	if(c_==27)
      		break;
  	}
  
  	// When everything done, release the video capture object
  	cap.release();
  	// video.release();
  	// Closes all the frames
  	destroyAllWindows();   
	return 0;
}
