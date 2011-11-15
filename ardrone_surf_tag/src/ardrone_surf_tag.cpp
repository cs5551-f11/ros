#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

static const char* TagFile = "images/clubs-3.jpg";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  IplImage* image;

public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("out", 1);
    image_sub_ = it_.subscribe("gscam/image_raw", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    IplImage* frame =  &((IplImage)cv_ptr->image);
    Mat img_object = imread( TagFile, CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_scene(frame); // = imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );
  
  //if( !img_object.data || !img_scene.data )
  //{ std::cout<< " --(!) Error reading images " << std::endl; return -1; }

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 300;

  SurfFeatureDetector detector( minHessian, 5, 5 );

  std::vector<KeyPoint> keypoints_object, keypoints_scene;

  detector.detect( img_object, keypoints_object );
  detector.detect( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;

  Mat descriptors_object, descriptors_scene;

  extractor.compute( img_object, keypoints_object, descriptors_object );
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  BruteForceMatcher< L2<float> > matcher;
  vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

Mat img_matches;
vector< DMatch > good_matches;

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }


  //printf("-- Max dist : %f \n", max_dist );
  //printf("-- Min dist : %f \n", min_dist );
  
  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  

  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
    { good_matches.push_back( matches[i]); }
  }  
  
  //good_matches = matches;
  

  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, 
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), 
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS ); 

//-- Localize the object from img_1 in img_2 
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt ); 
  }
    Mat objPtsMat(obj);
    Mat scenePtsMat(scene);
if( good_matches.size() >= 4 )
{
  Mat H = findHomography( objPtsMat, scenePtsMat, CV_RANSAC, 5 );

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  Mat objCornersMat(obj_corners);
  Mat sceneCornersMat(scene_corners);

  perspectiveTransform( objCornersMat, sceneCornersMat, H);
  
  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  line( img_scene, scene_corners[0], scene_corners[1] , Scalar(0, 255, 0), 4 );
  line( img_scene, scene_corners[1], scene_corners[2] , Scalar( 0, 255, 0), 4 );
  line( img_scene, scene_corners[2], scene_corners[3] , Scalar( 0, 255, 0), 4 );
  line( img_scene, scene_corners[3], scene_corners[0] , Scalar( 0, 255, 0), 4 );

  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
}
/*
// Read input images

    IplImage* frame =  &((IplImage)cv_ptr->image);
    Mat object = imread( TagFile, CV_LOAD_IMAGE_GRAYSCALE );
    Mat scene(frame); //CvMat(frame); //cvCreateMat( frame->width, frame->height, CV_8UC1 );
    //cvCvtColor(frame, image, CV_BGR2GRAY);

    SurfFeatureDetector detector(1000,1);
    vector<KeyPoint> sceneKeypoints, objectKeypoints;
    detector.detect(scene, sceneKeypoints);
    detector.detect(object, objectKeypoints);

    SurfDescriptorExtractor extractor;
    Mat sceneDescriptors, objectDescriptors;
    extractor.compute( scene, sceneKeypoints, sceneDescriptors );
    extractor.compute( object, objectKeypoints, objectDescriptors );

    FlannBasedMatcher matcher;
    vector<DMatch> matches;
    matcher.match( sceneDescriptors, sceneDescriptors, matches);

    Mat imageMatches;
    drawMatches( object, objectKeypoints, scene, sceneKeypoints, 
                 matches, imageMatches, Scalar::all(-1), Scalar::all(-1), 
                 vector<char>(), DrawMatchesFlags::DEFAULT ); 
*/
            
    imshow("scene", img_scene);
    imshow("matches", img_matches);

    waitKey(3);

    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

