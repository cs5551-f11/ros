#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
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
CvMemStorage* storage = cvCreateMemStorage(0);
int key = 0;
static CvScalar red_color[] ={0,0,255};
CvMat* prevgray = 0, *image = 0, *gray =0;
int firstFrame = gray == 0;
IplImage* frame = &((IplImage)cv_ptr->image);
if(!gray)
{
image = cvCreateMat(frame->height, frame->width, CV_8UC1);
}
//Convert the RGB image obtained from camera into Grayscale
cvCvtColor(frame, image, CV_BGR2GRAY);
//Define sequence for storing surf keypoints and descriptors
CvSeq *imageKeypoints = 0, *imageDescriptors = 0;
int i;

//Extract SURF points by initializing parameters
CvSURFParams params = cvSURFParams(50, 1);
cvExtractSURF( image, 0, &imageKeypoints, &imageDescriptors, storage, params );
printf("Image Descriptors: %d\n", imageDescriptors->total);

//draw the keypoints on the captured frame
for( i = 0; i < imageKeypoints->total; i++ )
{
CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, i );
CvPoint center;
int radius;
center.x = cvRound(r->pt.x);
center.y = cvRound(r->pt.y);
radius = cvRound(r->size*1.2/9.*2);
cv::circle( cv_ptr->image, center, radius, red_color[0], 1, 8, 0 );
}
cv::imshow(WINDOW, cv_ptr->image);
//cvShowImage( "Image", frame );
cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

