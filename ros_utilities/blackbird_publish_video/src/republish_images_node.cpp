#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>

class republishImages {
public:
  republishImages(ros::NodeHandle nh, image_transport::ImageTransport it): _nh(nh), _it(it) {
    _nh.getParam("video_name", video_name);
    ROS_INFO("Video name: %s", video_name.c_str());

    std::string timestamp_file_name;
    _nh.getParam("timestamp_file_name", timestamp_file_name);
    ROS_INFO("Timestamp file: %s", timestamp_file_name.c_str());

    _nh.getParam("scene_scale", _scene_scale);
    ROS_INFO("Scene scale: %d", _scene_scale);

    _nh.getParam("desired_rate", _desiredRate);
    ROS_INFO("Desired Rate: %f s", 1/_desiredRate);

    uint64_t stamp_us;
    std::ifstream infile (timestamp_file_name);
    while (infile >> stamp_us)
       timestamps.push_back(stamp_us * 1e-6);

    _capPtr.reset(new cv::VideoCapture(video_name.c_str()));
    
    _pub = _it.advertise("/image_raw",1);
    
    _info_sub = _nh.subscribe("/camera_info", 1, &republishImages::cb_camera, this);
  };


  // Publishes an image every time it receives a camera info message
  void cb_camera(const sensor_msgs::CameraInfo::ConstPtr& msg) {
     if ( (msg->header.stamp.toSec() - lastTimestamp) < (1/(_desiredRate+1)) )
        return;
     while (timestamps.at(_counter) != msg->header.stamp.toSec()) {
       *_capPtr >> _frame;
       _counter ++;
       if (_counter > timestamps.size())
	   throw std::runtime_error("Requested a timestamp not in the timestamps file");

     }
     // Publish the image
     if(!_frame.empty()) { 
       // If is depth image, convert pixels.
       cv::Mat 
       cv::Mat _floatDepthImage = cv::Mat(_frame.height, _frame.width, "32FC1");
       
       // Iterate through pixels and remap them
      for(int i=0; i<_frame.rows; i++)
        for(int j=0; j<_frame.cols; j++) 
        double z_compressed = _frame.at(i,j,0);
        // You can now access the pixel value with cv::Vec3b
        _floatDepthImage.at(i,j,0) = (_Z_near + std::pow(z_compressed, 4)*std::pow(_Z_far, 2)/(std::pow(255.0, 4)*(_Z_near + _Z_far)))*_C_scene;


       _msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", _frame).toImageMsg();


       _msg->header.stamp = msg->header.stamp;
       _pub.publish(_msg);
       lastTimestamp = msg->header.stamp.toSec();
     } 
     
  }
  
private:
  // Grab node handle to read the parameters
  ros::NodeHandle _nh;
  
  // Strings to store the video names
  std::string video_name;
  
  // Frame to grab the frame from the camera
  cv::Mat _frame;

  // Scene scale
  double _C_scene = 1.0;

  // Near and far planes of camera
  double _Z_near = 0.01;
  double _Z_far  = 100.0;

  // Message Ptrs to populate the image messages
  sensor_msgs::ImagePtr _msg;

  // Video Capture ptr to grab the video streams
  std::unique_ptr<cv::VideoCapture> _capPtr;
  
  // Need an image transport ptr to publish images
  image_transport::ImageTransport _it;
  image_transport::Publisher _pub;

  // Subscribers for camera info subscribers
  ros::Subscriber _info_sub;

  // Vector of all the timestamps
  std::vector<double> timestamps;
  int _counter = 0;

  // Rate subsampling
  double _desiredRate = 119;
  double lastTimestamp;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  republishImages _republishImages(nh, it);

  ros::spin();
}

