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

    //_nh.getParam("C_scene", _C_scene);
    //ROS_INFO("Scene scale: %d", _C_scene);

    _nh.getParam("desired_rate", _desiredRate);
    ROS_INFO("Desired Rate: %f s", 1/_desiredRate);

    uint64_t stamp_ns;
    std::ifstream infile (timestamp_file_name);
    while (infile >> stamp_ns){
       timestamps.push_back(stamp_ns);
    }

    _capPtr.reset(new cv::VideoCapture(video_name.c_str()));
    
    _pub = _it.advertise("/image_raw",10);
    _info_pub = _nh.advertise<sensor_msgs::CameraInfo>("camera_info", 10);

    _info_sub = _nh.subscribe("/camera_info", 10, &republishImages::cb_camera, this);

    ROS_INFO("Finished reading timestamp file");
  };


  // Publishes an image every time it receives a camera info message
  void cb_camera(const sensor_msgs::CameraInfo::ConstPtr& msg) {
     if ( (msg->header.stamp.toSec() - lastTimestamp) < (1/(_desiredRate+1)) ){
	     //ROS_INFO("Skipping frame");
	     return;
     }

     //ROS_INFO("Publishing Image");
     uint64_t camera_info_timestamp = ((uint64_t)(msg->header.stamp.sec*1e9) + (uint64_t)msg->header.stamp.nsec); 
     if (!std::binary_search(timestamps.begin(), timestamps.end(), camera_info_timestamp))
	     return;
     while (timestamps.at(_counter) != camera_info_timestamp) {
       *_capPtr >> _frame;
       _counter ++;
       if (_counter >= timestamps.size())
	   throw std::runtime_error("Requested a timestamp not in the timestamps file");

     }
     //ROS_INFO("Found matching image");
     // Publish the image
     if(!_frame.empty()) { 
       // If is depth image, convert pixels.
       cv::Mat _floatDepthImage = cv::Mat(_frame.rows, _frame.cols, CV_32FC1);
       float cx = msg->K[2];
       float cy = msg->K[5];
       float fx = msg->K[0];
       
       // Iterate through pixels and remap them
      for(int i=0; i<_frame.rows; i++) {
        for(int j=0; j<_frame.cols; j++) { 
        float z_compressed = _frame.at<uint8_t>(i,j,0)/(float)255;

	// This is 0 at camera, 1 at z-far.
	float eye_depth = ((std::pow(z_compressed, 4)*std::pow(_Z_far, 2)) + (_Z_far * _Z_near) + std::pow(_Z_near, 2)) / (_Z_far*(_Z_far+_Z_near));
	//float eye_depth = std::pow(z_compressed,4);
	// This depth is not the plane to plane depth, but is the depth along the ray.
        float xy_dist_from_center = std::pow(std::pow(i-cy, 2)+std::pow(j-cx,2),0.5);
        float theta = std::atan2(xy_dist_from_center, fx);
	
	float ray_depth = (_Z_far/std::cos(theta))*eye_depth;

	//float true_depth = eye_depth*(_Z_far-_Z_near)+_Z_near;
	// Convert to plane to plane
	float plane_dist = ray_depth*std::cos(theta);

        // You can now access the pixel value with cv::Vec3b
        _floatDepthImage.at<float>(i,j) = plane_dist; //(_Z_near + std::pow(z_compressed, 4)*std::pow(_Z_far, 2)/(std::pow(255.0, 4)*(_Z_near + _Z_far)))*_C_scene;

	}
      }

       _msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", _floatDepthImage).toImageMsg();

       // Copy header
       _msg->header = msg->header;
       // Fix frame
       _msg->header.frame_id = "uav/camera/left";

       // Create camera info
       sensor_msgs::CameraInfo cameraInfoMsg = *msg;
       cameraInfoMsg.header.frame_id = _msg->header.frame_id;


       _pub.publish(_msg);
       _info_pub.publish(cameraInfoMsg);
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
  double _C_scene = 1;

  // Near and far planes of camera
  double _Z_near = 0.01;
  double _Z_far  = 1000.0;

  // Message Ptrs to populate the image messages
  sensor_msgs::ImagePtr _msg;

  // Video Capture ptr to grab the video streams
  std::unique_ptr<cv::VideoCapture> _capPtr;
  
  // Need an image transport ptr to publish images
  image_transport::ImageTransport _it;
  image_transport::Publisher _pub;

  // Subscribers for camera info subscribers
  ros::Subscriber _info_sub;
  ros::Publisher _info_pub;

  // Vector of all the timestamps
  std::vector<uint64_t> timestamps;
  int _counter = 0;

  // Rate subsampling
  double _desiredRate = 360;
  double lastTimestamp = 0;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  republishImages _republishImages(nh, it);

  ros::spin();
}

