
#ifndef TEL_V2
#define TEL_V2


// Includes

#include <ros/ros.h>
#include <rospack/rospack.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

// image processing
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>

// Message types
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <telepresence/Click.h>



namespace telepresence
{
	class Telepresence {

	  private:
		/* fields */
		// ros
		ros::NodeHandle nh;
		geometry_msgs::TransformStamped camPose;
		geometry_msgs::TransformStamped goalPose;
		tf::TransformListener tfListener;
		tf::TransformBroadcaster br;

		image_transport::ImageTransport imageTransport;
		sensor_msgs::CameraInfo camInfo;
		image_transport::CameraSubscriber cameraSub;
		image_transport::Publisher imagePub;
		image_geometry::PinholeCameraModel camModel;


		ros::Subscriber cloudSub;
		ros::Subscriber gridSub;
		ros::Subscriber arrivedSub;
		ros::Publisher move_basePub;
		ros::Publisher goalPub;
		ros::Publisher rvizPub;
		visualization_msgs::Marker marker;
		ros::ServiceServer clickSrv;
		// std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> moveBaseAction;

		// big memory-stored objects
		cv_bridge::CvImagePtr inputBridge;
		cv::Mat image, image_undistorted;
		sensor_msgs::PointCloud2 cloud;
		nav_msgs::OccupancyGrid oc_grid;
		grid_map::GridMap grid_map;

		// parameters
		bool dynamic_map, ray_marker, use_pointcloud, fisheye;
		bool show_distance, use_waypoint, use_move_base;
		double ray_step, robot_radius, max_ray_dist, stepback_dist;
		int orig_width, orig_height, out_width, out_height, offset_w, offset_h;
		std::string goal_tf_frame, camera_frame, image_in_topic, camera_info_in_topic, move_base_instance;
		/* At user command, starting from the camera, the node is jumping along the ray 
		 * (ray_step long jumps), and at every step checks if the distance
		 * from the given point of the ray to any of the points from the 
		 * point cloud is less than a half ray_step. If so, the first such point is 
		 * chosen as a goal for the robot. */

		// other
		bool active_goal;


		// callbacks ------------------------------------------------------------

		/* The imageCb function receives the camera image, and if the robot has an
		 * active goal, it draws the goal marker on the image at the  appropriate
		 * position (project3dToPixel). Then it publishes the image again. */
		void imageCb(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr& infoMsg);

		/* The cloudCb function simply receives and stores the pointcloud from rtabmap. */
		void cloudCb(const sensor_msgs::PointCloud2 &msg);

		/* The gridCb function simply receives and stores the occupancy_grid */
		void gridCb(const nav_msgs::OccupancyGrid &msg);

		/* The clickCb function is called when the client clicks a pixel on the image.
		 * This function first calculates the direction (with the camera as an origin)
		 * in which the user 'clicked' (projectPixelTo3dRay). After that, it uses the
		 * pointcloud to calculate the point where the fictional ray would hit the first 
		 * object (the ray starts from the camera, in the given direction). Finally,
		 * it publishes this point as a goal for the robot (move_base and tf). */
		bool clickCb(telepresence::ClickRequest &req,
			telepresence::ClickResponse &res);

	  public:
		Telepresence(ros::NodeHandle);
	};

	std::string precision_2(float);
	void cropImage(cv::Mat& image, const int& o_w, const int& o_h, const int& width, const int& height);
}

#endif
