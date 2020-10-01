
#include <ros/ros.h>
#include <telepresence/telepresence.h>




using namespace telepresence;

Telepresence::Telepresence(ros::NodeHandle _nh) 
	:nh(_nh), imageTransport(_nh)
{
	// get params
	nh.param<bool>("use_waypoint",                use_waypoint,         true);
	nh.param<bool>("use_move_base",               use_move_base,        true);
	nh.param<bool>("dynamic_map",                 dynamic_map,          true);
	nh.param<bool>("ray_marker",                  ray_marker,           true);
	nh.param<bool>("use_pointcloud",              use_pointcloud,       true);
	nh.param<bool>("show_distance",               show_distance,        true);
	nh.param<bool>("fisheye",                     fisheye,              true);
	nh.param<int>("out_width",                    out_width,            400);
	nh.param<int>("out_height",                   out_height,           350);
	nh.param<double>("robot_radius",              robot_radius,         0.3);
	nh.param<double>("ray_step",                  ray_step,             0.1);
	nh.param<double>("robot_radius",              robot_radius,         0.3);
	nh.param<double>("max_ray_dist",              max_ray_dist,         20);
	nh.param<double>("stepback_dist",             stepback_dist,        0.6);
	nh.param<std::string>("goal_tf_frame",        goal_tf_frame,        "telepresence_goal");
	nh.param<std::string>("camera_frame",         camera_frame,         "camera_rgb_optical_frame");
	nh.param<std::string>("image_in",             image_in_topic,       "/camera/color/sync/image");
	nh.param<std::string>("camera_info_in",       camera_info_in_topic, "/camera/color/sync/camera_info");
	nh.param<std::string>("move_base_instance",   move_base_instance,   "move_base_simple");

	goalPose.header.frame_id = "map";
	goalPose.child_frame_id = goal_tf_frame;
	goalPose.transform.rotation.w = 1;

	if (use_move_base)
		move_basePub = nh.advertise<geometry_msgs::PoseStamped>("/"+move_base_instance+"/goal", 10);


	cameraSub = imageTransport.subscribeCamera(image_in_topic, 1, &Telepresence::imageCb, this);
	cloudSub = nh.subscribe("cloud_map", 5, &Telepresence::cloudCb, this);

	if (!use_pointcloud)
		gridSub  = nh.subscribe("map", 5, &Telepresence::gridCb, this);


	rvizPub  = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    imagePub = imageTransport.advertise("image", 10);
	clickSrv = nh.advertiseService("click", &Telepresence::clickCb, this);
} 



// callbacks
void Telepresence::imageCb(const sensor_msgs::ImageConstPtr& imageMsg, const sensor_msgs::CameraInfoConstPtr& infoMsg) 
{
	try
	{
		inputBridge = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
		image = inputBridge->image;

	} catch (cv_bridge::Exception& ex)
	{
		ROS_ERROR("[telepresence] Failed to convert image:\n%s", ex.what());
	}

	orig_width = infoMsg->width;
	orig_height = infoMsg->height;
	offset_w = (orig_width - out_width)/2;
	offset_h = (orig_height - out_height)/2;
	camModel.fromCameraInfo(infoMsg);

	if (use_move_base)
	{
		// TODO
		// auto mbState = moveBaseAction->getState();
		// if (moveBaseAction->getState() == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED ||
		// 		moveBaseAction->getState() == actionlib::SimpleClientGoalState::StateEnum::LOST)
		// {
		// 	imagePub.publish(inputBridge->toImageMsg());
		// 	return;
		// }
	}

	goalPose.header.stamp = ros::Time::now();
	br.sendTransform(goalPose);

	geometry_msgs::Point pt;
	try
	{
		auto t = ros::Time(0);
		tf::StampedTransform tr;
		tfListener.waitForTransform(camera_frame, goal_tf_frame, 
				t, ros::Duration(0.1));
		tfListener.lookupTransform(camera_frame, goal_tf_frame, t, tr);
		pt.x = tr.getOrigin().x();
		pt.y = tr.getOrigin().y();
		pt.z = tr.getOrigin().z();
	} catch (tf::TransformException& ex)
	{
		ROS_WARN("[telepresence] TF exception:\n%s", ex.what());
		cropImage(inputBridge->image, offset_w, offset_h, out_width, out_height);
		imagePub.publish(inputBridge->toImageMsg());
		return;
	}

	if (pt.z > 0) // if goal is in front of robot
		{
		cv::Point3d pt_cv(pt.x, pt.y, pt.z);
		cv::Point2d uv = camModel.project3dToPixel(pt_cv);

		static const int RADIUS = 15;
		cv::circle(image, uv, RADIUS, CV_RGB(88, 146, 211), 4);

		if (show_distance) 
		{
			float d = cv::norm(cv::Point2d(pt.x, pt.z));
			const char* txt = (precision_2(d) + " m").c_str();

			CvFont font;
			cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
			CvSize text_size;
			int baseline;
			cvGetTextSize(txt, &font, &text_size, &baseline);

			cv::Point2d origin = cvPoint(uv.x - text_size.width / 2, uv.y - RADIUS - baseline - 3);
			cv::putText(image, txt, origin, cv::FONT_HERSHEY_SIMPLEX, 0.7, CV_RGB(88, 146, 211), 2);
		}
	}

	cropImage(inputBridge->image, offset_w, offset_h, out_width, out_height);
	imagePub.publish(inputBridge->toImageMsg());
} 

void Telepresence::cloudCb(const sensor_msgs::PointCloud2 &msg) 
{
	if (!dynamic_map)
		cloudSub.shutdown();
	cloud = msg;
} 

bool Telepresence::clickCb(telepresence::ClickRequest &req, 
		telepresence::ClickResponse &res)
{
	float new_pix_x = req.x*out_width/orig_width*image.cols + offset_w;
	float new_pix_y = req.y*out_height/orig_height*image.rows + offset_h;
	cv::Point2d uv(new_pix_x, new_pix_y);
	ROS_WARN("[telepresence] clicked point %i, %i", (int)uv.x, (int)uv.y);

	cv::Point3d ray;
	if (fisheye)
		ray = camModel.projectPixelTo3dRay(uv);
	if (!fisheye)
		ray = camModel.projectPixelTo3dRay(camModel.rectifyPoint(uv));

	geometry_msgs::Vector3Stamped ray_tf_camera, ray_tf_map;
	ray_tf_camera.header.stamp = ros::Time::now();
	ray_tf_camera.header.frame_id = camera_frame;
	ray_tf_camera.vector.x = ray.x;
	ray_tf_camera.vector.y = ray.y;
	ray_tf_camera.vector.z = ray.z;
	try
	{
		tfListener.waitForTransform("map", ray_tf_camera.header.frame_id,
				ros::Time::now(), ros::Duration(1.0));
		tfListener.transformVector("map", ray_tf_camera, ray_tf_map);
	} catch (tf::TransformException& ex)
	{
		ROS_ERROR("[telepresence] TF exception:\n%s", ex.what());
		res.message = std::string("TF exception: ") + ex.what();
		res.success = false;
		return true;
	}

	ray.x = ray_tf_map.vector.x;
	ray.y = ray_tf_map.vector.y;
	ray.z = ray_tf_map.vector.z;

	cv::Point3d ray_transformed(ray_tf_map.vector.x, ray_tf_map.vector.y, ray_tf_map.vector.z);
	ray_tf_map.vector.x /= cv::norm(ray_transformed);
	ray_tf_map.vector.y /= cv::norm(ray_transformed);
	ray_tf_map.vector.z /= cv::norm(ray_transformed);
	ray /= cv::norm(ray);

	cv::Point3d step = ray * ( ray_step / cv::norm(ray) );

	auto t = ros::Time::now();
	tf::StampedTransform tr;
	tfListener.waitForTransform("map", camera_frame, t, ros::Duration(1.0));
	tfListener.lookupTransform("map", camera_frame, t, tr);
	geometry_msgs::Point pt;
	pt.x = tr.getOrigin().x();
	pt.y = tr.getOrigin().y();
	pt.z = tr.getOrigin().z();
	cv::Point3d pt_camera(pt.x, pt.y, pt.z); // camera pose
	cv::Point3d pt_ray = pt_camera; // given point on ray (this is gonna be iterated)

	if (use_pointcloud) 
	{
		if (!ray_marker)
			pt_ray += ray * (robot_radius + ray_step);

		if (ray_marker) // drawing ray in rviz
		{
			marker.id = 0;
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();

			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.05;
			marker.color.a = 1.0;
			marker.color.r = 88	/ 255.;
			marker.color.g = 146 / 255.;
			marker.color.b = 211 / 255.;
			marker.type = visualization_msgs::Marker::LINE_LIST;

			marker.points.clear();

			geometry_msgs::Point p;
			p.x = pt_ray.x;
			p.y = pt_ray.y;
			p.z = pt_ray.z;
			marker.points.push_back(p);
			pt_ray += ray * (robot_radius + ray_step);
			p.x = pt_ray.x;
			p.y = pt_ray.y;
			p.z = pt_ray.z;
			marker.points.push_back(p);
		}

		int count = 1;
		ROS_DEBUG("[teleprecence] iterating over ray...");
		while (true) // incrementing point on ray
		{
			ROS_DEBUG("[telepresence] %ith iter, distance from ray: %f", count, ray_step*count++);
			if (!ray_marker)
			{
				pt_ray += step;
			}

			if (ray_marker) // drawing ray in rviz
			{
				geometry_msgs::Point p;
				p.x = pt_ray.x;
				p.y = pt_ray.y;
				p.z = pt_ray.z;
				marker.points.push_back(p);
				pt_ray += step;
				p.x = pt_ray.x;
				p.y = pt_ray.y;
				p.z = pt_ray.z;
				marker.points.push_back(p);

				rvizPub.publish(marker);
			}

			for (sensor_msgs::PointCloud2ConstIterator<float> it(cloud, "x"); // iterating over pc
					it != it.end(); ++it)
			{

				cv::Point3d pt_cloud(it[0], it[1], it[2]);

				if (cv::norm(pt_ray - pt_cloud) > ray_step) // point not found
					continue;

				// point found

				ROS_INFO("[telepresence] found point at [%f, %f, %f] (map frame)",
						it[0], it[1], it[2]);


				// broadcasting tf frame of goal
				goalPose.header.stamp = ros::Time::now();
				goalPose.header.frame_id = "map";
				goalPose.child_frame_id = goal_tf_frame;
				goalPose.transform.translation.x = it[0];
				goalPose.transform.translation.y = it[1];
				goalPose.transform.translation.z = it[2];
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				goalPose.transform.rotation.x = q.x();
				goalPose.transform.rotation.y = q.y();
				goalPose.transform.rotation.z = q.z();
				goalPose.transform.rotation.w = q.w();
				br.sendTransform(goalPose);


				if (use_move_base) // sending move_base goal
				{
					static geometry_msgs::PoseStamped goal_mb;
					goal_mb.header.frame_id = "map";
					goal_mb.header.stamp = ros::Time::now();

					goal_mb.pose.position.x = it[0] - ray_tf_map.vector.x * stepback_dist;
					goal_mb.pose.position.y = it[1] - ray_tf_map.vector.y * stepback_dist;
					goal_mb.pose.position.z = 0;
					goal_mb.pose.orientation.w = 1;

					move_basePub.publish(goal_mb);
					// moveBaseAction->sendGoal(goal_mb);
					ROS_DEBUG("[telepresence] move_base goal sent");
				}

				if (use_waypoint) // sending waypoint
				{
					// TODO
					ROS_DEBUG("[telepresence] waypoint sent");
				}

				ROS_DEBUG("[telepresence] callback finished with success");
				cv::Point3d pt_print(it[0], it[1], it[2]);
				res.message = std::string("Point found!");
				res.success = true;
				return true;
			} 

			if (cv::norm(pt_camera-pt_ray) > max_ray_dist) // max distance, point not found
			{
				ROS_ERROR("[telerpesence] could not find click 3d pose");
				res.message = "Point not found, please check your TF tree and odometry";
				res.success = false;
				return true;
			}
		}
	} 
	if (!use_pointcloud) 
	{
		res.success = false;
		grid_map::Position hit_pos;
		if (!ray_marker)
			pt_ray += ray * robot_radius;

		if (ray_marker) // drawing ray in rviz
		{
			marker.id = 0;
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();

			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.05;
			marker.color.a = 1.0;
			marker.color.r = 88	/ 255.;
			marker.color.g = 146 / 255.;
			marker.color.b = 211 / 255.;
			marker.type = visualization_msgs::Marker::LINE_LIST;

			marker.points.clear();

			geometry_msgs::Point p;
			p.x = pt_ray.x;
			p.y = pt_ray.y;
			p.z = 0;
			marker.points.push_back(p);
			pt_ray += ray * robot_radius;
			p.x = pt_ray.x;
			p.y = pt_ray.y;
			p.z = 0;
			marker.points.push_back(p);
		}

		step /= sqrt(pow(step.x, 2) + pow(step.y, 2)); // horizontal proj is normalized

		grid_map::Position start(pt_ray.x, pt_ray.y);// pt_ray is near pt_camera at this point
		grid_map::Position end;

		auto ray_max_end = pt_ray + ray * (max_ray_dist - robot_radius);
		float dist_to_gnd = sqrt(1/pow(ray.z, 2.0) + 1) * pt_camera.z;

		ROS_DEBUG_STREAM("[telepresence] \n\tz: " << ray.z << 
			"\n\th: " << pt_camera.z <<
			"\n\td: " << dist_to_gnd);

		if (ray.z < 0 && max_ray_dist > dist_to_gnd) // don't go under ground
		{
			auto pt_hit_gnd = pt_camera + ray * dist_to_gnd / sqrt(pow(ray.x, 2) + pow(ray.y, 2));
			end = grid_map::Position(pt_hit_gnd.x, pt_hit_gnd.y);
		} else
			end = grid_map::Position(ray_max_end.x, ray_max_end.y);

		int count = 1;
		ROS_DEBUG_STREAM("[teleprecence] iterating over ray...	start: " << start << ", end: " << end);
		grid_map::Position prev_pos;

		for (grid_map::LineIterator it(grid_map, start, end); !it.isPastEnd(); ++it)
		{
			grid_map::Position curr_pos;
			grid_map.getPosition(*it, curr_pos);
			auto pix_value = grid_map.at("grid", *it);
			ROS_DEBUG_STREAM("[telepresence] " << count++ << "th iter, cell_value: " << pix_value << ", position: " << curr_pos);

			if (ray_marker) // drawing ray in rviz
			{
					geometry_msgs::Point p;
					p.x = prev_pos.x();
					p.y = prev_pos.y();
					p.z = 0;
					marker.points.push_back(p);

					p.x = curr_pos.x();
					p.y = curr_pos.y();
					p.z = 0;
					marker.points.push_back(p);

					rvizPub.publish(marker);
			}
			prev_pos = curr_pos;

			if (pix_value > 50) // hit target
			{
				grid_map.getPosition(*it, hit_pos);
				ROS_INFO_STREAM("[telepresence] hit obstacle at position " << hit_pos << ", cell_value: "<< pix_value);
				res.message = "Hit obstacle at position (" + std::to_string(hit_pos.x()) + ", " + std::to_string(hit_pos.y());
				res.success = true;
				break;
			}
		}
		if (!res.success)
		{
			hit_pos = end;
			ROS_INFO_STREAM("[telepresence] hit ground or max_ray_dist at position " << hit_pos << ", cell_value: "<< grid_map.atPosition("grid", hit_pos));
			res.message = "Hit ground or max_ray_dist at position (" + std::to_string(end.x()) + ", " + std::to_string(end.y());
			res.success = true;
		}

		double d = cv::norm(cv::Point2d(pt_camera.x, pt_camera.y) - cv::Point2d(hit_pos.x(), hit_pos.y()));
		double hit_pos_z = pt_camera.z + d * step.z;

		// broadcasting tf frame of goal
		goalPose.header.stamp = ros::Time::now();
		goalPose.header.frame_id = "map";
		goalPose.child_frame_id = goal_tf_frame;
		goalPose.transform.translation.x = hit_pos.x();
		goalPose.transform.translation.y = hit_pos.y();
		goalPose.transform.translation.z = hit_pos_z;
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		goalPose.transform.rotation.x = q.x();
		goalPose.transform.rotation.y = q.y();
		goalPose.transform.rotation.z = q.z();
		goalPose.transform.rotation.w = q.w();
		br.sendTransform(goalPose);


		if (use_move_base) // sending move_base goal
		{
			static geometry_msgs::PoseStamped goal_mb;
			goal_mb.header.frame_id = "map";
			goal_mb.header.stamp = ros::Time::now();

			goal_mb.pose.position.x = hit_pos.x() - ray_tf_map.vector.x * stepback_dist;
			goal_mb.pose.position.y = hit_pos.y() - ray_tf_map.vector.y * stepback_dist;
			goal_mb.pose.position.z = 0;
			goal_mb.pose.orientation.w = 1;

			move_basePub.publish(goal_mb);
			// moveBaseAction->sendGoal(goal_mb);
			ROS_DEBUG("[telepresence] move_base goal sent");
		}

		if (use_waypoint) // sending waypoint
		{
			// TODO
			ROS_DEBUG("[telepresence] waypoint sent");
		}

		return true;
	} 
} 

void Telepresence::gridCb(const nav_msgs::OccupancyGrid &msg) 
{
	if (!dynamic_map)
		gridSub.shutdown();
	oc_grid = msg;
	grid_map::GridMapRosConverter::fromOccupancyGrid(oc_grid, "grid", grid_map);
} 



std::string telepresence::precision_2(float number) 
{
		bool neg = false;
		if (number < 0)
		{
			neg = true;
			number = -number;
		}
		int decimal_part = (number * 100) - ((int)number * 100);
		if (neg)
		{
			if (decimal_part > 10)
				return "-" + std::to_string((int)number) + "." + std::to_string(decimal_part);
			else
				return "-" + std::to_string((int)number) + ".0" + std::to_string(decimal_part);
		} else
		{
			if (decimal_part > 10)
				return std::to_string((int)number) + "." + std::to_string(decimal_part);
			else
				return std::to_string((int)number) + ".0" + std::to_string(decimal_part);
		}
}

void telepresence::cropImage(cv::Mat& image, const int& o_w, const int& o_h, const int& width, const int& height)
{
	image = image(cv::Rect(o_w, o_h, width, height));
}

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "telepresence");
	ros::NodeHandle nh("telepresence");
	Telepresence tp(nh);
	ros::spin();
} 
