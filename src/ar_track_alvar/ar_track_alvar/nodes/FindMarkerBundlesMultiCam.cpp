/*
  Software License Agreement (BSD License)
  Copyright (c) 2012, Scott Niekum
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:
  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
  * Neither the name of the Willow Garage nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
  author: Scott Niekum
*/

#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkersMultiCam.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <ar_track_alvar/ParamsConfig.h>
#include <image_transport/subscriber_filter.h>

using namespace alvar;
using namespace std;

#define MAIN_MARKER 1
#define VISIBLE_MARKER 2
#define GHOST_MARKER 3

Camera* cam1;
Camera* cam2;
Camera* cam3;
std::vector<cv_bridge::CvImagePtr> cv_ptrs_;
cv_bridge::CvImagePtr cv_ptr_;
// image_transport::SubscriberFilter cam1_sub, cam2_sub, cam3_sub;



ros::Publisher arMarkerPub_;
ros::Publisher rvizMarkerPub_;
tf::TransformListener* tf_listener;
tf::TransformBroadcaster* tf_broadcaster;
MarkerDetector<MarkerData> marker_detector;
MultiMarkerBundle** multi_marker_bundles = nullptr;
Pose* bundlePose; // NOT AN ARRAY

int master_id;
bool bundle_seen;
std::vector<int>* bundle_indices;
bool init = true;

bool enable_switched = false;
bool enabled = true;
double marker_size;
double max_new_marker_error;
double max_track_error;
std::string cam1_image_topic;
std::string cam2_image_topic;
std::string cam3_image_topic;
std::string cam1_info_topic;
std::string cam2_info_topic;
std::string cam3_info_topic;
std::string output_frame;
double max_frequency = 8.0;  // default maximum frequency
int marker_resolution = 5;   // default marker resolution
int marker_margin = 2;       // default marker margin
int n_bundles = 0;

void GetMultiMarkerPoses(cv::Mat& image);
void callback(const sensor_msgs::ImageConstPtr& image1_msg,
              const sensor_msgs::ImageConstPtr& image2_msg,
              const sensor_msgs::ImageConstPtr& image3_msg);
void makeMarkerMsgs(int type, int id, Pose& p,
                    const sensor_msgs::ImageConstPtr& image_msg,
                    tf::StampedTransform& CamToOutput,
                    visualization_msgs::Marker* rvizMarker,
                    ar_track_alvar_msgs::AlvarMarker* ar_pose_marker);

// Updates the bundlePoses of the multi_marker_bundles by detecting markers and
// using all markers in a bundle to infer the master tag's position
void GetMultiMarkerPoses(cv::Mat& image, int camID) {
  Camera* cam;
  if (camID == 1) {
    cam = cam1; 
  } else if (camID == 2) {
    cam = cam2;
  } else if (camID == 3) {
    cam = cam3;
  } else {
    ROS_ERROR("[Jonathan@pancake_flipping]: camID must be 1, 2, or 3!");
    exit(EXIT_FAILURE);
  }
  if (marker_detector.Detect(image, cam, true, false, max_new_marker_error,
                             max_track_error, CVSEQ, true)) {
    multi_marker_bundles[camID-1]->Update(marker_detector.markers, cam, *bundlePose);
    if ((marker_detector.DetectAdditional(image, cam, false) > 0) &&
        (multi_marker_bundles[camID-1]->SetTrackMarkers(marker_detector, cam, *bundlePose, image) > 0)) {
      multi_marker_bundles[camID-1]->Update(marker_detector.markers, cam, *bundlePose);
    }
  }
}

// Given the pose of a marker, builds the appropriate ROS messages for later
// publishing
void makeMarkerMsgs(int type, int id, Pose& p,
                    const sensor_msgs::ImageConstPtr& image_msg,
                    tf::StampedTransform& CamToOutput,
                    visualization_msgs::Marker* rvizMarker,
                    ar_track_alvar_msgs::AlvarMarkersMultiCam* ar_cam,
                    int camID) {
  ar_track_alvar_msgs::AlvarMarker* ar_pose_marker;
  if (camID == 1){
    ar_pose_marker = &ar_cam->cam1_marker;
  } else if (camID == 2){
    ar_pose_marker = &ar_cam->cam2_marker;
  } else if (camID == 3){
    ar_pose_marker = &ar_cam->cam3_marker;
  } else {
    ROS_ERROR("[Jonathan@pancake_flipping]: [3] camID must be 1, 2, or 3!");
    exit(EXIT_FAILURE);
  }

  double px, py, pz, qx, qy, qz, qw;
  px = p.translation[0] / 100.0;
  py = p.translation[1] / 100.0;
  pz = p.translation[2] / 100.0;
  qx = p.quaternion[1];
  qy = p.quaternion[2];
  qz = p.quaternion[3];
  qw = p.quaternion[0];

  // Get the marker pose in the camera frame
  tf::Quaternion rotation(qx, qy, qz, qw);
  tf::Vector3 origin(px, py, pz);
  tf::Transform t(rotation, origin);  // transform from cam to marker

  tf::Vector3 markerOrigin(0, 0, 0);
  tf::Transform m(tf::Quaternion::getIdentity(), markerOrigin);
  tf::Transform markerPose = t * m;

  // Create the rviz visualization message
  tf::poseTFToMsg(markerPose, rvizMarker->pose);
  rvizMarker->header.frame_id = image_msg->header.frame_id;
  rvizMarker->header.stamp = image_msg->header.stamp;
  rvizMarker->id = id;
  rvizMarker->scale.x = 1.0 * marker_size / 100.0;
  rvizMarker->scale.y = 1.0 * marker_size / 100.0;
  rvizMarker->scale.z = 0.2 * marker_size / 100.0;
  rvizMarker->ns = type == MAIN_MARKER ? "main_shapes" : "basic_shapes";
  rvizMarker->type = visualization_msgs::Marker::CUBE;
  rvizMarker->action = visualization_msgs::Marker::ADD;

  // Determine a color and opacity, based on marker type
  if (type == MAIN_MARKER) {
    rvizMarker->color.r = 1.0f;
    rvizMarker->color.g = 0.0f;
    rvizMarker->color.b = 0.0f;
    rvizMarker->color.a = 1.0;
  } else if (type == VISIBLE_MARKER) {
    rvizMarker->color.r = 0.0f;
    rvizMarker->color.g = 1.0f;
    rvizMarker->color.b = 0.0f;
    rvizMarker->color.a = 0.7;
  } else if (type == GHOST_MARKER) {
    rvizMarker->color.r = 0.0f;
    rvizMarker->color.g = 0.0f;
    rvizMarker->color.b = 1.0f;
    rvizMarker->color.a = 0.5;
  }

  rvizMarker->lifetime = ros::Duration(1.0);

  // Only publish the pose of the master tag in each bundle, since that's all we
  // really care about aside from visualization
  if (type == MAIN_MARKER) {
    // Take the pose of the tag in the camera frame and convert to the output
    // frame (usually torso_lift_link for the PR2)
    tf::Transform tagPoseOutput = CamToOutput * markerPose;

    // Create the pose marker message
    tf::poseTFToMsg(tagPoseOutput, ar_pose_marker->pose.pose);
    ar_pose_marker->header.frame_id = output_frame;
    ar_pose_marker->header.stamp = image_msg->header.stamp;
    ar_pose_marker->id = id;

    // Publish the output frame to marker transform for main marker in each bundle
    std::string markerFrame = "ar_marker_" + std::to_string(id);
    tf::StampedTransform outputToMarker(tagPoseOutput, image_msg->header.stamp,
                                        output_frame, markerFrame);
    tf_broadcaster->sendTransform(outputToMarker);
  }
}

// Callback to handle getting video frames and processing them
void callback(const sensor_msgs::ImageConstPtr& image1_msg,
              const sensor_msgs::ImageConstPtr& image2_msg,
              const sensor_msgs::ImageConstPtr& image3_msg) {
  ROS_WARN_STREAM("\n\n\n CALLBACK \n\n\n");
  std::vector<std::reference_wrapper<const sensor_msgs::ImageConstPtr>> image_msgs;
  image_msgs.push_back(image1_msg);              
  image_msgs.push_back(image2_msg);              
  image_msgs.push_back(image3_msg);              
  // If we've already gotten the cam info, then go ahead
  if (cam1->getCamInfo_ && cam2->getCamInfo_ && cam3->getCamInfo_) {
    try {
      // Get the transformation from the Camera to the output frame for this image capture
      std::vector<tf::StampedTransform> CamToOutputTFs;
      for (int i = 0; i < 3; i++){
        try {
          tf::StampedTransform CamToOutput;
          // TODO(@Jonathan): Replace with subscribers
          tf_listener->waitForTransform(output_frame, image1_msg->header.frame_id,
                                        image1_msg->header.stamp,
                                        ros::Duration(1.0));                          
          tf_listener->lookupTransform(output_frame, image1_msg->header.frame_id,
                                      image1_msg->header.stamp, CamToOutput);
          CamToOutputTFs.push_back(CamToOutput);
        }
        catch (const tf::TransformException& ex) {
          ROS_ERROR("%s", ex.what());
        }
      }
      visualization_msgs::Marker rvizMarker;
      ar_track_alvar_msgs::AlvarMarkersMultiCam ar_cams;
      // Convert the image
      for (std::size_t i = 0; i < image_msgs.size(); i++) {
        cv_ptr_ = cv_bridge::toCvCopy(image_msgs[i], sensor_msgs::image_encodings::BGR8);
        // Get the estimated pose of the main markers by using all the markers in each bundle
        GetMultiMarkerPoses(cv_ptr_->image, i+1);

        bundle_seen = false;
        for (auto& marker : *marker_detector.markers) {
          int id = static_cast<int>(marker.GetId());
          // Draw if id is valid
          if (id < 0) continue;
          
          // Mark the bundle as "seen"
          for (size_t k = 0; k < bundle_indices->size(); k++) {
            if ((*bundle_indices)[k] == id) {
              bundle_seen = true;
              break;
            }
          }
        }
        // Draw the main markers, whether they are visible or not -- but only if
        // at least 1 marker from their bundle is currently seen
        if (bundle_seen) {
          makeMarkerMsgs(MAIN_MARKER, master_id, *bundlePose, image_msgs[i],
                          CamToOutputTFs[i], &rvizMarker, &ar_cams, i+1);
        }
      }
      // Publish the marker messages
      arMarkerPub_.publish(ar_cams);
    }
    // Draw the observed markers that are visible and note if bundles was seen
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("[BE CAREFUL!] It's not just image 1. Could not convert from '%s' to 'rgb8'.",
                image1_msg->encoding.c_str());
    } 
  }
}

void configCallback(ar_track_alvar::ParamsConfig& config, uint32_t level) {
  (void)level;
  ROS_INFO("AR tracker reconfigured: %s %.2f %.2f %.2f %.2f",
           config.enabled ? "ENABLED" : "DISABLED", config.max_frequency,
           config.marker_size, config.max_new_marker_error,
           config.max_track_error);

  enable_switched = enabled != config.enabled;

  enabled = config.enabled;
  max_frequency = config.max_frequency;
  marker_size = config.marker_size;
  max_new_marker_error = config.max_new_marker_error;
  max_track_error = config.max_track_error;
}

void enableCallback(const std_msgs::BoolConstPtr& msg) {
  enable_switched = enabled != msg->data;
  enabled = msg->data;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "marker_detect");
  ros::NodeHandle n, pn("~");

  vector<string> bundle_files;
  std::string bundle_string;
  // Get params from ros param server.
  pn.param("marker_size", marker_size, 10.0);
  pn.param("max_new_marker_error", max_new_marker_error, 0.08);
  pn.param("max_track_error", max_track_error, 0.2);
  pn.param("max_frequency", max_frequency, 60.0);
  pn.param("marker_resolution", marker_resolution, 5);
  pn.param("marker_margin", marker_margin, 2);
  pn.param<std::string>("bundle_files", bundle_string, "");
  if (!pn.getParam("output_frame", output_frame)) {
    ROS_ERROR("Param 'output_frame' has to be set.");
    exit(EXIT_FAILURE);
  }

  // extract bundles
  std::stringstream ss(bundle_string);
  std::string token;
  while (std::getline(ss, token, ' ')) {
    if (!token.empty()) {
      bundle_files.push_back(token);
    }
  }

  n_bundles = bundle_files.size();
  if (n_bundles > 1) {
    ROS_ERROR("[Jonathan@pancake_flipping]: Only one bundle at a time!");
    exit(EXIT_FAILURE);
  }
  std::string bundle_file = bundle_files[0];

  // Camera input topics. Use remapping to map to your camera topics.
  cam1_image_topic = "camera_image1";
  cam2_image_topic = "camera_image2";
  cam3_image_topic = "camera_image3";
  cam1_info_topic  = "camera_info1";
  cam2_info_topic  = "camera_info2";
  cam3_info_topic  = "camera_info3";
  

  // Set dynamically configurable parameters so they don't get replaced by
  // default values
  pn.setParam("max_frequency", max_frequency);
  pn.setParam("marker_size", marker_size);
  pn.setParam("max_new_marker_error", max_new_marker_error);
  pn.setParam("max_track_error", max_track_error);

  marker_detector.SetMarkerSize(marker_size, marker_resolution, marker_margin);
  multi_marker_bundles = new MultiMarkerBundle*[3];
  bundlePose = new Pose;
  bundle_indices = new std::vector<int>;

  // Load the marker bundle XML files
  bundlePose->Reset();
  MultiMarker loadHelper;
  
  if (loadHelper.Load(bundle_file.c_str(), FILE_FORMAT_XML)) {
    vector<int> id_vector = loadHelper.getIndices();
    for (int i = 0; i < 3; i++){
      multi_marker_bundles[i] = new MultiMarkerBundle(id_vector);
      multi_marker_bundles[i]->Load(bundle_file.c_str(), FILE_FORMAT_XML);
    } 
    master_id = multi_marker_bundles[0]->getMasterId();
    (*bundle_indices) = multi_marker_bundles[0]->getIndices();
  } else {
    cout << "Cannot load file " << bundle_file << endl;
    return 0;
  }

  // Set up camera, listeners, and broadcasters
  cam1 = new Camera(n, cam1_info_topic);
  cam2 = new Camera(n, cam2_info_topic);
  cam3 = new Camera(n, cam3_info_topic);
  tf_listener = new tf::TransformListener(n);
  tf_broadcaster = new tf::TransformBroadcaster();
  arMarkerPub_ =
      n.advertise<ar_track_alvar_msgs::AlvarMarkersMultiCam>("synced_pose", 0);
  rvizMarkerPub_ =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  // Prepare dynamic reconfiguration
  dynamic_reconfigure::Server<ar_track_alvar::ParamsConfig> server;
  dynamic_reconfigure::Server<ar_track_alvar::ParamsConfig>::CallbackType f;

  f = boost::bind(&configCallback, _1, _2);
  server.setCallback(f);

  // Give tf a chance to catch up before the camera callback starts asking for
  // transforms
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  // Subscribe to topics and set up callbacks
  // image_transport::ImageTransport it(n);

  // Run at the configured rate, discarding pointcloud msgs if necessary
  ros::Rate rate(max_frequency);

  /// Subscriber for enable-topic so that a user can turn off the detection if
  /// it is not used without
  /// having to use the reconfigure where he has to know all parameters
  ros::Subscriber enable_sub_ =
      pn.subscribe("enable_detection", 1, &enableCallback);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Subscriber<sensor_msgs::Image> cam1_sub(n, "camera1/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> cam2_sub(n, "camera2/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> cam3_sub(n, "camera3/color/image_raw", 1);
  message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy(10), cam1_sub, cam2_sub, cam3_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));
  enable_switched = true;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    if (std::abs((rate.expectedCycleTime() - ros::Duration(1.0 / max_frequency)).toSec()) > 0.001){
      // Change rate dynamically; if must be above 0, as 0 will provoke a
      // segfault on next spinOnce
      ROS_DEBUG("Changing frequency from %.2f to %.2f",
                1.0 / rate.expectedCycleTime().toSec(), max_frequency);
      rate = ros::Rate(max_frequency);
    }

    if (enable_switched) {
      // Enable/disable switch: subscribe/unsubscribe to make use of pointcloud
      // processing nodelet lazy publishing policy; in CPU-scarce computer as
      // TurtleBot's laptop this is a huge saving
      if (enabled) {
        ROS_INFO("Subscribing to image topics");
        // cam1_sub.subscribe(it, "camera1/color/image_raw", 1);
        // cam2_sub.subscribe(it, "camera2/color/image_raw", 1);
        // cam3_sub.subscribe(it, "camera3/color/image_raw", 1);
      } else {
        cam1_sub.unsubscribe();
        cam2_sub.unsubscribe();
        cam3_sub.unsubscribe();
      }
      enable_switched = false;
    }
  }
  return 0;
}
