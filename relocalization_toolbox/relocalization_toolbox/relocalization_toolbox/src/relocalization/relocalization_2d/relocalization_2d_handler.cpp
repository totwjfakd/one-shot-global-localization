/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <algorithm>
#include <limits>
#include <angles/angles.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <common/common_state_code.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <relocalization_toolbox_msgs/initial_pose_request.h>
#include <relocalization_toolbox_msgs/relocalization_request.h>
#include <relocalization_toolbox_msgs/get_candidates_request.h>
#include <relocalization_toolbox_msgs/score_pose_request.h>
#include <relocalization/utils/relocalization_2d/relocalization_2d.h>

#define NODE_NAME "relocalization_2d_handler"
#define TAG "2D Global Relocalization"

using namespace std;

// general parameters
bool publish_init_pose_topic;
bool call_init_pose_request;

string map_frame;
string base_frame;
string lidar_frame;
bool prefetch_map_from_topic;
bool lidar_reverted;
int lidar_sampling_step;
int map_sampling_ratio;

bool enable_visualization;

bool use_traversability_sampling;
float rrt_min_expand_dist, rrt_max_expand_dist;
int traversability_sampling_duration;

float fit_score_threshold_min, fit_score_threshold_max;
float angle_search_step;
int area_batch_proc_number;
int area_candidate_number;
float min_dist_to_obstacle;

// likelihood field model parameters
float sigma_hit;
float z_hit;
float z_rand;
float max_tolerance_dist;
float keep_ratio_1_min;
float keep_ratio_1_max;
float keep_ratio_2_min;
float keep_ratio_2_max;
bool geometric_mean_fusion;

// map-related parameters
int map_free_threshold;
int map_occupied_threshold;

// sub-area partitioning parameters
float sensing_radius;
float anchor_point_min_dist;

// icp matching parameters
float max_correspondence_distance;
float transformation_epsilon;
float euclidean_fitness_epsilon;
int max_iterations;

// tf components and transforms
unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
unique_ptr<tf2_ros::Buffer> tf_buffer;
geometry_msgs::TransformStamped tf_msg_lidar_base;
bool tf_msg_lidar_base_received = false;

// relocalization services
ros::ServiceServer relocalization_server;
ros::ServiceServer relocalization_simple_server;
ros::ServiceServer get_candidates_server;
ros::ServiceServer score_pose_server;
// relocalization client and publisher
ros::ServiceClient set_init_pose_client;
ros::Publisher init_pose_pub;

// relocalization core system
unique_ptr<relocalization_2d::relocalization_2d> relocalization_2d_instance;

// topic data
sensor_msgs::LaserScan scan_data;
nav_msgs::OccupancyGrid map_data;
bool scan_data_received = false;
bool map_data_received = false;

// callback functions
bool update_lidar_base_transform(const ros::Duration &timeout)
{
    try
    {
        tf_msg_lidar_base = tf_buffer->lookupTransform(
            lidar_frame,
            base_frame,
            ros::Time(0),
            timeout);
        tf_msg_lidar_base_received = true;

        return true;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s: Transform from %s to %s is not available yet: %s",
                 TAG, lidar_frame.c_str(), base_frame.c_str(), ex.what());
        tf_msg_lidar_base_received = false;

        return false;
    }
}

bool relocalization_request_callback(relocalization_toolbox_msgs::relocalization_request::Request &req,
                                     relocalization_toolbox_msgs::relocalization_request::Response &res)
{
    bool ret = false;

    // get current scan
    scan_data_received = false;

    if (req.scan_mode == req.MODE_FROM_SRV)
    {
        scan_data = req.scan;
        scan_data.header.frame_id = lidar_frame;

        scan_data_received = true;
    }
    else if (req.scan_mode == req.MODE_FROM_TOPIC)
    {
        auto scan_data_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan");

        if (scan_data_ptr)
        {
            scan_data = *scan_data_ptr;
            scan_data.header.frame_id = lidar_frame;

            scan_data_received = true;
        }
    }
    else
    {
        ROS_ERROR("%s: Request failed, invalid scan input mode.", TAG);

        res.result = CMD_RESPONSE_ERROR;
    }

    if (scan_data_received)
    {
        // check request map mode
        if (req.map_mode == req.MODE_FROM_SRV)
        {
            // load map from file
            auto data_loaded = load_map_2d(req.map_path, req.map_name, map_frame);
            map_data_received = get<0>(data_loaded);

            if (map_data_received)
            {
                map_data = get<1>(data_loaded);
            }
        }
        else if (req.map_mode == req.MODE_FROM_TOPIC)
        {
            // load map from topic
            auto map_data_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map");

            if (map_data_ptr)
            {
                map_data = *map_data_ptr;
                map_data.header.frame_id = map_frame;

                map_data_received = true;
            }
        }
        else
        {
            ROS_ERROR("%s: Request failed, invalid map input mode.", TAG);

            res.result = CMD_RESPONSE_ERROR;
        }

        if (map_data_received)
        {
            // Start to relocalize
            auto reloc_result = relocalization_2d_instance->relocalize(
                scan_data, lidar_reverted, lidar_sampling_step, map_sampling_ratio, map_data);

            if (get<0>(reloc_result))
            {
                if (!update_lidar_base_transform(ros::Duration(5.0)))
                {
                    ROS_ERROR("%s: Relocalization failed. Missing transform from %s to %s.", TAG,
                              lidar_frame.c_str(), base_frame.c_str());
                    res.result = CMD_RESPONSE_ERROR;
                    return true;
                }

                res.result = CMD_RESPONSE_OK;
                res.score = get<1>(reloc_result);

                // get transformations
                tf2::Transform tf_map_lidar, tf_lidar_base, tf_map_base;
                tf2::fromMsg(get<2>(reloc_result).transform, tf_map_lidar);
                tf2::fromMsg(tf_msg_lidar_base.transform, tf_lidar_base);
                tf_map_base = tf_map_lidar * tf_lidar_base;
                geometry_msgs::TransformStamped tf_map_to_base;
                tf_map_to_base.header.stamp = get<2>(reloc_result).header.stamp;
                tf_map_to_base.header.frame_id = map_frame;
                tf_map_to_base.child_frame_id = base_frame;
                tf_map_to_base.transform = tf2::toMsg(tf_map_base);

                res.tf_map_to_base = tf_map_to_base;

                geometry_msgs::PoseWithCovarianceStamped init_pose;
                init_pose.header.stamp = ros::Time::now();
                init_pose.header.frame_id = map_frame;
                init_pose.pose.pose.position.x = res.tf_map_to_base.transform.translation.x;
                init_pose.pose.pose.position.y = res.tf_map_to_base.transform.translation.y;
                init_pose.pose.pose.position.z = res.tf_map_to_base.transform.translation.z;
                init_pose.pose.pose.orientation = res.tf_map_to_base.transform.rotation;

                res.robot_pose_on_map = init_pose;

                if (publish_init_pose_topic)
                {
                    init_pose_pub.publish(init_pose);
                }

                if (call_init_pose_request)
                {
                    relocalization_toolbox_msgs::initial_pose_request init_pose_request;
                    init_pose_request.request.pose = init_pose;

                    set_init_pose_client.call(init_pose_request);
                }
            }
            else
            {
                ROS_ERROR("%s: Relocalization failed. All scores are lower than the threshold.", TAG);

                res.result = CMD_RESPONSE_ERROR;
            }
        }
        else
        {
            ROS_ERROR("%s: Request failed. No map data available.", TAG);

            res.result = CMD_RESPONSE_ERROR;
        }
    }
    else
    {
        ROS_ERROR("%s: Request failed. No scan data received.", TAG);

        res.result = CMD_RESPONSE_ERROR;
    }

    ret = true;

    return ret;
}

bool relocalization_simple_request_callback(std_srvs::Empty::Request &req,
                                            std_srvs::Empty::Response &res)
{
    relocalization_toolbox_msgs::relocalization_request srv;
    srv.request.map_mode = srv.request.MODE_FROM_TOPIC;
    srv.request.scan_mode = srv.request.MODE_FROM_TOPIC;

    return relocalization_request_callback(srv.request, srv.response);
}

bool get_candidates_request_callback(relocalization_toolbox_msgs::get_candidates_request::Request &req,
                                     relocalization_toolbox_msgs::get_candidates_request::Response &res)
{
    bool ret = false;

    // get current scan
    scan_data_received = false;

    if (req.scan_mode == req.MODE_FROM_SRV)
    {
        scan_data = req.scan;
        scan_data.header.frame_id = lidar_frame;

        scan_data_received = true;
    }
    else if (req.scan_mode == req.MODE_FROM_TOPIC)
    {
        auto scan_data_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan");

        if (scan_data_ptr)
        {
            scan_data = *scan_data_ptr;
            scan_data.header.frame_id = lidar_frame;

            scan_data_received = true;
        }
    }
    else
    {
        ROS_ERROR("%s: Request failed, invalid scan input mode.", TAG);

        res.result = CMD_RESPONSE_ERROR;
    }

    if (scan_data_received)
    {
        // check request mode
        if (req.map_mode == req.MODE_FROM_SRV)
        {
            // load map from file
            auto data_loaded = load_map_2d(req.map_path, req.map_name, map_frame);
            map_data_received = get<0>(data_loaded);

            if (map_data_received)
            {
                map_data = get<1>(data_loaded);
            }
        }
        else if (req.map_mode == req.MODE_FROM_TOPIC)
        {
            // load map from topic
            auto map_data_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map");

            if (map_data_ptr)
            {
                map_data = *map_data_ptr;
                map_data.header.frame_id = map_frame;
                
                map_data_received = true;
            }
        }
        else
        {
            ROS_ERROR("%s: Request failed, invalid map input mode.", TAG);

            res.result = CMD_RESPONSE_ERROR;
        }

        if (map_data_received)
        {
            // Start to get candidates
            auto reloc_candidates = relocalization_2d_instance->get_candidates(
                scan_data, lidar_reverted, lidar_sampling_step, map_sampling_ratio, map_data, req.max_num_of_candidates);

            if (get<0>(reloc_candidates))
            {
                if (!update_lidar_base_transform(ros::Duration(5.0)))
                {
                    ROS_ERROR("%s: Get candidates failed. Missing transform from %s to %s.", TAG,
                              lidar_frame.c_str(), base_frame.c_str());
                    res.result = CMD_RESPONSE_ERROR;
                    return true;
                }

                res.result = CMD_RESPONSE_OK;
                res.scores = get<1>(reloc_candidates);
                res.num_of_candidates = res.scores.size();

                // get transformations
                for (const auto &tf_msg_map_lidar : get<2>(reloc_candidates))
                {
                    tf2::Transform tf_map_lidar, tf_lidar_base, tf_map_base;
                    tf2::fromMsg(tf_msg_map_lidar.transform, tf_map_lidar);
                    tf2::fromMsg(tf_msg_lidar_base.transform, tf_lidar_base);
                    tf_map_base = tf_map_lidar * tf_lidar_base;
                    geometry_msgs::TransformStamped candidate_tf_map_to_base;
                    candidate_tf_map_to_base.header.stamp = tf_msg_map_lidar.header.stamp;
                    candidate_tf_map_to_base.header.frame_id = map_frame;
                    candidate_tf_map_to_base.child_frame_id = base_frame;
                    candidate_tf_map_to_base.transform = tf2::toMsg(tf_map_base);

                    res.candidates_tf_map_to_base.emplace_back(candidate_tf_map_to_base);

                    geometry_msgs::PoseWithCovarianceStamped candidate_pose;
                    candidate_pose.header.stamp = ros::Time::now();
                    candidate_pose.header.frame_id = map_frame;
                    candidate_pose.pose.pose.position.x = candidate_tf_map_to_base.transform.translation.x;
                    candidate_pose.pose.pose.position.y = candidate_tf_map_to_base.transform.translation.y;
                    candidate_pose.pose.pose.position.z = candidate_tf_map_to_base.transform.translation.z;
                    candidate_pose.pose.pose.orientation = candidate_tf_map_to_base.transform.rotation;

                    res.candidates_robot_pose_on_map.emplace_back(candidate_pose);
                }
            }
            else
            {
                ROS_ERROR("%s: Get candidates failed. All scores are lower than the threshold.", TAG);

                res.result = CMD_RESPONSE_ERROR;
            }
        }
        else
        {
            ROS_ERROR("%s: Request failed. No map data available.", TAG);

            res.result = CMD_RESPONSE_ERROR;
        }
    }
    else
    {
        ROS_ERROR("%s: Request failed. No scan data received.", TAG);

        res.result = CMD_RESPONSE_ERROR;
    }

    ret = true;

    return ret;
}

bool score_pose_request_callback(relocalization_toolbox_msgs::score_pose_request::Request &req,
                                 relocalization_toolbox_msgs::score_pose_request::Response &res)
{
    sensor_msgs::LaserScan score_scan_data;
    nav_msgs::OccupancyGrid score_map_data;
    bool score_scan_received = false;
    bool score_map_received = false;

    if (req.scan_mode == req.MODE_FROM_SRV)
    {
        score_scan_data = req.scan;
        score_scan_data.header.frame_id = lidar_frame;
        score_scan_received = true;
    }
    else if (req.scan_mode == req.MODE_FROM_TOPIC)
    {
        auto scan_data_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan");

        if (scan_data_ptr)
        {
            score_scan_data = *scan_data_ptr;
            score_scan_data.header.frame_id = lidar_frame;
            score_scan_received = true;
        }
    }
    else
    {
        ROS_ERROR("%s: Score request failed, invalid scan input mode.", TAG);
        res.result = CMD_RESPONSE_ERROR;
        return true;
    }

    if (!score_scan_received)
    {
        ROS_ERROR("%s: Score request failed. No scan data received.", TAG);
        res.result = CMD_RESPONSE_ERROR;
        return true;
    }

    if (req.map_mode == req.MODE_FROM_SRV)
    {
        auto data_loaded = load_map_2d(req.map_path, req.map_name, map_frame);
        score_map_received = get<0>(data_loaded);

        if (score_map_received)
        {
            score_map_data = get<1>(data_loaded);
        }
    }
    else if (req.map_mode == req.MODE_FROM_TOPIC)
    {
        auto map_data_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(5.0));

        if (map_data_ptr)
        {
            score_map_data = *map_data_ptr;
            score_map_data.header.frame_id = map_frame;
            score_map_received = true;
        }
    }
    else
    {
        ROS_ERROR("%s: Score request failed, invalid map input mode.", TAG);
        res.result = CMD_RESPONSE_ERROR;
        return true;
    }

    if (!score_map_received)
    {
        ROS_ERROR("%s: Score request failed. No map data received.", TAG);
        res.result = CMD_RESPONSE_ERROR;
        return true;
    }

    geometry_msgs::PoseWithCovarianceStamped known_pose = req.robot_pose_on_map;
    known_pose.header.stamp = score_scan_data.header.stamp;
    known_pose.header.frame_id = known_pose.header.frame_id.empty() ? map_frame : known_pose.header.frame_id;

    if (!update_lidar_base_transform(ros::Duration(5.0)))
    {
        ROS_ERROR("%s: Score request failed. Missing transform from %s to %s.", TAG,
                  lidar_frame.c_str(), base_frame.c_str());
        res.result = CMD_RESPONSE_ERROR;
        return true;
    }

    geometry_msgs::TransformStamped known_tf_map_to_base;
    known_tf_map_to_base.header.stamp = score_scan_data.header.stamp;
    known_tf_map_to_base.header.frame_id = known_pose.header.frame_id;
    known_tf_map_to_base.child_frame_id = base_frame;
    known_tf_map_to_base.transform.translation.x = known_pose.pose.pose.position.x;
    known_tf_map_to_base.transform.translation.y = known_pose.pose.pose.position.y;
    known_tf_map_to_base.transform.translation.z = known_pose.pose.pose.position.z;
    known_tf_map_to_base.transform.rotation = known_pose.pose.pose.orientation;

    tf2::Transform tf_map_base;
    tf2::Transform tf_lidar_base;
    tf2::fromMsg(known_pose.pose.pose, tf_map_base);
    tf2::fromMsg(tf_msg_lidar_base.transform, tf_lidar_base);
    tf2::Transform tf_map_lidar = tf_map_base * tf_lidar_base.inverse();

    geometry_msgs::TransformStamped known_tf_map_to_lidar;
    known_tf_map_to_lidar.header.stamp = score_scan_data.header.stamp;
    known_tf_map_to_lidar.header.frame_id = map_frame;
    known_tf_map_to_lidar.child_frame_id = lidar_frame;
    known_tf_map_to_lidar.transform = tf2::toMsg(tf_map_lidar);

    res.known_score = relocalization_2d_instance->score_transform(
        known_tf_map_to_lidar,
        score_scan_data,
        lidar_sampling_step,
        map_sampling_ratio,
        score_map_data);
    res.known_tf_map_to_base = known_tf_map_to_base;
    res.known_tf_map_to_lidar = known_tf_map_to_lidar;

    auto top_result = relocalization_2d_instance->get_candidates(
        score_scan_data,
        lidar_reverted,
        lidar_sampling_step,
        map_sampling_ratio,
        score_map_data,
        1);

    if (!get<0>(top_result) || get<1>(top_result).empty() || get<2>(top_result).empty())
    {
        ROS_ERROR("%s: Score request failed. No top relocalization candidate available.", TAG);
        res.result = CMD_RESPONSE_ERROR;
        return true;
    }

    res.top_score = get<1>(top_result)[0];
    res.top_tf_map_to_lidar = get<2>(top_result)[0];

    tf2::Transform tf_top_map_lidar, tf_top_map_base;
    tf2::fromMsg(res.top_tf_map_to_lidar.transform, tf_top_map_lidar);
    tf_top_map_base = tf_top_map_lidar * tf_lidar_base;

    res.top_tf_map_to_base.header.stamp = res.top_tf_map_to_lidar.header.stamp;
    res.top_tf_map_to_base.header.frame_id = map_frame;
    res.top_tf_map_to_base.child_frame_id = base_frame;
    res.top_tf_map_to_base.transform = tf2::toMsg(tf_top_map_base);

    res.dx = res.top_tf_map_to_base.transform.translation.x - res.known_tf_map_to_base.transform.translation.x;
    res.dy = res.top_tf_map_to_base.transform.translation.y - res.known_tf_map_to_base.transform.translation.y;
    res.distance = hypot(res.dx, res.dy);
    res.dyaw = angles::shortest_angular_distance(
        tf::getYaw(res.known_tf_map_to_base.transform.rotation),
        tf::getYaw(res.top_tf_map_to_base.transform.rotation));
    res.result = CMD_RESPONSE_OK;

    ROS_INFO("%s: Score comparison known=(%.3f, %.3f, %.3f) score=%.4f, top=(%.3f, %.3f, %.3f) score=%.4f, d=(%.3f, %.3f, %.3f, %.3f).",
             TAG,
             res.known_tf_map_to_base.transform.translation.x,
             res.known_tf_map_to_base.transform.translation.y,
             tf::getYaw(res.known_tf_map_to_base.transform.rotation),
             res.known_score,
             res.top_tf_map_to_base.transform.translation.x,
             res.top_tf_map_to_base.transform.translation.y,
             tf::getYaw(res.top_tf_map_to_base.transform.rotation),
             res.top_score,
             res.dx,
             res.dy,
             res.distance,
             res.dyaw);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    setlocale(LC_CTYPE, "en_US.utf8");

    ROS_INFO("%s: Node started.", TAG);

    ros::NodeHandle node_handle;
    ros::NodeHandle node_handle_param("~/relocalization_2d");

    // Load parameters
    node_handle_param.param<bool>("publish_init_pose_topic", publish_init_pose_topic, false);
    node_handle_param.param<bool>("call_init_pose_request", call_init_pose_request, true);

    node_handle_param.param<string>("map_frame", map_frame, "map");
    node_handle_param.param<string>("base_frame", base_frame, "base_link");
    node_handle_param.param<string>("lidar_frame", lidar_frame, "lidar_link");

    node_handle_param.param<bool>("prefetch_map_from_topic", prefetch_map_from_topic, true);

    node_handle_param.param<bool>("lidar_reverted", lidar_reverted, true);

    node_handle_param.param<int>("lidar_sampling_step", lidar_sampling_step, 4);
    node_handle_param.param<int>("map_sampling_ratio", map_sampling_ratio, 2);

    node_handle_param.param<bool>("enable_visualization", enable_visualization, true);

    node_handle_param.param<bool>("use_traversability_sampling", use_traversability_sampling, false);
    node_handle_param.param<float>("rrt_min_expand_dist", rrt_min_expand_dist, 0.5);
    node_handle_param.param<float>("rrt_max_expand_dist", rrt_max_expand_dist, 1.0);
    node_handle_param.param<int>("traversability_sampling_duration", traversability_sampling_duration, 5);

    node_handle_param.param<float>("fit_score_threshold_max", fit_score_threshold_max, 0.9f);
    node_handle_param.param<float>("fit_score_threshold_min", fit_score_threshold_min, 0.8f);
    node_handle_param.param<float>("angle_search_step", angle_search_step, 10.0f * M_PI / 180.0f);
    node_handle_param.param<int>("area_batch_proc_number", area_batch_proc_number, 500);
    node_handle_param.param<int>("area_candidate_number", area_candidate_number, 10);
    node_handle_param.param<float>("min_dist_to_obstacle", min_dist_to_obstacle, 0.4);

    node_handle_param.param<float>("sigma_hit", sigma_hit, 0.2f);
    node_handle_param.param<float>("z_hit", z_hit, 1.0f);
    node_handle_param.param<float>("z_rand", z_rand, 0.0f);
    node_handle_param.param<float>("max_tolerance_dist", max_tolerance_dist, 1.0f);
    node_handle_param.param<float>("keep_ratio_1_min", keep_ratio_1_min, 0.80f);
    node_handle_param.param<float>("keep_ratio_1_max", keep_ratio_1_max, 0.90f);
    node_handle_param.param<float>("keep_ratio_2_min", keep_ratio_2_min, 0.70f);
    node_handle_param.param<float>("keep_ratio_2_max", keep_ratio_2_max, 0.85f);
    node_handle_param.param<bool>("geometric_mean_fusion", geometric_mean_fusion, true);

    node_handle_param.param<int>("map_free_threshold", map_free_threshold, 30);
    node_handle_param.param<int>("map_occupied_threshold", map_occupied_threshold, 55);

    node_handle_param.param<float>("sensing_radius", sensing_radius, 15.0f);
    node_handle_param.param<float>("anchor_point_min_dist", anchor_point_min_dist, 3.0f);

    node_handle_param.param<float>("max_correspondence_distance", max_correspondence_distance, 1.0f);
    node_handle_param.param<float>("transformation_epsilon", transformation_epsilon, 1e-6);
    node_handle_param.param<float>("euclidean_fitness_epsilon", euclidean_fitness_epsilon, 1e-5);
    node_handle_param.param<int>("max_iterations", max_iterations, 50);

    // tf setup
    tf_broadcaster = unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);
    tf_buffer = unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
    tf2_ros::TransformListener tf_listener(*tf_buffer);

    // initialize relocalization instance
    relocalization_2d_instance = unique_ptr<relocalization_2d::relocalization_2d>(
        new relocalization_2d::relocalization_2d(
            enable_visualization, use_traversability_sampling,
            rrt_min_expand_dist, rrt_max_expand_dist, traversability_sampling_duration,
            fit_score_threshold_min, fit_score_threshold_max, angle_search_step,
            area_batch_proc_number, area_candidate_number, min_dist_to_obstacle,
            sigma_hit, z_hit, z_rand, max_tolerance_dist,
            keep_ratio_1_min, keep_ratio_1_max, keep_ratio_2_min, keep_ratio_2_max,
            geometric_mean_fusion,
            map_free_threshold, map_occupied_threshold,
            sensing_radius, anchor_point_min_dist,
            max_correspondence_distance, transformation_epsilon, euclidean_fitness_epsilon, max_iterations));

    if (prefetch_map_from_topic)
    {
        ROS_INFO("%s: Attempting to fetch map data...", TAG);

        auto map_data_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(5.0));

        if (map_data_ptr)
        {
            ROS_INFO("%s: Map data received successfully.", TAG);

            map_data = *map_data_ptr;
            map_data.header.frame_id = map_frame;

            map_data_received = true;

            relocalization_2d_instance->set_map(map_data, map_sampling_ratio);
        }
        else
        {
            ROS_WARN("%s: No map data. Will fetch when service is called.", TAG);
        }
    }

    // initialize services and publishers
    relocalization_server = node_handle.advertiseService("relocalization_request", relocalization_request_callback);
    relocalization_simple_server = node_handle.advertiseService("relocalization_simple_request", relocalization_simple_request_callback);
    get_candidates_server = node_handle.advertiseService("get_relocalization_candidates_request", get_candidates_request_callback);
    score_pose_server = node_handle.advertiseService("score_relocalization_pose", score_pose_request_callback);
    set_init_pose_client = node_handle.serviceClient<relocalization_toolbox_msgs::initial_pose_request>("set_init_pose_request");
    init_pose_pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    ROS_INFO("%s: Checking static transform from %s to %s...", TAG, lidar_frame.c_str(), base_frame.c_str());
    update_lidar_base_transform(ros::Duration(1.0));

    ROS_INFO("%s: Ready and listening for service requests.", TAG);

    ros::spin();

    if (relocalization_2d_instance)
    {
        relocalization_2d_instance.reset();
    }

    return 0;
}
