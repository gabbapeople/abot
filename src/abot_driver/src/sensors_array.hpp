


#ifndef SENSORS_ARRAY_HPP_
#define SENSORS_ARRAY_HPP_

#include <boost/thread.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>

class RangeSensor {
public:
    RangeSensor(std::string sensor_topic, std::string sensor_frame);
    double getRange();

    std::string frame;

private:
    ros::Subscriber _range_sub;

    double _range;

    ros::NodeHandle _node;

    std::string _topic;

    void rangeCallback(const sensor_msgs::Range& range_msg);
};

RangeSensor::RangeSensor(std::string sensor_topic, std::string sensor_frame) :
    _topic(sensor_topic),
    frame(sensor_frame) {

    _node = ros::NodeHandle();
    _range_sub = _node.subscribe(sensor_topic, 20, &RangeSensor::rangeCallback, this);
}

void RangeSensor::rangeCallback(const sensor_msgs::Range& range_msg) {
    if (frame.compare(range_msg.header.frame_id) == 0) {
        _range = range_msg.range;
    }
}

double RangeSensor::getRange() {
    return _range;
}


class RangeSensorArray {
public:
    RangeSensorArray(const std::string point_cloud_topic, const std::string point_cloud_frame);

    void addSensor(std::string sensor_topic, std::string sensor_frame);

private:
    void publishPointCloud();

    ros::NodeHandle _node;
    ros::Publisher _point_cloud_pub;
    boost::thread _publish_thread;

    double _publish_rate;

    std::string _point_cloud_frame;
    
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    std::vector<boost::shared_ptr<RangeSensor>> _sensors;
};

RangeSensorArray::RangeSensorArray(const std::string point_cloud_topic, const std::string point_cloud_frame) :
    _point_cloud_frame(point_cloud_frame),
    _tf_listener(_tf_buffer) {

    _node = ros::NodeHandle();
    _point_cloud_pub = _node.advertise<sensor_msgs::PointCloud2>(point_cloud_topic, 3);

    _publish_rate = 20;
    _publish_thread = boost::thread(boost::bind(&RangeSensorArray::publishPointCloud, this));

    ROS_INFO("Sensors array: Setup thread");
}

void RangeSensorArray::addSensor(std::string sensor_topic, std::string sensor_frame) {
    boost::shared_ptr<RangeSensor> sensorPtr(new RangeSensor(sensor_topic, sensor_frame));
    _sensors.push_back(sensorPtr);
    ROS_INFO("Sensors array: Added sensor %s  %s", sensor_topic.c_str(), sensor_frame.c_str());
}

void RangeSensorArray::publishPointCloud() {
    ros::Rate loop_rate(_publish_rate);
    while (ros::ok()) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        point_cloud->header.frame_id = _point_cloud_frame;
        point_cloud->height = 1;
        point_cloud->points.clear();

        std::vector<boost::shared_ptr<RangeSensor> >::iterator sensor_it;
        for (sensor_it = _sensors.begin(); sensor_it != _sensors.end(); ++sensor_it) {
            boost::shared_ptr<RangeSensor> sensor = *sensor_it;

            geometry_msgs::TransformStamped transform;

            try {
                transform = _tf_buffer.lookupTransform(_point_cloud_frame, sensor->frame, ros::Time(0));
            } catch (tf2::TransformException ex) {
                ROS_WARN("%s", ex.what());
                continue;
            }

            geometry_msgs::PointStamped pt;
            pt.point.x = sensor->getRange();
            geometry_msgs::PointStamped point_out;

            tf2::doTransform(pt, point_out, transform);

            pcl::PointXYZ pcl_point;
            pcl_point.x = point_out.point.x;
            pcl_point.y = point_out.point.y;
            pcl_point.z = point_out.point.z;
            point_cloud->points.push_back(pcl_point);
            ++(point_cloud->width);
        }

        _point_cloud_pub.publish(point_cloud);
        loop_rate.sleep();
    }
}

#endif // SENSORS_ARRAY_HPP_
