/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "RecognizedObjectPublisher.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

using namespace boost;
using namespace std;

namespace Processors {
namespace RecognizedObjectPublisher {

RecognizedObjectPublisher::RecognizedObjectPublisher(const std::string &name) :
        Base::Component(name),
        parent_frame_("parent.frame", std::string("/discode_camera")),
        ros_node_name_("ros.node", std::string("/discode_recognized_object")),
        ros_topic_name_("ros.topic", std::string("/recognized_object")),
        ros_spin_("ros.spin", false) {
    registerProperty(parent_frame_);
    registerProperty(ros_node_name_);
    registerProperty(ros_topic_name_);
    registerProperty(ros_spin_);
}

RecognizedObjectPublisher::~RecognizedObjectPublisher() {
}

void RecognizedObjectPublisher::prepareInterface() {
    // Register data streams, events and event handlers
    registerStream("in_object_pose", &in_object_pose_);
    registerStream("in_object_vertices_xyz", &in_object_vertices_xyz_);
    registerStream("in_object_triangles", &in_object_triangles_);
    registerStream("in_object_name", &in_object_name_);
    registerStream("in_object_confidence", &in_object_confidence_);

    // Register handlers
    registerHandler("spin", boost::bind(&RecognizedObjectPublisher::spin, this));
    addDependency("spin", NULL);

    registerHandler("publishPose", boost::bind(&RecognizedObjectPublisher::publishPose, this));
    addDependency("publishPose", &in_object_pose_);
    addDependency("publishPose", &in_object_name_);
    addDependency("publishPose", &in_object_vertices_xyz_);
    addDependency("publishPose", &in_object_triangles_);
    addDependency("publishPose", &in_object_confidence_);
}

bool RecognizedObjectPublisher::onInit() {
    CLOG(LTRACE) << "RecognizedObjectPublisher::onInit";
    static int argc;
    static char *argv = NULL;
    ros::init(argc, &argv, ros_node_name_, ros::init_options::NoSigintHandler);
    nh_ = new ros::NodeHandle;
    publisher_ = nh_->advertise<object_recognition_msgs::RecognizedObject>(ros_topic_name_, 1000);
    return true;
}

bool RecognizedObjectPublisher::onFinish() {
    CLOG(LTRACE) << "RecognizedObjectPublisher::onFinish";
    delete nh_;
    return true;
}

bool RecognizedObjectPublisher::onStop() {
    CLOG(LTRACE) << "RecognizedObjectPublisher::onStop";
    return true;
}

bool RecognizedObjectPublisher::onStart() {
    CLOG(LTRACE) << "RecognizedObjectPublisher::onStart";
    return true;
}

void RecognizedObjectPublisher::publishPose() {
    CLOG(LTRACE) << "RecognizedObjectPublisher::publishPose";
    Types::HomogMatrix pose = in_object_pose_.read();
    string name = in_object_name_.read();
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices = in_object_vertices_xyz_.read()[0];
    vector<pcl::Vertices> triangles = in_object_triangles_.read()[0];
    double confidence = in_object_confidence_.read();

    object_recognition_msgs::RecognizedObject result_message;
    createMessage(name, pose, vertices, triangles, confidence, result_message);

    publisher_.publish(result_message);
    spin();
}

void RecognizedObjectPublisher::spin() {
    if (ros_spin_) {
        ros::spinOnce();
    }
}

void RecognizedObjectPublisher::createMessage(const string &object_label, const Types::HomogMatrix& object_pose,
                                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& object_vertices,
                                              const vector<pcl::Vertices>& object_triangles,
                                              double object_confidence,
                                              object_recognition_msgs::RecognizedObject& message) {
    Eigen::Affine3d affine = object_pose;
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(affine, pose);

    float confidence = (float) object_confidence;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = parent_frame_;

    message.header = header;
    message.type.key = object_label;
    message.type.db = "";
    message.confidence = confidence;
    message.pose.header = header;
    message.pose.pose.pose = pose;

    geometry_msgs::Point pt;
    for (const pcl::PointXYZ &vertex : *object_vertices) {
        pt.x = vertex.x;
        pt.y = vertex.y;
        pt.z = vertex.z;
        message.bounding_mesh.vertices.push_back(pt);
    }
    shape_msgs::MeshTriangle mt;
    for (const pcl::Vertices &triangle : object_triangles) {
        mt.vertex_indices[0] = triangle.vertices[0];
        mt.vertex_indices[1] = triangle.vertices[1];
        mt.vertex_indices[2] = triangle.vertices[2];
        message.bounding_mesh.triangles.push_back(mt);
    }
}

} //: namespace RecognizedObjectPublisher
} //: namespace Processors
