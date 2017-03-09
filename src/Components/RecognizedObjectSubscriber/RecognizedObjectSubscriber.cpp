/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "RecognizedObjectSubscriber.hpp"
#include "Common/Logger.hpp"
#include "Types/HomogMatrix.hpp"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <boost/bind.hpp>

using namespace std;
using namespace cv;

namespace Processors {
namespace RecognizedObjectSubscriber {

RecognizedObjectSubscriber::RecognizedObjectSubscriber(const std::string &name) :
        Base::Component(name),
        parent_frame_("parent.frame", std::string("/camera")),
        ros_topic_("ros.topic", std::string("/recognized_object")) {
    registerProperty(parent_frame_);
    registerProperty(ros_topic_);
}

RecognizedObjectSubscriber::~RecognizedObjectSubscriber() {
}

void RecognizedObjectSubscriber::prepareInterface() {
    // Register data streams, events and event handlers HERE!
    registerStream("out_homog_matrix", &out_homog_matrix_);
    // Register handlers
    registerHandler("onNewMessage", boost::bind(&RecognizedObjectSubscriber::onNewMessage, this));
    addDependency("onNewMessage", NULL);

}

bool RecognizedObjectSubscriber::onInit() {
    static int argc;
    static char *argv = NULL;
    ros::init(argc, &argv, "changeit2", ros::init_options::NoSigintHandler);
    nh_ = new ros::NodeHandle;
    CLOG(LERROR) <<"Start! " << ros_topic_;
    sub_ = nh_->subscribe(ros_topic_, 1, &RecognizedObjectSubscriber::handleMessage, this);
    rate_ = new ros::Rate(10.0);
    return true;
}

bool RecognizedObjectSubscriber::onFinish() {
    delete rate_;
    delete nh_;
    return true;
}

bool RecognizedObjectSubscriber::onStop() {
    return true;
}

bool RecognizedObjectSubscriber::onStart() {
    return true;
}

void RecognizedObjectSubscriber::onNewMessage() {
    ros::spinOnce();
    rate_->sleep();
    out_homog_matrix_.write(homog_matrix_);
}

//geometry_msgs::Pose transformPose(const geometry_msgs::Pose &start_pose, const tf::StampedTransform &end_tf) {
//    tf::Transform start_tf;
//    tf::poseMsgToTF(start_pose, start_tf);
//    tf::Transform start_end_tf = end_tf * start_tf;
//
//    geometry_msgs::Pose transformed_pose;
//    tf::poseTFToMsg(start_end_tf, transformed_pose);
//    return transformed_pose;
//}

void RecognizedObjectSubscriber::handleMessage(const object_recognition_msgs::RecognizedObjectConstPtr &msg) {
    static tf::TransformListener tf_listener;
    static tf::Transform start_tf;
    static tf::StampedTransform end_tf;

    try {
        tf_listener.lookupTransform(parent_frame_, msg->pose.header.frame_id,
                                    /* msg->pose.header.stamp */ ros::Time(0), end_tf);

        tf::poseMsgToTF(msg->pose.pose.pose, start_tf);
        tf::Transform start_end_tf = end_tf * start_tf;

        Eigen::Affine3d affine;
        tf::poseTFToEigen(start_end_tf, affine);


//        geometry_msgs::Pose transformed_pose = transformPose(msg->pose.pose.pose, world_sensor_tf);
//
//        Eigen::Affine3d affine;
//        tf::poseMsgToEigen(transformed_pose, affine);
        homog_matrix_ = Types::HomogMatrix(affine);
    } catch (tf::TransformException &e) {
        CLOG(LWARNING) << e.what() << '\n' << parent_frame_;
        homog_matrix_ = Types::HomogMatrix();
    }
}


} //: namespace RecognizedObjectSubscriber
} //: namespace Processors
