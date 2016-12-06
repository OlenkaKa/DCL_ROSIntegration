/*!
 * \file
 * \brief
 * \author Aleksandra Karbarczyk
 */

#include <memory>
#include <string>

#include "ImageSubscriber.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

using namespace std;

namespace Processors {
namespace ImageSubscriber {

ImageSubscriber::ImageSubscriber(const string &name) :
        Base::Component(name),
        ros_topic_("ros.topic", std::string("/image")) {
    registerProperty(ros_topic_);
}

ImageSubscriber::~ImageSubscriber() {
}

void ImageSubscriber::prepareInterface() {
    // Register data streams, events and event handlers HERE!
    registerStream("out_img", &out_img_);
    // Register handlers
    registerHandler("onNewImage", boost::bind(&ImageSubscriber::onNewImage, this));
    addDependency("onNewImage", NULL);
}

bool ImageSubscriber::onInit() {
    static int argc;
    static char *argv = NULL;
    ros::init(argc, &argv, "changeit", ros::init_options::NoSigintHandler);
    nh_ = new ros::NodeHandle;
    it_ = new image_transport::ImageTransport(*nh_);
    CLOG(LERROR) <<"Start! " << ros_topic_;
    image_sub_ = it_->subscribe(ros_topic_, 1, &ImageSubscriber::handleImage, this);
    rate_ = new ros::Rate(10.0);
//    subscribe_thread_ = new boost::thread(boost::bind(&ImageSubscriber::subscribe, this));
//    subscribe_thread_->join();
    return true;
}

bool ImageSubscriber::onFinish() {
//    delete subscribe_thread_;
    delete rate_;
    delete it_;
    delete nh_;
    return true;
}

bool ImageSubscriber::onStop() {
    return true;
}

bool ImageSubscriber::onStart() {
    return true;
}

void ImageSubscriber::onNewImage() {
    ros::spinOnce();
    rate_->sleep();
    if (!image_.empty()) {
        out_img_.write(image_);
    }
}

void ImageSubscriber::handleImage(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        CLOG(LERROR) << "cv_bridge exception: " << e.what();
        return;
    }
    image_ = cv_ptr->image.clone();
}

//void ImageSubscriber::subscribe() {
//    while (nh_->ok()){
//        ros::spinOnce();
//        rate_->sleep();
//    }
//}

} //: namespace ImageSubscriber
} //: namespace Processors
