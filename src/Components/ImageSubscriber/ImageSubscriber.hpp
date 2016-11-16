/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef IMAGESUBSCRIBER_HPP_
#define IMAGESUBSCRIBER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace Processors {
namespace ImageSubscriber {

/*!
 * \class ImageSubscriber
 * \brief ImageSubscriber processor class.
 *
 * 
 */
class ImageSubscriber : public Base::Component {
public:
    /*!
     * Constructor.
     */
    ImageSubscriber(const std::string &name = "ImageSubscriber");

    /*!
     * Destructor
     */
    virtual ~ImageSubscriber();

    /*!
     * Prepare components interface (register streams and handlers).
     * At this point, all properties are already initialized and loaded to
     * values set in config file.
     */
    void prepareInterface();

protected:

    /*!
     * Connects source to given device.
     */
    bool onInit();

    /*!
     * Disconnect source from device, closes streams, etc.
     */
    bool onFinish();

    /*!
     * Start component
     */
    bool onStart();

    /*!
     * Stop component
     */
    bool onStop();


    // Input data streams

    // Output data streams
    Base::DataStreamOut<cv::Mat> out_img_;

    // Handlers

    // Properties
    Base::Property<std::string> ros_topic_;

    ros::NodeHandle *nh_;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_;

    cv::Mat image_;


    // Handlers
    void onNewImage();
    void handleImage(const sensor_msgs::ImageConstPtr& msg);

};

} //: namespace ImageSubscriber
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ImageSubscriber", Processors::ImageSubscriber::ImageSubscriber)

#endif /* IMAGESUBSCRIBER_HPP_ */
