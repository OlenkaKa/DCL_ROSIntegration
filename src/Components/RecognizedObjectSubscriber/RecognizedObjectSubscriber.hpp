/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef RECOGNIZEDOBJECTSUBSCRIBER_HPP_
#define RECOGNIZEDOBJECTSUBSCRIBER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/HomogMatrix.hpp"

#include <opencv2/opencv.hpp>

#include <object_recognition_msgs/RecognizedObject.h>
#include <ros/ros.h>

namespace Processors {
namespace RecognizedObjectSubscriber {

/*!
 * \class RecognizedObjectSubscriber
 * \brief RecognizedObjectSubscriber processor class.
 *
 */
class RecognizedObjectSubscriber : public Base::Component {
public:
    /*!
     * Constructor.
     */
    RecognizedObjectSubscriber(const std::string &name = "RecognizedObjectSubscriber");

    /*!
     * Destructor
     */
    virtual ~RecognizedObjectSubscriber();

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
    Base::DataStreamOut<Types::HomogMatrix> out_homog_matrix_;

    // Handlers

    // Properties
    Base::Property<std::string> parent_frame_;
    Base::Property<std::string> ros_topic_;

    ros::NodeHandle *nh_;
    ros::Rate *rate_;
    ros::Subscriber sub_;

    Types::HomogMatrix homog_matrix_;

    // Handlers
    void onNewMessage();
    void handleMessage(const object_recognition_msgs::RecognizedObjectConstPtr& msg);

};

} //: namespace RecognizedObjectSubscriber
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("RecognizedObjectSubscriber", Processors::RecognizedObjectSubscriber::RecognizedObjectSubscriber)

#endif /* RECOGNIZEDOBJECTSUBSCRIBER_HPP_ */
