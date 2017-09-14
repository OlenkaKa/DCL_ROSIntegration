/*!
 * \file
 * \brief 
 * \author Aleksandra Karbarczyk
 */

#ifndef RECOGNIZEDOBJECTPUBLISHER_HPP_
#define RECOGNIZEDOBJECTPUBLISHER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include "Types/HomogMatrix.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>

#include <object_recognition_msgs/RecognizedObject.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace Processors {
namespace RecognizedObjectPublisher {

/*!
 * \class RecognizedObjectPublisher
 * \brief RecognizedObjectPublisher processor class.
 *
 */
class RecognizedObjectPublisher : public Base::Component {
public:
    /*!
     * Constructor.
     */
    RecognizedObjectPublisher(const std::string &name = "RecognizedObjectPublisher");

    /*!
     * Destructor
     */
    virtual ~RecognizedObjectPublisher();

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


    /// Input data streams
    Base::DataStreamIn<Types::HomogMatrix, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_pose_;
    Base::DataStreamIn<std::string, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_name_;
    Base::DataStreamIn<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_vertices_xyz_;
    Base::DataStreamIn<std::vector<std::vector<pcl::Vertices> >, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_triangles_;

    Base::DataStreamIn<double, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_object_confidence_;

    /// Properties
    Base::Property<std::string> parent_frame_;
    Base::Property<std::string> ros_node_name_;
    Base::Property<std::string> ros_topic_name_;
    Base::Property<bool> ros_spin_;

    /// ROS
    ros::NodeHandle *nh_;
    ros::Publisher publisher_;

    /// Handlers
    void publishPose();

    /// Others
    void spin();
    void createMessage(const std::string &object_label, const Types::HomogMatrix &object_pose,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &object_vertices,
                       const std::vector<pcl::Vertices> &object_triangles,
                       double object_confidence,
                       object_recognition_msgs::RecognizedObject &message);
};

} //: namespace RecognizedObjectPublisher
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("RecognizedObjectPublisher", Processors::RecognizedObjectPublisher::RecognizedObjectPublisher)

#endif /* RECOGNIZEDOBJECTPUBLISHER_HPP_ */
