//
// Created by sebastiano on 8/18/16.
//

#ifndef ORB_SLAM2_ROSPUBLISHER_H
#define ORB_SLAM2_ROSPUBLISHER_H

#include "IPublisherThread.h"
#include "IFrameSubscriber.h"
#include "IMapPublisher.h"
#include "FrameDrawer.h"
#include "System.h"

#include <chrono>
#include <mutex>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace ORB_SLAM2 
{
    class Map;
    class Tracking;
}

class ROSPublisher :
    public ORB_SLAM2::IPublisherThread,
    public ORB_SLAM2::IMapPublisher,
    public ORB_SLAM2::IFrameSubscriber
{
public:
    static constexpr const char *DEFAULT_MAP_FRAME = "map";
    static constexpr const char *DEFAULT_CAMERA_FRAME = "camera";

    // `frequency` is max amount of messages emitted per second
    explicit ROSPublisher(
	ORB_SLAM2::Map *map,
	double frequency,
	std::string map_frame = DEFAULT_MAP_FRAME,
	std::string camera_frame = DEFAULT_CAMERA_FRAME,
	ros::NodeHandle nh = ros::NodeHandle());
    
    virtual void Run() override;
    virtual void Update(ORB_SLAM2::Tracking*);

protected:
    bool WaitCycleStart();

private:
    ORB_SLAM2::FrameDrawer drawer_;

    // Important: `nh_` goes before the `*_pub_`, because their construction relies on `nh_`!
    ros::NodeHandle nh_;
    std::string map_frame_name_, camera_frame_name_;
    ros::Publisher map_pub_, map_updates_pub_, image_pub_, odom_pub_, status_pub_;
    tf::TransformBroadcaster camera_tf_pub_;
    ros::Rate pub_rate_;
};

class ROSSystemBuilder : public ORB_SLAM2::System::GenericBuilder {
public:
    ROSSystemBuilder(const std::string& strVocFile,
                     const std::string& strSettingsFile,
                     ORB_SLAM2::System::eSensor sensor,
                     double frequency,
                     ros::NodeHandle nh = ros::NodeHandle(),
		     std::string map_frame = ROSPublisher::DEFAULT_MAP_FRAME,
                     std::string camera_frame = ROSPublisher::DEFAULT_CAMERA_FRAME);
    virtual ~ROSSystemBuilder();
    
    virtual ORB_SLAM2::IPublisherThread* GetPublisher() override;

private:
    std::unique_ptr<ROSPublisher> mpPublisher;
};


#endif //ORB_SLAM2_ROSPUBLISHER_H
