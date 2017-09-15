#ifndef _GAZEBO_FAULTY_GPS_PLUGIN_HH_
#define _GAZEBO_FAULTY_GPS_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/common.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/math/gzmath.hh"
#include "ros/ros.h"
// Custom callback queue
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
// All possible messages/services
#include "sensor_msgs/NavSatFix.h"


namespace gazebo {
    class GAZEBO_VISIBLE FaultyGPSPlugin : public SensorPlugin {
    //enumerate the different state of the plugin
    enum State {NONE, LOSS_SIGNAL, MULTIPATH, CORRECT};
        public: FaultyGPSPlugin();
        public: virtual ~FaultyGPSPlugin();
        public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
        protected: virtual void OnUpdate(sensors::GpsSensorPtr _sensor);
        protected: virtual void OnWorldUpdate(const common::UpdateInfo &_info);
    protected: void changeStatus();
    protected: void StartTimer();
        protected: sensors::GpsSensorPtr parentSensor;
        private: event::ConnectionPtr connection;
        private: event::ConnectionPtr updateConnection;
	private: ros::Subscriber sub;
	private: ros::Publisher pub;
	// A node handler used for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;
	// Probability of loss of signal
	private: float lossProb;
	// probability of multi path
	private: float multiPathProb;
	// variables for timing
	private: gazebo::common::Timer timer;
	private: gazebo::common::Time timerDuration;
	// currentState of the sensor
	private: State currentState;
	// translation applied when multipath state
	private: float latTranslation;
	private: float longTranslation;
	// max translation applied when multipath state
	private: float maxLatTranslation;
	private: float maxLongTranslation;
    };
}
#endif
