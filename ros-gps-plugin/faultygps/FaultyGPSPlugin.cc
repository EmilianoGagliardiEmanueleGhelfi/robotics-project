#include "FaultyGPSPlugin.hh"

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(FaultyGPSPlugin)
FaultyGPSPlugin::FaultyGPSPlugin() {}

FaultyGPSPlugin::~FaultyGPSPlugin() {
    this->parentSensor->DisconnectUpdated(this->connection);
    this->parentSensor.reset();
}

void FaultyGPSPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
    this->parentSensor = std::dynamic_pointer_cast<sensors::GpsSensor>(_parent);
    
    if (!this->parentSensor)
        gzthrow("FaultyGPSPlugin requires a gps sensor as its parent.");
    
    this->connection = this->parentSensor->ConnectUpdated(std::bind(&FaultyGPSPlugin::OnUpdate, this, this->parentSensor));
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&FaultyGPSPlugin::OnWorldUpdate, this, _1));
    // Initialize ros, if it has not already bee initialized
    if (!ros::isInitialized()){
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    // Create the ROS node
    rosNode.reset(new ros::NodeHandle("gazebo_client"));
    
    // read from the sdf server the parameters
    if (!_sdf->HasElement("lossProbability")){
      // if parameter tag does NOT exist
      std::cout << "Missing parameter <lossProbability> in GPSPlugin, default to .05" << std::endl;
      lossProb=0.05;
    }
    // if parameter tag exists, get its value
    else lossProb = _sdf->GetElement("lossProbability")->Get<double>();
    
    std::cout << "Read parameter lossProbability: " << lossProb << std::endl;
    
    if(lossProb>=1){
      std::cout << "The loss probability cannot be >= 1, using the default value 0.05";
      lossProb = 0.05;
    }
    
    // read from the sdf server the parameters
    if (!_sdf->HasElement("multiPathProbability")){
      // if parameter tag does NOT exist
      std::cout << "Missing parameter <multiPathProbability> in GPSPlugin, default to .01" << std::endl;
      multiPathProb=0.01;
    }
    // if parameter tag exists, get its value
    else multiPathProb = _sdf->GetElement("multiPathProbability")->Get<double>();
    
     std::cout << "Read parameter mulitPathProbability: " << multiPathProb << std::endl;
    
    if(multiPathProb>=1){
      std::cout << "The multipath probability cannot be >= 1, using the default value 0.01 ";
      multiPathProb = 0.01;
    }
    
     // read from the sdf server the parameters
    if (!_sdf->HasElement("maxLongTranslation")){
      // if parameter tag does NOT exist
      std::cout << "Missing parameter maxLongTranslation in GPSPlugin, default to .001" << std::endl;
      maxLongTranslation=0.001;
    }
    // if parameter tag exists, get its value
    else maxLongTranslation = _sdf->GetElement("maxLongTranslation")->Get<double>();
    
     std::cout << "Read parameter maxLongTranslation: " << maxLongTranslation << std::endl;
    
     // read from the sdf server the parameters
    if (!_sdf->HasElement("maxLatTranslation")){
      // if parameter tag does NOT exist
      std::cout << "Missing parameter maxLatTranslation in GPSPlugin, default to .001" << std::endl;
      maxLatTranslation=0.001;
    }
    // if parameter tag exists, get its value
    else maxLatTranslation = _sdf->GetElement("maxLatTranslation")->Get<double>();
    
     std::cout << "Read parameter maxLatTranslation: " << maxLatTranslation << std::endl;
    
    std::cout << "GPS plugin loaded succesfully"  << std::endl;
    // Create a named topic, advertise it
    pub = rosNode->advertise<sensor_msgs::NavSatFix>("/fix", 1);
    std::cout << "GPS plugin loaded succesfully" << std::endl;
    
    // set the default state
    this->currentState = FaultyGPSPlugin::State::NONE;
    
}

void FaultyGPSPlugin::OnUpdate(sensors::GpsSensorPtr _sensor) {
    sensor_msgs::NavSatFix msg;
    sensor_msgs::NavSatStatus navSatStatus;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    msg.status = navSatStatus;
    msg.header = header;
    msg.longitude = _sensor->Longitude().Degree();
    msg.latitude = _sensor->Latitude().Degree();
    msg.altitude = _sensor->Altitude();
    msg.position_covariance_type = msg.COVARIANCE_TYPE_UNKNOWN;
    // publish of fix topic according to the state
    switch(this->currentState){
      case FaultyGPSPlugin::State::LOSS_SIGNAL:
	
	break;
      case FaultyGPSPlugin::State::MULTIPATH:
	msg.latitude = msg.latitude + latTranslation;
	msg.longitude = msg.longitude + longTranslation;
	break;
      case FaultyGPSPlugin::State::NONE:
	
	break;
	
      case FaultyGPSPlugin::State::CORRECT:
	
	break;
    }
    
    this->changeStatus();
    
    // publish only if no loss of signal
    if(currentState!=FaultyGPSPlugin::State::LOSS_SIGNAL)
      pub.publish(msg);
}

void FaultyGPSPlugin::changeStatus(){
     // check for transition to another state accorfìding to the probability distribution
    // check for go back to none state if timer has finished
    if(this->currentState==FaultyGPSPlugin::State::NONE){
      // check if need to switch off
      double r = math::Rand::GetDblUniform();
      if(r < lossProb) {
	  std::cout << "Loss signal "<<std::endl;
	  //this->parentSensor->SetActive(false);
	  // start timer for counting time
	  this->StartTimer();
	  //change state
	  this->currentState=LOSS_SIGNAL;
	  
      }
      else{
	// sample from multipath probability
	double multiPathSample = math::Rand::GetDblUniform();
	if (multiPathSample < multiPathProb){
	  std::cout << "MultiPath "<<std::endl;
	  // start the timer
	  this->StartTimer();
	  //change state
	  this->currentState=MULTIPATH;
	  //compute translation, from -maxTranslation to +maxTranslation
	  latTranslation = (math::Rand::GetDblUniform()-0.5)*2*maxLatTranslation;
	  longTranslation = (math::Rand::GetDblUniform()-0.5)*2*maxLongTranslation;
	}
	else{
	  std::cout << "Correct position "<<std::endl;
	  this->currentState = CORRECT;
	  this->StartTimer();
	}
      }
    }
    else{
     // if timer is finished go back to NONE state 
     if(timer.GetElapsed() == 0){
	std::cout << "Timer finished "<<std::endl;
	timer.Stop();
	this->currentState = NONE;
     }
    } 
  
}

void FaultyGPSPlugin::StartTimer(){
      // generate a sample from a uniform probability between 5 and 10
      double time = (math::Rand::GetDblUniform()+5)*2;
      // start the timer
      timerDuration = gazebo::common::Time(time);
      timer = gazebo::common::Timer(timerDuration);
      timer.Start();
}

void FaultyGPSPlugin::OnWorldUpdate(const common::UpdateInfo & /*_info*/) { 
    
}
