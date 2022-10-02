
#include <gazebo/gazebo.hh>                // for accessing all gazebo classes
#include <gazebo/common/common.hh>         // for common fn in gazebo like ModelPlugin, event
#include <gazebo/physics/physics.hh>       // for gazebo physics, to access -- ModelPtr
#include <ignition/math/Vector3.hh>        // to access Vector3d() from ignition math class
#include <ros/ros.h>                     // for acceessing ros
#include <std_msgs/Bool.h>                // std_msgs/Bool for ros

namespace gazebo {
class SimpleModelRosPlugin : public ModelPlugin {
    
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
        return;
    }

    ROS_INFO("ROS Model Plugin Loaded!");

    this->sub = this->nh.subscribe("/model_move_up", 10, &SimpleModelRosPlugin::Activate_Callback, this);
    //                             topic name , queue size, callback, this pointer(to access class member fn) 

    // Store the pointer to the model
    this->model = _model;

    std::cout<< "Model Name=" << this->model->GetName() << std::endl;


    this->vel = 0.1;         // assign a default value
    if (_sdf->HasElement("model_vel")) // check if element existence 
    {
        this->vel = _sdf->Get<double>("model_vel");  // use _sdf pointer & Get to find value in <model_vel>
        
    }
    std::cout << "model_vel= " << this->vel << std::endl;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&SimpleModelRosPlugin::OnUpdate, this));

     //                                          bind() is use to bind this & OnUpdate i.e this->OnUpdate
  } //                         bind- we don't have define callback fn input parametes its replace by placeholder 


public:
  void Activate_Callback(const std_msgs::Bool::ConstPtr& msg){

    ROS_INFO("Received Message = %d", msg->data);

    this->activate_move = msg->data;
  }

  //keep on updating as simulation iterates 
public:
  void OnUpdate() {

    //once rostopic true than it move up for 1000 count

    if (this->activate_move) {     // if ros topic model_move_up recieves true

        if (this->count < 10000)  // move up till 10000 counts
        {
            // Apply a small linear velocity to the model.
            this->model->SetLinearVel(ignition::math::Vector3d(0, 0, this->vel));
            
        }
        else{

            //reset values
            activate_move = false;
            count = 0;
        }

        this->count++;  // increment count
    }
  }

// data members
private:
  physics::ModelPtr model;  // Pointer to the model

private:
  int count;             // to keep a count
  double vel;               // assign vel to model

  private:
  event::ConnectionPtr updateConnection;  // Pointer to the update event connection

  private:
  ros::NodeHandle nh;           // ros node handler
  ros::Subscriber sub;         // ros subscriber
  bool activate_move;    // to store boolen ros callback data
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SimpleModelRosPlugin)
} 