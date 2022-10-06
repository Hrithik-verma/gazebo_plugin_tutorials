
#include <gazebo/gazebo.hh>                // for accessing all gazebo classes
#include <gazebo/common/common.hh>         // for common fn in gazebo like ModelPlugin, event
#include <gazebo/physics/physics.hh>       // for gazebo physics, to access -- ModelPtr
#include <ignition/math/Vector3.hh>        // to access Vector3d() from ignition math class
#include <ros/ros.h>                      // for acceessing ros
#include <std_msgs/Bool.h>                // std_msgs/Bool for ros

#include <functional>                     // to access boost::bind()
#include <thread>                        // to use multithreading
#include "ros/callback_queue.h"         // for ros callback queue
#include "ros/subscribe_options.h"     // to access SubscribeOptions

namespace gazebo {
class ModelRosPlugin : public ModelPlugin {
    
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    // chech if ros is initized or not
        if (!ros::isInitialized())
        {
            int argc = 0; 
            char **argv = NULL;
            // good pratice of init ros node 
            ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
        }

    ROS_INFO("ROS Model Plugin Loaded!");

    //Create our ROS node. This acts in a similar manner to gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    // subscribeoptions help to better manage multisubscriber (multithreading) 

    ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>("/model_move_up", 
                                        1, boost::bind(&ModelRosPlugin::Activate_Callback, this, _1), ros::VoidPtr(), &this->rosQueue);
    //                                                          this->Activate_Callback

    //     VoidPtr() - if the reference count goes to 0 the subscriber callbacks will not get called

    this->sub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
    std::thread(std::bind(&ModelRosPlugin::QueueThread, this));   //c++ threading to keep 
                                         //this->QueueThread       this->QueueThread() fn running

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
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelRosPlugin::OnUpdate, this));

                                                // bind() is use to bind this & OnUpdate i.e this->OnUpdate
  }                          //bind- we don't have define callback fn input parametes its replace by placeholder 


  /// ROS helper function that processes messages  
 private: void QueueThread()           // here we have define till what it will spin 
    {                                       
    static const double timeout = 0.01;
        while (this->rosNode->ok())    // while rosnode exist 
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));  //invoke callback 
        //                                                              after check avaibility  
                                                                   
        }
    }

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
  std::unique_ptr<ros::NodeHandle> rosNode;         // ros node handler pointer
  ros::CallbackQueue rosQueue;                      // rosqueue
  std::thread rosQueueThread;                       // rosqueue thread
  ros::Subscriber sub;                             // ros subscriber
  bool activate_move;                             // to store boolen ros callback data
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelRosPlugin)
} 