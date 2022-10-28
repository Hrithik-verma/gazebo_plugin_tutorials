
#include <gazebo/gazebo.hh>                // for accessing all gazebo classes
#include <gazebo/common/common.hh>         // for common fn in gazebo like ModelPlugin, event
#include <gazebo/physics/physics.hh>       // for gazebo physics, to access -- ModelPtr
#include <ignition/math/Vector3.hh>        // to access Vector3d() from ignition math class

namespace gazebo {
class JointControllerPlugin : public ModelPlugin {
    
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
    // Store the pointer to the model
    this->model = _model;


    // method to get joint 
    this->jointByNum = _model->GetJoints()[1];              // by number

    this->jointByName = _model->GetJoint("joint_1");        // by name

    std::cout<< "Joint By Number =" << this->jointByNum->GetScopedName() << std::endl;
    std::cout<< "Joint By Name   =" << this->jointByName->GetScopedName() << std::endl;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&JointControllerPlugin::OnUpdate, this));

                                                // bind() is use to bind this & OnUpdate i.e this->OnUpdate
                            //bind- we don't have define callback fn input parametes its replace by placeholder

    
  }


  //keep on updating as simulation iterates 
public:
  void OnUpdate() {

    //method 1
    this->model->GetJoint("joint_1")->SetVelocity(0, 2.0);


    //method 2 
    // this->model->GetJoint("joint_1")->SetParam("fmax", 0, 100.0);
    // this->model->GetJoint("joint_1")->SetParam("vel", 0, 1.0);

    //method 3    
    // if (count == 0)
    // {
    //  this->jointController.reset(new physics::JointController(this->model));  //reset joint controller
    //  this->jointController->AddJoint(this->model->GetJoint("joint_1"));  //add a joint to control
    //  std::string name = this->model->GetJoint("joint_1")->GetScopedName(); //get full joint name
    //  this->jointController->SetVelocityPID(name, common::PID(1, 0, 0)); //set pid for the joint
    //  this->jointController->SetVelocityTarget(name, 0.1);   // set target velocity for the joint

    //  this->jointController->Update();
    // }


    count++;
  }

// data members
private:
  physics::ModelPtr model;  // Pointer to the model
  physics::JointPtr jointByNum;  // pointer to the joint
  physics::JointPtr jointByName;  // pointer to the joint
  
public:
    int count = 0;

private:
  common::PID pid;   // to set joint pid values

public: physics::JointControllerPtr jointController; // pointer to a joint controller for PID

  private:
  event::ConnectionPtr updateConnection;  // Pointer to the update event connection
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(JointControllerPlugin)
} 