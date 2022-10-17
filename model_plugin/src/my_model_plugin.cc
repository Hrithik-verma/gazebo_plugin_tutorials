
#include <gazebo/gazebo.hh>                // for accessing all gazebo classes
#include <gazebo/common/common.hh>         // for common fn in gazebo like ModelPlugin, event
#include <gazebo/physics/physics.hh>       // for gazebo physics, to access -- ModelPtr
#include <ignition/math/Vector3.hh>        // to access Vector3d() from ignition math class

namespace gazebo {
class MyModelPlugin : public ModelPlugin {
    
public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    
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
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MyModelPlugin::OnUpdate, this));

                                                // bind() is use to bind this & OnUpdate i.e this->OnUpdate
  }                          //bind- we don't have define callback fn input parametes its replace by placeholder 

  //keep on updating as simulation iterates 
public:
  void OnUpdate() {
      

    if (this->count < 10000)  // move up till 10000 counts
    {
        // Apply a small linear velocity to the model.
        this->model->SetLinearVel(ignition::math::Vector3d(0, 0, this->vel));
        
    }

    this->count++;  // increment count
  }


// data members
private:
  physics::ModelPtr model;  // Pointer to the model

private:
  int count;             // to keep a count
  double vel;               // assign vel to model

  private:
  event::ConnectionPtr updateConnection;  // Pointer to the update event connection
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MyModelPlugin)
} 