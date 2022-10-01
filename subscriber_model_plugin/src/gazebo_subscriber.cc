#include <gazebo/gazebo.hh>                // for accessing all gazebo classes
#include <gazebo/common/common.hh>         // for common fn in gazebo like ModelPlugin
#include <gazebo/physics/physics.hh>       // for gazebo physics, to access -- ModelPtr



namespace gazebo {
    class SubscriberGazeboPlugin : public ModelPlugin {
      public:
        SubscriberGazeboPlugin() : ModelPlugin() {
            printf("Subscriber Plugin Created!\n");
        }

      public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            
            // using _model pointer & GetName()to get model name
            std::cout << "Model Name = " << _model->GetName() << std::endl;

            // set a node to subscribe
            transport::NodePtr node(new transport::Node());
            node->Init();

            // subscribe to topic 
            //transport::SubscriberPtr subscri 
            this->sub = node->Subscribe("~/world_stats", &SubscriberGazeboPlugin::On_msg, this);

                                                               //equavalent: this->On_msg
        }
    
      public:
       void On_msg(ConstWorldStatisticsPtr &_msg)
        {
          
          // Dump the message contents to stdout.
          std::cout << _msg->DebugString();  //print output in the terminal
          
        }

     // good pratices to declare pub/sub as data member
     // reather than declaring them as local varibales which may cause issues
    private:
        transport::SubscriberPtr sub; 
    };
    // Register plugin 
    GZ_REGISTER_MODEL_PLUGIN(SubscriberGazeboPlugin)
}