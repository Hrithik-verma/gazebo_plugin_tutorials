#include <gazebo/gazebo.hh>          // for accessing all gazebo classes
#include <gazebo/common/common.hh>   // for common fn in gazebo like ModelPlugin, event, GetName()
#include <gazebo/physics/physics.hh> // for gazebo physics, to access -- ModelPtr
#include <ignition/math/Vector3.hh>  // to access Vector3d() from ignition math class
#include <ignition/math/Color.hh>    // to access Color() from ignition math class

namespace gazebo
{
    class LightMoveModelPlugin : public ModelPlugin
    {

    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->model = _model;

            // initlize tranport node to publish on light msg
            transport::NodePtr node(new transport::Node());
            node->Init();

            // publish light state
            this->light_pub = node->Advertise<msgs::Light>("~/light/modify");

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&LightMoveModelPlugin::OnUpdate, this));

            //                                                    bind() is use to bind this & OnUpdate i.e this->OnUpdate
            //                            bind- we don't have define callback fn input parametes its replace by placeholder

            // get model name 
            model_name = model->GetName();

            // c++11 onwoards, "auto" - automatically detects and assigns a data type to the variable

            // method to get light name from sdf in move_light_model.world
            auto Link = this->model->GetSDF()->GetElement("link");
            this->Link_name = Link->Get<std::string>("name");
            auto sdfLight = Link->GetElement("light");
            this->light_name = sdfLight->Get<std::string>("name");

            // naming the complete light to publish
            complete_light_name = this->model_name + "::" + this->Link_name + "::" + this->light_name;
            std::cout << "complete_light_name=" << complete_light_name << "\n";
        }

        // fn to publish light color on ~/light/modify topic 
        void control_light(bool activate)
        {

            msgs::Light light_msg;

            light_msg.set_name(this->complete_light_name);

            if (activate)
            {
                // set light color to white
                msgs::Set(light_msg.mutable_diffuse(), ignition::math::Color(0.5, 0.5, 0.5, 1.0));
                std::cout << "light on \n";
            }
            else
            {
                // set light color to black
                msgs::Set(light_msg.mutable_diffuse(), ignition::math::Color(0.0, 0.0, 0.0, 1.0));
                std::cout << "light off \n";
            }
            // Send the message on topic
            this->light_pub->Publish(light_msg);
        }

        // keep on updating as simulation iterates
    public:
        void OnUpdate()
        {

            if (this->counter < 10000)
            {
                // move model to +x direction
                this->model->SetLinearVel(ignition::math::Vector3d(0.3, 0, 0));
            }

            if (this->counter == 10000) // if count is 10000
            {
                this->control_light(false); // turn off light
            }

            if (this->counter > 10000)
            {
                // move model to -x direction
                this->model->SetLinearVel(ignition::math::Vector3d(-0.3, 0, 0));
            }

            
            if (this->counter == 20000) //if count is 20000
            {
                this->control_light(true); // turn on light
            }

            if (this->counter > 20000)
            {   
                // reset count
                counter = 0;
            }

            this->counter++;
        }

    
    private:
        physics::ModelPtr model;  // Pointer to the model

    private:
        int counter;
        std::string model_name, Link_name, light_name;   // model, link & light name
        std::string complete_light_name;     // complete light name to publish 

    public:
        transport::PublisherPtr light_pub;  // light publisher

        
    private:
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(LightMoveModelPlugin)
} 