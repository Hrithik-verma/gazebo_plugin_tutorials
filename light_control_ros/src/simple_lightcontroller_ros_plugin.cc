#include <gazebo/gazebo.hh>          // for accessing all gazebo classes
#include <gazebo/common/common.hh>   // for common fn in gazebo like ModelPlugin, event, GetName()
#include <gazebo/physics/physics.hh> // for gazebo physics, to access -- ModelPtr
#include <ignition/math/Vector3.hh>  // to access Vector3d() from ignition math class
#include <ignition/math/Color.hh>    // to access Color() from ignition math class

#include <ros/ros.h>                // for acceessing ros function
#include <std_msgs/String.h>        // to store ros sub msg

namespace gazebo
{
    class Simple_LightController_ROS_Plugin : public ModelPlugin
    {

    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {

            if (!ros::isInitialized()) // check if ros is initialized or not
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized");
                return;
            }

            ROS_INFO("ROS Model Plugin Loaded!");

            this->sub_light = ros_node.subscribe("/light_color", 1, &Simple_LightController_ROS_Plugin::ColourCallback, this);
            //                                    topic name , queue size, callback, this pointer(to access class member fn)

            // store model pointer 
            this->model = _model;

            // initlize tranport node
            transport::NodePtr node(new transport::Node());
            node->Init();

            // publish gazebo light state
            this->light_pub = node->Advertise<msgs::Light>("~/light/modify");

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Simple_LightController_ROS_Plugin::OnUpdate, this));

            //                                                      bind() is use to bind this & OnUpdate i.e this->OnUpdate
            //                              bind- we don't have define callback fn input parametes its replace by placeholder

            // get model name
            this->model_name = model->GetName();

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

    public:
        // fn to publish light color on ~/light/modify topic
        void control_light(std::string colour)
        {
            msgs::Light light_msg;

            light_msg.set_name(this->complete_light_name);

            if (colour == "red")  // if color is red
            {
                //                                                         Red,Green,Blue,Alpha
                msgs::Set(light_msg.mutable_diffuse(), ignition::math::Color(1.0, 0, 0.0, 1.0));    
            }

            if (colour == "green")   // if color is green
            {
                //                                                          Red,Green,Blue,Alpha
                msgs::Set(light_msg.mutable_diffuse(), ignition::math::Color(0.0, 1.0, 0.0, 1.0));
            }

            if (colour == "blue")   // if color is green
            {
                //                                                          Red,Green,Blue,Alpha
                msgs::Set(light_msg.mutable_diffuse(), ignition::math::Color(0.0, 0.0, 1.0, 1.0));
            }
            // Send the message
            this->light_pub->Publish(light_msg);
        }

        //ros callback function
        void ColourCallback(const std_msgs::String::ConstPtr &msg)
        {

            ROS_INFO("Received [%s]", msg->data.c_str());

            // store msg data on a data member
            this->light_colour_name = msg->data;
        }

    public:
        void OnUpdate()
        {
            // color to publish on gazebo light
            this->control_light(this->light_colour_name);
        }

    private:
        physics::ModelPtr model; // Pointer to the model

    private:
        int counter;
        std::string model_name, Link_name, light_name; // model, link & light name
        std::string complete_light_name;               // complete light name to pub on gazebo

    public:
        transport::PublisherPtr light_pub;     // light gazebo publisher

    private:
        event::ConnectionPtr updateConnection; // Pointer to the update event connect

    private:
        ros::NodeHandle ros_node;      // ros node handler
        ros::Subscriber sub_light;     // ros sub
        std::string light_colour_name; // light colour
    };
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Simple_LightController_ROS_Plugin)
}