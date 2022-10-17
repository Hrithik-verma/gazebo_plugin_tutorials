#include <gazebo/gazebo.hh>          // for accessing all gazebo classes
#include <gazebo/common/common.hh>   // for common fn in gazebo like ModelPlugin, event, GetName()
#include <gazebo/physics/physics.hh> // for gazebo physics, to access -- ModelPtr
#include <ignition/math/Vector3.hh>  // to access Vector3d() from ignition math class
#include <ignition/math/Color.hh>    // to access Color() from ignition math class


#include <ros/ros.h>                // for acceessing ros function
#include <std_msgs/String.h>        // to store ros sub msg

#include <functional>                     // to access boost::bind()
#include <thread>                        // to use multithreading
#include "ros/callback_queue.h"         // for ros callback queue
#include "ros/subscribe_options.h"     // to access SubscribeOptions


namespace gazebo
{
    class LightController_ROS_Plugin : public ModelPlugin
    {

    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {

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

            ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::String>("/light_color", 
                                        1, boost::bind(&LightController_ROS_Plugin::ColourCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
            //                                                          this->Activate_Callback

            //     VoidPtr() - if the reference count goes to 0 the subscriber callbacks will not get called

            this->sub_light = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
            std::thread(std::bind(&LightController_ROS_Plugin::QueueThread, this));   //c++ threading to keep 
                                                //this->QueueThread       this->QueueThread() fn running

            // store model pointer 
            this->model = _model;

            // initlize tranport node
            transport::NodePtr node(new transport::Node());
            node->Init();

            // publish gazebo light state
            this->light_pub = node->Advertise<msgs::Light>("~/light/modify");

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&LightController_ROS_Plugin::OnUpdate, this));

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

    // ROS helper function that processes messages  
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
        std::unique_ptr<ros::NodeHandle> rosNode;         // ros node handler pointer
        ros::CallbackQueue rosQueue;                      // rosqueue
        std::thread rosQueueThread;                       // rosqueue thread
        ros::Subscriber sub_light;     // ros sub
        std::string light_colour_name; // light colour
    };
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(LightController_ROS_Plugin)
}