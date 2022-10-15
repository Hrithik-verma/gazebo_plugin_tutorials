#include <gazebo/gazebo.hh>                // for accessing all gazebo classes
#include <gazebo/common/common.hh>         // for common fn in gazebo like WorldPlugin()
#include <gazebo/physics/physics.hh>       // for gazebo physics, to access -- WorldPtr
#include <ignition/math/Pose3.hh>          // for accessing -- ignition math Pose3d()

namespace gazebo {
    class WorldGazeboPlugin : public WorldPlugin {
      public:
        WorldGazeboPlugin() : WorldPlugin() {
            printf("World Plugin Created!\n");
        }

      public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

            //_world pointer can access the world name using Name() fn
            std::cout << "World name = " << _world->Name() << std::endl;

            // set a node to publish
            transport::NodePtr node(new transport::Node());
            node->Init(_world->Name());

            // set publisher
            transport::PublisherPtr publisher =
                node->Advertise<msgs::Factory>("~/factory");

            // create msg obj
            msgs::Factory msg;

            // model to use
            msg.set_sdf_filename("model://table");


            // set model pose
            msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(1.0, 1.0, 0.0, 0.0, 0.0, 0.0));


            // Send the message
            publisher->Publish(msg);


            // other method 
            //Insert model from file via function call
            // _world->InsertModelFile("model://cylinder");

            // another method 
            // Insert a sphere model from string via function call
            sdf::SDF sphereSDF;
            sphereSDF.SetFromString(
            "<sdf version ='1.6'>\
                <model name ='sphere'>\
                    <pose>1 0 0 0 0 0</pose>\
                    <link name ='link'>\
                    <pose>0 0 .5 0 0 0</pose>\
                    <collision name ='collision'>\
                        <geometry>\
                        <sphere><radius>0.5</radius></sphere>\
                        </geometry>\
                    </collision>\
                    <visual name ='visual'>\
                        <geometry>\
                        <sphere><radius>0.5</radius></sphere>\
                        </geometry>\
                    </visual>\
                    </link>\
                </model>\
                </sdf>");
            // Demonstrate using a custom model name.
            sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
            model->GetAttribute("name")->SetFromString("unique_sphere");
            _world->InsertModelSDF(sphereSDF);

        }        
    };

    // Register plugin 
    GZ_REGISTER_WORLD_PLUGIN(WorldGazeboPlugin)
}
