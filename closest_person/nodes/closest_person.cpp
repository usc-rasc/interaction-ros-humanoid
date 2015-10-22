#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <kinect_bridge2/KinectBodies.h>

class ClosestPerson
{
public:
    typedef kinect_bridge2::KinectBodies _KinectBodiesMsg;

protected:
    ros::NodeHandle _nh_rel;
    ros::Subscriber _kinect_bodies_sub;
    std::string _source_frame_name;
    std::string _target_frame_name;
    std::string _world_frame_name;

    tf::TransformBroadcaster _transform_broadcaster;
    tf::TransformListener _transform_listener;

public:
    ClosestPerson( ros::NodeHandle & nh_rel )
    :
        _nh_rel( nh_rel ),
        _kinect_bodies_sub( _nh_rel.subscribe<_KinectBodiesMsg>( "kinect_bodies", 10, &ClosestPerson::kinectBodiesCB, this ) ),
        _source_frame_name( "/ava/base_link" ),
        _target_frame_name( "base_link" ),
        _world_frame_name( "/world" )
    {
        //
    }

    void kinectBodiesCB( _KinectBodiesMsg::ConstPtr const & message )
    {
        auto const & bodies = message->bodies;

        // the bodies are necessarily packed in order and there will necessarily be a constant number of them
        // since bodies (but not necessarily joints) are packed regardless of tracking state
        // thus, the body's position in the vector will necessarily be identical to the corresponding tf skeleton index
        float shortest_distance = std::numeric_limits<float>::max();
        tf::Transform closest_body_transform;

        bool person_found = false;

        for( size_t body_idx = 0; body_idx < bodies.size(); ++body_idx )
        {
            auto const & body = bodies[body_idx];

            if( body.is_tracked )
            {
                std::stringstream tf_frame_basename_ss;
                tf_frame_basename_ss << "/kinect_client/skeleton" << body_idx << "/";
                std::string tf_frame_basename( tf_frame_basename_ss.str() );

                tf::StampedTransform target_transform;
                // get transform from source to target
                try
                {
                    _transform_listener.lookupTransform( _source_frame_name, tf_frame_basename + _target_frame_name, ros::Time( 0 ), target_transform );
                }
                catch( tf::TransformException & e )
                {
                    ROS_WARN( "transform lookup failed: %s", e.what() );
                    continue;
                }

                float body_distance = target_transform.getOrigin().length();
                if( body_distance < shortest_distance )
                {
                    shortest_distance = body_distance;
                    closest_body_transform = target_transform;
                    person_found = true;
                }
            }
        }

        if( person_found )
        {
            tf::StampedTransform world_to_source_transform;
            try
            {
                _transform_listener.lookupTransform( _world_frame_name, _source_frame_name, ros::Time( 0 ), world_to_source_transform );
                // publish base link transform
                _transform_broadcaster.sendTransform( tf::StampedTransform( world_to_source_transform * closest_body_transform, ros::Time::now(), _world_frame_name, "/closest_person/" + _target_frame_name ) );
            }
            catch( tf::TransformException & e )
            {
                ROS_WARN( "transform lookup failed: %s", e.what() );
            }
        }
    }

    void spinOnce()
    {
        // all of our logic happens in callbacks
    }

    void spin()
    {
        // process callbacks at 30 Hz
        ros::Rate loop_rate( 30 );
        while( ros::ok() )
        {
            spinOnce();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "closest_person" );
    ros::NodeHandle nh_rel( "~" );

    ClosestPerson closest_person( nh_rel );
    closest_person.spin();

    return 0;
}
