#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <kinect_bridge2/KinectBodies.h>

class PersonBaseLink
{
public:
    typedef kinect_bridge2::KinectBodies _KinectBodiesMsg;

protected:
    ros::NodeHandle _nh_rel;
    ros::Subscriber _kinect_bodies_sub;
    std::string _source_frame_name;
    std::string _target_frame_name;

    tf::TransformBroadcaster _transform_broadcaster;
    tf::TransformListener _transform_listener;

public:
    PersonBaseLink( ros::NodeHandle & nh_rel )
    :
        _nh_rel( nh_rel ),
        _kinect_bodies_sub( _nh_rel.subscribe<_KinectBodiesMsg>( "kinect_bodies", 10, &PersonBaseLink::kinectBodiesCB, this ) ),
        _source_frame_name( "/world" ),
        _target_frame_name( "spine_base" )
    {
        //
    }

    void kinectBodiesCB( _KinectBodiesMsg::ConstPtr const & message )
    {
        // select tracked bodies
        // look up their TF frames relative to some source frame
        // calculate projection of target frame into source_frame's x-y plane (so zero out its Z value)
        // publish new base_link frame in original tf namespace

        auto const & bodies = message->bodies;

        // the bodies are necessarily packed in order and there will necessarily be a constant number of them
        // since bodies (but not necessarily joints) are packed regardless of tracking state
        // thus, the body's position in the vector will necessarily be identical to the corresponding tf skeleton index
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
                _transform_listener.lookupTransform( _source_frame_name, tf_frame_basename + _target_frame_name, ros::Time( 0 ), target_transform );

                // get the base_link vec projected into the source frame's plane
                tf::Vector3 base_link_vec( target_transform.getOrigin().getX(), target_transform.getOrigin().getY(), 0 );
                tf::Vector3 x_axis_unit_vec( 1, 0, 0 );
                // get an "orientation" vector in the direction of the x-axis of the target joint's orientation
                tf::Vector3 base_link_ori_vec( tf::Transform( target_transform.getRotation() ) * x_axis_unit_vec );
                // project the orientation vector to 2D
                tf::Vector3 base_link_ori_2d_unit_vec( tf::Vector3( base_link_ori_vec.getX(), base_link_ori_vec.getY(), 0 ).normalized() );

                // our 2D orientation is just the atan2 of the 2d orientation vector
                // our 3D orientation is the 2D orientation around the z axis
                tf::Quaternion base_link_quat( tf::Vector3( 0, 0, 1 ), atan2( base_link_ori_2d_unit_vec.getX(), -base_link_ori_2d_unit_vec.getY() ) );

                // publish base link transform
                _transform_broadcaster.sendTransform( tf::StampedTransform( tf::Transform( base_link_quat, base_link_vec ), ros::Time::now(), _source_frame_name, tf_frame_basename + "base_link" ) );
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
    ros::init( argc, argv, "person_base_link" );
    ros::NodeHandle nh_rel( "~" );

    PersonBaseLink person_base_link( nh_rel );
    person_base_link.spin();

    return 0;
}
