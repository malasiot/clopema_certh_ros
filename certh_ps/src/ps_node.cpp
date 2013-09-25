#include <ros/ros.h>
#include <ros/single_subscriber_publisher.h>
#include <certh_ps/PhotometricStereo.h>
#include <iostream>

bool do_reconstruct(certh_ps::PhotometricStereo::Request &req,   certh_ps::PhotometricStereo::Response &res) {
   return true;
}


class PhotometricStereoServer {
public:
    PhotometricStereoServer() {

        ros::NodeHandle nh("~");
        ros::ServiceServer service = nh.advertiseService("reconstruct", &PhotometricStereoServer::do_reconstruct, this);

        pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1,
                                                      boost::bind(&PhotometricStereoServer::connectCallback, this, _1),
                                                      boost::bind(&PhotometricStereoServer::disconnectCallback, this, _1)) ;


    }

private:

    bool do_reconstruct(certh_ps::PhotometricStereo::Request &req,   certh_ps::PhotometricStereo::Response &res) {
       return true;
    }


    void connectCallback(const ros::SingleSubscriberPublisher &pub_) {

        if ( pub.getNumSubscribers() > 0 )
        {



        }
    }

    void disconnectCallback(const ros::SingleSubscriberPublisher &pub) {

    }

    ros::Publisher pub ;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "photometric_stereo");

  PhotometricStereoServer server ;

  ros::spin();

  return 0;
}
