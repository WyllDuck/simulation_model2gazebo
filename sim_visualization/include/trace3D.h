#ifndef __TRACE__
#define __TRACE__

#include "ros/ros.h"
#include <eigen3/Eigen/Core>

#include <geometry_msgs/Vector3Stamped.h>

#include <nav_msgs/Path.h>

using namespace std;

class Trace {

    private:

        ros::NodeHandle nh;

        size_t pos = 0; // position matrix storage
        unsigned int N = 100; // length trace
        
        unsigned int f = 10; // sampling frequency
        unsigned int n = 0;  // current sample

        Eigen::MatrixXd values = Eigen::MatrixXd(N, 3);

    public:

        ros::Publisher publisher;
        ros::Subscriber subscriber;

        Trace(string subTopic, string pubTopic);
        ~Trace();
        
        void callback(const geometry_msgs::Vector3Stamped::ConstPtr&);

};

#endif