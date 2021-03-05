#include <trace3D.h>

Trace::Trace (string subTopic, string pubTopic){
    
    // Publishers
    publisher = nh.advertise<nav_msgs::Path>(pubTopic, 1);    
    subscriber = nh.subscribe(subTopic, 1, &Trace::callback, this);

    // Set Storage Matrix to Zero
    for (int i = 0; i < values.rows(); i++){
        for (int j = 0; j < values.cols(); j++){
            values(i, j) = 0.0;
        }
    }
}

Trace::~Trace (){

}

void Trace::callback(const geometry_msgs::Vector3Stamped::ConstPtr &data){

    // Frequency control
    n += 1;
    if (n % f != 0) return;
    else n = 0;

    // Init Messages
    geometry_msgs::PoseStamped msg_beta;

    nav_msgs::Path msg = nav_msgs::Path();
    msg.header = data->header;

    // Save Data
    if (data->vector.x > -5000 && data->vector.x < 5000) values(pos, 0) = data->vector.x;
    else return;
    if (data->vector.y > -5000 && data->vector.y < 5000) values(pos, 1) = data->vector.y;
    else return;
    if (data->vector.z  > -0.1 && data->vector.y < 5000) values(pos, 2) = data->vector.z;
    else return;

    // Fill Message ROS
    for (unsigned int i = pos + 1; i < N; i++){
        msg_beta.pose.position.x = values(i,0);
        msg_beta.pose.position.y = values(i,1);
        msg_beta.pose.position.z = values(i,2);

        msg.poses.push_back(msg_beta);
    }

    for (unsigned int i = 0; i < pos ; i++){

        msg_beta.pose.position.x = values(i,0);
        msg_beta.pose.position.y = values(i,1);
        msg_beta.pose.position.z = values(i,2);

        msg.poses.push_back(msg_beta);
    }

    // Update Init Position Storage Matrix
    pos++;
    if (pos >= N){
        pos = 0;
    }

    // Publish
    publisher.publish(msg);
}