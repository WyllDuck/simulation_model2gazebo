/*
 * Copyright (c) 2018 Authors:
 *   - Félix Martí Valverde <martivalverde@hotmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INPUTS_HH
#define INPUTS_HH

// ROS
#include <ros/ros.h>

// ROS Msgs
#include <std_msgs/Float32.h>

// Includes
#include <eigen3/Eigen/Dense>
#include <thread>

class Inputs
{

public:
    Inputs();

    ~Inputs();

    void Init(boost::shared_ptr<ros::NodeHandle> &_nh);

    Eigen::VectorXd inputs;

private:
    boost::shared_ptr<ros::NodeHandle> nh;

    ros::Subscriber sub1, sub2, sub3, sub4, sub5, sub6;
    ros::Subscriber sub7, sub8, sub9, sub10, sub11, sub12;

    void callback_cmd1(const std_msgs::Float32::ConstPtr &msg) { inputs[0] = msg->data; }
    void callback_cmd2(const std_msgs::Float32::ConstPtr &msg) { inputs[1] = msg->data; }
    void callback_cmd3(const std_msgs::Float32::ConstPtr &msg) { inputs[2] = msg->data; }
    void callback_cmd4(const std_msgs::Float32::ConstPtr &msg) { inputs[3] = msg->data; }
    void callback_cmd5(const std_msgs::Float32::ConstPtr &msg) { inputs[4] = msg->data; }
    void callback_cmd6(const std_msgs::Float32::ConstPtr &msg) { inputs[5] = msg->data; }

    void callback_cmd7(const std_msgs::Float32::ConstPtr &msg) { inputs[6] = msg->data; }
    void callback_cmd8(const std_msgs::Float32::ConstPtr &msg) { inputs[7] = msg->data; }
    void callback_cmd9(const std_msgs::Float32::ConstPtr &msg) { inputs[8] = msg->data; }
    void callback_cmd10(const std_msgs::Float32::ConstPtr &msg) { inputs[9] = msg->data; }
    void callback_cmd11(const std_msgs::Float32::ConstPtr &msg) { inputs[10] = msg->data; }
    void callback_cmd12(const std_msgs::Float32::ConstPtr &msg) { inputs[11] = msg->data; }
};

Inputs::Inputs(){
    
    // Inputs
    inputs.resize(12);
}

void Inputs::Init(boost::shared_ptr<ros::NodeHandle> &_nh)
{

    this->nh = _nh;

    // Subscribers
    sub1 = nh->subscribe<std_msgs::Float32>("/command/1", 1, &Inputs::callback_cmd1, this);
    sub2 = nh->subscribe<std_msgs::Float32>("/command/2", 1, &Inputs::callback_cmd2, this);
    sub3 = nh->subscribe<std_msgs::Float32>("/command/3", 1, &Inputs::callback_cmd3, this);
    sub4 = nh->subscribe<std_msgs::Float32>("/command/4", 1, &Inputs::callback_cmd4, this);
    sub5 = nh->subscribe<std_msgs::Float32>("/command/5", 1, &Inputs::callback_cmd5, this);
    sub6 = nh->subscribe<std_msgs::Float32>("/command/6", 1, &Inputs::callback_cmd6, this);

    sub7 = nh->subscribe<std_msgs::Float32>("/command/7", 1, &Inputs::callback_cmd7, this);
    sub8 = nh->subscribe<std_msgs::Float32>("/command/8", 1, &Inputs::callback_cmd8, this);
    sub9 = nh->subscribe<std_msgs::Float32>("/command/9", 1, &Inputs::callback_cmd9, this);
    sub10 = nh->subscribe<std_msgs::Float32>("/command/10", 1, &Inputs::callback_cmd10, this);
    sub11 = nh->subscribe<std_msgs::Float32>("/command/11", 1, &Inputs::callback_cmd11, this);
    sub12 = nh->subscribe<std_msgs::Float32>("/command/12", 1, &Inputs::callback_cmd12, this);

}

#endif // INPUTS_HH
