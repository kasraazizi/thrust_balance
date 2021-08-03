/*Software License Agreement (BSD)

\file      thrustbalance_framework.cpp
\authors   Ali Shahrudi <alishahrudi@gmail.com>
           Kasra Azizi <kasraazizi1375@gmail.com>

\copyright Copyright (c) 2021, MRL UAV Lab (Qazvin Islamic Azad University), All rights reserved.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <tb_framework/tb_pwm.h>
#include <tb_framework/tb_feedback.h>
#include <sensor_msgs/JointState.h>
#include <regex>
#include <vector>

using namespace std;
using namespace serial;
Serial ser("/dev/ttyUSB0");

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    // initial the subscriber and the publisher
    feedback_pub_ = n_.advertise<tb_framework::tb_feedback>("/tb/feedback", 1000); // feedback publisher
    joint_pub_ = n_.advertise<sensor_msgs::JointState>("/tb/joint_states", 1000); // jointstate pulisher
    sub_ = n_.subscribe("/tb/cmd_pwm", 1000, &SubscribeAndPublish::callback, this); // pwm subscriber
  }

  vector<string> split (string s, string delimiter) {
    /*
     * fonction that get string and delimiter and
     * split the string and return vector of strings
     */
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != string::npos) {
      token = s.substr (pos_start, pos_end - pos_start);
      pos_start = pos_end + delim_len;
      res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
  }

  void callback(const tb_framework::tb_pwm::ConstPtr& msg){
    /*
     * callback function that takes subscribed pwm msg and send it
     * to arduino and return arduino response back that we published
     * given message to "/tb/feedback" and "/tb/joint_states" topic
     */

    regex wrong_message("wrong");
    regex wire_init_fail_message("wire");
    string pwm = "L" + to_string(msg->leftpwm) + "-" + to_string(msg->rightpwm) + "R" + "\n";
    ros::Time delay_time = ros::Time::now();
    ser.write(pwm);

    if (ser.available()){
      string imu_data = ser.readline();
      ser.flush();
      cout << imu_data;
      if (regex_search(imu_data, wrong_message) == false){
        vector<string> rool_split = split(imu_data, "L");
        vector<string> pwms_split = split(rool_split.back(), "R");
        vector<string> pwm_split = split(pwms_split[0], "-");

        //publishing feedback message
        tb_framework::tb_feedback pub_msg;
        pub_msg.frequency =(1 / (ros::Time::now().toSec() - delay_time.toSec()));
        pub_msg.roll = atof(rool_split[0].c_str());
        pub_msg.leftpwm = atoi(pwm_split[0].c_str());
        pub_msg.rightpwm = atoi(pwm_split.back().c_str());
        pub_msg.header.stamp = ros::Time::now();
        feedback_pub_.publish(pub_msg);

        //publishing joinstate message
        sensor_msgs::JointState joint_msg;
        joint_msg.position.push_back(((3.14 * atof(rool_split[0].c_str())) / 180));
        joint_msg.position.push_back(0);
        joint_msg.position.push_back(0);
        joint_msg.name.push_back("roll");
        joint_msg.name.push_back("right_motor_to_propeller");
        joint_msg.name.push_back("left_motor_to_propeller");
        joint_msg.header.stamp = ros::Time::now();
        joint_pub_.publish(joint_msg);
      }
    }
  }

private:
  //initial ros
  ros::NodeHandle n_;
  ros::Publisher feedback_pub_;
  ros::Publisher joint_pub_;
  ros::Subscriber sub_;
};

void init(){

  /*
   * fuction that wait for arduino initiating
   * and then breaks up;
   */

  // initial regex variables
  regex ok_messgae("imu OK\n");
  regex not_found_message("not found");
  regex wire_init_fail_message("wire");
  ser.setBaudrate(9600);
  int atemp = 0;
  int fail = 0;
  ROS_INFO("wait for initial ardiono !");
  while (true){
    // while recive ok from arduino
    if (ser.available()){
      string init_message =ser.readline();
      if (init_message != ""){

        if (regex_match(init_message, ok_messgae) == true){
          break;
        }
        else if (regex_search(init_message, not_found_message)== true){
          atemp++;
        }
        if (regex_search(init_message, wire_init_fail_message) == true){
          atemp++;
        }
        if (atemp > 3){
          //          cout << "fail to initiate arduino !!"<< endl;
          ROS_WARN("fail to initiate arduino !!");
          fail++;
          ser.close();
          ser.open();
        }
        if (fail > 3){
          //          cout << "All atemp fail to initiate arduino !!"<< endl;
          ROS_ERROR("All atemp fail to initiate arduino !!");
          exit(3);
        }
      }
    }
  }
}

int main(int argc, char **argv){
  // initial ros node
  ros::init(argc, argv, "tb_framwork");
  ser.flush();
  init();
  ROS_INFO("initialization sucsseced");
  SubscribeAndPublish SAPobj;
  ros::spin();
  return 0;
}
