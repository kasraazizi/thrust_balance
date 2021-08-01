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
    pub_ = n_.advertise<tb_framework::tb_feedback>("/tb/feedback", 1000);

    sub_ = n_.subscribe("/tb/cmd_pwm", 1000, &SubscribeAndPublish::callback, this);
  }

  vector<string> split (string s, string delimiter) {
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
    regex wrong_message("wrong");
    regex wire_init_fail_message("wire");
    string pwm = "L" + to_string(msg->leftpwm) + "-" + to_string(msg->rightpwm) + "R" + "\n";
    ros::Time delay_time = ros::Time::now();
    ser.write(pwm);

    if (ser.available()){
      string imu_data = ser.readline();
      if (regex_search(imu_data, wrong_message) == false){
        vector<string> rool_split = split(imu_data, "L");
        vector<string> pwms_split = split(rool_split.back(), "R");
        vector<string> pwm_split = split(pwms_split[0], "-");
        tb_framework::tb_feedback pub_msg;
        pub_msg.frequency =(1 / (ros::Time::now().toSec() - delay_time.toSec()));
        pub_msg.roll = atof(rool_split[0].c_str());
        pub_msg.leftpwm = atoi(pwm_split[0].c_str());
        pub_msg.rightpwm = atoi(pwm_split.back().c_str());
        pub_msg.header.stamp = ros::Time::now();
        pub_.publish(pub_msg);
      }else if (regex_search(imu_data, wire_init_fail_message)) {
        cout << "wire error !!" << endl;
      }else {
        cout << "wrong input !!" << endl;
      }
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};


void init(){
  regex ok_messgae("imu OK\n");
  regex not_found_message("not found");
  regex wire_init_fail_message("wire");
  ser.setBaudrate(9600);
  int atemp = 0;
  int fail = 0;
  while (true){
    // whlie for recive initial messagess
    if (ser.available()){
      string init_message =ser.readline();
      if (init_message != ""){
        cout <<init_message;
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
          cout << "fail to initiate arduino !!"<< endl;
          fail++;
          ser.close();
          ser.open();
        }
        if (fail > 3){
          cout << "All atemp fail to initiate arduino !!"<< endl;
          exit(3);
        }
      }
    }
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "tb_framwork");
  ser.flush();
  init();
  SubscribeAndPublish SAPobj;
  ros::spin();
  return 0;
}
