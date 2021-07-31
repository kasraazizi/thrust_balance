#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <tb_framework/tb_pwm.h>
#include <tb_framework/tb_feedback.h>
#include <regex>
using namespace std;
using namespace serial;



Serial ser("/dev/ttyUSB0");


void felan(const tb_framework::tb_pwm::ConstPtr& msg){
  string pwm = "L" + to_string(msg->leftpwm) + "-" + to_string(msg->rightpwm) + "R" + "\n";
  ser.write(pwm);
    if (ser.available()){
        cout <<ser.readline();
    }
}



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
    ros::init(argc, argv, "tb_framwork_subscriber");
    ros::NodeHandle n;
    ser.flush();
    init();
//    ser.flushInput();
//    ser.flushOutput();
    ros::Subscriber sub = n.subscribe("/tb/cmd_pwm", 1000, felan);
    ros::spin();
    return 0;
  }
