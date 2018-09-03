#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <numeric>
using namespace std;
float sonar_raw;
bool flag=0;
std_msgs::Float32 KF_msg;
float K, obs;
float X_, X = 0, P = 1, P_, R;
ros::Subscriber sub;
ros::Publisher pub;
void cal_var(const std_msgs::Int16 &msg);

      
int main(int argc, char **argv){
  ros::init(argc, argv, "KF_node2");
  ros::NodeHandle nh;
  sub = nh.subscribe("sonarD", 10, cal_var);
  pub = nh.advertise<std_msgs::Float32>("sonarD_KF", 10);
  ros::spin();
  return 0;
}
void cal_var(const std_msgs::Int16 &msg){
  static int count = 0;
  static float Array[10];
  float sum, mean;
  Array[count] = msg.data * 0.000001 * 17000;
  //obs = Array[9];
  count++;
  if (count == 10){ 
    for (int i=0; i<10; i++) sum += Array[i];
    mean = sum * 0.1;
    for (int i=0; i<10; i++) R += ((Array[i] - mean) * (Array[i] - mean));
    R *= 0.1;
    count = 0;
    for (int i=0; i<10; i++) Array[i] = 0;
    //ROS_INFO("Variance: [%f]", R);
    float Q = 0.0001;
    for (int i=0; i<10; i++){
      X_ = X;
      P_ = P + Q;
      K = P_ / (P_ + R);
      X = X_ + K * ((msg.data * 0.000001 * 17000) - X_);
      P = (1 - K) * P_;
    }
      //ROS_INFO("Value %f", X);
    KF_msg.data = X;
    pub.publish(KF_msg);
    //flag = 0;
  }
  
}


/*void myKFCb(const std_msgs::Int16 &msg){
      float Q = 0.1, R, mean;
      for (int i=0; i<10; i++){
         Array[i]= msg.data * 0.000001 * 17000;
         mean += Array[i];
      }
      
      for (int i=0; i<10; i++){
         R += ((Array[i]-mean) * (Array[i]-mean));
      }
      R *= 0.1;
      sonar_raw = Array[10];
  
      for (int i=0; i<100; i++){
        X_ = X;
        P_ = P + Q;
        K = P_ / (P_ + R);
        X = X_ + K * (sonar_raw - X_);
        P = (1 - K) * P_;
      }
      //ROS_INFO("Value %f", X);
      KF_msg.data = (int) X;
      pub.publish(KF_msg);
      //cout << "Value" << X << endl;
}*/
