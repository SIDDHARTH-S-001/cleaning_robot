#include <ros.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int32.h>

#define Encoder_output_A_left 3 // pin2 of the Arduino
#define Encoder_output_B_left 4 // pin 3 of the Arduino
#define Encoder_output_A_right 2 // pin2 of the Arduino
#define Encoder_output_B_right 5 // pin 3 of the Arduino

int64_t data_array[2] = {0};

#define Motor_A_L 6
#define Motor_B_L 7
#define Motor_A_R 8
#define Motor_B_R 9

volatile int Count_pulses_left = 0;
volatile int Count_pulses_right = 0;

ros::NodeHandle nh;

std_msgs::Int64MultiArray motor_feedback_msg;
ros::Publisher motor_feedback_pub("/motor/feedback", &motor_feedback_msg);

void leftCmdCallback(const std_msgs::Int32& left_cmd_msg) {
  // Process left motor command
  int pwm = left_cmd_msg.data;
  if(pwm > 0){
    analogWrite(Motor_A_L, pwm);
    analogWrite(Motor_B_L, 0);
  }
  else{
    analogWrite(Motor_A_L, 0);
    analogWrite(Motor_B_L, abs(pwm));
  }
}

ros::Subscriber<std_msgs::Int32> left_cmd_sub("/motor/left_cmd", leftCmdCallback);

void rightCmdCallback(const std_msgs::Int32& right_cmd_msg) {
  // Process right motor command
  int pwm = right_cmd_msg.data;
//  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
  if(pwm > 0){
    analogWrite(Motor_A_R, pwm);
    analogWrite(Motor_B_R, 0);
  }
  else{
    analogWrite(Motor_A_R, 0);
    analogWrite(Motor_B_R, abs(pwm));
  }
}

ros::Subscriber<std_msgs::Int32> right_cmd_sub("/motor/right_cmd", rightCmdCallback);

void setup() {
  nh.initNode();
  nh.subscribe(left_cmd_sub);
  nh.subscribe(right_cmd_sub);
  nh.advertise(motor_feedback_pub);

  pinMode(Encoder_output_A_left,INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_B_left,INPUT); // sets the Encoder_output_B pin as the input
  attachInterrupt(digitalPinToInterrupt(Encoder_output_A_left),DC_Motor_Encoder_left,RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_A_right),DC_Motor_Encoder_right,RISING);
}

void loop() {
  motor_feedback_msg.data_length = 2;
  data_array[0] = Count_pulses_left;
  data_array[1] = Count_pulses_right;
  motor_feedback_msg.data = data_array;
  motor_feedback_pub.publish(&motor_feedback_msg);

  // Handle ROS communication
  nh.spinOnce();

}

void DC_Motor_Encoder_left(){
  int b = digitalRead(Encoder_output_B_left);
  if(b > 0){
    Count_pulses_left++;
  }
  else{
    Count_pulses_left--;
  }
}
void DC_Motor_Encoder_right(){
  int b = digitalRead(Encoder_output_B_right);
  if(b > 0){
    Count_pulses_right++;
  }
  else{
    Count_pulses_right--;
  }
}
