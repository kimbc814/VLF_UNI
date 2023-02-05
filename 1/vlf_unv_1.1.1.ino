//VLF_대학 초음파 센서
#include <NewPing.h>
#include <Servo.h>

//서보모터
Servo servo;
#define NEURAL_ANGLE 85
#define LEFT_STEER_ANGLE  -40  // 실험으로 구할것
#define RIGHT_STEER_ANGLE  40  // 실험으로 구할



//초음파 센서 
#define L_TRIG 3
#define L_ECHO 8
#define F_TRIG 9
#define F_ECHO 10
#define R_TRIG 11
#define R_ECHO 12
//모터
#define MOTOR_DIR 4
#define MOTOR_PWM 5


#define maxDistance 800//cm

//초음파 데이터
float  F_distance = 0.0,L_distance = 0.0,R_distance = 0.0,
    F_distance_old = 0.0,L_distance_old = 0.0,R_distance_old = 0.0,
    F_distance_error = 0.0,L_distance_error = 0.0,R_distance_error = 0.0, distance_error=0.0;
//필터 데이터
int number_input = 0;
float presum_num = 0;


NewPing Sonar[3]={
NewPing(F_TRIG,F_ECHO,maxDistance),
NewPing(L_TRIG,L_ECHO,maxDistance),
NewPing(R_TRIG,R_ECHO,maxDistance)
};
void read_sonar_sensor(){
  F_distance = Sonar[0].ping_cm()*10;
  L_distance = Sonar[1].ping_cm()*10;
  R_distance = Sonar[2].ping_cm()*10;
  if(R_distance == 0) R_distance = maxDistance * 10.0;
  if(L_distance == 0) L_distance = maxDistance * 10.0;
  if(F_distance == 0) F_distance = maxDistance * 10.0;
  
  Serial.print("R_distance : ");
  Serial.print(R_distance);
  Serial.print("mm  ");
  
  Serial.print("F_distance : ");
  Serial.print(F_distance);
  Serial.print("mm  ");
  
  Serial.print("L_distance : ");
  Serial.print(L_distance);
  Serial.println("mm");
}
void update_sonar_old(){
  R_distance_old = R_distance;
  L_distance_old = L_distance;
  F_distance_old = F_distance;
  }
void update_sonar_error(){
  R_distance_error = R_distance - R_distance_old;
  L_distance_error = L_distance - L_distance_old;
  F_distance_error = F_distance - F_distance_old;
}
void update_sonar_error_RL(){
  distance_error = R_distance - L_distance;
}
void steering_control(int Steering_Angle){

  if(Steering_Angle<= LEFT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEURAL_ANGLE;

  if(Steering_Angle>= RIGHT_STEER_ANGLE + NEURAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEURAL_ANGLE;

  servo.write(Steering_Angle);  

}
float Recursive_Expression_Filter(int input_num){
	float resault = 0;
	number_input++;
	resault = (float)((presum_num + input_num) / number_input);
	presum_num += input_num;
	return resault;
}

void setup() {

pinMode(MOTOR_DIR,OUTPUT);
pinMode(MOTOR_PWM,OUTPUT);

  servo.attach(2);
  Serial.begin(115200);
  
}

void M_CTL(int speed){
	if(speed>0){
		int direction =1;
		digitalWrite(MOTOR_DIR,direction); //digitalWrite(MOTOR_DIR,0);
		analogWrite(MOTOR_PWM,speed);
	}
	else if(speed<0){
		int direction =0;
		speed=-speed;
		digitalWrite(MOTOR_DIR,direction); //digitalWrite(MOTOR_DIR,0);
		analogWrite(MOTOR_PWM,speed);
	}
	else {
		digitalWrite(MOTOR_DIR,1); //digitalWrite(MOTOR_DIR,0);
		analogWrite(MOTOR_PWM,0);
	}
}

void loop() {
    delay(500);
	 read_sonar_sensor();
      update_sonar_error();
      update_sonar_old();
      update_sonar_error_RL();
	
	M_CTL(50);

  steering_control(NEURAL_ANGLE *(Recursive_Expression_Filter(R_distance)/(float)150));


}