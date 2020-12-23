#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

#define MAX_MOVE_DELAY 600 // Время для максимального хода
#define MAX_PWM 70
#define START_PWM  90
#define MIN_PWM 110
#define STOP_PWM 90


int *state; // Храним значения

Servo servo_f_l; //1
Servo servo_b_l; //2
Servo servo_b_r; //3
Servo servo_f_r;//4

int count_servo = 4;
int img[4] = {0, 0, 0, 0} ;// должен быть динамическим
MPU6050 mpu;


void setup() {
  Serial.begin(115200);



  Serial.println("Initialize MPU6050");

  while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }





  for (int i = 0; i < count_servo; i++) { // ОТключаем все сервы че бы не жужали
    stop_servo(i);
  }

  state = new int[count_servo];
  for (int i = 0; i < count_servo; i++) {
    state[i] = 0;
  }

}


inline int* calculate_directions(int input[]) {
  int directions[count_servo];

  for (int i = 0; i < count_servo; i++) {
    int difference = input[i] - state[i];

    directions[i] = map(abs(difference), 0, 100, 0, MAX_MOVE_DELAY);


    if (difference < 0) {
      directions[i] = -directions[i];
    }
    Serial.print("Вывод функции ");
    Serial.println(directions[i]);
  }
  return directions;
}


inline void move_servo(int servo_id, bool is_to_down) {
  int pwm = (is_to_down) ? MIN_PWM : MAX_PWM;






  switch (servo_id) {
    case 0:
      servo_f_l.attach(6);
      servo_f_l.write(pwm);

      break;
    case 1:
      servo_b_l.attach(9);
      servo_b_l.write(pwm);
      break;
    case 2:
      servo_b_r.attach(10);
      servo_b_r.write(pwm);
      break;
    case 3:
      servo_f_r.attach(11);//
      servo_f_r.write(pwm);
      break;

  }

}


inline void stop_servo(int servo_id) {
  switch (servo_id) {
    case 0:
      servo_f_l.write(STOP_PWM);
      servo_f_l.detach();
      break;
    case 1:
      servo_b_l.write(STOP_PWM);
      servo_b_l.detach();
      break;
    case 2:
      servo_b_r.write(STOP_PWM);
      servo_b_r.detach();
      break;
    case 3:
      servo_f_r.write(STOP_PWM);
      servo_f_r.detach();//
      break;


  }
}


inline int get_max(int directions[]) {
  int result = directions[0];
  for (int i = 0; i < count_servo; i++) {
    int val = abs(directions[i]);
    result = (val > result) ? val : result;
  }
  return result;
}


void move_platform(int directions[]) {
  unsigned long start_time = millis();
  unsigned long end_time = millis() + get_max(directions) + 10;

  for (int i = 0; i < count_servo; i++) {
    int elem = directions[i];
    bool is_to_down = (elem < 0);
    if ( elem != 0 ) {
      move_servo(i, is_to_down);
    }
  }

  while (millis() < end_time) {
    for (int i = 0; i < count_servo; i++) {
      int rotate_time = abs(directions[i]);
      if (millis() > start_time + rotate_time) {
        stop_servo(i);
      }
    }
  }

  for (int i = 0; i < count_servo; i++) {
    stop_servo(i);
  }

}



void loop() {

  // Read normalized values
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & Roll
  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;

  
  Serial.print("TRUE ROLL is - ");
  Serial.println(roll);
 int step_size = 10;
 int  trash = 4;

 if (roll>trash || roll < -trash){//Если roll не попадает под параметры тогда принимаем меры
  if (roll<-trash){
    img[0] =  (state[0] + step_size  <= 101) ? state[0]+step_size : 101 ;
    img[1] = (state[1] + step_size  <= 101) ? state[1]+step_size : 101 ;
    img[2] = (state[2] - step_size  >= 1) ? state[2]-step_size : 1 ;
    img[3] = (state[3] - step_size  >= 1) ? state[3]-step_size : 1 ;

  }
  if (roll > trash){
    img[2] = (state[2] + step_size  <= 101) ? state[2]+step_size : 101 ;
    img[3] = (state[3] + step_size  <= 101) ? state[3]+step_size : 101 ;
    img[0] = (state[0] - step_size  >= 1) ? state[0]-step_size : 1 ;
    img[1] = (state[1] - step_size  >= 1) ? state[1]-step_size : 1 ;
  }
  
 }

 if (pitch>trash || pitch < -trash){
  if (pitch<-trash){
    img[1] =  (state[1] + step_size  <= 101) ? state[1]+step_size : 101 ;// 0 и 3 я опускаю а 2 и 1 поднимаю
    img[2] = (state[2] + step_size  <= 101) ? state[2]+step_size : 101 ;
    img[3] = (state[3] - step_size  >= 1) ? state[3]-step_size : 1 ;
    img[0] = (state[0] - step_size  >= 1) ? state[0]-step_size : 1 ;

  }
  
  if (pitch > trash){
    img[0] = (state[0] + step_size  <= 101) ? state[0]+step_size : 101 ;//1  и 2 опускаю а 0 и 3 поднимаю
    img[3] = (state[3] + step_size  <= 101) ? state[3]+step_size : 101 ;
    img[1] = (state[1] - step_size  >= 1) ? state[1]-step_size : 1 ;
    img[2] = (state[2] - step_size  >= 1) ? state[2]-step_size : 1 ;
  }
  
  
  }

 
  int *computed_array = calculate_directions(img);
  int copy_array[4];


  for (int i = 0; i < count_servo; i++) { //Это для проверки того что рассчитывает функция
    Serial.println (copy_array[i]);
    //  	Serial.print(": ");
    //  	Serial.println(*(computed_array + i));
    
  }
//  Serial.println("Computed array");

  for (int i = 0; i < count_servo; i++) {// Здесь мы запоминаем состояние
    state[i] = img[i];
 
  }

    move_platform(copy_array);
  delay(500);
}
