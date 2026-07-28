#include <Arduino.h>
#include <Servo.h>

#define J1 A0
#define J2 A1
#define J3 A2

#define IN1J1 6
#define IN2J1 7

#define IN1J2 3
#define IN2J2 2

#define IN1J3 5
#define IN2J3 4


float kp1=2;
float kd1=0;
float ki1=0;
float prev_error_1=0;
float inte_1=0;

float kp2=2;
float kd2=8;
float ki2=0;
float prev_error_2=0;
float inte_2=0;


float kp3=3;
float kd3=10;
float ki3=0;
float prev_error_3=0;
float inte_3=0;

double t2[100] = {
58.735650,
58.447054,
58.157998,
57.868460,
57.578419,
57.287856,
56.996748,
56.705075,
56.412815,
56.119946,
55.826446,
55.532292,
55.237462,
54.941931,
54.645676,
54.348674,
54.050899,
53.752327,
53.452932,
53.152688,
52.851569,
52.549548,
52.246596,
51.942686,
51.637790,
51.331876,
51.024916,
50.716877,
50.407730,
50.097440,
49.785975,
49.473300,
49.159380,
48.844180,
48.527661,
48.209786,
47.890516,
47.569808,
47.247623,
46.923917,
46.598644,
46.271760,
45.943217,
45.612965,
45.280954,
44.947130,
44.611440,
44.273826,
43.934230,
43.592591,
43.248846,
42.902927,
42.554766,
42.204292,
41.851430,
41.496102,
41.138225,
40.777716,
40.414483,
40.048435,
39.679471,
39.307490,
38.932383,
38.554035,
38.172326,
37.787129,
37.398310,
37.005727,
36.609229,
36.208657,
35.803841,
35.394600,
34.980743,
34.562062,
34.138338,
33.709336,
33.274801,
32.834460,
32.388020,
31.935160,
31.475535,
31.008768,
30.534449,
30.052128,
29.561312,
29.061455,
28.551955,
28.032141,
27.501266,
26.958487,
26.402856,
25.833293,
25.248564,
24.647244,
24.027677,
23.387918,
22.725654,
22.038106,
21.321878,
20.572750,
};

double t3[100] = {
-96.912216,
-96.438546,
-95.963063,
-95.485740,
-95.006548,
-94.525458,
-94.042441,
-93.557464,
-93.070497,
-92.581508,
-92.090464,
-91.597331,
-91.102073,
-90.604656,
-90.105043,
-89.603195,
-89.099075,
-88.592643,
-88.083857,
-87.572677,
-87.059058,
-86.542956,
-86.024326,
-85.503121,
-84.979292,
-84.452790,
-83.923562,
-83.391557,
-82.856719,
-82.318993,
-81.778320,
-81.234641,
-80.687893,
-80.138012,
-79.584934,
-79.028588,
-78.468905,
-77.905812,
-77.339232,
-76.769088,
-76.195298,
-75.617779,
-75.036442,
-74.451197,
-73.861950,
-73.268604,
-72.671056,
-72.069200,
-71.462928,
-70.852123,
-70.236667,
-69.616434,
-68.991295,
-68.361113,
-67.725747,
-67.085047,
-66.438858,
-65.787018,
-65.129354,
-64.465687,
-63.795828,
-63.119579,
-62.436731,
-61.747063,
-61.050342,
-60.346323,
-59.634745,
-58.915334,
-58.187797,
-57.451823,
-56.707083,
-55.953224,
-55.189873,
-54.416626,
-53.633055,
-52.838699,
-52.033062,
-51.215609,
-50.385764,
-49.542902,
-48.686346,
-47.815357,
-46.929130,
-46.026783,
-45.107345,
-44.169749,
-43.212810,
-42.235210,
-41.235479,
-40.211963,
-39.162795,
-38.085854,
-36.978709,
-35.838560,
-34.662148,
-33.445642,
-32.184495,
-30.873233,
-29.505173,
-28.072008,
};


void move_joint(int pin, float set, int IN1, int IN2, float &prev_error, float &inte, float kp, float kd, float ki){
  float error=0;
  if (pin == J1){
    error = -set + map(analogRead(pin), 0, 1023, 230, 20);
  }
  else if (pin ==J2){
    error = -set + map(analogRead(A1), 156, 1023, 210, 30)-15;
  }
  else{
    error = -set + map(analogRead(A2), 0, 1023, 110, -120);
  }
  int out = kp*error + kd*(error-prev_error)+ki*inte;
  // if (abs(error) >= 5)
  //   out = kp*error + kd*(error-prev_error)+ki*inte;
  // else{
  //   out = 0;
  // }
  out = constrain(out, -50, 50);
  Serial.println(out);
  if(out<=0){
    analogWrite(IN1, abs(out));
    digitalWrite(IN2, LOW);
  }
  else{
    analogWrite(IN1, abs(out));
    digitalWrite(IN2, HIGH);
  }
  prev_error = error;
  inte+=error;
  
}

void setup() {

  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  pinMode(IN1J1,OUTPUT);
  pinMode(IN2J1,OUTPUT);

  pinMode(IN1J2,OUTPUT);
  pinMode(IN2J2,OUTPUT);

  pinMode(IN1J3,OUTPUT);
  pinMode(IN2J3,OUTPUT);
}

void loop() {
  //move_joint(J2, 90, IN1J2, IN2J2, prev_error_2, inte_2, kp2, kd2, ki2);
  //move_joint(J3, 90, IN1J3, IN2J3, prev_error_3, inte_3, kp3, kd3, ki3);
  // for(int i=50;i<=150;i++){
  //   unsigned long t = millis();
  //   while(millis()-t < 20){
  //       move_joint(J2, i, IN1J2, IN2J2, prev_error_2, inte_2, kp2, kd2, ki2);
  //   }
  // }
    

  // for(int i=150;i>=50;i--){
  //   unsigned long t = millis();
  //   while(millis()-t < 20){
  //       move_joint(J2, i, IN1J2, IN2J2, prev_error_2, inte_2, kp2, kd2, ki2);
  //   }
  // }
    
//   for(int i = 0; i < 100; i++){  
//   move_joint(J2, t2[0], IN1J2, IN2J2, prev_error_2, inte_2, kp2, kd2, ki2);
//   move_joint(J3, t3[0], IN1J3, IN2J3, prev_error_3, inte_3, kp3, kd3, ki3);
//   Serial.print(0);
//   Serial.print(",");
//   Serial.print(map(analogRead(A1), 1023, 224, 0, 180)+15);
//   Serial.print(",");
//   Serial.print(map(analogRead(A2), 1023, 0, -20, 200)-90);
//   Serial.print(",");
//   Serial.println(0);
//   delay(10);
// }

//   for( int i=1;i<100;i++){
//     move_joint(J2, t2[i], IN1J2, IN2J2, prev_error_2, inte_2, kp2, kd2, ki2);
//     delay(10);
//     move_joint(J3, t3[i], IN1J3, IN2J3, prev_error_3, inte_3, kp3, kd3, ki3);
//     Serial.print(0);
//     Serial.print(",");
//     Serial.print(map(analogRead(A1), 1023, 224, 0, 180)+15);
//     Serial.print(",");
//     Serial.print(map(analogRead(A2), 1023, 0, -20, 200)-90);
//     Serial.print(",");
//     Serial.println(0);
//   }

  Serial.print(map(analogRead(A0), 0, 1023, 230, 20));
  Serial.print(",");
  Serial.print(map(analogRead(A1), 156, 1023, 210, 30)-15);
  Serial.print(",");
  Serial.print(map(analogRead(A2), 0, 1023, 110, -120));
  Serial.print(",");
  Serial.println(0);
 

}

