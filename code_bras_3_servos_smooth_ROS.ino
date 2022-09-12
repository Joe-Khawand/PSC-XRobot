//include ros library
#include <ros.h>
#include <std_msgs/UInt16.h>

#include <Arduino.h>

#define PROVIDE_ONLY_LINEAR_MOVEMENT 
#include "ServoEasing.hpp"

#include <math.h>
#include <Servo.h>

//initialiation de la node
ros::NodeHandle bra;

// On définit nos 3 servos qui vont piloter le bras articulé

ServoEasing based;
ServoEasing couded;
ServoEasing poignetd;



// On pose la pompe
const int pumpin = 12;

int pos_bras_0 [3] = {180, 180, 180};
int pos_test [3] = {180, 110, 100};

int pos_base = 0;
int pos_coude = 0;
int pos_poign = 0;


//fonction pour piloter les 3 servos en même temps à partir d'une donnée de 3 angles
void set(int pos_bras [3]){

    setSpeedForAllServos(40);
    based.setEaseTo(pos_bras[0]);    // This servo uses effectively 10 degrees per second, since it is synchronized to Servo3
    couded.setEaseTo(pos_bras[1]);   // "ServoEasing::ServoEasingArray[1]->" can be used instead of "Servo2."
    poignetd.setEaseTo(pos_bras[2]);                              // This servo has the longest distance -> it uses 20 degrees per second
    setEaseToForAllServosSynchronizeAndStartInterrupt(40);
    //synchronizeAllServosAndStartInterrupt(false); 
    while (ServoEasing::areInterruptsActive()) {
        delay(20);
    }
}



int pos_bas_hor [3] = {160, 140, 50};
int pos_haut_hor [3] = {180, 90, 30};
int pos_bas_ver [3] = {160, 140, 140};
int pos_haut_ver [3] = {180, 90, 100};
int pos_haut_retour [3] = {180, 140, 120};
int pos_bas_retour [3] = {160, 140, 120};



void prendre_hor(){
  //cette fonction aura pour but d'attraper un échantillon à l'horizontal
  set (pos_bas_hor); //position sur l'échantillon
  delay (1000);
  digitalWrite (pumpin, HIGH);
  set (pos_haut_hor); // position surélevée pour le bouger
  delay (1000);
}

void depo_hor(){
  //cette fonction aura pour but de déposer un échantillon à l'horizontal
  set (pos_bas_hor); //position basse
  delay (1000);
  digitalWrite (pumpin, LOW);
  delay(1000);
  set (pos_haut_hor); // position surélevée
  delay (1000);
}

void prendre_vert(){
  //cette fonction aura pour but d'attraper un échantillon à la verticale
  set (pos_bas_ver);
  delay (1000);
  digitalWrite (pumpin, HIGH);
  delay(100);
  set (pos_haut_ver);
  delay (1000);
}

void depo_vert(){
  //cette fonction aura pour but de déposer un échantillon à la verticale
  set (pos_bas_ver);
  delay (1000);
  digitalWrite (pumpin, LOW);
  delay(100);
  set (pos_haut_ver);
  delay (1000);
}


void retourner(){
  // cette fonction retourne un échantillon
  set(pos_haut_retour);
  set(pos_bas_retour);
  digitalWrite (pumpin, LOW);
  set(pos_haut_hor);
  
  
}
//creatiion du subricber
ros::Subscriber<std_msgs::UInt16>sub("/bras",&reception);


void reception(const std_msgs::UInt16& instructions){
  int n;
  n=instructions.data;
  if(n==0){
    depo_hor();
  }
  else if(n==1){
    prendre_hor();
  }
  else if(n==2){
    depo_vert();
  }
  else if(n==3){
    prendre_vert();
  }
  else if(n==4){
    retourner();
  }
}

void setup() {
  bra.initNode();
  bra.subscribe(sub);
  
// On connecte ces 3 servos aux pins de l'Arduino
  based.attach(10,90);
  couded.attach(11,180);
  poignetd.attach(9,90);
 
  pinMode(pumpin, OUTPUT);
  set (pos_bras_0);
}

void loop() {
  bra.spinOnce();
  delay(100);
}
