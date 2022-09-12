// 470ohm equipe violette,1000 ohm equipe jaune, 47000ohm personne ne pas basculler

#include <Servo.h>

//include ros library
#include <ros.h>

//include String messages
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

//initialisation de la node
ros::NodeHandle cf;

Servo servoBascule;
Servo servoPalpeur;
int ledR = 2;
int ledJ = 3;
int ledV = 4;

String msg;

int palpeur_pos = 140;
int palpeur_home = 30;

int bascule_pos = 80;
int bascule_home = 150;

int valCapteur;

//initialisation du message
std_msgs::String str_msg;

//fonction de recuperation des données
void readSerialPort(const std_msgs::String& instructions) { 
  msg=instructions.data;
}

// creating des subscribers et publishers
ros::Subscriber<std_msgs::String> sub("/inst_fouille", &readSerialPort);
ros::Publisher pub("/etat_fouille", &str_msg);

void setup() {
  cf.initNode();
  cf.subscribe(sub);
  cf.advertise(pub);
  pinMode(ledR, OUTPUT);
  pinMode(ledV, OUTPUT);
  pinMode(ledJ, OUTPUT);
  digitalWrite(ledR, LOW);
  digitalWrite(ledV, LOW);
  digitalWrite(ledJ, LOW);
  servoPalpeur.attach(5);
  servoBascule.attach(6);
  servoPalpeur.write(palpeur_home);    
  servoBascule.write(bascule_home);
}

void loop() {
  msg = msg[0];
    
  int valPalpeur;
      
  if(msg=="V"||msg=="J"){
    valPalpeur==palper();
    valPalpeur=1;
    if(msg == "V" && valPalpeur == 1){
      faireBasculer();     
    }
    else if(msg == "J" && valPalpeur == 2){
      faireBasculer();
    }
    else{
      str_msg.data="R";
      pub.publish(&str_msg); 
    }
  }
  msg = "";
  afficherLed();
  cf.spinOnce();
  delay(100);
}

int palper(){
  int etat = 0;
  servoPalpeur.write(palpeur_pos);
  delay(2000);
  valCapteur = analogRead(A0);
  if(valCapteur < 100){
      etat = 1;
    }
    else if(valCapteur > 100 && valCapteur < 700){
      etat = 2;
    }
  servoPalpeur.write(palpeur_home);
  delay(1000);
  return(etat);
}

void faireBasculer(){
  str_msg.data="B";
  pub.publish(&str_msg); //on demander à la rasp de reculer pour faire basculer
  delay(3000);
  servoBascule.write(bascule_pos);
  delay(6000);
  servoBascule.write(bascule_home);
}

void afficherLed(){
 
  valCapteur = analogRead(A0);
  if(valCapteur < 100){
      
      digitalWrite(ledR, HIGH);
      digitalWrite(ledV, LOW);
      digitalWrite(ledJ, LOW);
    }
    else if(valCapteur > 100 && valCapteur < 700){
      
      digitalWrite(ledR, LOW);
      digitalWrite(ledV, LOW);
      digitalWrite(ledJ, HIGH);
    }
    else if(valCapteur > 700){
      
      digitalWrite(ledR, LOW);
      digitalWrite(ledV, HIGH);
      digitalWrite(ledJ, LOW);
    }
    else{
      digitalWrite(ledR, LOW);
      digitalWrite(ledV, LOW);
      digitalWrite(ledJ, LOW);
    }
    
  
}
