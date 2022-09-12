
//avec ecran pas assez de memoire instable!!!
// Include the AccelStepper library:
#include <AccelStepper.h>
#include <math.h>

//include ros library
#include <ros.h>

//include String messages
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

//initialisation de la node
ros::NodeHandle nh;

//initialisation du message
std_msgs::String str_msg;
geometry_msgs::Point pt_msg;

#define moteurDdirPin 5
#define moteurDstepPin 2   //axe x
#define moteurGdirPin 6    //axe z
#define moteurGstepPin 3
#define motorInterfaceType 1

#define X0  0;    //position de depart (en metre et radian)
#define Y0  0;
#define A0  0;

AccelStepper moteurG = AccelStepper(motorInterfaceType, moteurGstepPin, moteurGdirPin);
AccelStepper moteurD = AccelStepper(motorInterfaceType, moteurDstepPin, moteurDdirPin);

 //position du robot, x, y en metre et angle rad
double X = X0;
double Y = Y0;
double A = A0;


String etat;
double x,y,a;    //commande de position par le rasp;   etat: S: stop, D: deplacement

double teta, d; //variable pour le calcule de position


double facteurMoteurG = 2*0.00008996*0.999;  // facteur metre/step
double facteurMoteurD = 2*0.00008970*0.9975;  // si deplacement de 9cm au lieu de 10cm => facteur * 0.9
double facteurRotation = 0.1358*1.03588;      // si 90 deg au lieu de 180 => facteur *2

String msg;

void readSerialPort(const std_msgs::String& instructions) {    
  msg = instructions.data;
    etat = msg.charAt(0);
        if(etat == "D" || etat == "R"|| etat == "N"){
      int xstop =0;
      int ystop =0;
      int astop =0;
      int maxIndex = msg.length()-1;
      for(int i=1; i<=maxIndex ; i++){
        if(msg.charAt(i)==';'){
          if(xstop == 0){
            xstop = i;
          }
          else if(ystop == 0){
             ystop = i;
          }
          else{
            astop = i;
          }
        }
      }
      x =  msg.substring(1,xstop).toDouble()/1000000;
      y = msg.substring(xstop+1,ystop).toDouble()/1000000; 
      int arecu =  msg.substring(ystop+1,astop).toInt() ;
      a = ((double) arecu)/10000;
      if(a < -PI){    //angle entre +-PI
        while(a < -PI){a+= 2*PI;}
      }
      else if(a > PI){
        while(a > PI){a-= 2*PI;}
      }
    }
  }

// creating des subscribers et publishers
ros::Subscriber<std_msgs::String> sub("/instructions", &readSerialPort);
ros::Publisher pos("/pos", &pt_msg);
//ros::Subscriber<std_msgs::Point> up("/pos", &updatepos);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pos);
  
  moteurG.setMaxSpeed(1400/(facteurMoteurD/facteurMoteurG));
  moteurG.setAcceleration(2000/(facteurMoteurD/facteurMoteurG));
  moteurD.setMaxSpeed(1400*1.2);
  moteurD.setAcceleration(2000*1.2);  
}

void loop() {
   
  if(etat == "D"){   //recoit DX;Y;A;
   
    d = sqrt( square((x-X)) + square((y-Y))  );
    if(d == 0){   //cas simple rotation
      rotation(a - A);
    }
    else{
      teta =  atan2 ( y-Y,  x-X) ;  // radian  de -pi a +pi
    
      rotation(teta - A);
      boolean translationComplete = translation(d);   //true si pas de message dans le serialPort, false si l'arduino ressoit un message
      if(translationComplete == true){rotation(a - teta);}
    }

    envoyerPosition();
  }

  else if(etat == "S"){stopMoteurs();}

  else if(etat == "R"){   //recoit RX;Y;A;
    
    d = sqrt( square((x-X)) + square((y-Y))  );
    if(d == 0){   //cas simple rotation
      rotation(a - A);
    }
    else{
    teta =  atan2 ( y-Y,  x-X) ;  // radian  de -pi a +pi
    
    rotation(PI+teta - A);
    
    boolean translationComplete = translation(-d);   //true si pas de message dans le serialPort, false si l'arduino ressoit un message
    
    if(translationComplete == true ){rotation(a - teta-PI);}
    }

    envoyerPosition();
  }

  else if(etat == "N"){   //UPDATE DE LA POSITION
    
    X = x;
    Y= y;
    A = a;
    envoyerPosition();
    delay(50);
  }
  //faire tourner ros
  etat="";
  nh.spinOnce();
  delay(100);

}

boolean translation(double distance){    // stoppe s recoit une autre commande
  //Serial.print("translation  ");
    //Serial.println(distance);
  boolean b = true;
  moteurG.move(long(distance/facteurMoteurG)); 
  moteurD.move(long(distance/facteurMoteurD));
  while(moteurG.isRunning()|moteurD.isRunning()) {             // doit quitter le while et le if !!!
    if(Serial.available()){
      stopMoteurs();
       X += distance * cos(A);   // en radian
       Y += distance * sin(A);
      return false;
    }
    moteurD.run();
    moteurG.run();
    
  }
  X += distance * cos(A);   // en radian
  Y += distance * sin(A);
  return(true);
}

void rotation ( double angle) {            //rotation attention pas de stoppe pendant la rotation
  if(angle < -PI){    //angle entre +-PI
    while(angle < -PI){angle+= 2*PI;}
  }
  else if(angle > PI){
    while(angle > PI){angle-= 2*PI;}
  }
  moteurG.move(long(-angle*facteurRotation/facteurMoteurG));    //ajoute nouvelle objectif
  moteurD.move(long(angle*facteurRotation/facteurMoteurD));   //tourne dans le sens inverse
  while(moteurG.isRunning() == true | moteurD.isRunning() == true){
    moteurG.run();
    moteurD.run();
    
  }
  A += angle;
  
}


void stopMoteurs() {
  long stepmoteurg = moteurG.currentPosition();
  double distanceObjectif = moteurG.distanceToGo()*facteurMoteurG;   //distance qu'il reste à parcourir
  moteurG.stop();   //nouvel objectif pour s'arreter le plus rapidement possible
  moteurD.stop();
  while(moteurG.isRunning() == true | moteurD.isRunning() == true){  //va à l'objectif
    moteurG.run();
    moteurD.run();
  }

  long newstepmoteurg =moteurG.currentPosition();

  d = (newstepmoteurg - stepmoteurg)*facteurMoteurG;   //calcule la distance parcourue pendant l'arret
  
  X += (d-distanceObjectif)*cos(A);
  Y += (d-distanceObjectif)*sin(A);
  envoyerPosition();
}

void envoyerPosition(){
  pt_msg.x=X*1000000;
  pt_msg.y=Y*1000000;
  pt_msg.z=A*1000000;
  pos.publish(&pt_msg);
} 
