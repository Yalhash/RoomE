// Haris Javid  2022 
int PWM_RIGHT=150, PWM_LEFT =120;  // change these varbiels to adjust room-e drift

const byte numChars = 32;
char receivedChars[numChars];
char tempChars2[numChars]; 
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data

int state =0;
int forward_or_backward = 0;
int left_or_right=0;
int emergency_stop = 0;
float distance_travel= 0.3798318779;
float angle= 90.986;
int Pi_confimation=0;

int Arduino_done=0;
int p=0;


boolean locksend= false;
boolean newData = false;   //lock to pervent writing and reading from serail at the same time

////////////////////////////////////

#define BIN1 5
#define BIN2 4
#define PWMB 6

#define AIN1 11
#define AIN2 10
#define PWMA 9

float Encoder_holes = 20, velocity=0.5;

int L_encoder_counts=0, R_encoder_counts=0,R_encoder_total_counts=0, L_encoder_total_counts=0, average_total_encoder_counts=0;
int L_encoder_total_counts_temp=0, R_encoder_total_counts_temp=0;
unsigned long  L_time_taken=0, L_pevtime=0, R_time_taken=0, R_pevtime=0;
float L_velocity=0,  R_velocity=0, L_wheel_rev=0, distance_travel_to_interrupts=0, angle_to_interrupts=0;

void setup() {
  Serial.begin(9600);
  pinMode(PWMB,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
 attachInterrupt(digitalPinToInterrupt(2), R_Encoder, RISING);
 attachInterrupt(digitalPinToInterrupt(3), L_Encoder, RISING);
 // Serial.println("Enter data in this style <state, direction, left_or_right, emergency_stop, distance, angle, pi_confirmation> ");  
}


void loop() {  
 
  recvWithStartEndMarkers();
 
    if (newData == true) {
        locksend=true;
        strcpy(tempChars, receivedChars); // this temporary copy is necessary to protect the original data
        strcpy(tempChars2, receivedChars); // this temporary copy is necessary to protect the original data
                                        //   because strtok() used in parseData() replaces the commas with \0           
        confirm_data_recivied();       
        
        newData = false;        
        angle_to_interrupts = angle *(Encoder_holes/90); 
        distance_travel_to_interrupts = (distance_travel/0.3798318779) * Encoder_holes; 
    }

    switch(state){                                     // we have 3 modes turn, forward/backward and standby. 
      case 1:
        turn_direction(angle_to_interrupts,left_or_right);
        break;
      
      case 2:
         Drive_stright(distance_travel_to_interrupts,forward_or_backward);
         break;
      
      default:
      digitalWrite(BIN2,HIGH);  // default stand by mode
      digitalWrite(BIN1,LOW);
      digitalWrite(AIN2,HIGH);
      digitalWrite(AIN1,LOW);
      analogWrite(PWMA,0);
      analogWrite(PWMB,0); 

      if(Arduino_done==1 && L_encoder_total_counts_temp==L_encoder_total_counts  &&  R_encoder_total_counts_temp==R_encoder_total_counts){
        Serial.print("<");
        Serial.print(String(L_encoder_total_counts));  
        Serial.print(",");
        Serial.print(String(R_encoder_total_counts));    
        Serial.println(">");
        Arduino_done=0;
      }
      L_encoder_total_counts_temp=L_encoder_total_counts; // check if room-e stopped moving 
      R_encoder_total_counts_temp=R_encoder_total_counts;
      delay(100);
      break;
   }
}


/////////////////////////////      Enocder code      //////////////////////////////////////////////
void L_Encoder(){  
    L_encoder_counts++;
    L_encoder_total_counts++;
  
    if(L_encoder_counts>=20){      
        L_time_taken = millis()-L_pevtime;
        L_velocity= (1000/(L_time_taken))*0.379831118;      // calculating velocty m/s  = r*2*π*(RPS) = 0.060452*2*pi*RPS
        L_pevtime = millis();
        L_wheel_rev+=1;
        L_encoder_counts =0;
    }
}

void R_Encoder(){
  
    R_encoder_counts++;
    R_encoder_total_counts++;   // total encoder counts to make sure we count all the interupts.
  
    if(R_encoder_counts>=20){
        R_time_taken = millis()-R_pevtime;
        R_velocity= (1000/(R_time_taken))*0.379831118;      // calculating velocty m/s  = r*2*π*(RPS) = 0.060452*2*pi*RPS
        R_pevtime = millis();
        R_encoder_counts =0; 
    }
 
}

/////////////////////////// turn direction ///////////////////////////////////

void turn_direction(int angle_to_interrupts,int Left_or_right){
   if(Left_or_right==0){       // left turn
        digitalWrite(BIN2,HIGH);
        digitalWrite(BIN1,LOW);
        digitalWrite(AIN1,HIGH);
        digitalWrite(AIN2,LOW);
        analogWrite(PWMA,0);
        analogWrite(PWMB,210);
        if(R_encoder_total_counts>= angle_to_interrupts or  emergency_stop == 1){     // will run untill required number of angle is completed  
        state=0;
        Arduino_done=1;
      } // check for next serial_cominication from pi.}
   }
   if(Left_or_right==1){      // right turn
        digitalWrite(BIN2,HIGH);
        digitalWrite(BIN1,LOW);
        digitalWrite(AIN1,HIGH);
        digitalWrite(AIN2,LOW);
        analogWrite(PWMA,160);
        analogWrite(PWMB,0);
        if(L_encoder_total_counts>= angle_to_interrupts or  emergency_stop == 1){     // will run untill required number of angle is completed  
            state=0;
            Arduino_done=1;
            // letting pi know we are done
        } // check for next serial_cominication from pi.
    }
}
     


     
////////////////////////////  Drive_stright   ////////////////////////

void Drive_stright(float distance_travel_to_interrupts,int forward_or_backward){
   
    if(forward_or_backward==0){       // drive forward
        digitalWrite(BIN2,HIGH);
        digitalWrite(BIN1,LOW);
        digitalWrite(AIN1,HIGH);
        digitalWrite(AIN2,LOW);
    }
     
    if(forward_or_backward==1){   // drive backward
        digitalWrite(BIN2,LOW);
        digitalWrite(BIN1,HIGH);
        digitalWrite(AIN1,LOW);
        digitalWrite(AIN2,HIGH);
   }
     
   average_total_encoder_counts= (L_encoder_total_counts+ R_encoder_total_counts)/2;
   analogWrite(PWMA,PWM_LEFT);
   analogWrite(PWMB,PWM_RIGHT);
   if (average_total_encoder_counts>=distance_travel_to_interrupts or  emergency_stop == 1){
      state=0;
      Arduino_done=1;
      // let pi know we are done
   }
}
    

////////////////////////////// maintain velcoity ///////////////////////////

void  maintain_velcoity(float velocity){
  if(L_velocity<velocity){
    PWM_LEFT+=5;
  }
    
  if(L_velocity>velocity){
      PWM_LEFT-=5;
  }  
    
  if(R_velocity<velocity){
    PWM_LEFT+=5;
  } 
    
  if(R_velocity>velocity){
     PWM_LEFT-=5;
  } 
}




///////////////


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first variable
    state = atoi(strtokIndx);
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    forward_or_backward = atoi(strtokIndx);     // convert this part to an integer
    
    strtokIndx = strtok(NULL, ",");
    left_or_right = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    emergency_stop = atoi(strtokIndx);     // convert this part to an integer
    
    strtokIndx = strtok(NULL, ",");
    distance_travel = atof(strtokIndx);     // convert this part to a float
    
    strtokIndx = strtok(NULL, ",");
    angle = atof(strtokIndx);     // convert this part to a float
}

//============


void confirm_data_recivied(){
        
    // sending data back to pi 
    p++;
    // check if pi confirmed that the data looks correct
        
    char * strtokIndx1; // this is used by strtok() as an index
    strtokIndx1 = strtok(tempChars2,",");      // get the first variable
    for(int j=0; j<6;j++){
        strtokIndx1 = strtok(NULL, ",");
        Pi_confimation = atoi(strtokIndx1);
    }         
    if(Pi_confimation == 1){ //  if data is correct we update our drivetrain values 
       locksend=false;
       parseData();
       L_encoder_total_counts=0;
       R_encoder_total_counts=0; 
       p=0;
    }
}

// 
void showParsedData() {
    Serial.print("<");
    Serial.print(String(state));
    Serial.print(","); 
    Serial.print(String(forward_or_backward)); 
    Serial.print(","); 
    Serial.print(String(left_or_right)); 
    Serial.print(","); 
    Serial.print(String(emergency_stop));  
    Serial.print(",");
    Serial.print(String(distance_travel));
    Serial.print(","); 
    Serial.print(String(angle));  
    Serial.print(","); 
    Serial.print(String(Pi_confimation));  
    Serial.println(">");
}
