#include <VarSpeedServo.h>

VarSpeedServo serr;
VarSpeedServo serl;
VarSpeedServo serm;

//---------------------------------------------------------------------------------//
//----------------------------Tuner Variables--------------------------------------//
//---------------------------------------------------------------------------------//
        #define pickup_distance 5
        
        //----------------------------Speeds-----------------------//
        int mspeed_high=150; //Motor Speed High
        int mspeed_low=140; //Motor Speed Low
        int slow_speed = 10;
        #define reverse_speed 50
        //--------------------------------------------------------//
        
        //----------------------Turns-----------------------------//
        #define delayturn 350
        #define delayforward 150
        #define reverse_delay 580
        
        #define delayturn_fixleft 400
        #define delayforward_fixleft 0
        //--------------------------------------------------------//

//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//


//---------------------------------------------------------------------------------//
//----------------------------Do not touch this------------------------------------//
//---------------------------------------------------------------------------------//
        #define redPin 34
        #define greenPin 36
        #define bluePin 38

        #define S0 10
        #define S1 9
        #define S2 12
        #define S3 11
        #define sensorOut 13
        
        int LeftMotorEnable=7;
        int RightMotorEnable=8;
        int LeftMotorBackward=29;
        int LeftMotorForward=27;
        int RightMotorForward=23;
        int RightMotorBackward=25;
        
        const int trigPin = 3;
        const int echoPin = 2;
        
        
        int ser1 = 6;
        int ser2 = 5;
        int ser3 = 4;
        int sers = 75;
        
        int s1,s2,s3,s4,s5,s6;
        bool gotbox = false;
        bool passed_junction = false;
        int box_color;
//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//




void setup() {
  //----------------------------Color Sensor--------------------------------//
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(sensorOut, INPUT);
    digitalWrite(S0,HIGH);
    digitalWrite(S1,LOW);
  //------------------------------------------------------------------------//
  //-----------------------------LDR---------------------------------------//
    pinMode(22,INPUT);
    pinMode(24,INPUT);
    pinMode(26,INPUT);
    pinMode(28,INPUT);
    pinMode(30,INPUT);
    pinMode(32,INPUT);
  //------------------------------------------------------------------------//

  //-----------------------------Motors-------------------------------------//
    pinMode(LeftMotorBackward,OUTPUT);
    pinMode(LeftMotorForward,OUTPUT);
    pinMode(RightMotorForward,OUTPUT);
    pinMode(RightMotorBackward,OUTPUT);
  //------------------------------------------------------------------------//
  
  //--------------------------------Arm-------------------------------------//
    serl.attach(ser1);
    serr.attach(ser2);
    serm.attach(ser3);                     
    resetArm();
  //------------------------------------------------------------------------//

  //----------------------------RGB LED-------------------------------------//
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
  //------------------------------------------------------------------------//
  
  //------------------------------Sonar-------------------------------------//
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  //------------------------------------------------------------------------//
  Serial.begin(9600); 

  moveForward(100);
  delay(200);
}

void loop() {
    //Check if Box is loaded
    if(!gotbox && !passed_junction) {
        setColor(0,0,0);
        // If predefined distance to the box is lower than Sonar Reading make sure Sonar reading is right again by updating again
        // If it's right pickup the box
        if(distance()<=pickup_distance){
            //moveForward(50);
            setColor(255,0,255);
            stopMotors();
            delay(50);
            moveForward(mspeed_low);
            delay(20);

            if(distance()<=pickup_distance){
                setColor(255,255,255);
                loadbox();
            }
            else{
                solve_maze();
            }
        }
        // Else just try to get to the loading area by solving the maze
        else{
            solve_maze();
        }
    }
    else if (gotbox && passed_junction){
        line_follow_2_unloading();
    }
    // if box is loaded do a basic line following till the 4 way junction
    else{
        basic_line_follow();
        // if robot get to the 4 way junction turn to the pre defined direction with relevant to the color of the boo 
        if(s1==HIGH && s2==HIGH && s3==HIGH && s4==HIGH && s5==HIGH && s6==HIGH){
            stopMotors();
            passed_junction = true;
            if(box_color == 1){
                moveRightl_with_fwd_slow();
                updateSensors();
                stopMotors();
                if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
                    while (s1==HIGH || s2==HIGH || s3==HIGH || s4==HIGH || s5==HIGH || s6==HIGH){
                        updateSensors();
                        moveRightl_nofwd_slow();
                        stopMotors();
                    }
                } 
            }
            else if(box_color == 2){
                moveLeftl_fixleft_slow();
                updateSensors();
                stopMotors();
                if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
                    while (s6==HIGH || s5==HIGH || s4==HIGH || s3==HIGH || s2==HIGH || s1==HIGH){
                        updateSensors();
                        moveLeftl_nofwd_slow();
                        stopMotors();
                    }
                }
            }
            else{
                moveForward(mspeed_low);
                delay(100);
            }
        }


}
}


//---------------------------------------------------------------------------------//
//------------------------Section 1 - Maze Soloving--------------------------------//
//---------------------------------------------------------------------------------//
void solve_maze(){
    updateSensors();
    //-----------------------------------------------------------------------------//
    //One Sensor
    
    //100000
    if(s1==HIGH && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //010000
    else if(s1==LOW && s2==HIGH && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //001000
    else if(s1==LOW && s2==LOW && s3==HIGH && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //000100
    else if(s1==LOW && s2==LOW && s3==LOW && s4==HIGH && s5==LOW && s6==LOW){
      moveRight();
    }
    //000010
    else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==HIGH && s6==LOW){
      moveRight();
    }
    //000001
    else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==HIGH){
      moveRight();
    }

    //-----------------------------------------------------------------------------//
    //Two Sensors
    
    //110000
    else if(s1==HIGH && s2==HIGH && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //011000
    else if(s1==LOW && s2==HIGH && s3==HIGH && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //001100
    else if(s1==LOW && s2==LOW && s3==HIGH && s4==HIGH && s5==LOW && s6==LOW){
      moveForward(mspeed_high);
    }
    //000110
    else if(s1==LOW && s2==LOW && s3==LOW && s4==HIGH && s5==HIGH && s6==LOW){
      moveRight();
    }
    //000011
    else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==HIGH && s6==HIGH){
      moveRight();
    }

    //-----------------------------------------------------------------------------//
    //Three Sensors
    
    //111000
    else if(s1==HIGH && s2==HIGH && s3==HIGH && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //011100
    else if(s1==LOW && s2==HIGH && s3==HIGH && s4==HIGH && s5==LOW && s6==LOW){
      moveLeft();
    }
    //001110
    else if(s1==LOW && s2==LOW && s3==HIGH && s4==HIGH && s5==HIGH && s6==LOW){
      moveRight();
    }
    //000111
    else if(s1==LOW && s2==LOW && s3==LOW && s4==HIGH && s5==HIGH && s6==HIGH){
      moveRight();
    }
    
    //-----------------------------------------------------------------------------//
    //Four Sensors
    
    //111100 or 111110 or 111000
    else if((s1==HIGH && s2==HIGH && s3==HIGH && s4==HIGH && s5==LOW && s6==LOW) || 
            (s1==HIGH && s2==HIGH && s3==HIGH && s4==HIGH && s5==HIGH && s6==LOW)) {
            //(s1==HIGH && s2==HIGH && s3==HIGH && s4==LOW && s5==LOW && s6==LOW)){
      //delay(2000);
      stopMotors();
      moveRightl_nofwd();
      delay(20);
      stopMotors();
      //delay(2000);
      updateSensors();
      if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
          while (s1==HIGH || s2==HIGH || s3==HIGH || s4==HIGH || s5==HIGH || s6==HIGH){
              updateSensors();
              allwhite();
          }
      }
      else{
          moveRight();
      }
    }
    //011110
    else if(s1==LOW && s2==HIGH && s3==HIGH && s4==HIGH && s5==HIGH && s6==LOW){
        moveRightl_nofwd();
    }
    
    //-----------------------------------------------------------------------------//
    //Else Sensors
    //001111
    else if(s1==LOW && s2==LOW && s3==HIGH && s4==HIGH && s5==HIGH && s6==HIGH){
      moveRightl_with_fwd();
    }
    //000000
    else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
      allwhite();
    }
    //111111
    else if(s1==HIGH && s2==HIGH && s3==HIGH && s4==HIGH && s5==HIGH && s6==HIGH){
      moveRightl_with_fwd();
    }
    //101100
    else if(s1==HIGH && s2==LOW && s3==HIGH && s4==HIGH && s5==LOW && s6==LOW){
      moveForward(mspeed_low);
    }
    //100100
    else if(s1==HIGH && s2==LOW && s3==LOW && s4==HIGH && s5==LOW && s6==LOW){
      moveForward(mspeed_low);
    }
    //001101
    else if(s1==LOW && s2==LOW && s3==HIGH && s4==HIGH && s5==LOW && s6==HIGH){
      moveForward(mspeed_low);
    }
    //001001
    else if(s1==LOW && s2==LOW && s3==HIGH && s4==LOW && s5==LOW && s6==HIGH){
      moveForward(mspeed_low);
    }
}

//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//













//---------------------------------------------------------------------------------//
//----------Do basic line following (No 90 degree or L turns) Section 2------------//
//---------------------------------------------------------------------------------//
void basic_line_follow(){
    updateSensors();
    //One Sensor
    
    //100000
    if(s1==HIGH && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //010000
    else if(s1==LOW && s2==HIGH && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //001000
    else if(s1==LOW && s2==LOW && s3==HIGH && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //000100
    else if(s1==LOW && s2==LOW && s3==LOW && s4==HIGH && s5==LOW && s6==LOW){
      moveRight();
    }
    //000010
    else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==HIGH && s6==LOW){
      moveRight();
    }
    //000001
    else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==HIGH){
      moveRight();
    }
    
    
    //-----------------------------------------------------------------------------------//
    //Two Sensors
    
    //110000
    else if(s1==HIGH && s2==HIGH && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //011000
    else if(s1==LOW && s2==HIGH && s3==HIGH && s4==LOW && s5==LOW && s6==LOW){
      moveLeft();
    }
    //001100
    else if(s1==LOW && s2==LOW && s3==HIGH && s4==HIGH && s5==LOW && s6==LOW){
      moveForward(mspeed_high);
    }
    //000110
    else if(s1==LOW && s2==LOW && s3==LOW && s4==HIGH && s5==HIGH && s6==LOW){
      moveRight();
    }
    //000011
    else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==HIGH && s6==HIGH){
      moveRight();
    }
}
//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//








// This function is called when robot passed 4 way junction and turn to the right direction
void line_follow_2_unloading(){
updateSensors();
  //One Sensor
  
  //100000
  if(s1==HIGH && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
    moveLeft();
  }
  //010000
  else if(s1==LOW && s2==HIGH && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
    moveLeft();
  }
  //001000
  else if(s1==LOW && s2==LOW && s3==HIGH && s4==LOW && s5==LOW && s6==LOW){
    moveLeft();
  }
  //000100
  else if(s1==LOW && s2==LOW && s3==LOW && s4==HIGH && s5==LOW && s6==LOW){
    moveRight();
  }
  //000010
  else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==HIGH && s6==LOW){
    moveRight();
  }
  //000001
  else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==HIGH){
    moveRight();
  }
  
  
  //-----------------------------------------------------------------------------------//
  //Two Sensors
  
  //110000
  else if(s1==HIGH && s2==HIGH && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
    moveLeft();
  }
  //011000
  else if(s1==LOW && s2==HIGH && s3==HIGH && s4==LOW && s5==LOW && s6==LOW){
    moveLeft();
  }
  //001100
  else if(s1==LOW && s2==LOW && s3==HIGH && s4==HIGH && s5==LOW && s6==LOW){
    moveForward(mspeed_low);
  }
  //000110
  else if(s1==LOW && s2==LOW && s3==LOW && s4==HIGH && s5==HIGH && s6==LOW){
    moveRight();
  }
  //000011
  else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==HIGH && s6==HIGH){
    moveRight();
  }

  //-----------------------------------------------------------------------------------//
  //Unloading Sensors
/*
  //011100
  else if(s1==LOW && s2==HIGH && s3==HIGH && s4==HIGH && s5==LOW && s6==LOW){
    unload();
  }
  
  //001110
  else if(s1==LOW && s2==LOW && s3==HIGH && s4==HIGH && s5==HIGH && s6==LOW){
    unload();
  }
  */
  //011110
  else if(s1==LOW && s2==HIGH && s3==HIGH && s4==HIGH && s5==HIGH && s6==LOW){
    unload();
  }
  
  //111100
  else if(s1==HIGH && s2==HIGH && s3==HIGH && s4==HIGH && s5==LOW && s6==LOW){
    unload();
  }
  //001111
  else if(s1==LOW && s2==LOW && s3==HIGH && s4==HIGH && s5==HIGH && s6==HIGH){
    unload();
  }
 /* //11111
  else if(s1==HIGH && s2==HIGH && s3==HIGH && s4==HIGH && s5==HIGH && s6==HIGH){
    unload();
  }
  */
  else if(s1==LOW && s2==LOW && s3==LOW && s4==LOW && s5==LOW && s6==LOW){
    if(box_color == 1)
      moveRightl_nofwd();
    else if (box_color == 2)
      moveLeftl_nofwd();
  }
}










//---------------------------------------------------------------------------------//
//------------------------------Motor Functions------------------------------------//
//---------------------------------------------------------------------------------//

void moveForward(int mspeed){
    analogWrite(LeftMotorEnable,mspeed);
    analogWrite(RightMotorEnable,mspeed);
    
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
}

void moveRight(){
    analogWrite(LeftMotorEnable,mspeed_low);
    analogWrite(RightMotorEnable,0);
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,LOW);
    digitalWrite(RightMotorBackward,LOW);
}
void moveLeft(){
    analogWrite(LeftMotorEnable,0);
    analogWrite(RightMotorEnable,mspeed_low);
    digitalWrite(LeftMotorForward,LOW);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
}

void moveRightl_with_fwd(){
    analogWrite(LeftMotorEnable,mspeed_high);
    analogWrite(RightMotorEnable,mspeed_high);       
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
    delay(delayforward);   
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,LOW);
    digitalWrite(RightMotorBackward,HIGH);
    delay(delayturn);

}

void moveLeftl_with_fwd(){
    analogWrite(LeftMotorEnable,mspeed_high);
    analogWrite(RightMotorEnable,mspeed_high);       
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
    delay(delayforward);
    
    digitalWrite(LeftMotorForward,LOW);
    digitalWrite(LeftMotorBackward,HIGH);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
    delay(delayturn);
}



void stopMotors(){
    analogWrite(LeftMotorEnable,0);
    analogWrite(RightMotorEnable,0); 
    digitalWrite(LeftMotorForward,LOW);
    digitalWrite(RightMotorForward,LOW);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorBackward,LOW);
}

void moveBack(int mspeed){
    analogWrite(LeftMotorEnable,mspeed);
    analogWrite(RightMotorEnable,mspeed);
    digitalWrite(LeftMotorForward,LOW);
    digitalWrite(RightMotorForward,LOW);
    digitalWrite(LeftMotorBackward,HIGH);
    digitalWrite(RightMotorBackward,HIGH);
}
void allwhite(){
    analogWrite(LeftMotorEnable,100);
    analogWrite(RightMotorEnable,60);
    digitalWrite(LeftMotorForward,LOW);
    digitalWrite(LeftMotorBackward,HIGH);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
}

//--------------------------Hacked Motor Functions---------------------------------//
void moveRightl_nofwd(){ 
    analogWrite(LeftMotorEnable,mspeed_low);
    analogWrite(RightMotorEnable,mspeed_low);
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,LOW);
    digitalWrite(RightMotorBackward,HIGH);

}

void moveLeftl_nofwd(){  
    analogWrite(LeftMotorEnable,mspeed_low);
    analogWrite(RightMotorEnable,mspeed_low);
    digitalWrite(LeftMotorForward,LOW);
    digitalWrite(LeftMotorBackward,HIGH);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
}

void moveLeftl_fixleft(){
    analogWrite(LeftMotorEnable,mspeed_low);
    analogWrite(RightMotorEnable,mspeed_low);       
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
    delay(delayforward_fixleft);
    analogWrite(LeftMotorEnable,mspeed_low);
    analogWrite(RightMotorEnable,mspeed_low); 
    digitalWrite(LeftMotorForward,LOW);
    digitalWrite(LeftMotorBackward,HIGH);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
    delay(delayturn_fixleft);
}


void moveRightl_with_fwd_slow(){
    analogWrite(LeftMotorEnable,mspeed_high-slow_speed);
    analogWrite(RightMotorEnable,mspeed_high-slow_speed);       
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
    delay(delayforward);   
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,LOW);
    digitalWrite(RightMotorBackward,HIGH);
    delay(delayturn);

}

void moveLeftl_with_fwd_slow(){
    analogWrite(LeftMotorEnable,mspeed_high-slow_speed);
    analogWrite(RightMotorEnable,mspeed_high-slow_speed);       
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
    delay(delayforward);
    
    digitalWrite(LeftMotorForward,LOW);
    digitalWrite(LeftMotorBackward,HIGH);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
    delay(delayturn);
}

void moveRightl_nofwd_slow(){ 
    analogWrite(LeftMotorEnable,mspeed_low-slow_speed);
    analogWrite(RightMotorEnable,mspeed_low-slow_speed);
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,LOW);
    digitalWrite(RightMotorBackward,HIGH);

}

void moveLeftl_nofwd_slow(){  
    analogWrite(LeftMotorEnable,mspeed_low-slow_speed);
    analogWrite(RightMotorEnable,mspeed_low-slow_speed);
    digitalWrite(LeftMotorForward,LOW);
    digitalWrite(LeftMotorBackward,HIGH);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
}

void moveLeftl_fixleft_slow(){
    analogWrite(LeftMotorEnable,mspeed_low-slow_speed);
    analogWrite(RightMotorEnable,mspeed_low-slow_speed);       
    digitalWrite(LeftMotorForward,HIGH);
    digitalWrite(LeftMotorBackward,LOW);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
    delay(delayforward_fixleft);
    analogWrite(LeftMotorEnable,mspeed_low-slow_speed);
    analogWrite(RightMotorEnable,mspeed_low-slow_speed); 
    digitalWrite(LeftMotorForward,LOW);
    digitalWrite(LeftMotorBackward,HIGH);
    digitalWrite(RightMotorForward,HIGH);
    digitalWrite(RightMotorBackward,LOW);
    delay(delayturn_fixleft);
}
//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//




//---------------------------------------------------------------------------------//
//-----------------------------Helper Functions------------------------------------//
//---------------------------------------------------------------------------------//
void updateSensors(){
    s1=digitalRead(22);
    s2=digitalRead(24);
    s3=digitalRead(26);
    s4=digitalRead(28);
    s5=digitalRead(30);
    s6=digitalRead(32);
}
// Check if something in range
// If something in range lower the main arm and close the hook to grab and lift the arm with the box
// If not just pass
void loadbox(){
    stopMotors();
    delay(500);
    if(distance()<=pickup_distance){
        moveBack(75);
        delay(20);
        stopMotors();
        lowerArm();
        closeArm();
        box_color = getboxColor();
        if (box_color == 1)
            setColor(255,0,0);
        else if (box_color == 2)
            setColor(0,255,0);
        else
            setColor(0,0,255);
        liftArm();
        gotbox = true;
    }
    else{
        solve_maze();
    }
}

// If unloading area was reached stop and move back a little bit so payload is on the unloading area
// Then release the payload and reset the arm
void unload(){
    moveBack(reverse_speed);
    delay(reverse_delay);
    stopMotors();
    lowerArm();
    openArm();
    liftArm();
    // All Done Just Change the Color of the LED
    while(true) {
        setColor(255,0,0);
        delay(500);
        setColor(0,255,0);
        delay(500);
        setColor(0,0,255);
        delay(500);
    }
}



//Servo
void closeArm() {
    serl.write(10, sers, true);
    serl.stop();
    serr.write(180, sers, true);
    serr.stop();

}

void openArm() {
    serr.write(80, sers, true);
    serr.stop();
    serl.write(90, sers, true);
    serl.stop();
}

void lowerArm() {
    serm.write(50, 50, true);
    serm.stop();
}

void liftArm() {
    serm.write(120, 70, true);
    serm.stop();
}

void resetArm() {
    openArm();
    serm.write(120, sers, true);
    serm.stop();
}

float distance(){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    return duration * 0.034 / 2;
}

void setColor(int redValue, int greenValue, int blueValue) {
    analogWrite(redPin, redValue);
    analogWrite(greenPin, greenValue);
    analogWrite(bluePin, blueValue);
}

int getboxColor(){
    int red = 0;
    int green = 0;
    int blue = 0;
    digitalWrite(S2,LOW);
    digitalWrite(S3,LOW);
    
    red = pulseIn(sensorOut, LOW);
    red = map(red,  25,72,255,0);
    
    delay(100);
    
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    
    green = pulseIn(sensorOut, LOW);
    green = map(green, 30,90,255,0);
    
    delay(100);
    
    digitalWrite(S2,LOW);
    digitalWrite(S3,HIGH);
    
    blue = pulseIn(sensorOut, LOW);
    blue = map(blue, 25,70,255,0);
    
    delay(100);
    /*Serial.print(red);
    Serial.print("\t");
    Serial.print(green);
    Serial.print("\t");
    Serial.print(blue);
    Serial.print("\t\n");*/
    if (red > green) {
      if (green > red) {
        return 2;
      }
      else{
        return 1;
      }
    }
    else{
      if (blue > green){
        return 3;
      }
      else{
        return 2;
      }
    }
}
//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//
//---------------------------------------------------------------------------------//
