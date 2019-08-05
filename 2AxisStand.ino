// All parts of this program dealing with obtaining data from the MPU6050 gyroscope are from code written by Jeff Rowberg

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL




#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

int Kp=10; // must be greater than 1
int Kd=3; // must be greater than 1
int Ki=2; // must be greater than 1

double TLout;
double TRout;
double FLout;
double FRout;

double baseSPD = 1250;
double TLO = baseSPD;
double FLO = baseSPD;
double TRO = baseSPD;
double FRO = baseSPD;

double deltX = 0;
double deltY = 0;

int Tleftshift = -4;
int Trightshift = 0;
int Fleftshift = -20;
int Frightshift = 195;

unsigned long time;
unsigned long tim1;
unsigned long dt;

double newangleX = 0;
double thetadesX=0.00; // Desired position
long thetaX=0;
double newangleY = 0;
double thetadesY=0.00; // Desired position
long thetaY=0;
bool notCal = true;

int val = 0;
long errX=0;
long errim1X=0;
long derrdtX=0;
long errintX=0;

int uconX=0;
int uconKpX=0;
int uconKiX=0;
int uconKdX=0;

long errY=0;
long errim1Y=0;
long derrdtY=0;
long errintY=0;

int uconY=0;
int uconKpY=0;
int uconKiY=0;
int uconKdY=0;

double TleftOut;
double TrightOut;
double FleftOut;
double FrightOut;

Servo Tleft;
Servo Tright;
Servo Fleft;
Servo Fright;

void setup() {

   Tleft.attach(3,1000,2000);
   Tright.attach(5,1000,2000);
   Fleft.attach(6,1000,2000);
   Fright.attach(9,1000,2000);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    
    pinMode(LED_PIN, OUTPUT);
    // this allows for a potentiometer to be attached and used as an emergency 
    //stop if something goes wrong
    pinMode(A3, OUTPUT);
    digitalWrite(A3,HIGH);
    

    
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    
    val = analogRead(A2);//this is the value of the potentiometer

    //if the potentiometer is turned to far it will stop all motors and halt the code until
    //it is turned back
    if(val > 10){
      Tright.write(1000);
      Tleft.write(1000);
      Fright.write(1000);
      Fleft.write(1000);
      while(val > 10){
        val = analogRead(A2);
        delay(100);
      }
      
    }
     //This allows for the user to specify any angle they want in the format (x,y)
     // where x and y are doubles, x is roll and y is pitch
     if(Serial.available() > 0){
      String input = Serial.readString();
      int ind1 = input.indexOf(',');
      String str1 = input.substring(0,ind1);
      String str2 = input.substring(ind1+1,input.length()-1);
      thetadesX = str1.toDouble();
      thetadesY = str2.toDouble();
     
     }
     
     //This calibrates the motor controllers the first time the code is ran
     if(notCal){
         Serial.println("set low");
         Tright.write(1120);
         Tleft.write(1120);
         Fright.write(1120+Frightshift);
         Fleft.write(1120);
         delay(1000);
         Serial.println("set high");
         Tright.write(1960);
         Tleft.write(1960);
         Fright.write(1960+Frightshift);
         Fleft.write(1960);
         delay(500);
         Serial.println("return low");
         Tright.write(1200);
         Tleft.write(1200);
         Fright.write(1200+Frightshift);
         Fleft.write(1200);
         delay(500);
         Serial.println("set active");  
         notCal = !notCal;
         Serial.setTimeout(50);
        }
        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI);
            //Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.print("\t");
        #endif

        long thetaX = ypr[2] * 180/M_PI;
        long thetaY = ypr[1] * 180/M_PI;
        time = millis(); 
        dt=(time-tim1);


        errX=thetadesX-thetaX;  // error
        errintX=errintX+0.5*(errX+errim1X)*dt;
        
        errY=thetadesY-thetaY;  // error
        errintY=errintY+0.5*(errY+errim1Y)*dt;

        // Since all variables defined as long integers
        // need to check that each control action is 
        // greater than 1 - best to do all multiplications at once
        uconKpX=0.1*Kp*errX; //Proportional
        uconKiX=0.0001*Ki*errintX; // Integral
        uconKdX=0.1*Kd*(errX-errim1X)/dt; //Derivative
        uconX=-1*(uconKpX+uconKiX+uconKdX);//PID control law

        // Since all variables defined as long integers
        // need to check that each control action is 
        // greater than 1 - best to do all multiplications at once
        uconKpY=0.1*Kp*errY; //Proportional
        uconKiY=0.0001*Ki*errintY; // Integral
        uconKdY=0.1*Kd*(errY-errim1Y)/dt; //Derivative
        uconY=-1*(uconKpY+uconKiY+uconKdY);//PID control law

        //stops the uconX and uconY from becoming too big
        if(uconX > 50){
          uconX = 50;
        }
        if(uconX < -50){
          uconX = -50;
        }
        if(uconY > 50){
          uconY = 50;
        }
        if(uconY < -50){
          uconY = -50;
        }


        
        //automatically sets the motors at a base speed that will be
        //raised if needed. For example, if you wanted the drone to tilt
        // left then you should raise the speed on the right two motors
        // while lowering the speed on the left two motors
        TLO = baseSPD;
        FLO = baseSPD;
        TRO = baseSPD;
        FRO = baseSPD;
        
        
        
        //if the drone is tilted too far to the left
        if(uconX < -2){
         //LEFT HIGH RIGHT LOW
        deltX = mapD(uconX,-2,-50,1,50);
        TLO += deltX;
        FLO += deltX;
        TRO -= deltX;
        FRO -= deltX;
        }
        //if the drone is tilted too far to the right
        else if(uconX >= 2){
        //RIGHT HIGH LEFT LOW
        deltX = mapD(uconX,2,50,1,50);
        TLO -= deltX;
        FLO -= deltX;
        TRO += deltX;
        FRO += deltX;       
        }
         //if the drone is tilted too far to the front
        if(uconY >= 2){
        deltY = mapD(uconY,2,50,1,40);
        //top motors low bottom motors high
        TLO -= deltY;
        TRO -= deltY;
        FLO += deltY;
        FRO += deltY;
        }
         //if the drone is tilted too far to the back
        else if(uconY < -2){
        deltY = mapD(uconY,-2,-50,1,40);
        //top motors high bottom motors low
        TLO += deltY;
        TRO += deltY;
        FLO -= deltY;
        FRO -= deltY;   
        }

        //each motor controller has a slightly different starting
        //value which it need, this adjusts for those values
        TLO += Tleftshift;
        TRO += Trightshift;
        FLO += Fleftshift;
        FRO += Frightshift;
        Tleft.write(TLO);
        Tright.write(TRO);
        Fleft.write(FLO);
        Fright.write(FRO);

        tim1=time;
        errim1X=errX;
        errim1Y=errY;
        //prints relvant data that is used for debugging
        Serial.print(uconX);
        Serial.print("\t");
        Serial.print(uconY);
        Serial.print("\t");
        Serial.print(thetadesX);
        Serial.print("\t");
        Serial.print(thetadesY);
        Serial.print("\t");
        Serial.print(TLO);
        Serial.print("\t");
        Serial.print(TRO);
        Serial.print("\t");
        Serial.print(FLO);
        Serial.print("\t");
        Serial.print(FRO);
        Serial.print("\t");
        Serial.println("\t");
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        
    }
}

//this function allows for the mapping of doubles
double mapD(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
