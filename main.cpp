
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13
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
//*************************************************************************************************************************************************************************
//                                                 Variables

float OldP = 0;         // Previous value used to calculate change in P //  DELTA P
float P = 0;            //  Proportional component
float I = 0;            //  Integral        just the sum of P over time
float OldI = 0;         //  previous value of I for calculation of Delta I
float D = 0;            //  Differential       D = P - OldP
float bp = -60;         // balance point
float pwm = 0;          // value of Pulse Width Modulation  to ENA ENB
long a = 0;             // L298N to IN 1 to 4
long b = 0;             //
const int PinR1 = 5;    //  arduino  pin 5 to l298  pin IN4
const int PinR2 = 6;    //  arduino  pin 6 to l298  pin IN3
const int PinL1 = 7;    //  arduino  pin 7 to l298  pin IN1
const int PinL2 = 8;    //  arduino  pin 8 to l298  pin IN2
const int PwmR  = 9;    //  arduino  pin 9 to l298  pin ENB
const int PwmL  = 10;   //  arduino  pin 10 to l298  pin ENA
//****************************************************************************************************************************************************


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

void setup() {
//*********************************************************************************************************************************************************
// arduino to l298 pins hopefully self explanatory
      pinMode(PinR1,OUTPUT);
      pinMode(PinR2,OUTPUT);
      pinMode(PinL1,OUTPUT);
      pinMode(PinL2,OUTPUT);
//**********************************************************************************************************************************************************
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(57600);
    while (!Serial); //

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


// load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
 //****************************************************************************************************************************************
 //  use calibration program to get your own values
    mpu.setXGyroOffset(44);//(220);
    mpu.setYGyroOffset(-21);//(76);
    mpu.setZGyroOffset(-30);//(-85);
    mpu.setXAccelOffset(-1875);//(1788); // 1688 factory default for my test chip
    mpu.setYAccelOffset(-1426);
    mpu.setZAccelOffset(2215);
 //****************************************************************************************************************************************
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
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

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

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

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


//*********************************************************************************************************************************************************************
//
// PID control based on Pseudocode from https://en.wikipedia.org/wiki/PID_controller
// and the balance point idea from https://www.youtube.com/user/jmhrvy1947

          OldP =P;                     // save value of P
          P = (ypr[2] * 1000) + bp;    // update P from MPU add bp to correct for balance
          OldI = I;                    // save old I
          I = I + (P * 0.05) ;
          I = I + ((I - OldI)*2  );     // calulate new I
          if( I >  250 ) I =  250;           // LIMIT  Stop I building up too high
          if( I < -250 ) I = -250;           // or too low value
          D = P - OldP;                      //  D differential   change in P
          pwm = ( P * 1 ) + ( I  ) + ( D * 10 ) ; // P I D

          a = 0;
          b = 0;
          if(pwm < 0){
            a = 0;
            b = 1;
             bp = bp - 0.01;
            digitalWrite(13, 0);
          }
          if(pwm > 0){
            a = 1;
            b = 0;
            bp = bp + 0.01;
            digitalWrite(13, 1);
          }
          /////////////////////////////
          // remove sign from PWM as - value has no meaning
          pwm  = abs(pwm);
          if ( pwm < 0) pwm = 0;
          if ( pwm > 255) pwm = 255;

            if(abs(ypr[2]) < abs(1.1)){
              analogWrite(PwmR, pwm);
              digitalWrite(PinR1, a);
              digitalWrite(PinR2 ,b);

              analogWrite(PwmL ,pwm);
              digitalWrite(PinL1 ,a);
              digitalWrite(PinL2 ,b);
              }
           else{
              analogWrite(PwmR , 0);
              analogWrite(PwmL , 0);
              I = 0;
              bp = -98;
              delay(1000);
           }

 //********************************************************************************************************************************************************
        #endif
    }
}
