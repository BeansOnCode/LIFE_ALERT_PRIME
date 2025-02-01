//Guide for breadboarding
//Connect SDA to D21
//Connect SCL to D22
//GND to GND and VCC to 3V3

//Referenced from https://randomnerdtutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

constexpr uint8_t Buzzer = 4; //Define Buzzer Pin
constexpr uint8_t sampleTime = 5; //Sampling time for differential acceleration (ms)
constexpr uint16_t threshold = 1.5; //Threshold for determining fall

Adafruit_MPU6050 mpu;
float xAcc, yAcc, zAcc; //Floats to store previous accelerations
float prev_xAcc = 0.0, prev_yAcc = 0.0, prev_zAcc = 1.0; //Floats to store new accelerations (z set to 1G as it tends to be around there at rest)


void setup() {
  Serial.begin(115200);

  Serial.println("Adafruit MPU6050 test!");

  while(!mpu.begin()){
    Serial.println("Error: MPU6050 Not Found");
    delay(10);//Give user opportunity to fix
  }
  Serial.println("MPU6050 Found!");

  //Play around with these values when testing
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void loop() {
  //Define event variables
  sensors_event_t a,g,temp;  
  static unsigned int lastSampleTime = 0;

  //From Lim, Donha et.al (Algorithm Paper) && Kartik9250 (Reference Code Github)
  //Will be implementing ADSVM
  //First Convert to Gs (#/9.8 m/s^2)
  //Then compare to old values
  //Get magnitude of change
  //Compare to threshold (A > Threshold = beep)
  //Save new values for next iteratioln
  if(millis() - lastSampleTime > sampleTime)
  {
    lastSampleTime = millis();
    mpu.getEvent(&a, &g, &temp);

    xAcc = a.acceleration.x / 9.8;
    yAcc = a.acceleration.y / 9.8;
    zAcc = a.acceleration.z / 9.8;

    float xAccDiff = xAcc - prev_xAcc;
    float yAccDiff = yAcc - prev_yAcc;
    float zAccDiff = zAcc - prev_zAcc;

    float AccDiffMag = sqrt(xAccDiff * xAccDiff + yAccDiff * yAccDiff + zAccDiff * zAccDiff);

    Serial.println(AccDiffMag);

    if(AccDiffMag > threshold)
    {
      tone(Buzzer, 967, 1000);
    }

    //Current values saved for next iteration
    prev_xAcc = xAcc;
    prev_yAcc = yAcc;
    prev_zAcc = zAcc;


  }

}
