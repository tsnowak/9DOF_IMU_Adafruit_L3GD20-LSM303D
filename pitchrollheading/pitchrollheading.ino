//Last Edited 11/25/2014 by: Theodore Nowak

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro = Adafruit_L3GD20_Unified(30303);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
int lasttime;
/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    Serial.println("Oooops, no L3GD20 detected ... Check Wiring!");
    while(1);
  }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();
  lasttime = 0;
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t gyro_event;
  sensors_vec_t   orientation;
  
 Serial.print(millis() - lasttime); Serial.print("\t");
 lasttime = millis(); 

  /* Edited to output the raw magnitudes for accel, gyro, and mag data */
  
  // Print raw accel values
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    /*Serial.print(F("Roll: "));
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));*/
    
    Serial.print(accel._accelData.x); Serial.print("\t");
    Serial.print(accel._accelData.y); Serial.print("\t");
    Serial.print(accel._accelData.z); Serial.print("\t");
  }
  
  // Print raw gyroscope values
  gyro.getEvent(&gyro_event);
  if (dof.gyroGetOrientation(&gyro_event)){
    Serial.print(gyro._gyroData.x); Serial.print("\t");
    Serial.print(gyro._gyroData.y); Serial.print("\t");
    Serial.print(gyro._gyroData.z); Serial.print("\t");
  }
  
  // Print raw magnetometer values
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    
    Serial.print(mag._magData.x); Serial.print("\t");
    Serial.print(mag._magData.y); Serial.print("\t");
    Serial.print(mag._magData.z); Serial.print("\t");
    
    /* 'orientation' should have valid .heading data now */
   /* Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));*/
  }

  Serial.println(F(""));
  //delay(1000);
}
