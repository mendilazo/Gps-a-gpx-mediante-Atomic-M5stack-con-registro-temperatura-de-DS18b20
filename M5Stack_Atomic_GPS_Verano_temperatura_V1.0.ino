#include <Arduino.h>
#include "M5Atom.h"
#include <SPI.h>
#include "FS.h"
#include <SD.h>
#include <TinyGPSPlus.h>
#include "esp_log.h"
#include <CSV_Parser.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define TIME_ZONE 0             // Time compensation 0=summer Spain
#define RECORD_FREQUENCY 5000   // Record frequency  

#define ACTIVATE_DEEP_SLEEP_ONCLICK true
#define GPS_TIMEOUT    5000     /* GPS acquisition Timeout  */
#define DISPLAY_GPS_DATA true   /* Display GPS message 

static const char* TAG = "m5atom_gps";

/****************************/
/*        PROTOTYPES        */
/****************************/
static void readGPS(unsigned long ms);
static void printGPSData();
static void createDataFile();
static void addGPXPoint();
static void blink_led_orange();
static void blink_led_red();
static void blink_led_blue();
static void blink_led_green();
static void updateStatFile();
static void printWakeupReason();

// Onewire & DallasTemperature const
const int oneWireBus = 26;     
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// The TinyGPS++ object
TinyGPSPlus gps;
HardwareSerial gps_uart(1);

bool firstStart = true;         // Create GPX file in first start 
char filepath_gpx[25];          // GPX file name 
char filepath_stats[25];        // Stat file name 
char today_folder[15];          // Today folder 
char point_date[22];
float prev_lat = NULL;            // No previous point on startup
float prev_long;  
bool ENABLE_GPS = true;
bool DESACTIVATE_GPS = false;
#define FORCE_UPDATE_STATS true
#define SPEED_BUFFER_SIZE 10

typedef struct TODAYSTATS
{
  float dist;
  float speed_max;
  float speed_mean; 
  double  speedbuffer[SPEED_BUFFER_SIZE];
  bool  speedbufferfull = false;
  int   speedbufferpos = 0;
} TODAYSTATS_t;

TODAYSTATS_t today_stats;

void external_button_pressed()
{
  if ( !ENABLE_GPS ) {
    
  } else {
    blink_led_green();
  }
}

void setup() {
    // Start the DS18B20 sensor
    sensors.begin();
   
    // begin(bool SerialEnable , bool I2CEnable , bool DisplayEnable )
    M5.begin(true,false,true);   

    // Disable WiFi modem to save power
    // Désactive le modem WiFi pour économiser de la batterie
    //esp_wifi_stop();  

    SPI.begin(23,33,19,-1);
    if(!SD.begin(-1, SPI, 40000000)){
      ESP_LOGE(TAG, "initialization failed!");
    } else {
      sdcard_type_t Type = SD.cardType();

        Serial.printf("SDCard Type = %d \r\n",Type);
        Serial.printf("SDCard Size = %d \r\n" , (int)(SD.cardSize()/1024/1024));
    }

    // Open Serial port with GPS module 
    // Ouvre le port série avec le module HPS
    gps_uart.begin(9600,SERIAL_8N1,22,-1);

    delay(250);

    M5.dis.drawpix(0,0,0);
    
    // Disable GPS when pressing M5Atom Lite Button (GPIO39)
    pinMode(39, INPUT);
    attachInterrupt(39, [] {
      ENABLE_GPS = !ENABLE_GPS;
      ESP_LOGI(TAG,"Change GPS stat %u", ENABLE_GPS);
      if ( !ENABLE_GPS ) DESACTIVATE_GPS = true;
    }, RISING);

    printWakeupReason();
}

void loop() 

{
  if ( DESACTIVATE_GPS ) {
    DESACTIVATE_GPS = false;
    blink_led_orange();
    blink_led_orange();
    // On met en sommeil le module
    if (ACTIVATE_DEEP_SLEEP_ONCLICK) {
      // Le tacker GPS pourra être 
      ESP_LOGI(TAG,"Wakeup on button activated");
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_39,0);
      esp_deep_sleep_start();
    }
  }
  // GPS module is activated 
  if ( ENABLE_GPS ) {
    readGPS(500);

    if (gps.location.isValid() && gps.date.isValid() ) {
        if ( firstStart ) {
          createDataFile();
          delay(250);
          firstStart = false;
        } 

        addGPXPoint();
        
    }  
        
    if (millis() > GPS_TIMEOUT && gps.charsProcessed() < 10)
      ESP_LOGW(TAG, "No GPS data received: check wiring or position");
  }

  delay(RECORD_FREQUENCY);
}

static void createDataFile()
{
  // GPX File name
  // Nom du fichier GPX
  sprintf(today_folder, "/%04d-%02d-%02d", gps.date.year(), gps.date.month(), gps.date.day() );
  if (!SD.exists(today_folder)){
    SD.mkdir(today_folder);
    ESP_LOGI(TAG, "Create today folder %s", today_folder);
  } else {
    ESP_LOGI(TAG, "Today folder already exists %s\n", today_folder);
  }
  // GPX file path
  sprintf(filepath_gpx, "%s/track.gpx", today_folder);
  sprintf(filepath_stats, "%s/stats.csv", today_folder);
  ESP_LOGI(TAG, "GPX file path %s\n", filepath_gpx);
  ESP_LOGI(TAG, "Stats file path %s\n", filepath_stats);
  
  // Reload today stats
  if ( SD.exists(filepath_stats) ) {
    File statsFile = SD.open(filepath_stats, FILE_READ);
    if ( statsFile ) {
      CSV_Parser cp(/*format*/ "sf", /*has_header*/ true, /*delimiter*/ ',');
      cp.readSDfile(filepath_stats);

      if ( cp.getRowsCount() > 0 ) {
        float *val_col = (float*)cp["value"];

        if (val_col) 
          today_stats.dist        = (float)val_col[0]; 
          today_stats.speed_max   = (float)val_col[1];  
          today_stats.speed_mean  = (float)val_col[2];  
          cp.print();
          //Serial.printf("dist %f | max speed %f | mean speed %f  \n", today_stats.dist, today_stats.speed_max, today_stats.speed_mean);
      } else {
        ESP_LOGW(TAG,"Stat file removed because probably corrupted. I'll be re-saved next time");
        SD.remove(filepath_stats);
      } 
    }  
  }  

  // Create today GPX file if not exists  
  if ( !SD.exists(filepath_gpx) ) {
    ESP_LOGI(TAG, "Create new GPX file %s", filepath_gpx);
    // Create GPX file on startup if not exists
    File gpxFile = SD.open(filepath_gpx, FILE_WRITE);
    // GPX file header
    if ( gpxFile ) {
      gpxFile.print(F(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n"
        "<gpx xmlns:gpxtpx1=\"http://www.garmin.com/xmlschemas/TrackPointExtension/v1\" \r\n"
        "xmlns=\"http://www.topografix.com/GPX/1/1\" \r\n"
        "xmlns:trp=\"http://www.garmin.com/xmlschemas/TripExtensions/v1\" \r\n"
        "xmlns:gpxx=\"http://www.garmin.com/xmlschemas/GpxExtensions/v3\" \r\n"
        "xmlns:gpxtpx=\"http://www.garmin.com/xmlschemas/TrackPointExtension/v2\">\r\n"
         "\t<trk>\r\n<trkseg>\r\n"));  
      gpxFile.print(F("</trkseg>\r\n</trk>\r\n</gpx>\r\n"));
      gpxFile.close();
    } else {
      ESP_LOGW(TAG, "Impossible to open %s GPX file", filepath_gpx);
      blink_led_red();
    }  
  } else {
    ESP_LOGI(TAG, "%s file already exists", filepath_gpx);
  }  
}

/********************************************/
/*    READ MESSAGE SENT BY GPS MODULE       */
/********************************************/
static void readGPS(unsigned long ms)
{
  ESP_LOGI(TAG, "Read GPS stream");
  bool led_state = true;
  unsigned long start = millis();
  do 
  {
    while (gps_uart.available())
      gps.encode(gps_uart.read());
      //Serial.println( (millis() - start) % 250);
      if ( (millis() - start) % 250 != 80 ) {
        led_state = !led_state;
      }
      if ( led_state ) 
        M5.dis.drawpix(0,0,0x0099ff);
      else
        M5.dis.drawpix(0,0,0);
      
  } while (millis() - start < ms);

  if ( DISPLAY_GPS_DATA ) printGPSData();
  M5.dis.drawpix(0,0,0);
}

static void addGPXPoint()
{
  // Only if speed > 5 km/h, update max/mean speed, distance traveled
  if ( gps.speed.kmph() > 5 ) {
    // Add new speed point in the buffer
    double current_speed = gps.speed.kmph();

    today_stats.speedbuffer[today_stats.speedbufferpos] = current_speed;
    today_stats.speedbufferpos += 1;

    if ( current_speed > today_stats.speed_max ) today_stats.speed_max = current_speed;
      
    if ( today_stats.speedbufferpos >= 10 ) {
      today_stats.speedbufferpos = 0;
      today_stats.speedbufferfull = true;
    }
      
    if ( today_stats.speedbufferfull ) {
      float speed_mean = 0;
      for (int l = 0; l < SPEED_BUFFER_SIZE; l++)
      {
        speed_mean += today_stats.speedbuffer[l];
        Serial.print("add to mean"); Serial.println(today_stats.speedbuffer[l]);
      }
      Serial.print("speed_mean total"); Serial.println(speed_mean);
      
      speed_mean = speed_mean / SPEED_BUFFER_SIZE;
      today_stats.speed_mean = speed_mean;
    }    
    
    // Estimate distance traveled from the lasted position
    if ( prev_lat != NULL ) {
      double _distance = gps.distanceBetween(gps.location.lat(), gps.location.lng(), prev_lat, prev_long);
      Serial.print("distance = "); Serial.println(_distance);
      // Ajoute la distance parcourue si < 1 km
      if ( _distance > 0 ) {
        today_stats.dist += _distance;

        updateStatFile();
      }
      prev_lat = gps.location.lat();
      prev_long = gps.location.lng();
      //ESP_LOGI(TAG,"Distance traveled %d", _distance );
    } else {
      prev_lat = gps.location.lat();
      prev_long = gps.location.lng();
    }
  }

  if ( FORCE_UPDATE_STATS ) updateStatFile();

  sprintf(point_date, "%4d-%02d-%02dT%02d:%02d:%02dZ",gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour() + TIME_ZONE, gps.time.minute(),gps.time.second());

  File gpxFile = SD.open(filepath_gpx, FILE_WRITE);
  if( !gpxFile ) {
    ESP_LOGW(TAG, "Impossible to open %s GPX file", filepath_gpx);
    blink_led_red();
  } else {
    
    // V1.0 //
    sensors.requestTemperatures();
    // V1.0 //
    ESP_LOGI(TAG, "Add new point %s", point_date);
    double _lat = gps.location.lat();
    double _lng = gps.location.lng();
    double _alt = gps.altitude.meters();
    double _hdop = gps.hdop.hdop();
    int    _sat  = gps.satellites.value();
     // V1.0 //
    float temperatureC = sensors.getTempCByIndex(0);   
     // V1.0 //
    unsigned long filesize = gpxFile.size();
    // back up the file pointer to just before the closing tags
    filesize -= 27;
    gpxFile.seek(filesize);
    gpxFile.print(F("<trkpt lat=\"")); 
    gpxFile.print(_lat,6);
    gpxFile.print(F("\" lon=\""));
    gpxFile.print(_lng,6);
    gpxFile.println(F("\">"));
    gpxFile.print(F("<time>"));
    gpxFile.print(point_date);
    gpxFile.println(F("</time>"));  
     // Temperatura Ambiente
    gpxFile.println(F("<extensions>"));
    gpxFile.println(F("<gpxtpx:TrackPointExtension>"));
    gpxFile.print(F("<gpxtpx:atemp>"));
    gpxFile.print(temperatureC);
    gpxFile.println(F("</gpxtpx:atemp>")); 
    gpxFile.println(F("</gpxtpx:TrackPointExtension>"));
    gpxFile.println(F("</extensions>"));
    // Satellites
    gpxFile.print(F("<sat>"));
    gpxFile.print(_sat);
    gpxFile.println(F("</sat>"));    
    // Elevation | Altitude 
    gpxFile.print(F("<ele>")); 
    gpxFile.print(_alt,1);
    gpxFile.print(F("</ele>\r\n<hdop>")); 
    gpxFile.print(_hdop,3);
    gpxFile.println(F("</hdop>\r\n</trkpt>"));
    gpxFile.print(F("</trkseg>\r\n</trk>\r\n</gpx>\r\n"));
    gpxFile.close();

    blink_led_blue();
  }  
}

/*********************************/
/*  CREATE OR UPDATE STAT FILE   */
/*********************************/
static void updateStatFile()
{
  File statsFile = SD.open(filepath_stats, FILE_WRITE);
  if(!statsFile) {
    ESP_LOGW(TAG, "Impossible to open %s stats file", filepath_stats);
  } else {
    ESP_LOGI(TAG, "Add stats data to file");
    
    statsFile.print(F("key,value,unit\r\n")); 
    // Distance parcourue aujourd'hui en mètres
    statsFile.print(F("distance,"));
    statsFile.print(today_stats.dist);
    statsFile.print(F(",m"));
    statsFile.print(F("\r\n"));
    // Distance parcourue aujourd'hui en km
    statsFile.print(F("distance,"));
    statsFile.print(today_stats.dist / 1000., 2);
    statsFile.print(F(",km"));
    statsFile.print(F("\r\n"));
    // Vitesse maxi aujourd'hui
    statsFile.print(F("speed_max,"));
    statsFile.print(today_stats.speed_max);
    statsFile.print(F(",hm/h"));
    statsFile.print(F("\r\n"));
    // Vitesse moyenne aujourd'hui
    statsFile.print(F("speed_mean,"));
    statsFile.print(today_stats.speed_mean);
    statsFile.print(F(",km/h"));
    statsFile.print(F("\r\n"));
    statsFile.close();
  }  
}
static void printGPSData()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(", age:"));
    Serial.print(gps.location.age(), 6);    
    Serial.print(F(", hdop:"));
    Serial.print(gps.hdop.hdop(), 3);
  }
  else
  {
    Serial.print(F("INVALID LOCATION"));
  }

  Serial.print(F(" Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID DATE"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    //if (gps.time.centisecond() < 10) Serial.print(F("0"));
    //Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID TIME"));
  }

  Serial.print(F(" Altitude (m):"));
  Serial.print(gps.altitude.meters());

  Serial.print(F(" Speed (km/h):"));
  if (gps.speed.isValid() )
  {
    Serial.print(gps.speed.kmph());
  }
  else
  {
    Serial.print(F("INVALID SPEED"));
  }

  Serial.print(F(" Course:"));
  if (gps.course.isValid() )
  {
    Serial.print(gps.course.deg());
  }
  else
  {
    Serial.print(F("INVALID COURSE"));
  }

  Serial.println();
}

static void printWakeupReason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_GPIO : Serial.println("Wakeup caused by GPIO"); break;
    case ESP_SLEEP_WAKEUP_UART : Serial.println("Wakeup caused by UART"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

static void blink_led_orange()
{
  M5.dis.drawpix(0,0,0xe68a00);
  delay(200);
  M5.dis.drawpix(0,0,0);
  delay(200);
}

static void blink_led_red()
{
  M5.dis.drawpix(0,0,0xff3300);
  delay(200);
  M5.dis.drawpix(0,0,0);
  delay(200);
}

static void blink_led_blue()
{
  M5.dis.drawpix(0,0,0x0000cc);
  delay(200);
  M5.dis.drawpix(0,0,0);
  delay(200);
}

static void blink_led_green()
{
  M5.dis.drawpix(0,0,0x00ff00);
  delay(200);
  M5.dis.drawpix(0,0,0);
  delay(200);
}
