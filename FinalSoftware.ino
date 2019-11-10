#include <SD.h>
#include <SPI.h>
#include <StackArray.h>
#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <Keypad.h>
#include <math.h>
#include "DEV_Config.h"
#include "LCD_Driver.h"
#include "LCD_GUI.h"
#include "Fonts.h"

#define Black 0x0000
#define White 0xFFFF
#define Light Gray 0xBDF7
#define Dark Gray 0x7BEF
#define Red 0xF800
#define Yellow 0xFFE0
#define Orange 0xFBE0
#define Brown 0x79E0
#define Green 0x7E0
#define Cyan 0x7FF
#define Blue 0x1F
#define Pink 0xF81F

int CS_PIN = 53;
File file;
File GMTfile;
File numOfMeasurefile;

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char hexaKeys[ROWS][COLS] = {
  {'1', '4', '7', '*'},
  {'2', '5', '8', '0'},
  {'3', '6', '9', '#'},
  {'A', 'B', 'C', 'D'}
};

byte rowPins[ROWS] = {23, 25, 27, 29}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {22, 24, 26, 28}; //connect to the column pinouts of the keypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
const float mps = 0.514444;
StackArray <float> finalSpeed;//Saves all speed data
StackArray <float> finalCourse;
float speedAVG;
float courseAVG;
float speedCoordinates;
int successPercentage;
boolean forTheFirstTime = true;
char* fileName = "MEASURE.txt";
enum State {
  WAIT,
  MEASURE,
  FINAL_DISPLAY,
  CHANGE_TIME,
  SAVE_DATA
};

struct RMC {
  int time_hh;
  int time_mm;
  int time_ss;
  bool data_valid;
  int lat_deg;
  float lat_sec;
  String NS_indicator;
  int lon_deg;
  float lon_sec;
  String WE_indicator;
  float ground_speed;
  float course;
  int date_dd;
  int date_mm;
  int date_yy;
};

int rmc_counter;
RMC first_RMC, last_RMC;
int number_of_GPS_loss;
int absgmt;
AltSoftSerial mySerial; // TX, RX. GPS
State state;

void setup() {
  System_Init();
  initializeSD();
  createFile(fileName);
  LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;
  LCD_Init( Lcd_ScanDir, 200);
  mySerial.begin(4800);//Contact to the GPS
  Serial.begin(9600);//Contact to the console
  while (!Serial) {
    Serial.print("waiting");
  }
  Flag();
  resetToWait();
}

void loop() {
  char key = customKeypad.getKey();
  switch (key) {
    case '1':
      if (state == WAIT) {
        startMeasurement();
      }
      break;
    case '2':
      if (state == MEASURE) {
        stopMeasurement();
      }
      break;
    case '3':
      if (state == FINAL_DISPLAY) {
        SaveData(speedAVG, speedCoordinates, courseAVG, successPercentage);
        Serial.println("Data saved");
        resetToWait();
      }
    case '4':
      if (state == FINAL_DISPLAY) {
        resetToWait();
      }
      break;
    case '5':
      if (state == WAIT) {
        gmtPrint();
        state = CHANGE_TIME;
      }
      break;
    case 'A':
      if (state == CHANGE_TIME) {
        plus1();
        printETgmt();
        state = CHANGE_TIME;
      }
      break;
    case 'B':
      if (state == CHANGE_TIME) {
        minus1();
        printETgmt();
        state = CHANGE_TIME;
      }
      break;
    case 'C':
      if (state == CHANGE_TIME) {
        resetToWait();
      }
      break;
    case 'D':
      if (state == WAIT) {
        SD.remove(fileName);
        Serial.println("Document deleted");
      }
      break;
    default:
      if (state == MEASURE) {
        if (rmc_counter >= 60) {
          stopMeasurement();
        } else {
          measure();
        }
      }
      break;
  }
  if (Serial.available()) {
    Serial.end();
  }
}

void startMeasurement() {
  Serial.print("Starting Measurement");
  startMeasuringPrint();
  state = MEASURE;
}

float avg(StackArray <float>  stack)
{
  float sum = 0;
  int m = stack.count();//Counter for the 'finalSpeed' array
  for (int i = 0; i < m; i++)
  {
    float v = stack.pop();
    sum = sum + v;//Adds the speed to the amount
  }
  return sum / m;
}

float avgSpeed()
{
  float sum = 0;
  int m = finalSpeed.count();//Counter for the 'finalSpeed' array
  for (int i = 0; i < m; i++)
  {
    float v = finalSpeed.pop();
    sum = sum + v;//Adds the speed to the amount
  }
  return sum / m;
}

float avgCourse()
{
  float sum = 0;
  int m = finalCourse.count();//Counter for the 'finalSpeed' array
  for (int i = 0; i < m; i++)
  {
    float v = finalCourse.pop();
    sum = sum + v;//Adds the speed to the amount
  }
  return sum / m;
}
void stopMeasurement() {
  Serial.print("Stoping Measurement");
  state = FINAL_DISPLAY;
  speedAVG = avgSpeed();
  speedAVG = speedAVG * mps * 60;
  courseAVG = avgCourse();
  speedCoordinates = DistanceBetweenCoordinates(first_RMC , last_RMC);
  successPercentage =  (rmc_counter - number_of_GPS_loss) * 100 / rmc_counter;
  printResults(speedAVG, speedCoordinates, courseAVG, successPercentage);
  printSaveData(speedAVG, speedCoordinates, courseAVG, successPercentage);
}

void resetToWait() {
  Serial.println("Main Menu");
  menu();
  rmc_counter = 0;
  number_of_GPS_loss = 0;
  speedAVG = 0;
  courseAVG = 0;
  speedCoordinates = 0;
  successPercentage = 0;
  forTheFirstTime = true;
  state = WAIT;
}

void measure() {
  String s = readGPSLine();
  if (s.length()<=75)
  {
   if (isRMC(s)) {
    Serial.println(s);
    RMC a = getRMCData(s);
    if (GPSSignal(a))
    {
      Serial.println(rmc_counter);
      last_RMC = getRMCData(s);
      finalSpeed.push(last_RMC.ground_speed);
      finalCourse.push(last_RMC.course);
      if (forTheFirstTime) {
        first_RMC = last_RMC;
        forTheFirstTime = false;
      }
    }
    else
    {
      number_of_GPS_loss ++;
    }
    rmc_counter++;
    String strs = String(rmc_counter);
    char * c = &strs[0];
    GUI_DisString_EN(240, 190, c, &Font24 , Black, White);
  } 
  }
}

bool GPSSignal(RMC r)
{
  return r.data_valid;
}

String readGPSLine() {
  String result = "";
  while (true) {
    if (mySerial.available()) {
      char readMS = mySerial.read();
      if (readMS == '\r') {
        return result;
      } else {
        result += readMS;
      }
    }
  }
}

// test if s is an RMC message
bool isRMC(String s) {
  if (s.substring(1, 7).equals("$GPRMC")) {
    return  true;
  } else {
    return false;
  }
}

RMC getRMCData(String s) {
  RMC result;
  result.time_hh = s.substring(8, 10).toInt();
  result.time_mm = s.substring(10, 12).toInt();
  result.time_ss = s.substring(12, 14).toInt();
  result.data_valid = s.substring(19, 20).equals("A");
  result.lat_deg = s.substring(21, 23).toInt();
  result.lat_sec = s.substring(23, 30).toFloat() / 60.0;
  result.NS_indicator = s.substring(31, 32);
  result.lon_deg = s.substring(33, 36).toInt();
  result.lon_sec = s.substring(36, 43).toFloat() / 60.0;
  result.WE_indicator = s.substring(44, 45);
  int comma_at_speed_end = s.indexOf(",", 46);
  result.ground_speed = s.substring(46, comma_at_speed_end).toFloat();
  int comma_at_course_end = s.indexOf(",", comma_at_speed_end + 1);
  result.course = s.substring(comma_at_speed_end + 1, comma_at_course_end).toFloat();
  result.date_dd = s.substring(comma_at_course_end + 1, comma_at_course_end + 3).toInt();
  result.date_mm = s.substring(comma_at_course_end + 3, comma_at_course_end + 5).toInt();
  result.date_yy = s.substring(comma_at_course_end + 5, comma_at_course_end + 7).toInt();
  return result;
}

void debugPrintRMC(RMC r) {
  Serial.print(r.time_hh);
  Serial.print(":");
  Serial.print(r.time_mm);
  Serial.print(":");
  Serial.print(r.time_ss);
  Serial.print(" ");
  Serial.print(r.data_valid);
  Serial.print(" ");
  Serial.print(r.lat_deg);
  Serial.print(".");
  Serial.print(r.lat_sec * 10000000);
  Serial.print(" ");
  Serial.print(r.NS_indicator);
  Serial.print(" ");
  Serial.print(r.lon_deg);
  Serial.print(".");
  Serial.print(r.lon_sec * 10000000);
  Serial.print(" ");
  Serial.print(r.WE_indicator);
  Serial.print(" ");
  Serial.print(r.ground_speed);
  Serial.print(" ");
  Serial.print(r.course);
  Serial.print(" ");
  Serial.print(r.date_dd);
  Serial.print("/");
  Serial.print(r.date_mm);
  Serial.print("/");
  Serial.print(r.date_yy);
  Serial.println("");
}

int DistanceBetweenDegrees(int startDeg, int finalDeg)
{
  if (startDeg == finalDeg)
  {
    return 0;
  }
  if (startDeg > finalDeg)
  {
    return startDeg - finalDeg;
  }
  if (finalDeg > startDeg)
  {
    return finalDeg - startDeg;
  }
  return 360;
}

float DistanceBetweenCoordinates(RMC start, RMC last)
{
  int startLatDeg = start.lat_deg;
  float startLatSec = start.lat_sec;
  int startLonDeg = start.lon_deg;
  float startLonSec = start.lon_sec;
  int lastLatDeg = last.lat_deg;
  float lastLatSec = last.lat_sec;
  int lastLonDeg = last.lon_deg;
  float lastLonSec = last.lon_sec;

  int DistanceBetweenLat = DistanceBetweenDegrees(startLatDeg , lastLatDeg);
  int DistanceBetweenLon = DistanceBetweenDegrees(startLonDeg , lastLonDeg);

  if (DistanceBetweenLat == 0 & DistanceBetweenLon == 0)
  {
    float result = distance(startLatSec, startLonSec, lastLatSec, lastLonSec);
    return result;
  }
  else
  {
    float result = distance(startLatDeg + startLatSec, startLonDeg + startLonSec, lastLatDeg + lastLatSec, lastLonDeg + lastLonSec);
    return result;
  }
}

float distance(float phi1, float lamda1, float phi2, float lamda2)
{
  float ER = 6371000;
  phi1 = phi1 / 180 * M_PI;
  phi2 = phi2 / 180 * M_PI;
  lamda1 = lamda1 / 180 * M_PI;
  lamda2 = lamda2 / 180 * M_PI;
  float dlamda = lamda2 - lamda1;
  float dphi = phi2 - phi1;
  float phim = (phi2 + phi1) / 2;

  float x = dlamda * cos(phim);
  float y = dphi;

  float d = ER * sqrt(x * x + y * y);
  return d;
}

void Flag()
{
  LCD_Clear(White);
  GUI_DrawRectangle(0, 30, 480, 70, BLUE, DRAW_FULL, DOT_PIXEL_DFT);
  GUI_DrawRectangle(0, 250, 480, 290, BLUE, DRAW_FULL, DOT_PIXEL_DFT);
  GUI_DrawLine(240, 100, 180, 190, BLUE, LINE_SOLID, DOT_PIXEL_2X2);
  GUI_DrawLine(180, 190, 300, 190, BLUE, LINE_SOLID, DOT_PIXEL_2X2);
  GUI_DrawLine(300, 190, 240, 100, BLUE, LINE_SOLID, DOT_PIXEL_2X2);
  GUI_DrawLine(240, 220, 180, 130, BLUE, LINE_SOLID, DOT_PIXEL_2X2);
  GUI_DrawLine(180, 130, 300, 130, BLUE, LINE_SOLID, DOT_PIXEL_2X2);
  GUI_DrawLine(300, 130, 240, 220, BLUE, LINE_SOLID, DOT_PIXEL_2X2);
  delay(1500);
}

void menu()
{
  GUI_Clear(Black);
  GUI_DisString_EN(170, 20, "Main Menu", &Font24 , Black, White);
  GUI_DisString_EN(50, 50, "What each button does?", &Font24 , Black, White);
  GUI_DrawLine(0, 80, 480, 80, WHITE, LINE_SOLID, DOT_PIXEL_2X2);

  GUI_DisString_EN(45, 100, "'1'", &Font24 , Black, White);
  GUI_DisString_EN(40, 130, "Start", &Font16 , Black, White);
  GUI_DisString_EN(20, 150, "Measuring", &Font16 , Black, White);

  GUI_DisString_EN(215, 100, "'2'", &Font24 , Black, White);
  GUI_DisString_EN(215, 130, "Stop", &Font16 , Black, White);
  GUI_DisString_EN(190, 150, "Measuring", &Font16 , Black, White);

  GUI_DisString_EN(395, 100, "'3'", &Font24 , Black, White);
  GUI_DisString_EN(395, 130, "Save", &Font16 , Black, White);
  GUI_DisString_EN(370, 150, "Measuring", &Font16 , Black, White);

  GUI_DisString_EN(45, 200, "'4'", &Font24 , Black, White);
  GUI_DisString_EN(35, 230, "Delete", &Font16 , Black, White);
  GUI_DisString_EN(20, 250, "Measuring", &Font16 , Black, White);

  GUI_DisString_EN(215, 200, "'5'", &Font24 , Black, White);
  GUI_DisString_EN(205, 230, "Change", &Font16 , Black, White);
  GUI_DisString_EN(195, 250, "GMT time", &Font16 , Black, White);

  GUI_DisString_EN(395, 200, "'D'", &Font24 , Black, White);
  GUI_DisString_EN(397, 230, "Delete", &Font16 , Black, White);
  GUI_DisString_EN(397, 250, "File", &Font16 , Black, White);
}

void printResults(float vGPS, float vPoint, float course, int som)
{

  GUI_Clear(Black);
  GUI_DrawLine(0, 70, 480, 70, WHITE, LINE_SOLID, DOT_PIXEL_2X2);
  GUI_DisString_EN(100, 20, "Measuring Results", &Font24 , Black, White);
  GUI_DrawLine(0, 200, 270, 200, WHITE, LINE_SOLID, DOT_PIXEL_2X2);
  GUI_DrawLine(270, 70, 270, 320, WHITE, LINE_SOLID, DOT_PIXEL_2X2);
  GUI_DrawLine(270, 250, 480, 250, WHITE, LINE_SOLID, DOT_PIXEL_2X2);
  GUI_DisString_EN(290, 260, "GPS Accuracy:", &Font20 , Black, White);
  if (som < 10)
  {
    GUI_DisNum(370, 295, som, &Font24 , Blue, White);
    GUI_DisString_EN(385, 295, "%", &Font24 , Blue, White);
  }
  if (som < 100 && som > 10)
  {
    GUI_DisNum(370, 295, som, &Font24 , Blue, White);
    GUI_DisString_EN(403, 295, "%", &Font24 , Blue, White);
  }
  if (som == 100)
  {
    GUI_DisNum(345, 295, som, &Font24 , Blue, White);
    GUI_DisString_EN(395, 295, "%", &Font24 , Blue, White);
  }
  GUI_DisString_EN(10, 90, "Average Speed", &Font24 , Black, White);
  GUI_DisString_EN(70, 120, "By GPS:", &Font24 , Black, White);
  String strs = String(vGPS);
  char * c = &strs[0];
  GUI_DisString_EN(40, 160, c, &Font24 , Blue, White);
  GUI_DisString_EN(160, 170, "m/mi", &Font16 , Black, White);

  GUI_DisString_EN(10, 220, "Average Speed", &Font24 , Black, White);
  GUI_DisString_EN(40, 250, "By points:", &Font24 , Black, White);
  strs = String(vPoint);
  c = &strs[0];
  GUI_DisString_EN(40, 290, c, &Font24 , Blue, White);
  GUI_DisString_EN(160, 300, "m/mi", &Font16 , Black, White);


  GUI_DisString_EN(320, 90, "Average", &Font24 , Black, White);
  GUI_DisString_EN(305, 120, " Course:", &Font24 , Black, White);
  strs = String(course);
  c = &strs[0];
  GUI_DisString_EN(330, 160, c, &Font24 , Yellow, Black);
  GUI_DisString_EN(345, 200, "degrees", &Font16 , Black, White);
}

void gmtPrint()
{
  GUI_Clear(Black);
  GUI_DisString_EN(110, 20, "Change GMT Menu", &Font24 , Black, White);
  GUI_DrawLine(0, 250, 480, 250, WHITE, LINE_SOLID, DOT_PIXEL_2X2);
  GUI_DisString_EN(20, 280, "A = +1", &Font24 , Black, White);
  GUI_DisString_EN(180, 280, "B = -1", &Font24 , Black, White);
  GUI_DisString_EN(320, 280, "C = save", &Font24 , Black, White);
  GUI_DisString_EN(170, 130, "GMT(", &Font24 , Black, White);
  printETgmt();
}

int readGMT()
{
  GMTfile = SD.open("GMT.txt", FILE_READ);
  String s = readLine();
  int gmt = s.toInt();
  GMTfile.close();
  Serial.println(s);
  return gmt;
}

void setGMT(int gmt)
{
  SD.remove("GMT.txt");
  GMTfile = SD.open("GMT.txt", FILE_WRITE);
  GMTfile.println(gmt);
  GMTfile.close();
}

void plus1()
{
  int gmt = readGMT();
  gmt = gmt + 1;
  setGMT(gmt);
}
void minus1()
{
  int gmt = readGMT();
  gmt = gmt - 1;
  setGMT(gmt);
}
void printETgmt()
{
  int gmt = readGMT();
  GUI_DrawRectangle(240, 120, 350, 190, Black, DRAW_FULL, DOT_PIXEL_DFT);
  if (gmt > 0)
  {
    GUI_DisString_EN(240, 130, "+", &Font24 , Black, White);
  }
  if (gmt < 0)
  {
    GUI_DisString_EN(240, 130, "-", &Font24 , Black, White);
    gmt = - gmt;
  }
  String strs = String(gmt);
  char * c = &strs[0];
  GUI_DisString_EN(260, 130, c, &Font24 , Black, White);
  if (gmt > 9)
  {
    GUI_DisString_EN(295, 130, ")", &Font24 , Black, White);
  }
  else
  {
    GUI_DisString_EN(275, 130, ")", &Font24 , Black, White);
  }
}

void startMeasuringPrint()
{
  GUI_Clear(Black);
  GUI_DisString_EN(60, 160, "Measuring starts in:", &Font24 , Black, White);
  for (int i = 3; i >= 0; i --)
  {
    String strs = String(i);
    char * c = &strs[0];
    GUI_DisString_EN(220, 210, c, &Font24 , Black, White);
    delay(1000);
  }
  GUI_DrawRectangle(0, 71, 480, 320, Black, DRAW_FULL, DOT_PIXEL_1X1);
  GUI_DisString_EN(150, 160, "Calculates", &Font24 , Black, White);
}

void printSaveData(float speedAVG, float speedCoordinates, float courseAVG, int successPercentage)
{
  String time1 = timeC();
  Serial.println("");
  Serial.print("Time of measuring: ");
  Serial.print(time1);
  Serial.write("\nAVG speed: ");
  Serial.print(speedAVG);
  Serial.write("\nAVG Course: ");
  Serial.print(courseAVG);
  Serial.write("\nSpeed by Coordinates: ");
  Serial.print(speedCoordinates);
  Serial.write("\nGPS Accuracy : ");
  Serial.print(successPercentage);
  int startLonDeg = first_RMC.lon_deg;
  float startLonSec = first_RMC.lon_sec*100000000;
  int startLatDeg = first_RMC.lat_deg;
  float startLatSec = first_RMC.lat_sec*100000000;
  String lat = checkingDirectionLat(first_RMC.NS_indicator);
  String startLat = String(lat) + String(startLatDeg) + "." + String(startLatSec);
  String lon = checkingDirectionLon(first_RMC.WE_indicator);
  String startLon = String(lon) + String(startLonDeg) + "." + String(startLonSec);
  Serial.write("\nStart lat: ");
  Serial.print(startLat);
  Serial.write("\nStart lon: ");
  Serial.print(startLon);
  int lastLonDeg = last_RMC.lon_deg;
  float lastLonSec = last_RMC.lon_sec*100000000;
  int lastLatDeg = last_RMC.lat_deg;
  float lastLatSec = last_RMC.lat_sec*100000000;
  String lat1 = checkingDirectionLat(last_RMC.NS_indicator);
  String lastLat = String(lat1) + String(lastLatDeg) + "." + String(lastLatSec);
  String lon1 = checkingDirectionLon(last_RMC.WE_indicator);
  String lastLon = String(lon1) + String(lastLonDeg) + "." + String(lastLonSec);
  Serial.write("\nLast lat: ");
  Serial.print(lastLat);
  Serial.write("\nLast lon: ");
  Serial.print(lastLon);
}

String checkingDirectionLat(String d)
{
  if (d.equals("N"))
  {
    return "+";
  }
  if (d.equals("S"))
  {
    return "-";
  }
  return "000";
}
String checkingDirectionLon(String d)
{
  if (d.equals("E"))
  {
    return "+";
  }
  if (d.equals("W"))
  {
    return "-";
  }
  return "000";
}

void SaveData(float speedAVG, float speedCoordinates, float courseAVG, int successPercentage)
{
  file = SD.open(fileName, FILE_WRITE);
  String time1 = timeC();
  file.print("Time of measuring: ");
  file.println(time1);
  file.print("AVG speed: ");
  file.println(speedAVG);
  file.print("AVG Course: ");
  file.println(courseAVG);
  file.print("Speed by Coordinates: ");
  file.println(speedCoordinates);
  file.print("GPS Accuracy : ");
  file.print(successPercentage);
  file.println("%");
  file.print("Start lat: ");
  file.print(checkingDirectionLat(first_RMC.NS_indicator)); 
  file.println((first_RMC.lat_deg + first_RMC.lat_sec),11);
  file.print("Start lon: ");
  file.print(checkingDirectionLon(first_RMC.WE_indicator));
  file.println((first_RMC.lon_deg + first_RMC.lon_sec),11);
  file.print("Final lat: ");
  file.print(checkingDirectionLat(last_RMC.NS_indicator));
  file.println((last_RMC.lat_deg + last_RMC.lat_sec),11);
  file.print("Final lon: ");
  file.print(checkingDirectionLon(last_RMC.WE_indicator));
  file.println((last_RMC.lon_deg + last_RMC.lon_sec),11);
  file.println(" ");
  file.close();
}

void initializeSD()
{
  Serial.println("Initializing SD card...");
  pinMode(CS_PIN, OUTPUT);

  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
}
int createFile(char filename[])
{
  file = SD.open(filename, FILE_WRITE);

  if (file)
  {
    Serial.println("File created successfully.");
    return 1;
  } else
  {
    Serial.println("Error while creating file.");
    return 0;
  }
}
String timeC()
{
  String time1;
  int ho = first_RMC.time_hh + readGMT();
  if (ho >= 24)
  {
    ho = ho - 24;
    addDay();
  }
  if (ho < 0)
  {
    int x = -readGMT() - first_RMC.time_hh;
    ho = 24 - x;
    decDay();
  }
  time1 += String(ho);
  time1 += ":";
  time1 += String(first_RMC.time_mm);
  time1 += ":";
  time1 += String(first_RMC.time_ss);
  time1 += " , ";
  time1 += String(first_RMC.date_dd);
  time1 += "/";
  time1 += String(first_RMC.date_mm);
  time1 += "/";
  time1 += String(first_RMC.date_yy);
  return time1;
}

void closeFile()
{
  if (file)
  {
    file.close();
    Serial.println("File closed");
  }
}

String readLine()
{
  String received = "";
  char ch;
  while (GMTfile.available())
  {
    ch = GMTfile.read();
    if (ch == '\n')
    {
      return String(received);
    }
    else
    {
      received += ch;
    }
  }
  return "";
}

int divided()
{
  if (first_RMC.date_yy%4==0&&first_RMC.date_yy%100!=0)
  {
    return 29;
  }
  else
  {
    return 28;
  }
}

int days()
{
  switch(first_RMC.date_mm)
  {
  case 1:
    return 31;
    break;
  case 2:
    return divided();
    break;
  case 3:
    return 31;
    break;
  case 4:
    return 30;
    break;
  case 5:
    return 31;
    break;
  case 6:
    return 30;
    break;
  case 7:
    return 31;
    break;
  case 8:
    return 31;
    break;
  case 9:
    return 30;
    break;
  case 10:
    return 31;
    break;
  case 11:
    return 30;
    break;
  case 12:
    return 31;
    break;
  default:
    return 0;
    break;        
  }
}

void addDay()
{
  int numOfDay = days();
  if (first_RMC.date_dd + 1 > numOfDay)
  {
    first_RMC.date_dd = 1;
    first_RMC.date_mm++;
    if (maxMonth())
    {
      first_RMC.date_mm = 1;
      first_RMC.date_yy++;
    }
  }
  else
  {
    first_RMC.date_dd ++; 
  }
}

void decDay()
{
  if (first_RMC.date_dd - 1 <= 0)
  {
    first_RMC.date_mm--;
    if (first_RMC.date_mm<=0)
    {
      first_RMC.date_mm = 12;
      first_RMC.date_yy--;
    }
    int numOfDay = days();  
    first_RMC.date_dd = numOfDay;
  }
  else
  {
    first_RMC.date_dd--;
  }
}
bool maxMonth()
{
 if (first_RMC.date_mm>12)
 {
  return true;
 }
 return false;
}
