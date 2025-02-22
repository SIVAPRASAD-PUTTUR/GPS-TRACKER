//------------------------------|HEARDER FILES|-----------------------------------
#include <Arduino.h>
#include <WiFi.h>
#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>



//------------------------------|PIN CONFIGURATIONS|------------------------------
#define A9G_PWR 14
#define A9G_POFF 13
#define A9G_LOWP 27
#define LORA_RST 26 //LoRa RST digital pin
#define SS 5
#define DIO0 2

//------------------------------|MQTT PARAMETERS|---------------------------------
String MQTT_BROKER = "io.adafruit.com";
String MQTT_PORT = "1883";
String MQTT_USERNAME = "Siva09";
String MQTT_PASSWORD = "aio_TYux93jVp9vkDhbJzjFsRaqyy6hC";

//------------------------------|SIM PARAMETERS|-----------------------------------
String SOS_NUM = "+919989708878";
String MY_NUM  = "+916305277531";
String APN_NAME = "WWW";// for vi

//-----------------------------|SWITCH CONFIGS|-----------------------------------
#define S1 16
#define S2 25
#define S3 33
#define S4 4
#define S5 32
#define S6 14

//-----------------------------|IMPORTANT PARAMETERS|------------------------------
#define DEBUG true
TaskHandle_t lora_srh_tsk = NULL;
String fromGSM = "";
String res = "";
const char* response;
String location_data;
String lati;
String longi;
String link1;
String link2;
String cmd;
String Gmaps_link;

bool lsch_tsk = false;
bool b1 = false;
bool b2 = false;
bool b3 = false;
bool b4 = false;
bool b5 = false;



//-----------------------------|Time|---------------------------------------------
uint16_t Send_Data_After = 30; // 60 sec waiting
int long_press_time = 2000;
int button_press_Time = 0;
//-----------------------------|TASKS|--------------------------------------------
void MQTT_RQST_Task(void *pvParameters);
void ACTIONS_Task(void *pvParameters);
void LORA_SRCH_Task(void *pvParameters);

void MSG_LORA(void);
void MSG_A9G(String num, String msg);
void MQTT(void);
String sendData(String command, const int timeout, boolean debug);


void setup()
{

  Serial.begin(115200); // For ESP32
  Serial1.begin(115200, SERIAL_8N1, 9, 10); //For A9G
  WiFi.mode(WIFI_OFF);  // WiFi OFF
  btStop();   // Bluetooth OFF

  pinMode(A9G_PWR, OUTPUT);//LOW LEVEL ACTIVE
  pinMode(A9G_POFF, OUTPUT);//HIGH LEVEL ACTIVE
  pinMode(A9G_LOWP, OUTPUT);//LOW LEVEL ACTIVE

  digitalWrite(A9G_POFF, LOW);
  digitalWrite(A9G_LOWP, HIGH);
  digitalWrite(A9G_PWR, HIGH);
  delay(1000);
  digitalWrite(A9G_PWR, LOW);

  LoRa.setPins(SS, LORA_RST, DIO0);
  LoRa.begin(433E6);
  LoRa.setSyncWord(0x34);

  //RESETTING A9G
  cmd = "";
  cmd = sendData("AT+RST=1", 2000, DEBUG);
  while (  cmd.indexOf("OK") == -1 ) {
    cmd = sendData("AT+RST = 1", 2000, DEBUG);
    Serial.println("Trying");
  }

  //DELAY
  Serial.println("Waiting...");
  delay(6000);
  Serial.println("completed!");

  //AT COMMAND CHECKING
  cmd = "";
  cmd = sendData("AT", 1000, DEBUG);
  while (  cmd.indexOf("OK") == -1 ) {
    cmd = sendData("AT", 1000, DEBUG);
    Serial.println("Trying");
  }

  //DISCONNECTING THE A9G FROM PREV MQTT
  cmd = "";
  cmd = sendData("AT+MQTTDISCONN", 1000, DEBUG);
  while (  cmd.indexOf("OK") == -1 ) {
    cmd = sendData("AT+MQTTDISCONN", 1000, DEBUG);
    Serial.println("Trying");
  }

  //STARTING GPS
  cmd = "";
  cmd = sendData("AT+GPS=1", 2000, DEBUG);
  while ( cmd.indexOf("OK") == -1 ) {
    cmd = sendData("AT+GPS=1", 1000, DEBUG);
    Serial.println("Trying");
  }

  // GPS low power
  cmd = "";
  cmd = sendData("AT+GPSLP = 2", 2000, DEBUG);
  while ( cmd.indexOf("OK") == -1 ) {
    cmd = sendData("AT+GPSLP = 2", 1000, DEBUG);
    Serial.println("Trying");
  }

  //ATTACH GSM SERVICE
  cmd = "";
  cmd = sendData("AT+CGATT=1", 2000, DEBUG);
  while ( cmd.indexOf("OK") == -1 ) {
    cmd = sendData("AT+CGATT=1", 1000, DEBUG);
    Serial.println("Trying");
  }
  //STABLISH CONN
  cmd = "";
  cmd = sendData("AT+CGDCONT=1,\"IP\",\"" + APN_NAME + "\"", 2000, DEBUG);
  while ( cmd.indexOf("OK") == -1 ) {
    cmd = sendData("AT+CGDCONT=1,\"IP\",\"" + APN_NAME + "\"", 1000, DEBUG);
    Serial.println("Trying");
  }

  cmd = "";
  cmd = sendData("AT+CGACT=1", 2000, DEBUG);
  while ( cmd.indexOf("OK") == -1 ) {
    cmd = sendData("AT+CGACT=1", 1000, DEBUG);
    Serial.println("Trying");
  }

  cmd = "";
  cmd = sendData("AT+MQTTCONN=\"" + MQTT_BROKER + "\"," + MQTT_PORT + ",\"ABCD\",120,0,\"" + MQTT_USERNAME + "\",\"" + MQTT_PASSWORD + "\"", 3000, DEBUG);
  while ( cmd.indexOf("OK") == -1 ) {
    cmd = sendData("AT+MQTTCONN=\"" + MQTT_BROKER + "\"," + MQTT_PORT + ",\"ABCD\",120,0,\"" + MQTT_USERNAME + "\",\"" + MQTT_PASSWORD + "\"", 1000, DEBUG);
    Serial.println("Trying");
  }



  xTaskCreatePinnedToCore(
    ACTIONS_Task,      // Task function
    "ACTIONSTask",    // Task name
    2000,           // Stack size
    NULL,           // Parameters
    1,              // Priority
    NULL,           // Task handle
    1               // Core ID
  );

  
}

void loop() {

}


void ACTIONS_Task(void *pvParameters){
  while(1){
    if(!b1){
      if(digitalRead(S1)==0){
        button_press_Time = millis();
        b1 = true;
        MQTT();
        MSG_A9G(SOS_NUM,Gmaps_link);

        //print on oled that msg is sent
      }
      else if(millis() - button_press_Time >= long_press_time){

        //print on oled that operation is cancelled
        
      }
      else{
        b1 = false;
      }
    }

    if(!b2){
      if(digitalRead(S2)==0){
        button_press_Time = millis();
        b2 = true;
        MQTT();
        MSG_LORA();
        //print on oled that msg is sent
      }
      else if(millis() - button_press_Time >= long_press_time){

        //print on oled that operation is cancelled
        
      }
      else{
        b2 = false;
      }
    }

    if(!b3){
      if(digitalRead(S3)==0){
        button_press_Time = millis();
        b3 = true;

        MQTT();
        MSG_LORA();
        MSG_A9G(SOS_NUM,Gmaps_link);
        
        //print on oled that msg is sent
      }
      else if(millis() - button_press_Time >= long_press_time){

        //print on oled that operation is cancelled
        
      }
      else{
        b3 = false;
      }
    }

    if(!b4){
      if(digitalRead(S4)==0){
        button_press_Time = millis();
        b4 = true;
        MSG_A9G(MY_NUM,link2);
        //print on oled that msg is sent
      }
      else if(millis() - button_press_Time >= long_press_time){

        //print on oled that operation is cancelled
        
      }
      else{
        b4 = false;
      }
    }

    if(!b5){
      if(digitalRead(S5)==0){
        button_press_Time = millis();
        b5 = true;
        MSG_LORA();
        //print on oled that msg is sent
      }
      else if(millis() - button_press_Time >= long_press_time){

        //print on oled that operation is cancelled
        
      }
      else{
        b5 = false;
      }
    }


    if(!lsch_tsk){  
      if(digitalRead(S6)==0){
        button_press_Time = millis();
        lsch_tsk = true;
        xTaskCreatePinnedToCore(
          LORA_SRCH_Task,      // Task function
          "LORA_Task",    // Task name
          3000,           // Stack size
          NULL,           // Parameters
          0,              // Priority
          &lora_srh_tsk,  // Task handle
          0               // Core ID
        );
      }
      else if(millis() - button_press_Time >= long_press_time ){
        if (lora_srh_tsk != NULL) {
          vTaskDelete(lora_srh_tsk);
          lora_srh_tsk = NULL;
        }
        lsch_tsk=false;
      }
      else{
        lsch_tsk=false;
      }
    }
  }
}
//-------------------------------------- LORA search task for other device alerts ------------------
void LORA_SRCH_Task(void *pvParameters){                                //print in the oled screen when alert received 
  int packetSize = LoRa.parsePacket();                                  //
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData); 
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}

//---------------------------------- To initiallise the at commands ------------------------ 
String sendData(String command, const int timeout, boolean debug)
{
  String temp = "";
  Serial1.println(command);
  long int time = millis();
  while ( (time + timeout) > millis())
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
      temp += c;
    }
  }
  if (debug)
  {
    Serial.print(temp);
  }
  return temp;
}
//------------------- Sends the coordinates / Alert from LORA --------------------------
void MSG_LORA(void){                                          //sending the coordinates link to other device 
  while(1){
  LoRa.beginPacket();
  LoRa.print("HELP...");
  LoRa.print(Gmaps_link);
  LoRa.endPacket();
  delay(10000);
  }
}
//------------------- Sends the coordinates (gmaps link) to mobile phone from A9G -----------------------
void MSG_A9G(String num, String msg){
  Serial1.println("AT+CMGS=\"" + num + "\"\r");
  delay(1000);
  delay(1000);
  Serial1.println("AT+CMGF=1");

  Serial1.println ("I'm here " + msg);
  delay(1000);
  Serial1.println((char)26);
  delay(1000);

  Serial1.println("AT+CMGD=1,4"); // delete stored SMS to save memory
  delay(5000);
}
void MQTT(){
  xTaskCreatePinnedToCore(
      MQTT_RQST_Task,      // Task function
      "MQTT_Task",    // Task name
      1000,           // Stack size
      NULL,           // Parameters
      2,              // Priority
      NULL,           // Task handle
      1               // Core ID
  );
}


void MQTT_RQST_Task(void *pvParameters){

   for (;;) // A Task shall never return or exit.
  {
    Serial1.println("AT+LOCATION=2\r\n");
    delay(2000);
    while (!Serial1.available())
    {
      Serial.println("Waiting");
      delay(10);
    }
    while (Serial1.available())
    {
      char add = Serial1.read();
      res = res + add;
      delay(1);
    }
    response = &res[0];

    if (strstr(response, "GPS NOT"))
    {
      Serial.println("No Location data");
    }
    else
    {

      int j = 0;
      while (res[j] != '2')
        j++;

      res = res.substring(j + 5);
      int k = 0;
      while (res[k] != '\n')
        k++;
      res = res.substring(0, k);
      response = &res[0];

      Serial.print("Current String -"); Serial.print(response); Serial.println("-");
      int i = 0;
      while (response[i] != ',')
        i++;

      location_data = (String)response;
      lati = location_data.substring(0, i);
      longi = location_data.substring(i + 1, i + 10);
      Serial.print("Lat - "); Serial.print(lati); Serial.println("-");
      Serial.print("Longi - "); Serial.print(longi); Serial.println("-");
      int longi_length = longi.length();
      Serial.print("Longi Length - "); Serial.print(longi_length); Serial.println("-");
      //lati.trim();

      Gmaps_link = ( "http://maps.google.com/maps?q=" + lati + "+" + longi);

      String coordinates = "0," + lati + "," + longi.c_str() + ",0";
      Serial.print("coordinates - "); Serial.print(coordinates); Serial.println("-");

      link1 = ("AT+MQTTPUB=\"Siva09/feeds/tracker.location\",\"" + coordinates + "\",0,0,0") ;
      Serial.print("link lat -"); Serial.println(link1);

      //Serial.print("For Serial Monitor-"); Serial.println(link1);

      // Serial1.println(link1);
      sendData(link1, 1000, DEBUG);

      delay(2000);

      Serial.println("Location DataSend");

    }

    cmd = "";
    cmd = sendData("AT+CBC?", 2000, DEBUG);
    while ( cmd.indexOf("OK") == -1 ) {
      cmd = sendData("AT+CBC?", 1000, DEBUG);
      Serial.println("Trying");
    }
    Serial.print("Recevied Data Before - "); Serial.println(cmd); // printin the String in lower character form
    int count = 0;
    while (cmd[count] != ',')
    {
      count++;
      Serial.print(cmd[count]);

    }

    cmd = cmd.substring(count + 2);

    count = 0;
    while (cmd[count] != '\n')
    {
      count++;
      Serial.print(cmd[count]);

    }

    cmd = cmd.substring(0, count - 1);

    Serial.print("Recevied Data - "); Serial.println(cmd); // printin the String in lower character form
    Serial.println("\n");

    link2 = ("AT+MQTTPUB=\"Siva09/feeds/tracker.battery\",\"" + cmd + "\",0,0,0") ;
    Serial.print("battery link -"); Serial.println(link2);
    Serial.print("For Serial Monitor-"); Serial.println(link2);

    // Serial1.println(link1);
    cmd = "";
    cmd =  (sendData(link2, 1000, DEBUG));
    char* cmd_char = &cmd[0];
    Serial.print("LAT cmd - "); Serial.println(cmd_char);
    if ( !(strstr(cmd_char, "OK" ))) {
     // MQTT_ReConnect();
    }
    delay(2000);

    Serial.println("Battery DataSend");

    response = "";
    res = "";


    Serial.println("Delay");

    vTaskDelay((Send_Data_After * 1000));
  }
}

