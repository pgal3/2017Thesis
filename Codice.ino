//===================================== Libraries
extern "C" {
  #include <user_interface.h>
  }
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>    //For the BME280 chip
#include <Adafruit_BME280.h>    //For the BME280 chip
#include <pgmspace.h>          
#include "./defmacORD.h"        //List of the IEEE registered MAC addresses

//===================================== Declarations

#define DEVICES_MAX             500            //Maximum number of captured devices
#define DATA_LENGTH             112            //See Espressif spec file
#define TYPE_MANAGEMENT         0x00           //For management type of frame
#define SUBTYPE_PROBE_REQUEST   0x04           //Probe request sub-type
#define SUBTYPE_PROBE_RESP      0x05           //Probe response sub-type
#define FINESSE                  40            //Number of power intervals/power sensitivity
#define HIST                     40            //Maximum number of input considered/ history
#define ssid                    "G2"           //ssid 
#define password             "lgg2wifi"        //password of WiFi
#define mqtt_server        "54.93.65.245"      //IP of the MQTT server
#define mqtt_port               1883           //MQTT port
#define username               "tesi"          //username for MQTT
#define pwd                    "1802"          //MQTT password 
#define SEALEVELPRESSURE      1013.25          //sea level pressure in hPa

//===================================== MQTT TOPICS
//the number of the room is changed based on the room you're into. (eg. room1, room2 o room3)

#define txtopic1           "room1/temperature"
#define txtopic2           "room1/pressure"
#define txtopic3           "room1/humidity"
#define txtopic4           "room1/altitude"
#define txtopic5           "room1/light"
#define txtopic6           "room1/nreal"
#define txtopic7           "room1/nstima"
#define txtopic8           "room1/npeople"
#define txtopic9           "room1/nrandom"
#define txtopic10          "room1/pwr"
#define txtopic11          "room1/alfa"
#define txtopic12          "room1/beta"
#define txtopic13          "room1/iterazione"
#define txtopic14          "room1/statistics/delta"
#define txtopic15          "room1/statistics/deltarndm"
#define rxtopic1           "room1/mqtt_freq"
#define rxtopic2           "room1/statistics/nreal"
#define rxtopic3           "room1/setPwr"

//===================================== Pin Declaration
#define ledpin               BUILTIN_LED       //Built-in led pin
#define button                  D4             //button pin
#define BME_SCK                 D1             //pin sck (i2c)
#define BME_SDI                 D2             //pin sdi (i2c)
#define photoRpin               A0             //photoresistor pin

//===================================== Variables declaration
Adafruit_BME280 bme;
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long CurrentMills=0;       
unsigned long PrevMills=0;          
unsigned long PrevChange=0;         
unsigned long mqtt_freq=300000;                      //Data sending frequency
unsigned long nciclo=0;                              //Cicle number (or number of time the data was sent)
unsigned int nstima=0;                               //Estimated number of persons
unsigned int delta[FINESSE]={0};                     //Number of MACs captured (related to the power)
unsigned int deltaRndm[FINESSE]={0};                 //Number of random MACs (related to the power)
unsigned int deltaSaved[HIST][FINESSE]={0};          //Past number of devices/MACs related to power
unsigned int deltaRndmSaved[HIST][FINESSE]={0};      //Past number of random MACs related to power
unsigned int i=0;                                    //index
unsigned int occupancy=0;                            //First free slot on the MAC array.
unsigned int nreal[HIST]={0};                        //ground truth (function of time)
int pwrange[]={-44,-48,-52,-54,-56,-58,-60,-62,-64,-66,-68,-70,-72,-73,-74,-75,-76,-77,-78,-79,-80,-81,-82,-83,-84,-85,-86,-87,-88,-89,-90,-91,-92,-93,-94,-95,-96,-97,-98,-120};
char topicbuf[30];                                   //buffer for the topic's name
char addr[] = "000000000000";                        //13 characters for the MAC address (12+il null)              
byte flag=0;
byte event=0;
byte indice=FINESSE-1;                               
byte buttonstatus=0;                                 //Button state
byte lastButtonStatus=0;                             //Latest button state
byte SetPower=0;                                     //Flag for the validity of the NReal number received. if =0 is valid.
float Alfa=1;                                        //Startup value of alpha
float Beta=0.001;                                    //DO NOT use 0, otherwise --> ERROR 0*0

//===================================== Structures from the ESPRESSIF resources
struct RxControl {
 signed rssi:8; // signal intensity of packet
 unsigned rate:4;
 unsigned is_group:1;
 unsigned:1;
 unsigned sig_mode:2; // 0:is 11n packet; 1:is not 11n packet;
 unsigned legacy_length:12; // if not 11n packet, shows length of packet.
 unsigned damatch0:1;
 unsigned damatch1:1;
 unsigned bssidmatch0:1;
 unsigned bssidmatch1:1;
 unsigned MCS:7; // if is 11n packet, shows the modulation and code used (range from 0 to 76)
 unsigned CWB:1; // if is 11n packet, shows if is HT40 packet or not
 unsigned HT_length:16;// if is 11n packet, shows length of packet.
 unsigned Smoothing:1;
 unsigned Not_Sounding:1;
 unsigned:1;
 unsigned Aggregation:1;
 unsigned STBC:2;
 unsigned FEC_CODING:1; // if is 11n packet, shows if is LDPC packet or not.
 unsigned SGI:1;
 unsigned rxend_state:8;
 unsigned ampdu_cnt:8;
 unsigned channel:4; //which channel this packet in.
 unsigned:12;
};

struct SnifferPacket{
    struct RxControl rx_ctrl;
    uint8_t data[DATA_LENGTH];
    uint16_t cnt;
    uint16_t len;
};

//===================================== Structure for data saving

struct Savings{
       char mac[13]={0};         // MAC address
       unsigned long Time=0;     // last time seen
	     int RSSI=0;         // received power
       boolean rndm=0;           // =0 if MAC random, =1 otherwise
}Saved[DEVICES_MAX];

//===================================== Function to get the MAC address

void getMAC(char *addr, uint8_t* data, uint16_t offset) {
  sprintf(addr, "%02x%02x%02x%02x%02x%02x", data[offset+0], data[offset+1], data[offset+2], data[offset+3], data[offset+4], data[offset+5]);
}

//===================================== Binary search for the MAC address in IEEE list
boolean ricerca(int inizio, int fine){                
  int media;
  if(inizio>fine) return 0; //not found
  else{
    media=(inizio+fine)/2;
    if(strncmp_P(addr,(char*)pgm_read_dword(&(mac_table[media])),6)==0) return 1; //EUREKA!
    else
       if(strncmp_P(addr,(char*)pgm_read_dword(&(mac_table[media])),6)>0)
               ricerca(media+1,fine);
       else ricerca(inizio,media-1);
  }
}

//===================================== Sorting the table to leave no free rows
void sort(unsigned int j){
   while(j<occupancy){
         Saved[j]=Saved[j+1];
         j++;
  }
  occupancy--;
  }

//===================================== Filtering and saving of probe requests

void showMetadata(SnifferPacket *snifferPacket) {
    
  unsigned int frameControl = ((unsigned int)snifferPacket->data[1] << 8) + snifferPacket->data[0];
  uint8_t frameType    = (frameControl & 0b0000000000001100) >> 2;
  uint8_t frameSubType = (frameControl & 0b0000000011110000) >> 4;
  
//===================================== we take only the probe requests
  
  if (frameType == TYPE_MANAGEMENT && frameSubType == SUBTYPE_PROBE_REQUEST) 
     {
      getMAC(addr, snifferPacket->data, 10);                 //getting the mac address
      flag=0;       
      for(i=0;i<occupancy;i++){                              //Check if I've already seen that MAC address
         if(strcmp(addr,Saved[i].mac)==0)
                      {
                       if(Saved[i].RSSI != snifferPacket->rx_ctrl.rssi){ //if the received power is different --> update it
					               for(int j=FINESSE-1;j>=0;j--){   //removes, from the total count based on the power, the old value
                           if(Saved[i].RSSI>=pwrange[j]){                
                                                 switch(Saved[i].rndm){  
                                                      case 0: delta[j]--; break;
                                                      case 1: deltaRndm[j]--; break; 
                                                 }                        
                           }
                           else break;
                           } 
					             Saved[i].RSSI=snifferPacket->rx_ctrl.rssi;		     //saving the new power value		             
                       for(int j=0;j<FINESSE;j++){                       //updates the device*power count.
                       if(Saved[i].RSSI>=pwrange[j] && Saved[i].rndm==0) delta[j]++;
                       if(Saved[i].RSSI>=pwrange[j] && Saved[i].rndm==1) deltaRndm[j]++;
                       }
                       }
                       Saved[i].Time=millis();                           //saves the time in which the probe has arrived
                       flag=1;                   
                       break;
                       }
                       }
                       
        if(flag==0){                                                     //if the MAC address is new (never seen before)
        strcpy(Saved[occupancy].mac,addr);
        Saved[occupancy].Time=millis();
		    Saved[occupancy].RSSI=snifferPacket->rx_ctrl.rssi;
        Saved[occupancy].rndm =!ricerca(0,lungh-1);
        
        if(Saved[occupancy].rndm==1){
          for(int j=FINESSE-1;j>=0;j--){
           if(Saved[occupancy].RSSI>=pwrange[j]) deltaRndm[j]++;
           if(Saved[occupancy].RSSI<pwrange[j]) break;
              }
          Serial.println();
          Serial.print("PROBE REQ. FROM RANDOM MAC: ");
          Serial.print(Saved[occupancy].mac);
          Serial.print(" RSSI = ");
          Serial.print(Saved[occupancy].RSSI);
          Serial.print(" ==> N°Dev RANDOM: ");
          Serial.print(deltaRndm[39]);         
        }
        if(Saved[occupancy].rndm==0) {
          
          for(int j=FINESSE-1;j>=0;j--){
           if(Saved[occupancy].RSSI>=pwrange[j])delta[j]++;
           if(Saved[occupancy].RSSI<pwrange[j]) break;
              }
               
        Serial.println();
        Serial.print("PROBE REQ: ");
        Serial.print("RSSI = ");
        Serial.print(Saved[occupancy].RSSI);
        Serial.print("dBm ==> MAC: ");
        Serial.print(Saved[occupancy].mac);
		    Serial.print(" ==> N°Dev: ");
        Serial.print(delta[39]);
        }
        occupancy++;
        }
     
     }

}


// ========================== Callback function
 
void sniffer_callback(uint8_t *buffer, uint16_t length) {
  struct SnifferPacket *snifferPacket = (struct SnifferPacket*) buffer;
  if(occupancy<DEVICES_MAX) showMetadata(snifferPacket);  
}

//======================================================================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println();
  Serial.print("Subscribed to topic ");
  Serial.print(topic);
  Serial.print(": ");
  if(strcmp(rxtopic1,topic)==0) mqtt_freq=atoi((char*)payload)*60000;
  if(strcmp(rxtopic2,topic)==0) nreal[event]=atoi((char*)payload);
  if(strcmp(rxtopic3,topic)==0) SetPower=atoi((char*)payload);
  PrevChange=millis();
}

boolean mqttconnect() {
  // Loop until we're reconnected
  i=0;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
       if (client.connect("ESP8266Client",username,pwd)){Serial.println("connected"); return 1;}
     else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      if(i>1) return 0;
      Serial.println(" try again in 5 seconds");
      delay(3000);
      }
    i++;
  }
}


//=================================================================
//============================= Wifi connection
boolean setup_wifi() {            
  delay(10);  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);  
  WiFi.begin(ssid, password);
  i=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  i++;
  if(i==10){
    Serial.println("WI-FI Connection Error");
    return 0;                     // it takes to long to connect
  }
  }
  Serial.println("");
  Serial.println("WiFi connected");  
  return 1;                                 //connection succeded
}

//============================= connection to the MQTT server and messages rx/tx
boolean mqtt_rx(){
  
  client.subscribe(rxtopic1);
  while(millis()-PrevChange>400){
  if(!client.loop()) return 0;
  }
  Serial.println(mqtt_freq);
  delay(500);
  client.unsubscribe(rxtopic1);
  
  client.subscribe(rxtopic2);
  while(millis()-PrevChange>400){
  if(!client.loop()) return 0;
  }
  Serial.println(nreal[event]);
  delay(500);
  client.unsubscribe(rxtopic2);
  
  client.subscribe(rxtopic3);
  while(millis()-PrevChange>400){
  if(!client.loop()) return 0;
  }
  Serial.println(SetPower);
  delay(500);
  client.unsubscribe(rxtopic3);
 
  return 1;
}

boolean mqtt_tx_basic(){
  char buf[10];
   dtostrf(bme.readTemperature(),0, 2, buf);
   i=0;
   while(!client.publish(txtopic1, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
      
   dtostrf(bme.readPressure()/100.0F,0, 2, buf);
   i=0;
   while(!client.publish(txtopic2, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
   
   dtostrf(bme.readHumidity(),0, 2, buf);
   i=0;
   while(!client.publish(txtopic3, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
   
   dtostrf(bme.readAltitude(SEALEVELPRESSURE),0, 2, buf);  
   i=0;
   while(!client.publish(txtopic4,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }

   dtostrf(analogRead(photoRpin),0,0,buf);
   i=0;
   while(!client.publish(txtopic5,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }

   dtostrf(nstima,0, 0, buf);
   i=0;
   while(!client.publish(txtopic7,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
      
   dtostrf(delta[39],0, 0, buf);
   i=0;
   while(!client.publish(txtopic8, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
   
   
   dtostrf(deltaRndm[39],0, 0, buf);
   i=0;
   while(!client.publish(txtopic9, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }

   dtostrf(pwrange[indice],0, 0, buf);
   i=0;
   while(!client.publish(txtopic10,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }

   dtostrf(Alfa,0,2,buf);
   i=0;
   while(!client.publish(txtopic11,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
      
   dtostrf(Beta,0,2,buf);
   i=0;
   while(!client.publish(txtopic12,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
      
   dtostrf(nciclo,0, 0, buf);
   i=0;
   while(!client.publish(txtopic13, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
   
   return 1;
  }

boolean mqtt_tx_complete(){
   char buf[10];
   dtostrf(bme.readTemperature(),0, 2, buf);
   i=0;
   while(!client.publish(txtopic1, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
   
   dtostrf(bme.readPressure()/100.0F,0, 2, buf);
   i=0;
   while(!client.publish(txtopic2, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
   
   dtostrf(bme.readHumidity(),0, 2, buf);
   i=0;
   while(!client.publish(txtopic3, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
   
   dtostrf(bme.readAltitude(SEALEVELPRESSURE),0, 2, buf); 
   i=0; 
   while(!client.publish(txtopic4,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }

   dtostrf(analogRead(photoRpin),0,0,buf);
   i=0;
   while(!client.publish(txtopic5,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }

   dtostrf(nstima,0, 0, buf);
   i=0;
   while(!client.publish(txtopic7,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
      
   dtostrf(delta[39],0, 0, buf);
   i=0;
   while(!client.publish(txtopic8, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }   
   
   dtostrf(deltaRndm[39],0, 0, buf);
   i=0;
   while(!client.publish(txtopic9, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }

   dtostrf(pwrange[indice],0, 0, buf);
   i=0;
   while(!client.publish(txtopic10,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }

   dtostrf(Alfa,0,2,buf);
   i=0;
   while(!client.publish(txtopic11,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
      
   dtostrf(Beta,0,2,buf);
   i=0;
   while(!client.publish(txtopic12,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
      
   dtostrf(nciclo,0, 0, buf);
   i=0;
   while(!client.publish(txtopic13, buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
   
   dtostrf(nreal[event],0, 0, buf);
   i=0;
   while(!client.publish(txtopic6,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
    
   for(int j=0;j<FINESSE;j++){    
    dtostrf(delta[j],0, 0, buf);
    i=0;
    while(!client.publish(txtopic14,buf)){
    client.loop();
    i++;
    if(i>2) return 0;
    delay(100);
    }
    dtostrf(deltaRndm[j],0, 0, buf);
    i=0;
    while(!client.publish(txtopic15,buf)){
      client.loop();
      i++;
      if(i>2) return 0;
      delay(100);
    }
   }
        
  if(event==HIST-1){
     for(int k=0;k<event;k++){
      nreal[k]=nreal[k+1];
      for(int j=0;j<FINESSE;j++){
        deltaSaved[k][j]=deltaSaved[k+1][j];
        deltaRndmSaved[k][j]=deltaRndmSaved[k+1][j];
      }
    }   
   }
   
   if(event<HIST-1) event++;   
   return 1;
 }


unsigned int arrotondo(float var){
  if((var- ((int) var))*10>5) return ceil(var);
  else return floor(var);
}
//============================= Power Filter
unsigned int powerfilter(){
    
    float minimum=2000;
    float error;
    float temp=0;
    float alfatemp=0;
    float betatemp=0;
       
    for(alfatemp=0.1;alfatemp<2;alfatemp=alfatemp+0.1){
      for(betatemp=0.001;betatemp<alfatemp;betatemp=betatemp+0.1){
         for(int j=FINESSE-1;j>=0;j--){
         error=0;    
            for(int k=0;k<=event;k++){
               temp=nreal[k]-alfatemp*deltaSaved[k][j];
               temp=temp-betatemp*deltaRndmSaved[k][j];
               error=error+pow(temp,2);   //Calculate the error at power j and step k
                                     }
         temp=event+1;
         error=error/temp;
         error=sqrt(error);
                          
         if(error<minimum){
          
           minimum=error;     
           indice=j;
           Alfa=alfatemp;
           Beta=betatemp;
                         }
           }
       ESP.wdtFeed();
      }
    }
 Serial.println();
 Serial.print("Power Limit = ");
 Serial.println(pwrange[indice]);  
 Serial.print("Minimum Error = ");
 Serial.println(minimum);
 temp=Alfa*delta[indice]+Beta*deltaRndm[indice];      
 return arrotondo(temp);
}


//====================================== SETUP E LOOP

void setup() {
  Serial.begin(115200);
  bme.begin();
  delay(200);
  pinMode(ledpin,OUTPUT);
  pinMode(button,INPUT);
  pinMode(photoRpin,INPUT);
  Serial.println();
  Serial.print("MQTT frequency: ");
  Serial.println(mqtt_freq);
  digitalWrite(ledpin,HIGH);
  delay(10);
  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(2);
  wifi_promiscuous_enable(0);
  delay(10);
  wifi_set_promiscuous_rx_cb(sniffer_callback);
  delay(10);
  wifi_promiscuous_enable(1);
}

void loop() {

//================ Removing of inactive address (if inactive for more than 7 minutes)

for(i=0;i<occupancy;i++){
        CurrentMills=millis();
        if(CurrentMills-Saved[i].Time>420000)
        {	      
                for(int j=FINESSE-1;j>=0;j--){
                           if(Saved[i].RSSI>=pwrange[j] && Saved[i].rndm==0) delta[j]--;
                           if(Saved[i].RSSI>=pwrange[j] && Saved[i].rndm==1) deltaRndm[j]--;
                           }                            
        Serial.println();
        Serial.print("The device with MAC ");
        Serial.print(Saved[i].mac);
        Serial.print(" was removed. ");
        sort(i);        
        }
        }


//================ MQTT Transmission

 buttonstatus=digitalRead(button);
 CurrentMills=millis();
 if(CurrentMills-PrevMills>mqtt_freq || buttonstatus!=lastButtonStatus){
    
  digitalWrite(ledpin,LOW);
  wifi_promiscuous_enable(0);
  // Wifi connect
  if(!setup_wifi()){
    buttonstatus=digitalRead(button);
    lastButtonStatus=buttonstatus;
    PrevMills=millis(); 
    digitalWrite(ledpin,HIGH); 
    wifi_promiscuous_enable(1);
    delay(10);
    return;
  }
  //Mqtt connect
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  if(!mqttconnect()){
    buttonstatus=digitalRead(button);
    lastButtonStatus=buttonstatus;
    PrevMills=millis(); 
    digitalWrite(ledpin,HIGH); 
    wifi_promiscuous_enable(1);
    delay(10);
    return;
  }
  
  if(!mqtt_rx()){  //checks if the mqtt data were correctly downloaded/seen. If not, it exits from this loop.
    buttonstatus=digitalRead(button);
    lastButtonStatus=buttonstatus;
    PrevMills=millis(); 
    digitalWrite(ledpin,HIGH);
    wifi_promiscuous_enable(1);
    delay(10); 
    return;
  }
    
  nciclo++;  
  if(SetPower==0){  //in this case the model adjust its parameters (alpha, beta, power) based on the new input
    for(int j=0;j<FINESSE;j++){
      deltaSaved[event][j]=delta[j];
      deltaRndmSaved[event][j]=deltaRndm[j];
      }
    nstima=powerfilter();
    mqtt_tx_complete();  
    }
 
  if(SetPower==1){
    Serial.print("Power Limit: ");
    Serial.println(pwrange[indice]);
    nstima=arrotondo(Alfa*delta[indice]+Beta*deltaRndm[indice]);
    mqtt_tx_basic();
    }

  buttonstatus=digitalRead(button);
  lastButtonStatus=buttonstatus;
  PrevMills=millis(); 
  digitalWrite(ledpin,HIGH);
  wifi_promiscuous_enable(1);
  delay(10);
  }
  
}
