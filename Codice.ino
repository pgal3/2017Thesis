//===================================== LIBRERIE USATE
extern "C" {
  #include <user_interface.h>
  }
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>    //Per il BME280
#include <Adafruit_BME280.h>    //Per il BME280
#include <pgmspace.h>          
#include "./defmacORD.h"        //Lista .h dei MAC registrati dall'IEEE

//===================================== DICHIARAZIONI

#define DEVICES_MAX             400            //numero massimo di dispositivi catturati
#define DATA_LENGTH             112            //vedi documento Espressif
#define TYPE_MANAGEMENT         0x00           //tipo di frame: management
#define SUBTYPE_PROBE_REQUEST   0x04           //sottotipo probe request
#define SUBTYPE_PROBE_RESP      0x05           //sottotipo probe response
#define FINESSE                  40            //numero intervalli di potenza considerati
#define HIST                     40            //numero eventi passati memorizzati e considerati
#define ssid                    "G2"           //ssid per il collegamento wifi
#define password             "lgg2wifi"        //password della rete wifi
#define mqtt_server        "54.93.65.245"      //IP del server mqtt (del server aws)
#define mqtt_port               1883           //porta del collegamento mqtt
#define username               "tesi"          //username del collegamento mqtt
#define pwd                    "1802"          //password del collegamento mqtt
#define SEALEVELPRESSURE      1013.25          //pressione al livello del mare in hPa

//===================================== MQTT TOPICS
//il numero della stanza è da cambiare in base alla stanza considerata in room1, room2 o room3

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

//===================================== DICHIARAZIONE DEI PIN USATI

#define ledpin               BUILTIN_LED       //pin del led (in questo caso quello interno)
#define button                  D4             //pin del pulsante 
#define BME_SCK                 D1             //pin sck (i2c)
#define BME_SDI                 D2             //pin sdi (i2c)
#define photoRpin               A0             //pin del fotoresistore

//===================================== DICHIARAZIONE VARIABILI
Adafruit_BME280 bme;
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long CurrentMills=0;       
unsigned long PrevMills=0;          
unsigned long PrevChange=0;         
unsigned long mqtt_freq=300000;                      //frequenza di invio dei dati, inizialmente impostata a 5 minuti
unsigned long nciclo=0;                              //numero di volte che ho inviato il dato
unsigned int nstima=0;                               //numero persone stimato
unsigned int delta[FINESSE]={0};                     //numero di dispositivi attuale per slot di potenza 
unsigned int deltaRndm[FINESSE]={0};                 //numero di dispositivi random attuale per slot di potenza
unsigned int deltaSaved[HIST][FINESSE]={0};          //record del numero di dispositivi passati per slot di potenza
unsigned int deltaRndmSaved[HIST][FINESSE]={0};      //record del numero di dispositivi random passati per slot di potenza
unsigned int i=0;                                    
unsigned int occupancy=0;                            //prima posizione libera nella tabella di MAC salvati
unsigned int nreal[HIST]={0};                        //ground truth in funzione del tempo
int pwrange[]={-44,-48,-52,-54,-56,-58,-60,-62,-64,-66,-68,-70,-72,-73,-74,-75,-76,-77,-78,-79,-80,-81,-82,-83,-84,-85,-86,-87,-88,-89,-90,-91,-92,-93,-94,-95,-96,-97,-98,-120};
char topicbuf[30];                                   //buffer interente al nome del topic
char addr[] = "000000000000";                        //sono 13 caratteri (12+il null)              
byte flag=0;
byte event=0;
byte indice=FINESSE-1;                               
byte buttonstatus=0;                                 //stato del pulsante
byte lastButtonStatus=0;                             //ultimo stato del pulsante registrato
byte SetPower=0;                                     //flag che indica la validità o no dell'Nreal letto nell'mqtt
float Alfa=1;                                        
float Beta=0.001;                                    //non posso usare 0 perchè in caso di 0*0 va tutto in crash


//===================================== STRUTTURE DAL DOCUMENTO ESPRESSIF


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

//===================================== STRUTTURA DI SALVATAGGIO DATI

struct Savings{
       char mac[13]={0};         // indirizzo MAC
       unsigned long Time=0;     // ultima volta che è stato visto
	     int RSSI=0;               // potenza ricevuta
       boolean rndm=0;           //flag per dire se è random o no
}Saved[DEVICES_MAX];

//===================================== STRUTTURA PER SALVATAGGIO DEL MAC E SSID

void getMAC(char *addr, uint8_t* data, uint16_t offset) {
  sprintf(addr, "%02x%02x%02x%02x%02x%02x", data[offset+0], data[offset+1], data[offset+2], data[offset+3], data[offset+4], data[offset+5]);
}

//===================================== FUNZIONE DI RICERCA DEL MAC NELLA LISTA DELL'IEEE
boolean ricerca(int inizio, int fine){                
  int media;
  if(inizio>fine) return 0; //non trovato
  else{
    media=(inizio+fine)/2;
    if(strncmp_P(addr,(char*)pgm_read_dword(&(mac_table[media])),6)==0) return 1; //trovato!
    else
       if(strncmp_P(addr,(char*)pgm_read_dword(&(mac_table[media])),6)>0)
               ricerca(media+1,fine);
       else ricerca(inizio,media-1);
  }
}

//===================================== FUNZIONE PER RIORDINARE LA TABELLA SENZA LASCIARE RIGHE VUOTE
void sort(unsigned int j){
   while(j<occupancy){
         Saved[j]=Saved[j+1];
         j++;
  }
  occupancy--;
  }

//===================================== FUNZIONE PRINCIPALE DI FILTRAGGIO E SALVATAGGIO DEI PROBE

void showMetadata(SnifferPacket *snifferPacket) {
    
  unsigned int frameControl = ((unsigned int)snifferPacket->data[1] << 8) + snifferPacket->data[0];
  uint8_t frameType    = (frameControl & 0b0000000000001100) >> 2;
  uint8_t frameSubType = (frameControl & 0b0000000011110000) >> 4;
  
//===================================== FILTRO I SOLI PROBE REQUEST
  
  if (frameType == TYPE_MANAGEMENT && frameSubType == SUBTYPE_PROBE_REQUEST) 
     {
      getMAC(addr, snifferPacket->data, 10);                 //prendo il MAC di chi ha inviato il probe
      flag=0;       
      for(i=0;i<occupancy;i++){                              //controllo se ho già il MAC in memoria e nel caso aggiorno l'rssi e la delta
         if(strcmp(addr,Saved[i].mac)==0)
                      {
                       if(Saved[i].RSSI != snifferPacket->rx_ctrl.rssi){ //se la potenza ricevuta è diversa allora la aggiorno
					               for(int j=FINESSE-1;j>=0;j--){                  //rimuovo dal conteggio dei dispositivi la vecchia potenza
                           if(Saved[i].RSSI>=pwrange[j]){                
                                                 switch(Saved[i].rndm){  
                                                      case 0: delta[j]--; break;
                                                      case 1: deltaRndm[j]--; break; 
                                                 }                        
                           }
                           else break;
                           } 
					             Saved[i].RSSI=snifferPacket->rx_ctrl.rssi;		     //salvo la nuova potenza ricevuta			             
                       for(int j=0;j<FINESSE;j++){                       //aggiorno il conteggio attuale dei dispositivi
                       if(Saved[i].RSSI>=pwrange[j] && Saved[i].rndm==0) delta[j]++;
                       if(Saved[i].RSSI>=pwrange[j] && Saved[i].rndm==1) deltaRndm[j]++;
                       }
                       }
                       Saved[i].Time=millis();                           //salvo il momento in cui ho ricevuto il probe
                       flag=1;                   
                       break;
                       }
                       }
                       
        if(flag==0){                                                     //se è un nuovo dispositivo non già registrato
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


// ========================== FUNZIONE DI CALLBACK
 
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
//============================= Creazione della connessione wifi
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
    return 0;                     //ci sta mettendo troppo a collegarsi, lascio perdere
  }
  }
  Serial.println("");
  Serial.println("WiFi connected");  
  return 1;                                 //connessione avvenuta con successo
}

//============================= Connessione ed invio del messaggio MQTT
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
               error=error+pow(temp,2); //così calcolo tutti gli errori alla potenza j al passato k
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

//================ TOLGO LE PERSONE INATTIVE PER PIU' DI 7 MIN

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


//================ TRASMISSIONE MQTT

 buttonstatus=digitalRead(button);
 CurrentMills=millis();
 if(CurrentMills-PrevMills>mqtt_freq || buttonstatus!=lastButtonStatus){
    
  digitalWrite(ledpin,LOW);
  wifi_promiscuous_enable(0);
  // CONNETTO AL WIFI
  if(!setup_wifi()){
    buttonstatus=digitalRead(button);
    lastButtonStatus=buttonstatus;
    PrevMills=millis(); 
    digitalWrite(ledpin,HIGH); 
    wifi_promiscuous_enable(1);
    delay(10);
    return;
  }
  //CONNETTO ALL'MQTT
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
  
  if(!mqtt_rx()){
    buttonstatus=digitalRead(button);
    lastButtonStatus=buttonstatus;
    PrevMills=millis(); 
    digitalWrite(ledpin,HIGH);
    wifi_promiscuous_enable(1);
    delay(10); 
    return;
  }
    
  nciclo++;  
  if(SetPower==0){  //allora in questo caso deve aggiustare i filtri di potenza per un nuovo nreal
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
