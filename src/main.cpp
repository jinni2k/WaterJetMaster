/*
   //##############################################################################
   //Local Panel DI/O mapping  Master
   01, DI Emergency Stop B접점    mDI_EmergencyStop          mdi01     MbReg[0.0]
   02, DI Supply Pump Start       mDI_PumpStart             mdi02      MbReg[0.1]
   03, DI Hyd' Motor Start        mDI_HydMotorStart         mdi03      MbReg[0.2]
   04, Not connected

   05, DI Nozzle On               mDI_NozzleOn              mdi05      MbReg[0.4]
   06, DI Abrasive(연마제) 투입    mDI_AbrasiveIn             mdi06     MbReg[0.5]
   07, DI Abrasive Level Low      mDI_AbrasiveLow           mdi07      MbReg[0.6]
   08, DI Servo Fault B접점       mDI_ServoFault            mdi08      MbReg[0.7]

   -- Relay Output
   01, DO Hyd' Motor Overload      mDO_HydMotorOVR            mdo01     MbReg[4.0]
   02, DO Hyd' Motor run Status    mDO_HydMotorStatus         mdo02     MbReg[4.1]
   03, DO Control Power Ready      mDO_CtrlPowerReady         mdo03     MbReg[4.2]
   04, DO Supply Pump Fault        mDO_SupplyPumpFault        mdo04     MbReg[4.3]

   08, DO Cumunication Fault      mDO_ComFault              mdo08      MbReg[4.7]
*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TaskScheduler.h>
#include <pcf8574_esp.h>
#include <advancedSerial.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <Ticker.h>
#include <SoftwareSerial.h>
#include <RemoteDebug.h>
#include "modbus.h"

#define  TXEN D7        //D7: DE     422 Comm' TX enable
#define  _RXEN D8       //D8: _RE    422 Comm' RX enable low Active
#define HOST_NAME "JWCS-Master"

void controlLoop();
void communications();
void display();
void comCheck();
void chk_key();
Task t0(100,   TASK_FOREVER, &controlLoop);

Scheduler runner;

RemoteDebug Debug;
SoftwareSerial swSerial(D5,D6, false, 1024);  //RX, TX
const char* host = "esp8266";
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
const char* OTAssid = "jujunet";
const char* OTApassword = "12345678";

int initial = 0,pumpStartT1=0,pumpStartT2=0;
void otaSetup();
unsigned int controlScan = 0,ocontrolScan,octlS[2],hydMotorTime;
bool mDI_EmergencyStop,mDI_PumpStart,mDI_HydMotorStart,mDI_NozzleOn,mDI_AbrasiveIn,mDI_AbrasiveLow,mDI_ServoFault;
bool mDO_CtrlPowerReady,mDO_HydMotorOVR,mDO_HydMotorStatus,mDO_SupplyPumpFault,mDO_ComFault;
bool sDI_HydMotorOVR,sDI_HydMotorStatus,sDI_CtrlPowerReady,sDI_HydMotorOn,sDI_OilLevelLow,sDI_OilTempLow,sDI_SupplyPressLow,sDI_EmergencyStop;
bool sDI_IntLeftSensor,sDI_IntRightSensor,sDI_OilTempHigh,sDI_SupplyMotorOL,sDI_LocalMode,sDI_SupplyPumpRunC,sDI_HydMotorRunC,sDI_PressHighC,sDI_SupplyPumpFault;
bool sDO_HydMotorStart,sDO_EmgStop,sDO_IntleftSol,sDO_IntRightSol,sDO_SupplySol,sDO_LowPresSol,sDO_HighPresSol,sDO_SupplyPumpStart;
bool sDO_HydYDTimer,sDO_BleedDownV,sDO_ServoFaultL,sDO_PumpFaultL,sDO_HydMotorFaultL,sDO_PumpRunL,sDO_HydMotorRunL,sDO_ComFault;

volatile bool PCFInterruptFlag = false;
PCF857x DI8_0(0x20, &Wire);  //DI00 ~ DI07      DI
PCF857x DI8_1(0x21, &Wire);  //DI08 ~ DI15      DI
PCF857x RO8_0(0x22, &Wire);  //DO00 ~ DO07      DO
PCF857x FaceBt(0x23, &Wire);  //Face Plate Botton Input
PCF857x DI8_2(0x24, &Wire);  //DI16 ~ DI23      DI
PCF857x DI8_3(0x25, &Wire);  //DI24 ~ DI31      DI
PCF857x RO8_1(0x26, &Wire);  //DO08 ~ DO15      DO

uint8_t DI[4];

void communications() {
        DI[0] = ~DI8_0.read8();
        DI[1] = ~DI8_1.read8();
        DI[2] = ~DI8_2.read8();
        DI[3] = ~DI8_3.read8();

        MbReg[10] = word(DI[1],DI[0]);         //DI00~15  DI
        MbReg[11] = word(DI[3],DI[2]);         //DI16~31  Extensoion DI

        sDI_HydMotorOVR       = bitRead(MbReg[0],0);
        sDI_HydMotorStatus    = bitRead(MbReg[0],1);
        sDI_CtrlPowerReady    = bitRead(MbReg[0],2);
        sDI_SupplyPumpFault   = bitRead(MbReg[0],3);
        sDI_OilLevelLow       = bitRead(MbReg[0],4);
        sDI_OilTempLow        = bitRead(MbReg[0],5);
        sDI_SupplyPressLow    = bitRead(MbReg[0],6);
        sDI_EmergencyStop     = bitRead(MbReg[0],7);
        sDI_IntLeftSensor     = bitRead(MbReg[0],8);
        sDI_IntRightSensor    = bitRead(MbReg[0],9);
        sDI_OilTempHigh       = bitRead(MbReg[0],10);
        sDI_SupplyMotorOL     = bitRead(MbReg[0],11);
        sDI_LocalMode         = bitRead(MbReg[0],12);
        sDI_SupplyPumpRunC    = bitRead(MbReg[0],13);
        sDI_HydMotorRunC      = bitRead(MbReg[0],14);
        sDI_PressHighC        = bitRead(MbReg[0],15);

        //slave Relay Output
        bitWrite(MbReg[4],0,sDO_HydMotorStart);
        bitWrite(MbReg[4],1,sDO_EmgStop);
        bitWrite(MbReg[4],2,sDO_IntleftSol);
        bitWrite(MbReg[4],3,sDO_IntRightSol);
        bitWrite(MbReg[4],4,sDO_SupplySol);
        bitWrite(MbReg[4],5,sDO_LowPresSol);
        bitWrite(MbReg[4],6,sDO_HighPresSol);
        bitWrite(MbReg[4],7,sDO_SupplyPumpStart);

        bitWrite(MbReg[4],8,sDO_HydYDTimer);
        bitWrite(MbReg[4],9,sDO_BleedDownV);
        bitWrite(MbReg[4],10,sDO_ServoFaultL);
        bitWrite(MbReg[4],11,sDO_PumpFaultL);
        bitWrite(MbReg[4],12,sDO_HydMotorFaultL);
        bitWrite(MbReg[4],13,sDO_PumpRunL);
        bitWrite(MbReg[4],14,sDO_HydMotorRunL);
        bitWrite(MbReg[4],15,sDO_ComFault);

        mDI_EmergencyStop     =  !bitRead(MbReg[10],0);
        mDI_PumpStart         =  bitRead(MbReg[10],1);
        mDI_HydMotorStart     =  bitRead(MbReg[10],2);

        mDI_NozzleOn          =  bitRead(MbReg[10],4);
        mDI_AbrasiveIn        =  bitRead(MbReg[10],5);
        mDI_AbrasiveLow       =  bitRead(MbReg[10],6);
        mDI_ServoFault        =  bitRead(MbReg[10],7);

        //Master Relay Output
      //  bitWrite(mDO_HydMotorOVR,     0,bitRead(MbReg[24],0));
      //  bitWrite(mDO_HydMotorStatus,  1,bitRead(MbReg[24],1));
      //  bitWrite(mDO_CtrlPowerReady,  2,bitRead(MbReg[24],2));
      //  bitWrite(mDO_SupplyPumpFault, 3,bitRead(MbReg[24],3));

        bitWrite(MbReg[14],7,mDO_ComFault);
        rdebugVln("MbReg[5] : %u,",MbReg[5]);
       //MbReg[24] =11;
        for(int i =0;i<4;i++)RO8_0.write(i,bitRead(~MbReg[5],i));

        RO8_0.write(7,bitRead(~MbReg[14],7));
        //for(int i =0;i<8;i++)RO8_1.write(i,bitRead(~MbReg[14],i+8));
        //RO8_1.write8(~highByte(MbReg[4]));
}

void comCheck(){
        if(octlS[0] == MbReg[3]) mDO_ComFault = 1;
        else mDO_ComFault = 0;
        rdebugVln("comScan : %u, %u",MbReg[3],octlS[0]);
        octlS[0] = MbReg[3];

}

void controlLoop() {
        wdt_reset();
        controlScan++;
        if(controlScan>10000) controlScan = 0;
        MbReg[13] =controlScan;

        communications();
        if(controlScan%10==0) comCheck();
}

void setup() {
        ESP.wdtEnable(2000);
        Serial.begin(19200, SERIAL_8E1);
        runner.init();
        runner.addTask(t0);
        t0.enable();

        DI8_0.begin();
        DI8_1.begin();
        DI8_2.begin();
        DI8_3.begin();
        RO8_0.begin();
        RO8_1.begin();
        FaceBt.begin();

        pinMode(TXEN,OUTPUT);
        pinMode(_RXEN,OUTPUT);
        digitalWrite(TXEN, HIGH);
        digitalWrite(_RXEN, LOW);
        Wire.begin();
        Wire.setClock(100000L);
        WiFi.mode(WIFI_STA);
        WiFi.begin(OTAssid-, OTApassword);
        otaSetup();
        String hostNameWifi = HOST_NAME;
        hostNameWifi.concat(".local");
        WiFi.hostname(hostNameWifi);
        MDNS.addService("telnet", "tcp", 23);
        Debug.begin(HOST_NAME); // Initiaze the telnet server
        Debug.setResetCmdEnabled(true); // Enable the reset command
}

void loop() {
        wdt_reset();
        runner.execute();
        ArduinoOTA.handle();
        httpServer.handleClient();
        Debug.handle();
        MBSlave(0, 234);
}

void otaSetup() {
        // Port defaults to 8266
        // ArduinoOTA.setPort(8266);
        // Hostname defaults to esp8266-[ChipID]
        // ArduinoOTA.setHostname("myesp8266");
        // No authentication by default
        // ArduinoOTA.setPassword((const char *)"123");
        ArduinoOTA.onStart([]() {
                Serial.println("Start");
        });
        ArduinoOTA.onEnd([]() {
                Serial.println("\nEnd");
        });
        //ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        //  Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        //});
        ArduinoOTA.onError([](ota_error_t error) {
                Serial.printf("Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });
        ArduinoOTA.begin();
        Serial.println("Ready - OTA Success!!!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        MDNS.begin(host);
        httpUpdater.setup(&httpServer);
        httpServer.begin();
        MDNS.addService("http", "tcp", 80);
        String ipaddress = WiFi.localIP().toString();
        String chipID = String(ESP.getChipId(), HEX);
        char charChipID[10];
        chipID.toCharArray(charChipID, sizeof(charChipID));
        char charipaddress[16];
        ipaddress.toCharArray(charipaddress, sizeof(charipaddress));
        Serial.printf("Now open http://%s.local/update in your browser or \n", host);
        Serial.printf("http://%s/update or http://%s.lan/update if you prefer.\n", charipaddress, charChipID);
}
