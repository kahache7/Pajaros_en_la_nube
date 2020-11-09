/*
 *  Pajaros en la nube V2.0 beta
 *  Author: Enrique Torres enrique.torres@unizar.es 
 *  Based on code from the examples, mixing lowpower wakeup by timer and LoRAWAN 
 *  region EU868,Class A, ABP, Unconfirmed, AT support off (save power)
 *  RGB active? no to save power, on for debug
 *  
 *  Dependencies: Adafruit VL6180x
 *  
 *  LoRAWan_APP, onewire and dallastemperature from Heltech framework (last framework, bug in previous)
 */
 
#include "LoRaWan_APP.h"
#include "Arduino.h"

//kike
  #include <Wire.h>
  #include "Adafruit_VL6180X.h"

  #include <OneWire.h> //kike lib renamed from the cubecell repositorie with conflict with my other onewire lib
  #include <DallasTemperature.h>

  enum constants{
    // Data wire is plugged into GPIO5 on the CubeCell
    ONE_WIRE_BUS          = GPIO5,
    NUM_DS18b20           = 3,
    TEMPERATURE_PRECISION = 12 //kike ideal 9 por compromiso precision tiempo de conversion, pero como le cuesta a la ds18b20 china lo mismo 9 que 12...
  };

  #define SHOW_DEBUGINFO
  #ifdef SHOW_DEBUGINFO
    #define BAUD_SERIAL 115200
    #define debugSerial Serial
    #define debugPrintln(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
    #define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }
    #define debugPrintf(...) { if (debugSerial) debugSerial.printf(__VA_ARGS__); }
    #define debugPrintfln(...) { if (debugSerial) debugSerial.printf(__VA_ARGS__); debugSerial.println("");}
  #else
    #define debugSerial null
    #define debugPrintln(...) {}
    #define debugPrint(...) {}
    #define debugPrintf(...) {}
    #define debugPrintfln(...) {}  
  #endif
// /kike

/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

////////
// LoRaWAN
  #include "lora_keys.h" //kike lora ABP app sesion key, network sesion key and device address

  /* OTAA para*/
  uint8_t devEui[] = { 0x22, 0x32, 0x33, 0x00, 0x00, 0x88, 0x88, 0x02 };
  uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint8_t appKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x66, 0x01 };
  
  /* ABP para*/
  //kike se cogen del fichero lora_key.h
  // dentro solo tiene algo del estilo a:
  //  /* ABP para*/
  //uint8_t mynwkSKey[] = { clave de sesion de red };
  //uint8_t myappSKey[] = { clave de sesion de la app };
  //uint32_t mydevAddr =  ( uint32_t )0xdexaddr del dispositivo;
  uint8_t nwkSKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x66, 0x01 };
  uint8_t appSKey[] = { 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x66, 0x01 };
  uint32_t devAddr =  ( uint32_t )0x26011881; 

  /*LoraWan channelsmask, default channels 0-7*/ 
  //uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
  uint16_t userChannelsMask[6]={ 0x0001,0x0000,0x0000,0x0000,0x0000,0x0000 }; //kike channel cero only to support ABP one channel gateways
  
  /*LoraWan region, select in arduino IDE tools*/
  LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
  
  /*LoraWan Class, Class A and Class C are supported*/
  DeviceClass_t  loraWanClass = LORAWAN_CLASS;
  
  /*the application data transmission duty cycle.  value in [ms].*/
  //uint32_t appTxDutyCycle = 15000;
  uint32_t appTxDutyCycle = 1 * 60 * 1000; //kike mitutos entre transmisiones
  
  /*OTAA or ABP*/
  bool overTheAirActivation = LORAWAN_NETMODE;
  
  /*ADR enable*/
  bool loraWanAdr = LORAWAN_ADR;
  
  /* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
  bool keepNet = LORAWAN_NET_RESERVE;
  
  /* Indicates if the node is sending confirmed or unconfirmed messages */
  bool isTxConfirmed = LORAWAN_UPLINKMODE;
  
  /* Application port */
  uint8_t appPort = 2;
  /*!
  * Number of trials to transmit the frame, if the LoRaMAC layer did not
  * receive an acknowledgment. The MAC performs a datarate adaptation,
  * according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  * to the following table:
  *
  * Transmission nb | Data Rate
  * ----------------|-----------
  * 1 (first)       | DR
  * 2               | DR
  * 3               | max(DR-1,0)
  * 4               | max(DR-1,0)
  * 5               | max(DR-2,0)
  * 6               | max(DR-2,0)
  * 7               | max(DR-3,0)
  * 8               | max(DR-3,0)
  *
  * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
  */
  uint8_t confirmedNbTrials = 4;

///////
// sensors
 
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire oneWire(ONE_WIRE_BUS);
  
  // Pass our oneWire reference to Dallas Temperature.
  DallasTemperature sensors(&oneWire);
  
  // arrays to hold device addresses
  DeviceAddress DS18b20_0, DS18b20_1, DS18b20_2;


  // TimeOfFlight //kike altura del nido
  Adafruit_VL6180X ToF = Adafruit_VL6180X();

//////
// ToF

  void ToF_init(){
    if (! ToF.begin()) {
      debugPrintln("Failed to find sensor");
      //kike todo retry? no seguir?
      //while ( xxxx && count < maxtry) { count++ ...
    }
  }
  
  uint8_t VL6180X_leer(){

    ToF_init(); //kike TODO es necesario el begin cada vez?
    
    uint8_t range = ToF.readRange();
    uint8_t status = ToF.readRangeStatus();
    Wire.end();
    
    if (status == VL6180X_ERROR_NONE) {
      debugPrint("VL6180X Range: "); debugPrint(range);
    }

    #ifdef SHOW_DEBUGINFO
      // Some error occurred, print it out!   
      if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
        debugPrintln("System error");
      }
      else if (status == VL6180X_ERROR_ECEFAIL) {
        debugPrintln("ECE failure");
      }
      else if (status == VL6180X_ERROR_NOCONVERGE) {
        debugPrintln("No convergence");
      }
      else if (status == VL6180X_ERROR_RANGEIGNORE) {
        debugPrintln("Ignoring range");
      }
      else if (status == VL6180X_ERROR_SNR) {
        debugPrintln("Signal/Noise error");
      }
      else if (status == VL6180X_ERROR_RAWUFLOW) {
        debugPrintln("Raw reading underflow");
      }
      else if (status == VL6180X_ERROR_RAWOFLOW) {
        debugPrintln("Raw reading overflow");
      }
      else if (status == VL6180X_ERROR_RANGEUFLOW) {
        debugPrintln("Range reading underflow");
      }
      else if (status == VL6180X_ERROR_RANGEOFLOW) {
        debugPrintln("Range reading overflow");
      }
    #endif
    
    return range;
  }
  
///////////
// ds18b20

//  float tempC1, tempC2, tempC3;
  
  void printAddress(DeviceAddress deviceAddress){
    #ifdef SHOW_DEBUGINFO 
      for (uint8_t i = 0; i < 8; i++)
      {
        debugPrint("0x");
        if (deviceAddress[i] < 0x10) debugPrint("0");
        debugPrint(deviceAddress[i], HEX);
        if (i < 7) debugPrint(", ");
      }
      debugPrintln("");
   #endif 
  }

  void DS18b20_init(){
    uint8_t num_detected;

    oneWire.reset_search();
    
    sensors.begin();

    //kike todo pensar los retry count = 0;
    debugPrintln("");
    debugPrintln("DS18b20:");
    // locate devices on the bus
    debugPrint("Locating devices...");
    num_detected = sensors.getDeviceCount();
    debugPrintfln("Found %d devices.", num_detected);

    if (num_detected != NUM_DS18b20){
      debugPrintfln("ERROR: se eseraban %d", NUM_DS18b20); 
    }
    
    // report parasite power requirements
    debugPrint("Parasite power is: ");
    if (sensors.isParasitePowerMode()) {debugPrintln("ON");} //kike TODO algo pasa con las macros y las {}
    else debugPrintln("OFF");
  
    // Search for devices on the bus and assign based on an index. Ideally,
    // you would do this to initially discover addresses on the bus and then
    // use those addresses and manually assign them (see above) once you know
    // the devices on your bus (and assuming they don't change).
    //
    // method 1: by index 
    debugPrintln("Printing addresses...");
    
    if (!sensors.getAddress(DS18b20_0, 0)){
      debugPrint("Unable to find address for Device ");
      debugPrintln(0);
    } else {
      debugPrintf("Sensor %d: ",0);
      printAddress(DS18b20_0);
    }
    //kike TODO hacer array de adress y no replicar!!
    if (!sensors.getAddress(DS18b20_1, 1)){
      debugPrint("Unable to find address for Device ");
      debugPrintln(1);
    } else {
      debugPrintf("Sensor %d: ",1);
      printAddress(DS18b20_1);
    }
    if (!sensors.getAddress(DS18b20_2, 2)){
      debugPrint("Unable to find address for Device ");
      debugPrintln(2);
    } else {
      debugPrintf("Sensor %d: ",2);
      printAddress(DS18b20_2);
    }
    
    // set the resolution to n bit per device
    sensors.setResolution(DS18b20_0, TEMPERATURE_PRECISION);
    sensors.setResolution(DS18b20_1, TEMPERATURE_PRECISION);
    sensors.setResolution(DS18b20_2, TEMPERATURE_PRECISION);

    #ifdef SHOW_DEBUGINFO
      debugPrintfln("Resoluccion Sensor %d: %d", 0, sensors.getResolution(DS18b20_0));
      debugPrintfln("Resoluccion Sensor %d: %d", 1, sensors.getResolution(DS18b20_1));
      debugPrintfln("Resoluccion Sensor %d: %d", 2, sensors.getResolution(DS18b20_2));
    #endif
  }
  
  void DS18b20_leer(float temperatura[NUM_DS18b20]){

    //kike TODO se puede no pasar por el begin cada vez que se vuelva de power down??? 
//    oneWire.reset(); //    

    sensors.begin();  //kike en main DS18b20_init();

    //kike TODO lectura asyncrona?????
    sensors.requestTemperatures();
//    delay(120);
    
    temperatura[0] = sensors.getTempC(DS18b20_0);
    temperatura[1] = sensors.getTempC(DS18b20_1);
    temperatura[2] = sensors.getTempC(DS18b20_2);

    #ifdef SHOW_DEBUGINFO
      debugPrintln();
      debugPrint("DS18b20:");
      for (int i = 0; i< NUM_DS18b20;i++){
        debugPrintf(" Temp C%d: ", i);
        debugPrint(temperatura[i]);
      }
      debugPrintln();
    #endif
  }

  void DS18b20_leer_asyn_start(){
   sensors.begin();  //kike en main DS18b20_init();

   //kike peticion asyncrona
   sensors.setWaitForConversion(false);
   sensors.requestTemperatures();
//   sensors.setWaitForConversion(true);    
  }
  
  void DS18b20_leer_asyn(float temperatura[NUM_DS18b20]){
    //kike si aun no ha terminado de convertir... esperar
    while(!sensors.isConversionComplete()){
      #ifdef SHOW_DEBUGINFO      
        debugPrint(".");
      #endif
      delay(50);
    }

    temperatura[0] = sensors.getTempC(DS18b20_0);
    temperatura[1] = sensors.getTempC(DS18b20_1);
    temperatura[2] = sensors.getTempC(DS18b20_2);
  }
  
// kike ******* fin sondas

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/

  float temperatura[NUM_DS18b20];
  
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(50); //kike era 500 para bme280, en el ejempplo del VL6180x 50

  DS18b20_leer_asyn_start();

  #ifdef SHOW_DEBUGINFO
    debugPrintln();
    debugPrintln("********** Formando nueva trama **********");
    debugPrintln();
  #endif
    
  uint8_t altura_nido = VL6180X_leer();
  debugPrintln();

  uint16_t voltage = getBatteryVoltage();

  #ifdef SHOW_DEBUGINFO
    debugPrintln();
    debugPrintfln("Bateria mV: %d", voltage);
    debugPrintln();
  #endif
  
  //kike aun convirtiendo las lentas temperauras seguro... adelantar trabajo
  appDataSize = 9;    
  appData[0] = highByte(voltage); //(uint8_t)(voltage >> 8)
  appData[1] = lowByte(voltage);  //(uint8_t)voltage

  appData[2] = altura_nido;

  DS18b20_leer_asyn(temperatura); 

  //kike power off lo antes posible
  digitalWrite(Vext, HIGH);

  #ifdef SHOW_DEBUGINFO
    debugPrintln();
    debugPrint("DS18b20:");
    for (int i = 0; i< NUM_DS18b20;i++){
      debugPrintf(" Temp C%d: ", i);
      debugPrint(temperatura[i]);
    }
    debugPrintln();
  #endif
  
  for (int i = 0; i< NUM_DS18b20;i++){
    int16_t itemp = int16_t(temperatura[i] * 100);

    if (itemp < 0){
        itemp *= -1;
        itemp |= 0x8000;
    }
    appData[3 + (i * 2)] = highByte(itemp); //(uint8_t)(voltage >> 8)
    appData[4 + (i * 2)] = lowByte(itemp);  //(uint8_t)voltage
  }

//kike TTN decoder at the end of code
}

/* //kike del ejemplo de downlink por si se desea...
//downlink data handle function example
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++)
  {
    Serial.printf("%02X",mcpsIndication->Buffer[i]);
  }
  Serial.println();
  uint32_t color=mcpsIndication->Buffer[0]<<16|mcpsIndication->Buffer[1]<<8|mcpsIndication->Buffer[2];
#if(LoraWan_RGB==1)
  turnOnRGB(color,5000);
  turnOffRGB();
#endif
}
*/

void setup() {

  //copy lora_keys
    memcpy(nwkSKey, (void*)mynwkSKey, sizeof(mynwkSKey)); //copy keys from .h
    memcpy(appSKey, (void*)myappSKey, sizeof(myappSKey));
    devAddr = mydevAddr;
//    memcpy(devAddr, (void*)mydevAddr, sizeof(mydevAddr));

  
	boardInitMcu();

  #ifdef SHOW_DEBUGINFO
	  Serial.begin(BAUD_SERIAL);
  #endif
  
  uint64_t chipID=getID();
  debugPrintf("ChipID: %04X%08X\r\n",(uint32_t)(chipID>>32),(uint32_t)chipID);

  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
  delay(500); //kike era 500 para bme280, en el ejempplo del VL6180x 50

  DS18b20_init(); //kike get ds18b20 address
  
  digitalWrite(Vext, HIGH);
  
  
#if(AT_SUPPORT)
	enableAt();
#endif

	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();
}

void loop()
{
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
#if(AT_SUPPORT)
			getDevParam();
#endif
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
			prepareTxFrame( appPort );
			LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
			deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
			LoRaWAN.sleep();
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}

/* //kike DECODER FUNCTION FOR TTN
 
function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};

  if (port === 2) {
    decoded.batt = bytes[0]<<8 | bytes[1];
    if (decoded.batt === 0)
       delete decoded.batt; 
    
    decoded.altura = bytes[2];
    
    var celciusInt = bytes[4] | (bytes[3] << 8);

    if (celciusInt & 0x8000) celciusInt = (celciusInt & 0x7fff) * -1;
    decoded.tempC1 = celciusInt / 100;
    
    celciusInt = bytes[6] | (bytes[5] << 8);

    if (celciusInt & 0x8000) celciusInt = (celciusInt & 0x7fff) * -1;
    decoded.tempC2 = celciusInt / 100;

    celciusInt = bytes[8] | (bytes[7] << 8);
    
    if (celciusInt & 0x8000) celciusInt = (celciusInt & 0x7fff) * -1;
    decoded.tempC3 = celciusInt / 100;    
  }
  return decoded;
}
*/
