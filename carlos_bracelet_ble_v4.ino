/*********************************************************************
Artem mofidied the Wire_nrf52.cpp file, and changed the master initialization code
The original adafruit feather code would hang when used with BNO080. 
*********************************************************************/
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "Adafruit_Si7021.h"
#include "max86150.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include "SparkFun_BNO080_Arduino_Library.h"

//#define BNO055_EULER_H_LSB_ADDR         0X1A


// BLE Service
BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery

//uint8_t pinSDA = 26;
//uint8_t pinSCL = 27;

//TwoWire nRFwire = TwoWire(pinSDA,pinSCL);
Adafruit_Si7021 sensor_humidity = Adafruit_Si7021();
MAX86150 max86150Sensor;
BNO080 myIMU;
//Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

uint16_t ppgunsigned16_red;
uint16_t ppgunsigned16_ir;

boolean READ_HUMIDITY_TEMPERATURE = false;
boolean READ_ORIENTATION = false;
boolean READ_PPG = true;

int _TRANSACTION_TIMEOUT = 100;

int LED_PIN = 34;
void setup()
{

  pinMode(LED_PIN, OUTPUT);
 // Serial.begin(115200);
//  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  
//  Serial.println("Bluefruit52 BLEUART Example");
//  Serial.println("---------------------------\n");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52_artemito2");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();
 //Wire.setClock(400000); //Increase I2C data rate to 400kHz
 if(READ_HUMIDITY_TEMPERATURE) {

  //activate pullups on I2C line for the humidity sensor I2C
   *pincfg_reg(27) = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                           | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                           | ((uint32_t)GPIO_PIN_CNF_PULL_Pullup      << GPIO_PIN_CNF_PULL_Pos)
                           | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0D1       << GPIO_PIN_CNF_DRIVE_Pos)
                           | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);

  *pincfg_reg(26) = ((uint32_t)GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos)
                           | ((uint32_t)GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                           | ((uint32_t)GPIO_PIN_CNF_PULL_Pullup      << GPIO_PIN_CNF_PULL_Pos)
                           | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0D1       << GPIO_PIN_CNF_DRIVE_Pos)
                           | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);

  
  sensor_humidity.begin();
 }
 if(READ_PPG) { 
  max86150Sensor.begin(Wire, 100000);
  max86150Sensor.readPartID();
  max86150Sensor.setup();
 }

 if (READ_ORIENTATION) { 
  Wire.begin();
  //Wire.setClock(400000); //Increase I2C data rate to 400kHz
  if (myIMU.begin() == false) {
    digitalWrite(LED_PIN, HIGH);
  }
  myIMU.enableRotationVector(10); //Send data update every 50ms
  delay(1000);
 }
// bno.begin();
// delay(1000);
// bno.setExtCrystalUse(true);
}



void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

static volatile uint32_t* pincfg_reg(uint32_t pin)
{
  NRF_GPIO_Type * port = nrf_gpio_pin_port_decode(&pin);
  return &port->PIN_CNF[pin];
}

void loop()
{
    //read the temperatutre sensor 
     // Delay to wait for enough input, since we have a limited transmission buffer
    char buf[64];
    
    if(READ_HUMIDITY_TEMPERATURE) {
      delay(50);
      float humidity_reading = sensor_humidity.readHumidity();
      float temperature_reading = sensor_humidity.readTemperature();
      
      char humidity_string[5]; 
      char temperature_string[5]; 
      snprintf(humidity_string, 6, "%f", humidity_reading);
      snprintf(temperature_string, 6, "%f", temperature_reading);
      snprintf(buf, sizeof buf, "H,%s,%s\r\n", humidity_string,temperature_string);
  
      int count = 18;
      bleuart.write( buf, count );
    } 
    delay(2);
    //read the ppg sensor
    if(READ_PPG){
      if(max86150Sensor.check()>0)
      {
          ppgunsigned16_red = (uint16_t) (max86150Sensor.getFIFORed()>>2);
          ppgunsigned16_ir = (uint16_t) (max86150Sensor.getFIFOIR()>>2);
          char red_string[5]; 
          char ir_string[5]; 
          snprintf(red_string, 6, "%i", ppgunsigned16_red);
          snprintf(ir_string, 6, "%i", ppgunsigned16_ir);
          snprintf(buf, sizeof buf, "P,%s,%s\r\n", red_string,ir_string);
          bleuart.write( buf, 14 );
      }
    }
    if(READ_ORIENTATION) { 

      if (myIMU.dataAvailable() == true)
      {
        uint8_t bufferz[6]; 
        int16_t x, y, z;
        int buff_count = 0;
        float quatI = myIMU.getQuatI();
        float quatJ = myIMU.getQuatJ();
        float quatK = myIMU.getQuatK();
        float quatReal = myIMU.getQuatReal();
        float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
      
   
        char x_string[5]; 
        char y_string[5];
        char z_string[5];  
        char real_string[5];  
        snprintf(x_string, 6, "%f", quatI);
        snprintf(y_string, 6, "%f", quatJ);
        snprintf(z_string, 6, "%f", quatK);
        snprintf(real_string, 6, "%f", quatReal);
        snprintf(buf, sizeof buf, "O,%s,%s,%s,%s\r\n", x_string,y_string, z_string,real_string );
 
        
        bleuart.write( buf, 30 );
        delay(20);
      }
    
    }   
    uint8_t ch;
    ch = (uint8_t) bleuart.read();

}


uint8_t * i2c_read_array(uint8_t addr, uint8_t size) { 
  uint8_t *data;
  uint8_t tx_buf[1]; 
  uint8_t *rx_buf; 
  //const uint8_t length = size; 
  static uint8_t buffera[8];
    // Enable shortcuts that starts a read right after a write and sends a stop condition after last TWI read
  NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STARTRX_Msk | TWIM_SHORTS_LASTRX_STOP_Msk;
  
  tx_buf[0] = addr;
  NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
  NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf[0];
  
  /* set the data pointer */
  NRF_TWIM0->RXD.MAXCNT = size; //Max number of bytes per transfer
  //NRF_TWIM0->RXD.MAXCNT = (size << TWIM_RXD_MAXCNT_MAXCNT_Pos);

  NRF_TWIM0->RXD.PTR = (uint32_t)&buffera; //point to RXD buffer
  
  //Start read sequence. Note that it uses starttx, not start RX
  NRF_TWIM0->EVENTS_STOPPED = 0;
  NRF_TWIM0->TASKS_STARTTX = 1;
  
  /* wait for the device to finish up */
  while (NRF_TWIM0->EVENTS_STOPPED == 0);

  //SEGGER_RTT_printf(0,"%x,%x,%x,%x,%x,%x,%x,%x \n,",buffer[0],buffer[1], buffer[2], buffer[3], buffer[4], buffer[5],buffer[6],buffer[7]);
  return buffera;
}

int16_t * BNO055_getEulerAngles() {
  uint8_t * bufferq;
  //memset (buffer, 0, 8);
  static int16_t angles[3]; 
  int16_t x, y, z;
  x = y = z = 0;
  bufferq = i2c_read_array(Adafruit_BNO055::BNO055_EULER_H_LSB_ADDR, 8);
  x = (((int16_t)bufferq[1]) << 8) | ((int16_t)bufferq[0]);
  y = (((int16_t)bufferq[3]) << 8) | ((int16_t)bufferq[2]);
  z = (((int16_t)bufferq[5]) << 8) | ((int16_t)bufferq[4]);
  
  angles[0] =x; // scale*w; 
  angles[1] =y;// scale*x; 
  angles[2] =z;// scale*y; 
  
  return angles;
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

//  Serial.print("Connected to ");
//  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

 // Serial.println();
 // Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}
