#include "ModbusXT.h"
#include <HX711.h> //This library can be obtained here http://librarymanager/All#Avia_HX711

HX711 scale;

// Configuration for modbus RTU ---------------------------
#define TIMEOUT 500   //Timeout for a failed packet. Timeout need to larger than polling
#define POLLING 2     //Wait time to next request

#define BAUD        115200  
#define RETRIES     10    //How many time to re-request packet frome slave if request is failed
#define BYTE_FORMAT SERIAL_8E1
// This config just for module RS485 don't have pin enable
#define TxEnablePin 10   //Arduino pin to enable transmission default: 2

#define print(x)  Serial.print(x)
#define println(x) Serial.println(x)
//--------------------------------------------------

// Configuration for HX711 ---------------------------
#define LOADCELL_DOUT_PIN  3
#define LOADCELL_SCK_PIN  2

float calibration_factor = -7050; //-7050 worked for my 440lb max scale setup
//--------------------------------------------------

//Name for register in regs[]
enum {
  button1,
  button2,
  button3,
  number_entry,
  password_entry,
  slider,
  total_packets,
  total_requests,
  total_failed,
  transfer_rate,
  transfer_delay,
  led_grn,
  led_blue,
  led_red,
  graph,
  // slideValue,  
  TOTAL_REGS //=15 -> 16
};

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum {
  PACKET1,
  PACKET2,
  NO_OF_PACKET  //=2
};

// Masters register array
int32_t regs[TOTAL_REGS];

//Modbus packet
Packet packets[NO_OF_PACKET];

// Access individual packet parameter. Uncomment it if you know what you're doing
// packetPointer packet1 = &packets[PACKET1];
// packetPointer packet2 = &packets[PACKET2];

int graph_value = 0;
int32_t slider_value = 0;
long sm,em,dm;
uint16_t temp,num;

const uint8_t hmiID = 1;  //ID of HMI. The ID need to match, unless program will not work

//Modbus Master class define
Modbus master;  

int16_t calib_HX711(int32_t slideValue)
{
  int16_t gramUnitValue = 0;
  scale.set_scale(slideValue); //Adjust to this calibration factor
  gramUnitValue = (scale.get_units()*0.45359237)*1000;
  print("slideValue: ");
  print(slideValue);
  print(" unit: ");
  print(scale.get_units());
  print(" Reading: ");
  print(gramUnitValue);
  print(" g"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  print(" calibration_factor: ");
  print(slideValue);
  println();

  return gramUnitValue;
}

void setup()
{
  //Config packets and register
  master.configure(packets, NO_OF_PACKET, regs);

  //Config individual packet: (packet, ID, Function, Address, Number of register or data, start register in master register array)
  master.construct(&packets[PACKET1], hmiID, READ_HOLDING_REGISTERS, 0, 6, 0);

  master.construct(&packets[PACKET2], hmiID, PRESET_MULTIPLE_REGISTERS, 100, 9, 6);

  //Start Modbus
  master.begin(&Serial1, BAUD, BYTE_FORMAT, TIMEOUT, POLLING, RETRIES, TxEnablePin);

  Serial.begin(115200);  //debug on serial0

  println("Arduino Modbus Master");
  
  pinMode(13, OUTPUT);

  println("HX711 calibration sketch");
  println("Remove all weight from scale");
  println("After readings begin, place known weight on scale");
  println("Using slider on HMI to find weight of items");

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale();
  scale.tare();	//Reset the scale to 0

  long zero_factor = scale.read_average(); //Get a baseline reading
  print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  println(zero_factor);
}


void loop()
{
  master.update();  //polling

  sm = millis();

  // graph_value++;  //update graph data, just increase from -32768 to 32767 (signed int)
  

  regs[total_packets] = NO_OF_PACKET;             //Total number of packet, here is 2
  // regs[total_requests] = master.total_requests(); //Update all requested packets. Take a look on ModbusXT.h
  regs[total_failed] = master.total_failed();     //Update all failed packet
  regs[graph] = graph_value;  //Update graph value

  //If button is press, turn on HMI's LED
  for (uint8_t i=0;i<3;i++)
  {
    if (regs[i] == 1)      
      regs[i+11] = 1;
    else
      regs[i+11] = 0;
  }

  //If Led green is on (or button 0 = 1) turn on Led on arduino
  if (regs[led_grn])
    digitalWrite(13, HIGH);
  else
    digitalWrite(13, LOW);

  //Print number value on HMI
  if (num != regs[number_entry] )
  {
    num = regs[number_entry];
    print("Number: ");
    println(num);
  }

  //Print slider value on HMI
  if (slider_value != regs[slider] )
  {
    slider_value = regs[slider]*100;
    // print("Slider: ");
    // println(slider_value);
    // regs[total_requests]  = slider_value;
    // calib_HX711(slider_value);
  }

  //update transfer rate and transfer delay
  if ( (sm-dm) > 1000) //update 1s
  {
    dm = sm;
    regs[transfer_rate] = regs[total_requests] - temp;
    temp = regs[total_requests];

    regs[transfer_delay] = (unsigned int) ((NO_OF_PACKET*100000UL)/regs[transfer_rate]);
  }

  graph_value = calib_HX711(slider_value);  //update graph data
}//end loop








