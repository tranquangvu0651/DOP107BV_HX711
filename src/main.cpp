#include "ModbusXT.h"
#include <HX711.h> 
#include <EEPROM.h>

//HX711 class define
HX711 scale;

//Modbus Master class define
Modbus master;  

// Configuration for modbus RTU ----------------------------------------------------------------
#define TIMEOUT 500   //Timeout for a failed packet. Timeout need to larger than polling
#define POLLING 2     //Wait time to next request

#define BAUD        115200  
#define RETRIES     10    //How many time to re-request packet frome slave if request is failed
#define BYTE_FORMAT SERIAL_8E1
#define TxEnablePin 2   //Arduino pin to enable transmission
//-----------------------------------------------------------------------------------------------

// Configuration for HX711 ----------------------------------------------------------------------
#define LOADCELL_DOUT_PIN  3
#define LOADCELL_SCK_PIN  2

const long LOADCELL_OFFSET = 50682624;
const long LOADCELL_DIVIDER = 5895655;

float calibration_factor = -7050;//-1065650.00;  // worked for my 440lb max scale setup
long currentOffset;
long zeroOffset;

#define LB2KG  0.45359237
// #define CALWEIGHT 3.00 // For flexible we using weight sample
#define DEFAULT_CALIFACTOR calibration_factor

int16_t calib_HX711(int32_t factor);
//-----------------------------------------------------------------------------------------------

#define print(x)  Serial.print(x)
#define println(x) Serial.println(x)

uint8_t inState = 0;
uint8_t scaleSetState1=0,scaleSetState2=0;
int graph_value = 0;
int slider_value = 0;
long sm,em,dm;
uint16_t temp,num,num2,num3,num4;
const uint8_t hmiID = 1;  //ID of HMI. The ID need to match, unless program will not work
uint8_t STATE=0;
int32_t total_scale_1,total_scale_2;
int32_t set_scale_1,set_scale_2;
uint32_t set_weight_sample_1,set_weight_sample_2;

enum {
  button_zero_1,
  button_zero_2,
  button_calib_1,
  button_calib_2,

  set_entry_1,
  set_entry_2,
  weight_sample_1,
  weight_sample_2,

  scale_1, //=8
  scale_2,
  led_grn_home_1,
  led_blue_home_1,
  led_red_home_1,
  led_grn_home_2,
  led_blue_home_2,
  led_red_home_2,

  led_grn_set_1, //=16
  led_blue_set_1,
  led_red_set_1,
  led_grn_set_2,
  led_blue_set_2,
  led_red_set_2,
  TOTAL_REGS
};

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum {
  PACKET1,
  PACKET2,
  PACKET3,
  PACKET4,
  NO_OF_PACKET
};

// Masters register array
int32_t regs[TOTAL_REGS];

//Modbus packet
Packet packets[NO_OF_PACKET];

/* Chương trình hiển thị giá trị ra HMI 
 * regs[scale_1]: giá trị hiển thị cân số 1 trên HMI
 * regs[scale_2]: giá trị hiển thị cân số 2 trên HMI
 * 
*///-----------------------------------------------
void display_scale(int32_t scale1, int32_t scale2)
{
    regs[scale_1] = scale1;
    regs[scale_2] = scale2;
}

/* Chương trình kiểm tra cài đặt g sample
 * Khi có cài đặt sample trên HMI thành công
 * chương trình sẽ báo số g được cài đặt trên serial
*///-----------------------------------------------
void check_set_weight_scale()
{
  if (set_weight_sample_1 != regs[weight_sample_1] )
  {
    set_weight_sample_1 = regs[weight_sample_1];
    print("Weight sample 1: ");
    println(set_weight_sample_1);
  }

  if (set_weight_sample_2 != regs[weight_sample_2] )
  {
    set_weight_sample_2 = regs[weight_sample_2];
    print("Weight sample 2: ");
    println(set_weight_sample_2);
  }
}

/* Chương trình điều khiển LED trang Home
 * scaleNo: 1 hoặc 2 tương ứng với cụm LED trên Home
 * greenLed,blueLed,redLed: Các led 1 sáng 0 tắt
*///-----------------------------------------------
void set_led_home(uint8_t scaleNo, bool greenLed,bool blueLed,bool redLed)
{
  switch(scaleNo)
  {
    // Scale 1
    case 1:
          regs[led_grn_home_1] = greenLed;
          regs[led_blue_home_1] = blueLed;
          regs[led_red_home_1] = redLed;
    break;

    // Scale 2
    case 2:
        regs[led_grn_home_2] = greenLed;
        regs[led_blue_home_2] = blueLed;
        regs[led_red_home_2] = redLed;
    break;
  }
}

/* Chương trình điều khiển LED trang Setting
 * scaleNo: 1 hoặc 2 tương ứng với cụm LED trên Setting
 * greenLed,blueLed,redLed: Các led 1 sáng 0 tắt
*///-----------------------------------------------
void set_led_setting(uint8_t scaleNo, bool greenLed,bool blueLed,bool redLed)
{
  switch(scaleNo)
  {
    // Scale 1
    case 1:
          regs[led_grn_set_1] = greenLed;
          regs[led_blue_set_1] = blueLed;
          regs[led_red_set_1] = redLed;
    break;

    // Scale 2
    case 2:
        regs[led_grn_set_2] = greenLed;
        regs[led_blue_set_2] = blueLed;
        regs[led_red_set_2] = redLed;
    break;
  }
}

/* Chương trình kiểm tra cài đặt g cảnh báo
 * có thể viết thêm tuỳ mục đích
 * 
*///-----------------------------------------------
void check_set_scale()
{
  if ((set_scale_1 != regs[set_entry_1]) && !inState)
  {
    set_scale_1 = regs[set_entry_1];
    print("Set scale 1: ");
    println(set_scale_1);
    scaleSetState1 = 1;
  }
  else if(scaleSetState1&&set_scale_1>0)
  {
    if(regs[total_scale_1] >= set_scale_1)
    {
      set_led_home(1,0,1,0);
      // scaleSetState1=0;
    }
    else
    {
      set_led_home(1,0,0,0);
    }
  }




  if ((set_scale_2 != regs[set_entry_2]) && !inState)
  {
    set_scale_2 = regs[set_entry_2];
    print("Set scale 2: ");
    println(set_scale_2);
    scaleSetState2 = 1;
  }
  else if(scaleSetState2&&set_scale_2>0)
  {

  }

}

/* Chương trình set điểm Zero của cân
 * Có thể viết thêm button 2 khi có cơ cấu cân 2
 * Khi bấm button 1 nó sẽ set cân về 0
*///-----------------------------------------------
void check_button_home()
{
  // println(inState);
  if((regs[button_zero_1]==1)&&(!inState))
  {
    inState=1;
    println("Button zero 1 push");
    // set regs[scale1] to 0
    scale.tare();	//Reset the scale to 0
    set_led_home(1,0,1,1);
    delay(1000);
    set_led_home(1,1,0,0);
    delay(1000);
  }
  else if(!regs[button_zero_1])
  {
    regs[button_zero_1]=0;
    set_led_home(1,0,0,0);
    delay(1000);
    inState=0;
  }
   
  if((regs[button_zero_2]==1)&&(!inState))
  {
    inState=true;
    println("Button zero 2 push");
    // set regs[scale1] to 0
   
    inState=false;
  }
}

/* Chương trình cân chỉnh cân dựa trên g sample
 * Khi bấm nút yêu cầu calib lại cân theo g sample
 * B1: Bỏ vật liệu cân chỉnh lên cơ cấu
 * B2: Cài đặt số g của vật liệu trang Setting
 * B3: Bấm nút calib
 * B4: Các Led sẽ sáng theo thứ tự Red - Blu - Gre
 * B5: Khi các Led tắt hết là quá trình cân đã hoàn thành
 * B6: Quay lại trang Home để thử
*///-----------------------------------------------
void check_button_setting()
{ 
  float data;
  float prevData;
  int32_t CALWEIGHT;
  bool done=false;
  int8_t direction = 1;
  uint8_t flipDirCount = 0;
  int32_t dirScale = 1000;
  float calib=0;

  if((regs[button_calib_1]==1)&&(!inState))
  {
    set_led_setting(1,0,0,1);
    noInterrupts();                       // disable all interrupts
    master.update();
  
    inState=true;
  
    CALWEIGHT = regs[weight_sample_1];//g
    data = abs(scale.get_units()*1000); // ->g
    prevData = data;
    println("CALWEIGHT " + String(CALWEIGHT) + "g");

    currentOffset = scale.read_average();

    println("scale.read_average() = " + String(currentOffset));
    println("zeroOffset = " + String(zeroOffset));

    calib = (LB2KG*(scale.read_average()-zeroOffset))/(CALWEIGHT);

    println("calib = " + String(calib));
    calibration_factor = calib*1000;
    scale.set_scale(calibration_factor / LB2KG);
    while (!done)
    {
      // get data
      data = abs(scale.get_units()*1000);

      println("data = " + String(data, 2));
      println("abs = " + String(abs(data - CALWEIGHT), 4));
      println("calibration_factor = " + String(calibration_factor));

      // if not match
      if (abs(data - CALWEIGHT) >= 0.01)
      {
        if (abs(data - CALWEIGHT) <= abs(prevData - CALWEIGHT) && direction != 1 && data < CALWEIGHT)
        {
          direction = 1;
          flipDirCount++;
        }
        else if (abs(data - CALWEIGHT) > abs(prevData - CALWEIGHT) && direction != -1 && data > CALWEIGHT)
        {
          direction = -1;
          flipDirCount++;
        }

        if (flipDirCount > 2)
        {
          if (dirScale != 1)
          {
            dirScale = dirScale / 10;
            flipDirCount = 0;
            println("dirScale = " + String(dirScale));
          }
        }
        println("direction = " + String(direction));
        println("dirScale = " + String(dirScale));
        // println("calibration_factor = " + String(calibration_factor));
        // set new factor 
        calibration_factor += direction * dirScale;
        scale.set_scale(calibration_factor / LB2KG);
        
        prevData = data;
      }
      // if match
      else
      {
        interrupts();                         // enable all interrupts
        // print("Read averaget: "); 
        // println(scale.read_average());
        currentOffset  = scale.read_average();
        println("----------------------------------------------------------------------");
        println("NEW currentOffset = " + String(currentOffset));
        println("NEW calibration_factor = " + String(calibration_factor));
        println("NEW calibration_factor / LB2KG = " + String(calibration_factor / LB2KG));
        println("----------------------------------------------------------------------");
        // EEPROM.put(0x00,0x01); // set init
        // EEPROM.put(0x01,currentOffset);
        // EEPROM.put(0x01+sizeof(long),calibration_factor);  
        done = true;
        // lcd.clear();
      }
    // [ ] Điều chỉnh temp value cần nạp scale dựa trên + - số g hiển thị trên scale đó
    // [ ] set lại giá trị __factor

  }
  
  println("----------------------------------------------------------------------");
  set_led_setting(1,1,0,0);
  delay(1000);
  // else if(!regs[button_calib_1])
  // {
  //   inState=0;
  //   set_led_setting(1,0,0,0);
  // }
  
  }
  else if(!regs[button_calib_1])
  {
    regs[button_calib_1]=0;
    set_led_setting(1,0,0,0);
    delay(1000);
    inState=0;
  }
}

/* Chương trình đọc giá trị HX711 chuyển thành g
 * 
 * 
*///-----------------------------------------------
int16_t calib_HX711(int32_t factor)
{
  int16_t gramUnitValue = 0;
  // scale.set_scale(factor/(LB2KG)); //Adjust to this calibration factor
  gramUnitValue = (scale.get_units())*1000;
  // print("slideValue: ");
  // print(slideValue);
  print(" unit: ");
  print(scale.get_units());
  print(" Reading: ");
  print(gramUnitValue);
  print(" g"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  // print(" Read average: ");
  // print(scale.read_average());
  println();
  return gramUnitValue<0?0:gramUnitValue;
}

/* Ngắt timer 1 200ms
 * Công thức trong setup
 * 
*///-----------------------------------------------
ISR (TIMER1_OVF_vect) 
{
  TCNT1 = 62410; 
  master.update();  //polling
}

void setup()
{

  // eeprom có thể code thêm để tự lưu trong eeprom
  // if (EEPROM.read(0x00) != 0x01) 
  // {
  //   println("NOT INIT !!!!");
  //   currentOffset = 0;
  //   calibration_factor = DEFAULT_CALIFACTOR;     
  //   // show instructions
  //   //wait for button press
  //   while (digitalRead(button));
  // }
  // else
  // {
  //   EEPROM.get(0x01,currentOffset);
  //   EEPROM.get(0x01+sizeof(long),calibration_factor);   
  //   println("currentOffset = " + String(currentOffset));
  //   println("calibration_factor = " + String(calibration_factor));
  // }

  //Config packets and register
  master.configure(packets, NO_OF_PACKET, regs);

  //Config individual packet: (packet, ID, Function, Address, Number of register or data, start register in master register array)
  master.construct(&packets[PACKET1], hmiID, READ_HOLDING_REGISTERS, 200, 8, 0);
  master.construct(&packets[PACKET2], hmiID, PRESET_MULTIPLE_REGISTERS, 300, 8, 8);
  master.construct(&packets[PACKET3], hmiID, PRESET_MULTIPLE_REGISTERS, 410, 6, 16);

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
  scale.set_scale(calibration_factor/LB2KG);
  scale.tare();	//Reset the scale to 0
  zeroOffset = scale.get_offset();
  print("Current offset: "); 
  println(currentOffset);
  long zero_factor = scale.read_average(); //Get a baseline reading
  print("Zero factor: ");
  println(zero_factor);

  //------------ Timer configuration ---------------------------------------------
  //
  //  To calculate preloader value for timer1 for time of 0.2 Sec (200ms):
  //  TCNT1 = 65535 – (16Mx(0.2) / 62410) = 34285 (16M: 16.000.000)
  //
  //------------------------------------------------------------------------------

  noInterrupts();                       // disable all interrupts
  
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 62410;                        // preload timer
  TCCR1B |= (1 << CS10)|(1 << CS12);    // 1024 prescaler 
  TIMSK1 |= (1 << TOIE1);               // enable timer overflow interrupt ISR
  interrupts();                         // enable all interrupts
  //--------------------------------------------------------------------------------

  // Reset Led when mega reset
  regs[led_grn_home_1] = 0; 
  regs[led_grn_home_2] = 0; 
  regs[led_grn_set_1] = 0; 
  regs[led_grn_set_2] = 0; 
  
}

void loop()
{
  sm = millis();

  //update transfer rate and transfer delay
  if ( (sm-dm) > 1000) //update 1s
  {
    dm = sm;
    total_scale_1 = calib_HX711(calibration_factor);  //update graph data
  }

  switch(STATE){
   
   case 0:
          display_scale(total_scale_1,total_scale_2);
   break;

   case 1:
          check_set_scale();
   break;
   
   case 2:
          check_set_weight_scale();
   break;
   
   case 3:
          check_button_home();
   break;

   case 4:
          check_button_setting();
   break;
   
   default:
   
   break;
    
  }

  STATE++;
}//end loop





