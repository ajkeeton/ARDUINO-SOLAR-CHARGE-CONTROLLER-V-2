
/* Arduino PWM charge controller
   This is a partial re-write of https://github.com/deba168/ARDUINO-SOLAR-CHARGE-CONTROLLER-V-2
   Thanks for your work, deba168

   Contains fixes for newer LiquidCrystal_I2C.h, some tweaks that seem 
   important (like preventing bouncing the load on and off, clamping the charge
   voltage, etc), and modifcations relevant to my LiFePO4 setup. The changes 
   apply to LiPOs as well, but the BAT_MAX, BAT_FLOAT, LOAD_OFF, and LOAD_ON 
   need to be changed appropriately.

   - Load should no longer bounce on and off when battery is near the 
     cut-off threshold
   - Charging is ramped up
   - Max charge voltage is clamped
   - Output load is /partially/ regulated
   - Load control is no longer affected by solar voltage (especially useful for 
     portable setups where the panels may be disconnected entirely)
   - Hacks for XL6009 boost controller
   - No longer tries to determine if this is a 6 volt or 12 volt setup. Hardcoded.

    NOTE: much like the original code, your display may use a different I2C 
    address. That address must be provided to the LiquidCrystal_I2C 
    constructor.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SOL_ADC     A0     // Solar panel side voltage divider is connected to pin A0 
#define BAT_ADC     A1     // Battery side voltage divider is connected to pin A1
#define CURRENT_ADC A2  // ACS 712 current sensor is connected to pin A2
#define TEMP_ADC    A3   // LM 35 Temperature is connected to pin A3
#define AVG_NUM     10    // number of iterations of the adc routine to average the adc readings
#define SERIAL_UPDATE_DELTA (0.5 * 1000)

#define SOLAR_MAX   14.4
#define BAT_MAX     14.4
#define BAT_FLOAT   14
#define BAT_WTFINAL 13.3
#define LOAD_OFF    12.97   // Kill the load if the battery voltage falls to this level
#define LOAD_ON     12.99   // Turn the load back on once the battery voltage exceeds this level

#define PIN_PWM     3   // Charging MOSFET
#define PIN_LOAD    2   // Load MOSFET 
#define BAT_RED_LED     5
#define BAT_GREEN_LED   6
#define BAT_BLUE_LED    7
#define LOAD_RED_LED    8
#define LOAD_GREEN_LED  9

enum {
    CHARGING_OFF,
    CHARGING_BULK,
    CHARGING_FLOAT,
    CHARGING_CLAMPED,
};

byte solar[8] = //icon for solar panel
{
  0b11111,0b10101,0b11111,0b10101,0b11111,0b10101,0b11111,0b00000
};
byte battery[8] =  //icon for battery
{
  0b01110,0b11011,0b10001,0b10001,0b10001,0b10001,0b10001,0b11111
};

byte energy[8] =  // icon for power
{
  0b00010,0b00100,0b01000,0b11111,0b00010,0b00100,0b01000,0b00000
};
/*byte alarm[8] =  // icon for alarm
{
 0b00000,0b00100,0b01110,0b01110,0b01110,0b11111,0b00000,0b00100
};*/
byte temp[8] = //icon for termometer
{
 0b00100,0b01010,0b01010,0b01110,0b01110,0b11111,0b11111,0b01110
};

byte charge[8] = // icon for battery charge
{
  0b01010,0b11111,0b10001,0b10001,0b10001,0b01110,0b00100,0b00100,
};
byte not_charge[8]=
{
  0b00000,0b10001,0b01010,0b00100,0b01010,0b10001,0b00000,0b00000,
};

float solar_volt = 0;
float bat_volt =   0;
float load_current = 0;
int temperature = 0;
float bulk_charge_sp = 0;
float float_charge_sp = 0;
int charge_status = 0;
float load_status = 0;
float error=0;
float Ep=0;
int duty = 0;
float elasped_msec=0;
float elasped_time=0;
float ampSecs = 0;
float ampHours=0;
float watts=0;
float wattSecs = 0;
float wattHours=0;

// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address // In my case 0x27

void setup()
{
    Serial.begin(9600);

    pinMode(BAT_RED_LED, OUTPUT);
    pinMode(BAT_GREEN_LED, OUTPUT);
    pinMode(BAT_BLUE_LED, OUTPUT);
    pinMode(LOAD_RED_LED, OUTPUT);
    pinMode(LOAD_GREEN_LED, OUTPUT);
    pinMode(PIN_PWM, OUTPUT);
    pinMode(PIN_LOAD, OUTPUT);
    digitalWrite(PIN_PWM, LOW);  // Default value of pwm duty cycle
    digitalWrite(PIN_LOAD, LOW); // Default load state is OFF

    lcd.begin(20,4);    // Initialize the lcd for 16 chars 2 lines, turn on backlight
    lcd.noBacklight();    // Finish with backlight on  
    lcd.createChar(1,solar);
    lcd.createChar(2, battery);
    lcd.createChar(3, energy);
    //lcd.createChar(4,alarm);
    lcd.createChar(5,temp);
    lcd.createChar(6,charge);
    lcd.createChar(7,not_charge);
    lcd.clear();
}

void loop()
{
    read_data();             // read different sensors data from analog pin of arduino
    setpoint();       // decide the charge set point according to system voltage
    charge_set();         // pwm charging of battery
    power();                // calculate the load power and energy
    load_control();         //control the load
    //led_indication();       // led indicator XXX

    static unsigned long last_update_time = 0;
    unsigned long current_time = millis();

    if(last_update_time > current_time) {
      // We wrapped around
      last_update_time = 0;
    }

    /* Throttle serial output */

    if(current_time - last_update_time < SERIAL_UPDATE_DELTA) {
      delay(10);
      return;
    }

    //print_serial();
    print_brief_serial();
    print_lcd();
}

void print_brief_serial() 
{
    Serial.print("Solar: "); Serial.print(solar_volt);
    Serial.print(" Bat: "); Serial.print(bat_volt);
    Serial.print(" Charge stat: "); Serial.print(charge_status);
    Serial.print(" Duty: "); Serial.print(duty);
    Serial.print(" Load: ");
    Serial.println(load_status);
}

int read_adc(int adc_parameter)
{
    int sum = 0;
    int sample = 0;

    for (int i=0; i<AVG_NUM; i++) 
    {                                      
        sample = analogRead(adc_parameter);
        sum += sample;                    
        delayMicroseconds(50);            
    }

    return sum / AVG_NUM;
}

void read_data(void) 
{
    // 5V = ADC value 1024 => 1 ADC value = (5/1024)Volt= 0.0048828Volt
    // Vout=Vin*R2/(R1+R2) => Vin = Vout*(R1+R2)/R2   R1=100 and R2=20

    solar_volt = read_adc(SOL_ADC) * 0.00488 * (120/20);
    bat_volt   = read_adc(BAT_ADC) * 0.00488 * (120/20);       
    // load_current = read_adc(CURRENT_ADC) * 0.0488 - 25; 
    // temperature = read_adc(TEMP_ADC) * 0.00488*100;
    temperature = 25;
}

void power(void)
{
    uint32_t msec = millis();
    static uint32_t last_msec = 0;

    elasped_msec = msec - last_msec; //Calculate how long has past since last call of this function
    elasped_time = elasped_msec / 1000.0; // 1sec=1000 msec
    watts = load_current * bat_volt; //Watts now
    ampSecs = (load_current*elasped_time); //AmpSecs since last measurement
    wattSecs = ampSecs * bat_volt; //WattSecs since last measurement
    ampHours = ampHours + ampSecs/3600; // 1 hour=3600sec //Total ampHours since program started
    wattHours = wattHours + wattSecs/3600; // 1 hour=3600sec //Total wattHours since program started

    last_msec = msec; //Store 'now' for next time
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////PRINT DATA IN SERIAL MONITOR/////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void print_serial(void) 
{
    delay(100);
    Serial.print("Solar Panel Voltage: ");
    Serial.print(solar_volt);
    Serial.println("V");
    Serial.print("Battery Voltage: ");
    Serial.print(bat_volt);
    Serial.println("V");
    Serial.print("Syestem Voltage: ");
    Serial.print(BAT_WTFINAL);
    Serial.println("V");
    Serial.print("Charge Set Point:");
    Serial.println(bulk_charge_sp);
    Serial.print("Temperature:");
    Serial.print(temperature);
    Serial.println("C");
    Serial.print("Load Current: ");
    Serial.print(load_current);
    Serial.println("A");
    Serial.print("Power: ");
    Serial.print(watts);
    Serial.println("W");
    Serial.print("Energy: ");
    Serial.print(wattHours);
    Serial.println("WH");
    Serial.print("Duty Cycle :");

    if (charge_status==CHARGING_BULK)
    {
        Serial.println("99%");
        Serial.println("BULK CHARGING");
    }
    else if (charge_status==CHARGING_FLOAT)
    {
        Serial.print(Ep);
        Serial.println("%");
        Serial.println("FLOAT CHARGING");
    }
    else if (charge_status==CHARGING_CLAMPED)
    {
        Serial.print(duty);
        Serial.println("CLAMPED CHARGING");
    }
    else
    {
        Serial.println("0%");
        Serial.println("NOT CHARGING");
    }
    if(load_status==1)
    {
        Serial.println("LOAD IS CONNECTED"); 
    }
    else
    {
        Serial.println("LOAD IS DISCONNECTED");   
    }
   
    Serial.println("***************************");
 }

void setpoint(void)
{
    static int temp_change = temperature - 25.0; // 25deg cel is taken as standard room temperature 

 // temperature compensation = -5mv/degC/Cell 
  // If temperature is above the room temp ;Charge set point should reduced
  // If temperature is bellow the room temp ;Charge set point should increased

     bulk_charge_sp = BAT_MAX - (0.030*temp_change);
     float_charge_sp = BAT_FLOAT - (0.030*temp_change);
}

void duty_set_bulk()
{
    float bulk_dest_pwm = 252.45;

  // Helps prevent bouncing, but much more importantly, gives my XL6009 a chance 
  // to start! When the load is immediately pulled the boost converter seems to
  // brown out and it does not recover.

    if(duty > bulk_dest_pwm) {
        charge_status = CHARGING_BULK; // indicate the charger is in BULK mode
        duty = bulk_dest_pwm;
    }
     
    if (solar_volt > SOLAR_MAX) 
    {
        /* Don't hit the battery with too much voltage! 
           Only increase PWM until we read SOLAR_MAX on the voltage divider for
           the battery.
        */
        if(bat_volt >= BAT_MAX) {
            duty -= 1;
            charge_status = CHARGING_CLAMPED;
        }
        else {
            charge_status = CHARGING_BULK;
        }
    }
    else if (charge_status != CHARGING_CLAMPED) {
        charge_status = CHARGING_BULK; // indicate the charger is in BULK mode
        duty += 0.5;
    }
}

void duty_set_float()
{
    charge_status = CHARGING_FLOAT; // indicate the charger is in FLOAT mode
    error = (bulk_charge_sp - bat_volt);      // duty cycle reduced when the battery voltage approaches the charge set point
    Ep = error * 100 ; //Ep= error* Kp // Assume  Kp=100
  
    if(Ep < 0) 
    { 
        Ep = 0;
    }
    else if(Ep>100) 
    {
        Ep = 100;
    }
    else if(Ep > 0 && Ep <= 100) // regulating
    {
        float target = Ep * 255 / 100;

        if (duty >= target)
        {
            duty = target;
        }
        else 
        {
            duty += 0.5;
        }
    }
}

void duty_set_off()
{
    charge_status = CHARGING_OFF;
    duty = 0;
}

void charge_set(void)
{
// XXX Double check what happens if solar voltage is way higher
// based on this logic, it looks like we won't charge because
// the voltage divider for the battery is going to be greater than
// BAT_MAX

// Old code:
//    if (solar_volt > bat_volt && bat_volt <= BAT_MAX)
// Proposed: use 'ramp_duty' to only ramp until bat_max
    if (solar_volt > bat_volt)
    {
        if (bat_volt <= float_charge_sp) // charging start
        {
            duty_set_bulk();
        }
        else if (bat_volt > float_charge_sp && bat_volt <= bulk_charge_sp)
        {   
            duty_set_float();
        }
    }
    else
    {
        duty_set_off();
    }

    analogWrite(PIN_PWM, duty);
}
 
void load_control()
{
    /* Using a separate constant for load off and load on prevent flapping the
       load on off when things are bouncing around the threshold. */
    if(bat_volt <= LOAD_OFF) {
        load_status = 0;
        digitalWrite(PIN_LOAD, LOW);
    }
    else if(bat_volt > LOAD_ON) {
        load_status = 1;

        // XXX TODO: measure load voltage at different PWM values and battery 
        // voltages and work out formula for regulating load voltage
        digitalWrite(PIN_LOAD, HIGH);
    }
}

void led_indication(void)
{
    /* Battery LEDs */
    if( (bat_volt > BAT_WTFINAL) && ( bat_volt <bulk_charge_sp))
    {   
        leds_off_all();
        digitalWrite(BAT_GREEN_LED,LOW);  // battery voltage is healthy
    } 
    else if(bat_volt >= bulk_charge_sp) 
    {
        leds_off_all();
        digitalWrite(BAT_BLUE_LED,LOW);  //battery is fully charged
    }
    else if(bat_volt < BAT_WTFINAL)
    {
        leds_off_all();
        digitalWrite(BAT_RED_LED,LOW);  // battery voltage low
    }
  
    /* Load LEDs */
    if(load_status==1)
    {
        digitalWrite(LOAD_GREEN_LED,HIGH);
    }
    else if(load_status==0)
    {
        digitalWrite(LOAD_RED_LED,HIGH);
    }
}

void leds_off_all(void)
{
    digitalWrite(BAT_RED_LED, HIGH);
    digitalWrite(BAT_GREEN_LED, HIGH);
    digitalWrite(BAT_BLUE_LED, HIGH);
    digitalWrite(LOAD_RED_LED, LOW);
    digitalWrite(LOAD_GREEN_LED, LOW);
}

void print_lcd()
{
    lcd.setCursor(0, 0);
    lcd.write(1);
    lcd.setCursor(2, 0);
    lcd.print(solar_volt);
    lcd.print("V");
    lcd.setCursor(14, 0);
    lcd.write(5);
    lcd.setCursor(16, 0);
    lcd.print(temperature);
    lcd.write(0b11011111);
    lcd.print("C");
    lcd.setCursor(0,1);
    lcd.write(2);
    lcd.setCursor(2, 1);
    lcd.print(bat_volt);
    lcd.print("V");
    lcd.setCursor(14, 1);
    lcd.write(2);

    if( (charge_status==CHARGING_FLOAT) || 
        (charge_status== CHARGING_BULK) || 
        (charge_status == CHARGING_CLAMPED) )
    {
        lcd.write(6);
    }
    else
    {
        lcd.write(7);
    }

    lcd.setCursor(0,2);
    lcd.write(3);
    lcd.setCursor(2,2);
    lcd.print(load_current); 
    lcd.print("A");
    lcd.setCursor(13,2); 
    lcd.print(watts); 
    lcd.print("W"); 
    lcd.setCursor(0,3);
    lcd.print("Energy:"); 
    lcd.print(wattHours);
    lcd.print("WH"); 
}
