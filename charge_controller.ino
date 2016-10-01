/* Arduino PWM charge controller

   This is a re-write of https://github.com/deba168/ARDUINO-SOLAR-CHARGE-CONTROLLER-V-2
   Thanks for your work, deba168.

    XXX Use this at your own risk. I make no guarantees that you won't break or
        injure things, including yourself

    XXX You need to set up a battery profile for your setup. See:
        #define DEF_BAT_PROFILE and set_battery_profile 

    XXX Current needs to be measured going in and going out, partially for 
        safety of the batteries, but also to determine when they're full 
        (voltage alone doesn't tell you when a LiIon is full)

    XXX If you're using the LEDs from the original instructable, you'll need to
        uncomment "#define USING_LEDS"

    XXX If you're using the ACS712, 0-5 amp version, comment out 
        #define NO_CURRENT. If it's the 0-30 amp version, comment out the 
        #define and change the ACS_712_MULT appropriately. Also, good luck.

   - Added "battery profiles", to support different battery packs, capacities,
     chemistries, etc.
   - Load no longers bounce on and off when battery is near the 
     cut-off threshold.
   - Sensor values are placed in bins and the median is used
   - Charging is ramped up and tapered down
   - Max charge voltage is clamped to help protect lithium batteries
   - Load control is no longer affected by solar voltage (especially useful for
     portable setups where the panels may be disconnected entirely)
   - Serial output is abbreviated 
   - No longer tries to determine if this is a 6 volt or 12 volt setup. Hardcoded.
   - Contains fixes for newer LiquidCrystal_I2C.h
   - TODO: Regulate output load voltage
   - TODO: temperature calcs disabled

    NOTE: much like the original code, your display may use a different I2C 
    address. That address must be provided to the LiquidCrystal_I2C 
    constructor.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SOL_ADC     A0   // Solar panel side voltage divider is connected to pin A0 
#define BAT_ADC     A1   // Battery side voltage divider is connected to pin A1
#define LOAD_CURRENT_ADC A2 
#define SOLAR_CURRENT_ADC A6
#define TEMP_ADC    A3   // LM 35 Temperature is connected to pin A3

// #define USING_LEDS
#define NO_CURRENT

#define ACS_712_MULT 0.185   // Depends on which ACS 712 you're using - mine is for 0 to 5 amps

#define PIN_PWM     3   // Charging MOSFET
#define PIN_LOAD    2   // Load MOSFET 

#define BAT_RED_LED     5
#define BAT_GREEN_LED   6
#define BAT_BLUE_LED    7
#define LOAD_RED_LED    8
#define LOAD_GREEN_LED  9

#define AVG_NUM     8   // number of iterations of the adc routine to average the adc readings
#define NUM_BINS    8    // number of iterations of the adc routine to average the adc readings
#define SERIAL_UPDATE_DELTA (50)

#define VOLTAGE_DIV_R1 100
#define VOLTAGE_DIV_R2 22

// 5V = ADC value 1024 => 1 ADC value = (5/1024)Volt= 0.0048828Volt
// Vout=Vin*R2/(R1+R2) => Vin = Vout*(R1+R2)/R2   R1=100 and R2=20
// Scale this value to calibrate voltage sensors. In my case, 
// when my Arduino is plugged up to serial, this number needs to be lower :/
// I suppose logic levels are no longer a true 5 volts.

#define ADC_MULTIPLIER 0.004895 // Found this to be more accurate than 0.004883
//#define ADC_MULTIPLIER 0.00463  // My adjustment for when using battery + serial

enum BAT_PROFILE_t {
    ALM_12V7,
    LION_2S,
    TEST
};

#define DEF_BAT_PROFILE TEST

struct bat_profile_t {
    float charge_max,
        // Max allowed input current
        current_in_max,
        // Max level to charge batteries to
        bat_full_volt,
        // Current_in_at_full determines the point the current must drop to 
        // declare the battery charged
        current_in_at_full,
        // We stop charging when full, but don't resume charging until we drop
        // to this level. Necessary to prevent overcharging lithium batteries.
        bat_recharge,
        // Point where we switch between bulk and float charging
        bat_float_thold, 
        // Minimum voltage needed to switch the load on. Should be higher than
        // load_off to prevent bouncing and to give us a little room to recharge
        load_on,
        // Voltage level where we switch the load off 
        load_off,

        // Good ole' PID loop. If you're not familiar with these...  good luck! 
        // (you're not alone.)
        P, I, D;
};

bat_profile_t profile;

enum {
    CHARGING_OFF,
    CHARGING_BULK,
    CHARGING_FLOAT,
    CHARGING_CLAMPED,
    CHARGING_DONE,
};

byte solar[8] = //icon for solar panel
{
  0b11111,0b10101,0b11111,0b10101,0b11111,0b10101,0b11111,0b00000
};
byte battery[8] =  //icon for battery
{
  0b01110,0b11011,0b10001,0b10001,0b10001,0b10001,0b10001,0b11111
};

byte energy[8] =  // icon for update_calcs
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

double solar_volt = 0;
double bat_volt =   0;
double solar_current = 0;
double load_current = 0;
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
float watts_in=0;
float wattSecs = 0;
float wattHours=0;

int solar_buf[NUM_BINS];
int bat_buf[NUM_BINS];
int solar_current_buf[NUM_BINS];
int load_current_buf[NUM_BINS];

// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address // In my case 0x27

void init_bin(int *bins) 
{
    for (int i=0; i<NUM_BINS; i++) 
        bins[i] = 1025;
}

void setup()
{
    Serial.begin(9600);

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

    init_bin(solar_buf);
    init_bin(bat_buf);
    init_bin(solar_current_buf);
    init_bin(load_current_buf);

    set_battery_profile(DEF_BAT_PROFILE);
}

void set_battery_profile(BAT_PROFILE_t prof) 
{

    // http://batteryuniversity.com/learn/article/charging_lithium_ion_batteries
    // For Li-ion, "Full charge is reached when the current decreases to 
    // between 3 and 5 percent of the Ah rating."

    switch(prof) {
        case ALM_12V7:
            profile.charge_max = 14.35; // Maximum level to set charging voltage to
            profile.current_in_max = 5;
            profile.current_in_at_full = 0.2;
            profile.bat_full_volt = 14; // Maximum level to charge battery to
            profile.bat_recharge = 13.9; // We stop charging when full. When we hit this point, recharge 
            profile.bat_float_thold = 13.8; // Transition point between bulk and float
            profile.load_off = 12.97;
            profile.load_on = 12.99;

            profile.P = 1;
            profile.I = 1;
            profile.D = 1;
            break;

        case LION_2S:
            profile.charge_max = 4.2*2; 
            profile.current_in_max = 1; 
            profile.current_in_at_full = 0.1; // Should be 5-10% of 1 C.
            profile.bat_recharge = 4.0*2; 
            profile.bat_full_volt = 4.2*2; 
            profile.bat_float_thold = 3.8*2; 
            profile.load_off = 3.1*2;
            profile.load_on = 3.25*2;

            profile.P = 1;
            profile.I = 1;
            profile.D = 1;
            break;

        case TEST:
            profile.charge_max = 10;
            profile.current_in_max = 2; 
            profile.current_in_at_full = 1;
            profile.bat_recharge = 7;
            profile.bat_full_volt = 9;
            profile.bat_float_thold = 5;
            profile.load_off = 1;
            profile.load_on = 3;

            profile.P = 5;
            profile.I = .1;
            profile.D = .01;
            break;
    } 
}

void test() 
{
    if (Serial.available() > 0) {
        // read the incoming byte:
        int b = Serial.read();

        if(b == 'b') {
            bat_volt = Serial.parseInt();
            
            Serial.print("Battery voltage set to: ");
            Serial.println(bat_volt);
        } else if(b == 's') {
            solar_volt = Serial.parseInt();
            Serial.print("Solar voltage set to: ");
            Serial.println(bat_volt);
        }
     }
}

void loop()
{
    delay(1);

    #if DEF_BAT_PROFILE == TEST
    test();
    #else
    read_data();
    #endif
    set_charge_points();
    charge_set();
    update_calcs();
    set_load_control();

    #ifdef USING_LEDS
    led_indication();
    #endif

    static unsigned long last_update_time = 0;
    uint32_t current_time = millis();

    /* Throttle serial output */

    if(last_update_time > current_time) {
        // We wrapped around
        last_update_time = 0;
    }

    if(current_time - last_update_time < SERIAL_UPDATE_DELTA) {
        return;
    }

    last_update_time = current_time;

    //print_serial();
    print_brief_serial();
    print_lcd();
}

void print_brief_serial() 
{
    Serial.print("Solar: "); Serial.print(solar_volt);
    Serial.print("v "); Serial.print(solar_current);
    Serial.print("A Bat: "); Serial.print(bat_volt);
    Serial.print("v Charge stat: "); print_charge_status(); // Serial.print(charge_status);
    Serial.print(" Duty: "); Serial.print(duty);
    Serial.print(" Load: ");
    Serial.print(load_status);
    Serial.print(" Temp: ");
    Serial.println(temperature);
}

void print_charge_status()
{
    switch(charge_status) {
        case CHARGING_OFF:
            Serial.print("off");
            break;
        case CHARGING_BULK:
            Serial.print("bulk");
            break;
        case CHARGING_FLOAT:
            Serial.print("float");
            break;
        case CHARGING_CLAMPED:
            Serial.print("clamped");
            break;
        case CHARGING_DONE:
            Serial.print("full");
            break;
        default:
            Serial.print("you added a new charging state, didn't you?");
    }
}

void print_bins(int num_bins, int *bins) {
    Serial.print("Bins: ");
    for (int i=0; i<num_bins; i++) {
        Serial.print(" "); Serial.print(bins[i]);
    }
    Serial.println();
}

void insert(int val, int num_bins, int *bins, int idx)
{
    if(idx == num_bins - 1) {
        bins[idx] = val;
        return;
    }

    for(int i=num_bins-1; i > idx; i--) {
        // Serial.print("i: "); Serial.print(i); Serial.print(" "); Serial.println(bins[i]);
        bins[i] = bins[i-1];
    }

    bins[idx] = val;
}

/*
double read_adc(int num_bins, int adc_parameter)
{
    int bins[num_bins];

    for (int i=0; i<num_bins; i++) 
        bins[i] = 1025;

    for (int i=0; i<num_bins; i++) {
        int cur = analogRead(adc_parameter);

        for(int j=0; j<num_bins; j++) {
            if(cur < bins[j]) {
                insert(cur, num_bins, bins, j);
                break;
            }
        }
        delay(1);
    }

    // print_bins(num_bins, bins);

    return bins[num_bins/2];
}
*/

void bin(int *bins, int cur) 
{
    for(int j=0; j<NUM_BINS; j++) {
        if(cur < bins[j]) {
            insert(cur, NUM_BINS, bins, j);
            break;
        }
    }

    delay(1);
}

inline int median(int *bins) {
    return bins[NUM_BINS/2];
}

void read_data(void) 
{
    init_bin(solar_buf);
    init_bin(bat_buf);
    #ifndef NO_CURRENT
    init_bin(solar_current_buf);
    init_bin(load_current_buf);
    #endif

    for(int i=0; i<NUM_BINS; i++) {
        bin(solar_buf, analogRead(SOL_ADC));
        bin(bat_buf, analogRead(BAT_ADC));
        #ifndef NO_CURRENT
        insert(solar_current_buf, analogRead(CURRENT_ADC));
        insert(load_current_buf, analogRead(CURRENT_ADC));
        #endif
    }

    static float voltage_div = (VOLTAGE_DIV_R1 + VOLTAGE_DIV_R2)/VOLTAGE_DIV_R2;

    solar_volt = median(solar_buf) * ADC_MULTIPLIER * (voltage_div+0.65);
    bat_volt = median(bat_buf) * ADC_MULTIPLIER * (voltage_div+0.65);

    #ifndef NO_CURRENT
    float tmp = median(curent_buf) * ADC_MULTIPLIER * voltage_div;
    // At 0A, Vout = 2.5, AKA 512
    solar_current = (tmp - 512) * ADC_MULTIPLIER * ACS_712_MULT; 
    #else
    solar_current = 0;
    load_current = 0;
    #endif

    temperature = analogRead(TEMP_ADC) * ADC_MULTIPLIER * 100;
}

void update_calcs(void)
{
    uint32_t msec = millis();
    static uint32_t last_msec = 0;

    elasped_msec = msec - last_msec; //Calculate how long has past since last call of this function
    elasped_time = elasped_msec / 1000.0; // 1sec=1000 msec
    // watts_out = solar_current * bat_volt; //Watts now
    watts_in = solar_current * solar_volt; //Watts now
    ampSecs = (solar_current*elasped_time); //AmpSecs since last measurement
    wattSecs = ampSecs * solar_volt; //WattSecs since last measurement
    ampHours = ampHours + ampSecs/3600; // 1 hour=3600sec //Total ampHours since program started
    wattHours = wattHours + wattSecs/3600; // 1 hour=3600sec //Total wattHours since program started

    last_msec = msec; //Store 'now' for next time
}

//------------------------------------------------------------------------------------------------------------
/////////////////////////////////PRINT DATA IN SERIAL MONITOR/////////////////////////////////////////////////
//------------------------------------------------------------------------------------------------------------
void print_serial(void) 
{
    Serial.print("Solar Panel Voltage: ");
    Serial.print(solar_volt);
    Serial.println("V");
    Serial.print("Battery Voltage: ");
    Serial.print(bat_volt);
    Serial.println("V");
    Serial.print("Charge Set Point:");
    Serial.println(bulk_charge_sp);
    Serial.print("Temperature:");
    Serial.print(temperature);
    Serial.println("C");
    Serial.print("Load Current: ");
    Serial.print(solar_current);
    Serial.println("A");
    Serial.print("Power in: ");
    Serial.print(watts_in);
    Serial.println("W");
    Serial.print("Energy in: ");
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

void set_charge_points(void)
{
    static int temp_change = temperature - 25.0; // 25deg cel is taken as standard room temperature 

    // temperature compensation = -5mv/degC/Cell 
    // If temperature is above the room temp ; Charge set point should reduced
    // If temperature is bellow the room temp ; Charge set point should increased

    bulk_charge_sp = profile.charge_max - (0.030*temp_change);
    float_charge_sp = profile.bat_float_thold - (0.030*temp_change);
}

void duty_set_bulk()
{
    int bulk_dest_pwm = 254;
bulk_dest_pwm = 100;

    // Ramping helps prevent bouncing and possible solar voltage collapse
    charge_status = CHARGING_BULK;

    if (solar_volt > profile.charge_max) 
    {
        /* Don't hit the lithium battery with too much voltage! 
           Only increase PWM until we read profile.charge_max on the voltage divider for
           the battery.
        */
        Serial.print("Solar input voltage > max charge. "); Serial.print(solar_volt); Serial.print(">"); Serial.println(profile.charge_max);
     
        if (bat_volt >= profile.charge_max) {
            duty *= 0.5;
            charge_status = CHARGING_CLAMPED;

            if (duty < 50) {
                Serial.println("Battery is over-charged");
                duty = 0;
            }
        }
        else {
            Serial.println("Solar input voltage > max charge, but battery voltage < max");
        }
    }

    if (charge_status != CHARGING_CLAMPED) {
        charge_status = CHARGING_BULK; // indicate the charger is in BULK mode
        duty += 3;
    }
    else {
        duty += 1; // Slowly walk up. We'll bounce back down next iteration if it gets too high
    }

    if(duty > bulk_dest_pwm) {
        duty = bulk_dest_pwm;
    }
}

void duty_set_float()
{
    // This is to halt charging until the battery falls below a certain 
    // threshold. Prevents overcharging.
    if(charge_status == CHARGING_DONE) {
        if(bat_volt >= profile.bat_recharge) {
            return;
        }
    }

    charge_status = CHARGING_FLOAT;

    // error = Max charge - current battery voltage
    // error * 
    error = (bulk_charge_sp - bat_volt);
    Ep = error * (profile.P + profile.I + profile.D); 
    // TODO left off here
    // PIDs not unimplemented
  
    if(Ep < 0) 
    { 
        Ep = 0;
    }
    else if(Ep>100) 
    {
        Ep = 100;
    }

    if(Ep > 0 && Ep <= 100) // regulating
    {
        float target = Ep * 255 / 100;

        if (duty >= target)
        {
            duty = target;
        }
        else 
        {
            duty += 2;
        }
    }

    Serial.print("Floating. Error: "); Serial.print(error);
    Serial.print(" Charge sp: "); Serial.print(bulk_charge_sp);
    Serial.print(" Ep: "); Serial.print(Ep); Serial.print(" ");
    Serial.print(" Target: "); Serial.print(Ep*255/100); Serial.print(" ");
}

void duty_set_off()
{
    charge_status = CHARGING_OFF;
    duty = 0;
}

void duty_taper()
{
    duty *= 0.75;

    if(duty <= 10) {
        duty_set_off();
    }
}

void charge_set(void)
{
    // XXX All of this needs to be a little more sophisticated. If there's a 
    // load attached and we're charged, we still need some amount of solar 
    // moving through. Right now this acts more like a battery charger.
    //
    // Need current sensor on the output. Use it with a current sensor on the
    // input to try to match currents.

    if (solar_current > profile.current_in_max) {
        // XXX This is clumsy. We might be sitting right at the max
        // then start bouncing on an off rather than tapering back.
        // Won't happen with my setup, but future TODO.
        Serial.println("Solar current exceeded max!");
        duty *= 0.5;
        if(duty < 50) {
            duty = 0;
        }
    }
    else if (solar_volt > bat_volt)
    {
        if (bat_volt >= profile.bat_full_volt) {
            // We're full 
            duty_set_off();
            charge_status = CHARGING_DONE;
        }
        else if (bat_volt <= float_charge_sp)
        {
            duty_set_bulk();
        }
        else
        {   
            duty_set_float();
        }

        #if 0
        else if (bat_volt <= profile.bat_recharge) {
            Serial.println("Topping off.");
            duty_set_float();
        }
        else {
            // Battery voltage < max and
            // battery voltage > float charge
            // battery voltage > bulk charge (max charge voltage adjusted for temperature)
            // If we're here there's either an issue with the #defs above or a temperature
            // variation caused the bulk charge set point to decrease while we were charged

            Serial.println("Error in charge_set");
            duty_set_off();
        }
        #endif
    }
    else
    {
        // If we were previously charging, and we're not full, might be voltage 
        // collapse. Dial back duty cycle rather than cut
        if(charge_status != CHARGING_OFF) 
            duty_taper();
        else
            duty_set_off();
    }

    // XXX Disabled during development
    // analogWrite(PIN_PWM, duty);
}
 
void set_load_control()
{
    /* Using a separate constant for load off and load on prevent flapping the
       load on off when things are bouncing around the threshold. */
    if(bat_volt <= profile.load_off) {
        load_status = 0;
        digitalWrite(PIN_LOAD, LOW);
    }
    else if(bat_volt >= profile.load_on) {
        load_status = 1;

        // XXX TODO: measure load voltage and alert load_duty as necessary
        // to half-assedly regulate output
        // int load_duty = 255;
        // analogWrite(PIN_LOAD, load_duty);
        digitalWrite(PIN_LOAD, HIGH);
    }
}

void led_indication(void)
{
    /* Battery LEDs */
    if( (bat_volt >= profile.bat_float_thold) && (bat_volt < bulk_charge_sp))
    {   
        leds_off_all();
        digitalWrite(BAT_GREEN_LED,LOW);  // battery voltage is healthy
    } 
    else if(bat_volt >= bulk_charge_sp) 
    {
        leds_off_all();
        digitalWrite(BAT_BLUE_LED,LOW);  //battery is fully charged
    }
    else if(bat_volt < profile.bat_float_thold)
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
    lcd.print(solar_current); 
    lcd.print("A");
    lcd.setCursor(13,2); 
    lcd.print(watts_in); 
    lcd.print("W"); 
    lcd.setCursor(0,3);
    lcd.print("Energy:"); 
    lcd.print(wattHours);
    lcd.print("WH"); 
}
