
Arduino PWM charge controller
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

