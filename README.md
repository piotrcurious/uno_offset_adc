# uno_offset_adc
op-amp based offset subtracting ADC
created using BingAI
code contains many bugs. 
But many parts are a bit ok.
I will mash it up together someday.

prompt:
use creativity to create Arduino uno code for voltage measurement using following assumptions and methods: -Assume there is an unity gain op-amp output connected thru adjustable voltage divider to A0 pin. Op amp non inverting input is connected to measurement voltage source and inverting input is connected to 16bit pwm pin 9. There is 1uF capacitor connected in-between PWM pin and ground, forming RC filter with internal resistance of the pin9. ADC is set to internal reference. Method of measurement: on each call to measurement function, 16bit PWM creating offset for ADC through op-amp is converged to find the measurement voltage window by binary search. As it will be not able to converge to perfect middle, window parameter is needed to exit search once ADC readout is within acceptable window. ADC setting after search provides upper 16 bit of measurement. Then the ADC is read to provide lower 10 bits. Then those results are combined. As the bit overlap in-between those two devices is not ideal, provide test function to calibrate the voltage divider on A0 input and internal variables used for correction of offset, stored in EEPROM after calibration. Calibration can be performed by asking user to connect 100uF capacitor to measurement input, 1k ohm resistor to digital pin 10 and then finding the real RC constant of this circuit in software and using it as reference voltage source and asking user to set the voltage divider on A0 pin to get maximum measurement range and then storing the correcting variables in EEPROM.
