///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
TwoWire HWire (2, I2C_FAST_MODE);

//////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
/////////////////////////////////////////////////////////////////////////////

float pid_p_gain_roll = 1.2;                //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.03;               //Gain setting for pitch roll I (default 0.04)
float pid_d_gain_roll = 18;               //Gain setting for pitch roll D (default 18.0)
int   pid_max_roll = 400;                   //Maximum output/value of the PID-controller

float pid_p_gain_pitch = pid_p_gain_roll;   //Gain setting for the pitch P controller
float pid_i_gain_pitch = pid_i_gain_roll;   //Gain setting for the pitch I controller
float pid_d_gain_pitch = pid_d_gain_roll;   //Gain setting for the pitch D controller
int   pid_max_pitch = pid_max_roll;         //Maximum output of the PID controller

float pid_p_gain_yaw = 5.0;
float pid_i_gain_yaw = 0.04;
float pid_d_gain_yaw = 0.0;
int   pid_max_yaw = 400;

//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
//altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
//Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
float battery_compensation = 25;

boolean auto_level = true;                  //autolevel (true) or angle mode (false)

//Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = -70;
int16_t manual_acc_roll_cal_value = 151;

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.
uint8_t use_manual_calibration = false; // Set to false or true;
int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;

float low_battery_warning = 10.5; //Set the battery warning at 10.5V (default = 10.5V).

uint8_t gyro_address = 0x68;

////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer

uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t start;
uint8_t error, error_counter, error_led;
uint8_t channel_select_counter;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle, cal_int;
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
int32_t acc_total_vector;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

uint32_t loop_timer, error_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;

/////////////////////////////////////////////////////////////////
//Setup
/////////////////////////////////////////////////////////////////
void setup() {
    //pinMode(PA4, INPUT_ANALOG);         //To read analog value of PA4, Battery Voltage

    afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);

    pinMode(PB3, OUTPUT);               //Set PB3 as output for LED.
    pinMode(PB4, OUTPUT);               //Set PB4 as output for LED.

    green_led(LOW);                     //Set output PB3 low.
    red_led(HIGH);                      //Set output PB4 high.

    timer_setup();
    delay(50);

    HWire.begin();
    HWire.beginTransmission(gyro_address);
    error = HWire.endTransmission();

    while (error != 0){
        error = 2;
        error_signal();
        delay(4);
    }

    gyro_setup();                       //Initiallize the gyro and set the correct registers.

    if (!use_manual_calibration)
    {
        //Create a 5 second delay before calibration.
        for (count_var = 0; count_var < 1250; count_var++)
        { //1250 loops of 4 microseconds = 5 seconds
            if (count_var % 125 == 0)
            {                                         //Every 125 loops (500ms).
                digitalWrite(PB4, !digitalRead(PB4)); //Change the led status.
            }
            delay(4); //Delay 4 microseconds
        }
        count_var = 0; //Set start back to 0.
    }

    calibrate_gyro();                   //Calibrate the gyro offset.

    //Wait until receiver is active.
    while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990){
        error = 3;
        error_signal();
        delay(4);
    }

    error = 0;

    //Wait until the throttle is set to the lower position.
    while (channel_3 < 990 || channel_3 > 1050) {
        error = 4;
        error_signal();
        delay(4);
    }
    error = 0;

    //Load the battery voltage to the battery_voltage variable.
    //The STM32 uses a 12 bit analog to digital converter.
    //analogRead => 0 = 0V ..... 4095 = 3.3V
    //The voltage divider (1k & 10k) is 1:11.
    //analogRead => 0 = 0V ..... 4095 = 36.3V
    //36.3 / 4095 = 112.81.
    //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
    battery_voltage = (float)analogRead(PA4) / 112.81;

    //When everything is done, turn off the led.
    red_led(LOW);

    loop_timer = micros();

    green_led(HIGH);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    error_signal();
    gyro_signalen();

    //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
    gyro_roll_input  = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);                          //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);                       //Gyro pid input is deg/sec.
    gyro_yaw_input   = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);                             //Gyro pid input is deg/sec.

    //Gyro angle calculations
    //0.0000611 = 1/(250Hz/65.5)
    angle_pitch += (float)gyro_pitch * 0.0000611;
    angle_roll += (float)gyro_roll * 0.0000611;

    //0.000001066 =  0.0000611 * PI/180, Arduino function is in radians.
    angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                                         //If the IMU has yawed transfer the roll angle to the pitch angle.
    angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                                         //If the IMU has yawed transfer the pitch angle to the roll angle.

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));                           //Calculate accelerometer vector.

    if (abs(acc_y) < acc_total_vector) {                                                                    //Prevent the asin function to produce a NaN.
        angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;                                   //Calculate the pitch angle.
    }
    if (abs(acc_x) < acc_total_vector) {                                                                    //Prevent the asin function to produce a NaN.
        angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;                                    //Calculate the roll angle.
    }

    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                                          //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                                             //Correct the drift of the gyro roll angle with the accelerometer roll angle.

    pitch_level_adjust = angle_pitch * 15;                                                                  //Calculate the pitch angle correction.
    roll_level_adjust = angle_roll * 15;                                                                    //Calculate the roll angle correction.

    if (!auto_level)
    {                                                                                                       //If the quadcopter is not in auto-level mode
        pitch_level_adjust = 0;                                                                             //Set the pitch angle correction to zero.
        roll_level_adjust = 0;                                                                              //Set the roll angle correcion to zero.
    }


    //For starting the motors: throttle low and yaw left (step 1).
    if (channel_3 < 1050 && channel_4 < 1050)
        start = 1;

    //When yaw stick is back in the center position start the motors (step 2).
    if (start == 1 && channel_3 < 1050 && channel_4 > 1450){
        start = 2;

        green_led(LOW);                                                                                     //Turn off the green LED

        angle_pitch = angle_pitch_acc;                                                                      //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
        angle_roll = angle_roll_acc;                                                                        //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

        //Reset the PID controllers for a bumpless start.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
    }

    //Stopping the motors: throttle low and yaw right.
    if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
        start = 0;
        green_led(HIGH);
    }

    //The PID set point in degrees per second is determined by the roll receiver input.
    //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).

    pid_roll_setpoint = 0;

    //We need a little dead band of 16us for better results.
    if (channel_1 > 1508)
        pid_roll_setpoint = channel_1 - 1508;
    else if (channel_1 < 1492)
        pid_roll_setpoint = channel_1 - 1492;

    pid_roll_setpoint -= roll_level_adjust;                                                                                 //Subtract the angle correction from the standardized receiver roll input value.
    pid_roll_setpoint /= 3.0;                                                                                               //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

    //The PID set point in degrees per second is determined by the pitch receiver input.
    //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).

    pid_pitch_setpoint = 0;

    //We need a little dead band of 16us for better results.
    if (channel_2 > 1508)
        pid_pitch_setpoint = channel_2 - 1508;
    else if (channel_2 < 1492)
        pid_pitch_setpoint = channel_2 - 1492;

    pid_pitch_setpoint -= pitch_level_adjust;                                                                               //Subtract the angle correction from the standardized receiver pitch input value.
    pid_pitch_setpoint /= 3.0;                                                                                              //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

    //The PID set point in degrees per second is determined by the yaw receiver input.
    //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).

    pid_yaw_setpoint = 0;

    //We need a little dead band of 16us for better results.
    if (channel_3 > 1050)
    { //Do not yaw when turning off the motors.
        if (channel_4 > 1508)
            pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
        else if (channel_4 < 1492)
            pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
    }

    calculate_pid();                                //PID inputs are known.

    //The battery voltage is needed for compensation.
    //A complementary filter is used to reduce noise.
    //1410.1 = 112.81 / 0.08.
    battery_voltage = battery_voltage * 0.92 + ((float)analogRead(PA4) / 1410.1);

    //Turn on the led if battery voltage is to low. Default setting is 10.5V
    if (battery_voltage > 6.0 && battery_voltage < low_battery_warning && error == 0)
        error = 1;
    else error = 0;

    throttle = channel_3; //We need the throttle signal as a base signal.

    if (start == 2)
    {                        //Start motors
        if (throttle > 1850) //Have controll at full throttle
            throttle = 1850;

        esc_1 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW).
        esc_2 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW).
        esc_3 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW).
        esc_4 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW).

        if (battery_voltage < 12.40 && battery_voltage > 6.0)
        {                                                              //Is the battery connected?
            esc_1 += (12.40 - battery_voltage) * battery_compensation; //Compensate the esc-1 pulse for voltage drop.
            esc_2 += (12.40 - battery_voltage) * battery_compensation; //Compensate the esc-2 pulse for voltage drop.
            esc_3 += (12.40 - battery_voltage) * battery_compensation; //Compensate the esc-3 pulse for voltage drop.
            esc_4 += (12.40 - battery_voltage) * battery_compensation; //Compensate the esc-4 pulse for voltage drop.
        }

            if (esc_1 < 1100)
                esc_1 = 1100; //Keep the motors running.
            if (esc_2 < 1100)
                esc_2 = 1100; //Keep the motors running.
            if (esc_3 < 1100)
                esc_3 = 1100; //Keep the motors running.
            if (esc_4 < 1100)
                esc_4 = 1100; //Keep the motors running.

            if (esc_1 > 2000)
                esc_1 = 2000; //Limit the esc-1 pulse to 2000us.
            if (esc_2 > 2000)
                esc_2 = 2000; //Limit the esc-2 pulse to 2000us.
            if (esc_3 > 2000)
                esc_3 = 2000; //Limit the esc-3 pulse to 2000us.
            if (esc_4 > 2000)
                esc_4 = 2000; //Limit the esc-4 pulse to 2000us.
    }

    else {
        esc_1 = 1000;
        esc_2 = 1000;
        esc_3 = 1000;
        esc_4 = 1000;
    }

    TIMER2_BASE->CCR1 = esc_1;                                                  //Set the throttle receiver input pulse to the ESC 1 output pulse.
    TIMER2_BASE->CCR2 = esc_2;                                                  //Set the throttle receiver input pulse to the ESC 2 output pulse.
    TIMER2_BASE->CCR3 = esc_3;                                                  //Set the throttle receiver input pulse to the ESC 3 output pulse.
    TIMER2_BASE->CCR4 = esc_4;                                                  //Set the throttle receiver input pulse to the ESC 4 output pulse.
    TIMER2_BASE->CNT = 5000;                                                    //This will reset timer 2 and the ESC pulses are directly created.

    //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    //Because of the angle calculation the loop time is getting very important. If the loop time is
    //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
    //that the loop time is still 4000us and no longer! More information can be found on
    //the Q&A page:
    //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !

    if (micros() - loop_timer > 4050)
        error = 5; //Turn on the LED if the loop time exceeds 4050us.
    while (micros() - loop_timer < 4000);                  //We wait until 4000us are passed.
    
    loop_timer = micros(); //Set the timer for the next loop.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//These functions handle the red and green LEDs. The LEDs on the flip 32 are inverted. That is why a Flip32 test is needed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void red_led(int8_t level)
{
    digitalWrite(PB4, level);                                                   //When using the BluePill the output should not be inverted.
}
void green_led(int8_t level)
{
    digitalWrite(PB3, level);                                                   //When using the BluePill the output should not be inverted.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(void){
    
    //Roll calculations
    pid_error_temp = gyro_roll_input - pid_roll_setpoint;
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

    if (pid_i_mem_roll > pid_max_roll)
        pid_i_mem_roll = pid_max_roll;
    else if (pid_i_mem_roll < pid_max_roll * -1)
        pid_i_mem_roll = pid_max_roll * -1;

    pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);

    if (pid_output_roll > pid_max_roll)
        pid_output_roll = pid_max_roll;
    else if (pid_output_roll < pid_max_roll * -1)
        pid_output_roll = pid_max_roll * -1;

    pid_last_roll_d_error = pid_error_temp;

    //Pitch calculations
    pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
    pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
    
    if (pid_i_mem_pitch > pid_max_pitch)
        pid_i_mem_pitch = pid_max_pitch;
    else if (pid_i_mem_pitch < pid_max_pitch * -1)
        pid_i_mem_pitch = pid_max_pitch * -1;

    pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

    if (pid_output_pitch > pid_max_pitch)
        pid_output_pitch = pid_max_pitch;
    else if (pid_output_pitch < pid_max_pitch * -1)
        pid_output_pitch = pid_max_pitch * -1;

    pid_last_pitch_d_error = pid_error_temp;

    //Yaw calculations
    pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
    pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
    
    if (pid_i_mem_yaw > pid_max_yaw)
        pid_i_mem_yaw = pid_max_yaw;
    else if (pid_i_mem_yaw < pid_max_yaw * -1)
        pid_i_mem_yaw = pid_max_yaw * -1;

    pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
    if (pid_output_yaw > pid_max_yaw)
        pid_output_yaw = pid_max_yaw;
    else if (pid_output_yaw < pid_max_yaw * -1)
        pid_output_yaw = pid_max_yaw * -1;

    pid_last_yaw_d_error = pid_error_temp;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
    if (use_manual_calibration)
        cal_int = 2000; //If manual calibration is used set cal_int to 2000 to skip the calibration.
    else
    {
        cal_int = 0;                     //If manual calibration is not used.
        manual_gyro_pitch_cal_value = 0; //Set the manual pitch calibration variable to 0.
        manual_gyro_roll_cal_value = 0;  //Set the manual roll calibration variable to 0.
        manual_gyro_yaw_cal_value = 0;   //Set the manual yaw calibration variable to 0.
    }

    if (cal_int != 2000) {
        //Determine average gyro offset (calibration);
        for (cal_int = 0; cal_int < 2000; cal_int ++) {
            if (cal_int % 25 == 0)
                digitalWrite(PB4, !digitalRead(PB4));
            gyro_signalen();                                                         //Read the gyro output.
            gyro_roll_cal += gyro_roll;                                          //Ad roll value to gyro_roll_cal.
            gyro_pitch_cal += gyro_pitch;                                           //Ad pitch value to gyro_pitch_cal.
            gyro_yaw_cal += gyro_yaw;                                               //Ad yaw value to gyro_yaw_cal.
            delay(4);
        }
    
        red_led(HIGH); //Set output PB3 low.
        //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
        gyro_roll_cal /= 2000;                                                          //Divide the roll total by 2000.
        gyro_pitch_cal /= 2000;                                                         //Divide the pitch total by 2000.
        gyro_yaw_cal /= 2000;                                                           //Divide the yaw total by 2000.
        manual_gyro_pitch_cal_value = gyro_pitch_cal;                                   //Set the manual pitch calibration variable to the detected value.
        manual_gyro_roll_cal_value = gyro_roll_cal;                                     //Set the manual roll calibration variable to the detected value.
        manual_gyro_yaw_cal_value = gyro_yaw_cal;                                       //Set the manual yaw calibration variable to the detected value.
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the error LED signal is generated.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void error_signal(void)
{
    if (error >= 100)
        red_led(HIGH);                                                              //When the error is 100 the LED is always on.
    else if (error_timer < millis())
    {                                                                               //If the error_timer value is smaller that the millis() function.
        error_timer = millis() + 250;                                               //Set the next error_timer interval at 250ms.
        if (error > 0 && error_counter > error + 3)
            error_counter = 0;                                                      //If there is an error to report and the error_counter > error +3 reset the error.
        if (error_counter < error && error_led == 0 && error > 0)
        {                                                                           //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
            red_led(HIGH);                                                          //Turn the LED on.
            error_led = 1;                                                          //Set the LED flag to indicate that the LED is on.
        }
        else
        {                                                                           //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
            red_led(LOW);                                                           //Turn the LED off.
            error_counter++;                                                        //Increment the error_counter variable by 1 to keep trach of the flashes.
            error_led = 0;                                                          //Set the LED flag to indicate that the LED is off.
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the various registers of the MPU-6050 are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_setup(void)
{
    HWire.beginTransmission(gyro_address); //Start communication with the MPU-6050.
    HWire.write(0x6B);                     //We want to write to the PWR_MGMT_1 register (6B hex).
    HWire.write(0x00);                     //Set the register bits as 00000000 to activate the gyro.
    HWire.endTransmission();               //End the transmission with the gyro.

    HWire.beginTransmission(gyro_address); //Start communication with the MPU-6050.
    HWire.write(0x1B);                     //We want to write to the GYRO_CONFIG register (1B hex).
    HWire.write(0x08);                     //Set the register bits as 00001000 (500dps full scale).
    HWire.endTransmission();               //End the transmission with the gyro.

    HWire.beginTransmission(gyro_address); //Start communication with the MPU-6050.
    HWire.write(0x1C);                     //We want to write to the ACCEL_CONFIG register (1A hex).
    HWire.write(0x10);                     //Set the register bits as 00010000 (+/- 8g full scale range).
    HWire.endTransmission();               //End the transmission with the gyro.

    HWire.beginTransmission(gyro_address); //Start communication with the MPU-6050.
    HWire.write(0x1A);                     //We want to write to the CONFIG register (1A hex).
    HWire.write(0x03);                     //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
    HWire.endTransmission();               //End the transmission with the gyro.
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//Reads PPM value of receiver into handler channel 1, pin PA6, TIMER 3
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void handler_channel_1(void) //Reads receiver PPM input and transfers them to respective channels
{
    measured_time = TIMER3_BASE->CCR1 - measured_time_start;
    if (measured_time < 0)
        measured_time += 0xFFFF;
    measured_time_start = TIMER3_BASE->CCR1;
    if (measured_time > 3000)
        channel_select_counter = 0;
    else
        channel_select_counter++;

    if (channel_select_counter == 1)
        channel_1 = measured_time;
    if (channel_select_counter == 2)
        channel_2 = measured_time;
    if (channel_select_counter == 3)
        channel_3 = measured_time;
    if (channel_select_counter == 4)
        channel_4 = measured_time;
    if (channel_select_counter == 5)
        channel_5 = measured_time;
    if (channel_select_counter == 6)
        channel_6 = measured_time;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(void)
{
    HWire.beginTransmission(gyro_address);          //Start communication with the gyro.
    HWire.write(0x3B);                              //Start reading @ register 43h and auto increment with every read.
    HWire.endTransmission();                        //End the transmission.
    HWire.requestFrom(gyro_address, 14);            //Request 14 bytes from the MPU 6050.
    acc_y = HWire.read() << 8 | HWire.read();       //Add the low and high byte to the acc_x variable.
    acc_x = HWire.read() << 8 | HWire.read();       //Add the low and high byte to the acc_y variable.
    acc_z = HWire.read() << 8 | HWire.read();       //Add the low and high byte to the acc_z variable.
    temperature = HWire.read() << 8 | HWire.read(); //Add the low and high byte to the temperature variable.
    gyro_roll = HWire.read() << 8 | HWire.read();   //Read high and low part of the angular data.
    gyro_pitch = HWire.read() << 8 | HWire.read();  //Read high and low part of the angular data.
    gyro_yaw = HWire.read() << 8 | HWire.read();    //Read high and low part of the angular data.
    //gyro_pitch *= -1;                               //Invert the direction of the axis.
    gyro_yaw *= -1;                                 //Invert the direction of the axis.

    acc_y -= manual_acc_pitch_cal_value;       //Subtact the manual accelerometer pitch calibration value.
    acc_x -= manual_acc_roll_cal_value;        //Subtact the manual accelerometer roll calibration value.
    gyro_roll -= manual_gyro_roll_cal_value;   //Subtact the manual gyro roll calibration value.
    gyro_pitch -= manual_gyro_pitch_cal_value; //Subtact the manual gyro pitch calibration value.
    gyro_yaw -= manual_gyro_yaw_cal_value;     //Subtact the manual gyro yaw calibration value.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void timer_setup(void)
{
    Timer3.attachCompare1Interrupt(handler_channel_1);
    //PPM Receiver will be attached to PA6, TIMER 3 Pin.
    
    TIMER3_BASE->CR1 = TIMER_CR1_CEN;
    TIMER3_BASE->CR2 = 0;
    TIMER3_BASE->SMCR = 0;
    TIMER3_BASE->DIER = TIMER_DIER_CC1IE;
    TIMER3_BASE->EGR = 0;
    TIMER3_BASE->CCMR1 = TIMER_CCMR1_CC1S_INPUT_TI1;
    TIMER3_BASE->CCMR2 = 0;
    TIMER3_BASE->CCER = TIMER_CCER_CC1E;

    //TIMER3_BASE->CCER |= TIMER_CCER_CC1P;     //Detect falling edge.
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P;      //Detect rising edge.
    TIMER3_BASE->PSC = 71;
    TIMER3_BASE->ARR = 0xFFFF;
    TIMER3_BASE->DCR = 0;

    TIMER2_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
    TIMER2_BASE->CR2 = 0;
    TIMER2_BASE->SMCR = 0;
    TIMER2_BASE->DIER = 0;
    TIMER2_BASE->EGR = 0;
    TIMER2_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE | (0b110 << 12) | TIMER_CCMR1_OC2PE;
    TIMER2_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE | (0b110 << 12) | TIMER_CCMR2_OC4PE;
    TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
    TIMER2_BASE->PSC = 71;
    TIMER2_BASE->ARR = 5000;
    TIMER2_BASE->DCR = 0;
    TIMER2_BASE->CCR1 = 1000;

    TIMER2_BASE->CCR1 = 1000;
    TIMER2_BASE->CCR2 = 1000;
    TIMER2_BASE->CCR3 = 1000;
    TIMER2_BASE->CCR4 = 1000;

    //ESC Output maps to TIMER2 Pins, PA0 - PA3
    
    pinMode(PA0, PWM);
    pinMode(PA1, PWM);
    pinMode(PA2, PWM);
    pinMode(PA3, PWM);
}