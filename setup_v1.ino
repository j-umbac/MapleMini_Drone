#include <Wire.h>

//HardWire HWire(2, I2C_FAST_MODE);
TwoWire HWire(2, I2C_FAST_MODE);

//Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = -70;
int16_t manual_acc_roll_cal_value = 151;

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.
uint8_t use_manual_calibration = false;
int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;



uint8_t disable_throttle;
uint8_t error;
uint32_t loop_timer;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;
int16_t loop_counter;
uint8_t data, start, warning;
int16_t acc_axis[4], gyro_axis[4], temperature;
int32_t gyro_axis_cal[4], acc_axis_cal[4];
int32_t cal_int;
int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
uint8_t channel_select_counter;

uint8_t gyro_address = 0x68;

void setup (){
    
    //pinMode(4, INPUT_ANALOG);
        //Port PB3 and PB4 are used as JTDO and JNTRST by default.
        //The following function connects PB3 and PB4 to the alternate output function.

    afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY); //Connects PB3 and PB4 to output function.

    pinMode(PB3, OUTPUT);               //PA9 as green_led        
    pinMode(PB4, OUTPUT);              //PA10 as red_led

    Serial.begin(57600);
    delay(100);
    timer_setup();
    delay(50);

    HWire.begin(); //Start the I2C as master

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

    print_intro(); //Print the intro on the serial monitor.

}

void loop()
{
    // put your main code here, to run repeatedly:
    delay(10);

    if (Serial.available() > 0)
    {
        data = Serial.read();             //Read the incoming byte
        delay(100);                       //Wait for any other bytse to come in.
        while (Serial.available() > 0)    //Empty the Serial buffer
            loop_counter = Serial.read(); //Set the throttle to 1000us to disable the motors.
        disable_throttle = 1;
    }
    if (!disable_throttle)
    { //If the throttle is not disabled, sets the throttle of receiver input pulse to the ESC 1 output pulse.
        TIMER2_BASE->CCR1 = channel_3;
        TIMER2_BASE->CCR2 = channel_3;
        TIMER2_BASE->CCR3 = channel_3;
        TIMER2_BASE->CCR4 = channel_3;
    }
    else
    {
        TIMER2_BASE->CCR1 = 1000;
        TIMER2_BASE->CCR2 = 1000;
        TIMER2_BASE->CCR3 = 1000;
        TIMER2_BASE->CCR4 = 1000;
    }

    if (data == 'a')
    {
        Serial.println(F("Reading receiver input pulses."));
        Serial.println(F("You can exit by sending a q (quit)."));
        delay(2500);
        reading_receiver_signals();
    }
    if (data == 'b')
    {
        Serial.println(F("Starting the I2C scanner. "));
        i2c_scanner();
    }
    if (data == 'c')
    {
        Serial.println(F("Reading raw gyro data."));
        Serial.println(F("You can exit by sending a q (quit)."));
        read_gyro_values();
    }

    if (data == 'd')
    {
        Serial.println(F("Reading the raw accelerometer data."));
        Serial.println(F("You can exit by sending a q (quit)."));
        delay(2500);
        read_gyro_values();
    }

    if (data == 'e')
    {
        Serial.println(F("Reading the IMU angles."));
        Serial.println(F("You can exit by sending a q (quit)."));
        check_imu_angles();
    }

    if (data == 'f')
    {
        Serial.println(F("Test the LEDs."));
        test_leds();
    }

    if (data == 'g')
    {
        Serial.println(F("Reading the battery voltage."));
        Serial.println(F("You can exit by sending a q (quit)."));
        check_battery_voltage();
    }

    if (data == 'h')
    {
        Serial.println(F("Get manual gyro and accelerometer calibration values."));
        manual_imu_calibration();
    }

    if (data == '1')
    {
        Serial.println(F("Check motor 1 (front right, counter clockwise direction)."));
        Serial.println(F("You can exit by sending a q (quit)."));
        delay(2500);
        check_motor_vibrations();
    }

    if (data == '2')
    {
        Serial.println(F("Check motor 2 (rear right, clockwise direction)."));
        Serial.println(F("You can exit by sending a q (quit)."));
        delay(2500);
        check_motor_vibrations();
    }

    if (data == '3')
    {
        Serial.println(F("Check motor 3 (rear left, counter clockwise direction)."));
        Serial.println(F("You can exit by sending a q (quit)."));
        delay(2500);
        check_motor_vibrations();
    }

    if (data == '4')
    {
        Serial.println(F("Check motor 4 (front left, clockwise direction)."));
        Serial.println(F("You can exit by sending a q (quit)."));
        delay(2500);
        check_motor_vibrations();
    }

    if (data == '5')
    {
        Serial.println(F("Check all motors."));
        Serial.println(F("You can exit by sending a q (quit)."));
        delay(2500);
        check_motor_vibrations();
    }
} //end loop

void red_led(int8_t level)
{
    digitalWrite(PB4, level);
} //end red_led
void green_led(int8_t level)
{
    digitalWrite(PB3, level);
} //end green_led

void gyro_signalen(void)
{
    //Read MPU-6050 data.
    HWire.beginTransmission(gyro_address); //Start comms with gyro
    HWire.write(0x3B);                     //Start reading at register 43h and auto increment with every read
    HWire.endTransmission();               //End transmission
    HWire.requestFrom(gyro_address, 14);   //Request 14 bytes from the MPU 6050

    acc_axis[1] = HWire.read() << 8 | HWire.read();  //acceleration x axis
    acc_axis[2] = HWire.read() << 8 | HWire.read();  //acceleration y axis
    acc_axis[3] = HWire.read() << 8 | HWire.read();  //acceleration z axis
    temperature = HWire.read() << 8 | HWire.read();  //read temp
    gyro_axis[1] = HWire.read() << 8 | HWire.read(); //angular data.
    gyro_axis[2] = HWire.read() << 8 | HWire.read(); //angular data.
    gyro_axis[3] = HWire.read() << 8 | HWire.read(); //angular data.
    //gyro_axis[2] *= -1;                              //Invert gyro so that nose up gives positive value.
    gyro_axis[3] *= -1;                              //Invert gyro so that nose right gives positive value.

    if (cal_int >= 2000)
    {
        gyro_axis[1] -= gyro_axis_cal[1]; //Subtract the manual gyro roll calibration value.
        gyro_axis[2] -= gyro_axis_cal[2]; //Subtract the manual gyro pitch calibration value.
        gyro_axis[3] -= gyro_axis_cal[3]; //Subtract the manual gyro yaw calibration value.
    }

} // end gyro_signalen

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

void reading_receiver_signals(void)
{
    while (data != 'q')
    {               //Stay in this loop until the data variable data holds a q.
        delay(250); //Print the receiver values on the screen every 250ms
        if (Serial.available() > 0)
        {                         //If serial data is available
            data = Serial.read(); //Read the incomming byte
            delay(100);           //Wait for any other bytes to come in
            while (Serial.available() > 0)
                loop_counter = Serial.read(); //Empty the Serial buffer
        }
        //For starting the motors: throttle low and yaw left (step 1).
        if (channel_3 < 1100 && channel_4 < 1100)
            start = 1;
        //When yaw stick is back in the center position start the motors (step 2).
        if (start == 1 && channel_3 < 1100 && channel_4 > 1450)
            start = 2;
        //Stopping the motors: throttle low and yaw right.
        if (start == 2 && channel_3 < 1100 && channel_4 > 1900)
            start = 0;

        Serial.print("Start:");
        Serial.print(start);

        Serial.print("  Roll:");
        if (channel_1 - 1480 < 0)
            Serial.print("<<<");
        else if (channel_1 - 1520 > 0)
            Serial.print(">>>");
        else
            Serial.print("-+-");
        Serial.print(channel_1);

        Serial.print("  Pitch:");
        if (channel_2 - 1480 < 0)
            Serial.print("^^^");
        else if (channel_2 - 1520 > 0)
            Serial.print("vvv");
        else
            Serial.print("-+-");
        Serial.print(channel_2);

        Serial.print("  Throttle:");
        if (channel_3 - 1480 < 0)
            Serial.print("vvv");
        else if (channel_3 - 1520 > 0)
            Serial.print("^^^");
        else
            Serial.print("-+-");
        Serial.print(channel_3);

        Serial.print("  Yaw:");
        if (channel_4 - 1480 < 0)
            Serial.print("<<<");
        else if (channel_4 - 1520 > 0)
            Serial.print(">>>");
        else
            Serial.print("-+-");
        Serial.print(channel_4);

        Serial.print("  CH5:");
        Serial.print(channel_5);

        Serial.print("  CH6:");
        Serial.println(channel_6);
    }
    print_intro();
}

void timer_setup(void) //Timers for reading pulses and for creating the output ESC pulses are set.
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
    TIMER3_BASE->CCER &= ~TIMER_CCER_CC1P; //Detect rising edge.
    TIMER3_BASE->PSC = 71;
    TIMER3_BASE->ARR = 0xFFFF;
    TIMER3_BASE->DCR = 0;

    //A test is needed to check if the throttle input is active and valid. Otherwise the ESC's might start without any warning.
    loop_counter = 0;
    while ((channel_3 > 2100 || channel_3 < 900) && warning == 0)
    {
        delay(100);
        loop_counter++;
        if (loop_counter == 40)
        {
            Serial.println(F("Waiting for a valid receiver channel-3 input signal"));
            Serial.println(F(""));
            Serial.println(F("The input pulse should be between 1000 till 2000us"));
            Serial.print(F("Current channel-3 receiver input value = "));
            Serial.println(channel_3);
            Serial.println(F(""));
            Serial.println(F("Is the receiver connected and the transmitter on?"));
            Serial.println(F("For more support and questions: www.brokking.net"));
            Serial.println(F(""));
            Serial.print(F("Waiting for another 5 seconds."));
        }
        if (loop_counter > 40 && loop_counter % 10 == 0)
            Serial.print(F("."));

        if (loop_counter == 90)
        {
            Serial.println(F(""));
            Serial.println(F(""));
            Serial.println(F("The ESC outputs are disabled for safety!!!"));
            warning = 1;
        }
    }
    if (warning == 0)
    {
        TIMER2_BASE->CR1 = TIMER_CR1_CEN | TIMER_CR1_ARPE;
        TIMER2_BASE->CR2 = 0;
        TIMER2_BASE->SMCR = 0;
        TIMER2_BASE->DIER = 0;
        TIMER2_BASE->EGR = 0;
        TIMER2_BASE->CCMR1 = (0b110 << 4) | TIMER_CCMR1_OC1PE | (0b110 << 12) | TIMER_CCMR1_OC2PE;
        TIMER2_BASE->CCMR2 = (0b110 << 4) | TIMER_CCMR2_OC3PE | (0b110 << 12) | TIMER_CCMR2_OC4PE;
        TIMER2_BASE->CCER = TIMER_CCER_CC1E | TIMER_CCER_CC2E | TIMER_CCER_CC3E | TIMER_CCER_CC4E;
        TIMER2_BASE->PSC = 71;
        TIMER2_BASE->ARR = 4000;
        TIMER2_BASE->DCR = 0;
        TIMER2_BASE->CCR1 = 1000;

        TIMER2_BASE->CCR1 = channel_3;
        TIMER2_BASE->CCR2 = channel_3;
        TIMER2_BASE->CCR3 = channel_3;
        TIMER2_BASE->CCR4 = channel_3;
        pinMode(PA0, PWM);
        pinMode(PA1, PWM);
        pinMode(PA2, PWM);
        pinMode(PA3, PWM);
    }
}

void check_battery_voltage(void)
{
    loop_counter = 0;                //Reset the loop counter.
    battery_voltage = analogRead(4); //Set battery voltage.
    while (data != 'q')
    {                            //Stay in this loop until the data variable data holds a q.
        delayMicroseconds(4000); //Wait for 4000us to simulate a 250Hz loop.
        if (Serial.available() > 0)
        {                         //If serial data is available.
            data = Serial.read(); //Read the incomming byte.
            delay(100);           //Wait for any other bytes to come in.
            while (Serial.available() > 0)
                loop_counter = Serial.read(); //Empty the Serial buffer.
        }
        loop_counter++;
        if (loop_counter == 250)
        {                                              //Print the battery voltage every second.
            Serial.print("Voltage = ");                //Print some preliminary information.
            Serial.print(battery_voltage / 112.81, 1); //Print the avarage battery voltage to the serial monitor.
            Serial.println("V");                       //Print some trailing information.
            loop_counter = 0;                          //Reset the loop counter.
        }
        //A complimentary filter is used to filter out the voltage spikes caused by the ESC's.
        battery_voltage = (battery_voltage * 0.99) + ((float)analogRead(4) * 0.01);
    }
    loop_counter = 0; //Reset the loop counter.
    print_intro();    //Print the intro to the serial monitor.
}
void check_imu_angles(void)
{
    uint8_t first_angle = 0;
    loop_counter = 0;
    first_angle = false; //If manual calibration is not used.
    cal_int = 0;

    while (data != 'q')
    {                                 //Stay in this loop until the data variable data holds a q.
        loop_timer = micros() + 4000; //Set the loop_timer variable to the current micros() value +4000
        if (Serial.available() > 0)
        { //Serial data available
            data = Serial.read();
            delay(100); //wait for other bytes
            while (Serial.available() > 0)
                loop_counter = Serial.read(); //Empty the Serial buffer.
        }
        if (cal_int == 0)
        {                         //if manual calibration is not used
            gyro_axis_cal[1] = 0; //Reset calibration variables for next calibration.
            gyro_axis_cal[2] = 0; //Reset calibration variables for next calibration.
            gyro_axis_cal[3] = 0; //Reset calibration variables for next calibration.
            Serial.print("Calibrating the gyro");

            //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).

            for (cal_int = 0; cal_int < 2000; cal_int++)
            { //Take 2000 readings
                if (cal_int % 125 == 0)
                {
                    digitalWrite(PB3, !digitalRead(PB3)); //Change the lead status to indicate calibartion
                    Serial.print(".");
                }
                gyro_signalen(); //Read the gyro output.
                gyro_axis_cal[1] += gyro_axis[1];
                gyro_axis_cal[2] += gyro_axis[2];
                gyro_axis_cal[3] += gyro_axis[3];
                delay(4); //Delay to make 250Hz calibration
            }
            Serial.println(".");
            green_led(LOW);

            //Now we have 2000 values, divide by 2000 to get average for each axis.

            gyro_axis_cal[1] /= 2000;
            gyro_axis_cal[2] /= 2000;
            gyro_axis_cal[3] /= 2000;
        }

        gyro_signalen(); //Collect current gyro data

        //Gyro angle calculations
        //0.0000611 = 1/ (250Hz/65.5)
        angle_pitch += gyro_axis[2] * 0.0000611; //Calculate pitch angle and add to total pitch
        angle_roll += gyro_axis[1] * 0.0000611;  //Calculate roll angle and add to total roll

        //0.000001066 = 0.0000611 * (3.42 PI / 180 degree)  The Arduino sin function is in radians
        angle_pitch -= angle_roll * sin(gyro_axis[3] * 0.000001066); //If hte IMU has yawed transfer the roll angle to the pitch angle.
        angle_roll += angle_pitch * sin(gyro_axis[3] * 0.000001066); //If the IMU has yawed transfer the pitch angle to the roll angle.

        //Accelerometer angle calculations
        if (acc_axis[1] > 4096)
            acc_axis[1] = 4096; //Limit accelerometer value.
        if (acc_axis[2] < -4096)
            acc_axis[2] = -4096; //Limit accelerometer value.
        if (acc_axis[3] > 4096)
            acc_axis[3] = 4096; //Limit accelerometer value.
        if (acc_axis[4] < -4096)
            acc_axis[4] = -4096; //Limit accelerometer value.

        //57.296 = 1 /(3.142/180) The Arduino arc sin function is in radians, convert to degrees
        angle_pitch_acc = asin((float)acc_axis[1] / 4096) * 57.296; //Calculate pitch angle
        angle_roll_acc = asin((float)acc_axis[2] / 4096) * 57.296;

        if (!first_angle)
        {                                  //When it is first time.
            angle_pitch = angle_pitch_acc; //Set the pitch angle to the accelerometer angle
            angle_roll = angle_roll_acc;   //Set the roll angle to the accerometer angle
            first_angle = true;
        }
        else
        { //Complementary filter for pitch and roll angles
            angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
            angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
        }

        //We can't print all the data at once. This takes to long and the angular readings will be off.
        if (loop_counter == 0)
            Serial.print("Pitch: ");
        if (loop_counter == 1)
            Serial.print(angle_pitch, 1);
        if (loop_counter == 2)
            Serial.print(" Roll: ");
        if (loop_counter == 3)
            Serial.print(angle_roll, 1);
        if (loop_counter == 4)
            Serial.print(" Yaw: ");
        if (loop_counter == 5)
            Serial.print(gyro_axis[3] / 65.5, 0);
        if (loop_counter == 6)
            Serial.print(" Temp: ");
        if (loop_counter == 7)
            Serial.println(temperature / 340.0 + 35.0, 1);

        loop_counter++;

        if (loop_counter == 60)
            loop_counter = 0;

        while (loop_timer > micros())
            ;
    }
    loop_counter = 0;
    print_intro();
}

void check_motor_vibrations(void)
{
    //Let's declare some variables so we can use them in this subroutine.
    //int16_t = signed 16 bit integer
    //uint16_t = unsigned 16 bit integer
    int32_t vibration_array[20], avarage_vibration_level, vibration_total_result;
    uint8_t array_counter, throttle_init_ok, vibration_counter;
    uint32_t wait_timer;
    throttle_init_ok = 0;
    while (data != 'q')
    {                                 //Stay in this loop until the data variable data holds a q.
        loop_timer = micros() + 4000; //Set the loop_timer variable to the current micros() value + 4000.
        if (Serial.available() > 0)
        {                         //If serial data is available
            data = Serial.read(); //Read the incomming byte
            delay(100);           //Wait for any other bytes to come in
            while (Serial.available() > 0)
                loop_counter = Serial.read(); //Empty the Serial buffer
        }

        if (throttle_init_ok)
        {                    //If the throttle is detected in the lowest position.
            gyro_signalen(); //Get the raw gyro and accelerometer data.
            //Calculate the total accelerometer vector.
            vibration_array[0] = sqrt((acc_axis[1] * acc_axis[1]) + (acc_axis[2] * acc_axis[2]) + (acc_axis[3] * acc_axis[3]));

            for (array_counter = 16; array_counter > 0; array_counter--)
            {                                                                        //Do this loop 16 times to create an array of accelrometer vectors.
                vibration_array[array_counter] = vibration_array[array_counter - 1]; //Shift every variable one position up in the array.
                avarage_vibration_level += vibration_array[array_counter];           //Add the array value to the acc_av_vector variable.
            }
            avarage_vibration_level /= 17; //Divide the acc_av_vector by 17 to get the avarage total accelerometer vector.

            if (vibration_counter < 20)
            {                                                                                //If the vibration_counter is less than 20 do this.
                vibration_counter++;                                                         //Increment the vibration_counter variable.
                vibration_total_result += abs(vibration_array[0] - avarage_vibration_level); //Add the absolute difference between the avarage vector and current vector to the vibration_total_result variable.
            }
            else
            {
                vibration_counter = 0;                       //If the vibration_counter is equal or larger than 20 do this.
                Serial.println(vibration_total_result / 50); //Print the total accelerometer vector divided by 50 on the serial monitor.
                vibration_total_result = 0;                  //Reset the vibration_total_result variable.
            }

            if (data == '1')
            {                                  //If the user requested 1.
                TIMER2_BASE->CCR1 = channel_3; //Set the ESC 1 output pulse equal to the throttle input.
                TIMER2_BASE->CCR2 = 1000;      //Keep the ESC 2 pulse at 1000us.
                TIMER2_BASE->CCR3 = 1000;      //Keep the ESC 3 pulse at 1000us.
                TIMER2_BASE->CCR4 = 1000;      //Keep the ESC 4 pulse at 1000us.
            }
            if (data == '2')
            {                                  //If the user requested 2.
                TIMER2_BASE->CCR1 = 1000;      //Keep the ESC 1 pulse at 1000us.
                TIMER2_BASE->CCR2 = channel_3; //Set the ESC 2 output pulse equal to the throttle input.
                TIMER2_BASE->CCR3 = 1000;      //Keep the ESC 3 pulse at 1000us.
                TIMER2_BASE->CCR4 = 1000;      //Keep the ESC 4 pulse at 1000us.
            }
            if (data == '3')
            {                                  //If the user requested 3.
                TIMER2_BASE->CCR1 = 1000;      //Keep the ESC 1 pulse at 1000us.
                TIMER2_BASE->CCR2 = 1000;      //Keep the ESC 2 pulse at 1000us.
                TIMER2_BASE->CCR3 = channel_3; //Set the ESC 3 output pulse equal to the throttle input.
                TIMER2_BASE->CCR4 = 1000;      //Keep the ESC 4 pulse at 1000us.
            }
            if (data == '4')
            {                                  //If the user requested 4.
                TIMER2_BASE->CCR1 = 1000;      //Keep the ESC 1 pulse at 1000us.
                TIMER2_BASE->CCR2 = 1000;      //Keep the ESC 2 pulse at 1000us.
                TIMER2_BASE->CCR3 = 1000;      //Keep the ESC 3 pulse at 1000us.
                TIMER2_BASE->CCR4 = channel_3; //Set the ESC 4 output pulse equal to the throttle input.
            }
            if (data == '5')
            {                                  //If the user requested 5.
                TIMER2_BASE->CCR1 = channel_3; //Set the ESC 1 output pulse equal to the throttle input.
                TIMER2_BASE->CCR2 = channel_3; //Set the ESC 2 output pulse equal to the throttle input.
                TIMER2_BASE->CCR3 = channel_3; //Set the ESC 3 output pulse equal to the throttle input.
                TIMER2_BASE->CCR4 = channel_3; //Set the ESC 4 output pulse equal to the throttle input.
            }
        }
        else
        {                                  //If the throttle is detected in the lowest position.
            wait_timer = millis() + 10000; //Set the wait_timer variable to the current millis() value incremented by 10 seconds.
            if (channel_3 > 1050)
            {                                                                 //If the trottle channel is not in the lowest position.
                Serial.println(F("Throttle is not in the lowest position.")); //Print a message on the serial monitor.
                Serial.print(F("Throttle value is: "));
                Serial.println(channel_3);
                Serial.print(F(""));
                Serial.print(F("Waiting for 10 seconds:"));
            }
            while (wait_timer > millis() && !throttle_init_ok)
            { //Stay in this wait loop for 10 seconds.
                if (channel_3 < 1050)
                    throttle_init_ok = 1; //If the throttle is in the lowest position set the throttle_init_ok variable.
                delay(500);               //Wait for 500 milliseconds.
                Serial.print(F("."));     //Print a dot to show something is still working.
            }
        }
        if (!throttle_init_ok)
        { //If the throttle is not detected low after the 10 seconds quit this loop and return to the main menu.
            data = 'q';
        }
        while (loop_timer > micros())
            ; //Create a 250Hz loop.
    }
    loop_counter = 0; //Reset the loop counter variable to 0.
    print_intro();    //Print the intro to the serial monitor.
}
void i2c_scanner(void)
{
    //Let's declare some variables so we can use them in this subroutine.
    Serial.println("The YMFC-32 needs the following devices:");
    Serial.println("0x68 = MPU-6050 gyro/accelerometer");
    Serial.println("They should appear in the list below.");
    Serial.println("");

    data = 0;
    uint8_t error, address, done;
    uint16_t nDevices;
    Serial.println("Scanning address 1 till 127...");
    Serial.println("");
    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        HWire.beginTransmission(address);
        error = HWire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    done = 1;
    if (nDevices == 0)
        Serial.println("No I2C devices found");
    else
        Serial.println("done");
    delay(2000);
    print_intro();
}
void print_intro(void)
{
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("          YMFC-32 quadcopter setup tool"));
    Serial.println(F("==================================================="));
    Serial.println(F("a = Read the receiver input pulses"));
    Serial.println(F("b = I2C scanner to detect any I2C sensors attached"));
    Serial.println(F("c = Read the raw gyro values"));
    Serial.println(F("d = Read the raw accelerometer values"));
    Serial.println(F("e = Check the IMU angles"));
    Serial.println(F("f = Test the LEDs"));
    Serial.println(F("g = Read the battery voltage input"));
    Serial.println(F("h = Check barometer"));
    Serial.println(F("i = Check GPS"));
    Serial.println(F("j = Check HMC5883L compass"));
    Serial.println(F("==================================================="));
    Serial.println(F("1 = Check motor 1 (front right, counter clockwise direction)"));
    Serial.println(F("2 = Check motor 2 (rear right, clockwise direction)"));
    Serial.println(F("3 = Check motor 3 (rear left, counter clockwise direction)"));
    Serial.println(F("4 = Check motor 4 (front left, clockwise direction)"));
    Serial.println(F("5 = Check all motors"));
    Serial.println(F("==================================================="));
    Serial.println(F("For support and questions: www.brokking.net"));
    Serial.println(F(""));
    if (!disable_throttle)
    { //If the throttle is not disabled.
        Serial.println(F("==================================================="));
        Serial.println(F("     WARNING >>>THROTTLE IS ENABLED<<< WARNING"));
        Serial.println(F("==================================================="));
    }
}
void read_gyro_values(void)
{
    cal_int = 0; //If manual calibration is not used.

    while (data != 'q')
    {
        delay(250);
        if (Serial.available() > 0)
        {
            data = Serial.read();
            delay(100);
            while (Serial.available() > 0)
                loop_counter = Serial.read();
        }
        if (data == 'c')
        { //If the user requested a 'c'.
            if (cal_int != 2000)
            {
                gyro_axis_cal[1] = 0;
                gyro_axis_cal[2] = 0;
                gyro_axis_cal[3] = 0; //Reset calibration variables for next calibration.
                Serial.print(".");

                //Let's take multiple gyro data samples so we can determine the average gyro offest (calibration)

                for (cal_int = 0; cal_int < 2000; cal_int++)
                {
                    if (cal_int % 125 == 0)
                    {
                        digitalWrite(PB3, !digitalRead(PB3));
                        Serial.print(".");
                    }
                    gyro_signalen();                  //Read the gyro output
                    gyro_axis_cal[1] += gyro_axis[1]; //Add roll value to gyro_roll_cal
                    gyro_axis_cal[2] += gyro_axis[2]; //Add pitch value to gyro_pitch_cal
                    gyro_axis_cal[3] += gyro_axis[3]; //Add yaw value to gyro_yaw_cal.
                    delay(4);
                }

                Serial.println(".");

                //Now that we have 2000 measure, we need to divide by 2000 to get average gyro offset.
                gyro_axis_cal[1] /= 2000;
                gyro_axis_cal[2] /= 2000;
                gyro_axis_cal[3] /= 2000;

                Serial.print("X calibration value:");
                Serial.println(gyro_axis_cal[1]);
                Serial.print("Y calibration value:");
                Serial.println(gyro_axis_cal[2]);
                Serial.print("Z calibration value:");
                Serial.println(gyro_axis_cal[3]);
            }
            gyro_signalen(); //Read gyro output
            Serial.print("Gyro_x = ");
            Serial.print(gyro_axis[1]);
            Serial.print(" Gyro_y = ");
            Serial.print(gyro_axis[2]);
            Serial.print(" Gyro_z = ");
            Serial.println(gyro_axis[3]);
        }
        else
        {
            gyro_signalen();
            Serial.print("ACC_x = ");
            Serial.print(acc_axis[1]);
            Serial.print(" ACC_y = ");
            Serial.print(acc_axis[2]);
            Serial.print(" ACC_z = ");
            Serial.println(acc_axis[3]);
        }
    }
    print_intro();
}
void test_leds(void)
{
    data = 0;
    if (Serial.available() > 0)
    {                         //If serial data is available
        data = Serial.read(); //Read the incomming byte
        delay(100);           //Wait for any other bytes to come in
        while (Serial.available() > 0)
            loop_counter = Serial.read(); //Empty the Serial buffer
    }
    Serial.println(F("The red LED is now ON for 3 seconds"));
    red_led(HIGH); //Set output PB4 high.
    delay(3000);
    Serial.println(F(""));
    Serial.println(F("The green LED is now ON for 3 seconds"));
    red_led(LOW);    //Set output PB4 low.
    green_led(HIGH); //Set output PB3 high.
    delay(3000);
    green_led(LOW); //Set output PB3 low.
    print_intro();
}

void manual_imu_calibration(void)
{
    data = 0;
    acc_axis_cal[1] = 0;
    acc_axis_cal[2] = 0;
    gyro_axis_cal[1] = 0;
    gyro_axis_cal[2] = 0;
    gyro_axis_cal[3] = 0;
    Serial.print("Calibrating the gyro");
    //Let's take multiple gyro data samples to stabilize the gyro.
    for (cal_int = 0; cal_int < 2000; cal_int++)
    { //Take 2000 readings for calibration.
        if (cal_int % 125 == 0)
        {
            digitalWrite(PB3, !digitalRead(PB3)); //Change the led status to indicate calibration.
            Serial.print(".");
        }
        gyro_signalen(); //Read the gyro output.
        delay(4);        //Small delay to simulate a 250Hz loop during calibration.
    }

    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 4000; cal_int++)
    { //Take 2000 readings for calibration.
        if (cal_int % 125 == 0)
        {
            digitalWrite(PB3, !digitalRead(PB3)); //Change the led status to indicate calibration.
            Serial.print(".");
        }
        gyro_signalen();                                                //Read the gyro output.
        acc_axis_cal[1] += acc_axis[1] + manual_acc_pitch_cal_value;    //Ad the Y accelerometer value to the calibration value.
        acc_axis_cal[2] += acc_axis[2] + manual_acc_roll_cal_value;     //Ad the X accelerometer value to the calibration value.
        gyro_axis_cal[1] += gyro_axis[1] + manual_gyro_roll_cal_value;  //Ad roll value to gyro_roll_cal.
        gyro_axis_cal[2] += gyro_axis[2] + manual_gyro_pitch_cal_value; //Ad pitch value to gyro_pitch_cal.
        gyro_axis_cal[3] += gyro_axis[3] + manual_gyro_yaw_cal_value;   //Ad yaw value to gyro_yaw_cal.
        delay(4);                                                       //Small delay to simulate a 250Hz loop during calibration.
    }
    Serial.println(".");
    green_led(LOW); //Set output PB3 low.
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    acc_axis_cal[1] /= 4000;  //Divide the accelerometer Y value by 4000.
    acc_axis_cal[2] /= 4000;  //Divide the accelerometer X value by 4000.
    gyro_axis_cal[1] /= 4000; //Divide the roll total by 4000.
    gyro_axis_cal[2] /= 4000; //Divide the pitch total by 4000.
    gyro_axis_cal[3] /= 4000; //Divide the yaw total by 4000.

    //Print the calibration values on the serial monitor.
    Serial.print("manual_acc_pitch_cal_value = ");
    Serial.println(acc_axis_cal[1]);
    Serial.print("manual_acc_roll_cal_value = ");
    Serial.println(acc_axis_cal[2]);
    Serial.print("manual_gyro_pitch_cal_value = ");
    Serial.println(gyro_axis_cal[2]);
    Serial.print("manual_gyro_roll_cal_value = ");
    Serial.println(gyro_axis_cal[1]);
    Serial.print("manual_gyro_yaw_cal_value = ");
    Serial.println(gyro_axis_cal[3]);

    print_intro(); //Print the intro to the serial monitor.
}