//By Nicholas Danas and Curran Flanders

#include "engr2350_msp432.h"

void GPIOInit();
void TimerInit();
void Encoder_ISR();
void T2_100ms_ISR();
void I2C_Init();
uint16_t readCompass(void);
uint16_t readCompass2(void);
int16_t tiltCompass(void);
int16_t readAccel(void);
Timer_A_UpModeConfig TA0cfg; // PWM timer
Timer_A_UpModeConfig TA2cfg; // 100 ms timer
Timer_A_ContinuousModeConfig TA3cfg; // Encoder timer
Timer_A_CompareModeConfig TA0_ccr3; // PWM Right
Timer_A_CompareModeConfig TA0_ccr4; // PWM Left
Timer_A_CaptureModeConfig TA3_ccr0; // Encoder Right
Timer_A_CaptureModeConfig TA3_ccr1; // Encoder Left
eUSCI_I2C_MasterConfig i2cConfig; // I2C


// Encoder total events
uint32_t enc_total_L,enc_total_R;
// Speed measurement variables
// Note that "Tach" stands for "Tachometer," or a device used to measure rotational speed
int32_t Tach_L_count,Tach_L,Tach_L_sum,Tach_L_sum_count,Tach_L_avg; // Left wheel
int32_t Tach_R_count,Tach_R,Tach_R_sum,Tach_R_sum_count,Tach_R_avg; // Right wheel
    // Tach_L,Tach_R are equivalent to enc_counts from Activity 10/Lab 3
    // Tach_L/R_avg is the averaged Tach_L/R value after every 12 encoder measurements
    // The rest are the intermediate variables used to assemble Tach_L/R_avg

uint8_t run_control = 0; // Flag to denote that 100ms has passed and control should be run.
uint16_t CompareValueLeft = 0;
uint16_t CompareValueRight = 0;
uint16_t heading = 0;
uint16_t heading2 = 0;
uint16_t acc_data = 0;
uint16_t desired_heading = 0;
uint16_t measured_heading = 0;
int16_t heading_error = 0;
int16_t previous_error = 0;
uint8_t calibration[2];
uint8_t calibration2[2];
uint8_t calib_flag = 0;
uint8_t calib_flag2 = 0;

int32_t desiredSpeed = 0;
float desiredLeft = 0;
float desiredRight = 0;
float totalErrorLeft = 0;
float totalErrorRight = 0;
float errorLeft = 0;
float errorRight = 0;
float kI = 0.01;
float kP = 0.01;
float kD = 0.0001;

float measuredSpeedLeft = 0;
float measuredSpeedRight = 0;
float controlling = 0;
float differentialSpeed = 0;
float correctedSpeedRight = 0;
float correctedSpeedLeft = 0;
float vmax = 7.45;
float absDesiredLeftPWM = 0;
float absDesiredRightPWM = 0;
float theta = 0;
float absTheta = 0;
int main(void)
{
    SysInit();
    GPIOInit();
    TimerInit();
    I2C_Init();

    __delay_cycles(24e6);

    while(!calib_flag || !calib_flag2){

        I2C_readData(EUSCI_B1_BASE, 0x60, 29, calibration, 2);
        //printf("%u\r\n", calibration[1]);
        if ((calibration[1] & 0x03)==3){
            //printf("%u\r\n", calibration[0]);
            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN0);
            calib_flag = 1;
        }

        I2C_readData(EUSCI_B1_BASE, 0x63, 29, calibration2, 2);
        if ((calibration2[1] & 0x03)==3){
            GPIO_setOutputHighOnPin(GPIO_PORT_P2,GPIO_PIN0);
            calib_flag2 = 1;
        }



    }




    while(1){


        if(run_control){    // If 100 ms has passed

            run_control = 0;    // Reset the 100 ms flag
            // Control routine ... Explicitly follow pseudocode from Lab document

            desired_heading = readCompass2();
            measured_heading = readCompass();
            desiredSpeed = ((tiltCompass())*5)/9;
            printf("desiredSpeed: %i\r\n", desiredSpeed);


            heading_error = desired_heading - measured_heading;

            if (heading_error > 1800){
                heading_error -= 3600;
            }
            if ( heading_error < -1800){
                heading_error += 3600;
            }

            differentialSpeed = (kP * heading_error) + kD * (heading_error - previous_error);
            //differentialSpeed = 0;



            if (differentialSpeed > 20){
                differentialSpeed = 20;
            }

            if (differentialSpeed < -20){
                differentialSpeed = -20;
            }

            previous_error = heading_error;

            // Mapping ADC value to desired speed
            // Assuming the ADC range is 0 to 16383 (14-bit) and the speed is mapped between -50% to +50% PWM duty cycle
            /*desiredSpeed = ((float)adcSpeed * 100 / 16383) - 50; // Convert to -50 to 50 range
            theta = ((float)adcTurn * 180 / 16383) - 90;
            if (theta < 0){
                absTheta = (-1*theta);
            }
            else{
                absTheta = theta;
            }

            if (absTheta < 15){
                differentialSpeed = 0;
            }
            else{
                differentialSpeed = vmax*(absTheta-15)/75;
            }
            */

            //Starting wheel speed for the left wheel


            if (theta < 0){
                desiredLeft = desiredSpeed - differentialSpeed;
                desiredRight = desiredSpeed + differentialSpeed;
            }else{
                desiredLeft = desiredSpeed + differentialSpeed;
                desiredRight = desiredSpeed - differentialSpeed;
            }
            printf("desiredLeft %1.3f\r\n",desiredLeft);


            //Left Motor
            if (desiredLeft < 0){
            absDesiredLeftPWM = (-1*desiredLeft);
            }
            else{
                absDesiredLeftPWM = desiredLeft;
            }

            if (desiredRight < 0){
                absDesiredRightPWM = (-1*desiredRight);
            }
            else{
            absDesiredRightPWM = desiredRight;
                        }

            printf("absDesiredLeftPWM %1.3f\r\n",absDesiredLeftPWM);

            if (absDesiredLeftPWM  < 5){
                CompareValueLeft = 0;

            }else {

                if (desiredLeft < 0){
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN4);

                }
                else{
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4);

                }
                measuredSpeedLeft = 1500000/Tach_L_avg;
                errorLeft = (absDesiredLeftPWM) - measuredSpeedLeft ;    //???
                totalErrorLeft += errorLeft;
                correctedSpeedLeft = (absDesiredLeftPWM) +(kI * totalErrorLeft);//?


                CompareValueLeft = correctedSpeedLeft * 999/100;
                if (CompareValueLeft < 100){
                    CompareValueLeft = 100;

               }else if(CompareValueLeft > 900){
                    CompareValueLeft = 900;
                }

            }

            TA0_ccr4.compareValue = CompareValueLeft;
            Timer_A_initCompare(TIMER_A0_BASE, &TA0_ccr4);



            //Right Motor
            if (absDesiredRightPWM  < 5){
                CompareValueRight = 0;

            }else {


                if (desiredRight < 0){
                    GPIO_setOutputHighOnPin(GPIO_PORT_P5,GPIO_PIN5);

                }
                else{
                    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN5);

                }
                measuredSpeedRight = 1500000/Tach_R_avg;
                errorRight = (absDesiredRightPWM) - measuredSpeedRight ;
                totalErrorRight += errorRight;
                correctedSpeedRight = (absDesiredRightPWM) +(kI * totalErrorRight);
                printf("desiredLeft final %1.3f\r\n",desiredLeft);
                printf("absDesiredLeftPWM final %1.3f\r\n",absDesiredLeftPWM);


                CompareValueRight = correctedSpeedRight * 999/100;
                if (CompareValueRight < 100){
                    CompareValueRight = 100;

                }else if(CompareValueRight > 900){
                    CompareValueRight = 900;
                }

            }

            TA0_ccr3.compareValue = CompareValueRight;
            Timer_A_initCompare(TIMER_A0_BASE, &TA0_ccr3);


            //printf("%1.3f-----------%u\r\n", desiredLeft, CompareValueLeft*100/999);
            //printf("%1.3f-----------%1.3f\r\n", desiredLeft, correctedSpeedLeft);
            //printf("%1.3f \r\n", measuredSpeedLeft);
            //printf("%1.3f         %1.3f         %1.3u          %1.3u\r\n", desiredLeft, desiredRight, CompareValueLeft, CompareValueRight);
            //printf("%1.3f           %1.3f           %1.3f\r\n", desiredSpeed, theta, differentialSpeed);

        }
    }
}



void GPIOInit(){
    GPIO_setAsOutputPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motor direction pins
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6|GPIO_PIN7);   // Motor enable pins
    GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN0);


        // Motor PWM pins
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6|GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
        // Motor Encoder pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10,GPIO_PIN4|GPIO_PIN5,GPIO_PRIMARY_MODULE_FUNCTION);

    // SDA and SCL
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setOutputLowOnPin(GPIO_PORT_P5,GPIO_PIN4|GPIO_PIN5);   // Motors set to forward
       // Motors are OFF
    GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3,GPIO_PIN6);
    //while(1);
}

void TimerInit(){
    // Configure PWM timer for 30 kHz
    TA0cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA0cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA0cfg.timerPeriod = 999;
    Timer_A_configureUpMode(TIMER_A0_BASE,&TA0cfg);
    // Configure TA0.CCR3 for PWM output, Right Motor
    TA0_ccr3.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
    TA0_ccr3.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr3.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr3);
    // Configure TA0.CCR4 for PWM output, Left Motor
    TA0_ccr4.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
    TA0_ccr4.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    TA0_ccr4.compareValue = 0;
    Timer_A_initCompare(TIMER_A0_BASE,&TA0_ccr4);
    // Configure Encoder timer in continuous mode
    TA3cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA3cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    TA3cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    Timer_A_configureContinuousMode(TIMER_A3_BASE,&TA3cfg);
    // Configure TA3.CCR0 for Encoder measurement, Right Encoder
    TA3_ccr0.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    TA3_ccr0.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr0.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr0.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr0.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr0);
    // Configure TA3.CCR1 for Encoder measurement, Left Encoder
    TA3_ccr1.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    TA3_ccr1.captureMode = TIMER_A_CAPTUREMODE_RISING_EDGE;
    TA3_ccr1.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    TA3_ccr1.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    TA3_ccr1.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    Timer_A_initCapture(TIMER_A3_BASE,&TA3_ccr1);
    // Register the Encoder interrupt
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCR0_INTERRUPT,Encoder_ISR);
    Timer_A_registerInterrupt(TIMER_A3_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,Encoder_ISR);
    // Configure 10 Hz timer
    TA2cfg.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    TA2cfg.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_64;
    TA2cfg.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    TA2cfg.timerPeriod = 18750;
    Timer_A_configureUpMode(TIMER_A2_BASE,&TA2cfg);
    Timer_A_registerInterrupt(TIMER_A2_BASE,TIMER_A_CCRX_AND_OVERFLOW_INTERRUPT,T2_100ms_ISR);
    // Start all the timers
    Timer_A_startCounter(TIMER_A0_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A2_BASE,TIMER_A_UP_MODE);
    Timer_A_startCounter(TIMER_A3_BASE,TIMER_A_CONTINUOUS_MODE);
}


void Encoder_ISR(){
    // If encoder timer has overflowed...
    if(Timer_A_getEnabledInterruptStatus(TIMER_A3_BASE) == TIMER_A_INTERRUPT_PENDING){
        Timer_A_clearInterruptFlag(TIMER_A3_BASE);
        Tach_R_count += 65536;
        if(Tach_R_count >= 1e6){ // Enforce a maximum count to Tach_R so stopped can be detected
            Tach_R_count = 1e6;
            Tach_R = 1e6;
        }
        Tach_L_count += 65536;
        if(Tach_L_count >= 1e6){ // Enforce a maximum count to Tach_L so stopped can be detected
            Tach_L_count = 1e6;
            Tach_L = 1e6;
        }
    // Otherwise if the Left Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        enc_total_R++;   // Increment the total number of encoder events for the left encoder
        // Calculate and track the encoder count values
        Tach_R = Tach_R_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        Tach_R_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);
        // Sum values for averaging
        Tach_R_sum_count++;
        Tach_R_sum += Tach_R;
        // If 6 values have been received, average them.
        if(Tach_R_sum_count == 12){
            Tach_R_avg = Tach_R_sum/12;
            Tach_R_sum_count = 0;
            Tach_R_sum = 0;
        }
    // Otherwise if the Right Encoder triggered...
    }else if(Timer_A_getCaptureCompareEnabledInterruptStatus(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1)&TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG){
        Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        enc_total_L++;
        Tach_L = Tach_L_count + Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_L_count = -Timer_A_getCaptureCompareCount(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1);
        Tach_L_sum_count++;
        Tach_L_sum += Tach_L;
        if(Tach_L_sum_count == 12){
            Tach_L_avg = Tach_L_sum/12;
            Tach_L_sum_count = 0;
            Tach_L_sum = 0;
        }
    }
}


void I2C_Init(){
    i2cConfig.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    i2cConfig.i2cClk = 24000000;
    i2cConfig.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
    i2cConfig.byteCounterThreshold = 0;
    i2cConfig.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

    //operate as I2C Master
    I2C_initMaster(EUSCI_B1_BASE, &i2cConfig);

    // Enable eUSCI_B1
    I2C_enableModule(EUSCI_B1_BASE);
}

uint16_t readCompass(void){
    uint8_t comp[2]; // array for byts of registers 2 and 3

    I2C_readData(EUSCI_B1_BASE, 0x60, 2, comp, 2 );

    //shifting this 8-bit value left by 8 bits to then be able to combine it with data 1 another 8 bit value
    heading = comp[0];
    heading=heading  << 8;
    heading = heading+comp[1];
    return heading;
}


/*int16_t readAccel(void){
    int8_t acc[2];

    I2C_readData(EUSCI_B1_BASE, 0x60, 14, acc, 2 );

    acc_data = acc[0];
    acc_data = acc_data << 8;
    acc_data = acc_data + acc[1];
    return acc_data;
}

*/

uint16_t readCompass2(void){
    uint8_t comp2[2];

    I2C_readData(EUSCI_B1_BASE, 0x63, 2, comp2, 2 );

    heading2 = comp2[0];
    heading2=heading2  << 8;
    heading2 = heading2+comp2[1];
    return heading2;

}

int16_t tiltCompass(void){

    int8_t pitch[2];

    I2C_readData(EUSCI_B1_BASE, 0x63, 4, pitch, 2 );
    printf("pitch %i \r\n",pitch[0]);
    return pitch[0];
}



void T2_100ms_ISR(){
    Timer_A_clearInterruptFlag(TIMER_A2_BASE);
    run_control = 1;
}



