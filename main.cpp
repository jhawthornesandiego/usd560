/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/services/HeartRateService.h"
#include "algorithm.h"
#include "MAX30102.h"

#define MAX_BRIGHTNESS 255



DigitalOut led2(LED2, 1);

uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
int i;
int32_t n_brightness;
float f_temp;
uint8_t hrmCounter;


#ifdef TARGET_MAX32630FTHR
PwmOut led1(LED_RED);    //initializes the pwm output that connects to the on board LED
DigitalIn INT(P3_0);  //pin P30 connects to the interrupt output pin of the MAX30102
#endif

const static char     DEVICE_NAME[] = "HRM";
static const uint16_t uuid16_list[] = {GattService::UUID_HEART_RATE_SERVICE};

static HeartRateService *hrServicePtr;

static EventQueue eventQueue(/* event count */ 16 * EVENTS_EVENT_SIZE);

void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params)
{
    BLE::Instance().gap().startAdvertising(); // restart advertising
}

void checkHR(){
    
    i=0;
    un_min=0x3FFFF;
    un_max=0;
        
    //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
    for(i=100;i<500;i++)
    {
        aun_red_buffer[i-100]=aun_red_buffer[i];
        aun_ir_buffer[i-100]=aun_ir_buffer[i];
            
        //update the signal min and max
        if(un_min>aun_red_buffer[i])
        un_min=aun_red_buffer[i];
        if(un_max<aun_red_buffer[i])
        un_max=aun_red_buffer[i];
    }
        
    //take 100 sets of samples before calculating the heart rate.
    for(i=400;i<500;i++)
    {
        un_prev_data=aun_red_buffer[i-1];
        while(INT.read()==1);
        maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
        
        if(aun_red_buffer[i]>un_prev_data)
        {
            f_temp=aun_red_buffer[i]-un_prev_data;
            f_temp/=(un_max-un_min);
            f_temp*=MAX_BRIGHTNESS;
            n_brightness-=(int)f_temp;
            if(n_brightness<0)
                n_brightness=0;
        }
        else
        {
            f_temp=un_prev_data-aun_red_buffer[i];
            f_temp/=(un_max-un_min);
            f_temp*=MAX_BRIGHTNESS;
            n_brightness+=(int)f_temp;
            if(n_brightness>MAX_BRIGHTNESS)
                n_brightness=MAX_BRIGHTNESS;
        }
#if defined(TARGET_KL25Z) || defined(TARGET_MAX32630FTHR)
        led1.write(1-(float)n_brightness/256);
#endif


    }
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
}

void updateSensorValue() {
    // Do blocking calls or whatever is necessary for sensor polling.
    // In our case, we simply update the HRM measurement.

    
    checkHR();
            
    hrmCounter = n_heart_rate;
        
    hrServicePtr->updateHeartRate(hrmCounter);
}

void periodicCallback(void)
{

    if (BLE::Instance().getGapState().connected) {
        eventQueue.call(updateSensorValue);
    }
}



void onBleInitError(BLE &ble, ble_error_t error)
{
    (void)ble;
    (void)error;
   /* Initialization error handling should go here */
}

void printMacAddress()
{
    /* Print out device MAC address to the console*/
    Gap::AddressType_t addr_type;
    Gap::Address_t address;
    BLE::Instance().gap().getAddress(&addr_type, address);
    printf("DEVICE MAC ADDRESS: ");
    for (int i = 5; i >= 1; i--){
        printf("%02x:", address[i]);
    }
    printf("%02x\r\n", address[0]);
}

void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE&        ble   = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        onBleInitError(ble, error);
        return;
    }

    if (ble.getInstanceID() != BLE::DEFAULT_INSTANCE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);

    /* Setup primary service. */
    hrServicePtr = new HeartRateService(ble, hrmCounter, HeartRateService::LOCATION_FINGER);

    /* Setup advertising. */
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list));
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::GENERIC_HEART_RATE_SENSOR);
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.gap().setAdvertisingInterval(1000); /* 1000ms */
    ble.gap().startAdvertising();

    printMacAddress();
}

void scheduleBleEventsProcessing(BLE::OnEventsToProcessCallbackContext* context) {
    BLE &ble = BLE::Instance();
    eventQueue.call(Callback<void()>(&ble, &BLE::processEvents));
}

int main()
{
    maxim_max30102_reset(); //resets the MAX30102

    
    //read and clear status register
    maxim_max30102_read_reg(0,&uch_dummy);
    
    
    maxim_max30102_init();  //initializes the MAX30102
    
    n_brightness=0;
    un_min=0x3FFFF;
    un_max=0;
    
    n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
    
    //read the first 500 samples, and determine the signal range
    for(i=0;i<n_ir_buffer_length;i++)
    {
        while(INT.read()==1);   //wait until the interrupt pin asserts
        
        maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
            
        if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];    //update signal min
        if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];    //update signal max
            
    }
    un_prev_data=aun_red_buffer[i];
    
    //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    
    
    while(1)
    {
        
        eventQueue.call_every(500, periodicCallback);
 
        BLE &ble = BLE::Instance();
        ble.onEventsToProcess(scheduleBleEventsProcessing);
        ble.init(bleInitComplete);
        
        eventQueue.dispatch_forever();
             
        
    }
}
