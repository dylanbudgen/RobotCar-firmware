#include "mbed.h"
#include "ble/BLE.h"
#include "HCSR04.h"

// Left motor
// NOTE - THE IN1 AND IN2 HAVE BEEN SWAPPED ON THE BOARD. THIS IS TO ACCOUNT FOR THE REVERSE ERROR.
DigitalIn motorL_CHB(D2); //P014
DigitalIn motorL_CHA(D3); //P015
DigitalOut motorL_SBY(D4); //P016
PwmOut motorL_PWM(D11);
DigitalOut motorL_IN2(D12); //P028
DigitalOut motorL_IN1(D13); //P029

// Right motor
DigitalIn motorR_CHB(p21); //P021
DigitalIn motorR_CHA(p22); //P022
DigitalOut motorR_SBY(p10); //P010
PwmOut motorR_PWM(p11); 
DigitalOut motorR_IN2(D0); //P012
DigitalOut motorR_IN1(D1); //P013

// Ultrasound sensors
HCSR04 leftSensor(A1, A0);
HCSR04 frontSensor(A5, A4);
HCSR04 rightSensor(A3, A2);

// Serial for printing to terminal
Serial pc(USBTX, USBRX);

// Ticker for the ultrasound check interrupts 
Ticker ultrasoundMonitor;

// Direction service and characteristic
uint16_t directionServiceUUID  = 0xA000;
uint16_t directionCharUUID  = 0xA002;

// Speed service and characteristic
uint16_t speedServiceUUID  = 0xA003;
uint16_t speedCharUUID  = 0xA004;

// Ultrasound service and characteristic
uint16_t ultrasoundServiceUUID    = 0xA005;
uint16_t ultrasoundLeftCharUUID    = 0xA007;
uint16_t ultrasoundFrontCharUUID    = 0xA006;
uint16_t ultrasoundRightCharUUID    = 0xA008;

// Directions
const static int STOP = 0;
const static int FORWARD = 1;
const static int LEFT = 2;
const static int RIGHT = 3;
const static int BACKWARD = 4;

// Values default values for speed and direction
volatile int direction = 0;
volatile float speed = 0.5;

// Ultrasound values
volatile uint16_t ultrasoundLeft = 0;
volatile uint16_t ultrasoundFront = 0;
volatile uint16_t ultrasoundRight = 0;

// Device name and UUID
const static char     DEVICE_NAME[]        = "BatMobile";
static const uint16_t uuid16_list[]        = {0xAF3D}; //Custom UUID, FFFF is reserved for development

// Setting up direction characteristics
static uint8_t directionValue[10] = {0};
WriteOnlyArrayGattCharacteristic<uint8_t, sizeof(directionValue)> directionChar(directionCharUUID, directionValue);

// Setting up speed characteristics
static uint8_t speedValue[10] = {0};
WriteOnlyArrayGattCharacteristic<uint8_t, sizeof(speedValue)> speedChar(speedCharUUID, speedValue);

// Setting up ultrasound characteristics
static uint8_t ultrasoundFrontValue[10] = {0};
ReadOnlyArrayGattCharacteristic<uint8_t, sizeof(ultrasoundFrontValue)> ultrasoundFrontChar(ultrasoundFrontCharUUID, ultrasoundFrontValue, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

static uint8_t ultrasoundLeftValue[10] = {0};
ReadOnlyArrayGattCharacteristic<uint8_t, sizeof(ultrasoundLeftValue)> ultrasoundLeftChar(ultrasoundLeftCharUUID, ultrasoundLeftValue, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

static uint8_t ultrasoundRightValue[10] = {0};
ReadOnlyArrayGattCharacteristic<uint8_t, sizeof(ultrasoundRightValue)> ultrasoundRightChar(ultrasoundRightCharUUID, ultrasoundRightValue, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

// Setting up direction service
GattCharacteristic *directionCharacteristics[] = {&directionChar};
GattService        directionService(directionServiceUUID, directionCharacteristics, sizeof(directionCharacteristics) / sizeof(GattCharacteristic *));

// Setting up speed service
GattCharacteristic *speedCharacteristics[] = {&speedChar};
GattService        speedService(speedServiceUUID, speedCharacteristics, sizeof(speedCharacteristics) / sizeof(GattCharacteristic *));

// Setting up ultrasound service
GattCharacteristic *ultrasoundCharacteristics[] = {&ultrasoundLeftChar, &ultrasoundFrontChar, &ultrasoundRightChar};
GattService        ultrasoundService(ultrasoundServiceUUID, ultrasoundCharacteristics, sizeof(ultrasoundCharacteristics) / sizeof(GattCharacteristic *));


/* Return the direction */
int getDirection() {
    return direction;
}

/* Set the direction */
void setDirection(int newDirection) {
    direction = newDirection;   
}

/* Return the speed */
float getSpeed() {
    return speed;
}

/* Set the speed */
void setSpeed(float newSpeed) {
    speed = newSpeed; 
}

/* Change the direction of the left wheel. */
void changeLeftWheel(int direction) {
    
    switch (direction) {
        case BACKWARD :
            motorL_SBY = 1; 
            motorL_PWM.write(getSpeed());
            motorL_IN1 = 1;
            motorL_IN2 = 0;            
            break;
        case STOP : 
            motorL_SBY = 1; 
            motorL_PWM.write(0);
            motorL_IN1 = 1;
            motorL_IN2 = 1; 
            break;
        case FORWARD : 
            motorL_SBY = 1; 
            motorL_PWM.write(getSpeed());
            motorL_IN1 = 0;
            motorL_IN2 = 1; 
            break;
    }
}

/* Change the direction of the right wheel. */
void changeRightWheel(int direction) {
    
    switch (direction) {
        case BACKWARD :
            motorR_SBY = 1; 
            motorR_PWM.write(getSpeed());
            motorR_IN1 = 1;
            motorR_IN2 = 0;            
            break;
        case STOP : 
            motorR_SBY = 1; 
            motorR_PWM.write(0);
            motorR_IN1 = 1;
            motorR_IN2 = 1; 
            break;
        case FORWARD : 
            motorR_SBY = 1; 
            motorR_PWM.write(getSpeed());
            motorR_IN1 = 0;
            motorR_IN2 = 1; 
            break;
    }
}

/* Restart advertising when phone app disconnects */
void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *)
{
    BLE::Instance(BLE::DEFAULT_INSTANCE).gap().startAdvertising();
}

/* Handle writes to writeCharacteristic  */
void writeCharCallback(const GattWriteCallbackParams *params) {
    
    // Check to see what value was updated
    if(params->handle == directionChar.getValueHandle()) {
            
        switch(params->data[0]) {
            case BACKWARD :
                changeLeftWheel(BACKWARD);
                changeRightWheel(BACKWARD);  
                break;
            case STOP :
                changeLeftWheel(STOP);
                changeRightWheel(STOP);
                break;
            case FORWARD :
                changeLeftWheel(FORWARD);
                changeRightWheel(FORWARD);
                break;
            case LEFT :
                changeLeftWheel(BACKWARD);
                changeRightWheel(FORWARD);                
                break;
            case RIGHT :
                changeLeftWheel(FORWARD);
                changeRightWheel(BACKWARD);                
                break;
        }
    }
    
    if(params->handle == speedChar.getValueHandle()) {
        //pc.printf("Changing speed setting \n\r");
            
        switch(params->data[0]) {
            case 1 :
                // Speed 1
                setSpeed(0.2);
                break;
            case 2 :
                // Speed 2
                setSpeed(0.5);
                break;
            case 3 :
                // Speed 3
                setSpeed(0.9);
                break;
        }
    }    
}


/* Initialization callback */
void bleInitComplete(BLE::InitializationCompleteCallbackContext *params)
{
    BLE &ble          = params->ble;
    ble_error_t error = params->error;

    if (error != BLE_ERROR_NONE) {
        return;
    }

    ble.gap().onDisconnection(disconnectionCallback);
    ble.gattServer().onDataWritten(writeCharCallback);

    // Setup advertising
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE); // BLE only, no classic BT
    ble.gap().setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED); // advertising type
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LOCAL_NAME, (uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME)); // add name
    ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_16BIT_SERVICE_IDS, (uint8_t *)uuid16_list, sizeof(uuid16_list)); // UUID's broadcast in advertising packet
    ble.gap().setAdvertisingInterval(100); // 100ms.

    // Add the direction service
    ble.addService(directionService);
    
    // Add the speed service
    ble.addService(speedService);

    // Add the ultrasound service
    ble.addService(ultrasoundService);

    // Start advertising
    ble.gap().startAdvertising();
    
}


/* Update the ultrasound characteristics if they have changed */
void updateUltrasoundValues()
{
    //pc.printf("Checking ultrasound distance\n\r");

    // Left ultrasound sensor
    uint16_t newUltrasoundLeft = leftSensor.distance(1);

    if (ultrasoundLeft != newUltrasoundLeft) {
        ultrasoundLeft = newUltrasoundLeft;
        //pc.printf("Left sensor recorded is:  %lu  \n\r", (unsigned long)ultrasoundLeft);
        BLE::Instance(BLE::DEFAULT_INSTANCE).gattServer().write(ultrasoundLeftChar.getValueHandle(), (uint8_t *)&ultrasoundLeft, sizeof(ultrasoundLeft));
    } 
    
    // Front ultrasound sensor
    uint16_t newUltrasoundFront = frontSensor.distance(1);
    
    if (ultrasoundFront != newUltrasoundFront) {
        ultrasoundFront = newUltrasoundFront;
        //pc.printf("Front sensor recorded is:  %lu  \n\r", (unsigned long)ultrasoundFront);
        BLE::Instance(BLE::DEFAULT_INSTANCE).gattServer().write(ultrasoundFrontChar.getValueHandle(), (uint8_t *)&ultrasoundFront, sizeof(ultrasoundFront));
    }

    // Right ultrasound sensor 
    uint16_t newUltrasoundRight = rightSensor.distance(1);
    
    if (ultrasoundRight != newUltrasoundRight) {
        ultrasoundRight = newUltrasoundRight;
        //pc.printf("Right sensor recorded is:  %lu  \n\r", (unsigned long)ultrasoundRight);
        BLE::Instance(BLE::DEFAULT_INSTANCE).gattServer().write(ultrasoundRightChar.getValueHandle(), (uint8_t *)&ultrasoundRight, sizeof(ultrasoundRight));
    }
    
}


/* Main method */
int main(void)
{
    // Stop the car on start and make sure the direction is STOP
    changeLeftWheel(STOP);
    changeRightWheel(STOP);
    setDirection(STOP);  

    // Initiate BLE
    BLE& ble = BLE::Instance(BLE::DEFAULT_INSTANCE);
    ble.init(bleInitComplete);

    // Wait until Bluetooth has initiated 
    while (ble.hasInitialized()  == false) {
        // wait
    }

    // Check the ultrasound values every second
    ultrasoundMonitor.attach(&updateUltrasoundValues, 1);

    // Infinite loop, interupted by Bluetooth events to save power
    while (true) {
        ble.waitForEvent();
    }
}