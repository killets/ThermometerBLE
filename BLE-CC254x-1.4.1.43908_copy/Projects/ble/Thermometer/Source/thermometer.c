/**************************************************************************************************
  Filename:       thermometer.c
  Revised:        $Date: 2015-05-04 11:23:57 -0700 (Mon, 04 May 2015) $
  Revision:       $Revision: 43654 $

  Description:    This file contains the Thermometer BLE sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2011 - 2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

// hal_keyÖÐ ´¦Àíº¯ÊýÖÐdevInfoµÈ´ýÈ¥µô×¢ÊÍHAL_ISR_FUNCTION
//0730 2015 edit basic function flow
// before 0730 basic demo measuring tem and accs

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_lcd.h"
#include "hal_key.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "thermometerservice.h"
#include "devinfoservice.h"
#include "thermometer.h"
#include "timeapp.h"
#include "OSAL_Clock.h"

//gj ACC
#include "math.h"
#include "bma250.h"
#include "accelerometer.h"
// How often (in ms) to read the accelerometer
#define ACCEL_READ_PERIOD                     50

// Minimum change in accelerometer before sending a notification
#define ACCEL_CHANGE_THRESHOLD                5
/*********************************************************************
 * MACROS
 */
//GJ
//#define T_RECODE



/*********************************************************************
 * CONSTANTS
 */

// Use limited discoverable mode to advertise for 30.72s, and then stop, or 
// use general discoverable mode to advertise indefinitely 
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
//#define DEFAULT_DISCOVERABLE_MODE           GAP_ADTYPE_FLAGS_GENERAL

// How often to perform periodic event
#define PERIODIC_EVT_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     200

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     1600

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         1

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Some values used to simulate measurements
#define FLAGS_IDX_MAX                         7      //3 flags c/f -- timestamp -- site

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Delay to begin discovery from start of connection in ms
#define DEFAULT_DISCOVERY_DELAY               1000

// Delay to terminate after connection
#define DEFAULT_TERMINATE_DELAY               60000

// Max measurement storage count 
#define TH_STORE_MAX                          3         

#define THERMOMETER_IMEAS_LEN                 6
#define THERMOMETER_MEAS_LEN                  13

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID
uint8 thermometerTaskId;

// Connection handle
uint16 gapConnHandle;

uint8 timeConfigDone;

//gj LOCAL
static uint8 recorded_cnt;
static bool recording;
static uint16 recode_time_mins;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// GAP State
static gaprole_States_t gapProfileState = GAPROLE_INIT;

// Service discovery state
static uint8 timeAppDiscState = DISC_IDLE;

// Service discovery complete
static uint8 timeAppDiscoveryCmpl = FALSE;

// Characteristic configuration state
static uint8 timeAppConfigState = TIMEAPP_CONFIG_START;

// GAP Profile - Name attribute for SCAN RSP data
static uint8 scanResponseData[] =
{
  0x12,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,   
  'T',
  'h',
  'e',
  'r',
  'm',
  'o',
  'm',
  'e',
  't',
  'e',
  'r',
  'S',
  'e',
  'n',
  's',
  'o',
  'r',
    // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),  
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),  

  // Tx power level
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm  
};

// Advertisement data
static uint8 advertData[] = 
{ 
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,
  LO_UINT16( THERMOMETER_SERV_UUID ),
  HI_UINT16( THERMOMETER_SERV_UUID ),

};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Thermometer Sensor";

// Bonded peer address
static uint8 timeAppBondedAddr[B_ADDR_LEN];

// Last connection address
static uint8 lastConnAddr[B_ADDR_LEN] = {0xf,0xf,0xf,0xf,0xf,0xe};;

static bool connectedToLastAddress = false;

// GAP connection handle
static uint16 gapConnHandle;

static attHandleValueInd_t thStoreMeas[10];
static uint8 thStoreStartIndex =0;
static uint8 thStoreIndex = 0;

static uint32 thermometerCelcius = 0X000173;
static bool temperatureMeasCharConfig = false;
static bool temperatureIMeasCharConfig = false;
static bool temperatureIntervalConfig = false;
static bool thMeasTimerRunning = FALSE;

// flags for simulated measurements
static const uint8 thermometerFlags[FLAGS_IDX_MAX] =
{
  THERMOMETER_FLAGS_CELCIUS | THERMOMETER_FLAGS_TIMESTAMP |THERMOMETER_FLAGS_TYPE,
  THERMOMETER_FLAGS_CELCIUS | THERMOMETER_FLAGS_TIMESTAMP,
  THERMOMETER_FLAGS_CELCIUS,
  THERMOMETER_FLAGS_FARENHEIT,
  THERMOMETER_FLAGS_FARENHEIT | THERMOMETER_FLAGS_TIMESTAMP,
  THERMOMETER_FLAGS_FARENHEIT | THERMOMETER_FLAGS_TIMESTAMP | THERMOMETER_FLAGS_TYPE,
  0x00
};

// initial value of flags
static uint8 thermometerFlagsIdx = 0;

// TRUE if pairing started
static uint8 timeAppPairingStarted = FALSE;

// Bonded state
static bool timeAppBonded = FALSE;

// TRUE if discovery postponed due to pairing
static uint8 timeAppDiscPostponed = FALSE;

static void timeAppPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status );

static void thermometer_Advertise( void );

static void thermometerSendStoredMeas();


//gj ACC
// Accelerometer Profile Parameters
static uint8 accelEnabler = FALSE;


/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void thermometerProcessGattMsg( gattMsgEvent_t *pMsg );
static void thermometer_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void performPeriodicImeasTask( void );
static void thermometer_HandleKeys( uint8 shift, uint8 keys );
static void thermometerMeasIndicate(void);
static void thermometerCB(uint8 event);
static void thermometerStoreIndications(attHandleValueInd_t* pInd);
static void updateUI( void );

//GJ ACC
static void accelEnablerChangeCB( void );
static void accelRead( void );


/*******TEST MENTAS*********/
//#include "hal_types.h"
//// Include Name definitions of individual bits and bit-fields in the CC254x device registers.
//#include "ioCC254x_bitdef.h"
//uint16 ADC_READ()
//{
//   uint16  adc_result;
//    /****************************************************************************
//    * Clock setup
//    * See basic software example "clk_xosc_cc254x"
//    */
//  
//    // Set system clock source to HS XOSC, with no pre-scaling.
//    CLKCONCMD = (CLKCONCMD & ~(CLKCON_OSC | CLKCON_CLKSPD)) | CLKCON_CLKSPD_32M;
//    while (CLKCONSTA & CLKCON_OSC);   // Wait until clock source has changed
//  
//    /* Note the 32 kHz RCOSC starts calibrating, if not disabled. */
//  
//  
//    /****************************************************************************
//    * I/O-Port configuration
//    * PIN0_7 is configured to an ADC input pin.
//    */
//
//    // Set [APCFG.APCFG0 = 1].
//    APCFG |= APCFG_APCFG0;
//
//  
//    /****************************************************************************
//    * ADC configuration:
//    *  - [ADCCON1.ST] triggered
//    *  - 12 bit resolution
//    *  - Single-ended
//    *  - Single-channel, due to only 1 pin is selected in the APCFG register
//    *  - Reference voltage is VDD on AVDD pin
//    */
//
//    // Set [ADCCON1.STSEL] according to ADC configuration.
//    ADCCON1 = (ADCCON1 & ~ADCCON1_STSEL) | ADCCON1_STSEL_ST;
//
//    // Set [ADCCON2.SREF/SDIV/SCH] according to ADC configuration.
//    ADCCON2 = ADCCON2_SREF_AVDD | ADCCON2_SDIV_512 | ADCCON2_SCH_AIN0;
//
//  
//    /****************************************************************************
//    * ADC conversion :
//    * The ADC conversion is triggered by setting [ADCCON1.ST = 1].
//    * The CPU will then poll [ADCCON1.EOC] until the conversion is completed.
//    */
//
//    // Set [ADCCON1.ST] and await completion (ADCCON1.EOC = 1). 
//    ADCCON1 |= ADCCON1_ST;
//    while( !(ADCCON1 & ADCCON1_EOC));
//
//    /* Store the ADC result from the ADCH/L register to the adc_result variable.
//    * The conversion result resides in the MSB section of the combined ADCH and
//    * ADCL registers.
//    */
//    adc_result = (ADCL >> 4);
//    adc_result |= (ADCH << 4);
//
//    // End function with infinite loop (for debugging purposes). 
//   // while(1);
//    return adc_result;
//}


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t thermometer_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller
};

// GAP Bond Manager Callbacks
static gapBondCBs_t thermometer_BondMgrCBs =
{
  timeAppPasscodeCB,
  timeAppPairStateCB
  //NULL,  //GJ
  //NULL   //GJ
};



//gj
#if defined (BMA_250)
// Accelerometer Profile Callbacks
static accelCBs_t keyFob_AccelCBs =
{
  accelEnablerChangeCB,          // Called when Enabler attribute changes
};
#endif

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Thermometer_Init
 *
 * @brief   Initialization function for the Thermometer App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Thermometer_Init( uint8 task_id )
{
  thermometerTaskId = task_id;
 
  /* Use for testing if chip default is 0xFFFFFF
  static uint8 bdAddress[6] = {0x1,0x2,0x3,0x4,0x5,0x6};
  HCI_EXT_SetBDADDRCmd(bdAddress);
  */
  
  // Setup the GAP Peripheral Role Profile
  {
    // Device doesn't start advertising until button is pressed
    uint8 initial_advertising_enable = TRUE; //def: FALSE gj

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
      
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanResponseData ), scanResponseData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = FALSE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }  

  // Setup the Thermometer Characteristic Values
  {
    uint8 thermometerSite = THERMOMETER_TYPE_MOUTH;
    Thermometer_SetParameter( THERMOMETER_TYPE, sizeof ( uint8 ), &thermometerSite );
    
    thermometerIRange_t thermometerIRange= {4,60000};
    Thermometer_SetParameter( THERMOMETER_IRANGE, sizeof ( thermometerIRange_t ),
                              &thermometerIRange );
  }

  // Stop config reads when done
  timeConfigDone = FALSE;
   
  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( thermometerTaskId );
  
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  Thermometer_AddService( GATT_ALL_SERVICES );
  DevInfo_AddService( );
  //GJ
#if defined(BMA_250)
  Accel_AddService( GATT_ALL_SERVICES );      // Accelerometer Profile
#endif
  
  // Register for Thermometer service callback
  Thermometer_Register ( thermometerCB );
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( thermometerTaskId );

#if defined( CC2540_MINIDK ) 
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.
  
  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output
  
  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low  
  
  


#endif // #if defined( CC2540_MINIDK )  
  
  //gj
  //HalAdcInit();
  GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 10);
  
  // Setup a delayed profile startup
  osal_set_event( thermometerTaskId, TH_START_DEVICE_EVT );
  //HAL_TURN_ON_LED1();

     //gj
  osal_start_timerEx( thermometerTaskId, TH_WAKEUP_EVT, 20000 );
 
}

/*********************************************************************
 * @fn      Thermometer_ProcessEvent
 *
 * @brief   Thermometer Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
int i=0;
uint16 Thermometer_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  uint8 notify_interval;
  int32 n32;
  
  
  //gj 
  if (events & TH_WAKEUP_EVT)
  {

    i++;
   
//   if(gapProfileState != GAPROLE_CONNECTED)
//   {
//     
//    //HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
//
//      // OSAL_SET_CPU_INTO_SLEEP( 10000 );
//
//   }
//   else
//   {
//      // HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
//     HAL_TURN_OFF_LED1();
//   }
    
//    
    
//    if(i>=2){
//      HAL_TURN_ON_LED2();
//      osal_start_timerEx( thermometerTaskId, TH_WAKEUP_EVT, 15000 );
//      halSleep(10000);
//      HAL_TURN_ON_LED2();
//      HAL_SYSTEM_RESET();
//    }
//     else
//     {
//       osal_start_timerEx( thermometerTaskId, TH_WAKEUP_EVT, 5000 );
//        HAL_TURN_ON_LED1();
//     }
//   // halSleep(10000);
//   //osal_set_event( thermometerTaskId, TH_START_DISCOVERY_EVT );
   

    
    if ( gapProfileState != GAPROLE_CONNECTED ){
    HAL_TURN_ON_LED2();
    uint8 adv_enabled = TRUE;
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &adv_enabled );
    
    }
    else
     HAL_TURN_OFF_LED2();
    
   osal_start_timerEx( thermometerTaskId, TH_WAKEUP_EVT, 20000);
 
   return (events ^ TH_WAKEUP_EVT);
  
  }
  //gj temperature mesure 
  if ( events & TH_MEAS_EVT )
  {
    if(recode_time_mins <= (5)){        // TEST MODE 
      if(recorded_cnt < (recode_time_mins*12)){ //12*5000 =1 MINUS
        osal_start_timerEx( thermometerTaskId, TH_MEAS_EVT, 5000 );
      }else{
        recorded_cnt = 0;
        recording = 0;
      }
    }else{
      if(recorded_cnt < (recode_time_mins/5)){
        osal_start_timerEx( thermometerTaskId, TH_MEAS_EVT, (uint32)5*60000 );
      }else{
        recorded_cnt = 0;
        recording = 0;
      }
    }

    if( recording==1&&recorded_cnt < 120){
      uint32 temperature = HalAdcRead(HAL_ADC_CHANNEL_5,HAL_ADC_RESOLUTION_14);
      devInfoModelNumber[recorded_cnt*2] = LO_UINT16( temperature );
      devInfoModelNumber[recorded_cnt*2+1] = HI_UINT16( temperature );
    }
    recorded_cnt++;

    return (events ^ TH_MEAS_EVT);
  }
  
  //gj
#if defined (BMA_250)
  if ( events & KFD_ACCEL_READ_EVT )
  {
    bStatus_t status = Accel_GetParameter( ACCEL_ENABLER, &accelEnabler );

    if (status == SUCCESS)
    {
      if ( accelEnabler )
      {
        // Restart timer
        if ( ACCEL_READ_PERIOD )
        {
          osal_start_timerEx( thermometerTaskId, KFD_ACCEL_READ_EVT, ACCEL_READ_PERIOD );
        }

        // Read accelerometer data
        accelRead();
      }
      else
      {
        // Stop the acceleromter
        osal_stop_timerEx( thermometerTaskId, KFD_ACCEL_READ_EVT);
      }
    }
    else
    {
        //??
    }
    return (events ^ KFD_ACCEL_READ_EVT);
  }
  
    if ( events & BMA250_FAKEkey_EVT )
  {
    HalLedSet( HAL_LED_1, HAL_LED_MODE_TOGGLE );
    thermometer_HandleKeys( 0, 0x02 ); //fake sw2
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
#endif  
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( thermometerTaskId )) != NULL )
    {
      thermometer_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & TH_START_DEVICE_EVT )
  {
    
    // Start the Device
    VOID GAPRole_StartDevice( &thermometer_PeripheralCBs );
    
    // Register with bond manager after starting device
    VOID GAPBondMgr_Register( &thermometer_BondMgrCBs );
//gj ACC
#if defined(BMA_250)
     // Start the Accelerometer Profile
    VOID Accel_RegisterAppCBs( &keyFob_AccelCBs );
#endif
    updateUI();
    //HalLedSet( HAL_LED_1, HAL_LED_MODE_OFF );
   
    return ( events ^ TH_START_DEVICE_EVT );
  }

  if ( events & TH_START_DISCOVERY_EVT )
  { //HalLedSet( HAL_LED_1, HAL_LED_MODE_ON ); //gj
    if ( timeAppPairingStarted )
    {
      // Postpone discovery until pairing completes
      timeAppDiscPostponed = TRUE;
    }
    else
    {
      timeAppDiscState = timeAppDiscStart();
    }
  
    timeAppDiscState = timeAppDiscStart();
    
    return ( events ^ TH_START_DISCOVERY_EVT );
  }
  
  //periodic indications - if enabled
  if ( events & TH_PERIODIC_MEAS_EVT )
  {
    // Perform periodic application task
    performPeriodicTask();
    return (events ^ TH_PERIODIC_MEAS_EVT);
  } 
  //periodic notifications for IMEAS
  if ( events & TH_PERIODIC_IMEAS_EVT )
  {
    // Perform periodic application task
    performPeriodicImeasTask();
    return (events ^ TH_PERIODIC_IMEAS_EVT);
  }

  // Disconnect after sending measurement
  if ( events & TH_DISCONNECT_EVT )
  {
    //HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
    uint8 advEnable = FALSE;
    
    //disable advertising on disconnect
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advEnable );
    
    // Terminate Connection
    GAPRole_TerminateConnection();

    return (events ^ TH_DISCONNECT_EVT);
  }


  
  
  if ( events & TH_CCC_UPDATE_EVT )
    
    //This event is triggered when CCC is enabled
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      temperatureMeasCharConfig = true;
      
      //if previously connected and measurements are active send stored
      if( connectedToLastAddress == true)
      {
        //send stored measurements
        thermometerSendStoredMeas();
      }
      
      //Only start meas timer if it's not running
      if(thMeasTimerRunning == FALSE)
      {
        //read stored interval value
        Thermometer_GetParameter( THERMOMETER_INTERVAL, &notify_interval );
        n32 = ((uint32)(notify_interval)) * (1000);
  
        //zero interval means should not perform meas
        if (n32 !=0)
        {
          // Start interval timer
          osal_start_timerEx( thermometerTaskId, TH_PERIODIC_MEAS_EVT, n32 );
          thMeasTimerRunning = TRUE;
        }
      }
      return (events ^ TH_CCC_UPDATE_EVT);
    }
   
   return 0;
}

/*********************************************************************
 * @fn      thermometer_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void thermometer_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      thermometer_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
    case GATT_MSG_EVENT:
      thermometerProcessGattMsg( (gattMsgEvent_t *) pMsg );
      break;

    default:
      break;
  }
}




void record_start(void){
  for(int i = 0; i < 240; i++){
    devInfoModelNumber[i] = 0;
  }

  //NPI_WriteTransport("interval_set!\n",16);

  recorded_cnt = 0;
  recording = 1;
  if(recode_time_mins <= 5){        // test mode 
    osal_start_timerEx( thermometerTaskId, TH_MEAS_EVT, 5000 );
  }else{
    osal_start_timerEx( thermometerTaskId, TH_MEAS_EVT, 300000 );
  }
}


/*********************************************************************
 * @fn      thermometer_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
//IT WAS STATIC
void thermometer_HandleKeys( uint8 shift, uint8 keys )
{

}


void thermometer_HandleKeys2( uint8 shift, uint8 keys )
{
 
  bStatus_t status; 
  uint8 notify_interval;
  
  if ( keys & HAL_KEY_SW_1 )
  {
    // set simulated measurement flag index
    thermometerFlagsIdx+=1;
    
    if (thermometerFlagsIdx == FLAGS_IDX_MAX)
    {
      thermometerFlagsIdx = 0;
    }  
  }
  
  
  //read stored interval value
  Thermometer_GetParameter( THERMOMETER_INTERVAL, &notify_interval ); 

  if(notify_interval == 0)
  {
    thMeasTimerRunning = FALSE;
  }
  
  if ( keys & HAL_KEY_SW_2 )
  {
    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off. If timer is running, then will adv when meas is ready
    if((gapProfileState != GAPROLE_CONNECTED) &&  (thMeasTimerRunning == FALSE))
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;
      
      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
      
      if ( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }
      
      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );   
    }
    else //timer is running, so allow simulated changes
    {
      
      #if defined( T_RECODE ) //gj notice
      
        if(!recording){
          recode_time_mins = 2;
          record_start();
        }
      
      #else     
      //change temperature, remove single precision
      if((thermometerCelcius) < 0X000175)
      {
        thermometerCelcius +=1;
      }
      else
      {
        uint16 thInterval = 30;
        
        thermometerCelcius = 0X000173;

        //Simulate interval change
        Thermometer_SetParameter( THERMOMETER_INTERVAL, THERMOMETER_INTERVAL_LEN,
                                  &thInterval );
        if(temperatureIntervalConfig == true) 
        {
          attHandleValueInd_t intervalInd;
          
          intervalInd.pValue = GATT_bm_alloc( gapConnHandle, ATT_HANDLE_VALUE_IND, 
                                              THERMOMETER_INTERVAL_LEN, NULL );
          if ( intervalInd.pValue != NULL )
          {
            intervalInd.len = THERMOMETER_INTERVAL_LEN;
            intervalInd.pValue[0] = LO_UINT16(thInterval);
            intervalInd.pValue[1] = HI_UINT16(thInterval);
            intervalInd.handle = THERMOMETER_INTERVAL_VALUE_POS;
          
            status = Thermometer_IntervalIndicate( gapConnHandle, &intervalInd,
                                                   thermometerTaskId );
            // we can fail if there was pending meas or not connected
            if (status != SUCCESS)
            {
              //queue indication
              thermometerStoreIndications(&intervalInd);
            }
          }
        }
      }
      #endif
    }
    
    
    
  }
}





/*********************************************************************
 * @fn      thermometer_Advertise
 *
 * @brief   Start advertisemement when measurement is ready
 *
 *
 * @return  none
 */
static void thermometer_Advertise( void )
{
    
  // Advertise if not connected 
  if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;
      
      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
      
      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      
      //change the GAP advertisement status 
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );   
    }
}

/*********************************************************************
 * @fn      timeAppProcessGattMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void thermometerProcessGattMsg( gattMsgEvent_t *pMsg )
{

  
  //Measurement Indication Confirmation
  if( pMsg->method ==ATT_HANDLE_VALUE_CFM)
  {
      thermometerSendStoredMeas();
  }
  
  if ( pMsg->method == ATT_HANDLE_VALUE_NOTI ||
       pMsg->method == ATT_HANDLE_VALUE_IND )
  {
    timeAppIndGattMsg( pMsg );
  }
  else if ( pMsg->method == ATT_READ_RSP ||
            pMsg->method == ATT_WRITE_RSP )
  {
    timeAppConfigState = timeAppConfigGattMsg ( timeAppConfigState, pMsg );
    if ( timeAppConfigState == TIMEAPP_CONFIG_CMPL )
    {
      timeAppDiscoveryCmpl = TRUE;
    }
  }
  else
  {
    timeAppDiscState = timeAppDiscGattMsg( timeAppDiscState, pMsg );
    if ( timeAppDiscState == DISC_IDLE )
    {      
      // Start characteristic configuration
      timeAppConfigState = timeAppConfigNext( TIMEAPP_CONFIG_START );
    }
  }
  
  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      timeAppDisconnected
 *
 * @brief   Handle disconnect. 
 *
 * @return  none
 */
static void timeAppDisconnected( void )
{
  // Initialize state variables
  timeAppDiscState = DISC_IDLE;
  timeAppPairingStarted = FALSE;
  timeAppDiscPostponed = FALSE;
}

/*********************************************************************
 * @fn      gapProfileStateCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  uint8 valFalse = FALSE;
  // if connected
  if ( newState == GAPROLE_CONNECTED )
  {
    linkDBItem_t  *pItem;

    // Get connection handle
    GAPRole_GetParameter( GAPROLE_CONNHANDLE, &gapConnHandle );

    // Get peer bd address
    if ( (pItem = linkDB_Find( gapConnHandle )) != NULL)
    {
      // If connected to device without bond do service discovery
      if ( !osal_memcmp( pItem->addr, timeAppBondedAddr, B_ADDR_LEN ) )
      {
        timeAppDiscoveryCmpl = FALSE;
      }
      else
      {
        timeAppDiscoveryCmpl = TRUE;
      }
      
      // if this was last connection address don't do discovery
      if(osal_memcmp( pItem->addr, lastConnAddr, B_ADDR_LEN ))
      {
        timeAppDiscoveryCmpl = TRUE;
        connectedToLastAddress = true;
      }
      else
      {
        //save the last connected address  
        osal_memcpy(lastConnAddr, pItem->addr, B_ADDR_LEN );
      }

      // Initiate service discovery if necessary
      if ( timeAppDiscoveryCmpl == FALSE )
      {
        osal_start_timerEx( thermometerTaskId, TH_START_DISCOVERY_EVT, DEFAULT_DISCOVERY_DELAY );
      }
      
      //on connection initiate disconnect timer in 20 seconds
      //osal_start_timerEx( thermometerTaskId, TH_DISCONNECT_EVT, DEFAULT_TERMINATE_DELAY );        
        
    } 
  }
  // if disconnected
  else if ( gapProfileState == GAPROLE_CONNECTED && 
            newState != GAPROLE_CONNECTED )
  {
    timeAppDisconnected();
    
    //always stop intermediate timer
    osal_stop_timerEx( thermometerTaskId, TH_PERIODIC_IMEAS_EVT ); 
	
	//gj
	#if defined (BMA_250)
    // Change attribute value of Accelerometer Enable to FALSE
    Accel_SetParameter(ACCEL_ENABLER, sizeof(valFalse), &valFalse);
    // Stop the acceleromter
    accelEnablerChangeCB(); // SetParameter does not trigger the callback
    #endif
	
  }    
  // if started
  else if ( newState == GAPROLE_STARTED )
  {
    // Initialize time clock 
    timeAppClockInit();
  }
  
  gapProfileState = newState;
  
  updateUI();
  
}

/*********************************************************************
 * @fn      thermometerSendStoredMeas
 *
 * @brief   Prepare and send a stored Meas Indication
 *
 * @return  none
 */
static void thermometerSendStoredMeas(void)
{
    bStatus_t status;  
  
  //we connected to this peer before so send any stored measurements
  if(thStoreStartIndex != thStoreIndex )
    {
     //send Measurement
     status  = Thermometer_TempIndicate( gapConnHandle, &thStoreMeas[thStoreStartIndex], thermometerTaskId);
     
     if(status == SUCCESS)
     {
        thStoreStartIndex = thStoreStartIndex +1;
      
        // wrap around buffer 
        if(thStoreStartIndex > TH_STORE_MAX)
        {
          thStoreStartIndex = 0;
        }
     }
   }  
}

/*********************************************************************
 * @fn      thermometerMeasIndicate
 *
 * @brief   Prepare and send a thermometer measurement indication
 *
 * @return  none
 */
static void thermometerMeasIndicate(void)
{
  // Thermometer measurement value stored in this structure.
  attHandleValueInd_t thermometerMeas;

  thermometerMeas.pValue = GATT_bm_alloc(gapConnHandle,
                                         ATT_HANDLE_VALUE_IND,
                                         THERMOMETER_MEAS_LEN, NULL);
  if (thermometerMeas.pValue != NULL)
  {
    // att value notification structure 
    uint8 *p = thermometerMeas.pValue;
    
    // temperature
    uint32 temperature;
    
    //flags
    uint8 flags = thermometerFlags[thermometerFlagsIdx];
    
    // flags 1 byte long
    *p++ = flags;
    
    if(flags & THERMOMETER_FLAGS_FARENHEIT)
    {//gj
      //temperature =  (thermometerCelcius *9/5) +320;
      temperature = HalAdcRead(HAL_ADC_CHANNEL_5,HAL_ADC_RESOLUTION_14);
      
    }
    else
    {
       //temperature = thermometerCelcius;
      temperature = HalAdcRead(HAL_ADC_CHANNEL_5,HAL_ADC_RESOLUTION_14);
    }
    
    temperature = 0xFF000000 | temperature;
    
   
      
    //osal_buffer_uint32
    p = osal_buffer_uint32( p, temperature );
     
    //timestamp
    if (flags & THERMOMETER_FLAGS_TIMESTAMP)
    {
      UTCTimeStruct time;
    
      // Get time structure from OSAL
      osal_ConvertUTCTime( &time, osal_getClock() );
      
      *p++ = LO_UINT16(time.year);
      *p++ = HI_UINT16(time.year);
      *p++ = time.month;    
      *p++ = time.day;  
      *p++ = time.hour;    
      *p++ = time.minutes;    
      *p++ = time.seconds;     
    }
    
    if(flags & THERMOMETER_FLAGS_TYPE)
    {
      uint8 site;
      Thermometer_GetParameter( THERMOMETER_TYPE, &site ); 
      *p++ =  site;    
    }
    
    thermometerMeas.len = (uint8) (p - thermometerMeas.pValue);
    thermometerMeas.handle = THERMOMETER_TEMP_VALUE_POS;
    
    // Queue indication.
    thermometerStoreIndications( &thermometerMeas);
      
    //advertise measurement is ready
    thermometer_Advertise();
  }
}


/*********************************************************************
 * @fn      thermometerStoreIndications
 *
 * @brief   Queue indications
 *
 * @return  none
 */
static void thermometerStoreIndications(attHandleValueInd_t* pInd)
{
  //store measurement
  osal_memcpy( &thStoreMeas[thStoreIndex], pInd, sizeof( attHandleValueInd_t ) );
  
  //store index
  thStoreIndex = thStoreIndex +1;
  if(thStoreIndex > TH_STORE_MAX)
  {
    thStoreIndex = 0;
  }
  
  if(thStoreIndex == thStoreStartIndex)
  {
    thStoreStartIndex = thStoreStartIndex +1;
    if(thStoreStartIndex > TH_STORE_MAX)
    {
      thStoreStartIndex = 0;
    }
  }  
}

double calTemp(uint32 temperature) //gj cal temp from adc voltage
{
  double tempcal;
         tempcal=temperature/(8191-temperature)*4750;
         tempcal=log(tempcal);
         tempcal=1 / (0.0011224922 + (0.0002359132 * tempcal) + (0.000000074995733 * tempcal * tempcal * tempcal))-273.15;
         tempcal=(tempcal*9/5+32)*100;
         return tempcal;
}

uint32 temp2char(uint32 Data)// transfer double to char
{
  uint8 buff[5];
  buff[0] = (Data/1000)%10+0x30; 
  buff[1] = (Data/100)%10+0x30; 
  buff[2] = (Data/10)%10+0x30;
  buff[3] =  Data%10+0x30;
  return (buff[0]<<6)+(buff[1]<<4)+(buff[2]<<2)+buff[3];
}

/*********************************************************************
 * @fn      thermometerMeasNotify
 *
 * @brief   Prepare and send a thermometer measurement notification
 *
 * @return  none
 */
static void thermometerImeasNotify(void)
{
  if (temperatureIMeasCharConfig == true)
  {
    attHandleValueNoti_t thermometerIMeas;
    
    thermometerIMeas.pValue = GATT_bm_alloc( gapConnHandle,
                                             ATT_HANDLE_VALUE_NOTI, 
                                             THERMOMETER_IMEAS_LEN, NULL );
    if ( thermometerIMeas.pValue != NULL )
    {
      // att value notification structure 
      uint8 *p = thermometerIMeas.pValue;
      
      // temperature
      uint32 temperature;
     
      
      //flags
      uint8 flags = thermometerFlags[thermometerFlagsIdx];
      
      // flags 1 byte long
      *p++ = flags;
      
      if(flags & THERMOMETER_FLAGS_FARENHEIT)
      {
        //temperature =  (thermometerCelcius *9/5) +320;
         temperature = HalAdcRead(HAL_ADC_CHANNEL_5,HAL_ADC_RESOLUTION_14);
          //gj
//         double tempcal=calTemp(temperature);
//         temperature=tempcal;
//         temperature=temp2char(temperature);   
      }
      else
      {
         //temperature = thermometerCelcius;
         temperature = HalAdcRead(HAL_ADC_CHANNEL_5,HAL_ADC_RESOLUTION_14);
      }
      
     // temperature = 0xFF000000 | temperature;
       //gj
    //temperature =3;  
      
      //osal_buffer_uint32
      p = osal_buffer_uint32( p, temperature );
      
      //timestamp
      if (flags & THERMOMETER_FLAGS_TIMESTAMP)
      {
        UTCTimeStruct time;
      
        // Get time structure from OSAL
        osal_ConvertUTCTime( &time, osal_getClock() );
        
        *p++ = LO_UINT16(time.year);
        *p++ = HI_UINT16(time.year);
        *p++ = time.month;    
        *p++ = time.day;  
        *p++ = time.hour;    
        *p++ = time.minutes;    
        *p++ = time.seconds;     
      }
    
      if(flags & THERMOMETER_FLAGS_TYPE)
      {
        uint8 site;
        Thermometer_GetParameter( THERMOMETER_TYPE, &site ); 
        *p++ =  site;    
      }
      
      thermometerIMeas.len = (uint8) (p - thermometerIMeas.pValue);
     
      if ( Thermometer_IMeasNotify( gapConnHandle, &thermometerIMeas) != SUCCESS )
      {
        GATT_bm_free( (gattMsg_t *)&thermometerIMeas, ATT_HANDLE_VALUE_NOTI );
      }
    }
  }
}

/*********************************************************************
 * @fn      thermometerCB
 *
 * @brief   Callback function for thermometer service.
 *
 * @param   event - service event
 *
 * @return  none
 */
static void thermometerCB(uint8 event)
{
  switch (event)
  {
  case THERMOMETER_TEMP_IND_ENABLED:
    osal_set_event( thermometerTaskId, TH_CCC_UPDATE_EVT );
  break;
        
  case  THERMOMETER_TEMP_IND_DISABLED:
    temperatureMeasCharConfig = false;
    osal_stop_timerEx( thermometerTaskId, TH_PERIODIC_MEAS_EVT );  
    thMeasTimerRunning = FALSE;
    break;

  case THERMOMETER_IMEAS_NOTI_ENABLED:
    temperatureIMeasCharConfig = true;
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      osal_start_timerEx( thermometerTaskId, TH_PERIODIC_IMEAS_EVT, 1000 );
    }      
    break;

  case  THERMOMETER_IMEAS_NOTI_DISABLED:
    temperatureIMeasCharConfig = false;
    osal_stop_timerEx( thermometerTaskId, TH_PERIODIC_IMEAS_EVT );  
    break;
  
  case THERMOMETER_INTERVAL_IND_ENABLED:
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      temperatureIntervalConfig = true;
    }      
    break;

  case  THERMOMETER_INTERVAL_IND_DISABLED:
    if (gapProfileState == GAPROLE_CONNECTED)
    {
      temperatureIntervalConfig = false;
    } 
    break;   
   
  default:  
    break;
  }
}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
   uint8 notify_interval;
   int32 n32;

   //Measurement Ready - send if Client Configuration is Configured
    if(temperatureMeasCharConfig == true)
    {  
      //read stored interval value
      Thermometer_GetParameter( THERMOMETER_INTERVAL, &notify_interval ); 
   
      n32 = ((uint32)(notify_interval)) * (1000);    
      
      //if interval is zero don't send indication
      if( n32 != 0)
      {
        // send thermometer measurement notification
        thermometerMeasIndicate();
        
        // Start interval timer
          osal_start_timerEx( thermometerTaskId, TH_PERIODIC_MEAS_EVT, n32 );
      }
    }
}
/*********************************************************************
 * @fn      performPeriodicImeasTask
 *
 * @brief   Perform a periodic application task.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicImeasTask( void )
{
  
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    // send thermometer measurement notification
    thermometerImeasNotify();
    
    // Start interval timer (simulated fast measurement for display)
    osal_start_timerEx( thermometerTaskId, TH_PERIODIC_IMEAS_EVT, 1000 );
  }
}

/*********************************************************************
 * @fn      timeAppPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void timeAppPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, 0 );
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void timeAppPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    timeAppPairingStarted = TRUE;
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    timeAppPairingStarted = FALSE;

    if ( status == SUCCESS )
    {
      linkDBItem_t  *pItem;
      
      if ( (pItem = linkDB_Find( gapConnHandle )) != NULL )
      {
        // Store bonding state of pairing
        timeAppBonded = ( (pItem->stateFlags & LINK_BOUND) == LINK_BOUND );
        
        if ( timeAppBonded )
        {
          osal_memcpy( timeAppBondedAddr, pItem->addr, B_ADDR_LEN );
        }
      }
      
      // If discovery was postponed start discovery
      if ( timeAppDiscPostponed && timeAppDiscoveryCmpl == FALSE )
      {
        timeAppDiscPostponed = FALSE;
        osal_set_event( thermometerTaskId, TH_START_DISCOVERY_EVT );
      }
      
    }
  }
}


#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}
#endif 

/*********************************************************************
 * @fn      Update UI
 *
 * @brief   Update user interface LCD and LED
 *
 * @param   none
 *
 * @return  none
 */
static void updateUI( void )
{

  LCD_WRITE_STRING( "Thermometer",  HAL_LCD_LINE_1 );
  
  static uint8 ownAddress[B_ADDR_LEN];

 #if (defined HAL_LCD) && (HAL_LCD == TRUE) 
    //number of stored measuremnts
    uint16 count =0;
  
    //store index
    if( thStoreIndex > thStoreStartIndex )
    {
      count = thStoreIndex - thStoreStartIndex;
    }
    
    if( thStoreStartIndex > thStoreIndex )
    {
       count = ( TH_STORE_MAX-thStoreStartIndex ) + thStoreIndex+1;
    }
#endif
  
  //State
   switch (gapProfileState)
  {
    case GAPROLE_INIT:
          LCD_WRITE_STRING( "Initialized",  HAL_LCD_LINE_2 );
          break;
    case GAPROLE_STARTED: 
          LCD_WRITE_STRING( "Started",  HAL_LCD_LINE_2 );
          GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
          LCD_WRITE_STRING( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_3 );
          break;
    case GAPROLE_ADVERTISING:
         LCD_WRITE_STRING( "Advertising",  HAL_LCD_LINE_2 );
         LCD_WRITE_STRING( "                  ",  HAL_LCD_LINE_3 );
         LCD_WRITE_STRING_VALUE("Stored", count, 10, HAL_LCD_LINE_3 );
         break;
    case GAPROLE_WAITING:
    case GAPROLE_WAITING_AFTER_TIMEOUT:
         LCD_WRITE_STRING( "Waiting    ",  HAL_LCD_LINE_2 );
         LCD_WRITE_STRING_VALUE("Stored", count, 10, HAL_LCD_LINE_3 );  
        break;
    case GAPROLE_CONNECTED:
        LCD_WRITE_STRING( "Connected  ",  HAL_LCD_LINE_2 );
        break;
    case GAPROLE_ERROR:
        LCD_WRITE_STRING( "Error      ",  HAL_LCD_LINE_2 );
        break;
    default:
        break;
  }
  
}

/*********************************************************************
 * @fn      accelEnablerChangeCB
 *
 * @brief   Called by the Accelerometer Profile when the Enabler Attribute
 *          is changed.
 *
 * @param   none
 *
 * @return  none
 */
static void accelEnablerChangeCB( void )
{
  bStatus_t status = Accel_GetParameter( ACCEL_ENABLER, &accelEnabler );

  if (status == SUCCESS){
    if (accelEnabler)
    {


    // Initialize accelerometer
  accInit();

  P1SEL &= ~BV(7); // gpio mode
  P1DIR &= ~BV(7); // input (pullup by default)
  P1IEN |= BV(7); // enable P1.7 interrupts

  IEN2 |= BV(4); // enable P1 interrupts   
  IEN0 |= BV(7); // global interrupt enable
      
      
      // Setup timer for accelerometer task
      osal_start_timerEx( thermometerTaskId, KFD_ACCEL_READ_EVT, ACCEL_READ_PERIOD );
    } else
    {
      // Stop the acceleromter
      accStop();
      osal_stop_timerEx( thermometerTaskId, KFD_ACCEL_READ_EVT);
    }
  } else
  {
    //??
  }
}

/*********************************************************************
 * @fn      accelRead
 *
 * @brief   Called by the application to read accelerometer data
 *          and put data in accelerometer profile
 *
 * @param   none
 *
 * @return  none
 */
static void accelRead( void )
{

  static int8 x, y, z;
  int8 new_x, new_y, new_z;

  // Read data for each axis of the accelerometer
  accReadAcc(&new_x, &new_y, &new_z);

  // Check if x-axis value has changed by more than the threshold value and
  // set profile parameter if it has (this will send a notification if enabled)
  if( (x < (new_x-ACCEL_CHANGE_THRESHOLD)) || (x > (new_x+ACCEL_CHANGE_THRESHOLD)) )
  {
    x = new_x;
    Accel_SetParameter(ACCEL_X_ATTR, sizeof ( int8 ), &x);
  }

  // Check if y-axis value has changed by more than the threshold value and
  // set profile parameter if it has (this will send a notification if enabled)
  if( (y < (new_y-ACCEL_CHANGE_THRESHOLD)) || (y > (new_y+ACCEL_CHANGE_THRESHOLD)) )
  {
    y = new_y;
    Accel_SetParameter(ACCEL_Y_ATTR, sizeof ( int8 ), &y);
  }

  // Check if z-axis value has changed by more than the threshold value and
  // set profile parameter if it has (this will send a notification if enabled)
  if( (z < (new_z-ACCEL_CHANGE_THRESHOLD)) || (z > (new_z+ACCEL_CHANGE_THRESHOLD)) )
  {
    z = new_z;
    Accel_SetParameter(ACCEL_Z_ATTR, sizeof ( int8 ), &z);
  }

}



/*********************************************************************
*********************************************************************/
