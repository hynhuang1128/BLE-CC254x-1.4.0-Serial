/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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
#include "simpleBLEPeripheral.h"

/*********************************************************************
 * MACROS
 */
   
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint16 postureChange_Threshold = USER_POSTURE_THRESHOLD_METRIC;
int16 pesk_Stop_Count = 0;
uint16 pesk_Current_Height;
uint16 device_PeskMove_Interval = PESKMOVE_INTERVAL_TRIPLESEG;
uint8 user_HealthData_Count = 0;
uint8 pesk_Lock_Status = PESK_UNLOCK;

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
uint8 device_Current_CtrlMode = DEVICE_CTRL_FREE;
#endif

bool device_SendHealthData_Enable = false;
bool device_BLE_Connect_Flag = false;

hardwareInfo_t pesk_Hardware_Info;
healthData_t user_HealthData[USER_HEALTHDATA_MAX_COUNT];
lockData_t device_LockData;
   
#ifdef AUTOMOVE_FUNC
autoMove_t autoMoveData;
#endif

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint16 uart_DataReceivedCounter;
extern peskStatus_t peskData;
extern uint8 pesk_Move_CurrentStatus;   
extern uint16 device_Height_Destinate;
extern memorySet_t device_Memory_Set[DEVICE_MEMORY_NUM];
extern uint8 cmd_Previous;
extern uint8 pesk_Move_PreviousStatus;
extern deviceInfo_t device_Init_Info;

#ifdef AUTOMOVE_FUNC
extern autoMoveTime_t autoMoveTimeData;
#endif
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x13,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  //SSID for identification
  'O','f','f','i','c','e','w','e','l','l',' ',
  '#','0','0','0','0','f','f',


  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
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
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  simpleBLEPeripheral_TaskID = task_id;

  // Npi uart init
  NPI_InitTransport(NpiSerialCallback);
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );
  
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined ( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

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

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue0[SIMPLEPROFILE_CHAR0_LEN] = { 0, 0, 0, 0 };
    uint8 charValue1[SIMPLEPROFILE_CHAR1_LEN] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8 charValue2[SIMPLEPROFILE_CHAR2_LEN] = { 0, 0 };
    uint8 charValue3[SIMPLEPROFILE_CHAR3_LEN] = { 0, 0, 0, 0 };
    uint8 charValue4 = 0;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 0, 0, 0, 0 };
    uint8 charValue6[SIMPLEPROFILE_CHAR6_LEN] = { 0, 0, 0, 0, 0 };
    uint8 charValue7[SIMPLEPROFILE_CHAR7_LEN] = { 0, 0, 0, 0 };
    uint8 charValue8[SIMPLEPROFILE_CHAR8_LEN] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
#ifdef AUTOMOVE_FUNC
    uint8 charValuef[SIMPLEPROFILE_CHARF_LEN] = { 0, 0, 0, 0, 0 };
#endif
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR0, SIMPLEPROFILE_CHAR0_LEN, charValue0 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, charValue6 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7, SIMPLEPROFILE_CHAR7_LEN, charValue7 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR8, SIMPLEPROFILE_CHAR8_LEN, charValue8 );
#ifdef AUTOMOVE_FUNC
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHARF, SIMPLEPROFILE_CHARF_LEN, charValuef );
#endif
  }


#if defined ( CC2540_MINIDK )

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLEPeripheral_TaskID );

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

  // Initializations for Port Outputs
  DEVICE_PORT_INIT();
    
#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  /* i have commented this line for solving the uart data receiving */
  //HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );
  /* i have added this line for solving the uart data receiving */
  HCI_EXT_HaltDuringRfCmd(HCI_EXT_HALT_DURING_RF_ENABLE);

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  static uint16 periodicTimeCounter;
  
  /* system event that we shouldn't modify */
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  /* event that start device */
  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    
    // Set timer for disposible event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_INIT_EVT, SBP_INIT_EVT_TIMEOUT );
    
    return ( events ^ SBP_START_DEVICE_EVT );
  }
  
  /* periodic event execute every 1 ms */
  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }
    // Process the time counter
    if( periodicTimeCounter >= PERIODIC_COUNTER_MAX )
    {
      periodicTimeCounter = 0;
    }
    else
    {
      periodicTimeCounter++;
    }
    
    // Perform periodic application task
    performPeriodicTask( periodicTimeCounter );
#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
    // To verify the uart data get from JC control box
    verifyingUartData();
#endif

    return (events ^ SBP_PERIODIC_EVT);
  }
  
#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
  /* pesk moving timeout event */
  if ( events & SBP_PESK_STOP_EVT )
  {
    device_Set_Stop();
  }
#endif
  
  /* pesk initialize event */
  if ( events & SBP_INIT_EVT )
  {
    device_HeightInfo_Setup();
    device_HardwareInfo_Setup();
  }
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined ( CC2540_MINIDK )||( WEBEE_BOARD )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined ( CC2540_MINIDK )||( WEBEE_BOARD )
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
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
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )    
  {
    NPI_WriteTransport("KEY K1\n",7);
  }
  
  if ( keys & HAL_KEY_SW_2 )    
  {
    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
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
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }
  }
}
#endif // #if defined( CC2540_MINIDK )

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_CONNECTED:
      {        
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        device_BLE_Connect_Flag = true;
#ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        if ( first_conn_flag == 0 ) 
        {
          uint8 adv_enabled_status = 1;
          GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &adv_enabled_status); // Turn on Advertising
          first_conn_flag = 1;
        }
#endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;      
    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        device_BLE_Connect_Flag = false;
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        device_BLE_Connect_Flag = false;
          
#ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        first_conn_flag = 0;
#endif //#ifdef (PLUS_BROADCASTER)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        device_BLE_Connect_Flag = false;
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every one millisecond as a result of the SBP_PERIODIC_EVT
 *          OSAL event.
 *
 * @param   type unsigned int parameter for count
 *
 * @return  none
 */
static void performPeriodicTask( uint16 timeParam )
{
  static int16 pesk_Move_Speed;
  static uint8 userPosture;
  
  if( !( timeParam % COUNT_PERIODIC_(20) ) )
  {    
    /* Send the health data */
    device_Send_HealthData();
  }
  
  if( !( timeParam % device_PeskMove_Interval ) )
  {
    /* Move the pesk */
    device_Set_PeskMoveStatus();
  }
  
  /* rj11 port can't observe the handset command */
#if (defined PRODUCT_TYPE_BAR2) || (defined PRODCUT_TYPE_CUBE)
  if( !( timeParam % COUNT_PERIODIC_(50) ) )
  {
    /* Get the handset pressed value */
    device_Get_HandsetStatus();
  }
#endif
  
  if( !( timeParam % COUNT_PERIODIC_(25) ) )
  {
    /* Send notification message to the APP */ 
    peskData.userPosture = userPosture;
    pesk_Move_Speed = device_Send_PeskData( CHAR5_SEND_PERIOD / COUNT_PERIODIC_(25) );
  }
  
  /* stop count calibration */
  if( !( timeParam % COUNT_PERIODIC_(200) ) )
  {
    device_Count_Calibration( pesk_Move_Speed );
  }
  
  /* feed the dog */
  if( !( timeParam % COUNT_PERIODIC_(500) ) )
  {
    DEVICE_WDT_FEED();
    DEBUG_MSG(pesk_Current_Height >> 8, pesk_Current_Height, 0, 0);
  }
  
  if( !( timeParam % COUNT_PERIODIC_(1000) ) )
  {
    /* User posture estimate */
    if( peskData.peskStatus == PESK_STATUS_NORMAL )
    {
      userPosture = user_PostureEstimate( pesk_Current_Height, postureChange_Threshold );
      
      // Save user health data here
#ifdef AUTOMOVE_FUNC
      if( device_HealthData_Save( peskData.userPosture ) )
      {
        if( peskData.userPosture )
        {
          if(autoMoveData.enable)
          {
            autoMove_Reset( peskData.userPosture );
          }
        }
        else
        {
          autoMoveData.enable = false;
        }
      }
#else
      device_HealthData_Save( peskData.userPosture );
#endif
    }
    
    /* Send current powered up time with Characteristic7 by 1000ms */
    device_Send_CurrentTime();

#ifdef AUTOMOVE_FUNC
    /* Countdown for the automatic movement */
    uint8 sendBuffer_F[5];
    if( autoMoveData.enable )
    {
      sendBuffer_F[0] = autoMoveData.autoMoveStatus | AUTOMOVE_HIGHEST_BIT;
    }
    else
    {
      sendBuffer_F[0] = autoMoveData.autoMoveStatus & ~AUTOMOVE_HIGHEST_BIT;
    }
    
    if( autoMoveData.enable && !pesk_Lock_Status )
    {
      if( autoMoveData.timeRemaining )
      {
        autoMoveData.timeRemaining = timeUpdate( autoMoveData.timeDestination );
        if( autoMoveData.userNextStatus == USER_STATUS_SIT )
        {
          sendBuffer_F[1] = autoMoveData.timeRemaining / 256;
          sendBuffer_F[2] = autoMoveData.timeRemaining % 256;
          sendBuffer_F[3] = autoMoveTimeData.timeToStand_L;
          sendBuffer_F[4] = autoMoveTimeData.timeToStand_H;
        }
        else
        {
          sendBuffer_F[1] = autoMoveTimeData.timeToSit_L;
          sendBuffer_F[2] = autoMoveTimeData.timeToSit_H;
          sendBuffer_F[3] = autoMoveData.timeRemaining / 256;
          sendBuffer_F[4] = autoMoveData.timeRemaining % 256;
        }
      }
      else
      {
        sendBuffer_F[1] = autoMoveTimeData.timeToSit_L;
        sendBuffer_F[2] = autoMoveTimeData.timeToSit_H;
        sendBuffer_F[3] = autoMoveTimeData.timeToStand_L;
        sendBuffer_F[4] = autoMoveTimeData.timeToStand_H;
        if( autoMoveData.autoMoveStatus == AUTOMOVE_MODE_2 )
        {
          autoMove_Reset( peskData.userPosture );
          autoMove_Move();
        }
        else if( autoMoveData.autoMoveStatus == AUTOMOVE_MODE_1 )
        {
          if( autoMove_Remind() )
          {
            autoMove_Reset( peskData.userPosture );
          }
        }
        else
        {
          //do nothing
        }
      }
    }
    else
    {
      sendBuffer_F[1] = autoMoveTimeData.timeToSit_L;
      sendBuffer_F[2] = autoMoveTimeData.timeToSit_H;
      sendBuffer_F[3] = autoMoveTimeData.timeToStand_L;
      sendBuffer_F[4] = autoMoveTimeData.timeToStand_H;
    }
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHARF, SIMPLEPROFILE_CHARF_LEN, sendBuffer_F );
#endif
    
#if (defined PRODUCT_TYPE_BAR2) || (defined PRODCUT_TYPE_CUBE)
    /* Get current lock status periodicly by 1000ms */
    pesk_Lock_Status = device_Get_Current_LockStatus( device_LockData );
    if( device_LockData.timeStamp )
    {
      device_LockData.timeStamp = timeUpdate( device_LockData.timeDestination );
    }
#elif (defined PRODUCT_TYPE_BAR)
    // TODO the lock command protocol by JieChang
#endif
  }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint16 stopTime;
  uint8 *newCharValue;
  uint16 memoryBuf[DEVICE_MEMORY_NUM];
  
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR0:
      newCharValue = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR0_LEN );
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR0, newCharValue );
      device_Set_InitInfo( newCharValue );
      break;
      
      
    case SIMPLEPROFILE_CHAR1:
      newCharValue = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR1_LEN );
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, newCharValue );
      
      for( int i = 0; i < DEVICE_MEMORY_NUM; i++ )
      {
        memoryBuf[i] = *(newCharValue + i * 2) * 256 + *(newCharValue + i * 2 + 1);
      }
      device_Set_Multiple_Memory( memoryBuf );
      break;
      
      /* reserved function
    case SIMPLEPROFILE_CHAR2:
      newCharValue = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR2_LEN );
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR2, newCharValue );
      device_Set_PostureThreshold( newCharValue );
      break;
      */
      
    case SIMPLEPROFILE_CHAR3:
      newCharValue = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR3_LEN );
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, newCharValue );
      // TODO with new protocol by JieChang
      device_Set_MoveRange( newCharValue );
      break;
      
      
    case SIMPLEPROFILE_CHAR4:
      newCharValue = (uint8 *)osal_mem_alloc( sizeof( uint8 ) );
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR4, newCharValue );
      /* Start timeout event when handset is not working & not in lock status */
      stopTime = peskMoveCommand( *newCharValue );
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PESK_STOP_EVT, stopTime );
      break;


    case SIMPLEPROFILE_CHAR6:
      newCharValue = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR6_LEN );
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, newCharValue );

      /* Get the lock data from app */
      device_Set_LockData( newCharValue );
      break;
      
#ifdef AUTOMOVE_FUNC
    case SIMPLEPROFILE_CHARF:
      newCharValue = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHARF_LEN );
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHARF, newCharValue );
      
      /* Set the automatic movement data from app */
      device_Set_AutoMove( newCharValue );
      autoMove_Reset( peskData.userPosture );
      break;
#endif
      
    default:
      // should not reach here!
      break;
  }
  osal_mem_free( newCharValue );
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
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

static void NpiSerialCallback( uint8 port, uint8 events )
{
  (void)port;
  uint8 numBytes = 0;
  
  if ( events & HAL_UART_RX_TIMEOUT )
  {
    numBytes = NPI_RxBufLen();
    uart_DataHandle( numBytes );
  }
}

/*********************************************************************
*********************************************************************/
