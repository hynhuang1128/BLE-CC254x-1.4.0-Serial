
/**************************************************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "math.h"   

#ifndef APPLICATIONBLE_H
#define APPLICATIONBLE_H

/**************************************************************************************************
 * MACROS
 */
   
//#define DEBUG_MSG( msg0, msg1, msg2, msg3 )     do                                                                                      \
                                                {                                                                                       \
                                                  uint8 buffer[4];                                                                      \
                                                  buffer[0] = msg0;                                                                     \
                                                  buffer[1] = msg1;                                                                     \
                                                  buffer[2] = msg2;                                                                     \
                                                  buffer[3] = msg3;                                                                     \
                                                  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, buffer );   \
                                                } while(0)

#define ENABLE                               1
#define DISABLE                              0   

// Pesk status macros
#define PESK_STATUS_NORMAL                   0x01
#define PESK_STATUS_ERROR                    0x02
#define PESK_STATUS_RESET                    0x04
#define PESK_STATUS_SAVE                     0x06

// User status macros
#define USER_STATUS_LEAVE                    0x00
#define USER_STATUS_SIT                      0x01
#define USER_STATUS_STAND                    0x02

// Pesk lock status
#define PESK_LOCKED                          0x01
#define PESK_UNLOCK                          0x00

// Pesk move status
#define PESK_STATUS_IDLE                     0x00
#define PESK_STATUS_UP                       0x01
#define PESK_STATUS_DOWN                     0x02
#define PESK_STATUS_SET                      0x03

// Device control method
#define DEVICE_CTRL_FREE                     0x00
#define DEVICE_CTRL_APP                      0x01
#define DEVICE_CTRL_HANDSET                  0x02

// Device control command
#define DEVICE_COMMON_CMD                    0x00
#define DEVICE_MEMORY_CMD                    0x10
#define DEVICE_ADVANCE_CMD                   0x20

#define DEVICE_COMMON_UP                     0x01
#define DEVICE_COMMON_DOWN                   0x02
#define DEVICE_COMMON_STOP                   0x00

#define DEVICE_MEMORY_SET1                   0x00
#define DEVICE_MEMORY_SET2                   0x01
#define DEVICE_MEMORY_SET3                   0x02
#define DEVICE_MEMORY_SET4                   0x03
   
#define DEVICE_MEMORY_NUM                    4

// Handset status
#define HANDSET_STATUS_IDLE                  0x0f
#define HANDSET_STATUS_UP                    0x0d
#define HANDSET_STATUS_DOWN                  0x07
#define HANDSET_STATUS_SET1                  0x05
#define HANDSET_STATUS_SET2                  0x0b
#define HANDSET_STATUS_SET3                  0x03
#define HANDSET_STATUS_SET4                  0x09
#define HANDSET_STATUS_SETTING               0x0e

// Handset command
#define CMD_PESK_UP                       0x40
#define CMD_PESK_DOWN                     0x80
#define CMD_PESK_STOP                     0x00
#define CMD_PESK_SET1                     0xc0
#define CMD_PESK_SET2                     0x20
#define CMD_PESK_SET3                     0xa0
#define CMD_PESK_SET4                     0x60
#define CMD_PESK_SETTING                  0x10

// Height value assertion
#define VALUE_INGNORED                       0x0000
#define VALUE_INVALID                        0xffff
#define HEIGHT_VALUE_MAX                     0x3fff
#define HEIGHT_VALUE_MIN                     0x0000

// Send command to the pesk 
#define DEVICE_SET_PORT_INPUT()              ( P0DIR &= 0x0f )
#define DEVICE_SET_PORT_OUTPUT()             ( P0DIR |= 0xf0 )
   
#define DEVICE_SENDCMD_PESK(x)               do                                 \
                                             {                                  \
                                              if( !x )                          \
                                              {                                 \
                                                P0 = x;                         \
                                                DEVICE_SET_PORT_INPUT();        \
                                              }                                 \
                                              else if( !pesk_Lock_Status )      \
                                              {                                 \
                                              DEVICE_SET_PORT_OUTPUT();         \
                                              P0 = x;                           \
                                              }                                 \
                                             } while(0)


#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
#define DEVICE_PORT_INIT()                   do                                 \
                                             {                                  \
                                              P0SEL = 0x0F;                     \
                                              P0DIR = 0x00;                     \
                                              P1SEL = 0xC0;                     \
                                              P1DIR = 0x30;                     \
                                              P0 = 0x00;                        \
                                             } while(0)
#elif (defined PRODUCT_TYPE_BAR)
#define DEVICE_PORT_INIT()                   do                                 \
                                             {                                  \
                                              P1SEL = 0x0C;                     \
                                              P1DIR = 0x03;                     \
                                             } while(0)
#endif

#define TRANSFER_IMPERIAL_TO_METRIC(x)       ( (x) * 127 / 50 )
   
// NVID for snv storage data
#define BLE_NVID_START_ADDR                  0x89
#define BLE_NVID_DATA_LEN                    19

#define BLE_NVID_HARDWARE_INFO               0x89
#define BLE_NVID_HARDWARE_INFO_LEN           4
#define BLE_NVID_MEMORY_HEIGHT1              0x8d
#define BLE_NVID_MEMORY_HEIGHT2              0x8f
#define BLE_NVID_MEMORY_HEIGHT3              0x91
#define BLE_NVID_MEMORY_HEIGHT4              0x93 
#define BLE_NVID_MEMORY_HEIGHT_LEN           2
#define BLE_NVID_PESK_MAX                    0x95
#define BLE_NVID_PESK_MIN                    0x97
#define BLE_NVID_PESK_RANGE_LEN              2
#define BLE_NVID_USER_POSTURE                0x99
#define BLE_NVID_USER_POSTURE_LEN            2
#define BLE_NVID_DEVICE_LOCK                 0x9b
#define BLE_NVID_DEVICE_LOCK_LEN             1
                                               
// Unit switch
#define PESK_UNIT_BIT                        0x80
#define PESK_METRICHEIGHT_MIN                600   
   
// Pesk type
#define PESK_TYPE_BITS                       0x7f
#define PESK_TYPE_TRIPLESEG                  0x01
#define PESK_TYPE_DOUBLESEG                  0x00

// User posture threshold
#define USER_POSTURE_THRESHOLD_METRIC        900
#define USER_POSTURE_THRESHOLD_IMPERIAL      354

// ERR status handle
#define PESKDATA_IN_RST                      0xaa010401
#define PESKDATA_IN_NORMAL                   0x00000101
#define PESKDATA_RST_ERRCODE                 0x01040101
#define PESKDATA_RST_ERRCODE2                0x04010101
#define PESKDATA_RST_ERRCODE3                0x01040201
#define PESKDATA_IN_SLEEP                    0x00000501
#define PESKDATA_IN_NORMAL_FILTER            0x0000ffff
#define PESKDATA_IN_ERROR                    0x00000201
#define PESK_RST_MIN_VALUE                   200
#define PESKDATA_FAILURE                     0x0000
                                               
// Health data
#define USER_HEALTHDATA_MAX_COUNT            201

// Device informations
#define DEVICE_TYPE_DEFAULT                  DEVICE_BAR2
#define DEVICE_PESK_DEFAULT                  PESK_TRIPLESEG_METRIC
#define DEVICE_VERSION_DEFAULT               0x0000

// Device types
#define DEVICE_BAR2                          0x02
#define DEVICE_BAR                           0x01
#define DEVICE_CUBE                          0x00

#define PESK_TRIPLESEG_IMPERIAL              0x81
#define PESK_TRIPLESEG_METRIC                0x01
#define PESK_DOUBLESEG_IMPERIAL              0x80
#define PESK_DOUBLESEG_METRIC                0x00

// Pesk move range
#define PESK_DOUBLESEG_METRIC_MAX            1213
#define PESK_DOUBLESEG_METRIC_MIN            710

#define PESK_TRIPLESEG_METRIC_MAX            1283
#define PESK_TRIPLESEG_METRIC_MIN            613

#define PESK_DOUBLESEG_IMPERIAL_MAX          480
#define PESK_DOUBLESEG_IMPERIAL_MIN          278

#define PESK_TRIPLESEG_IMPERIAL_MAX          501
#define PESK_TRIPLESEG_IMPERIAL_MIN          245

// Pesk critical value
#define PESK_CRITICAL_VALUE                  20

// FIFO relative
#define FIFO_DATA_MAX                        20

// Speed
#define PESK_SPEED_DOUBLESEG_MAX             11

// Interval ralative
#define PESKMOVE_INTERVAL_DOUBLESEG          20
#define PESKMOVE_INTERVAL_TRIPLESEG          15

// Metric to imperial scale
#define SCALE_IMPERIAL_TO_METRIC             (254 / 100)

// Characteristic5 send period
#define CHAR5_SEND_PERIOD                    500
                                               
// Automatic movement relative                                  
#ifdef AUTOMOVE_FUNC
#define BLE_NVID_AUTOMOVE                    0x9c
#define BLE_NVID_AUTOMOVE_LEN                5        
#define BLE_NVID_AUTOMOVE_EN_LEN             1
                                               
#define AUTOMOVE_MIN_STANDTOSIT_TIME         15
#define AUTOMOVE_MIN_SITTOSTAND_TIME         30
#define AUTOMOVE_DEFAULT_STANDTOSIT_TIME     15
#define AUTOMOVE_DEFAULT_SITTOSTAND_TIME     30
                                                
#define SETTING_DELAY_TIME                   3
#endif                                               
/*********************************************************************
 * TYPEDEFS
 */
typedef union
{
  uint32 currentPeskData;
  struct
  {
    uint8 peskStatus;
    uint8 userPosture;
    union
    {
      uint16 info;
      struct
      {
        uint8 info_L;
        uint8 info_H;
      };
    };
  };
} peskStatus_t;

typedef union
{
  uint16 height_Value;
  struct
  {
    uint8 height_L;
    uint8 height_H;
  };
} memorySet_t;

typedef struct
{
  uint16 height_Maximum;
  uint16 height_Minimum;
} hardwareInfo_t;

typedef struct
{
  uint8 peskType;
  uint8 deviceType;
  union
  {
    uint16 deviceVersion;
    struct
    {
      uint8 version_L;
      uint8 version_H;
    };
  };
} deviceInfo_t;

typedef union
{
  uint32 dataValue;
  uint8 data[4];
} seprate_DataU32_t;

typedef union
{
  uint16 dataValue;
  struct
  {
    uint8 data_L;
    uint8 data_H;
  };
} seprate_DataU16_t;

typedef struct
{
  uint8 userStatus;
  union
  {
    uint32 timeStamp;
    uint8 timeStamp_S[4];
  };
} healthData_t;

typedef struct
{
  uint8 lockStatus;
  union
  {
    uint32 timeStamp;
    uint8 timeStamp_S[4];
  };
} lockData_t;

typedef struct FIFO
{
  struct FIFO *addrNext;
  uint16 dataValue;
  uint8 dataCount;
} FIFO_Data_t;

#ifdef AUTOMOVE_FUNC
typedef struct
{
  bool enable;
  bool userNextStatus;
  uint32 timeRemaining;
} autoMove_t;
  
typedef struct
{
  union
  {
    uint16 timeToStand;
    struct
    {
      uint8 timeToStand_H;
      uint8 timeToStand_L;
    };
  };
  union
  {
    uint16 timeToSit;
    struct
    {
      uint8 timeToSit_H;
      uint8 timeToSit_L;
    };
  };
} autoMoveTime_t;
#endif

/*****************************************
*      we declare our functions here 
******************************************/

/*********************************************************************
 * @fn      device_HealthData_Save
 * 
 * @brief   save the health data once user's posture changed
 *
 * @param   userStatus - user posture data
 *
 * @return  success or not
 */
bool device_HealthData_Save( uint8 userStatus );

/*********************************************************************
 * @fn      verifyingUartData
 *
 * @brief   Process the incoming message from JC control box.
 *
 * @param   none
 *
 * @return  none
 */
void verifyingUartData();

/*********************************************************************
 * @fn      peskMoveCommand
 *
 * @brief   Transmit moving commands to the controller.
 *
 * @param   Bool varible that whether your device is locked
 *          Unsigned char type varible that instructions you transmit to the controller
 *
 * @return  none
 */
uint16 peskMoveCommand( uint8 cmd );

/*********************************************************************
 * @fn      device_Send_PeskData
 *
 * @brief   Send message from device to app with characteristic 5
 *
 * @param   send_Interval delay period to send messages to the app
 *
 * @return  return the diffrent value once the data changed 
 */
int16 device_Send_PeskData( uint16 send_Interval );

/*********************************************************************
 * @fn      user_PostureEstimate
 *
 * @brief   user's posture estimate by current height
 *
 * @param   unsigned int type parameter indicates height
 *          unsigned char type parameter the threshold to estimate user's posture 
 *
 * @return  unsigned char type data indicates user's posture
 */
uint8 user_PostureEstimate( uint16 height, uint16 heightThreshold );

/*********************************************************************
 * @fn      device_Info_Setup
 *
 * @brief   Set characteristic3 initial value
 *
 * @param   none
 *
 * @return  none
 */
void device_HeightInfo_Setup();

/*********************************************************************
 * @fn      device_Count_Calculate
 *
 * @brief   Calculate the count value to stop the pesk
 *
 * @param   int16 heightDiffer
 *
 * @return  Count value
 */
int16 device_Count_Calculate( int16 heightDiffer, uint8 standType );

/*********************************************************************
 * @fn      device_HardwareInfo_Setup
 *
 * @brief   Set hardware initial value
 *
 * @param   none
 *
 * @return  none
 */
void device_HardwareInfo_Setup();

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      device_Set_Single_Memory
 *
 * @brief   Set memory presets once it is changed by handset
 *
 * @param   index - the device_Memory_Set's index
 *          ble_NVID - the NVID address of the snv
 *          setHeight - the height that set to memory
 *  
 * @return  none
 */
void device_Set_Single_Memory( int index, uint8 ble_NVID, uint16 setHeight );
#endif

/*********************************************************************
 * @fn      device_Set_Multiple_Memory
 *
 * @brief   Set memory presets once it is changed by app
 *
 * @param   setHeight - the height array that set to memory
 *  
 * @return  none
 */
void device_Set_Multiple_Memory( uint16 *setHeight );

/*********************************************************************
 * @fn      device_Get_Current_Timestamp
 *
 * @brief   Get the current powered up timestamp
 *
 * @param   none
 *  
 * @return  return a timestamp union type value
 */
seprate_DataU32_t device_Get_Current_Timestamp();

/*********************************************************************
 * @fn      device_Get_Current_LockStatus
 *
 * @brief   Get the current lock status
 *
 * @param   none
 *  
 * @return  return a lockstatus
 */
uint8 device_Get_Current_LockStatus( lockData_t data );

/*********************************************************************
 * @fn      timestamp_Reverse
 *
 * @brief   make the timestamp data corrected
 *
 * @param   time - the timestamp type parameter need to be reverse
 *  
 * @return  return a correct timestamp
 */
uint32 timestamp_Reverse( uint32 time );

/*********************************************************************
 * @fn      device_Send_HealthData
 *
 * @brief   Send the health data periodicly by Characteristic 8
 *
 * @param   none
 *  
 * @return  none
 */
void device_Send_HealthData();

/*********************************************************************
 * @fn      device_Set_PeskMoveStatus
 *
 * @brief   Set pesk current move status
 *
 * @param   pesk_Move_PreviousStatus pesk previous move status
 *  
 * @return  none
 */
void device_Set_PeskMoveStatus();

/*********************************************************************
 * @fn      device_Get_HandsetStatus
 *
 * @brief   Get the key that handset is pressed
 *
 * @param   pesk_Move_PreviousStatus pesk previous move status
 *  
 * @return  none
 */
void device_Get_HandsetStatus();

/*********************************************************************
 * @fn      device_Send_CurrentTime
 *
 * @brief   Send the current time periodicly by Characteristic 7
 *
 * @param   none
 *  
 * @return  none
 */
void device_Send_CurrentTime();   

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      device_Set_LockData
 *
 * @brief   Set the lock data by Characteristic6
 *
 * @param   getData - pointer type data that get from app
 *  
 * @return  none
 */
void device_Set_LockData( uint8* getData );
#elif (defined PRODUCT_TYPE_BAR)
/*********************************************************************
 * @fn      device_Set_LockData
 *
 * @brief   Set the lock data by Characteristic6
 *
 * @param   getData - pointer type data that get from app
 *  
 * @return  none
 */
void device_Set_LockData( uint8* getData );
#endif

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      device_Set_Stop
 *
 * @brief   Set device into a stop status.
 *
 * @param   none
 *
 * @return  none
 */
void device_Set_Stop();
#elif (defined PRODUCT_TYPE_BAR)
/*********************************************************************
 * @fn      device_Set_Stop
 *
 * @brief   Set device into a stop status.
 *
 * @param   none
 *
 * @return  none
 */
void device_Set_Stop();
#endif

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      uart_DataHandle
 *
 * @brief   The uart data handler in case of device is bar2
 *
 * @param   numBytes - the bytes number received
 *
 * @return  none
 */
void uart_DataHandle( uint8 numBytes );
#elif (defined PRODUCT_TYPE_BAR)
/*********************************************************************
 * @fn      uart_DataHandle
 *
 * @brief   The uart data handler in case of device is bar
 *
 * @param   numBytes - the bytes number received
 *
 * @return  none
 */
void uart_DataHandle( uint8 numBytes );
#endif

/*********************************************************************
 * @fn      device_Set_Info
 *
 * @brief   Set the initial information data by Characteristic0
 *
 * @param   getData - pointer type data that get from app
 *  
 * @return  none
 */
void device_Set_InitInfo( uint8 *getData );

/*********************************************************************
 * @fn      device_Set_PostureThreshold
 *
 * @brief   Set the user's posture change threshold data by Characteristic2
 *
 * @param   getData - pointer type data that get from app
 *  
 * @return  none
 */
void device_Set_PostureThreshold( uint8 *getData );

/*********************************************************************
 * @fn      device_Count_Calibration
 *
 * @brief   calibrate the pesk stop count to move accurate
 *
 * @param   pesk_Move_Speed - integer type with sign
 *  
 * @return  none
 */
void device_Count_Calibration( int16 pesk_Move_Speed );

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      device_Set_MoveRange
 *
 * @brief   Set the pesk move range data by Characteristic2
 *
 * @param   getData - pointer type data that get from app
 *  
 * @return  none
 */
void device_Set_MoveRange( uint8 *getData );   
#elif (defined PRODUCT_TYPE_BAR)
/*********************************************************************
 * @fn      device_Set_MoveRange
 *
 * @brief   Set the pesk move range data by Characteristic2
 *
 * @param   getData - pointer type data that get from app
 *  
 * @return  none
 */
void device_Set_MoveRange( uint8 *getData ); 
#endif

#ifdef AUTOMOVE_FUNC
/*********************************************************************
 * @fn      autoMove_Reset
 *
 * @brief   Automatic movement data reset
 *
 * @param   posture - uint8 type data get from peskData.userPosture
 *  
 * @return  none
 */
void autoMove_Reset( uint8 posture );

/*********************************************************************
 * @fn      device_Set_AutoMove
 *
 * @brief   Automatic movement data set
 *
 * @param   getData - pointer type of an array
 *  
 * @return  none
 */
void device_Set_AutoMove( uint8 *getData );

/*********************************************************************
 * @fn      autoMove_Move
 *
 * @brief   Automatic movement go
 *
 * @param   data - pointer type data
 *          posture - uint8 type data get from peskData.userPosture
 *  
 * @return  result
 */
void autoMove_Move();

#endif

#endif