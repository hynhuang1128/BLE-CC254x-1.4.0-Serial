/**************************************************************************************************
 * INCLUDES
 */
#include "applicationBLE.h"
#include "hal_types.h"
#include "simpleBLEPeripheral.h"

/**************************************************************************************************
 * CONST
 */
#define UART_MAX_COUNTER      5000
#define PARAM_COUNT           4

const uint16 device_Memory_Default[DEVICE_MEMORY_NUM] = { 750, 800, 1000, 1273, };

/**************************************************************************************************
 * MACROS
 */

/**************************************************************************************************
 * GLOBAL VARIBLES
 */
uint8 pesk_Move_CurrentStatus = PESK_STATUS_IDLE;
uint8 pesk_Move_PreviousStatus = PESK_STATUS_IDLE;
uint8 cmd_Previous = DEVICE_COMMON_STOP;

uint16 uart_DataReceivedCounter;
uint16 device_Height_Destinate;

peskStatus_t peskData;
memorySet_t device_Memory_Set[DEVICE_MEMORY_NUM];
deviceInfo_t device_Init_Info;

/**************************************************************************************************
 * EXTERN VARIBLES
 */
extern uint8 pesk_Lock_Status;
extern uint16 postureChange_Threshold;
extern uint16 pesk_Current_Height;
extern uint8 device_Current_CtrlMode;
extern uint8 user_HealthData_Count;
extern int16 pesk_Stop_Count;
extern uint16 device_PeskMove_Interval;

extern healthData_t user_HealthData[USER_HEALTHDATA_MAX_COUNT];
extern hardwareInfo_t pesk_Hardware_Info;
extern lockData_t device_LockData;
extern bool device_SendHealthData_Enable;
extern bool device_BLE_Connect_Flag;
/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      peskCmdHandle
 */
static void peskCmdHandle( uint8 cmd_H, uint8 cmd_L )
{
  uint8 cmd = cmd_H | cmd_L;
  if( cmd_Previous != cmd )
    {
      pesk_Move_CurrentStatus = PESK_STATUS_SET;
      if( cmd_H == DEVICE_MEMORY_CMD )
      {
        device_Height_Destinate = device_Memory_Set[cmd_L].height_Value;
      }
      else if( cmd_H == DEVICE_COMMON_CMD )
      {
        if( cmd_L == DEVICE_COMMON_UP )
        {
          device_Height_Destinate = pesk_Hardware_Info.height_Maximum;
        }
        else if( cmd_L == DEVICE_COMMON_DOWN )
        {
          device_Height_Destinate = pesk_Hardware_Info.height_Minimum;
        }
        else
        {
          // Discard the useless command
          // Should not get here
        }
      }
    }
    else
    {
      pesk_Move_CurrentStatus = PESK_STATUS_IDLE;
    }
}

/*********************************************************************
 * @fn      device_Get_MemoryInit
 */
static void device_Get_MemoryInit( uint8 *buffer )
{
  uint8 dataTemp[BLE_NVID_MEMORY_HEIGHT_LEN];
  for( int i = 0;
       i < BLE_NVID_MEMORY_HEIGHT_LEN * DEVICE_MEMORY_NUM;
       i += BLE_NVID_MEMORY_HEIGHT_LEN )
  {
    if( osal_snv_read( BLE_NVID_MEMORY_HEIGHT1 + i, BLE_NVID_MEMORY_HEIGHT_LEN, dataTemp ) != SUCCESS )
    {
      device_Memory_Set[i / BLE_NVID_MEMORY_HEIGHT_LEN].height_Value = device_Memory_Default[i / BLE_NVID_MEMORY_HEIGHT_LEN];
    }
    else
    {
      device_Memory_Set[i / BLE_NVID_MEMORY_HEIGHT_LEN].height_H = dataTemp[0];
      device_Memory_Set[i / BLE_NVID_MEMORY_HEIGHT_LEN].height_L = dataTemp[1];
    }
    *(buffer + i) = device_Memory_Set[i / BLE_NVID_MEMORY_HEIGHT_LEN].height_H;
    *(buffer + i + 1) = device_Memory_Set[i / BLE_NVID_MEMORY_HEIGHT_LEN].height_L; 
  }
}

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      device_Get_MoveRange
 */
static void device_Get_MoveRange( uint8 *buffer )
{
  uint8 rangeTemp[BLE_NVID_PESK_RANGE_LEN];
  if( osal_snv_read( BLE_NVID_PESK_MAX, BLE_NVID_PESK_RANGE_LEN, rangeTemp ) !=  SUCCESS )
  {
    switch( device_Init_Info.peskType )
    {
      case PESK_DOUBLESEG_METRIC:
        pesk_Hardware_Info.height_Maximum = PESK_DOUBLESEG_METRIC_MAX;
        break;
        
      case PESK_TRIPLESEG_METRIC:
        pesk_Hardware_Info.height_Maximum = PESK_TRIPLESEG_METRIC_MAX;
        break;
        
      case PESK_DOUBLESEG_IMPERIAL:
        pesk_Hardware_Info.height_Maximum = PESK_DOUBLESEG_IMPERIAL_MAX;
        break;
        
      case PESK_TRIPLESEG_IMPERIAL:
        pesk_Hardware_Info.height_Maximum = PESK_TRIPLESEG_IMPERIAL_MAX;
        break;
        
      default:
        pesk_Hardware_Info.height_Maximum = HEIGHT_VALUE_MAX;
        break;
    }
  }
  else
  {
    pesk_Hardware_Info.height_Maximum = rangeTemp[0] * 256 + rangeTemp[1];
  }
  
  if( osal_snv_read( BLE_NVID_PESK_MIN, BLE_NVID_PESK_RANGE_LEN, rangeTemp ) !=  SUCCESS )
  {
    switch( device_Init_Info.peskType )
    {
      case PESK_DOUBLESEG_METRIC:
        pesk_Hardware_Info.height_Minimum = PESK_DOUBLESEG_METRIC_MIN;
        break;
        
      case PESK_TRIPLESEG_METRIC:
        pesk_Hardware_Info.height_Minimum = PESK_TRIPLESEG_METRIC_MIN;
        break;
        
      case PESK_DOUBLESEG_IMPERIAL:
        pesk_Hardware_Info.height_Minimum = PESK_DOUBLESEG_IMPERIAL_MIN;
        break;
        
      case PESK_TRIPLESEG_IMPERIAL:
        pesk_Hardware_Info.height_Minimum = PESK_TRIPLESEG_IMPERIAL_MIN;
        break;
        
      default:
        pesk_Hardware_Info.height_Minimum = HEIGHT_VALUE_MIN;
        break;
    }
  }
  else
  {
    pesk_Hardware_Info.height_Minimum = rangeTemp[0] * 256 + rangeTemp[1];
  }
  *buffer = pesk_Hardware_Info.height_Maximum / 256;
  *(buffer + 1) = pesk_Hardware_Info.height_Maximum % 256;
  *(buffer + 2) = pesk_Hardware_Info.height_Minimum / 256;
  *(buffer + 3) = pesk_Hardware_Info.height_Minimum % 256;
}
#endif

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      device_Get_Lock
 */
static void device_Get_Lock( uint8 *buffer )
{
  if( osal_snv_read( BLE_NVID_DEVICE_LOCK, BLE_NVID_DEVICE_LOCK_LEN, &device_LockData.lockStatus ) != SUCCESS )
  {
    device_LockData.lockStatus = PESK_UNLOCK;
  }
  *buffer = device_LockData.lockStatus;
  for( int i = 0; i < BLE_NVID_DEVICE_LOCK_LEN - 1; i++)
  {
    *(buffer + 1) = 0x00;
  }
}
#endif

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      device_Get_DeviceInfo
 */
static void device_Get_DeviceInfo( uint8 *buffer )
{
  uint8 tempData[BLE_NVID_HARDWARE_INFO_LEN];
  if( osal_snv_read( BLE_NVID_HARDWARE_INFO, BLE_NVID_HARDWARE_INFO_LEN, tempData ) != SUCCESS )
  {
    device_Init_Info.peskType = DEVICE_PESK_DEFAULT;
    if( pesk_Current_Height < PESK_METRICHEIGHT_MIN && peskData.peskStatus == PESK_STATUS_NORMAL )
    {
      device_Init_Info.peskType |= PESK_UNIT_BIT;
    }
    else
    {
      device_Init_Info.peskType &= ~PESK_UNIT_BIT;
    }
    device_Init_Info.deviceType = DEVICE_TYPE_DEFAULT;
    device_Init_Info.deviceVersion = DEVICE_VERSION_DEFAULT;
  }
  else
  {
    device_Init_Info.deviceType = *tempData;
    device_Init_Info.peskType = *(tempData + 1);
    device_Init_Info.deviceVersion = (*(tempData + 2) << 8) | *(tempData + 3);
  }
  if( !(device_Init_Info.peskType & PESK_TYPE_BITS) )
  {
    device_PeskMove_Interval = PESKMOVE_INTERVAL_DOUBLESEG;
  }
  else
  {
    device_PeskMove_Interval = PESKMOVE_INTERVAL_TRIPLESEG;
  }
  *buffer = device_Init_Info.deviceType;
  *(buffer + 1) = device_Init_Info.peskType;
  *(buffer + 2) = device_Init_Info.version_H;
  *(buffer + 3) = device_Init_Info.version_L;
}
#endif

/*********************************************************************
 * @fn      Data_FIFO_In
 */
FIFO_Data_t * Data_FIFO_In( FIFO_Data_t *head, int16 data, uint32 data_Max )
{
  FIFO_Data_t *p0, *p1, *p2;
  p0 = p1 = p2= head;
  if( head == NULL )
  {
    p0 = (FIFO_Data_t *)osal_mem_alloc( sizeof( FIFO_Data_t ) );
    p0->dataCount = 0;
    p0->dataValue = data;
    p0->addrNext = NULL;
    head = p0;
  }
  else
  {
    if( p0->dataCount < data_Max - 1 )
    {
      while( p0->addrNext != NULL )
      {
        p0 = p0->addrNext;
      }
      p1 = (FIFO_Data_t *)osal_mem_alloc( sizeof( FIFO_Data_t ) );
      p1->dataValue = data;
      p1->addrNext = NULL;
      p0->addrNext = p1;
      head ->dataCount++;
    }
    else
    {
      while( p0->addrNext != NULL )
      {
        p0 = p0->addrNext;
      }
      p1 = (FIFO_Data_t *)osal_mem_alloc( sizeof( FIFO_Data_t ) );
      p1->dataValue = data;
      p1->addrNext = NULL;
      p0->addrNext = p1;
      head = head->addrNext;
      head->dataCount = p2 ->dataCount;
      osal_mem_free(p2);
    }
  }
  return head;
}

/*********************************************************************
 * @fn      device_Speed_Calculate
 */
static int16 device_Speed_Calculate( FIFO_Data_t *speed )
{
  FIFO_Data_t *p0;
  int16 sum = 0;
  p0 = speed;
  while( p0->addrNext != NULL )
  {
    sum += p0->dataValue;
    p0 = p0->addrNext;
  }
  sum += p0->dataValue;
  return sum;
}

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      device_Get_Posture
 */
static void device_Get_Posture( uint8 *buffer )
{
  if( osal_snv_read( BLE_NVID_USER_POSTURE, BLE_NVID_USER_POSTURE_LEN, &postureChange_Threshold ) != SUCCESS )
  {
    if( !(device_Init_Info.peskType & PESK_UNIT_BIT) )
    {
      postureChange_Threshold = USER_POSTURE_THRESHOLD_METRIC;
    }
    else
    {
      postureChange_Threshold = USER_POSTURE_THRESHOLD_IMPERIAL;
    }
  }
  *buffer = postureChange_Threshold / 256;
  *(buffer + 1) = postureChange_Threshold % 256;
}
#endif


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      device_HealthData_Save
 * 
 * @brief   save the health data once user's posture changed
 *
 * @param   userStatus - user posture data
 *
 * @return  success or not
 */
bool device_HealthData_Save( uint8 userStatus )
{
  seprate_DataU32_t timeStampTemp;
  static uint8 userStatus_Previous;
  
  if( userStatus_Previous != userStatus )
  {
    timeStampTemp = device_Get_Current_Timestamp();
    if( user_HealthData_Count < USER_HEALTHDATA_MAX_COUNT )
    {
      user_HealthData[user_HealthData_Count].userStatus = userStatus;
      user_HealthData[user_HealthData_Count].timeStamp = timeStampTemp.dataValue;
      user_HealthData_Count++;
    }
    else
    {
      for( int i = 0; i < (user_HealthData_Count - 1); i++ )
      {
        user_HealthData[i] = user_HealthData[i + 1];
      }
      user_HealthData[user_HealthData_Count - 1].userStatus = userStatus;
      user_HealthData[user_HealthData_Count - 1].timeStamp = timeStampTemp.dataValue;
      user_HealthData_Count = 0;
    }
    userStatus_Previous = userStatus;
    return true;
  }
  else
  {
    return false;
  }
}

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      verifyingUartData
 *
 * @brief   Process the incoming message from JC control box.
 *
 * @param   none
 *
 * @return  none
 */
void verifyingUartData()
{
  if( uart_DataReceivedCounter > UART_MAX_COUNTER )
  {
    uart_DataReceivedCounter = 0;
  } 
  else 
  {
    uart_DataReceivedCounter++;
  }
}
#endif

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      peskMoveCommand
 *
 * @brief   Transmit moving commands to the controller.
 *
 * @param   Bool varible that whether your device is locked
 *          Unsigned char type varible that instructions you transmit to the controller
 *
 * @return  stop time
 */
uint16 peskMoveCommand( uint8 cmd )
{
  uint16 stopTime;
  switch( cmd & 0xf0 )
  {
  case DEVICE_COMMON_CMD:
    if( !pesk_Lock_Status && (device_Current_CtrlMode != DEVICE_CTRL_HANDSET) )
    {
    device_Current_CtrlMode = DEVICE_CTRL_APP;
    stopTime = PESK_STOP_TIMEOUT_COMMON;
    switch( cmd & 0x0f )
    {
      case DEVICE_COMMON_UP:
        peskCmdHandle( DEVICE_COMMON_CMD, DEVICE_COMMON_UP );
        break;
      
      case DEVICE_COMMON_DOWN:
        peskCmdHandle( DEVICE_COMMON_CMD, DEVICE_COMMON_DOWN );
        break;
    
      case DEVICE_COMMON_STOP:
        device_Set_Stop();
        break;
    
      default:
        // Discard the useless command!
        break;
      }
    }
    break;
    
  case DEVICE_MEMORY_CMD:
    if( !pesk_Lock_Status && (device_Current_CtrlMode != DEVICE_CTRL_HANDSET) )
    {
      device_Current_CtrlMode = DEVICE_CTRL_FREE;
      stopTime = PESK_STOP_TIMEOUT_MEMORY;
      switch( cmd & 0x0f )
      {
        case DEVICE_MEMORY_SET1:
          peskCmdHandle( DEVICE_MEMORY_CMD, DEVICE_MEMORY_SET1 );
          break;
      
        case DEVICE_MEMORY_SET2:
          peskCmdHandle( DEVICE_MEMORY_CMD, DEVICE_MEMORY_SET2 );
          break;
        
        case DEVICE_MEMORY_SET3:
          peskCmdHandle( DEVICE_MEMORY_CMD, DEVICE_MEMORY_SET3 );
          break;
      
        case DEVICE_MEMORY_SET4:
          peskCmdHandle( DEVICE_MEMORY_CMD, DEVICE_MEMORY_SET4 );
          break;
        
        default:
          // Discard the useless command!
          break;
      }
    }
    break;
    
  case DEVICE_ADVANCE_CMD:
    device_SendHealthData_Enable = true;
    stopTime = PESK_STOP_TIMEOUT_ADVANCE;
    break;
    
  default:
    // Discard the useless command!
    break;
  }
  cmd_Previous = cmd;
  return stopTime;
}
#endif

/*********************************************************************
 * @fn      device_Send_PeskData
 *
 * @brief   Send message from device to app with characteristic 5
 *
 * @param   send_Interval delay period to send messages to the app
 *
 * @return  return the diffrent value once the data changed 
 */
int16 device_Send_PeskData( uint16 send_Interval )
{
  uint8 *buffer;
  int16 speed = 0;
  static FIFO_Data_t *speed_FIFO;
  static peskStatus_t previousData, previousData_Send;
  static uint16 interval_Count;
  
  speed_FIFO = Data_FIFO_In( speed_FIFO, (peskData.info - previousData.info), FIFO_DATA_MAX );
  speed = device_Speed_Calculate( speed_FIFO );
  if( previousData.currentPeskData != peskData.currentPeskData )
  {
    previousData = peskData;
  }
  
  if( previousData_Send.currentPeskData != peskData.currentPeskData && 
      interval_Count > send_Interval )
  {
    previousData_Send = peskData;
    
    interval_Count = 0;
    buffer = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR5_LEN );
    *buffer = peskData.peskStatus;
    *(buffer + 1) = peskData.userPosture;
    *(buffer + 2) = peskData.info_H;
    *(buffer + 3) = peskData.info_L;
    
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, buffer );
    
    osal_mem_free( buffer );
  }
  interval_Count++;
  return speed;
}

#if (defined PRODUCT_TYPE_BAR) || (defined PRODUCT_TYPE_BAR2)
/*********************************************************************
 * @fn      user_PostureEstimate
 *
 * @brief   user's posture estimate by current height
 *
 * @param   unsigned int type parameter indicates height
 *
 * @return  unsigned char type data indicates user's posture
 */
uint8 user_PostureEstimate( uint16 height, uint16 heightThreshold )
{
  uint8 posture;
  if( height >= heightThreshold )
  {
    posture = USER_STATUS_STAND;
  }
  else
  {
    posture = USER_STATUS_SIT;
  }
  return posture;
}
#endif

/*********************************************************************
 * @fn      device_HeightInfo_Setup
 *
 * @brief   Set characteristic5 initial value
 *
 * @param   none
 *
 * @return  none
 */
void device_HeightInfo_Setup()
{
  uint8 *buffer;
  buffer = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR5_LEN );
  *buffer = peskData.peskStatus;
  *(buffer + 1) = peskData.userPosture;
  *(buffer + 2) = peskData.info_H;
  *(buffer + 3) = peskData.info_L;
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, buffer );
  osal_mem_free( buffer );
}

/*********************************************************************
 * @fn      device_HardwareInfo_Setup
 *
 * @brief   Set hardware initial value
 *
 * @param   none
 *
 * @return  none
 */
void device_HardwareInfo_Setup()
{
  uint8 *buffer;
  
  buffer = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR1_LEN );
  device_Get_MemoryInit( buffer );
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, buffer );
  osal_mem_free( buffer );
  
  buffer = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR6_LEN );
  device_Get_Lock( buffer );
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, buffer );
  osal_mem_free( buffer );
  
  buffer = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR0_LEN );
  device_Get_DeviceInfo( buffer );
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR0, SIMPLEPROFILE_CHAR0_LEN, buffer );
  osal_mem_free( buffer );
  
  buffer = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR3_LEN );
  device_Get_MoveRange( buffer );
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, buffer );
  osal_mem_free( buffer );
  
  buffer = (uint8 *)osal_mem_alloc( sizeof( uint8 ) * SIMPLEPROFILE_CHAR2_LEN );
  device_Get_Posture( buffer );
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, buffer );
  osal_mem_free( buffer );
  
  /* start the watch dog timer */
  DEVICE_WDT_START();
}

/*********************************************************************
 * @fn      device_Count_Calculate
 *
 * @brief   Calculate the count value to stop the pesk
 *
 * @param   int16 heightDiffer
 *          int16 inertiaValue
 *
 * @return  Count value
 */
int16 device_Count_Calculate( int16 heightDiffer, uint8 standType )
{
  int16 countValue;
  uint16 pesk_Max_Default;
  uint16 pesk_Min_Default;
  int16 differ_Destinate_Critical = 0;
  int16 differ_Current_Critical = 0;
  bool destinate_InCritical = false;
  bool over_Critical_Flag = false;
  /*-----------------------------------------
   * arg1[][0] for parameter_Double upward
   * arg1[][1] for parameter_Triple upward
   * arg1[][2] for parameter_Double downward
   * arg1[][3] for parameter_Triple downward
   */
  static int16 arg1[2][4] = {
                              { 2, -30, 2, -25 },  
                              { 2, -13, 2, -13 },                      
                            };
  static int16 arg2[2][4] = {
                              { -6, 7, 3, -7, },  
                              { -5, 7, 4, -9, },
                            };
  static int16 inertiaValue;
  
  switch( standType )
  {
    case PESK_TRIPLESEG_IMPERIAL:
      pesk_Max_Default = PESK_TRIPLESEG_IMPERIAL_MAX;
      pesk_Min_Default = PESK_TRIPLESEG_IMPERIAL_MIN;
      inertiaValue = 2;
      break;
      
    case PESK_TRIPLESEG_METRIC:
      pesk_Max_Default = PESK_TRIPLESEG_METRIC_MAX;
      pesk_Min_Default = PESK_TRIPLESEG_METRIC_MIN;
      inertiaValue = 4;
      break;
      
    case PESK_DOUBLESEG_IMPERIAL:
      pesk_Max_Default = PESK_DOUBLESEG_IMPERIAL_MAX;
      pesk_Min_Default = PESK_DOUBLESEG_IMPERIAL_MIN;
      inertiaValue = 2;
      break;
      
    case PESK_DOUBLESEG_METRIC:
      pesk_Max_Default = PESK_DOUBLESEG_METRIC_MAX;
      pesk_Min_Default = PESK_DOUBLESEG_METRIC_MIN;
      inertiaValue = 4;
      break;

    default:
      // Should not get here
      break;
  }
  
  if( heightDiffer > 0 ) /* upward */
  {
    if( pesk_Max_Default - device_Height_Destinate < PESK_CRITICAL_VALUE )
    {
      destinate_InCritical = true;
      differ_Destinate_Critical = device_Height_Destinate - (pesk_Max_Default - PESK_CRITICAL_VALUE);
      if(heightDiffer - differ_Destinate_Critical > 0)
      {
        differ_Current_Critical = heightDiffer - differ_Destinate_Critical;
      }
      else
      {
        differ_Current_Critical = 0;
        over_Critical_Flag = true;
      }
    }
    
    if( heightDiffer < inertiaValue )
    {
      countValue = 0;
    }
    else
    {
      // FIXME adjust the count value
      if( !destinate_InCritical )
      {
        switch(standType)
        {
          case PESK_TRIPLESEG_IMPERIAL:
            countValue = TRANSFER_IMPERIAL_TO_METRIC(heightDiffer) * 2
                       + TRANSFER_IMPERIAL_TO_METRIC(heightDiffer) / arg1[device_BLE_Connect_Flag][1]
                       + arg2[device_BLE_Connect_Flag][1];
            break;
      
          case PESK_TRIPLESEG_METRIC:
            countValue = heightDiffer * 2 + heightDiffer / arg1[device_BLE_Connect_Flag][1] + arg2[device_BLE_Connect_Flag][1];
            break;
      
          case PESK_DOUBLESEG_IMPERIAL:
            countValue = TRANSFER_IMPERIAL_TO_METRIC(heightDiffer) * 2
                       + TRANSFER_IMPERIAL_TO_METRIC(heightDiffer) / arg1[device_BLE_Connect_Flag][0]
                       + arg2[device_BLE_Connect_Flag][0];
            break;
      
          case PESK_DOUBLESEG_METRIC:
            countValue = heightDiffer * 2 + heightDiffer / arg1[device_BLE_Connect_Flag][0] + arg2[device_BLE_Connect_Flag][0];
            break;

          default:
            // Should not get here!
            break;
        }
      }
      else /* in critical */
      {
        switch(standType)
        {
          case PESK_TRIPLESEG_IMPERIAL:
            countValue = TRANSFER_IMPERIAL_TO_METRIC(differ_Current_Critical) * 2
                       + TRANSFER_IMPERIAL_TO_METRIC(differ_Current_Critical) / arg1[device_BLE_Connect_Flag][1]
                       + arg2[device_BLE_Connect_Flag][1] * (!over_Critical_Flag)
                       + (differ_Destinate_Critical - heightDiffer) * 3 * over_Critical_Flag
                       + (heightDiffer - differ_Destinate_Critical) * 3 * (!over_Critical_Flag);
            break;
      
          case PESK_TRIPLESEG_METRIC:
            countValue = differ_Current_Critical * 2
                       + differ_Current_Critical / arg1[device_BLE_Connect_Flag][1]
                       + arg2[device_BLE_Connect_Flag][1] * (!over_Critical_Flag)
                       + (differ_Destinate_Critical - heightDiffer) * 3 * over_Critical_Flag
                       + (heightDiffer - differ_Destinate_Critical) * 3 * (!over_Critical_Flag);
            break;
      
          case PESK_DOUBLESEG_IMPERIAL:
            countValue = TRANSFER_IMPERIAL_TO_METRIC(differ_Current_Critical) * 2
                       + TRANSFER_IMPERIAL_TO_METRIC(differ_Current_Critical) / arg1[device_BLE_Connect_Flag][0] 
                       + arg1[device_BLE_Connect_Flag][0] * (!over_Critical_Flag)
                       + (differ_Destinate_Critical - heightDiffer) * 3 * over_Critical_Flag
                       + (heightDiffer - differ_Destinate_Critical) * 3 * (!over_Critical_Flag);
            break;
      
          case PESK_DOUBLESEG_METRIC:
            countValue = differ_Current_Critical * 2
                       + differ_Current_Critical / arg1[device_BLE_Connect_Flag][0] 
                       + arg1[device_BLE_Connect_Flag][0] * (!over_Critical_Flag)
                       + (differ_Destinate_Critical - heightDiffer) * 3 * over_Critical_Flag
                       + (heightDiffer - differ_Destinate_Critical) * 3 * (!over_Critical_Flag);
            break;

          default:
            // Should not get here!
            break;
        }
      }
    }
  }
  else if( heightDiffer < 0 ) /* downward */
  {
    if( device_Height_Destinate - pesk_Min_Default < PESK_CRITICAL_VALUE )
    {
      destinate_InCritical = true;
      differ_Destinate_Critical = device_Height_Destinate - (pesk_Min_Default + PESK_CRITICAL_VALUE);
      if(heightDiffer - differ_Destinate_Critical < 0)
      {
        differ_Current_Critical = heightDiffer - differ_Destinate_Critical;
      }
      else
      {
        differ_Current_Critical = 0;
        over_Critical_Flag = true;
      }
    }
    if( heightDiffer > -inertiaValue )
    {
      countValue = 0;
    }
    else
    {
      // FIXME adjust the count value
      if( !destinate_InCritical )
      {
        switch(standType)
        {
          case PESK_TRIPLESEG_IMPERIAL:
            countValue = TRANSFER_IMPERIAL_TO_METRIC(heightDiffer) * 2
                       + TRANSFER_IMPERIAL_TO_METRIC(heightDiffer) / arg1[device_BLE_Connect_Flag][3]
                       + arg2[device_BLE_Connect_Flag][3];
            break;
      
          case PESK_TRIPLESEG_METRIC:
            countValue = heightDiffer * 2 + heightDiffer / arg1[device_BLE_Connect_Flag][3] + arg2[device_BLE_Connect_Flag][3];
            break;
      
          case PESK_DOUBLESEG_IMPERIAL:
            countValue = TRANSFER_IMPERIAL_TO_METRIC(heightDiffer) * 2
                       + TRANSFER_IMPERIAL_TO_METRIC(heightDiffer) / arg1[device_BLE_Connect_Flag][2]
                       + arg2[device_BLE_Connect_Flag][2];
            break;
      
          case PESK_DOUBLESEG_METRIC:
            countValue = heightDiffer * 2 + heightDiffer / arg1[device_BLE_Connect_Flag][2] + arg2[device_BLE_Connect_Flag][2];
            break;

          default:
            // Should not get here!
            break;
        }
      }
      else /* in critical */
      {
        switch(standType)
        {
          case PESK_TRIPLESEG_IMPERIAL:
            countValue = TRANSFER_IMPERIAL_TO_METRIC(differ_Current_Critical) * 2
                       + TRANSFER_IMPERIAL_TO_METRIC(differ_Current_Critical) / arg1[device_BLE_Connect_Flag][3]
                       + arg1[device_BLE_Connect_Flag][3] * (!over_Critical_Flag)
                       + (differ_Destinate_Critical - heightDiffer) * 3 * over_Critical_Flag
                       + (heightDiffer - differ_Destinate_Critical) * 3 * (!over_Critical_Flag);
            break;
      
          case PESK_TRIPLESEG_METRIC:
            countValue = differ_Current_Critical * 2
                       + differ_Current_Critical / arg1[device_BLE_Connect_Flag][3]
                       + arg1[device_BLE_Connect_Flag][3] * (!over_Critical_Flag)
                       + (differ_Destinate_Critical - heightDiffer) * 3 * over_Critical_Flag
                       + (heightDiffer - differ_Destinate_Critical) * 3 * (!over_Critical_Flag);
            break;
      
          case PESK_DOUBLESEG_IMPERIAL:
            countValue = TRANSFER_IMPERIAL_TO_METRIC(differ_Current_Critical) * 2
                       + TRANSFER_IMPERIAL_TO_METRIC(differ_Current_Critical) / arg1[device_BLE_Connect_Flag][2]
                       + arg1[device_BLE_Connect_Flag][2] * (!over_Critical_Flag)
                       + (differ_Destinate_Critical - heightDiffer) * 3 * over_Critical_Flag
                       + (heightDiffer - differ_Destinate_Critical) * 3 * (!over_Critical_Flag);
            break;
      
          case PESK_DOUBLESEG_METRIC:
            countValue = differ_Current_Critical * 2
                       + differ_Current_Critical / arg1[device_BLE_Connect_Flag][2]
                       + arg1[device_BLE_Connect_Flag][2] * (!over_Critical_Flag)
                       + (differ_Destinate_Critical - heightDiffer) * 3 * over_Critical_Flag
                       + (heightDiffer - differ_Destinate_Critical) * 3 * (!over_Critical_Flag);
            break;

          default:
            // Should not get here!
            break;
        }
      }
    }
  }
  else
  {
    countValue = 0;
  }
  return countValue;
}

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
void device_Set_Single_Memory( int index, uint8 ble_NVID, uint16 setHeight )
{
  uint8 dataTemp[BLE_NVID_MEMORY_HEIGHT_LEN];
  uint8 buffer[SIMPLEPROFILE_CHAR1_LEN];
  
  device_Memory_Set[index].height_Value = setHeight;
  dataTemp[0] = device_Memory_Set[index].height_H;
  dataTemp[1] = device_Memory_Set[index].height_L;
  osal_snv_write( ble_NVID, BLE_NVID_MEMORY_HEIGHT_LEN, dataTemp );
  for( int i = 0; i < DEVICE_MEMORY_NUM; i ++ )
  {
    *(buffer + i * 2) = device_Memory_Set[i].height_H;
    *(buffer + i * 2 + 1) = device_Memory_Set[i].height_L; 
  }
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, buffer );
}
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
void device_Set_Multiple_Memory( uint16 *setHeight )
{
  uint8 buffer[SIMPLEPROFILE_CHAR1_LEN];
  uint8 dataTemp[BLE_NVID_MEMORY_HEIGHT_LEN];
  
  for( int i = 0; i < DEVICE_MEMORY_NUM; i++ )
  {
    if( *(setHeight + i) != VALUE_INGNORED )
    {
      device_Memory_Set[i].height_Value = *(setHeight + i);
      dataTemp[0] = device_Memory_Set[i].height_H;
      dataTemp[1] = device_Memory_Set[i].height_L;
      osal_snv_write( (BLE_NVID_MEMORY_HEIGHT1 + i * BLE_NVID_MEMORY_HEIGHT_LEN), BLE_NVID_MEMORY_HEIGHT_LEN, dataTemp );
    }
    *(buffer + i * 2) = device_Memory_Set[i].height_H;
    *(buffer + i * 2 + 1) =  device_Memory_Set[i].height_L;
  }
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, buffer );
}

/*********************************************************************
 * @fn      timestamp_Reverse
 *
 * @brief   make the timestamp data corrected
 *
 * @param   time - the timestamp type parameter need to be reverse
 *  
 * @return  return a correct timestamp
 */
uint32 timestamp_Reverse( uint32 time )
{
  uint32 reverse_Temp = 0;
  
  uint8 i = 0;
  while( i < 4 )
  {
    reverse_Temp |= ( ( ( (time >> (8 * i)) & 0x000000ff ) ) << (8 * (3 - i)) ) ;
    i++;
  }
  return reverse_Temp;
}

/*********************************************************************
 * @fn      device_Get_Current_Timestamp
 *
 * @brief   Get the current powered up timestamp
 *
 * @param   none
 *  
 * @return  return a timestamp union type value
 */
seprate_DataU32_t device_Get_Current_Timestamp()
{
  seprate_DataU32_t timeStampTemp;
  osalTimeUpdate();
  timeStampTemp.dataValue = timestamp_Reverse( osal_getClock() );
  return timeStampTemp;
}

/*********************************************************************
 * @fn      device_Get_Current_LockStatus
 *
 * @brief   Get the current lock status
 *
 * @param   none
 *  
 * @return  return a lockstatus
 */
uint8 device_Get_Current_LockStatus( lockData_t data )
{
  uint8 *buffer;
  
  buffer = (uint8 *)osal_mem_alloc( sizeof(uint8) * SIMPLEPROFILE_CHAR6_LEN );
  
  *(buffer) = data.lockStatus;
  for( int i = 1; i < SIMPLEPROFILE_CHAR6_LEN; i++)
  {
    *(buffer + i) = data.timeStamp_S[SIMPLEPROFILE_CHAR6_LEN - i - 1];
  }
  
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, buffer );
  osal_mem_free( buffer );
  if( data.lockStatus && !data.timeStamp )
  {
    return PESK_LOCKED;
  }
  else
  {
    return PESK_UNLOCK;
  }
}

/*********************************************************************
 * @fn      device_Send_HealthData
 *
 * @brief   Send the health data periodicly by Characteristic 8
 *
 * @param   none
 *  
 * @return  none
 */
void device_Send_HealthData()
{
  uint8 *sendBuffer_Char8;
  uint8 *getBuffer;
  static uint8 healthData_Send_Count;
  
  if( device_SendHealthData_Enable )
    {
      sendBuffer_Char8 = (uint8 *)osal_mem_alloc( sizeof(uint8) * SIMPLEPROFILE_CHAR8_LEN );
      getBuffer = (uint8 *)user_HealthData + (SIMPLEPROFILE_CHAR8_LEN - 1) * healthData_Send_Count;
      *sendBuffer_Char8 = healthData_Send_Count++;
      for( int i = 0; i < SIMPLEPROFILE_CHAR8_LEN - 1 ; i++ )
      {
        *(sendBuffer_Char8 + i + 1) = *(getBuffer + i); 
      }
      SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR8, SIMPLEPROFILE_CHAR8_LEN, sendBuffer_Char8 ); 
      osal_mem_free( sendBuffer_Char8 );
      if( healthData_Send_Count >= USER_HEALTHDATA_MAX_COUNT / 3 )
      {
        healthData_Send_Count = 0;
        device_SendHealthData_Enable = false;
      }
    }
}

/*********************************************************************
 * @fn      device_Send_CurrentTime
 *
 * @brief   Send the current time periodicly by Characteristic 7
 *
 * @param   none
 *  
 * @return  none
 */
void device_Send_CurrentTime()
{
  seprate_DataU32_t device_Current_Time;
  uint8 *sendBuffer_Char7;
  
  device_Current_Time = device_Get_Current_Timestamp();
  sendBuffer_Char7 = (uint8 *)osal_mem_alloc( sizeof(uint8) * SIMPLEPROFILE_CHAR7_LEN );
  for( int i = 0; i < SIMPLEPROFILE_CHAR7_LEN; i++ )
  {
    *(sendBuffer_Char7 + i) = device_Current_Time.data[i];
  }
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7, SIMPLEPROFILE_CHAR7_LEN, sendBuffer_Char7 );
  osal_mem_free( sendBuffer_Char7 );
}

/*********************************************************************
 * @fn      device_Set_LockData
 *
 * @brief   Set the lock data by Characteristic6
 *
 * @param   getData - pointer type data that get from app
 *  
 * @return  none
 */
void device_Set_LockData( uint8* getData )
{
  device_LockData.lockStatus = *getData;
  for( int i = 1; i < SIMPLEPROFILE_CHAR6_LEN; i++ )
  {
    device_LockData.timeStamp_S[SIMPLEPROFILE_CHAR6_LEN - i - 1] = *(getData + i);
  }
  osal_snv_write( BLE_NVID_DEVICE_LOCK, BLE_NVID_DEVICE_LOCK_LEN, &device_LockData.lockStatus );
}

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
void device_Set_Stop()
{
  device_Current_CtrlMode = DEVICE_CTRL_FREE;
  pesk_Move_CurrentStatus = PESK_STATUS_IDLE;
  cmd_Previous = DEVICE_COMMON_STOP;
  pesk_Stop_Count = 0;
}
#endif

#pragma inline = forced
static bool INIT_INFO_ASSERTION( uint8 *data )
{
  if( data[1] == PESK_TRIPLESEG_IMPERIAL ||
      data[1] == PESK_TRIPLESEG_METRIC ||
      data[1] == PESK_DOUBLESEG_IMPERIAL ||
      data[1] == PESK_DOUBLESEG_METRIC )
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*********************************************************************
 * @fn      device_Set_InitInfo
 *
 * @brief   Set the initial information data by Characteristic0
 *
 * @param   getData - pointer type data that get from app
 *  
 * @return  none
 */
void device_Set_InitInfo( uint8 *getData )
{
  if( INIT_INFO_ASSERTION( getData ) )
  {
    osal_snv_write( BLE_NVID_HARDWARE_INFO, BLE_NVID_HARDWARE_INFO_LEN, getData );
  }
  device_HardwareInfo_Setup();
}

/*********************************************************************
 * @fn      device_Set_PostureThreshold
 *
 * @brief   Set the user's posture change threshold data by Characteristic2
 *
 * @param   getData - pointer type data that get from app
 *  
 * @return  none
 */
void device_Set_PostureThreshold( uint8 *getData )
{
  uint8 *sendBuffer_Char2;
  postureChange_Threshold = *(getData) * 256 + *(getData + 1);
  sendBuffer_Char2 = getData;
  osal_snv_write( BLE_NVID_USER_POSTURE, BLE_NVID_USER_POSTURE_LEN, getData );
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, sendBuffer_Char2 );
}

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
void device_Set_MoveRange( uint8 *getData )
{
  uint8 sendBuffer_Char3[SIMPLEPROFILE_CHAR3_LEN];
  seprate_DataU16_t heightBuffer[2];
  heightBuffer[0].data_H = getData[0];
  heightBuffer[0].data_L = getData[1];
  heightBuffer[1].data_H = getData[2];
  heightBuffer[1].data_L = getData[3];
  if( heightBuffer[0].dataValue )
  {
    pesk_Hardware_Info.height_Maximum = heightBuffer[0].dataValue;
    osal_snv_write( BLE_NVID_PESK_MAX, BLE_NVID_PESK_RANGE_LEN, getData );
  }
  else
  {
    heightBuffer[0].dataValue = pesk_Hardware_Info.height_Maximum;
  }
  if( heightBuffer[1].dataValue )
  {
    pesk_Hardware_Info.height_Minimum = heightBuffer[1].dataValue;
    osal_snv_write( BLE_NVID_PESK_MIN, BLE_NVID_PESK_RANGE_LEN, getData + 2 );
  }
  else
  {
    heightBuffer[1].dataValue = pesk_Hardware_Info.height_Minimum;
  }
  sendBuffer_Char3[0] = heightBuffer[0].data_H;
  sendBuffer_Char3[1] = heightBuffer[0].data_L;
  sendBuffer_Char3[2] = heightBuffer[1].data_H;
  sendBuffer_Char3[3] = heightBuffer[1].data_L;
  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, sendBuffer_Char3 );
}
#endif

/*********************************************************************
 * @fn      device_Set_PeskMoveStatus
 *
 * @brief   Set pesk current move status
 *
 * @param   none
 *  
 * @return  none
 */
void device_Set_PeskMoveStatus()
{
  if( peskData.peskStatus == PESK_STATUS_NORMAL )
  {
    pesk_Current_Height = peskData.info;
  }
  else if( peskData.peskStatus != PESK_STATUS_SAVE )
  {
    pesk_Current_Height = pesk_Hardware_Info.height_Maximum;
  }
  
  if( pesk_Move_CurrentStatus == PESK_STATUS_IDLE )
  {
    if( pesk_Move_PreviousStatus == PESK_STATUS_SET )
    {
      if( pesk_Stop_Count < 0 )
      {
        DEVICE_SENDCMD_PESK(CMD_PESK_DOWN);
        pesk_Stop_Count++;
      }
      else if( pesk_Stop_Count > 0 )
      {
        DEVICE_SENDCMD_PESK(CMD_PESK_UP);
        pesk_Stop_Count--;
      }
      else
      {
        DEVICE_SENDCMD_PESK(CMD_PESK_STOP);
        pesk_Move_PreviousStatus = PESK_STATUS_IDLE;
      }
    }
    else if( pesk_Move_PreviousStatus != PESK_STATUS_IDLE )
    {
      DEVICE_SENDCMD_PESK(CMD_PESK_STOP);
      pesk_Move_PreviousStatus = PESK_STATUS_IDLE;
    }
  }
  else
  {
    if( pesk_Move_CurrentStatus != pesk_Move_PreviousStatus )
    {
      
      switch( pesk_Move_CurrentStatus )
      {
        case PESK_STATUS_UP:
          if( pesk_Current_Height < pesk_Hardware_Info.height_Maximum )
          {
            device_Height_Destinate = pesk_Hardware_Info.height_Maximum;
            pesk_Stop_Count = device_Count_Calculate( (int16)(device_Height_Destinate - pesk_Current_Height),
                                                     device_Init_Info.peskType );
          }
          break;
          
        case PESK_STATUS_DOWN:
          if( pesk_Current_Height > pesk_Hardware_Info.height_Minimum )
          {
            device_Height_Destinate = pesk_Hardware_Info.height_Minimum;
            pesk_Stop_Count = device_Count_Calculate( (int16)(device_Height_Destinate - pesk_Current_Height),
                                                     device_Init_Info.peskType );
          }
          break;
            
        case PESK_STATUS_SET:
          if( device_Height_Destinate > pesk_Hardware_Info.height_Maximum )
          {
            device_Height_Destinate = pesk_Hardware_Info.height_Maximum;
          }
          else if( device_Height_Destinate < pesk_Hardware_Info.height_Minimum )
          {
            device_Height_Destinate = pesk_Hardware_Info.height_Minimum;
          }
          pesk_Stop_Count = device_Count_Calculate( (int16)(device_Height_Destinate - pesk_Current_Height),
                                                     device_Init_Info.peskType );
          break;
        
        default:
          // should not get here
          break;
      }
      pesk_Move_PreviousStatus = pesk_Move_CurrentStatus;
    }
    else
    {
      if( !pesk_Stop_Count )
      {
        DEVICE_SENDCMD_PESK(CMD_PESK_STOP);
        pesk_Move_CurrentStatus = PESK_STATUS_IDLE;
        cmd_Previous = DEVICE_COMMON_STOP;
        device_Current_CtrlMode = DEVICE_CTRL_HANDSET;
      }
      else
      {
        switch( pesk_Move_CurrentStatus )
        {
        case PESK_STATUS_UP:
          if( pesk_Current_Height < pesk_Hardware_Info.height_Maximum )
          {
            DEVICE_SENDCMD_PESK(CMD_PESK_UP);
            pesk_Stop_Count--;
          }
          else
          {
            DEVICE_SENDCMD_PESK(CMD_PESK_STOP);
            pesk_Stop_Count = 0;
          }
          break;
            
        case PESK_STATUS_DOWN:
          if( pesk_Current_Height > pesk_Hardware_Info.height_Minimum )
          {
            DEVICE_SENDCMD_PESK(CMD_PESK_DOWN);
            pesk_Stop_Count++;
          }
          else
          {
            DEVICE_SENDCMD_PESK(CMD_PESK_STOP);
            pesk_Stop_Count = 0;
          }
          break;
            
        default:
          // should not get here!
          break;
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      device_Get_HandsetStatus
 *
 * @brief   Get the current handset status
 *
 * @param   none
 *  
 * @return  none
 */
void device_Get_HandsetStatus()
{
  
  static uint8 handset_CurrentPressed;
  static uint8 handset_PreviousPressed;
  
  GET_CURRENT_HANDSET();
  if( device_Current_CtrlMode != DEVICE_CTRL_APP )
  {
    if( handset_CurrentPressed == HANDSET_STATUS_IDLE )
    {
      device_Current_CtrlMode = DEVICE_CTRL_FREE;
      pesk_Move_CurrentStatus = PESK_STATUS_IDLE;
      handset_PreviousPressed = HANDSET_STATUS_IDLE;
      /* i added this line to solve the problem that once pressed the setting button,
      handset will continuousily sending the setting command to the control box */
      if( pesk_Move_PreviousStatus != PESK_STATUS_SET )
      {
        DEVICE_SENDCMD_PESK( CMD_PESK_STOP );
      }
    }
    else
    {
      device_Current_CtrlMode = DEVICE_CTRL_HANDSET;
      if( !pesk_Lock_Status )
      {
        if( handset_PreviousPressed != handset_CurrentPressed )
        {
          if( pesk_Move_PreviousStatus != PESK_STATUS_SET )
          {
            switch( handset_CurrentPressed )
            {
            case HANDSET_STATUS_UP:
              pesk_Move_CurrentStatus = PESK_STATUS_UP;
              break;
              
            case HANDSET_STATUS_DOWN:
              pesk_Move_CurrentStatus = PESK_STATUS_DOWN;
              break;
              
            case HANDSET_STATUS_SET1:
              if( peskData.peskStatus != PESK_STATUS_SAVE )
              {
                if( device_Memory_Set[0].height_Value != VALUE_INVALID )
                {
                  pesk_Move_CurrentStatus = PESK_STATUS_SET;
                  device_Height_Destinate = device_Memory_Set[0].height_Value;
                }
              }
              else
              {
                device_Set_Single_Memory( 0, BLE_NVID_MEMORY_HEIGHT1, pesk_Current_Height );
                DEVICE_SENDCMD_PESK( CMD_PESK_SET1 );
              }
              break;
            
            case HANDSET_STATUS_SET2:
              if( peskData.peskStatus != PESK_STATUS_SAVE )
              {
                if( device_Memory_Set[1].height_Value != VALUE_INVALID )
                {
                  pesk_Move_CurrentStatus = PESK_STATUS_SET;
                  device_Height_Destinate = device_Memory_Set[1].height_Value;
                }
              }
              else
              {
                device_Set_Single_Memory( 1, BLE_NVID_MEMORY_HEIGHT2, pesk_Current_Height );
                DEVICE_SENDCMD_PESK( CMD_PESK_SET2 );
              }
              break;
              
            case HANDSET_STATUS_SET3:
              if( peskData.peskStatus != PESK_STATUS_SAVE )
              {
                if( device_Memory_Set[2].height_Value != VALUE_INVALID )
                {
                  pesk_Move_CurrentStatus = PESK_STATUS_SET;
                  device_Height_Destinate = device_Memory_Set[2].height_Value;
                }
              }
              else
              {
                device_Set_Single_Memory( 2, BLE_NVID_MEMORY_HEIGHT3, pesk_Current_Height );
                DEVICE_SENDCMD_PESK( CMD_PESK_SET3 );
              } 
              break;
              
            case HANDSET_STATUS_SET4:
              if( peskData.peskStatus != PESK_STATUS_SAVE )
              {
                if( device_Memory_Set[3].height_Value != VALUE_INVALID )
                {
                  pesk_Move_CurrentStatus = PESK_STATUS_SET;
                  device_Height_Destinate = device_Memory_Set[3].height_Value;
                }
              }
              else
              {
                device_Set_Single_Memory( 3, BLE_NVID_MEMORY_HEIGHT4, pesk_Current_Height );
                DEVICE_SENDCMD_PESK( CMD_PESK_SET4 );
              }  
              break;
                
            case HANDSET_STATUS_SETTING:
              pesk_Move_CurrentStatus = PESK_STATUS_IDLE;
              DEVICE_SENDCMD_PESK( CMD_PESK_SETTING );
              break;
                
            default:
              // should not get here!
              break;
            }
          }
          else
          {
            pesk_Move_CurrentStatus = PESK_STATUS_IDLE;
            pesk_Stop_Count = 0;
          }
          handset_PreviousPressed = handset_CurrentPressed;
        }
      }
    }
  }
}

#if (defined PRODUCT_TYPE_BAR2) || (defined PRODUCT_TYPE_CUBE)
/*********************************************************************
 * @fn      uart_DataHandle
 *
 * @brief   The uart data handler in case of device is bar2 or cube
 *
 * @param   numBytes - the bytes number received
 *  
 * @return  none
 */
void uart_DataHandle( uint8 numBytes )
{
  static uint8 buffer[L_RCV_JC_LEN];
  static seprate_DataU32_t previous_PeskData,current_PeskData; 
  if ( (numBytes == L_RCV_JC_LEN) && (uart_DataReceivedCounter > RX_TIME_DELAY) )
  {
    if ( uart_DataReceivedCounter < RX_TIME_OVERFLOW )
    {
      NPI_ReadTransport( buffer, numBytes );
    }
    else
    {
      NPI_ClearTransport( numBytes );
    }

    current_PeskData.data[0] = *( buffer );
    current_PeskData.data[1] = *( buffer + 1 );
    current_PeskData.data[2] = *( buffer + 2 );
    current_PeskData.data[3] = *( buffer + 3 );
    if( previous_PeskData.dataValue == PESKDATA_IN_RST &&
        ( current_PeskData.dataValue == PESKDATA_RST_ERRCODE ||
          ( (current_PeskData.dataValue & PESKDATA_IN_NORMAL_FILTER) != PESKDATA_IN_ERROR && 
            (current_PeskData.dataValue & PESKDATA_IN_NORMAL_FILTER) != PESKDATA_IN_NORMAL ) ||
          current_PeskData.dataValue == PESKDATA_RST_ERRCODE2 ||
          current_PeskData.dataValue == PESKDATA_RST_ERRCODE3) )
    {
      current_PeskData.dataValue = PESKDATA_IN_RST;
    }
    else if( (current_PeskData.dataValue & PESKDATA_IN_NORMAL_FILTER) == PESKDATA_IN_SLEEP )
    {
      current_PeskData = previous_PeskData;
    }
    else
    {
      peskData.peskStatus = *( buffer+1 );
      peskData.info_H = *( buffer+2 );
      peskData.info_L = *( buffer+3 );
    }
    if( previous_PeskData.dataValue != current_PeskData.dataValue )
    {
      previous_PeskData = current_PeskData;
    }
  }
  else if ( numBytes > L_RCV_JC_LEN )
  {
    NPI_ClearTransport( numBytes );
  }
  uart_DataReceivedCounter = 0;
}
#endif

/*********************************************************************
 * @fn      device_Count_Calibration
 *
 * @brief   calibrate the pesk stop count to move accurate
 *
 * @param   pesk_Move_Speed - integer type with sign
 *  
 * @return  none
 */
void device_Count_Calibration( int16 pesk_Move_Speed )
{
  static int16 arg_DoubleSeg[5] = { 7, 3, 9, 3, 10};
  static int16 arg_TripleSeg[5] = { 14, 5, 13, 5, 14 };
  int16 *args;
  int16 heightDiffer;
  heightDiffer = (int16)(device_Height_Destinate - pesk_Current_Height);
  switch(device_Init_Info.peskType)
  {
    case PESK_DOUBLESEG_METRIC:
      args = arg_DoubleSeg;
      break;
        
    case PESK_TRIPLESEG_METRIC:
      args = arg_TripleSeg;
      break;
      
    case PESK_DOUBLESEG_IMPERIAL:
      args = arg_DoubleSeg;
      pesk_Move_Speed *= SCALE_IMPERIAL_TO_METRIC;
      heightDiffer *= SCALE_IMPERIAL_TO_METRIC;
      break;
      
    case PESK_TRIPLESEG_IMPERIAL:
      args = arg_TripleSeg;
      pesk_Move_Speed *= SCALE_IMPERIAL_TO_METRIC;
      heightDiffer *= SCALE_IMPERIAL_TO_METRIC;
      break;
      
    default:
      // should not get here
      break;
  }
  if( pesk_Move_Speed > args[0] )
  {
    pesk_Stop_Count = device_Count_Calculate( (int16)(device_Height_Destinate - pesk_Current_Height), device_Init_Info.peskType )
                    - pesk_Move_Speed / args[1] * args[2];
  }
  else if( pesk_Move_Speed < -args[0] )
  {
    pesk_Stop_Count = device_Count_Calculate( (int16)(device_Height_Destinate - pesk_Current_Height), device_Init_Info.peskType )
                    - pesk_Move_Speed / args[3] * args[4];
  }
}

#ifdef AUTOMOVE_FUNC
/*********************************************************************
 * @fn      postureReverse
 */
#pragma inline = forced
static uint8 postureReverse( uint8 dataToReverse )
{
  return ( (dataToReverse == USER_STATUS_STAND) ? USER_STATUS_SIT : USER_STATUS_STAND );
}

/*********************************************************************
 * @fn      autoMove_Reset
 *
 * @brief   Automatic movement data reset
 *
 * @param   data - pointer type data
 *          posture - uint8 type data get from peskData.userPosture
 *  
 * @return  result
 */
autoMove_t autoMove_Reset( autoMove_t data, uint8 posture )
{
  if( posture )
  {
    data.userNextStatus = postureReverse( posture );
    
    data.timeRemaining = 
  }
}
#endif