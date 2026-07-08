/*
 * @File         : app_GO1.c
 * @Author       : JL HUANG
 * @Date         : 2023-11-26 17:37:27
 * @LastEditTime : 2024-03-26 19:51:42
 * @FilePath     : \MDK-ARMc:\Users\JLH\Desktop\balance2_1\Components\Devices\app_GO1.c
 * @Description  : GO1电机驱动层封装,输出轴逆时针为正力矩
 */

#include "app_GO1.h"
#include "chassis_task.h"
#include "usart.h"
#include "bsp_rs485.h"
#include "user_lib.h"

/************ 为GO1电机提供的CRC校验功能 ************/
/*
 * This mysterious table is just the CRC of each possible byte. It can be
 * computed using the standard bit-at-a-time methods. The polynomial can
 * be seen in entry 128, 0x8408. This corresponds to x^0 + x^5 + x^12.
 * Add the implicit x^16, and you have the standard CRC-CCITT.
 * https://github.com/torvalds/linux/blob/5bfc75d92efd494db37f5c4c173d3639d4772966/lib/crc-ccitt.c
 */
const uint16_t crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}

/**
 * @description  : recompute the CRC (CRC-CCITT variant) for the data
 * @param         {uint16_t} crc   :previous CRC value
 * @param         {uint8_t*} buffer:data pointer
 * @param         {size_t}   len   :number of bytes in the buffer
 * @return        {*}
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len)
{
	uint16_t tmp = crc;
	while (len--)
		tmp = crc_ccitt_byte(tmp, *buffer++);
	return tmp;
}
/************ 为GO1电机提供的CRC校验功能 ************/

/**
 * @description  : GO1电机打包函数
 * @param         {GO1_MotorSet_t*} go1_motor_set_t
 * @param         {uint8_t} id    :电机ID(1~4)
 * @param         {float} t_des   :期望浮点转子转矩
 * @param         {float} w_des   :期望浮点输出轴速度
 * @param         {float} pos_des :期望浮点输出轴位置
 * @param         {float} k_p     :位置系数
 * @param         {float} k_w     :速度系数
 * @return        {*}
 */
void Set_GO1_MotorDecode(GO1_MotorSet_t* go1_motor_set_t,uint8_t id,float t_des,float w_des,float pos_des,float k_p,float k_w)
{
  /* 转子减速比换算 */
  //t_des = t_des / GO1_Reduction_Ratio;
  w_des = w_des * GO1_Reduction_Ratio;
  pos_des = pos_des * GO1_Reduction_Ratio;
  
  go1_motor_set_t ->GO1_MotSetFloat.Torque_Des = t_des;
  go1_motor_set_t ->GO1_MotSetFloat.Speed_Des = w_des;
  go1_motor_set_t ->GO1_MotSetFloat.Position_Des = pos_des;
  go1_motor_set_t ->GO1_MotSetFloat.K_Position = k_p;
  go1_motor_set_t ->GO1_MotSetFloat.K_Speed = k_w;

  /* 包头 */
  go1_motor_set_t ->GO1_SetCmd.SetHead[0] = 0xFE;
  go1_motor_set_t ->GO1_SetCmd.SetHead[1] = 0xEE;
  
  /* 模式 */
  go1_motor_set_t ->GO1_SetCmd.GO1_MotorMode.ID = id;
  go1_motor_set_t ->GO1_SetCmd.GO1_MotorMode.Status = GO1_MODE_FOC;
  
  /* 限幅 */
  go1_motor_set_t ->GO1_MotSetFloat.Torque_Des   = Saturate_Float(go1_motor_set_t ->GO1_MotSetFloat.Torque_Des,  -127.99f,127.99f);     //期望转矩
  go1_motor_set_t ->GO1_MotSetFloat.Speed_Des    = Saturate_Float(go1_motor_set_t ->GO1_MotSetFloat.Speed_Des,   -804.00f,804.00f);     //期望速度
  go1_motor_set_t ->GO1_MotSetFloat.Position_Des = Saturate_Float(go1_motor_set_t ->GO1_MotSetFloat.Position_Des,-411774.0f,411774.0f); //期望位置
  go1_motor_set_t ->GO1_MotSetFloat.K_Position   = Saturate_Float(go1_motor_set_t ->GO1_MotSetFloat.K_Position,  0.0f,25.599f);         //期望刚度系数
  go1_motor_set_t ->GO1_MotSetFloat.K_Speed      = Saturate_Float(go1_motor_set_t ->GO1_MotSetFloat.K_Speed,     0.0f,25.599f);         //期望阻尼系数
  
  /* 浮点整型转换 */
  go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.T_Des   = go1_motor_set_t ->GO1_MotSetFloat.Torque_Des   *256;
  go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.W_Des   = go1_motor_set_t ->GO1_MotSetFloat.Speed_Des    /pi2*256;
  go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.Pos_Des = go1_motor_set_t ->GO1_MotSetFloat.Position_Des /pi2*32768;
  go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.K_Pos   = go1_motor_set_t ->GO1_MotSetFloat.K_Position   /25.6f*32768;
  go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.K_W     = go1_motor_set_t ->GO1_MotSetFloat.K_Speed      /25.6f*32768;
  
  /* CRC16校验 */
  go1_motor_set_t ->GO1_SetCmd.CRC_16 = crc_ccitt(0, (uint8_t *)&go1_motor_set_t->GO1_SetCmd, 15);
}

/**
 * @description  : GO1电机解码函数
 * @param         {GO1_MotorMeasure_t*} go1_motor_get_t
 * @return        {*}
 */
void Get_GO1_MotorDecode(GO1_MotorMeasure_t* go1_motor_get_t)
{
  /* 整型转浮点型 */
  go1_motor_get_t ->GO1_MotMeaFloat.Torque   = ((float)go1_motor_get_t ->GO1_MeaCmd.GO1_MotMeaInt.T)   /256;
  go1_motor_get_t ->GO1_MotMeaFloat.Speed    = ((float)go1_motor_get_t ->GO1_MeaCmd.GO1_MotMeaInt.W)   /256*pi2;
  go1_motor_get_t ->GO1_MotMeaFloat.Position = ((float)go1_motor_get_t ->GO1_MeaCmd.GO1_MotMeaInt.Pos) *pi2/32768;

  /* 输出轴减速比换算 */
 // go1_motor_get_t ->GO1_MotMeaFloat.Torque = go1_motor_get_t ->GO1_MotMeaFloat.Torque * GO1_Reduction_Ratio;
  go1_motor_get_t ->GO1_MotMeaFloat.Speed = go1_motor_get_t ->GO1_MotMeaFloat.Speed / GO1_Reduction_Ratio;
  go1_motor_get_t ->GO1_MotMeaFloat.Position = go1_motor_get_t ->GO1_MotMeaFloat.Position / GO1_Reduction_Ratio;

  /* 输出轴位置换算 */
  go1_motor_get_t ->OutPosition = go1_motor_get_t ->GO1_MotMeaFloat.Position - go1_motor_get_t ->OffSetPosition;
  // if( go1_motor_get_t -> GO1_MeaCmd.CRC_16 == crc_ccitt(0,(uint8_t*)&go1_motor_get_t -> GO1_MeaCmd,14) )
  // {
  //   go1_motor_get_t ->GO1_MotMeaFloat.Torque   = ((float)go1_motor_get_t ->GO1_MeaCmd.GO1_MotMeaInt.T)   /256;
  //   go1_motor_get_t ->GO1_MotMeaFloat.Speed    = ((float)go1_motor_get_t ->GO1_MeaCmd.GO1_MotMeaInt.W)   /256*pi2;
  //   go1_motor_get_t ->GO1_MotMeaFloat.Position = ((float)go1_motor_get_t ->GO1_MeaCmd.GO1_MotMeaInt.Pos) *pi2/32768;
  // }
  // else
  // {
  //   /* 接收数据CRC校验错误 */
  // }
  
}

/**
 * @description  : GO1电机发送函数
 * @param         {GO1_MotorSet_t*} go1_motor_set_t
 * @param         {uint8_t} id
 * @param         {float} t_des
 * @param         {float} w_des
 * @param         {float} pos_des
 * @param         {float} k_p
 * @param         {float} k_w
 * @return        {*}
 */
void Set_GO1_MotorInfo(GO1_MotorSet_t* go1_motor_set_t,uint8_t id,float t_des,float w_des,float pos_des,float k_p,float k_w)
{
  Set_GO1_MotorDecode(go1_motor_set_t, id, t_des, w_des, pos_des, k_p, k_w);
  //HAL_UART_Transmit_DMA(&huart1,(uint8_t*)go1_motor_set_t,sizeof(go1_motor_set_t ->GO1_SetCmd));
  HAL_UART_Transmit(&huart1,(uint8_t*)go1_motor_set_t,sizeof(go1_motor_set_t ->GO1_SetCmd),10);
}

/**
 * @description  : GO1电机接收函数
 * @param         {GO1_MotorMeasure_t*} go1_motor_get_t
 * @return        {*}
 */
void Get_GO1_MotorInfo(GO1_MotorMeasure_t* go1_motor_get_t)
{

  uint16_t rxlen = 0;
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t*)go1_motor_get_t,sizeof(go1_motor_get_t ->GO1_MeaCmd));
  HAL_UARTEx_ReceiveToIdle(&huart1,(uint8_t*)go1_motor_get_t,sizeof(go1_motor_get_t ->GO1_MeaCmd),&rxlen,10);
  uint8_t *rx_data = (uint8_t *)&go1_motor_get_t ->GO1_MeaCmd;
  if(rx_data[0] == 0xFD && rx_data[1] == 0xEE)
  {
    Get_GO1_MotorDecode(go1_motor_get_t);
  }

//  Get_GO1_MotorDecode(go1_motor_get_t);
}

/**
 * @description  : GO1电机数据更新
 * @param         {GO1_MotorSet_t*} go1_motor_set_t     :发送结构体
 * @param         {GO1_MotorMeasure_t*} go1_motor_get_t :接收结构体
 * @param         {uint8_t} id                          :电机ID(1~4)
 * @param         {float} t_des                         :期望转子转矩
 * @param         {float} w_des                         :期望输出轴速度
 * @param         {float} pos_des                       :期望输出轴位置
 * @param         {float} k_p                           :位置系数
 * @param         {float} k_w                           :速度系数
 * @return        {*}
 */
void GO1_Update(GO1_MotorSet_t* go1_motor_set_t,GO1_MotorMeasure_t* go1_motor_get_t,
                    uint8_t id,float t_des,float w_des,float pos_des,float k_p,float k_w)
{
  Set_GO1_MotorInfo(go1_motor_set_t, id, t_des, w_des, pos_des, k_p, k_w);
  Get_GO1_MotorInfo(go1_motor_get_t);
}

///**
// * @description  : GO1发送数据打包，位操作
// * @param         {GO1_MotorSet_t*} go1_motor_set_t
// * @param         {uint8_t*} tx_buf
// * @return        {*}
// */
//void Set_GO1_MotorDecode(GO1_MotorSet_t* go1_motor_set_t,uint8_t* tx_buf)
//{
//  tx_buf[0]  = go1_motor_set_t ->GO1_SetCmd.SetHead[0];
//  tx_buf[1]  = go1_motor_set_t ->GO1_SetCmd.SetHead[1];

//  tx_buf[2]  = go1_motor_set_t ->GO1_SetCmd.GO1_MotorMode.ID | (go1_motor_set_t ->GO1_SetCmd.GO1_MotorMode.Status << 4);

//  tx_buf[3]  = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.T_Des & 0xFF;
//  tx_buf[4]  = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.T_Des >> 8;
//  tx_buf[5]  = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.W_Des & 0xFF;
//  tx_buf[6]  = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.W_Des >> 8;
//  tx_buf[7]  = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.Pos_Des & 0xFF;
//  tx_buf[8]  = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.Pos_Des >> 8;
//  tx_buf[9]  = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.Pos_Des >> 16;
//  tx_buf[10] = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.Pos_Des >> 24;
//  tx_buf[11] = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.K_Pos & 0xFF;
//  tx_buf[12] = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.K_Pos >> 8;
//  tx_buf[13] = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.K_W & 0xFF;
//  tx_buf[14] = go1_motor_set_t ->GO1_SetCmd.GO1_MotSetInt.K_W >> 8;

//  tx_buf[15] = go1_motor_set_t ->GO1_SetCmd.CRC_16 & 0xFF;
//  tx_buf[16] = go1_motor_set_t ->GO1_SetCmd.CRC_16 >> 8;
//}
