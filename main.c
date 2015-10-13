/******************** ************************* **********************************
 * 文件名  ：main.c
 * 描述    ：主程序         
 * 实验平台：
 * 库版本  ：ST3.5.0
 * 作者    ：huang li
*******************************系统基本头文件**************************************/
#include "stm32f10x.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "platform_config.h"
/***********************************************************************************/

/*******************************用户头文件******************************************/
#include "stm32_spi.h"
#include "stm32_dac.h"
#include "stm32_gpio.h"
#include "protocol.h"
#include "global_var.h"
#include "comm_usb.h"

#include "GT9P_Application.h"
#include "GT9P_FW_Update.h"
/***********************************************************************************/
//asdfasdf

/******************** 主函数中变量以及宏定义声明区域********************************/
/***********************************************************************************/

int main(void)
{
	uint16_t ret;
  /******系统时钟设置********/
  RCC_Configuration();

  /******系统USB设置********/
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();

  /********GPIO设置********/
  GPIO_Configuration();
  delay_x_ms(5);

  /****DAC设置AVCC,VDDIO**/
  DAC_Configuration();
  delay_x_ms(5);

  /********SPI设置*********/
  SPI_Configuration();
  DMA_Configuration();
  delay_x_ms(5);
	/***** EXTI setting *****/
  EXTI_INT_Config();
  /***** NVIC setting *****/
  NVIC_Config();
	SPI_Configuration();
  DMA_Configuration();
	delay_x_ms(5);
  /********复位芯片********/
	gt9p_io_init();
  delay_x_ms(5);
	  //GF11_FloatingGND_9P_RawdataRead_Flag=1;//可以从9P读取数据  
    //g_rawdata_read_flag = RAWDATA_READ_ENABLE;  
    protocol.cmd_curr = USB_CMD_GET_HBD_DATA;
		hbd_fp_raw.rx_ptr = 0;
    delay_x_ms(5);
   //register_rw_directly_GT9P_TO_GF11();
   while(1)
   		{
			 
       switch (protocol.cmd_curr)
        {
				/********获取指纹RAW_DATA********/	
        case USB_CMD_GET_RAW_DATA:            
             break;
				
				/************固件更新************/
        case USB_CMD_FW_UPDATE:
            if (protocol.rx_ok)
            {
                EXIT_INT_Disable();
                ret = gt900_update(protocol.spec, protocol.rx_ptr, protocol.total_len);
                protocol.rx_ok = 0;
                usb_set_ack(ret);
                EXIT_INT_Enable();
            }
            usb_send_packet_repeat();
            break;
						
        /**********芯片寄存器读写*******/
        case USB_CMD_REG_RW:
					//	if(protocol.reg_op.addr == 0xb94c)  
								register_rw_directly();
					//	else  
					//	register_rw_directly_GT9P_TO_GF11();
            break;
				
        /*************硬件配置**********/
        case USB_CMD_HW_CFG:
            usb_send_packet_repeat();
            break;
				
				/*************版本获取**********/
        case USB_CMD_EVK_VERSION:
            if (protocol.spec	== 0)
                get_evk_version();
            else
                get_chip_version_GT9P();
            usb_send_packet_repeat();
            break;

        case USB_CMD_FLASH_RW:
            flash_rw_directly();
            break;
        /************PC端功能指令*********/  
        case USB_CMD_ORDER:
           
            break;
				
        /************HBD数据读取**********/    
        case USB_CMD_GET_HBD_DATA:
					
            GF11_HBD_Data_Read();
				 
				  //if((hbd_fp_raw.rx_ptr-hbd_protocol.tx_cnt)>= 56)
         		USB_send_HbdData();
            break;            
        default:
            break;
        }    
				
				
		  }
}
//eidt test third time
//error
