/******************** ************************* **********************************
 * �ļ���  ��main.c
 * ����    ��������         
 * ʵ��ƽ̨��
 * ��汾  ��ST3.5.0
 * ����    ��huang li
*******************************ϵͳ����ͷ�ļ�**************************************/
#include "stm32f10x.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "platform_config.h"
/***********************************************************************************/

/*******************************�û�ͷ�ļ�******************************************/
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

/******************** �������б����Լ��궨����������********************************/
/***********************************************************************************/

int main(void)
{
	uint16_t ret;
  /******ϵͳʱ������********/
  RCC_Configuration();

  /******ϵͳUSB����********/
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();

  /********GPIO����********/
  GPIO_Configuration();
  delay_x_ms(5);

  /****DAC����AVCC,VDDIO**/
  DAC_Configuration();
  delay_x_ms(5);

  /********SPI����*********/
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
  /********��λоƬ********/
	gt9p_io_init();
  delay_x_ms(5);
	  //GF11_FloatingGND_9P_RawdataRead_Flag=1;//���Դ�9P��ȡ����  
    //g_rawdata_read_flag = RAWDATA_READ_ENABLE;  
    protocol.cmd_curr = USB_CMD_GET_HBD_DATA;
		hbd_fp_raw.rx_ptr = 0;
    delay_x_ms(5);
   //register_rw_directly_GT9P_TO_GF11();
   while(1)
   		{
			 
       switch (protocol.cmd_curr)
        {
				/********��ȡָ��RAW_DATA********/	
        case USB_CMD_GET_RAW_DATA:            
             break;
				
				/************�̼�����************/
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
						
        /**********оƬ�Ĵ�����д*******/
        case USB_CMD_REG_RW:
					//	if(protocol.reg_op.addr == 0xb94c)  
								register_rw_directly();
					//	else  
					//	register_rw_directly_GT9P_TO_GF11();
            break;
				
        /*************Ӳ������**********/
        case USB_CMD_HW_CFG:
            usb_send_packet_repeat();
            break;
				
				/*************�汾��ȡ**********/
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
        /************PC�˹���ָ��*********/  
        case USB_CMD_ORDER:
           
            break;
				
        /************HBD���ݶ�ȡ**********/    
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
