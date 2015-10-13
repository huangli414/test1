/******************** (C) COPYRIGHT 2014 GOODIX ********************************
* File Name          : patch.c
* Author             : zhong hua
* Version            : 1.0
* Date               : 2014-12-16
* Description        : gf11 mcu patch code
*******************************************************************************/
#ifndef _PATCH_C_
#define _PATCH_C_

#include "common.h"         // all you need

//版本号
#define PATCH_MAJOR_VER     0x01        //主号
#define PATCH_MINOR_VER     0x00        //次号
#define PATCH_BUILD_VER     0x08        //内部号

//extern
void InitDev(void);
void InitConfigPar(void);
void WorkModeProc(void);
void ESDProtectionProc(void);
void ConfigParProc(u8 u8IsCheckEn);


/*********************************************************************************
Function : patch_main
Description : patch_main must be the first function in patch.c
Parameter :
Return  : NONE
**********************************************************************************/
void patch_main(void)
{
    InitConfigPar();
    ConfigParProc(1);
    InitDev();

    while (1)
    {
        ChickWatchDog();        /*<WatchDog  prd = 1s>*/
        ConfigParProc(0);       /*<config load>*/
        WorkModeProc();         /*<work mode init>*/
        TouchDetectProc();      /*<ring detext>*/
        FPScanProc();           /*<image scan>*/
        ESDProtectionProc();    /*<ESD protect>*/
    }
}

/*********************************************************************************
Function : WorkModeProc
Description : image/key/fingerflash/debug/sleep,which module need to turn
             on at each mode
Parameter :value of WORK_MODE which modified by host directly via spi
Return  : NONE
**********************************************************************************/
void WorkModeProc(void)
{
    u8 u8WorkMode = WORK_MODE;/*<update work mode>*/

    if (ctrl_flag.fMutexAnalogDev >= 1)
    {
        return;
    }

    ctrl_flag.fRingScanEn = 0;
    ctrl_flag.fImageScanEn = 0;
    //ctrl_flag.fKeyReportEn = 0;
    ctrl_flag.fDebugEn = 0;
    ctrl_flag.fLongPressEn = 0;


    if (0 == ctrl_flag.fKeyStaCurr) //无按钮时自动退出暂停
    {
        CLR_PAUSE_EN();
    }

    if (IS_PAUSE_EN())//主控要求暂停
    {
        ctrl_flag.fRingScanEn = 1;
        return;
    }


    switch (u8WorkMode)
    {
        case WORK_MODE_IMAGE:
        {
            ctrl_flag.fRingScanEn = 1;
            ctrl_flag.fImageScanEn = 1;
        }
        break;

        case WORK_MODE_TOUCH_KEY:
        {
            ctrl_flag.fRingScanEn = 1;
        }
        break;

        case WORK_MODE_FINGER_FLASH:
        {
            ctrl_flag.fRingScanEn = 1;
            ctrl_flag.fImageScanEn = 1;
            ctrl_flag.fLongPressEn = 1;
        }
        break;

        case WORK_MODE_DEBUG:
        {
            ctrl_flag.fRingScanEn = 1;
            ctrl_flag.fImageScanEn = 1;
            ctrl_flag.fDebugEn = 1;
        }
        break;

        default:
        {
            WORK_MODE = WORK_MODE_DEFAULT;/*<get an error work mode,change to default mode>*/
        }
        break;
    }

}

/*********************************************************************************
Function : GetCheckSumU16
Description :
Parameter :u16DataBuf[]---base address of memory,should be even
           u16Len--------number of u16's elements
Return  : sum = u16DataBuf[0] + u16DataBuf[1] + u16DataBuf[2]+...u16DataBuf[u16Len-1]
**********************************************************************************/
u16 GetCheckSumU16(u16 u16DataBuf[], u16 u16Len)
{
    u16 ii;
    u16 u16CheckSum;//
    u16CheckSum = 0xA5A5;

    for (ii = 0; ii < u16Len; ii++)
    {
        u16CheckSum = u16CheckSum + u16DataBuf[ii];
    }

    u16CheckSum = 0 - u16CheckSum;
    return u16CheckSum;
}

/*********************************************************************************
Function : ConfigParProc
Description : new config data from host,check it right or not,CONFIG_ERR requeset will
            be generate if checksum is error
Parameter :
Return  : NONE
**********************************************************************************/
void ConfigParProc(u8 u8IsCheckEn)
{
    if ((1 == g_ptrChipCfg->FreshFlag)
        || (1 == u8IsCheckEn))
    {
        //err
        while (0 != GetCheckSumU16((u16 *)g_ptrChipCfg, CONFIG_NUM - 1))
        {
            delay_x_ms(500);
            send_request_to_host(REQ_TYPE_CONFIG_ERR);
        }

        g_ptrChipCfg->FreshFlag = 0;
        //g_ptrChipCfg->CheckSum = GetCheckSumU16((u16 *)g_ptrChipCfg, CONFIG_NUM - 2);
    }
}

/*********************************************************************************
Function : ESDProtectionProc
Description : check whether the memory data is destroyed or not
Parameter :
Return  : NONE
**********************************************************************************/
void ESDProtectionProc(void)
{
    if (0 != GetCheckSumU16((u16 *)g_ptrChipCfg, CONFIG_NUM - 1))
    {
        g_u16EsdCnt++;
    }
    else
    {
        g_u16EsdCnt = 0;
    }

    if (g_u16EsdCnt > 50) //esd出错次数
    {
        g_u16EsdCnt = 0;
        send_request_to_host(REQ_TYPE_ESD_ERR);
    }

    if (IS_ESD_CHK_SET())
    {
        ESD_CHK_CLR();
    }
}

/*********************************************************************************
Function    :  Timer interrupt subroutine
Description :
Parameter   :
Return      :  NONE
**********************************************************************************/
void patch_TMR_isr(void)
{
    _ASM_("bic #0x0010, (10)(r1)"::);
    wDATA(misc_rg_int_clr) = 0x10;      // write 1 clear;
    //irq_flag |= 0x10;
    TimerIrqCallBack();
    //g_u8TimerCnt[0]++;
    //g_u8TimerCnt[1]++;

    //g_u8TestArray[5] = 0x55;
    //notify_host();
}

/*********************************************************************************
Function    :  ADC FIFO full interrupt subroutine
Description :
Parameter   :
Return      :  NONE
**********************************************************************************/
void patch_ADC_isr()
{
    _ASM_("bic #0x0010, (10)(r1)"::);
    wDATA(misc_rg_int_clr) = 0x20;      // write 1 clear;

    g_u8AdcIrqCnt++;

    WriteLeftReg(RW_LEFT__RG_INT_STA, 0x0000); //clr irq status reg
}
/*********************************************************************************
Function    :  InitConfigPar
Description :Initialization of global variables
Parameter   :
Return      :  NONE
**********************************************************************************/
void InitConfigPar(void)
{
    u8 ii;

    mem_set((u8 *)DATA_START_ADDR, 0x00, (u16)((u16)READ_SP - (u16)DATA_START_ADDR - 6));

    /*<PID>*/
    _pbxVersion[PID_0_INDEX] = (PID_1 << 8) | PID_0;//0x4647;//PID_0;FG
    _pbxVersion[PID_1_INDEX] = (PID_3 << 8) | PID_2;//0x3235;//PID_1;25
    _pbxVersion[PID_2_INDEX] = (PID_5 << 8) | PID_4;//0x5336;//PID_2;S6
    _pbxVersion[PID_3_INDEX] = (MAJOR_VER << 8) | SEPARATOR;//0x005F;//PID_3;00_
    _pbxVersion[PID_4_INDEX] = (BUILD_VER << 8) | MINOR_VER;//0x0000;//PID_4;00 00
    //_pbxVersion[PID_5_INDEX] = PID_5;
    /*<separtor'_'>*/
    //_pbxVersion[SEPARATOR_INDEX] = SEPARATOR;

    /*<VID>*/
    //_pbxVersion[MAJOR_INDEX] = MAJOR_VER;
    //_pbxVersion[MINOR_INDEX] = MINOR_VER;
    //_pbxVersion[BUILD_INDEX] = BUILD_VER;

    //ISR_GIO_INT_TO_MCU_PTR      = patch_GIO_isr;    // 0    ,,
    //ISR_INTIN_INT_TO_MCU_PTR    = patch_INT_isr;    // 1
    //ISR_CRC_INT_TO_MCU_PTR      = isr_nop;//patch_CRC_isr;  // 2
    //ISR_WDT_TMR_INT_TO_MCU_PTR  = patch_WDT_isr;  // 3
    ISR_TMR_INT_TO_MCU_PTR      = patch_TMR_isr;    // 4
    ISR_ADC_INT_TO_MCU_PTR      = patch_ADC_isr;    // 5    ,,
    //ISR_I2CM_INT_TO_MCU_PTR     = patch_IIC_isr;    // 6    ,,
    //ISR_KEY_INT_TO_MCU_PTR      = patch_KEY_isr;    // 7    ,,
    //ISR_EXC_8_PTR               = isr_nop;
    //ISR_EXC_PTR                 = isr_nop;          //

    ESD_CHK_SET();
    WORK_MODE = WORK_MODE_DEFAULT;
    g_ptrPkHead = (PK_HEAD_STR *) PK_HEAD_START_ADDR;
    g_ptrChipCfg = (CHIP_CONFIG_T *)CONFI_START_ADDR;

    InitTimer();

    iic_init();         // init iic before access to left side

    //读取OTP中主/副频/DAC校正值
    for (ii = 16; ii < 23; ii++)
    {
        WriteLeftReg(RW_LEFT_SYS__OTP_CTRL, 0x0020);       //PTM[5:4]
        WriteLeftReg(RW_LEFT_SYS__OTP_WD, (u16)(ii / 4));   //select bank
        WriteLeftReg(RW_LEFT_SYS__OTP_CTRL, 0x0021);       // bank switch ers=hi
        WriteLeftReg(RW_LEFT_SYS__OTP_CTRL, 0x0020);       // bank switch ers=low

        WriteLeftReg(RW_LEFT_SYS__OTP_WD, ((u16)ii % 4) << 8); //select addr in bank
        WriteLeftReg(RW_LEFT_SYS__OTP_CTRL, 0x0022);       //read ce=hi
        WriteLeftReg(RW_LEFT_SYS__OTP_CTRL, 0x0020);       //read ce=low

        g_u8OtpData[ii - 16] = (u8)ReadLeftReg(RO_LEFT_SYS__OTP_RD); //read data
    }

    EnableINT();
}

//不大于255行, {addr ,data_h,data_l}
const REG_CONFIG  g_GF12ConfigInfo[] =
{
    { WO_MISCTL__RG_INT_CLR, 0x0000},
    { RW_MISCTL__CPU_IRQ_CTR_REG, 0x0030},//0x0030},//adc_irq_en,timer_irq_en,watchdog en

    { RW_PIXEL__PIXEL_TCK2EN, 0x0002},
    //{ RW_PIXEL__PIXEL_TCODE, 0x00FF},
    { RW_PIXEL__PIXEL_T0, 0x0002},
    { RW_PIXEL__PIXEL_T1, 0x0004},
    { RW_PIXEL__PIXEL_TVDAC, 0x0018},

    { RW_PIXEL__ADC_CTRL_L, 0x000F},
    { RW_PIXEL__SYS_TMUX, 0x0000},
    { RW_PIXEL__SYS_TADC0, 0x000A},
    { RW_PIXEL__SYS_TADC1, 0x0010},
    { RW_PIXEL__SYS_TWAIT, 0x0000},
    { RW_PIXEL__SYS_TFWAE_L, 0x0000},
    { RW_PIXEL__SYS_TFWAE_H, 0x000a},

    { RW_PIXEL__DAC_CTRL_0_L, 0x0001},
    { RW_PIXEL__DAC_CTRL_0_H, 0x0001},
    { RW_PIXEL__DAC_CTRL_1_L, 0x0F0F},
    { RW_PIXEL__DAC_CTRL_2, 0x0F0F},
    { RW_PIXEL__DAC_CTRL_1_H, 0x0100},
    { RW_PIXEL__VCMI_SEL_L, 0x0000},
    { RW_PIXEL__VCMI_SEL_H, 0x0000},
    { RW_PIXEL__ANA_MISC_1, 0x0000},
    //{ RW_PIXEL__DAC_DATA_1, 0x0034},//0x002a},
    //{ RW_PIXEL__DAC_DATA_2, 0x0035},//0x002a},
    //{ RW_PIXEL__DAC_DATA_3, 0x0034},//0x002f},
    //{ RW_PIXEL__DAC_DATA_4, 0x003e},//0x0035},

    { RW_PIXEL__CLK_EN, 0x0000},
    { RW_PIXEL__PIXEL_CLK_GEN_L, 0x012f},//clk
    { RW_PIXEL__CLK_EN, 0x0100},

    { RW_PIXEL__DAC_CLK_GEN_L, 0x002F},// [7:0] dac_ck_div
    { RW_PIXEL__DAC_CLK_GEN_H, 0x0F00},// [27:24]pixel_ck_div_chp, [23:16]pixel_ck_txskew

    { RW_PIXEL__DAC_CHPEN_L, 0x0F0F},// [11:8] pixel_dac2_chp_en, [3:0] pixel_dac1_chp_en
    { RW_PIXEL__DAC_CHPEN_H, 0x0F0F},// [27:24] pixel_dac4_chp_en, [19:16] pixel_dac3_chp_en

    { RW_PIXEL__INT_CTRL_L, 0x0507},
    { RW_PIXEL__PIXEL_REPEAT, 0x000f},

    //tmr time = 50 * 2^tmr_cks * tmr_prd * sysclk_period
    { RW_MISCTL__TMR0, 0x0403},// tmr_cks=4,, tmr_en=rtc_en=1; 8*125 = 10ms
    { RW_MISCTL__TMR1, 0x007D}, //tmr_prd = 125

    //watchdog = 50 * 2^tmr_cks * tmr_prd * sysclk_period
    { RW_MISCTL__TMR0_WTD, 0x0402},// tmr_cks=4,, tmr_en=rtc_en=1; 8*12500 = 1000ms,改此寄存器值需要更改ChickWatchDog()函数
    { RW_MISCTL__TMR1_WTD, 0x30D4}, //tmr_prd = 12500
    { WO_MISCTL__RG_WTD_RST, 0x0001}, //wtd reset en

    //{ RW_PIXEL__INT_SCAN_L, 0xFFFF},
    //{ RW_PIXEL__INT_SCAN_H, 0x0000},

    //{ WO_PIXEL__SCAN, 0x0001},
    //{ WO_PIXEL__SCAN, 0x0100},

};

#define CONFI_INFO_SIZE (sizeof(g_GF12ConfigInfo)/sizeof(REG_CONFIG))

//不大于255行, {addr ,data_h,data_l}
const REG_CONFIG  g_GF12ConfigLeftInfo[] =
{
    { RW_LEFT_SYS__POWER_CTR_REG, 0x1500},// 3d10, [0]rg_bg_chop_en, 1510:AVDD25/DVDD25=2.44v
    //{ RW_LEFT__RG_ADC_RST_FIFO, 0x000F},
    //{ RW_LEFT__RG_ADC_RST_FIFO, 0x0000},
    { RW_LEFT_SYS__RST_REG, 0x0782},
    { RW_LEFT_SYS__RST_REG, 0x0780},

    //{ RW_LEFT_SYS__INT_EN, 0x0001},
    { RW_LEFT__RG_ADC_CTR, 0x0108}, //0x0000},// [13:8] adc_samp_sel, [3] adcclk_bypass, [1:0] adcclk_sel
    { RW_LEFT__RG_ADC_CTR2, 0x0000},

    { RW_LEFT__RG_COL_SCAN_CFG, 0x0000},

    //{ RW_LEFT__RG_LAST_ROW_ID1, 0x0047},
    //{ RW_LEFT__RG_LAST_ROW_ID2, 0x0047},
    //{ RW_LEFT__RG_LAST_ROW_ID3, 0x0047},
    //{ RW_LEFT__RG_LAST_ROW_ID4, 0x0047},
    { RW_LEFT__RG_AES_CTR, 0x0002},
    { RW_LEFT__RG_INT_STA, 0x0000},
};

#define CONFI_LEFT_INFO_SIZE (sizeof(g_GF12ConfigLeftInfo)/sizeof(REG_CONFIG))

/*********************************************************************************
Function    :  InitDev
Description :Peripheral initialize,such as timer,iic
Parameter   :
Return      :  NONE
**********************************************************************************/
void InitDev(void)
{

    u8 ii;

    for (ii = 0; ii < CONFI_INFO_SIZE; ii++)
    {
        wDATA(g_GF12ConfigInfo[ii].u16Addr) = g_GF12ConfigInfo[ii].u16Value;
    }


    for (ii = 0; ii < CONFI_LEFT_INFO_SIZE; ii++)
    {
        WriteLeftReg(g_GF12ConfigLeftInfo[ii].u16Addr, g_GF12ConfigLeftInfo[ii].u16Value);
    }

    //使用OTP中的主频，副频,DAC校正值
    WriteLeftReg(RW_LEFT_SYS__OSC_REG, (u16)(g_u8OtpData[0]));
    WriteLeftReg(RW_LEFT_SYS__WDT_REG, (u16)((0x0E << 8) | g_u8OtpData[1]));
    _wRW_PIXEL__DAC_DATA_1 = g_u8OtpData[2];
    _wRW_PIXEL__DAC_DATA_2 = g_u8OtpData[3];
    _wRW_PIXEL__DAC_DATA_3 = g_u8OtpData[4];
    _wRW_PIXEL__DAC_DATA_4 = g_u8OtpData[5];

    //使用配置参数，仅FP相关
    _wRW_PIXEL__PIXEL_TCODE = g_ptrChipCfg->FPTcode;
}

/*********************************************************************************
Function    :  iic_init
Description :iic initialize
Parameter   :
Return      :  NONE
**********************************************************************************/
void iic_init(void)
{
    // iic init.
    wDATA(IIC_MST_SCL_RATE_REG) = 0x0006;   // 10/(4+1)/2 = 7xx kHz ,, ,,,,,
    wDATA(IIC_MST_MASTER_WAIT_REG) = 0x0000; // no master wait
    //wDATA(misc_cpu_irq_ctr_reg)|=0x40;        // enable iic irq
}
/*********************************************************************************
Function    :  ReadLeftReg
Description :  ReadLeftReg
Parameter   :
Return      :  NONE
**********************************************************************************/
u16 ReadLeftReg(u16 u16Addr)
{
    //wDATA(IIC_MST_CMD_REG) = 0x5D00;    // slave address=0x5D cmd read
    _wRW_MASTER_I2C__MST_CMD = 0x5D00;

    //wDATA(IIC_MST_DATA_BYTE_REG) = 1;//len - 1; // write len
    _wRW_MASTER_I2C_MST_DATA_BYTE = 1;
    //wDATA(IIC_SLAVE_ADDR_REG) = u16Addr;   // target address
    _wRW_MASTER_I2C_SLAVE_ADDR = u16Addr;
    // polling
    //wDATA(IIC_MST_TRIGGER_REG) = 0x01;  // trigger iic
    _wWO_MASTER_I2C__MST_TRIGGER = 0x01;

    //while ((wDATA(IIC_MST_STATUS_REG) & 0x01));
    while ((_wRO_MASTER_I2C__MST_STATUS) & 0x01);

    return _wRW_MASTER_I2C__MST_DATA_0_L;
}

/*********************************************************************************
Function    :  WriteLastRowId
Description :
Parameter   :
Return      :  NONE
**********************************************************************************/
void WriteLastRowId(u8 u8LastRowId)
{
    u8 ii;

    for (ii = 0; ii < 8; ii += 2)
    {
        WriteLeftReg(RW_LEFT__RG_LAST_ROW_ID1 + ii, (u16)u8LastRowId);
    }
}

/*********************************************************************************
Function    :  WriteLeftReg
Description :
Parameter   :
Return      :  NONE
**********************************************************************************/
void WriteLeftReg(u16 u16Addr, u16 u16Value)
{
    //wDATA(IIC_MST_CMD_REG) = 0x5D00;    // slave address=0x5D cmd read
    _wRW_MASTER_I2C__MST_CMD = 0x5D01;

    //wDATA(IIC_MST_DATA_BYTE_REG) = 1;//len - 1; // write len
    _wRW_MASTER_I2C_MST_DATA_BYTE = 1;
    //wDATA(IIC_SLAVE_ADDR_REG) = u16Addr;   // target address
    _wRW_MASTER_I2C_SLAVE_ADDR = u16Addr;

    _wRW_MASTER_I2C__MST_DATA_0_L = u16Value;
    // polling
    //wDATA(IIC_MST_TRIGGER_REG) = 0x01;  // trigger iic
    _wWO_MASTER_I2C__MST_TRIGGER = 0x01;

    //while ((wDATA(IIC_MST_STATUS_REG) & 0x01));
    while ((_wRO_MASTER_I2C__MST_STATUS) & 0x01);
}

#endif
//edit test
/************************END OF FILE*******************************************/

