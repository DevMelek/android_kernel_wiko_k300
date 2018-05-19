#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/mt_gpio.h>
    #include <string.h>
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
#else
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define AUXADC_LCM_VOLTAGE_CHANNEL     12
#define AUXADC_ADC_FDD_RF_PARAMS_DYNAMIC_CUSTOM_CH_CHANNEL     1
#define MAX_VOLTAGE (1000)     // zhoulidong  add for lcm detect

// the ID need to check
#define LCM_ID_ICN9706                                    0x70
#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1440)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#if defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#endif

#define LCM_DBG(fmt, arg...) \
    LCM_PRINT ("[icn9706_hlt] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;
#if 0
#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#else
extern int DispTEpin_Enable(void);
extern int DispTEpin_Disable(void);
extern int get_lcm_id_status(void);

#define SET_RESET_PIN(v)    \
    if(v)                                           \
        DispTEpin_Enable(); \
    else                                           \
        DispTEpin_Disable();
#endif

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0

#define REGFLAG_END_OF_TABLE                                	  0xFD   // END OF REGISTERS MARKER
#define REGFLAG_DELAY                                           0xFC

struct LCM_setting_table
{
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

static struct LCM_setting_table  lcm_deep_sleep_mode_in_setting_v2[] = {

    {0xF0,2,{0x5A,0x5A}},
    {0xF1,2,{0xA5,0xA5}},

    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
        
    {0xB7,17,{0x01,0x01,0x09,0x11,0x0D,0x15,0x19,0x0D,0x21,0x1D,0x00,0x00,0x20,0x00,0x02,0xFF,0x3C}},      
        
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
};


static struct LCM_setting_table lcm_initialization_setting_v2[] = 
{
    {0xF0,  2 ,{0x5A,0x5A}},
    {0xF1,  2 ,{0xA5,0xA5}},
    //{0xB6,  2 ,{0x13,0x13}},
    {0xB4, 20 ,{0x0A,0x08,0x12,0x10,0x0E,0x0C,0x00,0x00,0x00,0x03,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x04,0x06}},
    {0xB3, 20 ,{0x0B,0x09,0x13,0x11,0x0F,0x0D,0x00,0x00,0x00,0x03,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x05,0x07}},
    {0xB0, 12 ,{0x54,0x32,0x23,0x45,0x44,0x44,0x44,0x44,0x60,0x01,0x60,0x01}},
    {0xB1,  8 ,{0x33,0x84,0x02,0x83,0x57,0x01,0x57,0x01}},
    {0xB2,  1 ,{0x73}},
    {0xBD, 10 ,{0x4E,0x0E,0x50,0x50,0x20,0x1e,0x00,0x14,0x42,0x03}},
    {0xB7, 17 ,{0x01,0x01,0x09,0x11,0x0D,0x55,0x19,0x19,0x21,0x1D,0x00,0x00,0x00,0x00,0x02,0xFF,0x3C}},
    {0xB8,  5 ,{0x23,0x01,0x30,0x34,0x13}},
    {0xB9,  4 ,{0xA0,0x22,0x00,0x44}},
    {0xBA,  2 ,{0x12,0x63}},
    {0xC1,  6 ,{0x16,0x16,0x04,0x0C,0x10,0x04}},
    {0xC2,  2 ,{0x12,0x68}},
    {0xC3,  3 ,{0x22,0x31,0x04}},
    {0xC7,  5 ,{0x05,0x23,0x6B,0x49,0x00}},
    {0xC5,  1 ,{0x00}},
    {0xD0,  3 ,{0x37,0xFF,0xFF}},
    {0xD2,  4 ,{0x62,0x0B,0x08,0x88}},
    {0xD3, 11 ,{0x01,0x00,0x00,0x01,0x01,0x37,0x25,0x38,0x31,0x06,0x07}},
    {0xD4,  6 ,{0x00,0x01,0x00,0x0E,0x04,0x60}},
    {0xC8, 38 ,{0x7C,0x57,0x43,0x33,0x2B,0x1C,0x20,0x0C,0x29,0x2D,0x30,0x53,0x43,0x4D,0x44,0x45,0x3E,0x35,0x2E,0x7C,0x57,0x43,0x33,0x2B,0x1C,0x20,0x0C,0x29,0x2D,0x30,0x53,0x43,0x4D,0x44,0x45,0x3E,0x35,0x2E}},
    {0xC6,  8 ,{0x00,0x00,0xFF,0x00,0x00,0xFF,0x00,0x00}},
    {0xF4,  2 ,{0x08,0x77}},
    {0x35,  1 ,{0x00}},
    
    {0x11,1,{0x00 }}, //sleep out
    {REGFLAG_DELAY,120,{}}, 
    {0xB8,  5 ,{0x23,0x01,0x30,0x34,0x63}},
    {0x29,1,{0x00 }}, //display on
    //{REGFLAG_DELAY,20,{}},
    {0xF1,  2 ,{0x5A,0x5A}},
    {0xF0,  2 ,{0xA5,0xA5}},
        {REGFLAG_END_OF_TABLE, 0x00, {}}
};


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;
        switch (cmd)
        {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE :
                break;
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }

}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
        
		params->dsi.noncont_clock= 1;
		params->dsi.noncont_clock_period= 2;
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_THREE_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active = 6;
		params->dsi.vertical_backporch = 20;
		params->dsi.vertical_frontporch	= 25;
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active = 10;
		params->dsi.horizontal_backporch = 37;
		params->dsi.horizontal_frontporch = 37;
		params->dsi.horizontal_active_pixel = FRAME_WIDTH;

		// Bit rate calculation
		//params->dsi.PLL_CLOCK=205;
		params->dsi.PLL_CLOCK=303;//294; 
		params->dsi.ssc_disable=1;
		
		//yixuhong 20150511 add esd check function
#ifndef BUILD_LK	
	params->dsi.esd_check_enable = 1; 
	params->dsi.customization_esd_check_enable = 1;//0:te esd check 1:read register
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	
	//params->dsi.lcm_esd_check_table[1].cmd = 0x0D;
	//params->dsi.lcm_esd_check_table[1].count = 1;
	//params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
#endif /*BUILD_LK*/

}

static void lcm_init(void)
{
	LCM_DBG(); 
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(10); 
	SET_RESET_PIN(1);
	MDELAY(20);
	push_table(lcm_initialization_setting_v2, sizeof(lcm_initialization_setting_v2) / sizeof(struct LCM_setting_table), 1);
	LCM_DBG("jacy debug,lcm init end \n");
}

static void lcm_suspend(void)
{	
	LCM_DBG();
    push_table(lcm_deep_sleep_mode_in_setting_v2, sizeof(lcm_deep_sleep_mode_in_setting_v2) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(1);
    MDELAY(20); // 1ms
    SET_RESET_PIN(0);
    MDELAY(20); // 1ms

    SET_RESET_PIN(1);
    MDELAY(20);
}

static void lcm_resume(void)
{
	LCM_DBG();
    SET_RESET_PIN(1);
    MDELAY(20); // 1ms
	SET_RESET_PIN(0);
	MDELAY(10); 
	SET_RESET_PIN(1);
	MDELAY(20); 
	push_table(lcm_initialization_setting_v2, sizeof(lcm_initialization_setting_v2) / sizeof(struct LCM_setting_table), 1);
}      

static unsigned int lcm_compare_id(void)
{
     int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;
	
	int array[4];
	char buffer[4]={0,0,0,0};
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0] = 0x00023700;// read id return 2 bytes,version and id
	dsi_set_cmdq(array, 1, 1);
	MDELAY(1);	
	read_reg_v2(0x0C, buffer, 2);
	id = buffer[0];
	
#ifdef BUILD_LK
	LCM_DBG(" icn9706 hlt LK read id=0x%x, id1=0x%x, id2=0x%x",id, id_high,id_low);
#else
        printk(" icn9706 hlt LK read id=0x%x, id1=0x%x, id2=0x%x",id, id_high,id_low);
#endif 

#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    {
   #ifdef BUILD_LK 
    LCM_DBG("read adc error");
   #else 
   printk("read adc error");
   #endif
    //return 0;

    }
    lcm_vol = data[0]*1000+data[1]*10;
#endif
    
#ifdef BUILD_LK
   	LCM_DBG("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
#else
    printk("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
#endif

/*********************************************************
* LINE<JIRA_ID><DATE20171222><BUG_INFO>zenghaihui
* zgd: ILI881C   ID=1.4V, read adc = 1370
* hlt:  ICN9706  ID=0V, read adc = 0
*
* if have more vendor with same ic, need to check adc
* now using LCM ID only
**********************************************************/

    //if ((lcm_vol > MAX_VOLTAGE )&&(LCM_ID_ICN9706 == id))
    if (LCM_ID_ICN9706 == id)
    {
    return 1;
    }
    return 0;
}

LCM_DRIVER icn9706_k400_hdplus_dsi_vdo_hlt_lcm_drv = 
{
	.name		= "icn9706_hdplus_dsi_vdo_hlt",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};
