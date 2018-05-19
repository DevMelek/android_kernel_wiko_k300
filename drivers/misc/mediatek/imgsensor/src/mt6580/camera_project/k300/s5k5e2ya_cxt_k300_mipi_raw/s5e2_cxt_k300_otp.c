#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k5e2ya_cxt_k300_mipi_raw_Sensor.h"

//#define LSC_PARAM_QTY 240
#define SENSOR_FAIL   -1
#define SENSOR_SUCCESS 0
struct otp_info_t {
	uint16_t flag;
	uint16_t module_id;
	uint16_t lens_id;
	//uint16_t year;
	//uint16_t month;
	//uint16_t day;
	//uint16_t rg_ratio_current;
	//uint16_t bg_ratio_current;
	//uint16_t gg_ratio_current;
	uint16_t rg_ratio_typical;
	uint16_t bg_ratio_typical;
	uint16_t r_current;
	uint16_t g_current;
	uint16_t b_current;
	uint16_t r_typical;
	uint16_t g_typical;
	uint16_t b_typical;
	//uint16_t lsc_param[LSC_PARAM_QTY];
}Cxt_S5K5E2_OTP;

#define OTP_AUTO_LOAD_LSC_s5k5e2ya_cxt_k300

#define RG_TYPICAL_s5k5e2ya_cxt_k300    0x0331
#define BG_TYPICAL_s5k5e2ya_cxt_k300    0x028A

//#define R_TYPICAL_s5k5e2ya_cxt_k300		0x0040
//#define G_TYPICAL_s5k5e2ya_cxt_k300		0x0080
//#define B_TYPICAL_s5k5e2ya_cxt_k300		0x00A0

//#define S5K5E2YA_CXT_K300M_OTP_RG_MAX  0x25D  //0.78 x 1024  = 799
//#define S5K5E2YA_CXT_K300M_OTP_RG_MIN  0x207  //0.57 x 1024  = 584
//#define S5K5E2YA_CXT_K300M_OTP_BG_MAX  0x1F7  //0.68 x 1024  = 697
//#define S5K5E2YA_CXT_K300M_OTP_BG_MIN  0x1B4  //0.48 x 1024  = 492
#define LOG_TAG  "S5K5E2YA_CXT_K300:"
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define SENSOR_PRINT(fmt,arg...)  printk(LOG_TAG "%s: " fmt "\n", __FUNCTION__ ,##arg)
#else
#define SENSOR_PRINT(fmt,arg...)
#endif
extern  void S5K5E2_CXT_K300_write_cmos_sensor(u16 addr, u32 para);
extern  kal_uint16 S5K5E2_CXT_K300_read_cmos_sensor(u32 addr);

uint32_t s5k5e2ya_cxt_k300_read_otp_info(struct otp_info_t *otp_info)
{
	uint32_t rtn = SENSOR_SUCCESS;
	//uint16_t temp1 = 0,temp2 = 0;
	//uint16_t flag = 0;
	
	//uint16_t temp1,temp2,temp3;
	//uint16_t infoMidTemp1,infoMidTemp2,infoMidTemp3,infoLensIDTemp1,infoLensIDTemp2,infoLensIDTemp3;
	//uint16_t wbFlagTemp1,wbFlagTemp2,wbFlagTemp3,wbRGMsb1,wbRGMsb2,wbRGMsb3,wbRGLsb1,wbRGLsb2,wbRGLsb3;
	//uint16_t wbBGMsb1,wbBGMsb2,wbBGMsb3,wbBGLsb1,wbBGLsb2,wbBGLsb3;
	//struct otp_info_t *otp_info=(struct otp_info_t *)param_ptr;
	otp_info->rg_ratio_typical=RG_TYPICAL_s5k5e2ya_cxt_k300;
	otp_info->bg_ratio_typical=BG_TYPICAL_s5k5e2ya_cxt_k300;
	//otp_info->r_typical=R_TYPICAL_s5k5e2ya_cxt_k300;
	//otp_info->g_typical=G_TYPICAL_s5k5e2ya_cxt_k300;
	//otp_info->b_typical=B_TYPICAL_s5k5e2ya_cxt_k300;

	#if 0
	/*TODO*/
	//read lsc data. check moudule for debug
	{
		uint32_t i,check_sum,page;
		uint32_t lsc_data_count = 0;
		uint16_t data_lsc[360];
		//Check lsc value is empty or valid
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04); //make initial state
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a02, 0x05); //page set
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x01); //otp enable read
		mdelay(5);

		flag = S5K5E2_CXT_K300_read_cmos_sensor(0x0A04) | S5K5E2_CXT_K300_read_cmos_sensor(0x0A05) | S5K5E2_CXT_K300_read_cmos_sensor(0x0A06);

		check_sum =  S5K5E2_CXT_K300_read_cmos_sensor(0x0A43);

		if(flag == 0x01)
		{
			for(page = 0x09; page < 0x0f; page++)
			{
				S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04);
				S5K5E2_CXT_K300_write_cmos_sensor(0x0a02, page);
				S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x01);
				for(i = 0x0A04; i <= 0x0A43; i++)
				{
					if((page == 0x0E) && i > 0x0A2B)
					{
						break;
					}
					else{
						data_lsc[i] = S5K5E2_CXT_K300_read_cmos_sensor(i);
						lsc_data_count += data_lsc[i];

					}
				}
			}

		}
		else 
		{
			SENSOR_PRINT("otp lsc is empty or invalid\n");
		}

		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04); //make initial state
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x00); //disable enable read	
		
		if( check_sum == (lsc_data_count%255 + 1)) {
			SENSOR_PRINT("OTP LSC data is ok!\n");
		}			
	}
	#endif
	
	
	#if 0
   	//Read module info	
	S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04); //make initial state
	S5K5E2_CXT_K300_write_cmos_sensor(0x0a02, 0x05); //page5 ,module info part in page 5
	S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x01); //otp enable read
	mdelay(5);
	
	flag = S5K5E2_CXT_K300_read_cmos_sensor(0x0A07)| S5K5E2_CXT_K300_read_cmos_sensor(0x0A08) | S5K5E2_CXT_K300_read_cmos_sensor(0x0A09);

	printk("Module info module awb flag = %d \n",flag);

	if(flag == 0x01)  // Module info group1
	{
		otp_info->module_id = S5K5E2_CXT_K300_read_cmos_sensor(0x0A0F);
		otp_info->lens_id = S5K5E2_CXT_K300_read_cmos_sensor(0x0A10);
		//otp_info->year = S5K5E2_CXT_K300_read_cmos_sensor(0x0A09);
		//otp_info->month = S5K5E2_CXT_K300_read_cmos_sensor(0x0A0A);
		//otp_info->day = S5K5E2_CXT_K300_read_cmos_sensor(0x0A0B);
		otp_info->rg_ratio_current = ((S5K5E2_CXT_K300_read_cmos_sensor(0x0A14) << 8) & 0xFF00) | (S5K5E2_CXT_K300_read_cmos_sensor(0x0A15) & 0xFF);
		otp_info->bg_ratio_current = ((S5K5E2_CXT_K300_read_cmos_sensor(0x0A16) << 8) & 0xFF00) | (S5K5E2_CXT_K300_read_cmos_sensor(0x0A17) & 0xFF);
		otp_info->gg_ratio_current = ((S5K5E2_CXT_K300_read_cmos_sensor(0x0A18) << 8) & 0xFF00) | (S5K5E2_CXT_K300_read_cmos_sensor(0x0A19) & 0xFF);
		otp_info->flag = 1;
		//SENSOR_PRINT("Module info module awb group1\n");
	}
	else if(flag == 0x07)  //Module info group2
	{
		otp_info->module_id = S5K5E2_CXT_K300_read_cmos_sensor(0x0A1F);
		otp_info->lens_id = S5K5E2_CXT_K300_read_cmos_sensor(0x0A20);
		//otp_info->year = S5K5E2_CXT_K300_read_cmos_sensor(0x0A24);
		//otp_info->month = S5K5E2_CXT_K300_read_cmos_sensor(0x0A25);
		//otp_info->day = S5K5E2_CXT_K300_read_cmos_sensor(0x0A26);
		otp_info->rg_ratio_current = ((S5K5E2_CXT_K300_read_cmos_sensor(0x0A24) << 8) & 0xFF00) | (S5K5E2_CXT_K300_read_cmos_sensor(0x0A25) & 0xFF);
		otp_info->bg_ratio_current = ((S5K5E2_CXT_K300_read_cmos_sensor(0x0A26) << 8) & 0xFF00) | (S5K5E2_CXT_K300_read_cmos_sensor(0x0A27) & 0xFF);
		otp_info->gg_ratio_current = ((S5K5E2_CXT_K300_read_cmos_sensor(0x0A28) << 8) & 0xFF00) | (S5K5E2_CXT_K300_read_cmos_sensor(0x0A29) & 0xFF);
		otp_info->flag = 2;
		//SENSOR_PRINT("Module info module awb group2\n");
	}
	else
	{
		otp_info->module_id = 0x00;
		otp_info->lens_id = 0x00;
		otp_info->rg_ratio_current = 0x00;
		otp_info->bg_ratio_current = 0x00;
		otp_info->gg_ratio_current = 0x00;
		otp_info->flag = 0;
		//SENSOR_PRINT("otp data is empty or invalid\n");
	}
	S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04);
	S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x00);
	mdelay(5);
	#endif
	
	uint16_t flag[3] = {0};
	int i = 0;
	
	//Read module info	
	for(i = 0; i < 3; i++){
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04); //make initial state
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a02, i + 2); //page 5,module info part in page 5
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x01); //otp enable read
		mdelay(5);
		flag[i] = S5K5E2_CXT_K300_read_cmos_sensor(0x0A04);
		printk("wubingzhong: flag[%d]= %d\n",i,flag[i]);
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04);
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x00);
		mdelay(5);
	}
	//flag = S5K5E2_CXT_K300_read_cmos_sensor(0x0A07)| S5K5E2_CXT_K300_read_cmos_sensor(0x0A08) | S5K5E2_CXT_K300_read_cmos_sensor(0x0A09);
	
	if(((flag[0] & 0xff) >> 7)== 0x00)
	{  //Module info group2
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04); //make initial state
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a02, 0x02); //page 5,module info part in page 5
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x01); //otp enable read
		otp_info->module_id = flag[0] & 0x7f;
		otp_info->lens_id = S5K5E2_CXT_K300_read_cmos_sensor(0x0A05);
		otp_info->r_current = S5K5E2_CXT_K300_read_cmos_sensor(0x0A06);
		otp_info->g_current = S5K5E2_CXT_K300_read_cmos_sensor(0x0A07);
		otp_info->b_current = S5K5E2_CXT_K300_read_cmos_sensor(0x0A08);
		otp_info->r_typical = S5K5E2_CXT_K300_read_cmos_sensor(0x0A09);
		otp_info->b_typical = S5K5E2_CXT_K300_read_cmos_sensor(0x0A0A);
		otp_info->g_typical = S5K5E2_CXT_K300_read_cmos_sensor(0x0A0B);
		otp_info->flag = 2;
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04);
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x00);
	}
	else if(((flag[1] & 0xff) >> 7)== 0x00)
	{ //Module info group3
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04); //make initial state
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a02, 0x03); //page 5,module info part in page 5
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x01); //otp enable read
		otp_info->module_id = flag[1] & 0x7f;
		otp_info->lens_id = S5K5E2_CXT_K300_read_cmos_sensor(0x0A05);
		otp_info->r_current = S5K5E2_CXT_K300_read_cmos_sensor(0x0A06);
		otp_info->g_current = S5K5E2_CXT_K300_read_cmos_sensor(0x0A07);
		otp_info->b_current = S5K5E2_CXT_K300_read_cmos_sensor(0x0A08);
		otp_info->r_typical = S5K5E2_CXT_K300_read_cmos_sensor(0x0A09);
		otp_info->b_typical = S5K5E2_CXT_K300_read_cmos_sensor(0x0A0A);
		otp_info->g_typical = S5K5E2_CXT_K300_read_cmos_sensor(0x0A0B);
		otp_info->flag = 3;
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04);
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x00);
	}
	else if(((flag[2] & 0xff) >> 7)== 0x00)
	{
		//Module info group4
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04); //make initial state
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a02, 0x04); //page 5,module info part in page 5
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x01); //otp enable read
		otp_info->module_id = flag[2] & 0x7f;
		otp_info->lens_id = S5K5E2_CXT_K300_read_cmos_sensor(0x0A05);
		otp_info->r_current = S5K5E2_CXT_K300_read_cmos_sensor(0x0A06);
		otp_info->g_current = S5K5E2_CXT_K300_read_cmos_sensor(0x0A07);
		otp_info->b_current = S5K5E2_CXT_K300_read_cmos_sensor(0x0A08);
		otp_info->r_typical = S5K5E2_CXT_K300_read_cmos_sensor(0x0A09);
		otp_info->b_typical = S5K5E2_CXT_K300_read_cmos_sensor(0x0A0A);
		otp_info->g_typical = S5K5E2_CXT_K300_read_cmos_sensor(0x0A0B);
		otp_info->flag = 4;
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04);
		S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x00);
	}
	else {
		printk("S5K5E2_CXT_K300 otp data is invalid\n");
	}
	/*print otp information*/
	SENSOR_PRINT("flag=0x%x\n",otp_info->flag);
	SENSOR_PRINT("module_id=0x%x\n",otp_info->module_id);
	SENSOR_PRINT("lens_id=0x%x\n",otp_info->lens_id);
	//SENSOR_PRINT("data=%d-%d-%d\n",otp_info->year,otp_info->month,otp_info->day);
	SENSOR_PRINT("r_current=0x%x\n",otp_info->r_current);
	SENSOR_PRINT("b_current=0x%x\n",otp_info->b_current);
	SENSOR_PRINT("g_current=0x%x\n",otp_info->g_current);
	SENSOR_PRINT("r_typical=0x%x\n",otp_info->r_typical);
	SENSOR_PRINT("b_typical=0x%x\n",otp_info->b_typical);
	SENSOR_PRINT("g_typical=0x%x\n",otp_info->g_typical);

	return rtn;

}

uint32_t s5k5e2ya_cxt_k300_update_awb(struct otp_info_t *otp_info)
{
	uint32_t rtn = SENSOR_SUCCESS;

	/*TODO*/
	uint16_t stream_value = 0;
	uint32_t g_gain=0,r_gain=0,b_gain = 0,g_gain_b ,g_gain_r;
	uint32_t rg_ratio_current,bg_ratio_current;

	//calculate R,G,B gain
	//rg_ratio_current = 1024 *otp_info->r_current/otp_info->g_current + 0.5;
	//bg_ratio_current = 1024 *otp_info->b_current/otp_info->g_current + 0.5;
	rg_ratio_current = 1024 *otp_info->r_current/otp_info->g_current;
	bg_ratio_current = 1024 *otp_info->b_current/otp_info->g_current;
	//SENSOR_PRINT("rg_ratio_current=%d\n,bg_ratio_current = %d\n",rg_ratio_current,bg_ratio_current);
	
	if(bg_ratio_current < otp_info->bg_ratio_typical){
		if(rg_ratio_current< otp_info->rg_ratio_typical){
			g_gain= 0x100;
			b_gain = 0x100 * otp_info->bg_ratio_typical / bg_ratio_current;
			r_gain = 0x100 *  otp_info->rg_ratio_typical / rg_ratio_current;
		}
		else{
			r_gain = 0x100;
			g_gain = 0x100 * rg_ratio_current / otp_info->rg_ratio_typical;
			b_gain = g_gain *otp_info->bg_ratio_typical  / bg_ratio_current;	        
		}
	}
	else{
		if(rg_ratio_current < otp_info->rg_ratio_typical){
			b_gain = 0x100;
			g_gain = 0x100 * bg_ratio_current /otp_info->bg_ratio_typical;
			r_gain = g_gain *otp_info->rg_ratio_typical /rg_ratio_current;
		}
		else{
			g_gain_b = 0x100*bg_ratio_current / otp_info->bg_ratio_typical;
			g_gain_r = 0x100*rg_ratio_current / otp_info->rg_ratio_typical;

			if(g_gain_b > g_gain_r)	{
				b_gain = 0x100;
				g_gain = g_gain_b;
				r_gain = g_gain * otp_info->rg_ratio_typical / rg_ratio_current;
			}
			else	{
				r_gain = 0x100;
				g_gain = g_gain_r;
				b_gain= g_gain * otp_info->bg_ratio_typical / bg_ratio_current;
			}        
		}	
	}

	//write to register
	SENSOR_PRINT("r_Gain=0x%x\n", r_gain);	
	SENSOR_PRINT("g_Gain=0x%x\n", g_gain);	
	SENSOR_PRINT("b_Gain=0x%x\n", b_gain);	

	S5K5E2_CXT_K300_write_cmos_sensor(0x020e, (g_gain&0xff00)>>8);
	S5K5E2_CXT_K300_write_cmos_sensor(0x020f, g_gain&0xff);
	S5K5E2_CXT_K300_write_cmos_sensor(0x0210, (r_gain&0xff00)>>8);
	S5K5E2_CXT_K300_write_cmos_sensor(0x0211, r_gain&0xff);
	S5K5E2_CXT_K300_write_cmos_sensor(0x0212, (b_gain&0xff00)>>8);
	S5K5E2_CXT_K300_write_cmos_sensor(0x0213, b_gain&0xff);
	S5K5E2_CXT_K300_write_cmos_sensor(0x0214, (g_gain&0xff00)>>8);
	S5K5E2_CXT_K300_write_cmos_sensor(0x0215, g_gain&0xff);

	return SENSOR_SUCCESS;
}

#ifndef OTP_AUTO_LOAD_LSC_s5k5e2ya_cxt_k300

static uint32_t s5k5e2ya_cxt_k300_update_lsc(void *param_ptr)
{
	uint32_t rtn = SENSOR_SUCCESS;
	struct otp_info_t *otp_info=(struct otp_info_t *)param_ptr;

	/*TODO*/
	
	return rtn;
}

#endif
static uint32_t s5k5e2ya_cxt_k300_update_otp(struct otp_info_t *otp_info)
{
	uint32_t rtn = SENSOR_SUCCESS;
	//struct otp_info_t *otp_info=(struct otp_info_t *)param_ptr;

	rtn=s5k5e2ya_cxt_k300_update_awb(otp_info);
	if(rtn!=SENSOR_SUCCESS)
	{
		SENSOR_PRINT("OTP awb appliy error!");
		return rtn;
	}

	#ifndef OTP_AUTO_LOAD_LSC_s5k5e2ya_cxt_k300
	
	rtn=s5k5e2ya_cxt_k300_update_lsc(otp_info);
	if(rtn!=SENSOR_SUCCESS)
	{
		SENSOR_PRINT("OTP lsc appliy error!");
		return rtn;
	}
	#endif
	
	return rtn;
}

static uint32_t s5k5e2ya_cxt_k300_identify_otp(struct otp_info_t *otp_info)
{
	uint32_t rtn = SENSOR_SUCCESS;

	rtn=s5k5e2ya_cxt_k300_read_otp_info(otp_info);
	SENSOR_PRINT("rtn=%d",rtn);

	return rtn;
}

uint16_t s5k5e2ya_cxt_k300_get_otp_module_id(void)
{
	uint16_t module_id = 0;
	uint16_t flag[3] = {0};
	int i = 0;
	
	//Read module info	
	for(i = 0; i < 3; i++){
	S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04); //make initial state
	S5K5E2_CXT_K300_write_cmos_sensor(0x0a02, i + 2); //page 5,module info part in page 5
	S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x01); //otp enable read
	mdelay(5);
	flag[i] = S5K5E2_CXT_K300_read_cmos_sensor(0x0A04);
	printk("wubingzhong: flag[%d]= %d\n",i,flag[i]);
	S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x04);
	S5K5E2_CXT_K300_write_cmos_sensor(0x0a00, 0x00);
	mdelay(5);
	}
	//flag = S5K5E2_CXT_K300_read_cmos_sensor(0x0A07)| S5K5E2_CXT_K300_read_cmos_sensor(0x0A08) | S5K5E2_CXT_K300_read_cmos_sensor(0x0A09);
	
	if(((flag[0] & 0xff) >> 7)== 0x00)
	{  //Module info group2
		module_id = flag[0] & 0x7f;
		printk("group2 module_id = %d\n",module_id);
	}
	else if(((flag[1] & 0xff) >> 7)== 0x00)
	{ //Module info group3
		module_id = flag[1] & 0x7f;
		printk("group3 module_id = %d\n",module_id);
	}
	else if(((flag[2] & 0xff) >> 7)== 0x00)
	{
		//Module info group4
		module_id = flag[2] & 0x7f;
		printk("group4 module_id = %d\n",module_id);
	}
	else {
		printk("S5K5E2_CXT_K300 otp data is invalid\n");
	}
	return module_id;
}
