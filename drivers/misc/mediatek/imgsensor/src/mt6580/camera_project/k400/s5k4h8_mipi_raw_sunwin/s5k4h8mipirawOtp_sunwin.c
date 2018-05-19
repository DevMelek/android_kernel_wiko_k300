#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include "kd_camera_hw.h"
#include "kd_imgsensor.h" 
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "cam_cal.h"
#include "cam_cal_define.h"

#include "s5k4h8mipirawSensor_sunwin.h"

/************Module Info**********/
#define MODULE_ID			0x06
#define LENS_ID				0x68
#define VCM_ID				0x01
#define DRIVER_IC			0x01
/********Register address Info*******/
#define WB_GROUP1_ADD		0x0A04
#define WB_GROUP2_ADD		0x0A1A
#define AF_GROUP1_ADD		0x0A30
#define AF_GROUP2_ADD		0x0A37
#define LSC_GROUP1_ADD		0x0A3E
#define LSC_GROUP2_ADD		0x0A41

#define WB_GROUP1_YEAR	0x0A09
#define WB_GROUP1_MONTH	0x0A0A
#define WB_GROUP1_DATE	0x0A0B
#define WB_GROUP2_YEAR	0x0A1F
#define WB_GROUP2_MONTH	0x0A20
#define WB_GROUP2_DATE	0x0A21
/************Const Info*************/
#define RESULT_VALID_VALUE	0x01
#define RGr_ratio_Typical		597
#define BGr_ratio_Typical		720
#define GbGr_ratio_Typical		1015

#define GROUP_EMPTY      0x00
#define GROUP_VALID      0x01
#define GROUP_INVALID    0x03

#define LOG_INF(format, args...)	printk("[%s]"format, __FUNCTION__, ##args)

int flag_otp = 0x00;
int RGr_ratio = 0x00;
int BGr_ratio = 0x00;
int GbGr_ratio = 0x00;
u16 otp_4h8_af_data[2];
int otp_infos[7];

extern void S5K4H8_bytewrite_cmos_sensor(u16 addr, kal_uint16 para);
extern kal_uint16 S5K4H8_byteread_cmos_sensor(u16 addr);

 void enable_read_otp(void)
{
    S5K4H8_bytewrite_cmos_sensor(0x0A02, 0x0f); //Set the PAEGE0 of OTP(0 <= the number of PAGE <= 15)
    S5K4H8_bytewrite_cmos_sensor(0x0A00, 0x01); //Read enable
    mdelay(10);
}

 void disable_read_otp(void)
{
    S5K4H8_bytewrite_cmos_sensor(0x0A00, 0x00); //Reset the NVM interface bywriting 00h to 0xD0000A00
}

int S5K4H8_read_OTP(void)
{
    int year = 0x00, month = 0x00, date = 0x00, ret = 0, AWBFlag, AFFlag;
    u16 addr;
    int AWBFlag1 = 0x00,AWBFlag2 = 0x00,AFFlag1 = 0x00,AFFlag2 = 0x00;
    enable_read_otp();
#if 1
    AWBFlag1 = S5K4H8_byteread_cmos_sensor(WB_GROUP1_ADD); //flag of info and WB
    AWBFlag = AWBFlag1;

    //0x13&0x37 is holitech module flag
    if (AWBFlag == 0x13 || AWBFlag == 0x37) {
        AWBFlag = -1;
    } else if (AWBFlag == RESULT_VALID_VALUE) {
        addr = WB_GROUP1_ADD;
    } else {
        AWBFlag2 = S5K4H8_byteread_cmos_sensor(WB_GROUP2_ADD);
        AWBFlag = AWBFlag2;
        addr = WB_GROUP2_ADD;
    }
    LOG_INF("AWBFlag=%d, AWBFlag1=%d, AWBFlag2=%d\n", AWBFlag, AWBFlag1, AWBFlag2);
    switch (AWBFlag) {
        case GROUP_VALID:
            flag_otp = 0x01;
            otp_infos[0] = S5K4H8_byteread_cmos_sensor(addr + 1); //Module Vendor ID
            LOG_INF("MID=0x%02x\n", otp_infos[0]);
            if (otp_infos[0] == MODULE_ID) {
                otp_infos[1] = S5K4H8_byteread_cmos_sensor(addr + 2); //Lens ID
                otp_infos[2] = S5K4H8_byteread_cmos_sensor(addr + 3); //VCM ID
                otp_infos[3] = S5K4H8_byteread_cmos_sensor(addr + 4); //Drivers IC
                otp_infos[4] = S5K4H8_byteread_cmos_sensor(addr + 5); //Year
                otp_infos[5] = S5K4H8_byteread_cmos_sensor(addr + 6); //Month
                otp_infos[6] = S5K4H8_byteread_cmos_sensor(addr + 7); //Date
                RGr_ratio = S5K4H8_byteread_cmos_sensor(addr + 8) | (S5K4H8_byteread_cmos_sensor(addr + 9)<<8);
                BGr_ratio = S5K4H8_byteread_cmos_sensor(addr + 10) | (S5K4H8_byteread_cmos_sensor(addr + 11)<<8);
                GbGr_ratio = S5K4H8_byteread_cmos_sensor(addr + 12) | (S5K4H8_byteread_cmos_sensor(addr + 13)<<8);
                LOG_INF("LensID=0x%02x VCM ID=0x%02x DriversIC=0x%02x\n",otp_infos[1],otp_infos[2],otp_infos[3]);
                LOG_INF("Year=20%d, Month=%2d, Date=%2d\n",otp_infos[4],otp_infos[5],otp_infos[6]);
                LOG_INF("RGr_ratio=0x%x, BGr_ratio=0x%x, GbGr_ratio=0x%x\n",RGr_ratio,BGr_ratio,GbGr_ratio);
                ret = 1;
            }
            break;
        case GROUP_EMPTY:
        case GROUP_INVALID:
            ret = 0;
            break;
        default:
            ret = 0;
            break;
    }

    //if awb group valid(ret==1), then read af info.
    if (ret) {
        AFFlag1 = S5K4H8_byteread_cmos_sensor(AF_GROUP1_ADD); //falg of VCM/AF
        AFFlag = AFFlag1;
        addr = AF_GROUP1_ADD;
        if (AFFlag != RESULT_VALID_VALUE) {
            AFFlag2 = S5K4H8_byteread_cmos_sensor(AF_GROUP2_ADD);
            AFFlag = AFFlag2;
            addr = AF_GROUP2_ADD;
        }
        LOG_INF("AFFlag=%d, AFFlag1=%d, AFFlag2=%d\n", AFFlag, AFFlag1, AFFlag2);
        switch (AFFlag) {
            case GROUP_VALID:
                flag_otp = flag_otp | 0x02;
                otp_4h8_af_data[0] = S5K4H8_byteread_cmos_sensor(addr + 1) | (S5K4H8_byteread_cmos_sensor(addr + 2)<<8);
                otp_4h8_af_data[1] = S5K4H8_byteread_cmos_sensor(addr + 3) | (S5K4H8_byteread_cmos_sensor(addr + 4)<<8);
                LOG_INF("otp_4h8_af_data[0]=%d, otp_4h8_af_data[1]=%d\n", otp_4h8_af_data[0],otp_4h8_af_data[1]);
                ret = 1;
                break;
            case GROUP_EMPTY:
            case GROUP_INVALID:
                ret = 0;
                break;
            default:
                ret = 0;
                break;
        }
    }
#else
    //flag of info and WB
    AWBFlag1 = S5K4H8_byteread_cmos_sensor(WB_GROUP1_ADD);
    LOG_INF("AWBFlag1 == %d\n", AWBFlag1);
    if (AWBFlag1 != RESULT_VALID_VALUE) {
	 AWBFlag2 = S5K4H8_byteread_cmos_sensor(WB_GROUP2_ADD);
	 LOG_INF("AWB group2 read 0x0A1A = 0x%02x\n", AWBFlag2);
    }
    if (AWBFlag1 == RESULT_VALID_VALUE) {
	 flag_otp = 0x01;
	 otp_infos[0] = S5K4H8_byteread_cmos_sensor(0x0A05);//Module Vendor ID
	 otp_infos[1] = S5K4H8_byteread_cmos_sensor(0x0A06);//Lens ID
	 otp_infos[2] = S5K4H8_byteread_cmos_sensor(0x0A07);//VCM ID
	 otp_infos[3] = S5K4H8_byteread_cmos_sensor(0x0A08);//Drivers IC
	 otp_infos[4] = S5K4H8_byteread_cmos_sensor(0x0A09);//Year
	 otp_infos[5] = S5K4H8_byteread_cmos_sensor(0x0A0A);//Month
	 otp_infos[6] = S5K4H8_byteread_cmos_sensor(0x0A0B);//Date
	 RGr_ratio = S5K4H8_byteread_cmos_sensor(0x0A0C) | (S5K4H8_byteread_cmos_sensor(0x0A0D)<<8);
	 BGr_ratio = S5K4H8_byteread_cmos_sensor(0x0A0E) | (S5K4H8_byteread_cmos_sensor(0x0A0F)<<8);
	 GbGr_ratio = S5K4H8_byteread_cmos_sensor(0x0A10) | (S5K4H8_byteread_cmos_sensor(0x0A11)<<8);
	 LOG_INF("AWB group1 read 0x0A04 = 0x%02x\n", AWBFlag1);
	 LOG_INF("AWBFlag1 RGr_ratio=0x%x, BGr_ratio=0x%x, GbGr_ratio=0x%x\n",RGr_ratio,BGr_ratio,GbGr_ratio);
    } else if (AWBFlag2 == RESULT_VALID_VALUE) {
	 flag_otp = 0x01;
	 otp_infos[0] = S5K4H8_byteread_cmos_sensor(0x0A1B);//Module Vendor ID
	 otp_infos[1] = S5K4H8_byteread_cmos_sensor(0x0A1C);//Lens ID
	 otp_infos[2] = S5K4H8_byteread_cmos_sensor(0x0A1D);//VCM ID
	 otp_infos[3] = S5K4H8_byteread_cmos_sensor(0x0A1E);//Drivers IC
	 otp_infos[4] = S5K4H8_byteread_cmos_sensor(0x0A1F);//Year
	 otp_infos[5] = S5K4H8_byteread_cmos_sensor(0x0A20);//Month
	 otp_infos[6] = S5K4H8_byteread_cmos_sensor(0x0A21);//Date
	 RGr_ratio = S5K4H8_byteread_cmos_sensor(0x0A22) | (S5K4H8_byteread_cmos_sensor(0x0A23)<<8);
	 BGr_ratio = S5K4H8_byteread_cmos_sensor(0x0A24) | (S5K4H8_byteread_cmos_sensor(0x0A25)<<8);
	 GbGr_ratio = S5K4H8_byteread_cmos_sensor(0x0A26) | (S5K4H8_byteread_cmos_sensor(0x0A27)<<8);
	 LOG_INF("AWBFlag2 RGr_ratio=0x%x, BGr_ratio=0x%x, GbGr_ratio=0x%x\n",RGr_ratio,BGr_ratio,GbGr_ratio);
    }
    LOG_INF("ModuleID=0x%02x LensID=0x%02x VCM ID=0x%02x Drivers IC=0x%02x\n", otp_infos[0],otp_infos[1],otp_infos[2],otp_infos[3]);
    LOG_INF("Year=20%d, Month=%2d, Date=%2d\n", otp_infos[4],otp_infos[5],otp_infos[6]);

    //falg of VCM/AF
    AFFlag1 = S5K4H8_byteread_cmos_sensor(AF_GROUP1_ADD);
    if (AFFlag1 != RESULT_VALID_VALUE) {
	 AFFlag2 = S5K4H8_byteread_cmos_sensor(AF_GROUP2_ADD);
	 LOG_INF("VCM/AF read group2 0x0A37 = 0x%02x\n", AFFlag2);
    }
    if (AFFlag1 == RESULT_VALID_VALUE) {
	 flag_otp = flag_otp | 0x02;
      	 otp_4h8_af_data[0] = S5K4H8_byteread_cmos_sensor(0x0A31) | (S5K4H8_byteread_cmos_sensor(0x0A32)<<8);
	 otp_4h8_af_data[1] = S5K4H8_byteread_cmos_sensor(0x0A33) | (S5K4H8_byteread_cmos_sensor(0x0A34)<<8);
	 LOG_INF("VCM/AF read group1 0x0A30 = 0x%02x\n", AFFlag1);
	 LOG_INF("AFFlag1 otp_4h8_af_data[0]=0x%x, otp_4h8_af_data[1]=0x%x\n", otp_4h8_af_data[0],otp_4h8_af_data[1]);
    } else if (AFFlag2 == RESULT_VALID_VALUE) {
	 flag_otp = flag_otp | 0x02;
      	 otp_4h8_af_data[0] = S5K4H8_byteread_cmos_sensor(0x0A38) | (S5K4H8_byteread_cmos_sensor(0x0A39)<<8);
	 otp_4h8_af_data[1] = S5K4H8_byteread_cmos_sensor(0x0A3A) | (S5K4H8_byteread_cmos_sensor(0x0A3B)<<8);
	 LOG_INF("AFFlag2 otp_4h8_af_data[0]=0x%x, otp_4h8_af_data[1]=0x%x\n", otp_4h8_af_data[0],otp_4h8_af_data[1]);
    }
#endif
    //falg of LSC
    /*LSCFlag1 = S5K4H8_byteread_cmos_sensor(LSC_GROUP1_ADD);
    if (LSCFlag1 != RESULT_VALID_VALUE) {
	 LSCFlag2 = S5K4H8_byteread_cmos_sensor(LSC_GROUP2_ADD);
	 LOG_INF("LSC read group2 0x0A41 = 0x%02x\n", LSCFlag2);
    }
    if (LSCFlag1 == RESULT_VALID_VALUE) {
	 flag_otp = flag_otp | 0x04;
	 LOG_INF("LSC read group1 0x0A3E = 0x%02x\n", LSCFlag1);
    } else if (LSCFlag2 == RESULT_VALID_VALUE) {
	 flag_otp = flag_otp | 0x04;
    }*/
    disable_read_otp();
    LOG_INF("S5K4H8_read_OTP ret == %d\n", ret);
    return ret;
}

void S5K4H8_apply_OTP(void)
{
    if (flag_otp != 0x03) { //0X03->AWB&AF; 0x07->AWB&AF&LSC
	 LOG_INF("S5K4H8_apply_OTP  Err:  flag_otp = 0x%02x\n", flag_otp);
	 return;
    }

    int R_to_G = 0, B_to_G = 0;
    int R_Gain = 0, B_Gain = 0, G_Gain = 0;
    int G_gain_R = 0, G_gain_B = 0;
    int RG_RATIO_TYPICAL_VALUE = 0, BG_RATIO_TYPICAL_VALUE = 0;

    R_to_G = RGr_ratio;
    B_to_G = BGr_ratio;
    RG_RATIO_TYPICAL_VALUE = RGr_ratio_Typical;
    BG_RATIO_TYPICAL_VALUE = BGr_ratio_Typical;
    LOG_INF("apply, R_to_G:0x%0x,B_to_G:0x%0x\n", R_to_G, B_to_G);
    LOG_INF("apply, R_to_G_golden:0x%0x,B_to_G_golden:0x%0x\n", RG_RATIO_TYPICAL_VALUE,BG_RATIO_TYPICAL_VALUE);
    if (B_to_G < BG_RATIO_TYPICAL_VALUE) {
	 if (R_to_G < RG_RATIO_TYPICAL_VALUE) {
		G_Gain = 0x100;
		B_Gain = 0x100 * BG_RATIO_TYPICAL_VALUE / B_to_G;
		R_Gain = 0x100 * RG_RATIO_TYPICAL_VALUE / R_to_G;
	 } else {
		R_Gain = 0x100;
		G_Gain = 0x100 * R_to_G / RG_RATIO_TYPICAL_VALUE;
		B_Gain = G_Gain * BG_RATIO_TYPICAL_VALUE / B_to_G;			
	 }
    } else {
	 if (R_to_G < RG_RATIO_TYPICAL_VALUE) {
		B_Gain = 0x100;
		G_Gain = 0x100 * B_to_G / BG_RATIO_TYPICAL_VALUE;
		R_Gain = G_Gain * RG_RATIO_TYPICAL_VALUE / R_to_G;
	 } else {
		G_gain_B = 0x100*B_to_G / BG_RATIO_TYPICAL_VALUE;
		G_gain_R = 0x100*R_to_G / RG_RATIO_TYPICAL_VALUE;
		if (G_gain_B > G_gain_R) {
			B_Gain = 0x100;
			G_Gain = G_gain_B;
			R_Gain = G_Gain * RG_RATIO_TYPICAL_VALUE / R_to_G;
		} else {
			R_Gain = 0x100;
			G_Gain = G_gain_R;
			B_Gain = G_Gain * BG_RATIO_TYPICAL_VALUE / B_to_G;
		}
	 }
    }
    /* update digital gain */
    LOG_INF("R_gain:0x%0x,G_gain:0x%0x, B_gain:0x%0x,", R_Gain, G_Gain, B_Gain);
    S5K4H8_bytewrite_cmos_sensor(0x3058,0x01);
    if (G_Gain>0x100)
    {
	S5K4H8_bytewrite_cmos_sensor(0x020E,G_Gain>>8);
	S5K4H8_bytewrite_cmos_sensor(0x020F,G_Gain&0xff);
    }
    if (R_Gain>0x100)
    {
	S5K4H8_bytewrite_cmos_sensor(0x0210,R_Gain>>8);
	S5K4H8_bytewrite_cmos_sensor(0x0211,R_Gain&0xff);
    }
    if (B_Gain>0x100)
    {
	S5K4H8_bytewrite_cmos_sensor(0x0212,B_Gain>>8);
	S5K4H8_bytewrite_cmos_sensor(0x0213,B_Gain&0xff);
    }
    if (G_Gain>0x100)
    {
	S5K4H8_bytewrite_cmos_sensor(0x0214,G_Gain>>8);
	S5K4H8_bytewrite_cmos_sensor(0x0215,G_Gain&0xff);
    }
    LOG_INF("update otp ok \n,");
}
	/*************************************************************************************************
* Function : otp_update_wb()
* Description : update otp_sunny data from otp_sunny , it otp_sunny data is valid,
it include get ID and WB update function
* Return : [bool] 0 : update fail
1 : update success
**************************************************************************************************/
void otp_update_wb(void)
{
	LOG_INF("statrt read otp  \n");
	S5K4H8_read_OTP();
	LOG_INF("statrt write otp  \n");
	S5K4H8_apply_OTP();
	LOG_INF("write otp end \n");
}

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "S5K4H8SUNWIN_CAM_CAL_DRV"
/*******************************************************************************
*
********************************************************************************/
static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0x20>>1)}; //make dummy_eeprom co-exist

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(226, 0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
/*******************************************************************************
*
********************************************************************************/
//UINT32 S5k4H8_CheckID = 0x884000dd;
u8 S5k4H8_CheckID[]= {0x10,0xdd,0x00,0x40,0x88};

static int selective_read_region(u32 offset, BYTE* data,u16 i2c_id,u32 size)
{    	
    printk("[s5k4h8_CAM_CAL] selective_read_region offset =%d size %d data read = %d\n", offset,size, *data);

	if (size == 2) {
		if (offset == 7) {
			memcpy((void *)data,(void *)&otp_4h8_af_data[0],size);
		} else if (offset == 9) {		 
			memcpy((void *)data,(void *)&otp_4h8_af_data[1],size);
		}
	}

	if (size == 4) {
		memcpy((void *)data,(void *)&S5k4H8_CheckID[1],size);
	}
	printk("+ls.test[s5k4h8_CAM_CAL]  data1 = %x\n",*(UINT32 *)data);
}
/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;

#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if (_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if (NULL == pBuff)
        {
            LOG_INF("[S5K4H8_CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if (_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if (copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                LOG_INF("[S5K4H8_CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if (NULL == pWorkingBuff)
    {
        kfree(pBuff);
        LOG_INF("[S5K4H8_CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }

    if (copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        LOG_INF("[S5K4H8_CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
	case CAM_CALIOC_S_WRITE:
      		LOG_INF("[SENSORDB_S5K4H8_OTP] CAM_CALIOC_S_WRITE \n");        
	#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
	#endif
            i4RetValue = 0;
           // i4RetValue=iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
	#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
	#endif
            break;
        case CAM_CALIOC_G_READ:
            LOG_INF("[S5K4H8_CAM_CAL] Read CMD \n");
	#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
	#endif 
            LOG_INF("[S5K4H8_CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            LOG_INF("[S5K4H8_CAM_CAL] length %d \n", ptempbuf->u4Length);

	if (ptempbuf->u4Length == 2) {
       i4RetValue = selective_read_region(ptempbuf->u4Offset, pWorkingBuff, 0x20, ptempbuf->u4Length);
	} else if (ptempbuf->u4Length == 4) {
	   i4RetValue = selective_read_region(ptempbuf->u4Offset, pWorkingBuff, 0x20, ptempbuf->u4Length);
 	}

	#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
	#endif            

            break;
        default :
      	     LOG_INF("[S5K4H8_CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        LOG_INF("[S5K4H8_CAM_CAL] to user length %d \n", ptempbuf->u4Length);
		if (copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            LOG_INF("[S5K4H8_CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    LOG_INF("[s5k4h8 CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);
    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);
    g_u4Opened = 0;
    atomic_set(&g_CAM_CALatomic,0);
    spin_unlock(&g_CAM_CALLock);
    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if (alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME))
    {
        LOG_INF("[S5K4H8_CAM_CAL] Allocate device no failed\n");
        return -EAGAIN;
    }
#else
    if (register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME))
    {
        LOG_INF("[S5K4H8_CAM_CAL] Register device no failed\n");
        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if (NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);
        LOG_INF("[S5K4H8_CAM_CAL] Allocate mem for kobject failed\n");
        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);
    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if (cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        LOG_INF("[S5K4H8_CAM_CAL] Attatch file operation failed\n");
        unregister_chrdev_region(g_CAM_CALdevno, 1);
        return -EAGAIN;
    }
 
    CAM_CAL_class = class_create(THIS_MODULE, CAM_CAL_DRVNAME);
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        LOG_INF("Unable to create class, err = %d\n", ret);
        return ret;
    }
	
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);
    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);
    unregister_chrdev_region(g_CAM_CALdevno, 1);
    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
    return 0;//i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    //i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};

static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{
    int i4RetValue = 0;
    LOG_INF("[s5k4h8_CAM_CAL]\n");
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if (i4RetValue){
 	   LOG_INF(" [s5k4h8_CAM_CAL] register char device failed!\n");
	   return i4RetValue;
	}
	LOG_INF(" [s5k4h8_CAM_CAL] Attached!! \n");

   
    if (platform_driver_register(&g_stCAM_CAL_Driver)){
        printk("failed to register CAM_CAL driver\n");
        return -ENODEV;
    }
    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        printk("failed to register CAM_CAL driver\n");
        return -ENODEV;
    }	
	LOG_INF(" s5k4h8_CAM_CAL  Attached Pass !! \n");
    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);
