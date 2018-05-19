#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "cam_cal.h"
#include "cam_cal_define.h"

#include "s5k3h7yxmipiraw_Sensor.h"

extern void wordwrite_cmos_sensor(u16 addr, kal_uint16 para);
extern void bytewrite_cmos_sensor(u16 addr, kal_uint16 para);
extern unsigned char byteread_cmos_sensor(u16 addr);

#define USHORT unsigned short
#define BYTE unsigned char

#define MODULE_ID 0x06
#define LENS_ID 0x46

int OTP_RGr_ratio = 0x00;
int OTP_BGr_ratio = 0x00;
int OTP_GbGr_ratio = 0x00;
int otp_module_infos[4];
u16 otp_af_data[3];

#define OTP_RGr_ratio_Typical  597
#define OTP_BGr_ratio_Typical  720
#define OTP_GbGr_ratio_Typical 1015

#define TAG "s5k3h7yx_otp"
#define LOG_INF(format, args...) printk(TAG "[%s]"format, __FUNCTION__, ##args)

/*************************************************************************************************
* Function : start_read_otp
* Description : before read otp , set the reading block setting
* Return : 0, reading block setting err; 1, reading block setting ok
**************************************************************************************************/
static bool start_read_otp(BYTE page)
{
    BYTE val = 0;
    int i;
    wordwrite_cmos_sensor(0xFCFC, 0xD000);
    bytewrite_cmos_sensor(0x0A02, page); //Select the page to write by writing to 0xD0000A02 0x01~0x0C
    bytewrite_cmos_sensor(0x0A00, 0x01); //Enter read mode by writing 01h to 0xD0000A00
    //mdelay(10);
    for(i = 0; i < 100; i++)
    {
        val = byteread_cmos_sensor(0x0A01);
        LOG_INF("read 0x0A01 = 0x%x, i = %d\n", val, i);
        if(val == 0x01)
            break;
        mdelay(2);
    }
    if(i == 100)
    {
        LOG_INF("Read Page %d Err!\n", page);
        bytewrite_cmos_sensor(0x0A00, 0x00); //Reset the NVM interface by  writing 00h to 0xD0000A00
        return 0;
    }
    return 1;
}

/*************************************************************************************************
* Function : stop_read_otp
* Description : after read otp , stop and reset otp block setting
**************************************************************************************************/
static void stop_read_otp(void)
{
    bytewrite_cmos_sensor(0x0A00, 0x00); //Reset the NVM interface bywriting 00h to 0xD0000A00
}

/*************************************************************************************************
* Function : get_otp_infos
* Description : get otp WRITTEN_INFOS
* Return : [BYTE], if 0x40 or 0x10 , this type has valid otp data, otherwise, invalid otp data
**************************************************************************************************/
static BYTE get_otp_infos(void)
{
    BYTE flag = 0;
    u16 start_addr, checksum_addr;
    int sum = 0, i;

    flag = byteread_cmos_sensor(0x0A04);
    LOG_INF("read 0x0A04 = 0x %x\n",flag);
    if ((flag & 0xc0) == 0x40) //Group 1 valid:Bit[7:6] 00:Empty,01:Valid,11:Invalid
    {
        start_addr = 0x0A05;
        checksum_addr = 0x0A1B;
    }
    else if ((flag & 0x30) == 0x10) //Group 2 valid:Bit[5:4] 00:Empty,01:Valid,11:Invalid
    {
        start_addr = 0x0A09;
        checksum_addr = 0x0A1E;
    }
    else
    {
        LOG_INF("otp_infos invalid\n");
        return 0;
    }
    //otp_module_infos[0] = byteread_cmos_sensor(start_addr); //module_id
    //otp_module_infos[1] = byteread_cmos_sensor(start_addr + 1); //lens_id
    //otp_module_infos[2] = byteread_cmos_sensor(start_addr + 2); //vcm_id
    //otp_module_infos[3] = byteread_cmos_sensor(start_addr + 3); //driver_id

    for (i = 0; i < 4 ; i++)
    {
        otp_module_infos[i] = byteread_cmos_sensor(start_addr + i);
        LOG_INF("otp_module_infos[%d] = 0x%x\n", i, otp_module_infos[i]);
        sum += otp_module_infos[i];
    }
    int checksum = byteread_cmos_sensor(checksum_addr);
    LOG_INF("infos sum = %d, checksum = 0x%x, datachecksum = 0x%x\n", sum, checksum, sum%255+1);
    if (checksum != (sum%255+1))
    {
        LOG_INF("infos checksum err\n");
        return 0;
    }

    return 1;
}

/*************************************************************************************************
* Function : get_otp_wb
* Description : get otp WRITTEN_WB_DATA
* Return : [BYTE], if 0x40 , this type has valid otp data, otherwise,
invalid otp data
**************************************************************************************************/
static BYTE get_otp_wb(void)
{
    BYTE flag = 0;
    u16 start_addr, checksum_addr, otp_wb_data[6];
    int sum = 0, i;

    flag = byteread_cmos_sensor(0x0A0D);
    LOG_INF("read 0x0A0D = 0x %x\n",flag);
    if ((flag & 0xc0) == 0x40) //Group 1 valid:Bit[7:6] 00:Empty,01:Valid,11:Invalid
    {
        start_addr = 0x0A0E;
        checksum_addr = 0x0A1C;
    }
    else if ((flag & 0x30) == 0x10) //Group 2 valid:Bit[5:4] 00:Empty,01:Valid,11:Invalid
    {
        start_addr = 0x0A14;
        checksum_addr = 0x0A1F;
    }
    else 
    {
        LOG_INF("otp_wb invalid\n");
        return 0;
    }

    //otp_wb_data[0] = byteread_cmos_sensor(start_addr); //rg_ratio_h
    //otp_wb_data[1] = byteread_cmos_sensor(start_addr + 1); //rg_ratio_l
    //otp_wb_data[2] = byteread_cmos_sensor(start_addr + 2); //bg_ratio_h
    //otp_wb_data[3] = byteread_cmos_sensor(start_addr + 3); //bg_ratio_l
    //otp_wb_data[4] = byteread_cmos_sensor(start_addr + 4); //g_ratio_h
    //otp_wb_data[5] = byteread_cmos_sensor(start_addr + 5); //g_ratio_l
    for (i = 0; i < 6 ; i++)
    {
        otp_wb_data[i] = byteread_cmos_sensor(start_addr + i);
        LOG_INF("otp_wb_data[%d] = 0x%x\n", i, otp_wb_data[i]);
        sum += otp_wb_data[i];
    }

    int checksum = byteread_cmos_sensor(checksum_addr);
    LOG_INF("wb sum = %d, checksum = 0x%x, datachecksum = 0x%x\n", sum, checksum, sum%255+1);
    if (checksum != (sum%255+1))
    {
        LOG_INF("wb checksum err\n");
        return 0;
    }

    OTP_RGr_ratio = ((otp_wb_data[0] << 8) | otp_wb_data[1]); //R/G
    OTP_BGr_ratio = ((otp_wb_data[2] << 8) | otp_wb_data[3]); //B/G
    OTP_GbGr_ratio = ((otp_wb_data[4] << 8) | otp_wb_data[5]); //Gr/Gb

    return 1;
}

/*************************************************************************************************
* Function : get_otp_lsc
* Description : get otp WRITTEN_LSC_DATA
* Return : [BYTE], if 0x40 , this type has valid otp data, otherwise,
invalid otp data
**************************************************************************************************/
static BYTE get_otp_lsc(void)
{
    BYTE flag = 0;
    flag = byteread_cmos_sensor(0x0A1A);
    LOG_INF("read 0x0A1A = 0x %x\n",flag);
    flag = flag & 0xc0;
    LOG_INF("Flag:0x%02x",flag);
    return flag;
}

/*************************************************************************************************
* Function : get_otp_af
* Description : get otp WRITTEN_AF_DATA
* Return : [BYTE], if 0x40 , this type has valid otp data, otherwise, invalid otp data
**************************************************************************************************/
static BYTE get_otp_af(void)
{
    BYTE flag = 0;
    u16 start_addr, checksum_addr, af_data[6];
    int sum = 0, i;

    flag = byteread_cmos_sensor(0x0A25);
    LOG_INF("read 0x0A25 = 0x %x\n",flag);
    if ((flag & 0xc0) == 0x40) //Group 1 valid:Bit[7:6] 00:Empty,01:Valid,11:Invalid
    {
        start_addr = 0x0A26;
        checksum_addr = 0x0A32;
    }
    else if ((flag & 0x30) == 0x10) //Group 2 valid:Bit[5:4] 00:Empty,01:Valid,11:Invalid
    {
        start_addr = 0x0A2C;
        checksum_addr = 0x0A33;
    }
    else 
    {
        LOG_INF("otp_af invalid\n");
        return 0;
    }

    //af_data[0] = byteread_cmos_sensor(start_addr); //infinity_h
    //af_data[1] = byteread_cmos_sensor(start_addr + 1); //infinity_l
    //af_data[2] = byteread_cmos_sensor(start_addr + 2); //medium_h
    //af_data[3] = byteread_cmos_sensor(start_addr + 3); //medium_l
    //af_data[4] = byteread_cmos_sensor(start_addr + 4); //macro_h
    //af_data[5] = byteread_cmos_sensor(start_addr + 5); //macro_l
    for (i = 0; i < 6 ; i++)
    {
        af_data[i] = byteread_cmos_sensor(start_addr + i);
        LOG_INF("af_data[%d] = 0x%x\n", i, af_data[i]);
        sum += af_data[i];
    }
    int checksum = byteread_cmos_sensor(checksum_addr);
    LOG_INF("af sum = %d, checksum = 0x%x, datachecksum = 0x%x\n", sum, checksum, sum%255+1);
    if (checksum != (sum%255+1))
    {
        LOG_INF("af checksum err\n");
        return 0;
    }

    otp_af_data[0] = ((af_data[0] << 8) | af_data[1]); //infinity
    otp_af_data[1] = ((af_data[2] << 8) | af_data[3]); //medium
    otp_af_data[2] = ((af_data[4] << 8) | af_data[5]); //macro
    LOG_INF("otp_af_data[0] = %d, otp_af_data[1] = %d, otp_af_data[2] = %d\n",
		otp_af_data[0], otp_af_data[1], otp_af_data[2]);

    return 1;
}

/*************************************************************************************************
* Function : apply_otp_wb
* Description : Set WB ratio to register gain setting 512x
* Parameters : [int] r_ratio : R ratio data compared with golden module R
                     b_ratio : B ratio data compared with golden module B
* Return : [bool] 0 : set wb fail; 1 : WB set success
**************************************************************************************************/
static bool apply_otp_wb(void)
{
    int R_to_G = 0, B_to_G = 0;
    int R_Gain = 0, B_Gain = 0, G_Gain = 0;
    int G_gain_R = 0, G_gain_B = 0;
    int RG_RATIO_TYPICAL_VALUE = 0, BG_RATIO_TYPICAL_VALUE = 0;

    if(!OTP_RGr_ratio || !OTP_BGr_ratio)
    {
        LOG_INF("otp WB ratio Data Err!");
        return 0;
    }

    R_to_G = OTP_RGr_ratio;
    B_to_G = OTP_BGr_ratio;
    RG_RATIO_TYPICAL_VALUE = OTP_RGr_ratio_Typical;
    BG_RATIO_TYPICAL_VALUE = OTP_BGr_ratio_Typical;
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
    if(R_Gain < 0x100)
        R_Gain = 0x100;
    if(G_Gain < 0x100)
        G_Gain = 0x100;
    if(B_Gain < 0x100)
        B_Gain = 0x100;

    if (G_Gain > 0x100)
    {
	bytewrite_cmos_sensor(0x020E, G_Gain >> 8);
	bytewrite_cmos_sensor(0x020F, G_Gain & 0xff);
    }
    if (R_Gain > 0x100)
    {
	bytewrite_cmos_sensor(0x0210, R_Gain >> 8);
	bytewrite_cmos_sensor(0x0211, R_Gain & 0xff);
    }
    if (B_Gain > 0x100)
    {
	bytewrite_cmos_sensor(0x0212, B_Gain >> 8);
	bytewrite_cmos_sensor(0x0213, B_Gain & 0xff);
    }
    if (G_Gain > 0x100)
    {
	bytewrite_cmos_sensor(0x0214, G_Gain >> 8);
	bytewrite_cmos_sensor(0x0215, G_Gain & 0xff);
    }
    LOG_INF("apply otp wb Finished! \n");
    return 1;
}

/*************************************************************************************************
* Function : update_otp_wb()
* Description : update otp data from otp , it otp data is valid, it include get ID and WB update function
* Return : [bool] 0 : update fail; 1 : update success
**************************************************************************************************/
bool update_otp_wb(void)
{
    BYTE page = 0x01;
    if(!start_read_otp(page))
    {
        LOG_INF("Start read Page %d fail!\n", page);
        return 0;
    }
    if(!get_otp_infos())
    {
        LOG_INF("get_otp_infos fail!\n");
        return 0;
    }
    if(!get_otp_wb())
    {
        LOG_INF("get_otp_wb fail!");
        return 0;
    }
    if(!get_otp_af())
    {
        LOG_INF("get_otp_af fail!\n");
        return 0;
    }
    stop_read_otp();

    if (!apply_otp_wb())
    {
        LOG_INF("apply_otp_wb fail!\n");
        return 0;
    }

    return 1;
}


static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
/*******************************************************************************
*
*******************************************************************************/
#define CAM_CAL_DRVNAME "S5K3H7YX_CAM_CAL_DRV"
/*******************************************************************************
*
*******************************************************************************/
//make dummy_eeprom co-exist
static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0x20>>1)};

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(226, 0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
static struct class * CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;

/*******************************************************************************
*
********************************************************************************/
//UINT32 S5K3H7YX_CheckID = 0x884000bb;
u8 S5K3H7YX_CheckID[]= {0x10, 0xbb, 0x00, 0x40, 0x88};

static int selective_read_region(u32 offset, BYTE* data,u16 i2c_id,u32 size)
{    	
    printk("[S5K3H7YX_CAM_CAL] selective_read_region offset =%d size %d data read = %d\n", offset,size, *data);

    if (size == 2) {
        if (offset == 7) {
            memcpy((void *)data, (void *)&otp_af_data[0], size);
        } else if (offset == 9) {
            memcpy((void *)data, (void *)&otp_af_data[2], size);
        }
    }

    if (size == 4) {
        memcpy((void *)data, (void *)&S5K3H7YX_CheckID[1], size);
    }
    printk("+ls.test[S5K3H7YX_CAM_CAL]  data1 = %x\n", *(UINT32 *)data);
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
            LOG_INF("[S5K3H7YX_CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if (_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if (copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {
                //get input structure address
                kfree(pBuff);
                LOG_INF("[S5K3H7YX_CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if (NULL == pWorkingBuff)
    {
        kfree(pBuff);
        LOG_INF("[S5K3H7YX_CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }

    if (copy_from_user((u8*)pWorkingBuff, (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        LOG_INF("[S5K3H7YX_CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            LOG_INF("[SENSORDB_S5K3H7YX_OTP] CAM_CALIOC_S_WRITE \n");        
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
            LOG_INF("[S5K3H7YX_CAM_CAL] Read CMD \n");
        #ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
        #endif 
            LOG_INF("[S5K3H7YX_CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            LOG_INF("[S5K3H7YX_CAM_CAL] length %d \n", ptempbuf->u4Length);
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
            LOG_INF("[S5K3H7YX_CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
            break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        LOG_INF("[S5K3H7YX_CAM_CAL] to user length %d \n", ptempbuf->u4Length);
        if (copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            LOG_INF("[S5K3H7YX_CAM_CAL] ioctl copy to user failed\n");
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
    LOG_INF("[S5K3H7YX_CAM_CAL] CAM_CAL_Open\n");
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
    if (alloc_chrdev_region(&g_CAM_CALdevno, 0, 1, CAM_CAL_DRVNAME))
    {
        LOG_INF("[S5K3H7YX_CAM_CAL] Allocate device no failed\n");
        return -EAGAIN;
    }
#else
    if (register_chrdev_region(g_CAM_CALdevno, 1, CAM_CAL_DRVNAME))
    {
        LOG_INF("[S5K3H7YX_CAM_CAL] Register device no failed\n");
        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();
    if (NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);
        LOG_INF("[S5K3H7YX_CAM_CAL] Allocate mem for kobject failed\n");
        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);
    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if (cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        LOG_INF("[S5K3H7YX_CAM_CAL] Attatch file operation failed\n");
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
    .probe = CAM_CAL_probe,
    .remove = CAM_CAL_remove,
    .driver = {
        .name = CAM_CAL_DRVNAME,
        .owner = THIS_MODULE,
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
    LOG_INF("[S5K3H7YX_CAM_CAL]\n");
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();
    if (i4RetValue){
        LOG_INF("[S5K3H7YX_CAM_CAL] register char device failed!\n");
        return i4RetValue;
    }
    LOG_INF("[S5K3H7YX_CAM_CAL] Attached!! \n");

    if (platform_driver_register(&g_stCAM_CAL_Driver)){
        printk("failed to register CAM_CAL driver\n");
        return -ENODEV;
    }
    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        printk("failed to register CAM_CAL device\n");
        return -ENODEV;
    }	
    LOG_INF("[S5K3H7YX_CAM_CAL] Attached Pass !! \n");
    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
    platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);
