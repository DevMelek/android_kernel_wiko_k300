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

#include "s5k4h8mipirawSensor_holitech.h"

#define LOG_INF(format, args...)	printk("[%s]"format, __FUNCTION__, ##args)

#define RG_Ratio_Typical		597
#define BG_Ratio_Typical		720
#define GROUP_EMPTY         0x00
#define GROUP_1_VALID       0x01
#define GROUP_2_VALID       0x13
#define GROUP_INVALID       0x37
#define OTP_MID_VALID       0x42

struct Holitech OTPInfos;

extern void S5K4H8Hol_bytewrite_cmos_sensor(u16 addr, kal_uint16 para);
extern kal_uint16 S5K4H8Hol_byteread_cmos_sensor(u16 addr);

int s5k4h8Holitech_read_otp(void)
{
    int basicFlag, AFFlag, ret = 0;
    u16 startAddr;

    S5K4H8Hol_bytewrite_cmos_sensor(0x0a02, 0x0f); //page 15
    S5K4H8Hol_bytewrite_cmos_sensor(0x0a00, 0x01); //Read enable
    mdelay(10);

    basicFlag = S5K4H8Hol_byteread_cmos_sensor(0x0a04); // OTP WB Calibration
#if 1
    if (basicFlag == GROUP_1_VALID) {
        startAddr = 0x0a05;
    } else if (basicFlag == GROUP_2_VALID) {
        startAddr = 0x0a14;
    }

    LOG_INF("basicFlag=%d\n", basicFlag);
    switch (basicFlag) {
        case GROUP_1_VALID:
        case GROUP_2_VALID:
            OTPInfos.flag = 0x01;
            OTPInfos.mid = S5K4H8Hol_byteread_cmos_sensor(startAddr);
            LOG_INF("mid=0x%x\n", OTPInfos.mid);
            if (OTPInfos.mid == OTP_MID_VALID) {
                OTPInfos.year = S5K4H8Hol_byteread_cmos_sensor(startAddr + 1);
                OTPInfos.month = S5K4H8Hol_byteread_cmos_sensor(startAddr + 2);
                OTPInfos.day = S5K4H8Hol_byteread_cmos_sensor(startAddr + 3);
                OTPInfos.lens_id = S5K4H8Hol_byteread_cmos_sensor(startAddr + 4);
                OTPInfos.vcm_id = S5K4H8Hol_byteread_cmos_sensor(startAddr + 5);
                OTPInfos.driver_ic_id = S5K4H8Hol_byteread_cmos_sensor(startAddr + 6);
                OTPInfos.sensor_id = S5K4H8Hol_byteread_cmos_sensor(startAddr + 7);
                OTPInfos.rg_ratio = (S5K4H8Hol_byteread_cmos_sensor(startAddr + 8)<<8) + S5K4H8Hol_byteread_cmos_sensor(startAddr + 9);
                OTPInfos.bg_ratio = (S5K4H8Hol_byteread_cmos_sensor(startAddr + 10)<<8) + S5K4H8Hol_byteread_cmos_sensor(startAddr + 11);
                OTPInfos.gbgr_ratio = (S5K4H8Hol_byteread_cmos_sensor(startAddr + 12)<<8) + S5K4H8Hol_byteread_cmos_sensor(startAddr + 13);
                LOG_INF("year=20%d month=%2d day=%2d\n", OTPInfos.year, OTPInfos.month, OTPInfos.day);
                LOG_INF("rg_ratio=%d bg_ratio=%d\n", OTPInfos.rg_ratio, OTPInfos.bg_ratio);
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
        AFFlag = S5K4H8Hol_byteread_cmos_sensor(0x0a23); // OTP AF Calibration
        if (AFFlag == GROUP_1_VALID) {
            startAddr = 0x0a24;
        } else if (AFFlag == GROUP_2_VALID){
            startAddr = 0x0a29;
        }
        LOG_INF("AFFlag=%d\n", AFFlag);
        switch (AFFlag) {
            case GROUP_1_VALID:
            case GROUP_2_VALID:
                OTPInfos.flag = OTPInfos.flag | 0x02;
                OTPInfos.AFInfinity = (S5K4H8Hol_byteread_cmos_sensor(startAddr)<<8) + S5K4H8Hol_byteread_cmos_sensor(startAddr + 1);
                OTPInfos.AFMacro = (S5K4H8Hol_byteread_cmos_sensor(startAddr + 2)<<8) + S5K4H8Hol_byteread_cmos_sensor(startAddr + 3);
                LOG_INF("AFInfinity=%d AFMacro=%d\n", OTPInfos.AFInfinity, OTPInfos.AFMacro);
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
    if (basicFlag == GROUP_1_VALID) {
        OTPInfos.flag = 0x01;
        OTPInfos.mid = S5K4H8Hol_byteread_cmos_sensor(0x0a05);
        OTPInfos.year = S5K4H8Hol_byteread_cmos_sensor(0x0a06);
        OTPInfos.month = S5K4H8Hol_byteread_cmos_sensor(0x0a07);
        OTPInfos.day = S5K4H8Hol_byteread_cmos_sensor(0x0a08);
        OTPInfos.lens_id = S5K4H8Hol_byteread_cmos_sensor(0x0a09);
        OTPInfos.vcm_id = S5K4H8Hol_byteread_cmos_sensor(0x0a0a);
        OTPInfos.driver_ic_id = S5K4H8Hol_byteread_cmos_sensor(0x0a0b);
        OTPInfos.sensor_id = S5K4H8Hol_byteread_cmos_sensor(0x0a0c);
        OTPInfos.rg_ratio = (S5K4H8Hol_byteread_cmos_sensor(0x0a0D)<<8) + S5K4H8Hol_byteread_cmos_sensor(0x0a0E);
        OTPInfos.bg_ratio = (S5K4H8Hol_byteread_cmos_sensor(0x0a0F)<<8) + S5K4H8Hol_byteread_cmos_sensor(0x0a10);
        OTPInfos.gbgr_ratio = (S5K4H8Hol_byteread_cmos_sensor(0x0a11)<<8) + S5K4H8Hol_byteread_cmos_sensor(0x0a12);
        LOG_INF("basicFlag=0x01 rg_ratio=%d bg_ratio=%d\n", OTPInfos.rg_ratio, OTPInfos.bg_ratio);
    } else if (basicFlag == GROUP_2_VALID) {
        OTPInfos.flag = 0x01;
        OTPInfos.mid = S5K4H8Hol_byteread_cmos_sensor(0x0a14);
        OTPInfos.year = S5K4H8Hol_byteread_cmos_sensor(0x0a15);
        OTPInfos.month = S5K4H8Hol_byteread_cmos_sensor(0x0a16);
        OTPInfos.day = S5K4H8Hol_byteread_cmos_sensor(0x0a17);
        OTPInfos.lens_id = S5K4H8Hol_byteread_cmos_sensor(0x0a18);
        OTPInfos.vcm_id = S5K4H8Hol_byteread_cmos_sensor(0x0a19);
        OTPInfos.driver_ic_id = S5K4H8Hol_byteread_cmos_sensor(0x0a1a);
        OTPInfos.sensor_id = S5K4H8Hol_byteread_cmos_sensor(0x0a1b);
        OTPInfos.rg_ratio = (S5K4H8Hol_byteread_cmos_sensor(0x0a1C)<<8) + S5K4H8Hol_byteread_cmos_sensor(0x0a1D);
        OTPInfos.bg_ratio = (S5K4H8Hol_byteread_cmos_sensor(0x0a1E)<<8) + S5K4H8Hol_byteread_cmos_sensor(0x0a1F);
        OTPInfos.gbgr_ratio = (S5K4H8Hol_byteread_cmos_sensor(0x0a20)<<8) + S5K4H8Hol_byteread_cmos_sensor(0x0a21);
        LOG_INF("basicFlag=0x13 rg_ratio=%d bg_ratio=%d\n", OTPInfos.rg_ratio, OTPInfos.bg_ratio);
    }
    // OTP AF Calibration
    AFFlag = S5K4H8Hol_byteread_cmos_sensor(0x0a23);
    if (AFFlag == GROUP_1_VALID) {
        OTPInfos.flag = OTPInfos.flag | 0x02;
        OTPInfos.AFInfinity = (S5K4H8Hol_byteread_cmos_sensor(0x0a24)<<8) + S5K4H8Hol_byteread_cmos_sensor(0x0a25);
        OTPInfos.AFMacro = (S5K4H8Hol_byteread_cmos_sensor(0x0a26)<<8) + S5K4H8Hol_byteread_cmos_sensor(0x0a27);
        LOG_INF("AFFlag=0x01 AFInfinity=%d AFMacro=%d\n", OTPInfos.AFInfinity, OTPInfos.AFMacro);
    } else if (AFFlag == GROUP_2_VALID) {
        OTPInfos.flag = OTPInfos.flag | 0x02;
        OTPInfos.AFInfinity = (S5K4H8Hol_byteread_cmos_sensor(0x0a29)<<8) + S5K4H8Hol_byteread_cmos_sensor(0x0a2a);
        OTPInfos.AFMacro = (S5K4H8Hol_byteread_cmos_sensor(0x0a2b)<<8) + S5K4H8Hol_byteread_cmos_sensor(0x0a2c);
        LOG_INF("AFFlag=0x13 AFInfinity=%d AFMacro=%d\n", OTPInfos.AFInfinity, OTPInfos.AFMacro);
    }
    LOG_INF("basicFlag=0x%2x AFFlag=0x%2x mid=0x%x year=20%d month=%2d day=%2d\n", basicFlag, AFFlag, OTPInfos.mid, OTPInfos.year, OTPInfos.month, OTPInfos.day);
#endif
    S5K4H8Hol_bytewrite_cmos_sensor(0x0a00, 0x01);  //Stop read
    LOG_INF("ret == %d\n", ret);
    return ret;
}

void s5k4h8Holitech_apply_awb(void)
{
    int rg, bg, R_gain, G_gain, B_gain, Base_gain;
    LOG_INF("===>>>flag = 0x%2x\n", OTPInfos.flag);
    // Apply OTP WB Calibration
    if (OTPInfos.flag == 0x03) {
        LOG_INF("apply, rg_ratio=%d bg_ratio=%d\n", OTPInfos.rg_ratio, OTPInfos.bg_ratio);
        rg = OTPInfos.rg_ratio;
        bg = OTPInfos.bg_ratio;
        //calculate G gain
        R_gain = RG_Ratio_Typical * 256 / rg;
        B_gain = BG_Ratio_Typical * 256 / bg;
        G_gain = 256;
        if (R_gain < 256 || B_gain < 256)
        {
		if (R_gain < B_gain)
			Base_gain = R_gain;
		else
			Base_gain = B_gain;
        }
        else
        {
        	Base_gain = G_gain;
        }
        R_gain = 256 * R_gain / (Base_gain);
        B_gain = 256 * B_gain / (Base_gain);
        G_gain = 256 * G_gain / (Base_gain);
        LOG_INF("R_gain=%d B_gain=%d G_gain=%d\n", R_gain, B_gain, G_gain);
        // Update sensor WB gain
        if (R_gain > 256) {
        	S5K4H8Hol_bytewrite_cmos_sensor(0x210, R_gain >> 8);
        	S5K4H8Hol_bytewrite_cmos_sensor(0x211, R_gain & 0x00ff);
        }
        if (G_gain > 256) {
        	S5K4H8Hol_bytewrite_cmos_sensor(0x20e, G_gain >> 8);
        	S5K4H8Hol_bytewrite_cmos_sensor(0x20f, G_gain & 0x00ff);
        	S5K4H8Hol_bytewrite_cmos_sensor(0x214, G_gain >> 8);
        	S5K4H8Hol_bytewrite_cmos_sensor(0x215, G_gain & 0x00ff);
        }
        if (B_gain > 256) {
        	S5K4H8Hol_bytewrite_cmos_sensor(0x212, B_gain >> 8);
        	S5K4H8Hol_bytewrite_cmos_sensor(0x213, B_gain & 0x00ff);
        }
    }else {
	  LOG_INF("s5k4h8Holitech OTP apply Error!!!\n");
    }
}

/*************************************************************************************************
* Function : otp_update_wb()
* Description : update otp_sunny data from otp_sunny , it otp_sunny data is valid,
it include get ID and WB update function
* Return : [bool] 0 : update fail
1 : update success
**************************************************************************************************/
void HolitechOTPUpdateAWB(void)
{
	LOG_INF("Holitech statrt read otp  \n");
	s5k4h8Holitech_read_otp();
	LOG_INF("statrt write otp  \n");
	s5k4h8Holitech_apply_awb();
	LOG_INF("write otp end \n");
}

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "S5K4H8HOLITECH_CAM_CAL_DRV"
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
u8 Holitech_CheckID[]= {0x10,0xcc,0x00,0x40,0x88}; //0x884000cc
static int selective_read_region(u32 offset, BYTE* data,u16 i2c_id,u32 size)
{    	
    printk("[s5k4h8_CAM_CAL] selective_read_region offset =%d size %d data read = %d\n", offset,size, *data);

	if (size == 2) {
		if (offset == 7) {
			memcpy((void *)data, OTPInfos.AFMacro, size);
		} else if (offset == 9) {		 
			memcpy((void *)data, OTPInfos.AFInfinity,size);
		}
	}

	if (size == 4) {
		memcpy((void *)data,(void *)&Holitech_CheckID[1],size);
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
