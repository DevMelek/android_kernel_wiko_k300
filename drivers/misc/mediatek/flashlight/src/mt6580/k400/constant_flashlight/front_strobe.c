#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
#include <strobe_infos.h>

static DEFINE_SPINLOCK(g_strobeSMPLock_sub); /* cotta-- SMP proection */

static u32 frontStrobeRes = 0;
static int gFrontDuty = 0;
static int gFrontTimeOutTimeMs = 0;
static struct hrtimer gFrontTimeOutTimer;
static struct work_struct workTimeOut_sub;

extern struct pinctrl *flashlightpinctrl;
extern struct pinctrl_state *flashlight_en_h ;
extern struct pinctrl_state *flashlight_en_l ;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
	static DEFINE_MUTEX(g_strobeSem_sub);
#else
	static DECLARE_MUTEX(g_strobeSem_sub);
#endif

/*** IFlash(mA) ~= (int)Value * 11.72mA + 11.35mA ****
**** ITorch(mA) ~= (int)Value * 2.91mA + 2.55mA   ****/
static int frontFlashDuty[4]= {5,6,7,8};//70mA,82mA,93mA,105mA
extern int LM3643_write_reg_ext(u8 reg, u8 val);
extern int readReg(int reg);

static int FL_Enable(void)
{
	PK_DBG(" FL_Enable sub line=%d\n",__LINE__);
	PK_DBG("FL_enable sub :g_duty=%d \n", gFrontDuty);

	pinctrl_select_state(flashlightpinctrl, flashlight_en_h);
	mdelay(20);

	LM3643_write_reg_ext(LM3643_REG_BOOST_CONFIG, 0x89);
	LM3643_write_reg_ext(LM3643_REG_TIMING, 0x1F);

	int i = 0;

	if (gFrontDuty == 0) {
		LM3643_write_reg_ext(LM3643_REG_TORCH_LED1, 0x00);
		LM3643_write_reg_ext(LM3643_REG_TORCH_LED2, 0x20);//100mA
		LM3643_write_reg_ext(LM3643_REG_ENABLE, 0x0a);
		PK_DBG("FL_Enable sub line=%d\n", __LINE__);
	} else {
		gFrontDuty = gFrontDuty > sizeof(frontFlashDuty)/sizeof(int) ? sizeof(frontFlashDuty)/sizeof(int)-1 : gFrontDuty;
		LM3643_write_reg_ext(LM3643_REG_FLASH_LED1, 0x00);
		LM3643_write_reg_ext(LM3643_REG_FLASH_LED2, frontFlashDuty[gFrontDuty]/*0x08*/);
		LM3643_write_reg_ext(LM3643_REG_ENABLE, 0x0e);
		PK_DBG("FL_Enable frontFlashDuty[gFrontDuty]=%d\n", frontFlashDuty[gFrontDuty]);
		PK_DBG("FL_Enable sub line=%d\n", __LINE__);
	}

	return 0;
}

static int FL_Disable(void)
{
	pinctrl_select_state(flashlightpinctrl, flashlight_en_h);
    	mdelay(20);

	pinctrl_select_state(flashlightpinctrl, flashlight_en_l);
    	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    	return 0;
}
static int FL_dim_duty(kal_uint32 duty)
{
    	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    	gFrontDuty =  duty;
    	return 0;
}

static int FL_Init(void)
{
	pinctrl_select_state(flashlightpinctrl, flashlight_en_l);
    	PK_DBG(" FL_Init line=%d\n",__LINE__);
    	return 0;
}

static int FL_Uninit(void)
{
    FL_Disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static void workTimeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
}

static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut_sub);
    return HRTIMER_NORESTART;
}

static void timerInit(void)
{
    INIT_WORK(&workTimeOut_sub, workTimeOutFunc);
    gFrontTimeOutTimeMs=1000;
    hrtimer_init( &gFrontTimeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
    gFrontTimeOutTimer.function=ledTimeOutCallback;
}

static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	PK_DBG("sub dummy ioctl");
    int i4RetValue = 0;
    int ior_shift;
    int iow_shift;
    int iowr_shift;
    ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
    iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
    iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
    PK_DBG("RT4505 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%ld\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

        case FLASH_IOC_SET_TIME_OUT_TIME_MS:
            PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %ld\n", arg);
            gFrontTimeOutTimeMs = arg;
	     PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", arg);
	     gFrontTimeOutTimeMs = arg;
            break;

        case FLASH_IOC_SET_DUTY :
            PK_DBG("FLASHLIGHT_DUTY: %ld\n",arg);
            FL_dim_duty(arg);
            break;

        case FLASH_IOC_SET_STEP:
            PK_DBG("FLASH_IOC_SET_STEP: %ld\n",arg);
            break;

        case FLASH_IOC_SET_ONOFF :
            PK_DBG("FLASHLIGHT_ONOFF: %ld\n",arg);
            if (arg==1)
            {
                if (gFrontTimeOutTimeMs != 0)
                {
                    ktime_t ktime;
                    ktime = ktime_set( 0, gFrontTimeOutTimeMs*1000000);
                    hrtimer_start( &gFrontTimeOutTimer, ktime, HRTIMER_MODE_REL);
                }
                FL_Enable();
            }
            else
            {
                FL_Disable();
                hrtimer_cancel(&gFrontTimeOutTimer);
            }
            break;
        default :
            PK_DBG("No such command \n");
            i4RetValue = -EPERM;
            break;
    }
    return i4RetValue;
}

static int sub_strobe_open(void *pArg)
{
    PK_DBG("sub dummy open");

    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    if (0 == frontStrobeRes)
    {
        FL_Init();
        timerInit();
    }
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
    spin_lock_irq(&g_strobeSMPLock_sub);

    if (frontStrobeRes)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        frontStrobeRes += 1;
    }
    spin_unlock_irq(&g_strobeSMPLock_sub);

    return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
    PK_DBG("sub dummy release");

    if (frontStrobeRes)
    {
        spin_lock_irq(&g_strobeSMPLock_sub);

        frontStrobeRes = 0;

        spin_unlock_irq(&g_strobeSMPLock_sub);

        FL_Uninit();
    }

    return 0;
}

FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};

MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}
