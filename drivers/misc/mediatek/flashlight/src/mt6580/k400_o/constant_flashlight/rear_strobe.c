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
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include "kd_camera_typedef.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>
#include <strobe_infos.h>

/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */

static int gRearDuty = 0;
static u32 strobe_Res = 0;
static int g_timeOutTimeMs = 0;

static struct work_struct workTimeOut;
static struct i2c_client *LM3643_i2c_client;

/*** IFlash(mA) ~= (int)Value * 11.72mA + 11.35mA ****
**** ITorch(mA) ~= (int)Value * 2.91mA + 2.55mA   ****/
static int rearFlashDuty[12] = {42,42,46,50,55,59,63,67,72,76,80,84};//503,503,550,597,656,703,750,797,855,902,949,996mA
/*****************************************************************************
  Functions
 *****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

struct LM3643_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct LM3643_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int LM3643_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	//	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	//	mutex_unlock(&chip->lock);

	return val;
}

int readReg(int reg)
{
	int val;
	val = LM3643_read_reg(LM3643_i2c_client, reg);
	return (int)val;
}

static int LM3643_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	printk("LM3643_write_reg reg = 0x%02x val = %d\n", reg, val);
	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writing at 0x%02x\n", reg);
	return ret;
}

int writeReg(int reg, int data)
{
	char buf[2];
	buf[0]=reg;
	buf[1]=data;

	iWriteRegI2C(buf, 2, STROBE_DEVICE_ID);
	return 0;
}

int init_LM3643(void)
{
	int err;
	err =  writeReg(LM3643_REG_ENABLE, 0x00);
	err =  writeReg(LM3643_REG_TIMING, 0x1F);
	return err;
}

struct LM3643_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

int LM3643_write_reg_ext(u8 reg, u8 val)
{
	return LM3643_write_reg(LM3643_i2c_client, reg, val);
}

int LM3643_read_reg_ext(u8 reg)
{
	return LM3643_read_reg(LM3643_i2c_client, reg);
}

static int LM3643_chip_init(struct LM3643_chip_data *chip)
{
	return 0;
}

static int LM3643_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct LM3643_chip_data *chip;
	struct LM3643_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	printk("Farewell LM3643_probe start--->.\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("LM3643 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3643_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("AW3643 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3643_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (LM3643_chip_init(chip) < 0)
		goto err_chip_init;

	LM3643_i2c_client = client;
	printk("Farewell LM3643 Initializing is done\n");
	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	printk("Farewell LM3643 probe is failed\n");
	return -ENODEV;
}

static int LM3643_remove(struct i2c_client *client)
{
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}

#define LM3643_NAME "leds-LM3643"
static const struct i2c_device_id LM3643_id[] = {
	{LM3643_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id LM3643_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver LM3643_i2c_driver = {
	.driver = {
		.name = LM3643_NAME,
#ifdef CONFIG_OF
		.of_match_table = LM3643_of_match,
#endif
	},
	.probe = LM3643_probe,
	.remove = LM3643_remove,
	.id_table = LM3643_id,
};
static int __init LM3643_init(void)
{
	printk("Farewell LM3643_init\n");
	return i2c_add_driver(&LM3643_i2c_driver);
}

static void __exit LM3643_exit(void)
{
	i2c_del_driver(&LM3643_i2c_driver);
}

module_init(LM3643_init);
module_exit(LM3643_exit);

MODULE_DESCRIPTION("Flash driver for AW3643");
MODULE_AUTHOR("Farewell <kuanxiong.chen@tinno.com>");
MODULE_LICENSE("GPL v2");

#ifdef CONFIG_OF
static const struct of_device_id Flashlight_use_gpio_of_match[] = {
	{.compatible = "mediatek,strobe_gpio_main"},
	{},
};
#endif

struct pinctrl *flashlightpinctrl = NULL;
struct pinctrl_state *flashlight_en_h = NULL;
struct pinctrl_state *flashlight_en_l = NULL;

static int Flashlight_use_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct task_struct *keyEvent_thread = NULL;
	printk("Flashlight_use_gpio_probe enter \n");


	flashlightpinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flashlightpinctrl)) {
		PK_DBG("IS_ERR(flashlightpinctrl) \n");
		return -1;	
	}

	flashlight_en_l= pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_en0");
	if (IS_ERR(flashlight_en_l)) {
		PK_DBG("IS_ERR(flashlight_en_l) \n");
		return -1;	
	}
	flashlight_en_h= pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_en1");
	if (IS_ERR(flashlight_en_h)) {
		PK_DBG("IS_ERR(flashlight_en_h) \n");
		return -1;	
	}

	printk("Flashlight_use_gpio_probe exit\n");

	return 0;
}

static int Flashlight_use_gpio_remove(struct platform_device *dev)	
{
	return 0;
}

static struct platform_driver Flashlight_use_gpio_driver = {
	.probe	= Flashlight_use_gpio_probe,
	.remove  = Flashlight_use_gpio_remove,
	.driver    = {
		.name = "flashlight",
		.of_match_table = Flashlight_use_gpio_of_match,	
	},
};

static int __init Flashlight_use_gpio_init(void)
{
	printk("Flashlight_use_gpio_init\n");
	platform_driver_register(&Flashlight_use_gpio_driver);
}

static void __exit Flashlight_use_gpio_exit(void)
{
	printk("Flashlight_use_gpio_exit\n");
	platform_driver_unregister(&Flashlight_use_gpio_driver);
}

module_init(Flashlight_use_gpio_init);
module_exit(Flashlight_use_gpio_exit);

MODULE_DESCRIPTION("Flash driver for GPIO flashlight");
MODULE_AUTHOR("Farewell <kuanxiong.chen@tinno.com>");
MODULE_LICENSE("GPL v2");

int FL_Enable(void)
{
	int compatile_id = 0;

	PK_DBG("YCC FL_enable :gRearDuty=%d \n", gRearDuty);
	PK_DBG(" FL_Enable line=%d\n",__LINE__);

	pinctrl_select_state(flashlightpinctrl, flashlight_en_h);

	LM3643_write_reg(LM3643_i2c_client, LM3643_REG_BOOST_CONFIG, 0x89);
	// LEO-TEST  for 400ms timeout 
	LM3643_write_reg(LM3643_i2c_client, LM3643_REG_TIMING, 0x1F);
	compatile_id = readReg(LM3643_REG_COMPATIB);
	PK_DBG(" FL_Enable compatile_id = %2x\n",compatile_id);
	if (0x12 == compatile_id) //0x12 is AW3643 device id
	{
		PK_DBG(" FL_Enable line=%d\n",__LINE__);
		if (gRearDuty == 0) {
			PK_DBG(" FL_Enable line=%d\n",__LINE__);
			LM3643_write_reg(LM3643_i2c_client, LM3643_REG_TORCH_LED2, 0x00);
			LM3643_write_reg(LM3643_i2c_client, LM3643_REG_TORCH_LED1, 0x33);//150mA
			LM3643_write_reg(LM3643_i2c_client, LM3643_REG_ENABLE, 0x09);
		} else {
			gRearDuty = gRearDuty > sizeof(rearFlashDuty)/sizeof(int) ? sizeof(rearFlashDuty)/sizeof(int)-1 : gRearDuty;
			PK_DBG(" FL_Enable line=%d\n",__LINE__);
			LM3643_write_reg(LM3643_i2c_client, LM3643_REG_FLASH_LED2, 0x00);
			LM3643_write_reg(LM3643_i2c_client, LM3643_REG_FLASH_LED1, rearFlashDuty[gRearDuty]);
			LM3643_write_reg(LM3643_i2c_client, LM3643_REG_ENABLE, 0x0D);
			PK_DBG("FL_Enable rearFlashDuty[gRearDuty]=%d\n", rearFlashDuty[gRearDuty]);
			PK_DBG("FL_Enable rear line=%d\n", __LINE__);
		}
	}

	PK_DBG("exit FL_Enable line=%d\n",__LINE__);
	return 0;
}

int FL_Disable(void)
{
	pinctrl_select_state(flashlightpinctrl, flashlight_en_l);
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	PK_DBG(" FL_dim_duty duty=%d\n", duty);

	gRearDuty = duty;
	return 0;
}

int FL_Init(void)
{
	pinctrl_select_state(flashlightpinctrl, flashlight_en_l);

	PK_DBG("enter  FL_Init line=%d\n", __LINE__);
	PK_DBG("enter  FL_Init line=%d\n", __LINE__);

	INIT_WORK(&workTimeOut, work_timeOutFunc);
	PK_DBG("out  FL_Init line=%d\n",__LINE__);
	return 0;
}

int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
  User interface
 *****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;

void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}

static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("LM3643 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
	switch(cmd)
	{
		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", arg);
			g_timeOutTimeMs=arg;
			break;

		case FLASH_IOC_SET_DUTY :
			PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
			FL_dim_duty(arg);
			break;

		case FLASH_IOC_SET_STEP:
			PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);
			break;

		case FLASH_IOC_SET_ONOFF :
			PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
			if (arg==1)
			{
				if (g_timeOutTimeMs!=0)
				{
					ktime_t ktime;
					ktime = ktime_set(0, g_timeOutTimeMs*1000000);
					hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL );
				}
				FL_Enable();
			} else {
				FL_Disable();
				hrtimer_cancel(&g_timeOutTimer);
			}
			break;
		case FLASH_IOC_SET_REG_ADR:
			break;
		case FLASH_IOC_SET_REG_VAL:
			break;
		case FLASH_IOC_SET_REG:
			break;
		case FLASH_IOC_GET_REG:
			break;

		default :
			PK_DBG(" No such command \n");
			i4RetValue = -EPERM;
			break;
	}
	return i4RetValue;
}

static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);

	if (strobe_Res)
	{
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}

	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	return i4RetValue;

}

static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res)
	{
		spin_lock_irq(&g_strobeSMPLock);
		strobe_Res = 0;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");
	return 0;
}

FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};

MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
	{
		*pfFunc = &constantFlashlightFunc;
	}
	return 0;
}

/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{
	return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);
