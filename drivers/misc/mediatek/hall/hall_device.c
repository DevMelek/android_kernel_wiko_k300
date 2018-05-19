#include <linux/kthread.h>

#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mt-plat/aee.h>
#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#include <linux/sbsuspend.h>	/* smartbook */
#endif
#include <linux/atomic.h>

#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/kobject.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/types.h>


#ifdef CONFIG_TINNO_PRODUCT_INFO 
#include <dev_info.h>
#endif

#define EN_DEBUG

#if defined(EN_DEBUG)
#define TRACE_FUNC 	printk("[hall_dev] function: %s, line: %d \n", __func__, __LINE__);
#define HALL_DEBUG  printk
#else
#define TRACE_FUNC(x,...)
#define HALL_DEBUG(x,...)
#endif

#define  HALL_CLOSE  0
#define  HALL_OPEN    1

/****************************************************************/
/*******static function defination                             **/
/****************************************************************/

static struct device *hall_nor_device = NULL;
static struct input_dev *hall_input_dev;
static int cur_hall_status = HALL_OPEN;
static int hall_key_event = 0;
static int g_hall_first = 1;
static struct wake_lock hall_key_lock;
static int hall_irq;
static struct pinctrl *hallpinctrl = NULL;
static irqreturn_t hall_eint_func(int irq,void *data);
static atomic_t send_event_flag = ATOMIC_INIT(0);
static DECLARE_WAIT_QUEUE_HEAD(send_event_wq);
static int hall_probe(struct platform_device *pdev);
static int hall_remove(struct platform_device *dev);
static int sendKeyEvent(void *unuse);
static struct task_struct *kr_monior_thread = NULL;
static ssize_t hall_enable_read(struct device_driver *ddri, char *buf);

/****************************************************************/
/*******export function defination                             **/
/****************************************************************/
struct of_device_id hall_of_match[] = {
    { .compatible = "mediatek, hall", },
    {},
};

static struct platform_driver hall_driver = {
    .probe	= hall_probe,
    .remove = hall_remove,
    .driver = {
        .name = "hall",
        .of_match_table = hall_of_match,	
    },
};

static ssize_t hall_status_info_show(struct device_driver *ddri, char *buf)
{
    HALL_DEBUG("[hall_dev] cur_hall_status=%d\n", cur_hall_status);
    return sprintf(buf, "%d\n", cur_hall_status);
}

static ssize_t hall_status_info_store(struct device_driver * ddri,char * buf,size_t count)
{
    HALL_DEBUG("[hall_dev] %s ON/OFF value = %d:\n ", __func__, cur_hall_status);
    if(sscanf(buf, "%u", &cur_hall_status) != 1) {
        HALL_DEBUG("[hall_dev]: Invalid values\n");
        return -EINVAL;
    }
    return count;
}

int get_hall_enable_state(char *buf, void *args) {
    return hall_status_info_show(NULL, buf);
}

/** request irq */
static int hall_dev_request_irq(void)
{
    int ret = 0;
    u32 ints[2] = {0, 0};
    unsigned int debounce;
    struct device_node *node;

    /** find hall node from dts */
	node = of_find_compatible_node(NULL, NULL, "mediatek, MHALL-eint");
    if (node) {
        /** map irq num from dts config */
        of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
        hall_irq = irq_of_parse_and_map(node, 0);
        HALL_DEBUG("[Hall] irq num = %d\n", hall_irq);
        /** linux api to request irq */
        if (hall_irq <= 0) {
            goto err;
        }
        ret = request_irq(hall_irq, hall_eint_func, IRQF_TRIGGER_NONE, "MHALL-eint", NULL);
        if(ret > 0) {
            goto err;
        }
    }
    else {
        goto err;
    }

    return 0;

err:
    HALL_DEBUG("[Hall] request irq failed!:%d\n", ret);
    return -1;
}

/** init gpio pin ctrl */
int setIntPinConfig(void)
{
    struct pinctrl_state *pins_cfg = NULL;
 
    /** get pin config from dts */
    pins_cfg = pinctrl_lookup_state(hallpinctrl, "pin_cfg");
    if (IS_ERR(pins_cfg)) {
        return -1;
    }

    /** set pin to input mode */
    pinctrl_select_state(hallpinctrl, pins_cfg);
    HALL_DEBUG("[hall_device]%s:%d success!\n",__func__,__LINE__);
    return 0;
}

/** hall irq init */
int hall_enable_set(int enable, void *dev) 
{
    HALL_DEBUG("[hall_dev]:%s:%d,enable:%d\n",__func__,__LINE__,enable);

    if (kr_monior_thread == NULL) {
        /** init kernel wake lock */
        wake_lock_init(&hall_key_lock, WAKE_LOCK_SUSPEND, "hall key wakelock");
        
        /** init sleep queue */
        init_waitqueue_head(&send_event_wq);

        /** create kernel thread loop */
        kr_monior_thread = kthread_run(sendKeyEvent, 0, "key_report_trhead");
        if (IS_ERR(kr_monior_thread)) {
            HALL_DEBUG("[hall_dev]:%s:%d kthread_run err!%d\n",__func__,__LINE__);
            goto err;
        } 
        
        /** request irq */
        if (hall_dev_request_irq() < 0) {
            goto err;
        }
    }

    HALL_DEBUG("[hall_dev]:%s:%d init success!\n",__func__,__LINE__);
    return 0;

err:
    HALL_DEBUG("[hall_dev]:%s:%d init failed!\n",__func__,__LINE__);
    return -1;
}


/* This function called by init.rc:
   on property:ro.feature.leather=true
       write /sys/bus/platform/drivers/hall/hall_enable 1
*/
static ssize_t hall_enable_store(struct device_driver * ddri, char * buf, size_t count)
{
    HALL_DEBUG("[hall_dev]:%s:%d,enable:%s\n",__func__,__LINE__,buf);
    if (buf[0] == '1') {
        hall_enable_set(1, ddri);
    } 
    return count;
}

static ssize_t hall_enable_read(struct device_driver *ddri, char *buf)
{
    if (kr_monior_thread != NULL) {
        return sprintf(buf, "%d\n", cur_hall_status);
    }
    else {
        return sprintf(buf, "%s\n", "-1");
    }
}

static DEVICE_ATTR(hall_enable, 0664, hall_enable_read,  hall_enable_store);
static DEVICE_ATTR(hall_state, 0664, hall_status_info_show,  hall_status_info_store);

static int sendKeyEvent(void *unuse)
{
    while(1)
    {
        HALL_DEBUG("[hall_dev]:sendKeyEvent wait\n");
        enable_irq(hall_irq);
        //wait for signal
        wait_event_interruptible(send_event_wq, (atomic_read(&send_event_flag) != 0));
        wake_lock_timeout(&hall_key_lock, 2*HZ);    //set the wake lock.
        HALL_DEBUG("[hall_dev]:going to send event %d\n", hall_key_event);
        disable_irq(hall_irq);
        //send key event
        if (HALL_OPEN == hall_key_event) {
            HALL_DEBUG("[hall_dev]:HALL_OPEN!\n");
            input_report_key(hall_input_dev, KEY_HALLOPEN, 1);
            input_report_key(hall_input_dev, KEY_HALLOPEN, 0);
            input_sync(hall_input_dev);
        }
        else if (HALL_CLOSE == hall_key_event) {
            HALL_DEBUG("[hall_dev]:HALL_CLOSE!\n");
            input_report_key(hall_input_dev, KEY_HALLCLOSE, 1);
            input_report_key(hall_input_dev, KEY_HALLCLOSE, 0);
            input_sync(hall_input_dev);
        }
        atomic_set(&send_event_flag, 0);
    }
    return 0;
}

static ssize_t notify_sendKeyEvent(int event)
{
    hall_key_event = event;
    atomic_set(&send_event_flag, 1);
    wake_up(&send_event_wq);
    HALL_DEBUG("[hall_dev]:notify_sendKeyEvent !\n");
    return 0;
}

static irqreturn_t hall_eint_func(int irq,void *data)
{
    int ret=0;

    HALL_DEBUG("hall_eint_func \n");	
    if(cur_hall_status == HALL_CLOSE) {
        HALL_DEBUG("hall_eint_func  HALL_OPEN\n");
        notify_sendKeyEvent(HALL_OPEN);
        irq_set_irq_type(hall_irq,IRQ_TYPE_LEVEL_LOW);
        /* update the eint status */
        cur_hall_status = HALL_OPEN;
    } 
    else {
        HALL_DEBUG("hall_eint_func  HALL_CLOSE\n");
        notify_sendKeyEvent(HALL_CLOSE);
        irq_set_irq_type(hall_irq,IRQ_TYPE_LEVEL_HIGH);
        cur_hall_status = HALL_CLOSE;
    }
    return IRQ_HANDLED;
}

static int hall_probe(struct platform_device *pdev)
{
    int ret = 0;

    hallpinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(hallpinctrl)) {
        HALL_DEBUG("hall_probe   Cannot find hall pinctrl!");
        hallpinctrl = NULL;
        goto error;
    }

    hall_input_dev = input_allocate_device();
    if (hall_input_dev == NULL) {
        HALL_DEBUG("[hall_dev]:hall_input_dev : fail!\n");
        goto error;
    }

    __set_bit(EV_KEY, hall_input_dev->evbit);
    __set_bit(KEY_HALLOPEN, hall_input_dev->keybit);
    __set_bit(KEY_HALLCLOSE, hall_input_dev->keybit);

    hall_input_dev->id.bustype = BUS_HOST;
    hall_input_dev->name = "HALL_DEV";
    if(input_register_device(hall_input_dev)) {
        HALL_DEBUG("[hall_dev]:hall_input_dev register : fail!\n");
        goto error;
    }

    if (setIntPinConfig() < 0) {
        HALL_DEBUG("[hall_dev]:setIntPinConfig failed!\n");
        goto error;
    }
    ret = driver_create_file(&hall_driver.driver, &dev_attr_hall_state.attr);
    ret = driver_create_file(&hall_driver.driver, &dev_attr_hall_enable.attr);
    if (ret) {
        HALL_DEBUG("[hall_dev]:%s: sysfs_create_file failed\n", __func__);
        goto error;
    }

#ifdef CONFIG_TINNO_PRODUCT_INFO 
    FULL_PRODUCT_DEVICE_CB(ID_HALL, get_hall_enable_state, NULL);
#endif
    return 0;

error:
    if (hall_input_dev != NULL) {
        kfree(hall_input_dev);
        hall_input_dev = NULL;
    }
    if (hallpinctrl != NULL) {
        devm_pinctrl_put(hallpinctrl);
        hallpinctrl = NULL;
    }

    HALL_DEBUG("[hall_dev]:hall probe failed!\n");
    return -1;
}

static int hall_remove(struct platform_device *dev)	
{
	input_unregister_device(hall_input_dev);
    if (hall_input_dev != NULL) {
        kfree(hall_input_dev);
        hall_input_dev = NULL;
    }
    if (hallpinctrl != NULL) {
        devm_pinctrl_put(hallpinctrl);
        hallpinctrl = NULL;
    }
    return 0;
}

static int __init hall_init(void)
{
#ifdef CONFIG_TINNO_PERSO_SENSORS
    extern int g_iHall_sensor;
    printk(KERN_INFO "Tinnod Board Config, g_iHall_sensor = %d\n", g_iHall_sensor);
    if (g_iHall_sensor == 0)
    {
        HALL_DEBUG("[hall_dev]: g_iHall_sensor == 0! \n");
        return 0;
    }
#endif
    platform_driver_register(&hall_driver);
    return 0;
}

static void __exit hall_exit(void)
{
    platform_driver_unregister(&hall_driver);
}

module_init(hall_init);
module_exit(hall_exit);
MODULE_DESCRIPTION("HALL DEVICE driver");
MODULE_AUTHOR("liling <ling.li@tinno.com>");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("halldevice");

