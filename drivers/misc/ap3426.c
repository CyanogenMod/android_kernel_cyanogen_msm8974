/*
 * This file is part of the AP3426, AP3212C and AP3216C sensor driver.
 * AP3426 is combined proximity and ambient light sensor.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 *	    Templeton Tsai <templeton.tsai@dyna-image.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3426.c
 *
 * Summary:
 *	AP3426 device driver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 02/02/12 YC       1. Modify irq function to seperate two interrupt routine. 
 *					 2. Fix the index of reg array error in em write. 
 * 02/22/12 YC       3. Merge AP3426 and AP3216C into the same driver. (ver 1.8)
 * 03/01/12 YC       Add AP3212C into the driver. (ver 1.8)
 * 07/25/14 John	  Ver.2.1 , ported for Nexus 7
 * 08/21/14 Templeton AP3426 Ver 1.0, ported for Nexus 7
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/workqueue.h>
#include <linux/timer.h>

#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#include "ap3426.h"

#define AP3426_DRV_NAME		"ap3426"
#define DRIVER_VERSION		"1"
#define PS_CALIBRATION //ansun add 20150316

#define PL_TIMER_DELAY 2000
#define POLLING_MODE 0

#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("AP3426: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif
static void plsensor_work_handler(struct work_struct *w);
#if POLLING_MODE
static void pl_timer_callback(unsigned long pl_data);
#endif
static int ap3426_set_phthres(struct i2c_client *client, int val);
static int ap3426_set_plthres(struct i2c_client *client, int val);

struct ap3426_data {
    struct i2c_client *client;
    u8 reg_cache[AP3426_NUM_CACHABLE_REGS];//TO-DO
    u8 power_state_before_suspend;
    int irq;
	int irq_gpio;
        struct regulator *vdd;
        struct regulator *vcc_i2c;
    int hsensor_enable;
    struct input_dev	*psensor_input_dev;
    struct input_dev	*lsensor_input_dev;
    struct input_dev	*hsensor_input_dev;
    struct workqueue_struct *plsensor_wq;
    struct work_struct plsensor_work;
#if POLLING_MODE
    struct timer_list pl_timer;
#endif
#ifdef PS_CALIBRATION
		u16 PsCalibration;
		u16 PsLthres;
		u16 PsHthres;
		u8 cali_cmd;
		int cali_status;
#endif
};

static struct ap3426_data *ap3426_data_g = NULL;
// AP3426 register
static u8 ap3426_reg[AP3426_NUM_CACHABLE_REGS] = 
{0x00,0x01,0x02,0x06,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
    0x10,0x14,0x1A,0x1B,0x1C,0x1D,
    0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x28,0x29,0x2A,0x2B,0x2C,0x2D, 0x30, 0x32};

// AP3426 range
static int ap3426_range[4] = {32768,8192,2048,512};



static u8 *reg_array;
static int *range;
static int reg_num = 0;

static int cali = 100;
static int misc_ps_opened = 0;
static int misc_ls_opened = 0;

static DEFINE_MUTEX(ap3426_lock);
static DEFINE_MUTEX(ap3426_ls_lock);
static DEFINE_MUTEX(ap3426_ps_lock);
static DEFINE_MUTEX(ap3426_heartbeat_lock);
#define ADD_TO_IDX(addr,idx)	{														\
    int i;												\
    for(i = 0; i < reg_num; i++)						\
    {													\
	if (addr == reg_array[i])						\
	{												\
	    idx = i;									\
	    break;										\
	}												\
    }													\
}


/*
 * register access helpers
 */

static int __ap3426_read_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    u8 idx = 0xff;

    ADD_TO_IDX(reg,idx)
	return (data->reg_cache[idx] & mask) >> shift;
}

static int __ap3426_write_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift, u8 val)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8 tmp;
    u8 idx = 0xff;

    ADD_TO_IDX(reg,idx)
	if (idx >= reg_num)
	    return -EINVAL;
    mutex_lock(&ap3426_lock);
    tmp = data->reg_cache[idx];
    tmp &= ~mask;
    tmp |= val << shift;

    ret = i2c_smbus_write_byte_data(client, reg, tmp);
    mutex_unlock(&ap3426_lock);
    if (!ret)
	data->reg_cache[idx] = tmp;

    return ret;
}

/*
 * internally used functions
 */

/* range */
static int ap3426_get_range(struct i2c_client *client)
{
    u8 idx = __ap3426_read_reg(client, AP3426_REG_ALS_CONF,
	    AP3426_ALS_RANGE_MASK, AP3426_ALS_RANGE_SHIFT); 
    return range[idx];
}

static int ap3426_set_range(struct i2c_client *client, int range)
{
    return __ap3426_write_reg(client, AP3426_REG_ALS_CONF,
	    AP3426_ALS_RANGE_MASK, AP3426_ALS_RANGE_SHIFT, range);
}


static int ap3426_set_ir_data(struct i2c_client *client, int en)
{
    int ret = 0;

    if(en == 9) {
	ret = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_RESET);
	mdelay(200);
	ret = __ap3426_write_reg(client, AP3426_REG_PS_CONF,
		AP3426_REG_PS_CONF_MASK, AP3426_REG_PS_CONF_SHIFT, 0);
	ret = __ap3426_write_reg(client, AP3426_REG_PS_DC_1,
		AP3426_REG_PS_DC_1_MASK, AP3426_REG_PS_DC_1_SHIFT, 0);
	ret = __ap3426_write_reg(client, AP3426_REG_PS_DC_2,
		AP3426_REG_PS_DC_2_MASK, AP3426_REG_PS_DC_2_SHIFT, 0);
	ret = __ap3426_write_reg(client, AP3426_REG_PS_LEDD,
		AP3426_REG_PS_LEDD_MASK, AP3426_REG_PS_LEDD_SHIFT, 1);
	ret = __ap3426_write_reg(client, AP3426_REG_PS_MEAN,
		AP3426_REG_PS_MEAN_MASK, AP3426_REG_PS_MEAN_SHIFT, 0);
	ret = __ap3426_write_reg(client, AP3426_REG_PS_PERSIS,
		AP3426_REG_PS_PERSIS_MASK, AP3426_REG_PS_PERSIS_SHIFT, 0);
	ret = ap3426_set_plthres(client, 0);
	ret = ap3426_set_phthres(client, 535);
	ret = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_PS_ENABLE);
    }else if(en == 0){
	ret = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_DOWN);
	mdelay(200);
    }

    return ret;
}
/* mode */
static int ap3426_get_mode(struct i2c_client *client)
{
    int ret;

    ret = __ap3426_read_reg(client, AP3426_REG_SYS_CONF,
	    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT);
    return ret;
}

static int ap3426_set_mode(struct i2c_client *client, int mode)
{
    int ret;

    ret = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
	    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, mode);

    return ret;
}

/* ALS low threshold */
static int ap3426_get_althres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_ALS_THDL_L,
	    AP3426_REG_ALS_THDL_L_MASK, AP3426_REG_ALS_THDL_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_ALS_THDL_H,
	    AP3426_REG_ALS_THDL_H_MASK, AP3426_REG_ALS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_althres(struct i2c_client *client, int val)
{

    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_ALS_THDL_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDL_L,
	    AP3426_REG_ALS_THDL_L_MASK, AP3426_REG_ALS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDL_H,
	    AP3426_REG_ALS_THDL_H_MASK, AP3426_REG_ALS_THDL_H_SHIFT, msb);

    return err;
}

/* ALS high threshold */
static int ap3426_get_ahthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_ALS_THDH_L,
	    AP3426_REG_ALS_THDH_L_MASK, AP3426_REG_ALS_THDH_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_ALS_THDH_H,
	    AP3426_REG_ALS_THDH_H_MASK, AP3426_REG_ALS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_ahthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_ALS_THDH_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDH_L,
	    AP3426_REG_ALS_THDH_L_MASK, AP3426_REG_ALS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDH_H,
	    AP3426_REG_ALS_THDH_H_MASK, AP3426_REG_ALS_THDH_H_SHIFT, msb);

    return err;
}

/* PX low threshold */
static int ap3426_get_plthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_PS_THDL_L,
	    AP3426_REG_PS_THDL_L_MASK, AP3426_REG_PS_THDL_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_PS_THDL_H,
	    AP3426_REG_PS_THDL_H_MASK, AP3426_REG_PS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_plthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_PS_THDL_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDL_L,
	    AP3426_REG_PS_THDL_L_MASK, AP3426_REG_PS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDL_H,
	    AP3426_REG_PS_THDL_H_MASK, AP3426_REG_PS_THDL_H_SHIFT, msb);

    return err;
}

/* PX high threshold */
static int ap3426_get_phthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_PS_THDH_L,
	    AP3426_REG_PS_THDH_L_MASK, AP3426_REG_PS_THDH_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_PS_THDH_H,
	    AP3426_REG_PS_THDH_H_MASK, AP3426_REG_PS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_phthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_PS_THDH_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDH_L,
	    AP3426_REG_PS_THDH_L_MASK, AP3426_REG_PS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDH_H,
	    AP3426_REG_PS_THDH_H_MASK, AP3426_REG_PS_THDH_H_SHIFT, msb);

    return err;
}

static int ap3426_get_adc_value(struct i2c_client *client)
{
    unsigned int lsb, msb, val;
#ifdef LSC_DBG
    unsigned int tmp,range;
#endif
    mutex_lock(&ap3426_lock);
    lsb = i2c_smbus_read_byte_data(client, AP3426_REG_ALS_DATA_LOW);
    mutex_unlock(&ap3426_lock);

    if (lsb < 0) {
	return lsb;
    }
    mutex_lock(&ap3426_lock);
    msb = i2c_smbus_read_byte_data(client, AP3426_REG_ALS_DATA_HIGH);
    mutex_unlock(&ap3426_lock);
    if (msb < 0)
	return msb;

    range = ap3426_get_range(client);

    tmp = (((msb << 8) | lsb) * range) >> 16;
    tmp = tmp * cali / 100;
    val = msb << 8 | lsb;
    return val;
}


static int ap3426_get_object(struct i2c_client *client)
{
    int val;
    mutex_lock(&ap3426_lock);
    val = i2c_smbus_read_byte_data(client, AP3426_OBJ_COMMAND);
    val &= AP3426_OBJ_MASK;
    mutex_unlock(&ap3426_lock);
    return val >> AP3426_OBJ_SHIFT;
}

static int ap3426_get_intstat(struct i2c_client *client)
{
    int val;
    mutex_lock(&ap3426_lock);
    val = i2c_smbus_read_byte_data(client, AP3426_REG_SYS_INTSTATUS);
    val &= AP3426_REG_SYS_INT_MASK;
    mutex_unlock(&ap3426_lock);

    return val >> AP3426_REG_SYS_INT_SHIFT;
}


static int ap3426_get_px_value(struct i2c_client *client)
{
    int lsb, msb;
    mutex_lock(&ap3426_lock);
    lsb = i2c_smbus_read_byte_data(client, AP3426_REG_PS_DATA_LOW);
    mutex_unlock(&ap3426_lock);

    if (lsb < 0) {
	return lsb;
    }
    mutex_lock(&ap3426_lock);

    //LDBG("%s, IR = %d\n", __func__, (u32)(lsb));
    msb = i2c_smbus_read_byte_data(client, AP3426_REG_PS_DATA_HIGH);
    mutex_unlock(&ap3426_lock);

    if (msb < 0)
	return msb;

    //LDBG("%s, IR = %d\n", __func__, (u32)(msb));

    return (u32)(((msb & AL3426_REG_PS_DATA_HIGH_MASK) << 8) | (lsb & AL3426_REG_PS_DATA_LOW_MASK));
}




#ifdef CONFIG_HAS_EARLYSUSPEND
static int ap3426_lsensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if((mode & AP3426_SYS_ALS_ENABLE) == 0){
	mode |= AP3426_SYS_ALS_ENABLE;
	ret = ap3426_set_mode(client,mode);
    }

    return ret;
}

static int ap3426_lsensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if(mode & AP3426_SYS_ALS_ENABLE){
	mode &= ~AP3426_SYS_ALS_ENABLE;
	if(mode == AP3426_SYS_DEV_RESET)
	    mode = 0;
	ret = ap3426_set_mode(client,mode);
    }

    return ret;
}
#endif

static int ap3426_register_lsensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device lsensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for lsensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->lsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "lightsensor-level";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_MISC, 0, 8, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for lsensor\n", __FUNCTION__);
	goto done;
    }
done:
    return rc;
}


static void ap3426_unregister_lsensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->lsensor_input_dev);
}
static int ap3426_register_heartbeat_sensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device heartbeat sensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for heartbeat sensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->hsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "heartbeat";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_REL, input_dev->evbit);
    input_set_capability(input_dev, EV_REL, ABS_WHEEL);
    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for heartbeat sensor\n", __FUNCTION__);
	goto done;
    }
done:
    return rc;
}

static void ap3426_unregister_heartbeat_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->hsensor_input_dev);
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static int ap3426_psensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if((mode & AP3426_SYS_PS_ENABLE) == 0){
	mode |= AP3426_SYS_PS_ENABLE;
	ret = ap3426_set_mode(client,mode);
    }

    return ret;
}

static int ap3426_psensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if(mode & AP3426_SYS_PS_ENABLE){
	mode &= ~AP3426_SYS_PS_ENABLE;
	if(mode == AP3426_SYS_DEV_RESET)
	    mode = AP3426_SYS_DEV_DOWN;
	ret = ap3426_set_mode(client,mode);
    }
    return ret;
}
#endif


static int ap3426_register_psensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device psensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for psensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->psensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "proximity";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for psensor\n", __FUNCTION__);
	goto done;
    }

    return 0;

done:
    return rc;
}

static void ap3426_unregister_psensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->psensor_input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend ap3426_early_suspend;
static void ap3426_suspend(struct early_suspend *h)
{

    if (misc_ps_opened)
	ap3426_psensor_disable(ap3426_data_g->client);
    if (misc_ls_opened)
	ap3426_lsensor_disable(ap3426_data_g->client);
}

static void ap3426_resume(struct early_suspend *h)
{

    if (misc_ls_opened)
	ap3426_lsensor_enable(ap3426_data_g->client);
    if (misc_ps_opened)
	ap3426_psensor_enable(ap3426_data_g->client);
}
#endif


/* range */
static ssize_t ap3426_show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%i\n", ap3426_get_range(data->client));
}

static ssize_t ap3426_store_range(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
	return -EINVAL;

    ret = ap3426_set_range(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}




static ssize_t ap3426_store_ir_data(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_ir_data(data->client, val);

    if (ret < 0)
	return ret;
#if POLLING_MODE
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}
/* mode */
static ssize_t ap3426_show_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%d\n", ap3426_get_mode(data->client));
}

static ssize_t ap3426_store_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_mode(data->client, val);

    if (ret < 0)
	return ret;
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}


static ssize_t ap3426_ls_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long mode;
    int ret, val = 0;

    LDBG("mode = %s,%s\n", __func__,buf);
    if (strict_strtoul(buf, 10, &mode) < 0)
	return -EINVAL;
    mutex_lock(&ap3426_ls_lock);
    //if((mode == AP3426_SYS_ALS_ENABLE) && ap3426_get_mode(data->client) != AP3426_SYS_ALS_ENABLE) {
    val = ap3426_get_mode(data->client);
    if(mode == AP3426_SYS_ALS_ENABLE ) {
	if (!(val & AP3426_SYS_ALS_ENABLE)) {
		ap3426_set_althres(data->client, 1000);
		ap3426_set_ahthres(data->client, 2000);
		misc_ls_opened = 1;
		if (val & AP3426_SYS_PS_ENABLE) {
			LDBG("%s: ap3426_mode: [0x%x] -> [0x%x] (als enable, als+ps)\n", __func__, val, AP3426_SYS_ALS_PS_ENABLE);
			ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_PS_ENABLE);
		} else {
			LDBG("%s: ap3426_mode: [0x%x] -> [0x%x] (als enable, als)\n", __func__, val, AP3426_SYS_ALS_ENABLE);
			ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_ENABLE);
		}
	} else
		LDBG("%s: ap3426_mode: [0x%x] (als enable, nothing)\n", __func__, val);
	//if (ret < 0)
	//    return ret;

    } else {
	if (val & AP3426_SYS_ALS_ENABLE) {
		misc_ls_opened = 0;
		if (!(val & AP3426_SYS_PS_ENABLE)) {
			LDBG("%s: ap3426_mode: [0x%x] (als disable, down)\n", __func__, val);
			ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_RESET);
			ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_DOWN);
		} else {
			LDBG("%s: ap3426_mode: [0x%x] -> [0x%x] (als disable, ps)\n", __func__, val, AP3426_SYS_PS_ENABLE);
			ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_PS_ENABLE);
		}
	} else
		LDBG("%s: ap3426_mode: [0x%x] (als disable, nothing)\n", __func__, val);
    }
    mutex_unlock(&ap3426_ls_lock);
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}


static ssize_t ap3426_ps_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long mode;
    int ret, val = 0;

    LDBG("%s: ap3426_mode = %s\n", __func__,buf);
    if (strict_strtoul(buf, 10, &mode) < 0)
	return -EINVAL;

    mutex_lock(&ap3426_ps_lock);
    //if((mode == AP3426_SYS_PS_ENABLE ) && ap3426_get_mode(data->client) != AP3426_SYS_PS_ENABLE) {
    val = ap3426_get_mode(data->client);
    if(mode == AP3426_SYS_PS_ENABLE ) {
	if (!(val & AP3426_SYS_PS_ENABLE)) {
		ret = ap3426_set_plthres(data->client, 100);
		ret = ap3426_set_phthres(data->client, 500);
		misc_ps_opened = 1;
		if (val & AP3426_SYS_ALS_ENABLE) {
			LDBG("%s: ap3426_mode: [0x%x] -> [0x%x] (ps enable, als+ps)\n", __func__, val, AP3426_SYS_ALS_PS_ENABLE);
			ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_PS_ENABLE);
		} else {
			LDBG("%s: ap3426_mode: [0x%x] -> [0x%x] (ps enable, ps)\n", __func__, val, AP3426_SYS_PS_ENABLE);
			ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_PS_ENABLE);
		}
	} else
		LDBG("%s: ap3426_mode: [0x%x] (ps enable, nothing)\n", __func__, val);
	//if (ret < 0)
	//    return ret;

    } else {
	if (val & AP3426_SYS_PS_ENABLE) {
		misc_ps_opened = 0;
		if (!(val & AP3426_SYS_ALS_ENABLE)) {
			LDBG("%s: ap3426_mode: [0x%x] (ps disable, down)\n", __func__, val);
			ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_RESET);
			ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_DOWN);
		} else {
			LDBG("%s: ap3426_mode: [0x%x] -> [0x%x] (ps disable, als)\n", __func__, val, AP3426_SYS_ALS_ENABLE);
			ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
				AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_ALS_ENABLE);
		}
	} else
		LDBG("%s: ap3426_mode: [0x%x] (ps disable, nothing)\n", __func__, val);
    }
    mutex_unlock(&ap3426_ps_lock);
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}

static ssize_t ap3426_hs_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long mode;
    int ret;

    LDBG("mode = %s,%s\n", __func__,buf);
    if (strict_strtoul(buf, 10, &mode) < 0)
	return -EINVAL;

    mutex_lock(&ap3426_heartbeat_lock);


    if(mode == 9) {
	data-> hsensor_enable = 1;
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_CONF,
		AP3426_REG_PS_CONF_MASK, AP3426_REG_PS_CONF_SHIFT, 0);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_DC_1,
		AP3426_REG_PS_DC_1_MASK, AP3426_REG_PS_DC_1_SHIFT, 0);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_DC_2,
		AP3426_REG_PS_DC_2_MASK, AP3426_REG_PS_DC_2_SHIFT, 0);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_LEDD,
		AP3426_REG_PS_LEDD_MASK, AP3426_REG_PS_LEDD_SHIFT, 1);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_MEAN,
		AP3426_REG_PS_MEAN_MASK, AP3426_REG_PS_MEAN_SHIFT, 0);
	ret = __ap3426_write_reg(data->client, AP3426_REG_PS_PERSIS,
		AP3426_REG_PS_PERSIS_MASK, AP3426_REG_PS_PERSIS_SHIFT, 0);
	ret = ap3426_set_plthres(data->client, 0);
	ret = ap3426_set_phthres(data->client, 535);
	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_PS_ENABLE);

//	if (ret < 0)
//	    return ret;
    } else {
	data-> hsensor_enable = 0;
	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_RESET);
	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_CONF,
		AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, AP3426_SYS_DEV_DOWN);
    }
    mutex_unlock(&ap3426_heartbeat_lock);
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}

/* lux */
static ssize_t ap3426_show_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;

    /* No LUX data if power down */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
	return sprintf((char*) buf, "%s\n", "Please power up first!");

    return sprintf(buf, "%d\n", ap3426_get_adc_value(data->client));
}



/* Px data */
static ssize_t ap3426_show_pxvalue(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;

    /* No Px data if power down */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
	return -EBUSY;

    return sprintf(buf, "%d\n", ap3426_get_px_value(data->client));
}



/* proximity object detect */
static ssize_t ap3426_show_object(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%d\n", ap3426_get_object(data->client));
}



/* ALS low threshold */
static ssize_t ap3426_show_althres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%d\n", ap3426_get_althres(data->client));
}

static ssize_t ap3426_store_althres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_althres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}



/* ALS high threshold */
static ssize_t ap3426_show_ahthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    return sprintf(buf, "%d\n", ap3426_get_ahthres(data->client));
}

static ssize_t ap3426_store_ahthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_ahthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}


#ifdef PS_CALIBRATION
/*ps calibration */
static int ap3426_set_pcrosstalk(struct i2c_client *client, int val)
{
	int lsb, msb, err = 0;

	msb = val >> 8;
	lsb = val & 0xFF;
	
	err =__ap3426_write_reg(client, AP3426_REG_PS_CAL_L, 0xFF, 0x00, lsb);    
	if (err<0) {
		dev_err(&client->dev, "%s, write reg 0x%x err, %d\n", __func__, AP3426_REG_PS_CAL_L,err);
		return err;            
	}
	err =__ap3426_write_reg(client, AP3426_REG_PS_CAL_H, 0xFF, 0x00, msb);
	if (err<0) {
		dev_err(&client->dev, "%s, write reg 0x%x err, %d\n", __func__, AP3426_REG_PS_CAL_H,err);
	}
          
	return err;
}

static int ap3426_set_ps_Lthrd(struct i2c_client *client,u16* buff)
{
	#define ARYSIZE_1 5
	int err = 0;
	u16 ps_data, g_pscali = 0;
	u8 mode,databuf[2],temp_pdata[ARYSIZE_1];	
	int i, j;
 
 //save the mode 
	databuf[0] = AP3426_REG_SYS_CONF;   
	mode=__ap3426_read_reg(client,databuf[0], 0xFF,0x00);	 
 //open the ps	
	databuf[1] = 0x02;
	__ap3426_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);

 //firstly set calibration data
	if (!ap3426_data_g->PsCalibration) {
				err= -3;//  ps > =0
				dev_err(&client->dev, "%s, cannot set cali data [%d]\n", __func__, ap3426_data_g->PsCalibration);
				goto EXIT_ERR;
	}
	LDBG("ps calibration data is %d\n", ap3426_data_g->PsCalibration);
	ap3426_set_pcrosstalk(client,ap3426_data_g->PsCalibration);

	for(i=0; i<ARYSIZE_1; ++i)
	{
		mdelay(50);
		//AP3xx6_get_ps_adc(client, temp_pdata+i);
		temp_pdata[i] = ap3426_get_px_value(client);
	}
	
	/* pdata sorting */
 	for (i = 0; i < ARYSIZE_1 - 1; i++)
		for (j = i+1; j < ARYSIZE_1; j++)
			if (temp_pdata[i] > temp_pdata[j])
				{
				 ps_data=temp_pdata[i];		
				 temp_pdata[i]=temp_pdata[j];
				 temp_pdata[j]=ps_data;
				}
	
	for (i = 0; i < 5; i++) 
	{
		LDBG("Lthrd pdata %2d: %d\n", i, temp_pdata[i]);
	}

	for (i = 1; i < 3; i++) 
	{
		g_pscali = g_pscali + temp_pdata[i];
	}
	LDBG("Lthrd pdata total(2): %d\n", g_pscali);
	g_pscali /= 2;
	LDBG("Lthrd pdata average : %d\n", g_pscali);
	
	databuf[0] = AP3426_REG_SYS_CONF;    
	databuf[1] = mode;
	__ap3426_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);

	if(g_pscali <=0)
	{
				err= -1;//  ps > =0
				dev_err(&client->dev, "%s, Lthrd error, pdata < 0\n", __func__);
				goto EXIT_ERR;
	}
	ap3426_data_g->PsLthres=g_pscali;
	
		 //ap3426_set_plthres(client,g_pscali);
		 *buff=g_pscali;
    		LDBG("Lthrd data is %u, 0x%x\n", g_pscali, g_pscali);
		 return 0;

EXIT_ERR:
	LDBG("set_ps_lthrd_calibration err---\n");
	return err;
}


static int ap3426_set_ps_Hthrd(struct i2c_client *client,u16* buff)
{
	#define ARYSIZE_2 5
	int err = 0;
	u16 ps_data, g_pscali = 0;
	u8 mode,databuf[2],temp_pdata[ARYSIZE_2];	
	int i, j;
 
 //save the mode 
	databuf[0] = AP3426_REG_SYS_CONF;   
	mode=__ap3426_read_reg(client,databuf[0], 0xFF,0x00);	 
 //open the ps	
	databuf[1] = 0x02;
	__ap3426_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);

 //firstly set calibration data
	if (!ap3426_data_g->PsCalibration) {
				err= -3;//  ps > =0
				dev_err(&client->dev, "%s, cannot set cali data [%d]\n", __func__, ap3426_data_g->PsCalibration);
				goto EXIT_ERR;
	}
	LDBG("ps calibration data is %d\n", ap3426_data_g->PsCalibration);
	ap3426_set_pcrosstalk(client,ap3426_data_g->PsCalibration);

	for(i=0; i<ARYSIZE_2; ++i)
	{
		mdelay(50);
		temp_pdata[i] = ap3426_get_px_value(client);		
	}
	
	/* pdata sorting */
 	for (i = 0; i < ARYSIZE_2 - 1; i++)
		for (j = i+1; j < ARYSIZE_2; j++)
			if (temp_pdata[i] > temp_pdata[j])
				{
				 ps_data=temp_pdata[i];		
				 temp_pdata[i]=temp_pdata[j];
				 temp_pdata[j]=ps_data;
				}
	
	for (i = 0; i < 5; i++) 
	{
		LDBG("Hthrd pdata %2d: %d\n", i, temp_pdata[i]);
	}

	for (i = 1; i < 3; i++) 
	{
		g_pscali = g_pscali + temp_pdata[i];
	}
	LDBG("Hthrd pdata total(2): %d\n", g_pscali);
	g_pscali /= 2;
	LDBG("Hthrd pdata average : %d\n", g_pscali);
	
	databuf[0] = AP3426_REG_SYS_CONF;    
	databuf[1] = mode;
	__ap3426_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);

	if(g_pscali <ap3426_data_g->PsLthres)
	{
				err= -1;// 
				dev_err(&client->dev, "%s, psHthres < psLthres\n", __func__);
				goto EXIT_ERR;
	}
	ap3426_data_g->PsHthres=g_pscali;
	
		 //ap3426_set_phthres(client,g_pscali);
		 *buff=g_pscali;
    		LDBG("Hthrd data is %u, 0x%x\n", g_pscali, g_pscali);
		 return 0;

EXIT_ERR:
	LDBG("set_ps_hthrd_calibration err--\n");
	return err;
}

/*
	suc:return 0 
*/

#define CALI_REQ_CMD			1
#define CALI_READ_BIAS_CMD		2
#define CALI_WRITE_BIAS_CMD		3
#define CALI_READ_STATUS_CMD		4

#define CALI_STATUS_NO_REQ		0
#define CALI_STATUS_RUNNING		1
#define CALI_STATUS_DONE_OK		2
#define CALI_STATUS_DONE_ERROR		-1

static int ap3426_set_ps_calibration(struct i2c_client *client,u16* buff)
{
	#define ARYSIZE_3 20
	
	int i=0, j;

	u8 mode ,databuf[2];
  u16 ps_data=0,g_pscali=0,temp_pdata[ARYSIZE_3];

	memset(databuf, 0x00, sizeof(databuf));

 //save the mode 
	databuf[0] = AP3426_REG_SYS_CONF;   
	mode=__ap3426_read_reg(client,databuf[0], 0xFF,0x00);	 
 //open the ps	
	databuf[1] = 0x02;
	__ap3426_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);
 //clean the ps
	ap3426_set_pcrosstalk(client,0);

	for(i=0; i<ARYSIZE_3; ++i)
	{
		mdelay(50);
		temp_pdata[i] = ap3426_get_px_value(client);
	}
	
	/* pdata sorting */
 	for (i = 0; i < ARYSIZE_3 - 1; i++)
		for (j = i+1; j < ARYSIZE_3; j++)
			if (temp_pdata[i] > temp_pdata[j])
				{
				 ps_data=temp_pdata[i];		
				 temp_pdata[i]=temp_pdata[j];
				 temp_pdata[j]=ps_data;
				}
	
	for (i = 0; i < 20; i++) 
	{
		LDBG("cali pdata %2d: %d\n", i, temp_pdata[i]);
	}

	for (i = 5; i < 15; i++) 
	{
		g_pscali = g_pscali + temp_pdata[i];
	}	
	LDBG("cali pdata total(10) : %d\n", g_pscali);
	g_pscali /= 10;
	LDBG("cali pdata average   : %d\n", g_pscali);


	databuf[0] = AP3426_REG_SYS_CONF;    
	databuf[1] = mode;
	__ap3426_write_reg(client,databuf[0], 0xFF,0x00,databuf[1]);   
			
	if(g_pscali > 510)
	{
		dev_err(&client->dev, "%s, x-talkt* too big\n", __func__);
		return -1;	        	
	}
	
	ap3426_data_g->PsCalibration=g_pscali;
    	LDBG("calibration data is %u, 0x%x\n", g_pscali, g_pscali);
	
	//ap3426_set_pcrosstalk(client,g_pscali);
	*buff=g_pscali;

	return 0;
}

static ssize_t ap3426_show_calibration_ps(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	if (ap3426_data_g->cali_cmd == CALI_READ_STATUS_CMD)
		return sprintf(buf, "%d\n", ap3426_data_g->cali_status);

	return sprintf(buf, "%d\n", ap3426_data_g->PsCalibration);
}

static ssize_t ap3426_store_calibration_ps(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ap3426_data *data = ap3426_data_g;
	u16 psData=0,ret;
	int buf_count=0, temp[3];
	char *pch, source[512], *myBuf= source;

	strcpy(source,buf);
	while ((pch = strsep(&myBuf, ", ")) != NULL) {
		buf_count++;
	}

	if (buf_count == 1) {
		sscanf(buf, "%d",&temp[0]);
		ap3426_data_g->cali_cmd = (u8)temp[0];
		LDBG("calibrator => cmd:%d\n", ap3426_data_g->cali_cmd);

		switch (ap3426_data_g->cali_cmd) {
	
		case CALI_REQ_CMD:
			ap3426_data_g->cali_status = CALI_STATUS_RUNNING;
  			ret = ap3426_set_ps_calibration(data->client,&psData);
			if (ret < 0) {
				dev_err(&data->client->dev, "get calibrator error [%d]\n", ret);
				ap3426_data_g->cali_status = CALI_STATUS_DONE_ERROR;
			} else {
				ap3426_data_g->PsCalibration = psData;
				ap3426_data_g->cali_status = CALI_STATUS_DONE_OK;
			}
			break;
			
		case CALI_READ_BIAS_CMD:
		case CALI_READ_STATUS_CMD:
			break;

		default:
			dev_err(&data->client->dev, "1input parameter incorrect !!! | %zd\n", count);
			break;
		}
	} else if (buf_count == 2) {
		sscanf(buf, "%d %d",&temp[0], &temp[1]);
		ap3426_data_g->cali_cmd = (u8)temp[0];
		LDBG("calibrator => cmd:%d\n", ap3426_data_g->cali_cmd);

		switch (ap3426_data_g->cali_cmd) {

		case CALI_WRITE_BIAS_CMD:
			psData = (u16)temp[1];
			LDBG("calibrator => setbias:%d\n", psData);
			if(psData > 510)
				dev_err(&data->client->dev, "%s, x-talkt* too big\n", __func__);
			else {
				ap3426_data_g->PsCalibration = psData;
				ret = ap3426_set_pcrosstalk(data->client, ap3426_data_g->PsCalibration);
				if (ret < 0) {
					ap3426_data_g->cali_status = CALI_STATUS_DONE_ERROR;
					dev_err(&data->client->dev, "%s, write calibrator error: %d\n", __func__, ret);
				} else
					ap3426_data_g->cali_status = CALI_STATUS_DONE_OK;
			}
			break;

		default:
			dev_err(&data->client->dev, "2input parameter incorrect !!! | %zd\n", count);
			break;
		}
	} else
		dev_err(&data->client->dev, "3input parameter incorrect !!! | %zd\n", count);

	return count;
}

static ssize_t ap3426_show_plthres2(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", ap3426_data_g->PsLthres);
}

static ssize_t ap3426_store_plthres2(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    u16 plthrd;
    struct ap3426_data *data = ap3426_data_g;

    ret = ap3426_set_ps_Lthrd(data->client, &plthrd);

    if (ret < 0)
	return ret;

    return count;
}

static ssize_t ap3426_show_phthres2(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", ap3426_data_g->PsHthres);
}

static ssize_t ap3426_store_phthres2(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    u16 phthrd;
    struct ap3426_data *data = ap3426_data_g;

    ret = ap3426_set_ps_Hthrd(data->client, &phthrd);

    if (ret < 0)
	return ret;

    return count;
}
#endif

/* Px low threshold */
static ssize_t ap3426_show_plthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
#ifdef PS_CALIBRATION
    ap3426_data_g->PsLthres= ap3426_get_plthres(data->client);
#endif
    return sprintf(buf, "%d\n", ap3426_get_plthres(data->client));
}

static ssize_t ap3426_store_plthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

#ifdef PS_CALIBRATION
    ap3426_data_g->PsLthres= val;
#endif
    ret = ap3426_set_plthres(data->client, val);

    if (ret < 0)
	return ret;

    return count;
}


/* Px high threshold */
static ssize_t ap3426_show_phthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
#ifdef PS_CALIBRATION
    ap3426_data_g->PsHthres= ap3426_get_phthres(data->client);
#endif
    return sprintf(buf, "%d\n", ap3426_get_phthres(data->client));
}

static ssize_t ap3426_store_phthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

#ifdef PS_CALIBRATION
    ap3426_data_g->PsHthres= val;
#endif
    ret = ap3426_set_phthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}



/* calibration */
static ssize_t ap3426_show_calibration_state(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    return sprintf(buf, "%d\n", cali);
}

static ssize_t ap3426_store_calibration_state(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    int stdls, lux; 
    char tmp[10];

    LDBG("DEBUG ap3426_store_calibration_state..\n");

    /* No LUX data if not operational */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
    {
	printk("Please power up first!");
	return -EINVAL;
    }

    cali = 100;
    sscanf(buf, "%d %s", &stdls, tmp);

    if (!strncmp(tmp, "-setcv", 6))
    {
	cali = stdls;
	return -EBUSY;
    }

    if (stdls < 0)
    {
	printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
		Set calibration factor to 100.\n", stdls);
	return -EBUSY;
    }

    lux = ap3426_get_adc_value(data->client);
    cali = stdls * 100 / lux;

    return -EBUSY;
}


#ifdef LSC_DBG
/* engineer mode */
static ssize_t ap3426_em_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct ap3426_data *data = ap3426_data_g;
    int i;
    u8 tmp;

    LDBG("DEBUG ap3426_em_read..\n");
    mutex_lock(&ap3426_lock);
    for (i = 0; i < reg_num; i++)
    {
	tmp = i2c_smbus_read_byte_data(data->client, reg_array[i]);

	LDBG("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
    }
    mutex_unlock(&ap3426_lock);

    return 0;
}

static ssize_t ap3426_em_write(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3426_data *data = ap3426_data_g;
    u32 addr,val,idx=0;
    int ret = 0;

    LDBG("DEBUG ap3426_em_write..\n");

    sscanf(buf, "%x%x", &addr, &val);

    printk("Write [%x] to Reg[%x]...\n",val,addr);

    mutex_lock(&ap3426_lock);
    ret = i2c_smbus_write_byte_data(data->client, addr, val);
    ADD_TO_IDX(addr,idx)
    mutex_unlock(&ap3426_lock);

	if (!ret)
	    data->reg_cache[idx] = val;

    return count;
}
#endif


static struct device_attribute attributes[] = {
    __ATTR(range, S_IWUSR | S_IRUGO, ap3426_show_range, ap3426_store_range),
    __ATTR(mode, 0666, ap3426_show_mode, ap3426_store_mode),
    __ATTR(lsensor, 0666, ap3426_show_mode, ap3426_ls_enable),
    __ATTR(psensor, 0666, ap3426_show_mode, ap3426_ps_enable),
    __ATTR(hsensor, 0666, ap3426_show_mode, ap3426_hs_enable),
    __ATTR(lux, S_IRUGO, ap3426_show_lux, NULL),
    __ATTR(pxvalue, S_IRUGO, ap3426_show_pxvalue, NULL),
    __ATTR(object, S_IRUGO, ap3426_show_object, NULL),
    __ATTR(althres, S_IWUSR | S_IRUGO, ap3426_show_althres, ap3426_store_althres),
    __ATTR(ahthres, S_IWUSR | S_IRUGO, ap3426_show_ahthres, ap3426_store_ahthres),
    __ATTR(plthres, S_IWUSR | S_IRUGO, ap3426_show_plthres, ap3426_store_plthres),
    __ATTR(phthres, S_IWUSR | S_IRUGO, ap3426_show_phthres, ap3426_store_phthres),
#ifdef PS_CALIBRATION    
    __ATTR(plthres2, S_IWUSR | S_IRUGO, ap3426_show_plthres2, ap3426_store_plthres2),
    __ATTR(phthres2, S_IWUSR | S_IRUGO, ap3426_show_phthres2, ap3426_store_phthres2),
    __ATTR(calibration_al, S_IWUSR | S_IRUGO, ap3426_show_calibration_state, ap3426_store_calibration_state),
    __ATTR(calibration_ps, S_IWUSR | S_IRUGO, ap3426_show_calibration_ps, ap3426_store_calibration_ps),
#else
    __ATTR(calibration, S_IWUSR | S_IRUGO, ap3426_show_calibration_state, ap3426_store_calibration_state),
#endif    
    __ATTR(ir_data, 0666, NULL, ap3426_store_ir_data),
#ifdef LSC_DBG
    __ATTR(em, S_IWUSR | S_IRUGO, ap3426_em_read, ap3426_em_write),
#endif

};

static int create_sysfs_interfaces(struct ap3426_data *sensor)
{
    int i;
    struct class *ap3426_class = NULL;
    struct device *ap3426_dev = NULL;
    int ret;

    ap3426_class = class_create(THIS_MODULE, "sensors");
    if (IS_ERR(ap3426_class)) {
	ret = PTR_ERR(ap3426_class);
	ap3426_class = NULL;
	LDBG("%s: could not allocate ap3426_class, ret = %d\n", __func__, ret);
	goto ap3426_class_error;
    }

    ap3426_dev= device_create(ap3426_class,
	    NULL, 0, "%s", "di_sensors");

    if(ap3426_dev == NULL)
	goto ap3426_device_error;
    for (i = 0; i < ARRAY_SIZE(attributes); i++)
	if (device_create_file(ap3426_dev, attributes + i))
	    goto ap3426_create_file_error;

    return 0;

ap3426_create_file_error:
    for ( ; i >= 0; i--)
	device_remove_file(ap3426_dev, attributes + i);

ap3426_device_error:
    class_destroy(ap3426_class);
ap3426_class_error:
    dev_err(&sensor->client->dev, "%s:Unable to create interface\n", __func__);
    return -1;
}
static int ap3426_init_client(struct i2c_client *client)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    int i;

    LDBG("DEBUG ap3426_init_client..\n");

    /* read all the registers once to fill the cache.
     * if one of the reads fails, we consider the init failed */
    for (i = 0; i < reg_num; i++) {

	int v = 0;
    mutex_lock(&ap3426_lock);
	i2c_smbus_read_byte_data(client, reg_array[i]);
    mutex_unlock(&ap3426_lock);

	if (v < 0)
	    return -ENODEV;

	data->reg_cache[i] = v;
    }
    /* set defaults */

    ap3426_set_range(client, AP3426_ALS_RANGE_0);
    ap3426_set_mode(data->client, AP3426_SYS_DEV_DOWN);

    return 0;
}

#if POLLING_MODE
static void pl_timer_callback(unsigned long pl_data)
{
    struct ap3426_data *data;
    int ret =0;

    data = ap3426_data_g;
    queue_work(data->plsensor_wq, &data->plsensor_work);

    ret = mod_timer(&ap3426_data_g->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");

}
#endif
static void plsensor_work_handler(struct work_struct *w)
{

    struct ap3426_data *data =
	container_of(w, struct ap3426_data, plsensor_work);
    u8 int_stat;
    int pxvalue;
    int obj;
    int ret;
    int value;

    int_stat = ap3426_get_intstat(data->client);
    LDBG("int_stat: 0x%0x\n", int_stat);


    // ALS int
    if (int_stat & AP3426_REG_SYS_INT_AMASK)
    {
	LDBG("LS INT Status: %0x\n", int_stat);

	value = ap3426_get_adc_value(data->client);
	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_INTSTATUS,
		AP3426_REG_SYS_INT_AMASK, AP3426_REG_SYS_INT_LS_SHIFT, 0);
	input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
	input_sync(data->lsensor_input_dev);
    }

    // PX int
    if (int_stat & AP3426_REG_SYS_INT_PMASK)
    {
	LDBG("PS INT Status: %0x\n", int_stat);
	obj = ap3426_get_object(data->client);
	pxvalue = ap3426_get_px_value(data->client); 

	ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_INTSTATUS,
		AP3426_REG_SYS_INT_PMASK, AP3426_REG_SYS_INT_PS_SHIFT, 0);
	LDBG("obj is %d, %s\n", obj, obj ? "obj near":"obj far");
	input_report_abs(data->psensor_input_dev, ABS_DISTANCE, (!obj)*10);
	input_sync(data->psensor_input_dev);
	LDBG("psensor: input report value is %d\n", (!obj)*10);

	if(data->hsensor_enable) {
	    LDBG("pxvalue = %d\n", pxvalue);
	    input_report_rel(data->hsensor_input_dev, ABS_WHEEL, pxvalue);
	    input_sync(data->hsensor_input_dev);
	}

    }
    else
    {
        LDBG("other INT Status: %0x\n", int_stat);
    }


    enable_irq(data->client->irq);
}
/*
 * I2C layer
 */

static irqreturn_t ap3426_irq(int irq, void *data_)
{
    struct ap3426_data *data = data_;

    LDBG("interrupt\n");
    disable_irq_nosync(data->client->irq);
    queue_work(data->plsensor_wq, &data->plsensor_work);

    return IRQ_HANDLED;
}

static int ap3426_parse_dt(struct device *dev,
			struct ap3426_data *data)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	ret = of_get_named_gpio(np, "irq-gpio", 0);
	if (ret < 0) {
		dev_err(&data->client->dev,"failed to get \"irq-gpio\"\n");
		goto err;
	}
	data->irq_gpio = ret;

err:
	return ret;
}


#define FT_VTG_MIN_UV           2600000
#define FT_VTG_MAX_UV           3300000
#define FT_I2C_VTG_MIN_UV       1800000
#define FT_I2C_VTG_MAX_UV       1800000

static int ap3426_power_on(struct ap3426_data *data, bool on)
{
	int rc = 0;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,"Regulator vdd enable failed rc=%d\n", rc);
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		return rc;
	}

	return rc;
}

static int ap3426_power_init(struct ap3426_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int __devinit ap3426_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct ap3426_data *data;
    int err = 0;

    LDBG("ap3426_probe\n");

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
	err = -EIO;
	goto exit_free_gpio;
    }

    reg_array = ap3426_reg;
    range = ap3426_range;
    reg_num = AP3426_NUM_CACHABLE_REGS;

    data = kzalloc(sizeof(struct ap3426_data), GFP_KERNEL);
    if (!data){
	err = -ENOMEM;
	goto exit_free_gpio;
    }

    data->client = client;
    i2c_set_clientdata(client, data);

	err = ap3426_parse_dt(&client->dev, data);
	if (err < 0) {
                dev_err(&client->dev,"failed to parse device tree: %d\n", err);
		goto exit_free_gpio;
        }
	dev_err(&client->dev,"irq_gpio = %d, irq = %d\n", data->irq_gpio, client->irq);

	err = ap3426_power_init(data, true);
	if (err) {
		dev_err(&client->dev,"power init failed");
		goto exit_free_gpio;
	}

	err = ap3426_power_on(data, true);
	if (err) {
		dev_err(&client->dev,"power on failed");
		goto exit_free_gpio;
	}

    data->irq = client->irq;

	msleep(200);

    /* initialize the AP3426 chip */
    err = ap3426_init_client(client);
    if (err)
	goto exit_kfree;

    err = ap3426_register_lsensor_device(client,data);
    if (err){
	dev_err(&client->dev, "failed to register_lsensor_device\n");
	goto exit_kfree;
    }

    err = ap3426_register_psensor_device(client, data);
    if (err) {
	dev_err(&client->dev, "failed to register_psensor_device\n");
	goto exit_free_ls_device;
    }

    err = ap3426_register_heartbeat_sensor_device(client, data);
    if (err) {
	dev_err(&client->dev, "failed to register_heartbeatsensor_device\n");
	goto exit_free_heartbeats_device;
    }

    err = create_sysfs_interfaces(data);
    if (err)
	goto exit_free_ps_device;

#ifdef CONFIG_HAS_EARLYSUSPEND
    ap3426_early_suspend.suspend = ap3426_suspend;
    ap3426_early_suspend.resume  = ap3426_resume;
    ap3426_early_suspend.level   = 0x02;
    register_early_suspend(&ap3426_early_suspend);
#endif


    //err = request_threaded_irq(client->irq, NULL, ap3426_irq,
    err = request_irq(client->irq, ap3426_irq,
	    IRQF_TRIGGER_FALLING,
	    "ap3426", data);
    if (err) {
	dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",err,client->irq);
	goto exit_free_ps_device;
    }

    data->plsensor_wq = create_singlethread_workqueue("plsensor_wq");
    if (!data->plsensor_wq) {
	LDBG("%s: create workqueue failed\n", __func__);
	err = -ENOMEM;
	goto err_create_wq_failed;
    }

    INIT_WORK(&data->plsensor_work, plsensor_work_handler);

#if POLLING_MODE
    LDBG("Timer module installing\n");
    setup_timer(&data->pl_timer, pl_timer_callback, 0);
#endif

    ap3426_data_g = data;
    
#ifdef PS_CALIBRATION
				    ap3426_data_g->PsCalibration=0;
				    ap3426_data_g->PsLthres=0;
				    ap3426_data_g->PsHthres=0;
				    ap3426_data_g->cali_status=CALI_STATUS_NO_REQ;
#endif    
    
    dev_info(&client->dev, "Driver version %s enabled\n", DRIVER_VERSION);
    return 0;
err_create_wq_failed:
#if POLLING_MODE
    if(&data->pl_timer != NULL)
	del_timer(&data->pl_timer);
#endif
    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
exit_free_ps_device:
    ap3426_unregister_psensor_device(client,data);

exit_free_heartbeats_device:
    ap3426_unregister_heartbeat_device(client,data);
exit_free_ls_device:
    ap3426_unregister_lsensor_device(client,data);

exit_kfree:
    kfree(data);

exit_free_gpio:

    LDBG("ap3426_probe error \r\n");
    return err;
}

static int __devexit ap3426_remove(struct i2c_client *client)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    free_irq(data->irq, data);

    ap3426_unregister_psensor_device(client,data);
    ap3426_unregister_lsensor_device(client,data);
    ap3426_unregister_heartbeat_device(client,data);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ap3426_early_suspend);
#endif

    ap3426_set_mode(data->client, 0);
    kfree(i2c_get_clientdata(client));

    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
#if POLLING_MODE
    if(&data->pl_timer)
	del_timer(&data->pl_timer);
#endif
    return 0;
}

static const struct i2c_device_id ap3426_id[] = {
    { AP3426_DRV_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, ap3426_id);

static struct of_device_id ap3426_match_table[] = {
        { .compatible = "dyna,ap3426",},
        { },
};

static struct i2c_driver ap3426_driver = {
    .driver = {
	.name	= AP3426_DRV_NAME,
	.owner	= THIS_MODULE,
	.of_match_table = ap3426_match_table,
    },
    .probe	= ap3426_probe,
    .remove	= __devexit_p(ap3426_remove),
    .id_table = ap3426_id,
};

static int __init ap3426_init(void)
{
    int ret;

    LDBG("ap3426_init\n");
    ret = i2c_add_driver(&ap3426_driver);
    return ret;	

}

static void __exit ap3426_exit(void)
{
    i2c_del_driver(&ap3426_driver);
}

MODULE_AUTHOR("Templeton Tsai Dyna-Image Corporation.");
MODULE_DESCRIPTION("AP3426 driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(ap3426_init);
module_exit(ap3426_exit);



