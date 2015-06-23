//
// cclogic-core.c
//
// Core of CC-Logic drivers
//
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#include "cclogic-core.h"


static struct cclogic_dev *cclogic_priv;
static struct mutex cclogic_list_lock;
static struct mutex cclogic_ops_lock;
static struct list_head cclogic_dev_list;

#define DRIVER_NAME "cclogic"

/*
 *
 */
static int cclogic_reg_set_optimum_mode_check(struct regulator *reg,
                                              int load_uA)
{
	pr_debug("[%s][%d]\n", __func__, __LINE__);

	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}
/*
 *
 */
static int cclogic_power_on(struct cclogic_dev *cclogic_dev, bool on) 
{
	int ret=0;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	if (cclogic_dev->platform_data->i2c_pull_up) {	
		if (on == false){
			if(cclogic_dev->regulator_en){
				cclogic_reg_set_optimum_mode_check(
						       cclogic_dev->vcc_i2c, 0);
				regulator_disable(cclogic_dev->vcc_i2c);
				cclogic_dev->regulator_en = false;
			}
		}else{
			if(!cclogic_dev->regulator_en){
				ret = cclogic_reg_set_optimum_mode_check(
				      cclogic_dev->vcc_i2c,CCLOGIC_I2C_LOAD_UA);
				if (ret < 0) {
					dev_err(&cclogic_dev->i2c_client->dev,
						"%s-->Regulator vcc_i2c set_opt"
						" failed rc=%d\n",__func__,ret);
					goto error_reg_opt_i2c;
				}
				ret = regulator_enable(cclogic_dev->vcc_i2c);
				if (ret) {
					dev_err(&cclogic_dev->i2c_client->dev,
						"%s-->Regulator vcc_i2c enable "
						"failed rc=%d\n", __func__,ret);
					goto error_reg_en_vcc_i2c;
				}
				cclogic_dev->regulator_en = true;
			}
		}
	}
	return 0;

error_reg_en_vcc_i2c:
	cclogic_reg_set_optimum_mode_check(cclogic_dev->vcc_i2c, 0);
error_reg_opt_i2c:
	cclogic_dev->regulator_en = false;
	return ret;
}
/*
 *
 */
static int cclogic_regulator_configure(struct cclogic_dev *cclogic_dev, bool on)
{
	int ret=0;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	if (!cclogic_dev->platform_data->i2c_pull_up) {
		return 0;
	}
	if (on == false && cclogic_dev->vcc_i2c){
		if (regulator_count_voltages(cclogic_dev->vcc_i2c) > 0)
			regulator_set_voltage(cclogic_dev->vcc_i2c,0,
					CCLOGIC_I2C_VTG_MAX_UV);
		regulator_put(cclogic_dev->vcc_i2c);
		cclogic_dev->vcc_i2c = NULL;
	}else if(!cclogic_dev->vcc_i2c){
		cclogic_dev->vcc_i2c = 
			regulator_get(&cclogic_dev->i2c_client->dev,"vcc_i2c");
		if (IS_ERR(cclogic_dev->vcc_i2c)) {
			dev_err(&cclogic_dev->i2c_client->dev,
					"%s: Failed to get i2c regulator\n",
					__func__);
			ret = PTR_ERR(cclogic_dev->vcc_i2c);
			goto err_get_vtg_i2c;
		}

		if (regulator_count_voltages(cclogic_dev->vcc_i2c) > 0) {
			ret = regulator_set_voltage(cclogic_dev->vcc_i2c, 
					CCLOGIC_I2C_VTG_MIN_UV, 
					CCLOGIC_I2C_VTG_MAX_UV);
			if (ret) {
				dev_err(&cclogic_dev->i2c_client->dev,
						"%s-->reg set i2c vtg failed "
						"ret =%d\n", __func__,ret);
				goto err_set_vtg_i2c;
			}
		}
	}

	return 0;

err_set_vtg_i2c:
	regulator_put(cclogic_dev->vcc_i2c);
err_get_vtg_i2c:
	cclogic_dev->vcc_i2c=NULL;
	return ret;

};
/*
 *
 */
static int cclogic_irq_enable(struct cclogic_dev *cclogic_dev, bool enable)
{
	int ret = 0;

	pr_debug("[%s][%d] enable=%d  irq_enabled=%d\n", __func__, __LINE__,
			enable,cclogic_dev->irq_enabled);

	if (enable) {
		if (!cclogic_dev->irq_enabled){
			enable_irq(cclogic_dev->irq);
			enable_irq_wake(cclogic_dev->irq);
			cclogic_dev->irq_enabled = true;
		}
	} else {
		if (cclogic_dev->irq_enabled) {
			disable_irq_wake(cclogic_dev->irq);
			disable_irq(cclogic_dev->irq);
			cclogic_dev->irq_enabled = false;
		}
	}

	return ret;
}
/*
 *
 */
static int cclogic_parse_dt(struct device *dev, struct cclogic_platform *pdata)
{
	struct device_node *np = dev->of_node;
	struct device_node *temp;
	int idx = 0;
	int ret;
	unsigned int val;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	pdata->chip_num = 0;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "cc_logic,irq-gpio", 0, 
			&pdata->irq_flags);
	pdata->function_switch_gpio1 = of_get_named_gpio_flags(np,
			"cc_logic,function-switch-gpio1", 0, NULL);
	pdata->function_switch_gpio2 = of_get_named_gpio_flags(np,
			"cc_logic,function-switch-gpio2", 0, NULL);
	pdata->usb_ss_gpio = of_get_named_gpio_flags(np,
			"cc_logic,usb-ss-gpio", 0, NULL);
	pdata->enb_gpio = of_get_named_gpio_flags(np,
			"cc_logic,power-control", 0, NULL);
	pdata->i2c_pull_up = of_property_read_bool(np,"cc_logic,i2c-pull-up");

	for_each_child_of_node(np, temp) {
		ret = of_property_read_string(temp, "chip-name",
				&pdata->chip[idx].chip_name);
		if (ret)
			return ret;

		ret = of_property_read_u32(temp, "chip-address", &val);
		if(ret)
			return ret;

		pdata->chip[idx].address = val;

		pr_debug("%s--> chip:%s, address:0x%02x\n",__func__,
				pdata->chip[idx].chip_name,val);

		idx++;
		if(idx>=CCLOGIC_MAX_SUPPORT_CHIP){
			dev_err(dev,"%s-->too many devices\n",__func__);
			break;
		}
	}

	pdata->chip_num = idx;

	return 0;
}


/*
 *
 */
#ifdef CCLOGIC_ENABLE_IRQ
static irqreturn_t cclogic_irq(int irq, void *data)
{
	struct cclogic_dev *cclogic_dev = (struct cclogic_dev *)data;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	if(!cclogic_dev || !cclogic_dev->i2c_client){
		return IRQ_HANDLED;
	}

#ifdef CCLOGIC_ENABLE_WORK
	if (!wake_lock_active(&cclogic_dev->wakelock)){
		wake_lock(&cclogic_dev->wakelock);
	}
	cancel_delayed_work(&cclogic_dev->work);
	schedule_delayed_work(&cclogic_dev->work, 0);
#endif

	return IRQ_HANDLED;
}
#endif


/*
 *
 */
static ssize_t cclogic_show_real_status(struct cclogic_state *pdata, char *buf)
{
	char * vbus=NULL;
	char * charging=NULL;
	char * port=NULL;
	char * polarity=NULL;
	char * evt=NULL;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	switch(pdata->evt){
		case CCLOGIC_EVENT_NONE:
			evt="No Event";
			break;
		case CCLOGIC_EVENT_DETACHED:
			evt = "Detached Event";
			break;
		case CCLOGIC_EVENT_ATTACHED:
			evt = "Attached Event";
			break;
	}

	if(pdata->vbus){
		vbus = "vbus=1";
	}else{
		vbus = "vbus=0";
	}

	switch(pdata->cc){
		case CCLOGIC_CC1:
			polarity = "polarity=cc1";
			break;
		case CCLOGIC_CC2:
			polarity = "polarity=cc2";
			break;
		case CCLOGIC_CC_UNKNOWN:
			polarity = "polarity=unknown";
			break;
	}

	switch(pdata->device){
		case CCLOGIC_NO_DEVICE:
			port = "attached=nothing";
			break;
		case CCLOGIC_USB_DEVICE:
			port = "attached=device";
			break;
		case CCLOGIC_USB_HOST:
			port = "attached=host";
			break;
		case CCLOGIC_DEBUG_DEVICE:
			if(pdata->vbus){
				port = "attached=debug";	
			}else{
				port = "attached=shenqi_audio";	
			}
			break;
		case CCLOGIC_AUDIO_DEVICE:
			port = "attached=audio";	
			break;
		case CCLOGIC_DEVICE_UNKNOWN:
			port = "attached=unknown";
			break;
	}

	switch(pdata->charger){
		case CCLOGIC_CURRENT_NONE:
			charging = "charging=none";
			break;
		case CCLOGIC_CURRENT_DEFAULT:
			charging = "charging=default";
			break;
		case CCLOGIC_CURRENT_MEDIUM:
			charging = "charging=medium";
			break;
		case CCLOGIC_CURRENT_ACCESSORY:
			charging = "charging=from accessory";
			break;
		case CCLOGIC_CURRENT_HIGH:
			charging = "charging=high";
			break;
	}

	return sprintf(buf, " %15s:%8s;%25s;%23s;%15s\n",
			evt,vbus,charging,port,polarity);

}

/*
 *
 */
static ssize_t cclogic_chip_store_enable(struct device *dev, 
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct cclogic_dev *cclogic_dev = dev_get_drvdata(dev);
        int value;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	if (gpio_is_valid(cclogic_dev->platform_data->enb_gpio)) {
		if (1 != sscanf(buf, "%d", &value)) {
			dev_err(dev, "Failed to parse integer: <%s>\n", buf);
			return -EINVAL;
		}

#ifndef CCLOGIC_DEBUG_TUSB320   //for debug
		gpio_set_value_cansleep(cclogic_dev->platform_data->enb_gpio,!value);
#else
		if(value){
			gpio_direction_output(cclogic_dev->platform_data->enb_gpio,0);
		}else{
			gpio_direction_input(cclogic_dev->platform_data->enb_gpio);
		}
#endif
	}

	return count;
}

/*
 *
 */
static ssize_t cclogic_chip_show_enable(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct cclogic_dev *cclogic_dev = dev_get_drvdata(dev);
	int ret;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	if (gpio_is_valid(cclogic_dev->platform_data->enb_gpio)) {
		ret = gpio_get_value_cansleep(cclogic_dev->platform_data->enb_gpio);
		if(ret){
			return sprintf(buf, "disabled\n"); 
		}else{
			return sprintf(buf, "enabled\n"); 
		}
	}else{
		return sprintf(buf, "No enabled pin\n"); 
	}
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, cclogic_chip_show_enable, cclogic_chip_store_enable);

/*
 *
 */
static ssize_t cclogic_show_status(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct cclogic_dev *cclogic_dev = dev_get_drvdata(dev);
	//int ret;

	pr_debug("[%s][%d]\n", __func__, __LINE__);


#ifdef CCLOGIC_UPDATE_STATUS
	mutex_lock(&cclogic_ops_lock);
	if(cclogic_dev->ops && cclogic_dev->ops->get_state){
		ret = cclogic_dev->ops->get_state(cclogic_dev->i2c_client,
				&cclogic_dev->state);
		if(ret){
			mutex_unlock(&cclogic_ops_lock);
			return sprintf(buf, "error\n"); 
		}
	}else{
		mutex_unlock(&cclogic_ops_lock);
		return sprintf(buf, "no chip\n");	
	}
	mutex_unlock(&cclogic_ops_lock);
#endif

	return cclogic_show_real_status(&cclogic_dev->state, buf);
}

static DEVICE_ATTR(status, S_IRUGO, cclogic_show_status, NULL);

/*
 *
 */
static void cclogic_func_set(struct cclogic_platform *p,enum cclogic_func_type func)
{
	switch(func){
	case CCLOGIC_FUNC_HIZ:
		gpio_set_value_cansleep(p->function_switch_gpio2,0);
		if (gpio_is_valid(p->function_switch_gpio1)){
			gpio_set_value_cansleep(p->function_switch_gpio1,0);
		}
		break;
	case CCLOGIC_FUNC_USB:
		gpio_set_value_cansleep(p->function_switch_gpio2,1);
		if (gpio_is_valid(p->function_switch_gpio1)){
			gpio_set_value_cansleep(p->function_switch_gpio1,1);
		}
		break;
	case CCLOGIC_FUNC_AUDIO:
		gpio_set_value_cansleep(p->function_switch_gpio2,0);
		if (gpio_is_valid(p->function_switch_gpio1)){
			gpio_set_value_cansleep(p->function_switch_gpio1,1);
		}
		break;
	case CCLOGIC_FUNC_UART:
		gpio_set_value_cansleep(p->function_switch_gpio2,1);
		if (gpio_is_valid(p->function_switch_gpio1)){
			gpio_set_value_cansleep(p->function_switch_gpio1,0);
		}
		break;
	}
}
/*
 *
 */
static int cclogic_do_real_work(struct cclogic_state *state, 
				struct cclogic_dev *pdata)
{
	int ret=0;
	struct cclogic_platform *p = pdata->platform_data;

	pr_debug("[%s][%d]\n", __func__, __LINE__);


	switch(state->evt){
	case CCLOGIC_EVENT_DETACHED:
		pr_debug("%s-->cable detached\n",__func__);
		if(pdata->vbus_on){
			otg_func_set(false);
			pdata->vbus_on=false;
		}

		if(pdata->shenqi_audio){
			pdata->shenqi_audio=false;
			switch_set_state(&pdata->sdev,0);
		}
		cclogic_func_set(p,CCLOGIC_FUNC_UART);
		return 0;
	case CCLOGIC_EVENT_NONE: 
		pr_debug("%s-->No event\n",__func__);
		break;
	case CCLOGIC_EVENT_ATTACHED:
		pr_debug("%s-->cable attached\n",__func__);
		break;
	}

	switch(state->cc){
	case CCLOGIC_CC1:
		pr_debug("%s-->usb_ss signal to cc1\n",__func__);
		gpio_set_value_cansleep(p->usb_ss_gpio,1);
		break;
	case CCLOGIC_CC2:
		pr_debug("%s-->usb_ss signal to cc2\n",__func__);
		gpio_set_value_cansleep(p->usb_ss_gpio,0);
		break;
	case CCLOGIC_CC_UNKNOWN:
		pr_debug("%s-->usb_ss signal to unknown\n",__func__);
		ret = -1;
		break;
	}

	/*
	   if(state->vbus){
	   switch(state->charger){
	   case CCLOGIC_CURRENT_NONE:
	   case CCLOGIC_CURRENT_DEFAULT:
	   case CCLOGIC_CURRENT_MEDIUM:
	   case CCLOGIC_CURRENT_ACCESSORY:
	   case CCLOGIC_CURRENT_HIGH:
	   }
	   }
	 */

	switch(state->device){
	case CCLOGIC_NO_DEVICE://nothing attached
		pr_debug("%s-->nothing attached,switch to UART\n",__func__);
		if(pdata->vbus_on){
			otg_func_set(false);
			pdata->vbus_on=false;
		}
		if(pdata->shenqi_audio){
			pdata->shenqi_audio=false;
			switch_set_state(&pdata->sdev,0);
		}
		cclogic_func_set(p,CCLOGIC_FUNC_UART);
		break;
	case CCLOGIC_USB_DEVICE:
		pr_debug("%s-->function switch set to usb host\n",__func__);
		if(!pdata->vbus_on){
			otg_func_set(true);
			pdata->vbus_on=true;
		}
		cclogic_func_set(p,CCLOGIC_FUNC_USB);
		break;
	case CCLOGIC_USB_HOST:
		pr_debug("%s-->function switch set to usb device\n",__func__);
		if(pdata->vbus_on){
			otg_func_set(false);
			pdata->vbus_on=false;
		}
		cclogic_func_set(p,CCLOGIC_FUNC_USB);
		break;
	case CCLOGIC_DEBUG_DEVICE:
		if(!state->vbus){
			pr_debug("%s-->function switch set to shenqi audio\n", 
					__func__);
			if(!pdata->vbus_on){
				otg_func_set(true);
				pdata->vbus_on=true;
			}
			cclogic_func_set(p,CCLOGIC_FUNC_AUDIO);

			if(!pdata->shenqi_audio){
				pdata->shenqi_audio=true;
				switch_set_state(&pdata->sdev,1);
			}
		}else{
			pr_debug("%s-->function switch set to debug(HiZ)\n",
					__func__);
			cclogic_func_set(p,CCLOGIC_FUNC_HIZ);

			if(pdata->vbus_on){
				otg_func_set(false);
				pdata->vbus_on=false;
			}
		}
		break;
	case CCLOGIC_AUDIO_DEVICE:
		pr_debug("%s-->function switch set to audio(HiZ)\n",__func__);
		cclogic_func_set(p,CCLOGIC_FUNC_HIZ);
		if(pdata->vbus_on){
			otg_func_set(false);
			pdata->vbus_on=false;
		}
		break;
	case CCLOGIC_DEVICE_UNKNOWN:
		dev_err(&pdata->i2c_client->dev,"%s-->function unknown,"
				" switch to HiZ\n",__func__);
		cclogic_func_set(p,CCLOGIC_FUNC_HIZ);
		ret = -1;
	}

	return ret;

}

/*
 *
 */
static int cclogic_get_chip(struct cclogic_dev *cclogic_dev)
{
	int ret = -1;
	struct cclogic_chip *cclogic,*next;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	mutex_lock(&cclogic_list_lock);
	list_for_each_entry_safe(cclogic, next, &cclogic_dev_list,
			chip_list) {
		if (!cclogic || !cclogic->chip_check) {
			break;
		}

		pr_debug("%s-->check chip:%s\n",__func__,cclogic->chip_name);

		cclogic_dev->i2c_client->addr = cclogic->addr;

		ret = cclogic->chip_reset(cclogic_dev->i2c_client);
		if(!ret){
			ret = cclogic->chip_check(cclogic_dev->i2c_client);
			if(!ret){
				cclogic_dev->ops = cclogic;
				ret = cclogic->chip_config(cclogic_dev->i2c_client);
				break;
			}
		}
	}
	mutex_unlock(&cclogic_list_lock);

	return ret;
}

/*
 *
 */
static void cclogic_do_work(struct work_struct *w)
{
	struct cclogic_dev *pdata = container_of(w, 
					struct cclogic_dev, work.work);
	int ret=0;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	mutex_lock(&cclogic_ops_lock);

	if(!pdata->ops){
		ret = cclogic_get_chip(pdata);
		if(ret){
			goto work_end;
		}
	}

	if(pdata->ops && pdata->ops->get_state){
		ret = pdata->ops->get_state(pdata->i2c_client,&pdata->state);
	}else{
		ret = -1;
	}
	if(ret){
		goto work_end;
	}

	if(pdata->ops && pdata->ops->ack_irq){
		ret = pdata->ops->ack_irq(pdata->i2c_client);
		if(ret){
			goto work_end;
		}
	}

	ret = cclogic_do_real_work(&pdata->state,pdata);

work_end:
	mutex_unlock(&cclogic_ops_lock);

	if(ret && 
	       !gpio_get_value_cansleep(cclogic_priv->platform_data->irq_gpio)){
		schedule_delayed_work(&pdata->work, msecs_to_jiffies(10));//TODO limit retry times
		return;
	}

	if(wake_lock_active(&pdata->wakelock))
		wake_unlock(&pdata->wakelock);
}

/*
 *
 */
static int cclogic_init_gpio(struct cclogic_dev *cclogic_dev)
{
	struct i2c_client *client = cclogic_dev->i2c_client;
	struct cclogic_platform *pdata = cclogic_dev->platform_data;
	int ret = 0;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	if (gpio_is_valid(pdata->function_switch_gpio1)) {
		ret = gpio_request(pdata->function_switch_gpio1, 
					"cclogic_func_gpio1");
		if (ret) {
			dev_err(&client->dev, 
					"%s-->unable to request gpio [%d]\n",
					__func__,pdata->function_switch_gpio1);
			goto err_gpio1;
		}
		ret = gpio_direction_output(pdata->function_switch_gpio1,0);
		if (ret) {
			dev_err(&client->dev,
				"%s-->unable to set direction for gpio [%d]\n",
				__func__,pdata->function_switch_gpio1);
			goto err_gpio1_dir;
		}
	}

	if (gpio_is_valid(pdata->function_switch_gpio2)) {
		ret = gpio_request(pdata->function_switch_gpio2, 
					"cclogic_func_gpio2");
		if (ret) {
			dev_err(&client->dev,
				 "%s-->unable to request gpio [%d]\n",
				__func__,pdata->function_switch_gpio2);
			goto err_gpio1_dir;
		}
		ret = gpio_direction_output(pdata->function_switch_gpio2,1);
		if (ret) {
			dev_err(&client->dev,
				"%s-->unable to set direction for gpio [%d]\n",
				__func__,pdata->function_switch_gpio2);
			goto err_gpio2_dir;
		}
	} else {
		ret = -ENODEV;
		dev_err(&client->dev,
			 "%s-->function_switch_gpio2 not provided\n",__func__);
		goto err_gpio1_dir;
	}

	if (gpio_is_valid(pdata->usb_ss_gpio)) {
		ret = gpio_request(pdata->usb_ss_gpio, "usb_ss_gpio");
		if (ret) {
			dev_err(&client->dev, 
					"%s-->unable to request gpio [%d]\n",
					__func__,pdata->usb_ss_gpio);
			goto err_gpio2_dir;
		}
		ret = gpio_direction_output(pdata->usb_ss_gpio,1);
		if (ret) {
			dev_err(&client->dev,
				"%s-->unable to set direction for gpio [%d]\n",
				__func__,pdata->usb_ss_gpio);
			goto err_ss_gpio;
		}
	} else {
		ret = -ENODEV;
		dev_err(&client->dev, "%s-->usb_ss_gpio not provided\n",
					__func__);
		goto err_gpio2_dir;
	}

	if (gpio_is_valid(pdata->enb_gpio)) {
		ret = gpio_request(pdata->enb_gpio, "enb_gpio");
		if (ret) {
			dev_err(&client->dev, 
					"%s-->unable to request gpio [%d]\n",
					__func__,pdata->enb_gpio);
			goto err_ss_gpio;
		}
#ifndef CCLOGIC_DEBUG_TUSB320   //for debug
		ret = gpio_direction_output(pdata->enb_gpio,0);
		if (ret) {
			dev_err(&client->dev, 
				"%s-->unable to set direction for gpio [%d]\n",
				__func__,pdata->enb_gpio);
			goto err_enb_gpio;
		}
#endif
	}

	return ret;

#ifndef CCLOGIC_DEBUG_TUSB320   //for debug
err_enb_gpio:
#endif
	if (gpio_is_valid(pdata->enb_gpio))
		gpio_free(pdata->enb_gpio);	

err_ss_gpio:
	if (gpio_is_valid(pdata->usb_ss_gpio))
		gpio_free(pdata->usb_ss_gpio);

err_gpio2_dir:
	if (gpio_is_valid(pdata->function_switch_gpio2))
		gpio_free(pdata->function_switch_gpio2);

err_gpio1_dir:
	if (gpio_is_valid(pdata->function_switch_gpio1))
		gpio_free(pdata->function_switch_gpio1);
err_gpio1:
	return ret;
}

/*
 *
 */
static int cclogic_remove_gpio(struct cclogic_dev *cclogic_dev)
{
	struct cclogic_platform *pdata = cclogic_dev->platform_data;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	if (gpio_is_valid(pdata->enb_gpio)){
		//gpio_direction_input(pdata->enb_gpio);
		gpio_free(pdata->enb_gpio); 
	}

	if (gpio_is_valid(pdata->usb_ss_gpio))
		gpio_free(pdata->usb_ss_gpio);

	if (gpio_is_valid(pdata->function_switch_gpio2))
		gpio_free(pdata->function_switch_gpio2);

	if (gpio_is_valid(pdata->function_switch_gpio1))
		gpio_free(pdata->function_switch_gpio1);

	return 0;

}

/**
 * cclogic_match_id()
 */
static int cclogic_match_id(const char *name)
{
	int i=0;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	if(unlikely(!name)){
		pr_err("%s-->No name\n",__func__);
		return -ENODEV;
	}

	if(cclogic_priv && cclogic_priv->platform_data){
		for(i=0;i<cclogic_priv->platform_data->chip_num;i++){
			struct cclogic_of_chip *chip = 
				&cclogic_priv->platform_data->chip[i];

			if(!chip->chip_name){
				pr_err("%s-->%s mismatch\n",__func__,name);
				return -ENODEV;
			}
			if(!strcmp(chip->chip_name, name))
				break;
		}
	}else{
		pr_err("%s-->%s mismatch\n",__func__,name);
		return -ENODEV;
	}

	return i;

}

/**
 * cclogic_register()
 */
int cclogic_register(struct cclogic_chip *c)
{
	int ret = 0;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	ret = cclogic_match_id(c->chip_name);
	if(ret<0){
		return ret;
	}

	c->addr = cclogic_priv->platform_data->chip[ret].address;

	mutex_lock(&cclogic_list_lock);
	list_add_tail(&c->chip_list, &cclogic_dev_list);
	mutex_unlock(&cclogic_list_lock);

	cancel_delayed_work(&cclogic_priv->work);
	flush_delayed_work(&cclogic_priv->work);	
	if (!wake_lock_active(&cclogic_priv->wakelock)){
		wake_lock(&cclogic_priv->wakelock);
	}
	schedule_delayed_work(&cclogic_priv->work, 0);

	pr_info("%s:Add device \"%s\"\n",__func__, c->chip_name);

	return 0;
}
EXPORT_SYMBOL(cclogic_register);

/**
 * cclogic_unregister()
 */
void cclogic_unregister(struct cclogic_chip *c)
{
	struct cclogic_chip *cclogic,*next;

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	mutex_lock(&cclogic_list_lock);
	list_for_each_entry_safe(cclogic, next, &cclogic_dev_list,
			chip_list) {
		if (cclogic == c) {
			pr_info("%s: Deleting device \"%s\" \n",
					__func__, c->chip_name);
			list_del(&c->chip_list);
			break;
		}
	}
	mutex_unlock(&cclogic_list_lock);

	cancel_delayed_work(&cclogic_priv->work);
	flush_delayed_work(&cclogic_priv->work);	

	mutex_lock(&cclogic_ops_lock);
	if(cclogic_priv->ops == c){
		cclogic_priv->ops = NULL;
	}
	mutex_unlock(&cclogic_ops_lock);
	

}
EXPORT_SYMBOL(cclogic_unregister);

static struct attribute *cclogic_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute_group cclogic_attr_group = {
	.attrs = cclogic_attrs,
};

/**
 * cclogic_probe()
 */
static int __devinit cclogic_probe(struct i2c_client *client,
				   const struct i2c_device_id *dev_id)
{
	int ret = 0;
	struct cclogic_dev *cclogic_dev = cclogic_priv;
	struct cclogic_platform *platform_data;

	pr_info("%s start\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev,"%s: i2c check failed\n", __func__);
		ret = -ENODEV;
		goto err_i2c;
	}

	if(!cclogic_dev){
		ret = -ENODEV;
		goto err_i2c;
	}

	memset(cclogic_dev,0,sizeof(*cclogic_dev));		

	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
					sizeof(*platform_data),GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev,"%s-->Failed to allocate memory\n",
						__func__);
			ret = -ENOMEM;
			goto err_i2c;
		}

		ret = cclogic_parse_dt(&client->dev, platform_data);
		if (ret){
			dev_err(&client->dev,"%s-->Failed parse dt\n",__func__);
			goto err_i2c;
		}
	} else {
		platform_data = client->dev.platform_data;
	}

	cclogic_dev->platform_data = platform_data;
	cclogic_dev->i2c_client   = client;
	cclogic_dev->dev = &client->dev;
	cclogic_dev->shenqi_audio = false;

	i2c_set_clientdata(client,cclogic_dev);

	ret = cclogic_regulator_configure(cclogic_dev, true);
	if (ret < 0) {
		dev_err(&client->dev, "%s-->Failed to configure regulators\n",
					__func__);
		goto err_i2c;
	}

	ret = cclogic_power_on(cclogic_dev, true);
	if (ret < 0) {
		dev_err(&client->dev, "%s-->Failed to power on\n",__func__);
		goto err_regulator_conf;
	}

	ret = cclogic_init_gpio(cclogic_dev);
	if(ret){
		dev_err(&client->dev,"%s-->error in set gpio\n",__func__);
		goto err_regulator_on;
	}

	if (gpio_is_valid(platform_data->irq_gpio)) {
		/* configure cclogic irq gpio */
		ret = gpio_request(platform_data->irq_gpio, "cclogic_irq_gpio");
		if (ret) {
			dev_err(&client->dev, 
					"%s-->unable to request gpio [%d]\n",
					__func__,platform_data->irq_gpio);
			goto err_set_gpio;
		}
		ret = gpio_direction_input(platform_data->irq_gpio);
		if (ret) {
			dev_err(&client->dev, 
				"%s-->unable to set direction for gpio [%d]\n",
				__func__,platform_data->irq_gpio);
			goto err_irq_gpio_dir;
		}
	} else {
		dev_err(&client->dev, "%s-->irq gpio not provided\n",__func__);
		goto err_set_gpio;
	}
	cclogic_dev->irq = gpio_to_irq(platform_data->irq_gpio);

	wake_lock_init(&cclogic_dev->wakelock, WAKE_LOCK_SUSPEND,
				"cclogic_wakelock");

	device_init_wakeup(cclogic_dev->dev, 1);

	INIT_DELAYED_WORK(&cclogic_dev->work, cclogic_do_work);

	ret = sysfs_create_group(&client->dev.kobj, &cclogic_attr_group);
	if (ret) {
		dev_err(&client->dev,
				"%s-->Unable to create sysfs for cclogic,"
				" errors: %d\n", __func__, ret);
		goto err_chip_check;
	}

	cclogic_dev->sdev.name = "usb_audio";
	ret = switch_dev_register(&cclogic_dev->sdev);
        if (ret) {
                dev_err(&client->dev, 
			"Failed to setup switch dev for cclogic driver");
		goto err_sdev;
        }


#ifdef CCLOGIC_ENABLE_IRQ
	ret = request_threaded_irq(cclogic_dev->irq,NULL, cclogic_irq, 
			cclogic_dev->platform_data->irq_flags, DRIVER_NAME, 
			cclogic_dev);
	if (ret) {
		dev_err(&client->dev, 
				"%s: Failed to create irq thread\n", __func__);
		goto err_irq_req;
	}
	disable_irq(cclogic_dev->irq);
	ret = cclogic_irq_enable(cclogic_dev,true);
	if(ret){
		dev_err(&client->dev,"%s: Failed to enable irq\n",__func__);
		goto err_irq_enable;
	}
#endif

	pr_info("%s Success\n", __func__);

	return 0;

	cancel_delayed_work_sync(&cclogic_dev->work);
	cclogic_irq_enable(cclogic_dev,false);
err_irq_enable:
	free_irq(cclogic_dev->irq,cclogic_dev);
err_irq_req:
	switch_dev_unregister(&cclogic_dev->sdev);
err_sdev:
	sysfs_remove_group(&client->dev.kobj, &cclogic_attr_group);
err_chip_check:	
	device_init_wakeup(cclogic_dev->dev, 0);
	wake_lock_destroy(&cclogic_dev->wakelock);
err_irq_gpio_dir:
	if (gpio_is_valid(platform_data->irq_gpio))
		gpio_free(platform_data->irq_gpio);
err_set_gpio:
	cclogic_remove_gpio(cclogic_dev);
err_regulator_on:
	cclogic_power_on(cclogic_dev, false);
err_regulator_conf:
	cclogic_regulator_configure(cclogic_dev, false);
err_i2c:
	dev_err(&client->dev,"%s Failed\n", __func__);

	return ret;
}



/**
 * cclogic_remove()
 */
static int __devexit cclogic_remove(struct i2c_client *client)
{
	struct cclogic_dev *cclogic_dev;

	pr_info("%s\n", __func__);

	cclogic_dev = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &cclogic_attr_group);
	switch_dev_unregister(&cclogic_dev->sdev);

	device_init_wakeup(cclogic_dev->dev, 0);
	cclogic_irq_enable(cclogic_dev,false);	
	wake_lock_destroy(&cclogic_dev->wakelock);

	cancel_delayed_work_sync(&cclogic_dev->work);	
	free_irq(cclogic_dev->irq,cclogic_dev);

	cclogic_remove_gpio(cclogic_dev);
	if (gpio_is_valid(cclogic_dev->platform_data->irq_gpio))
		gpio_free(cclogic_dev->platform_data->irq_gpio);

	cclogic_power_on(cclogic_dev, false);

	cclogic_regulator_configure(cclogic_dev, false);

	return 0;
}

#ifdef CONFIG_PM
/*
 *
 */
static int cclogic_suspend(struct device *dev)
{
	struct cclogic_dev *cclogic_dev = dev_get_drvdata(dev);

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	//cclogic_power_on(cclogic_dev, false);
	disable_irq(cclogic_dev->irq);
	return 0;
}

/*
 *
 */
static int cclogic_resume(struct device *dev)
{
	struct cclogic_dev *cclogic_dev = dev_get_drvdata(dev);

	pr_debug("[%s][%d]\n", __func__, __LINE__);

	//cclogic_power_on(cclogic_dev, true);
	enable_irq(cclogic_dev->irq);
	return 0;
}
#else
#define cclogic_suspend NULL
#define cclogic_resume NULL
#endif

static const struct dev_pm_ops cclogic_pm_ops = {
	.suspend		= cclogic_suspend,
	.resume			= cclogic_resume,
};

static const struct i2c_device_id cclogic_id_table[] = {
	{DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, cclogic_id_table);

#ifdef CONFIG_OF
static struct of_device_id cclogic_match_table[] = {
	{ .compatible = "typec,cclogic", },
	{ },
};
#else
#define cclogic_match_table NULL
#endif

static struct i2c_driver cclogic_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm     = &cclogic_pm_ops,
		.of_match_table = cclogic_match_table,
	},
	.probe	 = cclogic_probe,
	.remove	 = __devexit_p(cclogic_remove),
	.id_table = cclogic_id_table,
};


static int __init cclogic_init(void)
{
	pr_debug("[%s][%d]\n", __func__, __LINE__);

	mutex_init(&cclogic_list_lock);
	mutex_init(&cclogic_ops_lock);
	INIT_LIST_HEAD(&cclogic_dev_list);

	cclogic_priv = kzalloc(sizeof(*cclogic_priv), GFP_KERNEL);
	if (cclogic_priv == NULL) {
		pr_err("%s-->failed to allocate memory for module data\n",
				__func__);
		return -ENOMEM;
	}

	return i2c_add_driver(&cclogic_driver);
}

static void __exit cclogic_exit(void)
{
	pr_debug("[%s][%d]\n", __func__, __LINE__);

	i2c_del_driver(&cclogic_driver);

	if(cclogic_priv)
		kfree(cclogic_priv);
	cclogic_priv = NULL;

	return;
}

module_init(cclogic_init);
module_exit(cclogic_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Yang ShaoYing <yangsy2@lenovo.com>");
MODULE_DESCRIPTION("Drivers core for CC-Logic");
MODULE_ALIAS("platform:cc-logic");
