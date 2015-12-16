/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/cdev.h>

#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/types.h>
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <linux/poll.h>

#include "fpc1020_common.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fingerprint Cards AB <tech@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 touch sensor driver.");

DECLARE_WAIT_QUEUE_HEAD(fnger_detect_wq);
static int fpc1020_device_count;
#define FPC1020_DBG

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
static ssize_t fpc1020_store_attr_setup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t fpc1020_show_attr_setup(struct device *dev, struct device_attribute *attr, char *buf);

static int __devinit fpc1020_create_device(fpc1020_data_t *fpc1020);

#define FNGR_DETECT     KEY_FNGR_DETECT
#define FPC1020_RESET_RETRIES			2
#define FPC1020_RESET_LOW_US			1000
#define FPC1020_RESET_HIGH1_US			100
#define FPC1020_RESET_HIGH2_US			1250
#define DEVFS_SETUP_MODE (S_IWUSR|S_IWGRP|S_IRUSR|S_IRGRP|S_IROTH)

#define FPC1020_ATTR(__grp, __field, __mode)				\
{									\
	.attr = __ATTR(__field, (__mode),				\
	fpc1020_show_attr_##__grp,					\
	fpc1020_store_attr_##__grp),					\
	.offset = offsetof(struct fpc1020_##__grp, __field)		\
}
struct fpc1020_attribute {
	struct device_attribute attr;
	size_t offset;
};

#define FPC1020_DEV_ATTR(_grp, _field, _mode)				\
struct fpc1020_attribute fpc1020_attr_##_field =			\
					FPC1020_ATTR(_grp, _field, (_mode))

static FPC1020_DEV_ATTR(setup, enable_navi, DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, irq, DEVFS_SETUP_MODE);
static FPC1020_DEV_ATTR(setup, wakeup, DEVFS_SETUP_MODE);

static struct attribute *fpc1020_setup_attrs[] = {
    &fpc1020_attr_enable_navi.attr.attr,
    &fpc1020_attr_irq.attr.attr,
    &fpc1020_attr_wakeup.attr.attr,
	NULL
};

static const struct attribute_group fpc1020_setup_attr_group = {
	.attrs = fpc1020_setup_attrs,
	.name = "setup"
};

volatile int spi_clk_access_cnt = 0;


int fpc1020_hw_reset(fpc1020_data_t *fpc1020)
{
    dev_warn(&fpc1020->spi->dev, "%s\n", __func__);
    //hard reset FPC1150A
    fpc1020_gpio_reset(fpc1020);
    mdelay(50);
    return 0;
}
static ssize_t fpc1020_show_attr_setup(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct fpc1020_attribute *fpc_attr;
	int val = -1;
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);
	fpc_attr = container_of(attr, struct fpc1020_attribute, attr);
    if (fpc_attr->offset == offsetof(fpc1020_setup_t, enable_navi))
        val = spi_clk_access_cnt;
    if (fpc_attr->offset == offsetof(fpc1020_setup_t, wakeup))
        val = fpc1020->wakeup_status;
    if (fpc_attr->offset == offsetof(fpc1020_setup_t, irq))
        val = fpc1020->irq_status;
    if (val >= 0)
        return scnprintf(buf, PAGE_SIZE, "%i\n", val);
	return -ENOENT;
}

static ssize_t fpc1020_store_attr_setup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    u64 val;
    struct fpc1020_attribute *fpc_attr;
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);
    int error = kstrtou64(buf, 0, &val);
	fpc_attr = container_of(attr, struct fpc1020_attribute, attr);
    if (!error) {
        if (fpc_attr->offset == offsetof(fpc1020_setup_t, enable_navi)) {
            if (val == 0) {
                if (spi_clk_access_cnt == 0) {
                    spi_clk_access_cnt = 1;
                    clk_prepare_enable(fpc1020->core_clk);
                    clk_prepare_enable(fpc1020->iface_clk);
                }
            } else if (val == 1) {
                if (spi_clk_access_cnt == 1) {
                    spi_clk_access_cnt = 0;
                    clk_disable_unprepare(fpc1020->core_clk);
                    clk_disable_unprepare(fpc1020->iface_clk);
                }
            }
        } else if (fpc_attr->offset == offsetof(fpc1020_setup_t, irq)) {
            if (val == 0 && fpc1020->irq_status == 1) {
                dev_info(&fpc1020->spi->dev, "IRQ disable\n");
                disable_irq(fpc1020->irq);
                fpc1020->irq_status = 0;
            } else if (val == 1 && fpc1020->irq_status == 0) {
                dev_info(&fpc1020->spi->dev, "IRQ enable\n");
                enable_irq(fpc1020->irq);
                fpc1020->irq_status = 1;
            }
        } else if (fpc_attr->offset == offsetof(fpc1020_setup_t, wakeup)) {
            dev_info(&fpc1020->spi->dev, "Modify wakeup status(%d)\n", (int)val);
            if (val != fpc1020->wakeup_status) {
                fpc1020->wakeup_status = (int)val;
                (fpc1020->wakeup_status == 1) ? enable_irq_wake(fpc1020->irq) : disable_irq_wake(fpc1020->irq);
            }
        } else
            return -ENOENT;
        return strnlen(buf, count);
    }
    return error;
}

#ifdef CONFIG_OF
static struct of_device_id fpc1020_of_match[] __devinitdata = {
	{ .compatible = "fpc,fpc1020", },
	{}
};

MODULE_DEVICE_TABLE(of, fpc1020_of_match);
#endif

bool irq_flag = false;
irqreturn_t fpc1020_interrupt(int irq, void *_fpc1020)
{
    fpc1020_data_t *fpc1020 = _fpc1020;

    if (gpio_get_value(fpc1020->irq_gpio)) {
        //dev_info(&fpc1020->spi->dev, "%s : Valided interrupt\n", __func__);
        irq_flag = true;
        if (!wake_lock_active(&fpc1020->wake_lock)) 
            wake_lock_timeout(&fpc1020->wake_lock, 5*HZ);
        wake_up_interruptible(&fnger_detect_wq);
        return IRQ_HANDLED;
    } else {
        return IRQ_NONE;
    }
}

#define FPC1020_CLASS_NAME "fpc1020"
static int __devinit fpc1020_create_class(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	fpc1020->class = class_create(THIS_MODULE, FPC1020_CLASS_NAME);

	if (IS_ERR(fpc1020->class)) {
		dev_err(&fpc1020->spi->dev, "failed to create class.\n");
		error = PTR_ERR(fpc1020->class);
	}

	return error;
}

static int __devinit fpc1020_create_device(fpc1020_data_t *fpc1020)
{
	int error = 0;

	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if (FPC1020_MAJOR > 0) {
		fpc1020->devno = MKDEV(FPC1020_MAJOR, fpc1020_device_count++);

		error = register_chrdev_region(fpc1020->devno,
						1,
						FPC1020_DEV_NAME);
	} else {
		error = alloc_chrdev_region(&fpc1020->devno,
					fpc1020_device_count++,
					1,
					FPC1020_DEV_NAME);
	}

	if (error < 0) {
		dev_err(&fpc1020->spi->dev,
				"%s: FAILED %d.\n", __func__, error);
		goto out;

	} else {
		dev_info(&fpc1020->spi->dev, "%s: major=%d, minor=%d\n",
						__func__,
						MAJOR(fpc1020->devno),
						MINOR(fpc1020->devno));
	}

	fpc1020->device = device_create(fpc1020->class, NULL, fpc1020->devno,
						NULL, "%s", FPC1020_DEV_NAME);

	if (IS_ERR(fpc1020->device)) {
		dev_err(&fpc1020->spi->dev, "device_create failed.\n");
		error = PTR_ERR(fpc1020->device);
	}
out:
	return error;
}
static int fpc1020_manage_sysfs(fpc1020_data_t *fpc1020,
        struct spi_device *spi, bool create)
{
    int error = 0;
    if (create) {
        dev_info(&fpc1020->spi->dev, "%s create\n", __func__);
        error = sysfs_create_group(&spi->dev.kobj,
                &fpc1020_setup_attr_group);
        if (error) {
			sysfs_remove_group(&spi->dev.kobj,
					&fpc1020_setup_attr_group);
            dev_err(&fpc1020->spi->dev,
                    "sysf_create_group failed.\n");
            return error;
        }
    }
    return error;
}

static int __devinit fpc1020_get_of_pdata(struct device *dev,
					struct fpc1020_platform_data *pdata)
{
	const struct device_node *node = dev->of_node;
	/* required properties */
	const void *irq_prop = of_get_property(node, "fpc,gpio_irq",   NULL);
	const void *rst_prop = of_get_property(node, "fpc,gpio_reset", NULL);
	const void *cs_prop  = of_get_property(node, "fpc,gpio_cs",    NULL);

	/* optional properties */
	const void *vddtx_prop = of_get_property(node, "fpc,vddtx_mv", NULL);
	const void *boost_prop =
			of_get_property(node, "fpc,txout_boost_enable", NULL);

	if (node == NULL) {
		dev_err(dev, "%s: Could not find OF device node\n", __func__);
		goto of_err;
	}

	if (!irq_prop || !rst_prop || !cs_prop) {
		dev_err(dev, "%s: Missing OF property\n", __func__);
		goto of_err;
	}

	pdata->irq_gpio   = be32_to_cpup(irq_prop);
	pdata->reset_gpio = be32_to_cpup(rst_prop);
	pdata->cs_gpio    = be32_to_cpup(cs_prop);

	pdata->external_supply_mv =
			(vddtx_prop != NULL) ? be32_to_cpup(vddtx_prop) : 0;

	pdata->txout_boost = (boost_prop != NULL) ? 1 : 0;

	return 0;

of_err:
	pdata->reset_gpio = -EINVAL;
	pdata->irq_gpio   = -EINVAL;
	pdata->cs_gpio    = -EINVAL;

	pdata->external_supply_mv = 0;
	pdata->txout_boost = 0;

	return -ENODEV;
}

static int __devinit fpc1020_irq_init(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata)
{
	int error = 0;

	if (gpio_is_valid(pdata->irq_gpio)) {

		dev_info(&fpc1020->spi->dev,
			"Assign IRQ -> GPIO%d\n",
			pdata->irq_gpio);
		error = gpio_request(pdata->irq_gpio, "fpc1020_irq");

		if (error) {
			dev_err(&fpc1020->spi->dev,
				"gpio_request (irq) failed.\n");

			return error;
		}

		fpc1020->irq_gpio = pdata->irq_gpio;

		error = gpio_direction_input(fpc1020->irq_gpio);

		if (error) {
			dev_err(&fpc1020->spi->dev,
				"gpio_direction_input (irq) failed.\n");
			return error;
		}
	} else {
		return -EINVAL;
	}

	fpc1020->irq = gpio_to_irq(fpc1020->irq_gpio);
	if (fpc1020->irq < 0) {
		dev_err(&fpc1020->spi->dev, "gpio_to_irq failed.\n");
		error = fpc1020->irq;
		return error;
	}
	error = request_irq(fpc1020->irq, fpc1020_interrupt,
			IRQF_TRIGGER_RISING, "fpc1020", fpc1020);
	if (error) {
		dev_err(&fpc1020->spi->dev,
			"request_irq %i failed.\n",
			fpc1020->irq);

		fpc1020->irq = -EINVAL;

		return error;
	}

    wake_lock_init(&fpc1020->wake_lock, WAKE_LOCK_SUSPEND, "fpc1020_wl");
    disable_irq(fpc1020->irq);
	return error;
}


static int __devinit fpc1020_reset_init(fpc1020_data_t *fpc1020,
        struct fpc1020_platform_data *pdata)
{
    int error = 0;

    if (gpio_is_valid(pdata->reset_gpio)) {
        dev_info(&fpc1020->spi->dev,
                "Assign HW reset ->GPIO(%d)\n", pdata->reset_gpio);
		error = gpio_request(pdata->reset_gpio, "fpc1020_reset");
		if (error) {
			dev_err(&fpc1020->spi->dev,
				"gpio_request (reset) failed.\n");
			return error;
		}

		fpc1020->reset_gpio = pdata->reset_gpio;
		error = gpio_direction_output(fpc1020->reset_gpio, 1);
		if (error) {
			dev_err(&fpc1020->spi->dev,
			"gpio_direction_output(reset) failed.\n");
			return error;
		}
    } else {
        dev_info(&fpc1020->spi->dev, "RESET PIN setup failed\n");
        return -1;
    }
    return error;
}

static int fpc1020_clk_init(fpc1020_data_t *fpc1020)
{
    fpc1020->iface_clk = clk_get(&fpc1020->spi->dev, "iface_clk");
    if (IS_ERR(fpc1020->iface_clk)) {
        clk_put(fpc1020->iface_clk);
        pr_err("Unable to get the fpc's driver iface_clk\n");
        fpc1020->iface_clk = NULL;
        return -EIO;
    }
    fpc1020->core_clk = clk_get(&fpc1020->spi->dev, "core_clk");
    if (IS_ERR(fpc1020->core_clk)) {
        clk_put(fpc1020->core_clk);
        pr_err("Unable to get fpc's core clk\n");
        fpc1020->core_clk = NULL;
        return -EIO;
    }
    return 0;
}



#define FPC1020_TOUCH_PAD_DEV_NAME              "fpc1020tp"

int __devinit fpc1020_input_init(fpc1020_data_t *fpc1020)
{
    int error = 0;
    dev_info(&fpc1020->spi->dev, "%s\n", __func__);
    fpc1020->input_dev = input_allocate_device();
    if (!fpc1020->input_dev) {
        dev_err(&fpc1020->spi->dev, "Input_allocate_device failed.\n");
        error = -ENOMEM;
    }

    if (!error) {
        fpc1020->input_dev->name = FPC1020_TOUCH_PAD_DEV_NAME;
		set_bit(EV_KEY, fpc1020->input_dev->evbit);
		set_bit(FNGR_DETECT, fpc1020->input_dev->keybit);
		input_set_capability(fpc1020->input_dev, EV_KEY, FNGR_DETECT);
		/* Register the input device */
		error = input_register_device(fpc1020->input_dev);
		if (error) {
			dev_err(&fpc1020->spi->dev, "Input_register_device failed.\n");
			input_free_device(fpc1020->input_dev);
			fpc1020->input_dev = NULL;
		}
    }
    return error;
}

void __devexit fpc1020_input_destroy(fpc1020_data_t *fpc1020)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	if (fpc1020->input_dev != NULL)
		input_free_device(fpc1020->input_dev);
}

static int fpc1020_cleanup(fpc1020_data_t *fpc1020, struct spi_device *spidev)
{
	dev_dbg(&fpc1020->spi->dev, "%s\n", __func__);

	//class_destroy(fpc1020->class);

	if (fpc1020->irq >= 0)
		free_irq(fpc1020->irq, fpc1020);

	if (gpio_is_valid(fpc1020->irq_gpio))
		gpio_free(fpc1020->irq_gpio);

	if (gpio_is_valid(fpc1020->reset_gpio))
		gpio_free(fpc1020->reset_gpio);

    if (gpio_is_valid(fpc1020->cs_gpio))
		gpio_free(fpc1020->reset_gpio);

    wake_lock_destroy(&fpc1020->wake_lock);

	fpc1020_input_destroy(fpc1020);

	kfree(fpc1020);

	spi_set_drvdata(spidev, NULL);

	return 0;
}

static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    int *blank;
    struct fb_event *evdata = data;
    fpc1020_data_t *fpc1020 = container_of(self, fpc1020_data_t, fb_notif);

    blank = evdata->data;
    if (evdata && evdata->data && event == FB_EVENT_BLANK && fpc1020) {
        blank = evdata->data;

        if (*blank == FB_BLANK_UNBLANK) {
            dev_warn(&fpc1020->spi->dev, "ScreenOn\n");
            fpc1020->report_key_flag = true;
        } else if (*blank == FB_BLANK_POWERDOWN) {
            dev_warn(&fpc1020->spi->dev, "ScreenOff\n");
            fpc1020->report_key_flag = false;
        }
    }
    return 0;
}

void fpc1020_report_work_func(struct work_struct *work)
{
    fpc1020_data_t *fpc1020 = NULL;
    fpc1020 = container_of(work, fpc1020_data_t, input_report_work);
    if (fpc1020->report_key_flag == true) {
        dev_warn(&fpc1020->spi->dev, "FNGR_DETECT\n");
        //wake_up_interruptible(&fnger_detect_wq);
        input_report_key(fpc1020->input_dev, fpc1020->report_key, 1);
        input_sync(fpc1020->input_dev);
        input_report_key(fpc1020->input_dev, fpc1020->report_key, 0);
        input_sync(fpc1020->input_dev);
    }
}

static int __devinit fpc1020_spi_setup(fpc1020_data_t *fpc1020,
					struct fpc1020_platform_data *pdata)
{
	int error = 0;

	fpc1020->spi->mode = SPI_MODE_0;
	fpc1020->spi->bits_per_word = 8;
    fpc1020->spi->chip_select = 0;

	error = spi_setup(fpc1020->spi);

	if (error) {
		dev_err(&fpc1020->spi->dev, "spi_setup failed\n");
		goto out_err;
	}

#ifdef CONFIG_ARCH_MSM
	if (gpio_is_valid(pdata->cs_gpio)) {

		dev_info(&fpc1020->spi->dev,
			"Assign SPI.CS -> GPIO%d\n",
			pdata->cs_gpio);

		error = gpio_request(pdata->cs_gpio, "fpc1020_cs");
		if (error) {
			dev_err(&fpc1020->spi->dev,
				"gpio_request (cs) failed.\n");

			goto out_err;
		}

		fpc1020->cs_gpio = pdata->cs_gpio;

		error = gpio_direction_output(fpc1020->cs_gpio, 1);
		if (error) {
			dev_err(&fpc1020->spi->dev,
				"gpio_direction_output(cs) failed.\n");
			goto out_err;
		}
	} else {
		error = -EINVAL;
	}
#endif

out_err:
	return error;
}

static int fpc1020_open(struct inode *inode, struct file *file)
{
    fpc1020_data_t *fpc1020;
    fpc1020 = container_of(inode->i_cdev, fpc1020_data_t, cdev);
    //dev_info(&fpc1020->spi->dev, "%s\n", __func__);
    file->private_data = fpc1020;
    return 0;
}

static ssize_t fpc1020_write(struct file *file, const char *buff,
					size_t count, loff_t *ppos)
{
	printk(KERN_INFO "%s\n", __func__);

	return -ENOTTY;
}

static ssize_t fpc1020_read(struct file *file, char *buff,
				size_t count, loff_t *ppos)
{
    printk(KERN_INFO "%s\n", __func__);
    return -ENOENT;
}


static int fpc1020_release(struct inode *inode, struct file *file)
{

    //printk("%s\n", __func__);
    return -ENOENT;
}

static unsigned int fpc1020_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	fpc1020_data_t *fpc1020 = file->private_data;
    poll_wait(file, &fnger_detect_wq, wait);
    if (irq_flag == true) {
	if (fpc1020->report_key_flag == false && fpc1020->wakeup_status == 0) {
	    dev_info(&fpc1020->spi->dev, "%s, ScreenOff&WakeupDisable\n", __func__);
	    mask = POLLNVAL;
	} else {
	    dev_warn(&fpc1020->spi->dev, "%s, FNGR_DETECT\n", __func__);
	    mask = POLLIN|POLLRDNORM;
	}
	irq_flag = false;
    }
    return mask;
}

static const struct file_operations fpc1020_fops = {
	.owner          = THIS_MODULE,
	.open           = fpc1020_open,
	.write          = fpc1020_write,
	.read           = fpc1020_read,
	.release        = fpc1020_release,
	.poll           = fpc1020_poll,

};

static int __devinit fpc1020_probe(struct spi_device *spi)
{
    int error = 0;
    struct fpc1020_platform_data *fpc1020_pdata;
    struct fpc1020_platform_data pdata_of;
    struct device *dev = &spi->dev;
    fpc1020_data_t *fpc1020 = NULL;
    fpc1020 = kzalloc(sizeof(*fpc1020), GFP_KERNEL);
    if (!fpc1020) {
        dev_err(&spi->dev,
                "failed to allocate memory for fpc1020_data\n");
        return -ENOMEM;
    }
    spi_set_drvdata(spi, fpc1020);
    fpc1020->spi = spi;
    fpc1020->reset_gpio = -EINVAL;
    fpc1020->irq_gpio = -EINVAL;
    fpc1020->irq = -EINVAL;
    fpc1020->cs_gpio = -EINVAL;
    fpc1020->wakeup_status = 1;  //Default enable IRQ wakeup
    fpc1020->irq_status = 0;    //Default disable IRQ

    fpc1020_pdata = spi->dev.platform_data;
    if (!fpc1020_pdata) {
        error = fpc1020_get_of_pdata(dev, &pdata_of);
		fpc1020_pdata = &pdata_of;
        if (error)
            goto err;
    }

	if (!fpc1020_pdata) {
		dev_err(&fpc1020->spi->dev,
				"spi->dev.platform_data is NULL.\n");
		error = -EINVAL;
		goto err;
	}

    dev_info(&fpc1020->spi->dev, "InputDevice allocating....\n");
    error = fpc1020_input_init(fpc1020);
    if (error)
        goto err;

    fpc1020->fpc1020_wq = create_workqueue("fpc1020_wq");
    if (!fpc1020->fpc1020_wq) {
        dev_err(&fpc1020->spi->dev, "Create input workqueue failed\n");
        error = -ENOMEM;
        goto err;
    }
    INIT_WORK(&fpc1020->input_report_work, fpc1020_report_work_func);

    error = fpc1020_manage_sysfs(fpc1020, spi, true);
    if (error)
        goto err;
    error = fpc1020_clk_init(fpc1020);
    if (error)
        goto err;

    error = fpc1020_reset_init(fpc1020, fpc1020_pdata);
    if (error)
        goto err;
    error = fpc1020_irq_init(fpc1020, fpc1020_pdata);
    if (error)
        goto err;
    error = fpc1020_spi_setup(fpc1020, fpc1020_pdata);
	if (error)
		goto err;
    error = fpc1020_gpio_reset(fpc1020);
    if (error)
        goto err;

    error = fpc1020_check_hw_id(fpc1020);
    if (error)
        goto err;

    error = fpc1020_setup_defaults(fpc1020);
    if (error)
        goto err;

    error = fpc1020_write_sensor_1150_setup(fpc1020);
    if (error) {
        dev_err(&fpc1020->spi->dev, "write sensor 1150 setup failed\n");
        goto err;
    } else {
        dev_err(&fpc1020->spi->dev, "Write sensor success\n");
    }
    error = fpc1020_create_class(fpc1020);
    if (error)
        goto err;

    error = fpc1020_create_device(fpc1020);
    if (error)
        goto err;

    /*Init cdev*/
    cdev_init(&fpc1020->cdev, &fpc1020_fops);
    fpc1020->cdev.owner = THIS_MODULE;
    error = cdev_add(&fpc1020->cdev, fpc1020->devno, 1);
    if (error) {
        dev_err(&fpc1020->spi->dev, "cdev_add failed.\n");
        cdev_del(&fpc1020->cdev);
        unregister_chrdev_region(fpc1020->devno, 1);
        fpc1020_manage_sysfs(fpc1020, spi, false);
        goto err;
    }
    fpc1020->fb_notif.notifier_call = fb_notifier_callback;
    error = fb_register_client(&fpc1020->fb_notif);
    if (error) {
        dev_err(&fpc1020->spi->dev,
                "Unable to register fb_notifier:%d\n", error);
        goto err;
    }

    if (fpc1020->wakeup_status) {
	enable_irq_wake(fpc1020->irq);
    }

    return 0;

err:
    fpc1020_cleanup(fpc1020, spi);
    return error;

}

static int __devexit fpc1020_remove(struct spi_device *spi)
{
	fpc1020_data_t *fpc1020 = spi_get_drvdata(spi);

	printk(KERN_INFO "%s\n", __func__);
	fpc1020_cleanup(fpc1020, spi);
	return 0;
}

static int fpc1020_suspend(struct device *dev)
{
    fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);
    dev_info(&fpc1020->spi->dev, "%s\n", __func__);
    if (spi_clk_access_cnt == 1) {
        dev_warn(&fpc1020->spi->dev, "Disable iface&core clk\n");
        spi_clk_access_cnt = 0;
        clk_disable_unprepare(fpc1020->core_clk);
        clk_disable_unprepare(fpc1020->iface_clk);
    }
    if (fpc1020->wakeup_status == 1) {
        dev_info(&fpc1020->spi->dev, "Enable fingerIRQ wakeup!\n");
        //if (fpc1020->irq_status == 0) {
        //    enable_irq(fpc1020->irq);
        //}
    }

    return 0;
}

static int fpc1020_resume(struct device *dev)
{
	fpc1020_data_t *fpc1020 = dev_get_drvdata(dev);
	dev_info(&fpc1020->spi->dev, "%s\n", __func__);
    return 0;
}

#define FPC1020_DEV_NAME                        "fpc1020"
static const struct dev_pm_ops fpc1020_pm = {
    .suspend = fpc1020_suspend,
    .resume = fpc1020_resume
};

static struct spi_driver fpc1020_driver = {
    .driver = {
        .name   = FPC1020_DEV_NAME,
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
        .pm     = &fpc1020_pm,
#ifdef CONFIG_OF
		.of_match_table = fpc1020_of_match,
#endif
	},
	.probe	= fpc1020_probe,
	.remove	= __devexit_p(fpc1020_remove)
};

static int __init fpc1020_init(void)
{
	if (spi_register_driver(&fpc1020_driver))
		return -EINVAL;

	return 0;
}

static void __exit fpc1020_exit(void)
{
	printk(KERN_INFO "%s\n", __func__);

	spi_unregister_driver(&fpc1020_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);
