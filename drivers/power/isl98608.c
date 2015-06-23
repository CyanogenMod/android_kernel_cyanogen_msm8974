#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>

#include "mdss_mdp.h"

static struct of_device_id intersil_i2c_table [] = {
    { . compatible = "intersil,isl98608" ,}, // Compatible node must match dts
    { },
};


static const struct i2c_device_id intersil_id [] = {
    { " intersil_i2c ", 0 },
    { }
};

static int intersil_2c_transfer ( struct i2c_client * client )
{
    u8 vbst[] = {0x06,0x04}; // vbus voltage, default +4
    u8 vn[] = {0x08,0x04};
    u8 vp[] = {0x09,0x04};

    struct i2c_msg msgs[3] = {
        {
             .addr = client->addr,
             .flags = 0,
             .len = 2,
             .buf = vbst,
        }, {
             .addr = client->addr,
             .flags = 0,
             .len = 2,
             .buf = vn,
        }, {
             .addr = client->addr,
             .flags = 0,
             .len = 2,
             .buf = vp,
        }
    };

    return i2c_transfer(client->adapter, msgs, 3);
}
//extern int lcdmodule;
static int __devinit intersil_i2c_probe ( struct i2c_client *client ,const struct i2c_device_id *id)
{
#if 0
    int irq_gpio = -1;
    int irq;
    int addr ;
    // Parse data using dt.
    if( client->dev.of_node ){
    irq_gpio = of_get_named_gpio_flags(client->dev.of_node , "intersil_i2c,irq-gpio", 0, NULL );
    }

    irq = client->irq; // GPIO irq #. already converted to gpio_to_irq
    addr = client->addr ;
    dev_err(&client->dev , " gpio  [%d] irq  [%d]  gpio_irq  [%d]  Slaveaddr  [%x] \n",
        irq_gpio , irq ,gpio_to_irq (irq_gpio), addr );
if(lcdmodule == 2){
    printk(KERN_ERR "%s write intersil register\n",__func__);
}
#endif
    struct mdss_data_type *mdata;
    struct mdss_panel_cfg *pan_cfg;

    mdata = mdss_mdp_get_mdata();
    if (!mdata) {
        pr_info("REI ERR\n");
        pr_err("%s: Cannot get panel data\n", __func__);
        goto end;
    }

    pan_cfg = &mdata->pan_cfg;
    if (!pan_cfg) {
        pr_info("REI ERR\n");
        pr_err("%s: Cannot get panel configuration\n", __func__);
        goto end;
    }

    if (strstr(pan_cfg->arg_cfg, "nt35532")) {
        intersil_2c_transfer ( client );
    }
end:
    return 0;
}

static struct i2c_driver intersil_i2c_driver = {
    .driver = {
    .name = " intersil_i2c ",
    .owner = THIS_MODULE ,
    .of_match_table = intersil_i2c_table ,
    },
    .probe = intersil_i2c_probe ,
    .id_table = intersil_id ,
};

module_i2c_driver (intersil_i2c_driver);
MODULE_DESCRIPTION ("INTERSIL I2C DRIVER");
MODULE_AUTHOR("Li Yupan");
MODULE_LICENSE ("GPL v2");

