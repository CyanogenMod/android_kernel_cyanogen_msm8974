#ifndef __CCLOGIC_COMMON_H
#define __CCLOGIC_COMMON_H 

#include <linux/switch.h>

//#define DEBUG

#ifdef DEBUG
#undef	pr_debug
#undef  pr_info
#define pr_debug(fmt, args...) pr_err(fmt, ##args); 
#define pr_info(fmt, args...) pr_err(fmt, ##args); 
#endif


//#define CCLOGIC_DEBUG_TUSB320      /* see also: arch/arm/mach-msm/board-8974-gpiomux.c */

#define CCLOGIC_ENABLE_IRQ
#define CCLOGIC_ENABLE_WORK

//#define CCLOGIC_UPDATE_STATUS

#define CCLOGIC_I2C_VTG_MIN_UV		1800000
#define CCLOGIC_I2C_VTG_MAX_UV		1800000
#define CCLOGIC_I2C_LOAD_UA      	1800

#define CCLOGIC_MAX_SUPPORT_CHIP    2

struct cclogic_of_chip {
	const char * chip_name;
	int  address;
};


struct cclogic_platform {
	unsigned int irq_flags;
	unsigned int irq_gpio;
	bool 	     i2c_pull_up;
	unsigned int function_switch_gpio1;
	unsigned int function_switch_gpio2;
	unsigned int usb_ss_gpio;
	unsigned int enb_gpio;
	int	     chip_num;
	struct cclogic_of_chip chip[CCLOGIC_MAX_SUPPORT_CHIP];
};


enum cclogic_attached_type {
	CCLOGIC_DEVICE_UNKNOWN,
	CCLOGIC_NO_DEVICE,
	CCLOGIC_USB_DEVICE,
	CCLOGIC_USB_HOST,
	CCLOGIC_DEBUG_DEVICE,
	CCLOGIC_AUDIO_DEVICE,
};

enum cclogic_current_type {
	CCLOGIC_CURRENT_NONE,
	CCLOGIC_CURRENT_DEFAULT,
	CCLOGIC_CURRENT_MEDIUM,
	CCLOGIC_CURRENT_ACCESSORY,
	CCLOGIC_CURRENT_HIGH,
};

enum cclogic_event_type {
	CCLOGIC_EVENT_NONE,
	CCLOGIC_EVENT_DETACHED,
	CCLOGIC_EVENT_ATTACHED,
};

enum cclogic_cc_type {
	CCLOGIC_CC_UNKNOWN,
	CCLOGIC_CC1,
	CCLOGIC_CC2,	
};

struct cclogic_state {
	bool vbus;
	enum cclogic_cc_type  cc;
	enum cclogic_event_type evt;
	enum cclogic_attached_type device;
	enum cclogic_current_type charger;
};

struct cclogic_chip;
struct cclogic_dev	{
	struct device		*dev;
	struct i2c_client	*i2c_client;
	unsigned int		irq;
	bool				irq_enabled;
	struct regulator 	*vcc_i2c;
	bool 				regulator_en;
	struct delayed_work	work;
	struct cclogic_platform *platform_data;
	struct wake_lock 	wakelock;
	bool				vbus_on;
	struct cclogic_chip *ops;
	struct cclogic_state state;
	struct switch_dev sdev;
	bool shenqi_audio;
};

struct cclogic_chip {
	const char * chip_name;
	unsigned char addr;
    int (*get_state)(struct i2c_client *client,struct cclogic_state *result);
	int (*ack_irq)(struct i2c_client *client);
	int (*chip_config)(struct i2c_client *client);
	int (*chip_reset)(struct i2c_client *client);
	int (*chip_check)(struct i2c_client *client);
	struct list_head  chip_list;
};

enum cclogic_func_type {
	CCLOGIC_FUNC_HIZ,
	CCLOGIC_FUNC_USB,
	CCLOGIC_FUNC_AUDIO,
	CCLOGIC_FUNC_UART,
};



extern int cclogic_register(struct cclogic_chip *c);
extern void cclogic_unregister(struct cclogic_chip *c);



extern void otg_func_set(bool enable);

#endif
