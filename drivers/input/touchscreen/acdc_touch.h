/*
 * Copyright (c) 2016-2022 ACDC Co., Ltd. All rights reserved.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _ACDC_TOUCH_H_
#define _ACDC_TOUCH_H_

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define ACDC_TOUCH_DRIVER_VERSION			"ACDC Touch I2C Driver V1.7.0"

#define DEBUG_TOUCH		 					0   /* full log can be displayed */

#define DBG_PRINT(fmt, args...) \
	do { if (DEBUG_TOUCH) { \
		printk(KERN_INFO "[acdc_touch]: " fmt, ## args); \
	} } while (0)

#define DBG_PRINT_ALWAYS(fmt, args...) \
	do { \
		printk(KERN_INFO "[acdc_touch]: " fmt, ## args); \
	} while (0)

#define ACDC_TOUCH_NAME						"acdc_touch"
#define ACDC_TOUCH_I2C_SLAVE_ADDR			(0x3c)


#define ACDC_TOUCH_FINGER_MAX				10
#define ACDC_TOUCH_KEY_MAX					0

#define I2C_RETRY_COUNT						0   /* I2C Retry Count */

#define X_REVERSE_ENABLE					0
#define Y_REVERSE_ENABLE					0
#define XY_CHANGE_ENABLE					0

#define TOUCH_WAKEUP_ENABLE					0
#define TOUCH_TOOL_COMMUNICATION_ENABLE		1
#define TOUCH_RECOVER_TIME					3

//--------------------------------------------
// Touch Interrupt mode define
//--------------------------------------------
#define	IRQ_MODE_THREAD						0
#define	IRQ_MODE_NORMAL						1
#define	IRQ_MODE_POLLING					2

#define ACDC_TOUCH_POINT_FW_MAX				10

#define UNUSED_GPIO							(-1)
#define GPIO_FOR_TS_IRQ						UNUSED_GPIO /* the number of GPIO for TSC interrupt */
#define GPIO_FOR_TS_RESETB					UNUSED_GPIO

typedef unsigned char			u8;
typedef unsigned short			u16;
typedef unsigned int			u32;
typedef unsigned long long		u64;
typedef signed char				s8;
typedef short					s16;
typedef int						s32;
typedef long long				s64;

typedef enum
{
	DEVICE_TYPE_ERROR							= -1,
	DEVICE_TYPE_UNKNOW							= 0,
	DEVICE_TYPE_AM126B							= 2,
	DEVICE_TYPE_AM84B							= 3,
	DEVICE_TYPE_AM127B							= 4,
}DEVICE_TYPE_e;

typedef struct FINGER {
	u32 event;							// ts event type
	u32 id;								// ts received id
	u32 x;								// ts data x
	u32 y;								// ts data y
	u32 area;							// ts finger area
	u32 pressure;						// ts finger pressure
} __attribute__ ((packed)) finger_t;

struct acdc_ts_data {
	s32							irq;
	struct i2c_client			*client;
	struct touch_pdata			*pdata;
	struct input_dev			*input;
	char						phys[32];

	finger_t					*finger;
	struct mutex				mutex;

	DEVICE_TYPE_e				device_type;
	u32							fw_version_addr;
	u32							touch_package_len_addr;
	u32							touch_data_base_addr;

	struct workqueue_struct		*work_queue;
	struct work_struct			interrupt_work;
	struct delayed_work 		recover_work;
	u32							touch_recover_bit;

#if TOUCH_WAKEUP_ENABLE
	struct wake_lock			acdc_wake_lock;
#if defined(CONFIG_FB)
	struct notifier_block		fb_notif;
#endif
#endif

	bool						system_suspend;
};

struct touch_pdata {
	char						*name;
	s32  						irq_gpio;
	s32  						reset_gpio;	
	s32							irq_mode;
	s32							irq_flags;
	
	s32							abs_min_x;
	s32							abs_min_y;
	s32							abs_max_x;
	s32							abs_max_y;

	s32							area_min;
	s32							area_max;
	s32							press_min;
	s32							press_max;

	s32							id_min;
	s32							id_max;

	s32							max_fingers;

	s32							*keycode;
	s32							keycnt;

	void (*touch_work)			(struct acdc_ts_data *ts);
	irqreturn_t (*irq_func)		(s32 irq, void *handle);


	s32 (*i2c_write)			(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len);
	s32 (*i2c_read)				(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len);
};

typedef struct _hid_finger_t {
	u8 touch_event;
	u8 touch_id;	
	u16 x;									/* 0 to MAX_X_POS */
	u16 y;									/* 0 to MAX_Y_POS */	
} __attribute__((packed)) hid_finger_t;

typedef struct _hid_multi_touch_t {
	uint16_t touch_num;
	hid_finger_t fingers[ACDC_TOUCH_POINT_FW_MAX];
} __attribute__((packed)) hid_multi_touch_t;

#if TOUCH_TOOL_COMMUNICATION_ENABLE

typedef struct _I2C_IF_CTX_ {
    u8 seqNum;
    u8 dir;
    u32 addr;
    u16 len;
	u8 checksum;
} __attribute__((packed)) i2c_if_ctx_t;

#endif

#endif
