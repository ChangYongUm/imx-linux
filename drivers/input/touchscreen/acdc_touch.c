/*
 * Copyright (c) 2016-2024 ACDC Co., Ltd. All rights reserved.
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <linux/version.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <asm/unaligned.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>
#include <linux/input/mt.h>
#include <acdc_touch.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif


#define SWAP_16BITS(p) \
	((((p) & 0xFF00) >> 8) | (((p) & 0x00FF) << 8))
#define SWAP_32BITS(p) \
	((((p) & 0xFF000000) >> 24) | (((p) & 0x00FF0000) >> 8) | \
	(((p) & 0x0000FF00) << 8)  | (((p) & 0x000000FF) << 24))


#define AM126B_FW_VERSION_ADDR					SWAP_32BITS(0x20000010)
#define	AM126B_TOUCH_PACKAGE_LENGTH				SWAP_32BITS(0x40010084) 
#define	AM126B_TOUCH_DATA_BASE					SWAP_32BITS(0x40010000) 
#define	AM126B_GEN_REG01_ADDR					SWAP_32BITS(0x400DB004) 

#define AM84B_FW_VERSION_ADDR					SWAP_32BITS(0x40000010)
#define	AM84B_TOUCH_PACKAGE_LENGTH				SWAP_32BITS(0x50000084) 
#define	AM84B_TOUCH_DATA_BASE					SWAP_32BITS(0x50000000) 
#define	AM84B_GEN_REG01_ADDR					SWAP_32BITS(0x500DB004)
#define	AM84B_BASE_FW_VERSION_ADDR				SWAP_32BITS(0x20007BE0) 
#define	AM84B_BASE_BL_VERSION_ADDR				SWAP_32BITS(0x200003E0) 

#define AM127B_FW_VERSION_ADDR					SWAP_32BITS(0x40000010)
#define	AM127B_TOUCH_PACKAGE_LENGTH				SWAP_32BITS(0x50000084) 
#define	AM127B_TOUCH_DATA_BASE					SWAP_32BITS(0x50000000) 
#define	AM127B_GEN_REG01_ADDR					SWAP_32BITS(0x500DB004)
#define	AM127B_BASE_FW_VERSION_ADDR				SWAP_32BITS(0x20007BE0) 
#define	AM127B_BASE_BL_VERSION_ADDR				SWAP_32BITS(0x200003E0) 

#define	TS_EVENT_UNKNOWN						0x00
#define	TS_EVENT_PRESS							0x01
#define	TS_EVENT_MOVE							0x02
#define	TS_EVENT_RELEASE						0x03

#define TOUCH_PACKAGE_LEN_MAX					(62)

static int g_touchKeycodeBuf[] = {
	KEY_MENU,
	KEY_HOME,
	KEY_BACK,
	KEY_POWER
};

static int acdc_touch_probe(struct i2c_client *client);
static void acdc_touch_remove(struct i2c_client *client);

static int acdc_touch_parse_dt(struct device *dev, struct touch_pdata *pdata);
static int acdc_init_touch_function(struct touch_pdata *pdata);
static int acdc_touch_init_gpio(struct touch_pdata *pdata);
static void acdc_touch_work_q(struct work_struct *work);
static void acdc_touch_hw_reset(struct acdc_ts_data *ts);
static int acdc_touch_get_device_type(struct acdc_ts_data *ts);
static irqreturn_t acdc_touch_irq(int irq, void *handle);
static int acdc_ts_i2c_read(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len);
static int acdc_ts_i2c_write(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len);
static void acdc_recover_work(struct work_struct *work);
static void acdc_ts_work(struct acdc_ts_data *ts);
static int acdc_get_fw_version(struct acdc_ts_data *ts);

#if TOUCH_WAKEUP_ENABLE
#if defined(CONFIG_FB)
static int acdc_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif

static void acdc_touch_suspend(struct acdc_ts_data *ts);
static void acdc_touch_resume(struct acdc_ts_data *ts);

#endif

#if TOUCH_TOOL_COMMUNICATION_ENABLE

#define ACDC_DEVICE_NODE_PERMISSON				0777

#define I2C_CMD_HW_RESET						0x53
#define	I2C_CMD_READ_DIR						0x72
#define I2C_CMD_WRITE_DIR						0x77

static i2c_if_ctx_t g_i2c_if_ctx;
static struct acdc_ts_data *p_g_ts;

static int acdc_file_open(struct inode *inode, struct file *file);
static ssize_t acdc_file_read(struct file *pFile, char __user *buf, size_t size, loff_t *pPos);
static ssize_t acdc_file_write(struct file *pFile, const char *buf, size_t size, loff_t *pPos);

static int acdc_file_open(struct inode *inode, struct file *file)
{
	DBG_PRINT("acdc_file_open\r\n");
	return 0;
}

static ssize_t acdc_file_read(struct file *pFile, char __user *buf, size_t size, loff_t *pPos)
{
	int ret = 0;
	u8 tmpbuf[256] = { 0 };
	u16 i = 0;
	u8 sum = 0;

	DBG_PRINT("acdc_file_read\r\n");
	if ((*pPos != 0) || (size <= 1)) {
		return 0;
	}

	if ((g_i2c_if_ctx.dir == I2C_CMD_READ_DIR) && (g_i2c_if_ctx.len == (size - 1)))
	{
		disable_irq(p_g_ts->irq);
		mutex_lock(&p_g_ts->mutex);
		ret = p_g_ts->pdata->i2c_read(p_g_ts->client, (u8 *)&g_i2c_if_ctx.addr, 4, tmpbuf, g_i2c_if_ctx.len);
		mutex_unlock(&p_g_ts->mutex);
		enable_irq(p_g_ts->irq);
		g_i2c_if_ctx.dir = 0;

		if (ret < 0)
		{
			return -1;
		}
	}
	else
	{
		return 0;
	}

	for (i = 0; i < g_i2c_if_ctx.len; i++)
	{
		sum += tmpbuf[i];
	}
	
	tmpbuf[g_i2c_if_ctx.len] = sum;

	*pPos += size;

	ret = copy_to_user(buf, tmpbuf, size);

	if (ret != 0)
	{
		return -1;
	}

	return g_i2c_if_ctx.len;
}

static ssize_t acdc_file_write(struct file *pFile, const char *buf, size_t size, loff_t *pPos)
{
	int ret = 0;
	u8 tmpbuf[256] = { 0 };
	i2c_if_ctx_t *p_i2c_if_ctx;
	u16 i = 0;
	u8 sum = 0;

	DBG_PRINT("acdc_file_write\r\n");
	if (size > sizeof(tmpbuf))
	{
		return -1;
	}

	ret = copy_from_user(tmpbuf, buf, size);
	if (ret != 0)
	{
		return -1;
	}
	p_i2c_if_ctx = (i2c_if_ctx_t *)tmpbuf;

	for (i = 0; i < sizeof(i2c_if_ctx_t) - 1; i++)
	{
		sum += tmpbuf[i];
	}
	for (i = sizeof(i2c_if_ctx_t); i < size; i++)
	{
		sum += tmpbuf[i];
	}
	
	if (p_i2c_if_ctx->checksum != sum)
	{
		return -1;
	}
	
	// DBG_PRINT("%d, %02X, %08X, %d\r\n", p_i2c_if_ctx->seqNum, p_i2c_if_ctx->dir, SWAP_32BITS(p_i2c_if_ctx->addr), p_i2c_if_ctx->len);

	if (p_i2c_if_ctx->dir == I2C_CMD_READ_DIR)
	{
		memcpy((u8 *)&g_i2c_if_ctx, (u8 *)p_i2c_if_ctx, sizeof(i2c_if_ctx_t));
	}
	else if (p_i2c_if_ctx->dir == I2C_CMD_WRITE_DIR)
	{
		disable_irq(p_g_ts->irq);
		mutex_lock(&p_g_ts->mutex);
		ret = p_g_ts->pdata->i2c_write(p_g_ts->client, (u8 *)&p_i2c_if_ctx->addr, 4, &tmpbuf[sizeof(i2c_if_ctx_t)], p_i2c_if_ctx->len);
		mutex_unlock(&p_g_ts->mutex);
		enable_irq(p_g_ts->irq);

		if (ret < 0)
		{
			return -1;
		}
	}
	else if (p_i2c_if_ctx->dir == I2C_CMD_HW_RESET)
	{
		acdc_touch_hw_reset(p_g_ts);
	}
	else
	{
		return -1;
	}

	return size;
}

static struct proc_ops fops =
{
	.proc_open	=	acdc_file_open,
	.proc_read	=	acdc_file_read,
	.proc_write	=	acdc_file_write,
};

#endif


#if TOUCH_WAKEUP_ENABLE

static void acdc_touch_suspend(struct acdc_ts_data *ts)
{
	DBG_PRINT("acdc_touch_suspend\r\n");

	if (ts->system_suspend == false)
	{
		ts->system_suspend = true;

	}
}

static void acdc_touch_resume(struct acdc_ts_data *ts)
{
	DBG_PRINT("acdc_touch_resume\r\n");

	if (ts->system_suspend == true)
	{
		ts->system_suspend = false;
	}
}

#if defined(CONFIG_FB)

static int acdc_notifier_callback(struct notifier_block *self, unsigned long event, void *data) 
{
	struct fb_event *ev_data = data;
	int *blank;
	struct acdc_ts_data *ts = container_of(self, struct acdc_ts_data, fb_notif);

	DBG_PRINT("acdc_notifier_callback = %ld\r\n", event);

	if (ev_data && ev_data->data && (event == FB_EVENT_BLANK) && ts) {
		blank = ev_data->data;
		if (*blank == FB_BLANK_POWERDOWN) {
			DBG_PRINT("FB_BLANK_POWERDOWN\r\n");
			acdc_touch_suspend(ts);
		}
		else if (*blank == FB_BLANK_UNBLANK || *blank == FB_BLANK_NORMAL) {
			DBG_PRINT("FB_BLANK_NORMAL\r\n");
			acdc_touch_resume(ts);
		}
	}

	return 0;
}
#endif
#endif

static int acdc_get_fw_version(struct acdc_ts_data *ts)
{
	u32 cmd = ts->fw_version_addr;
	u8 rdata[0x20] = {0,};

	if (ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&rdata[0], 0x10) < 0) {
		DBG_PRINT("fail to get touch fw version.\n");
		return	-1;
	}

	DBG_PRINT_ALWAYS("touch fw version : %c%c%c%c_%d_%d.%d_V%d.%d_%c%c%c \r\n", 
						rdata[0], rdata[1], rdata[2], rdata[3], 
						rdata[4], ((rdata[6] >> 4) & 0x0F) * 100 + (rdata[6] & 0x0F) * 10 + ((rdata[5] >> 4) & 0x0F), rdata[5] & 0x0F,
						rdata[8], rdata[7],
						rdata[9], rdata[10], rdata[11]);

	return 0;
}

static void acdc_recover_work(struct work_struct *work)
{
	u32 i = 0;
	struct acdc_ts_data *ts = container_of((struct delayed_work *)work, struct acdc_ts_data, recover_work);

	DBG_PRINT("acdc_recover_work\r\n");

	if (ts->touch_recover_bit != 0)
	{
		for (i = 0; i < ACDC_TOUCH_FINGER_MAX; i++)
		{
			if ((ts->touch_recover_bit & (1 << i)) != 0)
			{
				input_mt_slot(ts->input, i);
				input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
				// input_report_abs(ts->input, ABS_MT_TRACKING_ID, i);
				input_report_abs(ts->input, ABS_MT_POSITION_X, 0);
				input_report_abs(ts->input, ABS_MT_POSITION_Y, 0);
				input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
				// input_report_abs(ts->input, ABS_MT_PRESSURE, 0);
				// input_report_key(ts->input, BTN_TOUCH, 0);
			}
		}
		input_report_key(ts->input, BTN_TOUCH, 0);
		ts->touch_recover_bit = 0;
		input_sync(ts->input);
	}
}

static void acdc_ts_work(struct acdc_ts_data *ts)
{
	int ret = 0;
	u32 cmd = 0;
	int i = 0;
	u16 touch_package_len = 0;
	hid_multi_touch_t tMultiTouchPkt;
	u32 touch_num = 0;

	// DBG_PRINT("acdc_ts_work\r\n");

	mutex_lock(&ts->mutex);
	cmd = ts->touch_package_len_addr;
	ret = ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&touch_package_len, sizeof(touch_package_len));

	if ((ret > 0) && (touch_package_len > 0) && (touch_package_len <= TOUCH_PACKAGE_LEN_MAX))
	{
		cmd = ts->touch_data_base_addr;
		ret = ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&tMultiTouchPkt, touch_package_len);
		
		if ((ret > 0) && (tMultiTouchPkt.touch_num > 0) && (tMultiTouchPkt.touch_num <= ACDC_TOUCH_POINT_FW_MAX)) {
			for (i = 0; i < tMultiTouchPkt.touch_num; i++) {
				u32 id;
				u32 x;
				u32 y;

				id = tMultiTouchPkt.fingers[i].touch_id;

				if (id > ts->pdata->id_max)
				{
					continue;
				}
				
				x = (((u32)tMultiTouchPkt.fingers[i].x * ts->pdata->abs_max_x) / 32767);
				y = (((u32)tMultiTouchPkt.fingers[i].y * ts->pdata->abs_max_y) / 32767);

#if X_REVERSE_ENABLE
				x = ts->pdata->abs_max_x - x;
#endif

#if Y_REVERSE_ENABLE
				y = ts->pdata->abs_max_y - y;
#endif

#if XY_CHANGE_ENABLE
				x = y;
				y = x;
				y = ts->pdata->abs_max_y - y;
#endif

				// DBG_PRINT("id = %d, x = %d, y = %d, e = %d\r\n", id, x, y, tMultiTouchPkt.fingers[i].touch_event);

				ts->finger[touch_num].event		= tMultiTouchPkt.fingers[i].touch_event;
				ts->finger[touch_num].id		= id;
				ts->finger[touch_num].x			= x;
				ts->finger[touch_num].y			= y;
				ts->finger[touch_num].area		= ts->pdata->area_max;
				ts->finger[touch_num].pressure	= ts->pdata->press_max;
				touch_num++;
			}
		}
		// DBG_PRINT("touch_num = %d\r\n", touch_num);
		if (touch_num != 0) {

#if TOUCH_WAKEUP_ENABLE
			if ((ts->finger[0].event == TS_EVENT_PRESS) && (ts->system_suspend))	{
				wake_lock_timeout(&ts->acdc_wake_lock, 5 * HZ);
				DBG_PRINT("KEY_POWER wakeup\r\n");
				input_report_key(ts->input, KEY_POWER, 1);
				input_sync(ts->input);
				input_report_key(ts->input, KEY_POWER, 0);
				input_sync(ts->input);
				// ts->system_suspend = false;
			}
#endif			

			for (i = 0; i < touch_num; i++) {
				// DBG_PRINT("id = %d, e = %d, x = %d, y = %d\r\n", ts->finger[i].id, ts->finger[i].event, ts->finger[i].x, ts->finger[i].y);
				input_mt_slot(ts->input, ts->finger[i].id);
				switch(ts->finger[i].event) {
					case TS_EVENT_MOVE:
					case TS_EVENT_PRESS:
						ts->touch_recover_bit |= (1 << ts->finger[i].id);
						input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
						// input_report_abs(ts->input, ABS_MT_TRACKING_ID, ts->finger[i].id);
						input_report_abs(ts->input, ABS_MT_POSITION_X, ts->finger[i].x);
						input_report_abs(ts->input, ABS_MT_POSITION_Y, ts->finger[i].y);
						input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, ts->finger[i].area);
						// input_report_abs(ts->input, ABS_MT_PRESSURE, ts->finger[i].pressure);
						// input_report_key(ts->input, BTN_TOUCH, 1);
						// input_mt_sync(ts->input);
						break;

					case TS_EVENT_RELEASE:
						ts->touch_recover_bit &= ~(1 << ts->finger[i].id);
						input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, false);
						// input_report_abs(ts->input, ABS_MT_TRACKING_ID, ts->finger[i].id);
						// input_report_abs(ts->input, ABS_MT_POSITION_X, ts->finger[i].x);
						// input_report_abs(ts->input, ABS_MT_POSITION_Y, ts->finger[i].y);
						// input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR, 0);
						// input_report_abs(ts->input, ABS_MT_PRESSURE, 0);
						// input_report_key(ts->input, BTN_TOUCH, 0);
						// input_mt_sync(ts->input);
						break;
				}
			}
			if (ts->touch_recover_bit != 0)
			{
				input_report_key(ts->input, BTN_TOUCH, 1);
			}
			else
			{
				input_report_key(ts->input, BTN_TOUCH, 0);
			}
			input_sync(ts->input);
			
			if (ts->touch_recover_bit != 0)
			{
				cancel_delayed_work_sync(&ts->recover_work);
				queue_delayed_work(ts->work_queue, &ts->recover_work, TOUCH_RECOVER_TIME * HZ);
			}
			else
			{
				cancel_delayed_work_sync(&ts->recover_work);
			}	
			


		}
	}

	mutex_unlock(&ts->mutex);
}

static int acdc_ts_i2c_read(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len)
{
	struct i2c_msg msg[2];
	int ret = 0;
	u8 i = 0;
	u8 cmd_tmp[10] = {0, };
	u8 retryCount = 0;

	if ((len == 0) || (data == NULL)) {
		dev_err(&client->dev, "I2C read error: Null pointer or length == 0\n");	
		return 	-1;
	}

	memset(cmd_tmp, 0x00, sizeof(cmd_tmp));

	if (cmd_len) {
		for (i = 0; i < cmd_len; i++) {
			cmd_tmp[i] = cmd[cmd_len - 1 - i];
		}
	}
	memset(msg, 0x00, sizeof(msg));
	msg[0].addr		= client->addr;
	msg[0].flags	= client->flags & I2C_M_TEN;
	msg[0].len		= cmd_len;
	msg[0].buf		= cmd_tmp;

	msg[1].addr		= client->addr;
	msg[1].flags	= client->flags & I2C_M_TEN;
	msg[1].flags	|= I2C_M_RD;
	msg[1].len		= len;
	msg[1].buf		= data;

#if I2C_RETRY_COUNT 
	do {
		if ((ret = i2c_transfer(client->adapter, msg, 2)) != 2) {
			dev_err(&client->dev, "Retry(%d) I2C read error: (%d) reg: 0x%X len: %d\n", 
					retryCount, ret, cmd_tmp[0], len);
		}
		else {
			break;
		}
	} while (++retryCount <= I2C_RETRY_COUNT);
#else
	if ((ret = i2c_transfer(client->adapter, msg, 2)) != 2) {
		dev_err(&client->dev, "Retry(%d) I2C read error: (%d) reg: 0x%X len: %d\n", 
				retryCount, ret, cmd_tmp[0], len);
	}
#endif

	if (ret < 0)
	{
		return ret;
	}
	

	return 	len;
}

static int acdc_ts_i2c_write(struct i2c_client *client, u8 *cmd, u32 cmd_len, u8 *data, u32 len)
{
	int ret = 0;
	u8 block_data[256] = {0, };
	u8 i = 0;
	u8 cmd_tmp[10] = {0, };
	u8 retryCount = 0;

	if ((cmd_len + len) >= sizeof(block_data)) {
		dev_err(&client->dev, "I2C write error: wdata overflow reg: 0x%X len: %d\n", 
				cmd[0], cmd_len + len);
		return	-1;
	}

	if ((len == 0) || (data == NULL)) {
		dev_err(&client->dev, "I2C write error: Null pointer or length == 0\n");	
		return 	-1;
	}

	memset(block_data, 0x00, sizeof(block_data));
	memset(cmd_tmp, 0x00, sizeof(cmd_tmp));

	if (cmd_len) {
		for (i = 0; i < cmd_len; i++) {
			cmd_tmp[i] = cmd[cmd_len - 1 - i];
		}
	}

	if (cmd_len) {
		memcpy(&block_data[0], &cmd_tmp[0], cmd_len);
	}

	if (len) {
		memcpy(&block_data[cmd_len], &data[0], len);
	}

#if I2C_RETRY_COUNT 
	do {
		if ((ret = i2c_master_send(client, block_data, (cmd_len + len))) < 0) {
			dev_err(&client->dev, "Retry(%d) I2C write error: (%d) reg: 0x%X len: %d\n", 
					retryCount, ret, cmd[0], len);
		} else { 
			break;
		}
	} while (++retryCount <= I2C_RETRY_COUNT);
#else
	if ((ret = i2c_master_send(client, block_data, (cmd_len + len))) < 0) {
		dev_err(&client->dev, "Retry(%d) I2C write error: (%d) reg: 0x%X len: %d\n", 
				retryCount, ret, cmd[0], len);
	}
#endif

	if (ret < 0)
	{
		return ret;
	}

	return len;
}

static irqreturn_t acdc_touch_irq(int irq, void *handle)
{
	struct acdc_ts_data *ts = handle;

	// DBG_PRINT("acdc_touch_irq\r\n");

	if (ts->pdata->irq_mode == IRQ_MODE_NORMAL) {
		queue_work(ts->work_queue, &ts->interrupt_work);	
	} else if (ts->pdata->irq_mode == IRQ_MODE_THREAD) {
		disable_irq_nosync(ts->irq);
		queue_work(ts->work_queue, &ts->interrupt_work);	
		enable_irq(ts->irq);
	} else if (ts->pdata->irq_mode == IRQ_MODE_POLLING) {
		ts->pdata->touch_work(ts);
	}

	return IRQ_HANDLED;
}

static void acdc_touch_hw_reset(struct acdc_ts_data *ts)
{
	disable_irq(ts->irq);
	if (gpio_is_valid(ts->pdata->reset_gpio)) {
		DBG_PRINT("no power control for touch, then just do reset (high -> low -> high)\n");
		gpio_direction_output(ts->pdata->reset_gpio, 0);

		gpio_set_value(ts->pdata->reset_gpio, 0); 
		mdelay(5);

		gpio_set_value(ts->pdata->reset_gpio, 1); 
	}
	mdelay(500); 
	enable_irq(ts->irq);
}

static int acdc_touch_get_device_type(struct acdc_ts_data *ts)
{
	u32 cmd = 0;
	u32 reg_val = {0,};
	u8 i;
	u8 buf[8];
	const u8 RETRY_COUNT_NUM = 3;

	ts->device_type = DEVICE_TYPE_UNKNOW;
	ts->fw_version_addr = AM84B_FW_VERSION_ADDR;
	ts->touch_package_len_addr = AM84B_TOUCH_PACKAGE_LENGTH;
	ts->touch_data_base_addr = AM84B_TOUCH_DATA_BASE;
	for (i = 0; i < RETRY_COUNT_NUM; i++)
	{
		cmd = AM84B_GEN_REG01_ADDR;
		if (ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&reg_val, sizeof(reg_val)) < 0) {
			DBG_PRINT("fail to get AM84B_GEN_REG01\n");
			mdelay(200); 
			continue;
		}

		if ((((reg_val >> 24) & 0xFF) == 0xB7) || (((reg_val >> 24) & 0xFF) == 0xA1)) {

			if (((reg_val >> 24) & 0xFF) == 0xA1)
			{
				cmd = AM84B_BASE_FW_VERSION_ADDR;
				if (ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), buf, 4) < 0) {
					DBG_PRINT("fail to get AM84B_BASE_VERSION\n");
					mdelay(200); 
					continue;
				}

				if ((buf[0] == '0') && (buf[1] == '8') && (buf[2] == '4'))
				{
					DBG_PRINT("device is AM84B\r\n");
					ts->device_type = DEVICE_TYPE_AM84B;
					ts->fw_version_addr = AM84B_FW_VERSION_ADDR;
					ts->touch_package_len_addr = AM84B_TOUCH_PACKAGE_LENGTH;
					ts->touch_data_base_addr = AM84B_TOUCH_DATA_BASE;
					return DEVICE_TYPE_AM84B;
				}
				else if ((buf[0] == '1') && (buf[1] == '2') && (buf[2] == '7'))
				{
					DBG_PRINT("device is AM127B\r\n");
					ts->device_type = DEVICE_TYPE_AM127B;
					ts->fw_version_addr = AM127B_FW_VERSION_ADDR;
					ts->touch_package_len_addr = AM127B_TOUCH_PACKAGE_LENGTH;
					ts->touch_data_base_addr = AM127B_TOUCH_DATA_BASE;
					return DEVICE_TYPE_AM127B;
				}
			}
			else
			{
				cmd = AM84B_BASE_BL_VERSION_ADDR;
				if (ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), buf, 8) < 0) {
					DBG_PRINT("fail to get AM84B_BASE_BL_VERSION\n");
					mdelay(200); 
					continue;
				}

				if ((buf[2] == '0') && (buf[3] == '8') && (buf[4] == '4'))
				{
					DBG_PRINT("device is AM84B\r\n");
					ts->device_type = DEVICE_TYPE_AM84B;
					ts->fw_version_addr = AM84B_FW_VERSION_ADDR;
					ts->touch_package_len_addr = AM84B_TOUCH_PACKAGE_LENGTH;
					ts->touch_data_base_addr = AM84B_TOUCH_DATA_BASE;
					return DEVICE_TYPE_AM84B;
				}
				else if ((buf[2] == '1') && (buf[3] == '2') && (buf[4] == '7'))
				{
					DBG_PRINT("device is AM127B\r\n");
					ts->device_type = DEVICE_TYPE_AM127B;
					ts->fw_version_addr = AM127B_FW_VERSION_ADDR;
					ts->touch_package_len_addr = AM127B_TOUCH_PACKAGE_LENGTH;
					ts->touch_data_base_addr = AM127B_TOUCH_DATA_BASE;
					return DEVICE_TYPE_AM127B;
				}
			}
        }

		mdelay(200);

		cmd = AM126B_GEN_REG01_ADDR;
		if (ts->pdata->i2c_read(ts->client, (u8 *)&cmd, sizeof(cmd), (u8 *)&reg_val, sizeof(reg_val)) < 0) {
			DBG_PRINT("fail to get AM126B_GEN_REG01\n");
			mdelay(200); 
			continue;
		}

		if ((((reg_val >> 24) & 0xFF) == 0xB7) || (((reg_val >> 24) & 0xFF) == 0xA1)) {
			DBG_PRINT("device is AM126B\r\n");
			ts->device_type = DEVICE_TYPE_AM126B;
			ts->fw_version_addr = AM126B_FW_VERSION_ADDR;
			ts->touch_package_len_addr = AM126B_TOUCH_PACKAGE_LENGTH;
			ts->touch_data_base_addr = AM126B_TOUCH_DATA_BASE;
			return DEVICE_TYPE_AM126B;
        }

		mdelay(200);
	}

	DBG_PRINT("device is unknow\r\n");
	return DEVICE_TYPE_UNKNOW;
}

static void acdc_touch_work_q(struct work_struct *work)
{
	struct acdc_ts_data *ts = container_of(work, struct acdc_ts_data, interrupt_work);

	ts->pdata->touch_work(ts);
}

#ifdef CONFIG_OF
static int acdc_touch_parse_dt(struct device *dev, struct touch_pdata *pdata)
{
	struct device_node *np = dev->of_node;
	u32 temp;

	pdata->reset_gpio = of_get_named_gpio(np, "acdc_reset_gpio", 0);
	DBG_PRINT("pdata->reset_gpio = %d\r\n", pdata->reset_gpio);
	if (pdata->reset_gpio <= 0) {
		pdata->reset_gpio = GPIO_FOR_TS_RESETB; 
	}

	pdata->irq_gpio = of_get_named_gpio(np, "acdc_irq_gpio", 0);
	DBG_PRINT("pdata->irq_gpio = %d\r\n", pdata->irq_gpio);
	if (pdata->irq_gpio <= 0) { 
		pdata->irq_gpio = GPIO_FOR_TS_IRQ; 
	}

	of_property_read_u32(np, "acdc_x_resolution", &temp);
	pdata->abs_max_x = (u16)temp - 1;

	of_property_read_u32(np, "acdc_y_resolution", &temp);
	pdata->abs_max_y = (u16)temp - 1;

	return 0;
}
#endif

static int acdc_init_touch_function(struct touch_pdata *pdata)
{
	if (!pdata) {
		DBG_PRINT("Error : Platform data is NULL pointer!\n");	
		return	-1;
	}

	pdata->name 		= ACDC_TOUCH_NAME;
	pdata->irq_mode 	= IRQ_MODE_THREAD;			// IRQ_MODE_THREAD, IRQ_MODE_NORMAL, IRQ_MODE_POLLING

	pdata->irq_flags 	= IRQF_TRIGGER_FALLING | IRQF_ONESHOT;		// | IRQF_SHARED;

	pdata->max_fingers	= ACDC_TOUCH_FINGER_MAX;
	pdata->area_max		= 10; 
	pdata->press_max	= 255;
	pdata->id_min		= 0;
	pdata->id_max		= ACDC_TOUCH_FINGER_MAX - 1;

	pdata->keycnt		= ACDC_TOUCH_KEY_MAX;
	pdata->keycode		= g_touchKeycodeBuf;

	pdata->touch_work	= acdc_ts_work;
	pdata->i2c_read		= acdc_ts_i2c_read;
	pdata->i2c_write 	= acdc_ts_i2c_write;
	pdata->irq_func 	= acdc_touch_irq;

	return 0;
}

static int acdc_touch_init_gpio(struct touch_pdata *pdata)
{
	int ret = -EINVAL;

	/* 1. Configuration for Reset GPIO */
	if (gpio_is_valid(pdata->reset_gpio)) {
		ret = gpio_request(pdata->reset_gpio, "touch_reset");
		if (ret < 0) {
			pr_err("%s: touch_reset(%d) gpio request fail\n", __func__, pdata->reset_gpio);
			return ret;
		}
		gpio_direction_output(pdata->reset_gpio, 1);
	}

	/* 2. Configuration for Interrupt GPIO */
	if (gpio_is_valid(pdata->irq_gpio)) {
		ret = gpio_request(pdata->irq_gpio, "touch_irq");
		if (ret < 0) {
			pr_err("%s: touch_irq(%d) gpio request fail\n", __func__, pdata->irq_gpio);
			return ret;
		}
		gpio_direction_input(pdata->irq_gpio);
	}

	return ret;
}

//static int acdc_touch_probe(struct i2c_client *client, const struct i2c_device_id *client_id)
static int acdc_touch_probe(struct i2c_client *client)
{
	int ret = 0;
	struct device *dev = &client->dev;
	struct acdc_ts_data *ts;
	struct touch_pdata *pdata;

	DBG_PRINT("acdc_touch_probe\r\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not compatible i2c function\n");
		return -EIO;
	}

#ifdef CONFIG_OF 
	if (client->dev.of_node) {
		DBG_PRINT("Device Tree Based Touch Device Driver.\r\n");
		pdata = devm_kzalloc(&client->dev, sizeof(struct touch_pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory!\n");
			return -ENOMEM;
		}

		ret = acdc_touch_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Failed to parse DT!\n");
			return ret;
		}
	}
	else 
#endif
	{
		DBG_PRINT("Platform Data Based Touch Device Driver.\r\n");
		pdata = client->dev.platform_data;
	}

	if (acdc_init_touch_function(pdata) < 0) {
		dev_err(&client->dev, "Platform data is not available!\n");
		return -EINVAL;
	}

	if (!(ts = kzalloc(sizeof(struct acdc_ts_data), GFP_KERNEL))) {
		dev_err(&client->dev, "touch struct malloc error!\n");			
		return	-ENOMEM;
	}
	ts->client	= client;
	ts->pdata 	= pdata;

	i2c_set_clientdata(client, ts);

	acdc_touch_init_gpio(ts->pdata);

	if (client->irq) {
		ts->irq = client->irq;
		DBG_PRINT("client->irq\r\n");
	}
	else 
	{
		ts->irq = gpio_to_irq(ts->pdata->irq_gpio);
		DBG_PRINT("gpio_to_irq\r\n");
	}

	DBG_PRINT("ts->irq = %d\r\n", ts->irq);

	acdc_touch_hw_reset(ts);
	if (acdc_touch_get_device_type(ts) == DEVICE_TYPE_UNKNOW)
	{
		ret = -6;
		goto err_device_unknow;
	}
	acdc_get_fw_version(ts);

	if (ts->pdata->max_fingers) {
		if (!(ts->finger = kzalloc(sizeof(finger_t) * ts->pdata->max_fingers, GFP_KERNEL))) {
			kfree(ts);
			DBG_PRINT("touch data struct malloc error!\n");	
			return	-ENOMEM;
		}
	}

	dev_set_drvdata(dev, ts);

	if (!(ts->input = input_allocate_device())) { 
		DBG_PRINT("Fail to input_allocate_device\r\n");
		ret = -6;
		goto err_free_mem;
	}

	snprintf(ts->phys, sizeof(ts->phys), "%s/input0", dev_name(&client->dev));

	ts->input->name			= ts->pdata->name;
	ts->input->phys			= ts->phys;
	ts->input->id.bustype	= BUS_I2C;
	DBG_PRINT("ts->input->name = %s\r\n", ts->input->name);

	set_bit(EV_SYN, ts->input->evbit);	
	set_bit(EV_ABS, ts->input->evbit);
	set_bit(BTN_TOUCH, ts->input->keybit); /* for multi-touch, indicates whether the tool is touching the device */
	// set_bit(ABS_PRESSURE, ts->input->absbit);
	/* device type will be set to touch screen */
	set_bit(INPUT_PROP_DIRECT, ts->input->propbit);

	/* Register Touch Key Event */
	if (ts->pdata->keycode) {
		int	key;

		set_bit(EV_KEY,	ts->input->evbit);
		DBG_PRINT("ts->pdata->keycnt = %d\r\n", ts->pdata->keycnt);
		for (key = 0; key < ts->pdata->keycnt; key++) {
			if (ts->pdata->keycode[key] <= 0) continue;
			set_bit(ts->pdata->keycode[key] & KEY_MAX, ts->input->keybit);
		}
	}

	input_set_abs_params(ts->input, ABS_MT_POSITION_X, ts->pdata->abs_min_x, ts->pdata->abs_max_x, 0, 0);
	input_set_abs_params(ts->input, ABS_MT_POSITION_Y, ts->pdata->abs_min_y, ts->pdata->abs_max_y, 0, 0);

	if (ts->pdata->area_max) {
		input_set_abs_params(ts->input, ABS_MT_TOUCH_MAJOR, ts->pdata->area_min, ts->pdata->area_max, 0, 0);
	}

	// if (ts->pdata->press_max) {
	// 	input_set_abs_params(ts->input, ABS_MT_PRESSURE, ts->pdata->press_min, ts->pdata->press_max, 0, 0);
	// }

	if (ts->pdata->id_max) {
		// input_set_abs_params(ts->input, ABS_MT_TRACKING_ID, ts->pdata->id_min, ts->pdata->id_max, 0, 0);

	input_mt_init_slots(ts->input, ts->pdata->max_fingers, INPUT_MT_DIRECT);

	}

	input_set_drvdata(ts->input, ts);
	if ((ret = input_register_device(ts->input))) {
		dev_err(dev, "input(%s) register fail!\n", ts->input->name);
		goto err_free_input_mem;
	}

	mutex_init(&ts->mutex);
	ts->touch_recover_bit = 0;

	if (ts->irq) {
		/* don't enable irq(ts->irq) automatically when requesting irq */
		irq_set_status_flags(ts->irq, IRQ_NOAUTOEN);
		INIT_DELAYED_WORK(&ts->recover_work, acdc_recover_work);

		switch(ts->pdata->irq_mode)	{
			case IRQ_MODE_THREAD:
				INIT_WORK(&ts->interrupt_work, acdc_touch_work_q);
				if ((ts->work_queue = create_singlethread_workqueue("ts_work_queue")) == NULL) {	
					goto err_free_input_mem;
				}

				DBG_PRINT("ts->pdata->irq_flags = %d\r\n", ts->pdata->irq_flags);
				if ((ret = request_threaded_irq(ts->irq, NULL, ts->pdata->irq_func, ts->pdata->irq_flags, ts->pdata->name, ts))) {
					dev_err(dev, "irq %d request fail!\n", ts->irq);
					DBG_PRINT("request_threaded_irq = %d\r\n", ret);
					goto err_free_input_mem;
				}
				DBG_PRINT("IRQ_MODE_THREAD: irq %d request ok!\n", ts->irq);
				break;

			case IRQ_MODE_NORMAL:
				INIT_WORK(&ts->interrupt_work, acdc_touch_work_q);
				if ((ts->work_queue = create_singlethread_workqueue("ts_work_queue")) == NULL) {	
					goto err_free_input_mem;
				}

				if ((ret = request_irq(ts->irq, ts->pdata->irq_func, ts->pdata->irq_flags, ts->pdata->name, ts))) {
					dev_err(dev, "irq %d request fail!\n", ts->irq);
					goto err_free_input_mem;
				}
				DBG_PRINT("IRQ_MODE_NORMAL: irq %d request ok!\n", ts->irq);
				break;

			case IRQ_MODE_POLLING:
				if ((ret = request_threaded_irq(ts->irq, NULL, ts->pdata->irq_func, ts->pdata->irq_flags, ts->pdata->name, ts))) {
					dev_err(dev, "irq %d request fail!\n", ts->irq);
					DBG_PRINT("request_threaded_irq = %d\r\n", ret);
					goto err_free_input_mem;
				}
				DBG_PRINT("IRQ_MODE_POLLING: irq %d request ok!\n", ts->irq);
				break;

			default	:
				DBG_PRINT("IRQ_MODE(%d) is not supported\n", ts->pdata->irq_mode);
				break;
		}
	}

#if TOUCH_WAKEUP_ENABLE
#if defined(CONFIG_FB)

	DBG_PRINT("CONFIG_FB\r\n");
	
	ts->fb_notif.notifier_call = acdc_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		DBG_PRINT("Fail to register fb_notifier: %d\n", ret);

#endif

	input_set_capability(ts->input, EV_KEY, KEY_POWER);
	device_init_wakeup(&ts->client->dev, 1);
	wake_lock_init(&ts->acdc_wake_lock, WAKE_LOCK_SUSPEND, "acdc_wakelock");

#endif

#if TOUCH_TOOL_COMMUNICATION_ENABLE

	if (!proc_create("acdc_touch", ACDC_DEVICE_NODE_PERMISSON, NULL, &fops)) {
		DBG_PRINT("%s\r\n", "Failed to creat acdc_touch");
		goto err_free_input_mem;
	}

	p_g_ts = ts;

#endif

	enable_irq(ts->irq);

	DBG_PRINT("acdc_touch_probe end = %d\r\n", ret);

	return 0;

err_free_input_mem:
	input_free_device(ts->input);
	ts->input = NULL;

err_free_mem:
	kfree(ts->finger); 
	ts->finger = NULL;
err_device_unknow:
	kfree(ts); 
	ts = NULL;
	return ret;
}


static void acdc_touch_remove(struct i2c_client *client)
{
    struct device *dev = &client->dev;

	struct acdc_ts_data *ts = dev_get_drvdata(dev);

	DBG_PRINT("acdc_touch_remove\r\n");

#if TOUCH_WAKEUP_ENABLE
#if defined(CONFIG_FB)

	device_init_wakeup(&ts->client->dev, 0);
	wake_lock_destroy(&ts->acdc_wake_lock);

#endif
#endif

	if (ts->irq) {							   
		free_irq(ts->irq, ts);
	}

	if (gpio_is_valid(ts->pdata->reset_gpio)) {
		gpio_free(ts->pdata->reset_gpio);
	}

	if (gpio_is_valid(ts->pdata->irq_gpio)) {
		gpio_free(ts->pdata->irq_gpio);
	}

	input_unregister_device(ts->input);

	dev_set_drvdata(dev, NULL);

	kfree(ts->finger); 
	ts->finger = NULL;
	kfree(ts); 
	ts = NULL;
}

#ifdef CONFIG_OF
static struct of_device_id acdc_match_table[] = {
	{ .compatible = "acdc,acdc_touch",},
	{ },
};
#else
#define acdc_match_table NULL
#endif

static const struct i2c_device_id acdc_device_id[] = {
	{ ACDC_TOUCH_NAME, 0 },
	{ }
};

static struct i2c_driver acdc_touch_driver = {
	.driver = {
		.owner			= THIS_MODULE,
		.name			= ACDC_TOUCH_NAME,
		.of_match_table	= acdc_match_table,
	},
	.probe		= acdc_touch_probe,
	.remove		= acdc_touch_remove,
	.id_table	= acdc_device_id,
};

static int __init acdc_touch_init(void)
{
	DBG_PRINT_ALWAYS("===============================\r\n");
	DBG_PRINT_ALWAYS(ACDC_TOUCH_DRIVER_VERSION);
	DBG_PRINT_ALWAYS("===============================\r\n");
	DBG_PRINT("acdc_touch_init\r\n");

	return i2c_add_driver(&acdc_touch_driver);
}

static void __exit acdc_touch_exit(void)
{
	DBG_PRINT("acdc_touch_exit\r\n");

	i2c_del_driver(&acdc_touch_driver);
}

module_init(acdc_touch_init);
module_exit(acdc_touch_exit);

MODULE_AUTHOR("FAE@advancustom.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(ACDC_TOUCH_DRIVER_VERSION);
