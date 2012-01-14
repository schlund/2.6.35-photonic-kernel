/* drivers/misc/bma150_spi.c - bma150 G-sensor driver
 *
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/bma150.h>
#include <asm/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <mach/atmega_microp.h>

#define D(x...) printk(KERN_DEBUG "[GSNR][BMA150 SPI] " x)
#define E(x...) printk(KERN_ERR "[GSNR][BMA150 SPI ERROR] " x)
#define DIF(x...) if (debug_flag) \
			printk(KERN_DEBUG "[GSNR][BMA150 SPI DEBUG] " x)

#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
struct device_attribute dev_attr_##_name = __ATTR(_name, _mode, _show, _store)
#define MODULE_NAME "bma150"
#define EVENT_TYPE_TEMPERATURE      ABS_THROTTLE
#define BMA_POLL_TIMER_MS 200

struct early_suspend bma_early_suspend;
static struct bma150_platform_data *this_pdata;
static struct mutex gsensor_RW_mutex;
static struct mutex gsensor_set_mode_mutex;
static atomic_t PhoneOn_flag = ATOMIC_INIT(0);

//r0bin: added for asynchronous polling (input dev)
static enum hrtimer_restart bma150_poll_timer(struct hrtimer *timer)
{
	struct bma150_platform_data *bma150_w;
	bma150_w = container_of(timer, struct bma150_platform_data, timer);
#ifdef CONFIG_ANDROID_POWER
	android_lock_suspend(&bma150_w->suspend_lock);
#endif
	schedule_work(&bma150_w->work.work);
	return HRTIMER_NORESTART;
}

static int debug_flag;

static int spi_microp_enable(uint8_t on)
{
	int ret;
	ret = microp_spi_vote_enable(SPI_GSENSOR, on);
	if (ret < 0)
		E("%s: i2c_write_block fail\n", __func__);

	return ret;
}

static int spi_gsensor_read(uint8_t *data)
{
	int ret;

	mutex_lock(&gsensor_RW_mutex);

	ret = microp_i2c_write(MICROP_I2C_WCMD_GSENSOR_REG_DATA_REQ, data, 1);
	if (ret < 0) {
		E("%s: i2c_write_block fail\n", __func__);
		mutex_unlock(&gsensor_RW_mutex);
		return ret;
	}

	ret = microp_i2c_read(MICROP_I2C_RCMD_GSENSOR_REG_DATA, data, 2);
	if (ret < 0) {
		E("%s: i2c_read_block fail\n", __func__);
		mutex_unlock(&gsensor_RW_mutex);
		return ret;
	}

	mutex_unlock(&gsensor_RW_mutex);

	return ret;
}

static int spi_gsensor_write(uint8_t *data)
{
	int ret;

	mutex_lock(&gsensor_RW_mutex);

	ret = microp_i2c_write(MICROP_I2C_WCMD_GSENSOR_REG, data, 2);
	if (ret < 0) {
		E("%s: i2c_write_block fail\n", __func__);
		mutex_unlock(&gsensor_RW_mutex);
		return ret;
	}

	mutex_unlock(&gsensor_RW_mutex);

	return ret;
}

static int spi_gsensor_init_hw(void)
{
	char buffer[2];

	memset(buffer, 0x0, sizeof(buffer));
	buffer[0] = RANGE_BWIDTH_REG;
	if (spi_gsensor_read(buffer) < 0)
		return -EIO;

	/*printk("spi_gsensor_init_hw,read RANGE_BWIDTH_REG = %x "
	, buffer[1]);*/

	buffer[1] = (buffer[1]&0xe0);
	buffer[0] = RANGE_BWIDTH_REG;
	if (spi_gsensor_write(buffer) < 0)
		return -EIO;

	buffer[0] = SMB150_CONF2_REG;
	if (spi_gsensor_read(buffer) < 0)
		return -EIO;

	buffer[1] = buffer[1]|1<<3;
	buffer[0] = SMB150_CONF2_REG;
	if (spi_gsensor_write(buffer) < 0)
		return -EIO;

	return 0;
}

/*
static int spi_gsensor_read_version(void)
{
	uint8_t buffer[2];
	int ret = -EIO;

	buffer[0] = VERSION_REG;
	buffer[1] = 1;
	ret = spi_gsensor_read(buffer);
	if (ret < 0) {
		printk(KERN_ERR "%s: get al_version fail(%d)\n", __func__, ret);
		return ret;
	}
	printk(KERN_INFO "%s: al_version: 0x%2.2X\n", __func__, buffer[0]);

	buffer[0] = CHIP_ID_REG;
	buffer[1] = 1;
	ret = spi_gsensor_read(buffer);
	if (ret < 0) {
		printk(KERN_ERR "%s: get chip_id fail(%d)\n", __func__, ret);
		return ret;
	}
	printk(KERN_INFO "%s: chip_id: 0x%2.2X\n", __func__, buffer[0]);
	return 0;
}
*/
static int spi_bma150_TransRBuff(short *rbuf)
{
	int ret;
	unsigned char buffer[6];
	memset(buffer, 0, 6);

	mutex_lock(&gsensor_RW_mutex);

	buffer[0] = 1;
	ret = microp_i2c_write(MICROP_I2C_WCMD_GSENSOR_DATA_REQ, buffer, 1);
	if (ret < 0) {
		E("%s: i2c_write_block fail\n", __func__);
		mutex_unlock(&gsensor_RW_mutex);
		return ret;
	}

	if (this_pdata && this_pdata->microp_new_cmd &&
			this_pdata->microp_new_cmd == 1) {
		/*printk(KERN_DEBUG "%s: New MicroP command\n", __func__);*/
		ret = microp_i2c_read(MICROP_I2C_RCMD_GSENSOR_DATA, buffer, 6);
		rbuf[0] = buffer[0]<<2|buffer[1]>>6;
		if (rbuf[0]&0x200)
			rbuf[0] -= 1<<10;
		rbuf[1] = buffer[2]<<2|buffer[3]>>6;
		if (rbuf[1]&0x200)
			rbuf[1] -= 1<<10;
		rbuf[2] = buffer[4]<<2|buffer[5]>>6;
		if (rbuf[2]&0x200)
			rbuf[2] -= 1<<10;
	} else {
		/* For Passion with V01 ~ V05 Microp */
		/*printk(KERN_DEBUG "%s: Old MicroP command\n", __func__);*/
		ret = microp_i2c_read(MICROP_I2C_RCMD_GSENSOR_X_DATA,
					buffer, 2);
		if (ret < 0) {
			E("%s: i2c_read_block fail\n", __func__);
			mutex_unlock(&gsensor_RW_mutex);
			return ret;
		}
		rbuf[0] = buffer[0]<<2|buffer[1]>>6;
		if (rbuf[0]&0x200)
			rbuf[0] -= 1<<10;

		ret = microp_i2c_read(MICROP_I2C_RCMD_GSENSOR_Y_DATA,
					buffer, 2);
		if (ret < 0) {
			E("%s: i2c_read_block fail\n", __func__);
			mutex_unlock(&gsensor_RW_mutex);
			return ret;
			}
		rbuf[1] = buffer[0]<<2|buffer[1]>>6;
		if (rbuf[1]&0x200)
			rbuf[1] -= 1<<10;

		ret = microp_i2c_read(MICROP_I2C_RCMD_GSENSOR_Z_DATA,
					buffer, 2);
		if (ret < 0) {
			E("%s: i2c_read_block fail\n", __func__);
			mutex_unlock(&gsensor_RW_mutex);
			return ret;
			}
		rbuf[2] = buffer[0]<<2|buffer[1]>>6;
		if (rbuf[2]&0x200)
			rbuf[2] -= 1<<10;
	}
	//printk("X=%d, Y=%d, Z=%d\n",rbuf[0],rbuf[1],rbuf[2]);

/*	printk(KERN_DEBUG "%s: 0x%2.2X 0x%2.2X 0x%2.2X \
0x%2.2X 0x%2.2X 0x%2.2X\n",
		__func__, buffer[0], buffer[1], buffer[2], \
		buffer[3], buffer[4], buffer[5]);*/

	DIF("%s: (x, y, z) = (%d, %d, %d)\n",
		__func__, rbuf[0], rbuf[1], rbuf[2]);
	mutex_unlock(&gsensor_RW_mutex);

	return 1;
}

static int spi_bma150_read_temp()
{
	char buffer[2];
	int ret;

	buffer[0] = TEMP_RD_REG;
	ret = spi_gsensor_read(buffer);
	if (ret < 0 )
		return -EIO;
	//temp correction
	return buffer[1];
}


static int __spi_bma150_set_mode(char mode)
{
	char buffer[2] = "";
	int ret;
	mutex_lock(&gsensor_set_mode_mutex);
	if (mode == BMA_MODE_NORMAL) {
		spi_microp_enable(1);
		printk(KERN_INFO "[GSNR] Gsensor enable\n");
		
		if (this_pdata)	{
			//update suspend state
			this_pdata->susp = 0;
			//printk("%s: this_pdata=%x, susp=%d\n",__func__,this_pdata,this_pdata->susp);
			//restart the timer
			if(!hrtimer_active(&this_pdata->timer))
				hrtimer_start(&this_pdata->timer,ktime_set(0, BMA_POLL_TIMER_MS*1000000),HRTIMER_MODE_REL);
		}else{
			//printk("%s: error, this_pdata=NULL\n",__func__);
		}
	}

	buffer[0] = SMB150_CTRL_REG;
	ret = spi_gsensor_read(buffer);
	if (ret < 0) {
		mutex_unlock(&gsensor_set_mode_mutex);
		return -1;
	}

	buffer[1] = (buffer[1]&0xfe)|mode;
	buffer[0] = SMB150_CTRL_REG;
	ret = spi_gsensor_write(buffer);

	if (mode == BMA_MODE_SLEEP) {
		spi_microp_enable(0);
		printk(KERN_INFO "[GSNR] Gsensor disable\n");
		
		if (this_pdata)	{
			//update suspend state
			this_pdata->susp = 1;
			//printk("%s: this_pdata=%x, susp=%d\n",__func__,this_pdata,this_pdata->susp);
			//stop the timer
			if(hrtimer_active(&this_pdata->timer))
				hrtimer_cancel(&this_pdata->timer);
		}else{
			//printk("%s: error, this_pdata=NULL\n",__func__);
		}
	}
	mutex_unlock(&gsensor_set_mode_mutex);
	return ret;
}


static int spi_bma150_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}

static int spi_bma150_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int spi_bma150_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	char rwbuf[8] = "";
	char *toRbuf;
	int ret = -1;
	short buf[8], temp;
	int kbuf = 0;


	switch (cmd) {
	case BMA_IOCTL_READ:
	case BMA_IOCTL_WRITE:
	case BMA_IOCTL_SET_MODE:
	case BMA_IOCTL_SET_CALI_MODE:
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		if (copy_from_user(&buf, argp, sizeof(buf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_WRITE_CALI_VALUE:
		if (copy_from_user(&kbuf, argp, sizeof(kbuf)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case BMA_IOCTL_INIT:
		ret = spi_gsensor_init_hw();
		if (ret < 0)
			return ret;
		break;

	case BMA_IOCTL_READ:
		if (rwbuf[0] < 1)
			return -EINVAL;
		ret = spi_gsensor_read(&rwbuf[1]);
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_WRITE:
		if (rwbuf[0] < 2)
			return -EINVAL;
		ret = spi_gsensor_write(&rwbuf[1]);
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_WRITE_CALI_VALUE:
		this_pdata->gs_kvalue = kbuf;
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		ret = spi_bma150_TransRBuff(&buf[0]);
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_READ_CALI_VALUE:
		if ((this_pdata->gs_kvalue & (0x67 << 24)) != (0x67 << 24)) {
			rwbuf[0] = 0;
			rwbuf[1] = 0;
			rwbuf[2] = 0;
		} else {
			rwbuf[0] = (this_pdata->gs_kvalue >> 16) & 0xFF;
			rwbuf[1] = (this_pdata->gs_kvalue >>  8) & 0xFF;
			rwbuf[2] =  this_pdata->gs_kvalue        & 0xFF;
		}
		break;
	case BMA_IOCTL_SET_MODE:
		/*printk(KERN_DEBUG
		"%s: BMA_IOCTL_SET_MODE by ioctl = %d\n",
			__func__,rwbuf[0]);*/
		ret = __spi_bma150_set_mode(rwbuf[0]);
		if (ret < 0)
			return ret;
		break;
	case BMA_IOCTL_GET_INT:
		temp = 0;
		break;
	case BMA_IOCTL_GET_CHIP_LAYOUT:
		if (this_pdata)
			temp = this_pdata->chip_layout;
		break;
	case BMA_IOCTL_GET_CALI_MODE:
		if (this_pdata)
			temp = this_pdata->calibration_mode;
		break;
	case BMA_IOCTL_SET_CALI_MODE:
		if (this_pdata)
			this_pdata->calibration_mode = rwbuf[0];
		break;
	case BMA_IOCTL_ARE_YOU_SLEEPING:
		if(this_pdata) {
			buf[0] = this_pdata->susp;
		}else{
			buf[0] = 0;
		}
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case BMA_IOCTL_READ:
		toRbuf = &rwbuf[1];
		if (copy_to_user(argp, toRbuf, sizeof(rwbuf)-1))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_ACCELERATION:
		if (copy_to_user(argp, &buf, sizeof(buf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_READ_CALI_VALUE:
		if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
			return -EFAULT;
		break;
	case BMA_IOCTL_ARE_YOU_SLEEPING:
		if (copy_to_user(argp, &buf, 1))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_INT:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_CHIP_LAYOUT:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	case BMA_IOCTL_GET_CALI_MODE:
		if (copy_to_user(argp, &temp, sizeof(temp)))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

static struct file_operations spi_bma_fops = {
	.owner = THIS_MODULE,
	.open = spi_bma150_open,
	.release = spi_bma150_release,
	.ioctl = spi_bma150_ioctl,
};

static struct miscdevice spi_bma_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma150",
	.fops = &spi_bma_fops,
};

static void bma150_early_suspend(struct early_suspend *handler)
{
	int ret = 0;
	if (!atomic_read(&PhoneOn_flag)) {
		ret = __spi_bma150_set_mode(BMA_MODE_SLEEP);
	} else
		printk(KERN_DEBUG "bma150_early_suspend: PhoneOn_flag is set\n");
	/*printk(KERN_DEBUG
		"%s: spi_bma150_set_mode returned = %d!\n",
			__func__, ret);*/
}

static void bma150_early_resume(struct early_suspend *handler)
{
	/*printk(KERN_DEBUG
		"%s: spi_bma150_set_mode returned = %d!\n",
			__func__, ret);*/
}
static ssize_t spi_bma150_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char *s = buf;
	s += sprintf(s, "%d\n", atomic_read(&PhoneOn_flag));
	return (s - buf);
}

static ssize_t spi_bma150_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		atomic_set(&PhoneOn_flag, 1);
		printk(KERN_DEBUG "spi_bma150_store: PhoneOn_flag=%d\n", atomic_read(&PhoneOn_flag));
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		atomic_set(&PhoneOn_flag, 0);
		printk(KERN_DEBUG "spi_bma150_store: PhoneOn_flag=%d\n", atomic_read(&PhoneOn_flag));
		return count;
	}
	E("spi_bma150_store: invalid argument\n");
	return -EINVAL;

}

static DEVICE_ACCESSORY_ATTR(PhoneOnOffFlag, 0664, \
	spi_bma150_show, spi_bma150_store);

int spi_bma150_registerAttr(void)
{
	int ret;
	struct class *htc_accelerometer_class;
	struct device *accelerometer_dev;

	htc_accelerometer_class = class_create(THIS_MODULE, "htc_accelerometer");
	if (IS_ERR(htc_accelerometer_class)) {
		ret = PTR_ERR(htc_accelerometer_class);
		htc_accelerometer_class = NULL;
		goto err_create_class;
	}

	accelerometer_dev = device_create(htc_accelerometer_class,
				NULL, 0, "%s", "accelerometer");
	if (unlikely(IS_ERR(accelerometer_dev))) {
		ret = PTR_ERR(accelerometer_dev);
		accelerometer_dev = NULL;
		goto err_create_accelerometer_device;
	}

	/* register the attributes */
	ret = device_create_file(accelerometer_dev, &dev_attr_PhoneOnOffFlag);
	if (ret)
		goto err_create_accelerometer_device_file;

	return 0;

err_create_accelerometer_device_file:
	device_unregister(accelerometer_dev);
err_create_accelerometer_device:
	class_destroy(htc_accelerometer_class);
err_create_class:

	return ret;
}


//this function is called every x ms, gather accel data and send it as an event
static void bma150_work(struct work_struct *work)
{
	short vals[3];
	int x,y,z, temp;
	struct bma150_platform_data *bma150_w;
	
	bma150_w = container_of(work, struct bma150_platform_data, work.work);
	
	mutex_lock(&bma150_w->mWorkLock);
	
	//get acceleration values
	vals[0] = vals[1] = vals[2] = 0;
	spi_bma150_TransRBuff(&vals[0]);
	x = vals[0];
	y = vals[2];//invert y and z!
	z = vals[1];
	//retrieve temperature
	temp = spi_bma150_read_temp();
	//printk("MAAT bma150_work called, x=%d y=%d z=%d temp=%d\n",x,y,z,temp);	
	//report update
	input_report_abs(bma150_w->idev, ABS_X, x);
	input_report_abs(bma150_w->idev, ABS_Y, y);
	input_report_abs(bma150_w->idev, ABS_Z, z);
	input_report_abs(bma150_w->idev, EVENT_TYPE_TEMPERATURE, temp);
	input_sync(bma150_w->idev);
	
	//restart timer if not suspended (500ms)
	if (!bma150_w->susp)
		hrtimer_start(&bma150_w->timer,ktime_set(0, BMA_POLL_TIMER_MS*1000000),HRTIMER_MODE_REL);

#ifdef CONFIG_ANDROID_POWER
	android_unlock_suspend(&bma150_w->suspend_lock);
#endif
	mutex_unlock(&bma150_w->mWorkLock);
}

static int spi_gsensor_initial(void)
{
	int ret;

	ret = misc_register(&spi_bma_device);
	if (ret < 0) {
		E("%s: init misc_register fail\n", __func__);
		return ret;
	}

	mutex_init(&gsensor_RW_mutex);
	mutex_init(&gsensor_set_mode_mutex);

	ret = spi_microp_enable(1);
	if (ret) {
		E("%s: spi_microp_enable(1) fail!\n", __func__);
		goto err_spi_enable;
	}

	//r0bin: fix g-sensor calibration at init
	ret = spi_gsensor_init_hw();
	if (ret) {
		printk(KERN_ERR "%s: spi_gsensor_init_hw fail, continue anyway\n", __func__);
	}
	
	ret = __spi_bma150_set_mode(BMA_MODE_SLEEP);
	if (ret) {
		E("%s: set BMA_MODE_SLEEP fail!\n", __func__);
		goto err_set_mode;
	}

	bma_early_suspend.suspend = bma150_early_suspend;
	bma_early_suspend.resume = bma150_early_resume;
	register_early_suspend(&bma_early_suspend);

	ret = spi_bma150_registerAttr();
	if (ret) {
		E("%s: set spi_bma150_registerAttr fail!\n", __func__);
		goto err_registerAttr;
	}

	debug_flag = 0;

	return 0;

err_registerAttr:
err_set_mode:
	spi_microp_enable(0);
err_spi_enable:
	misc_deregister(&spi_bma_device);

	return ret;
}

static int  spi_bma150_probe(struct platform_device *pdev)
{
	printk(KERN_INFO "%s: G-sensor connect with microP: "
			"start initial, kvalue = 0x%x\n", __func__, gs_kvalue);

	this_pdata = pdev->dev.platform_data;
	this_pdata->gs_kvalue = gs_kvalue;

	//r0bin: added for async poll
	this_pdata->idev = input_allocate_device();
	if (this_pdata->idev) {
		this_pdata->idev->name = MODULE_NAME;
		//idev->phys=kzalloc(12, GFP_KERNEL);
		//snprintf((char*)idev->phys, 11, "i2c/0-%04x", _bma->micropklt_t->client->addr);
		set_bit(EV_ABS, this_pdata->idev->evbit);
		input_set_abs_params(this_pdata->idev, ABS_X, -2048, 2047, 0, 0);
		input_set_abs_params(this_pdata->idev, ABS_Y, -2048, 2047, 0, 0);
		input_set_abs_params(this_pdata->idev, ABS_Z, -2048, 2047, 0, 0);
		input_set_abs_params(this_pdata->idev, EVENT_TYPE_TEMPERATURE, 0, 256, 0, 0);
		//input_set_abs_params(this_pdata->idev, ABS_GAS, 0, 65535, 0, 0);
		if (!input_register_device(this_pdata->idev)) {
			printk( MODULE_NAME 
					": Input device successfuly registered\n");
		} else {
			this_pdata->idev = 0;
			printk(KERN_ERR MODULE_NAME 
					": Failed to register input device\n");
		}
	}
	
/*
	printk(KERN_DEBUG "%s: this_pdata->microp_new_cmd = %d\n",
			__func__, this_pdata->microp_new_cmd);
*/
	spi_gsensor_initial();

	//dont forget to init mutex
	mutex_init(&this_pdata->mWorkLock);
	
#ifdef CONFIG_ANDROID_POWER
	this_pdata->suspend_lock.name = MODULE_NAME;
	android_init_suspend_lock(&this_pdata->suspend_lock);
	android_lock_suspend(&this_pdata->suspend_lock);
#endif
	//r0bin: added for async poll
	INIT_DELAYED_WORK(&this_pdata->work, bma150_work);
	hrtimer_init(&this_pdata->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	this_pdata->timer.function = bma150_poll_timer;
	this_pdata->susp = 1;
	return 0;
}

static int spi_bma150_remove(struct platform_device *pdev)
{
	mutex_destroy(&gsensor_set_mode_mutex);
#ifdef CONFIG_ANDROID_POWER
	android_uninit_suspend_lock(&(pdev->dev.platform_data->suspend_lock)->suspend_lock);
#endif
	return 0;
}

static struct platform_driver spi_bma150_driver = {
	.probe		= spi_bma150_probe,
	.remove		= spi_bma150_remove,
	.driver		= {
		.name		= MODULE_NAME,
		.owner		= THIS_MODULE,
	},
};

static int __init spi_bma150_init(void)
{
	return platform_driver_register(&spi_bma150_driver);

}

static void __exit spi_bma150_exit(void)
{
	platform_driver_unregister(&spi_bma150_driver);
}


module_init(spi_bma150_init);
module_exit(spi_bma150_exit);

MODULE_DESCRIPTION("BMA150 G-sensor driver");
MODULE_LICENSE("GPL");
