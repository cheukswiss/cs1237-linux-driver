#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#ifdef CONFIG_IIO
#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>
#endif
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/version.h>

#include "cs1237.h"

#define CS1237_NUM_CHANNELS	1

enum {
	CS1237_STATUS_IDLE,
	CS1237_STATUS_PREPARE,
	CS1237_STATUS_READY,
}; 

struct cs1237_data {
	struct platform_device	*pdev;
#ifdef CONFIG_IIO
	struct iio_dev		*indio_dev;
	struct iio_chan_spec	channels[CS1237_NUM_CHANNELS];
#endif
	struct tasklet_struct	tasklet;
	spinlock_t 		slock;
	wait_queue_head_t	wait;
	int			irq;
	int			status;
	unsigned long		flags;
	int32_t			adc_value;

	/* chip config */
	cs1237_config_t config;
	unsigned 	dclk;	/* gpio: sclk */
	unsigned 	dio;	/* gpio: DRDY/DOUT */
	int 		ref;	/* ref voltage μV */
	unsigned long	stamp;	/* read time stamp */
	unsigned long	freq;	/* data ready time */
	unsigned long	dlock;	/* data update time */
};

static int cs1237_open(struct inode *inode, struct file *file);
static int cs1237_close(struct inode *inodep, struct file *filp);
static ssize_t cs1237_read(struct file *file, char __user *buf,
			   size_t len, loff_t *ppos);
static ssize_t cs1237_write(struct file *file, const char __user *buf,
			    size_t len, loff_t *ppos);
static unsigned int cs1237_poll(struct file *file, poll_table *wait);
static long cs1237_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static const struct file_operations cs1237_fops = {
	.owner = THIS_MODULE,
	.read = cs1237_read,
	.write = cs1237_write,
	.open = cs1237_open,
	.release = cs1237_close,
	.poll = cs1237_poll,
	.llseek = no_llseek,
	.unlocked_ioctl = cs1237_ioctl,
};

struct miscdevice cs1237_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cs1237",
	.fops = &cs1237_fops,
};

#define cs1237_get_pdev() dev_get_drvdata(cs1237_device.this_device)
#define cs1237_get_data() platform_get_drvdata(dev_get_drvdata(cs1237_device.this_device))

#ifdef CONFIG_MS_GPIO
extern void MDrv_GPIO_Set_High(uint8_t u8IndexGPIO);
extern void MDrv_GPIO_Set_Low(uint8_t u8IndexGPIO);
extern uint8_t MDrv_GPIO_Pad_Read(uint8_t u8IndexGPIO);

#define spi_get_value(gpio) MDrv_GPIO_Pad_Read(gpio)
#define spi_set_value(gpio, value) (value ? MDrv_GPIO_Set_High(gpio) : MDrv_GPIO_Set_Low(gpio))
#else
#define spi_get_value(gpio) gpio_get_value(gpio)
#define spi_set_value(gpio, value) gpio_set_value(gpio, value)
#endif

#if 0
static void cs1237_hw_reset(void)
{
}

static void cs1237_set_pwrsave(int pwrsave)
{
	if (pwrsave && !cs1237_data->pwrsave) {
		gpio_set_value_cansleep(cs1237_data->dclk, 1);
		udelay(150);
	} else {
		gpio_set_value_cansleep(cs1237_data->dclk, 0);
		udelay(20);
	}
}
#endif

static inline int cs1237_get_gain(int pga)
{
	switch (pga) {
	case PGA_SEL_1:
		return 1;
	case PGA_SEL_2:
		return 2;
	case PGA_SEL_64:
		return 64;
	case PGA_SEL_128:
		return 128;
	}
	return 0;
}

static inline unsigned int cs1237_build_time(int speed)
{
	/* hz to usec */
	switch (speed) {
	case SPEED_SEL_10HZ:
		return 100000;
	case SPEED_SEL_40HZ:
		return 25000;
	case SPEED_SEL_640HZ:
		return 1563;
	case SPEED_SEL_1280HZ:
		return 781;
	}
	return 0;
}

#if 0
/**
 * adc format: singned 24bit
 * positive max 7FFFFFH
 * negative max 800000H
 */
static inline bool cs1237_data_validate(uint32_t value)
{
	if (value < 0X007FFFFF && value > 0XFF800000)
		return false;
	else
		return true;
}
#endif

/**
 * @brief convert adc value to voltage
 * 	voltage: (±0.5Vref/gain)/(2^23-1) * value
 * @param value: adc value
 * @return voltage(μV)
 */
static inline int64_t cs1237_get_voltage(int64_t value)
{
	struct cs1237_data *data = cs1237_get_data();
	int64_t ref  = data->ref;
	int64_t gain = cs1237_get_gain(data->config.pga);
	const int64_t full_scale = 0xFFFFFF;
	return ((ref * value) / (gain * full_scale));
}

static inline uint32_t cs1237_spi_read(uint8_t bits)
{
	struct cs1237_data *data = cs1237_get_data();
	uint32_t value = 0;
	uint8_t i;

	/* MSB first */
	for (i = 0; i < bits; i++) {
		spi_set_value(data->dclk, 1);
		ndelay(500);

		value = (value << 1) | spi_get_value(data->dio);

		spi_set_value(data->dclk, 0);
		ndelay(500);
	}

	return value;
}

static inline void cs1237_spi_write(uint32_t value, uint8_t bits)
{
	struct cs1237_data *data = cs1237_get_data();
	uint8_t i;
	
	compiletime_assert(bits <= 8, "bits must be less than 8");

	for (i = 0; i < bits; i++) {
		spi_set_value(data->dclk, 1);
		ndelay(500);

		/* MSB first */
		spi_set_value(data->dio, value & 0x80);
		value <<= 1;

		spi_set_value(data->dclk, 0);
		ndelay(500);
	}
}

static uint32_t cs1237_setup(uint8_t reg, uint8_t value)
{
	struct cs1237_data *data = cs1237_get_data();
	uint8_t tires = 0;

	while (gpio_get_value_cansleep(data->dio)) {
		if (tires++ > 10) {
			goto out;
		}
		udelay(30);
	}

	data->freq = usecs_to_jiffies(cs1237_build_time(data->config.speed));

	spin_lock_irqsave(&data->slock, data->flags);
	/* clock 0~27 discard */
	cs1237_spi_read(27);

	/* clock 28~29 */
	gpio_direction_output(data->dio, 1);
	cs1237_spi_read(2);

	/* clock 30~36 write r/w cmd */
	cs1237_spi_write(reg << 1, 7);

	/* clock 37 */
	cs1237_spi_read(1);

	/* clock 38~45 r/w reg */
	if (reg == CS1237_REG_RD) {
		gpio_direction_input(data->dio);
		value = cs1237_spi_read(8);
	} else
		cs1237_spi_write(value, 8);

	/* clock 46 */
	gpio_direction_input(data->dio);
	cs1237_spi_read(1);
	spin_unlock_irqrestore(&data->slock, data->flags);

	msleep(1);
out:
	return value;
}

static inline bool cs1237_read_adc(int32_t *val)
{
	struct cs1237_data *data = cs1237_get_data();

	spin_lock_irqsave(&data->slock, data->flags);
	if (unlikely(data->status != CS1237_STATUS_READY)) {
		spin_unlock_irqrestore(&data->slock, data->flags);
		return false;
	}

	*val = (data->adc_value << 8) >> 8; /* 24bit extend to 32bit */

	enable_irq(data->irq);
	data->status = CS1237_STATUS_PREPARE;
	spin_unlock_irqrestore(&data->slock, data->flags);
	return true;
}

static void cs1237_tasklet_handler(unsigned long arg)
{
	struct cs1237_data *data = (struct cs1237_data *)arg;

	spin_lock_irqsave(&data->slock, data->flags);
	switch (data->status) {
	case CS1237_STATUS_IDLE:
		break;

	case CS1237_STATUS_PREPARE:
		if (unlikely(spi_get_value(data->dio)))
			break;

		if (unlikely(time_after(jiffies, data->stamp + data->freq)))
			break;

		disable_irq(data->irq);
		data->status = CS1237_STATUS_READY;

		data->adc_value = (int32_t)cs1237_spi_read(24);
		cs1237_spi_read(3);
		break;

	default:
		break;
	}
	spin_unlock_irqrestore(&data->slock, data->flags);

	wake_up_interruptible(&data->wait);
}

/* cs1237_irq_handler */
static irqreturn_t cs1237_irq_handler(int irq, void *dev_id)
{
	struct cs1237_data *data = (struct cs1237_data *)dev_id;
	data->stamp = jiffies;
	tasklet_schedule(&data->tasklet);
	return IRQ_HANDLED;
}

static int cs1237_gpio_request(struct platform_device *pdev)
{
	struct cs1237_data *data = dev_get_drvdata(&pdev->dev);
	int ret;

	ret = devm_gpio_request_one(&pdev->dev, data->dclk, GPIOF_OUT_INIT_LOW,
				    "cs1237-dclk");
	if (ret) {
		dev_err(&pdev->dev, "failed to request gpio dclk\n");
		return ret;
	}

	ret = devm_gpio_request_one(&pdev->dev, data->dio, GPIOF_IN, "cs1237-dio");
	if (ret) {
		dev_err(&pdev->dev, "failed to request gpio dio\n");
		return ret;
	}

	data->irq = gpio_to_irq(data->dio);
	if (data->irq < 0) {
		dev_err(&pdev->dev, "failed to get gpio irq\n");
		return data->irq;
	}

	return ret;
}

static int cs1237_open(struct inode *inode, struct file *file)
{
	file->private_data = cs1237_get_data();
	return 0;
}

static int cs1237_close(struct inode *inodep, struct file *filp)
{
	return 0;
}

static long cs1237_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct cs1237_data *data = file->private_data;
	cs1237_config_t config;
	int32_t value;
	int ret = 0;

	switch (cmd) {
	case CS1237_IOC_SET_CONFIG:
		if (copy_from_user(&config, (void __user *)arg, sizeof(config)))
			return -EFAULT;
		config.value = cs1237_setup(CS1237_REG_WR, config.value);
		if (config.reserved)
			return -EIO;
		if (config.value != cs1237_setup(CS1237_REG_RD, 0))
			return -EIO;
		data->config = config;
		break;
	case CS1237_IOC_GET_CONFIG:
		if (copy_to_user((void __user *)arg, &data->config, sizeof(data->config)))
			return -EFAULT;
		break;
	case CS1237_IOC_GET_ADC:
		if (!cs1237_read_adc(&value))
			return -EAGAIN;
		if (copy_to_user((void __user *)arg, &value, sizeof(value)))
			return -EFAULT;
		break;
	default:
		dev_info(&data->pdev->dev, "invalid ioctl cmd %#X\n", cmd);
		return -EINVAL;
	}

	return ret;
}

static ssize_t cs1237_write(struct file *file, const char __user *buf,
			    size_t len, loff_t *ppos)
{
	struct cs1237_data *data = file->private_data;
	int32_t value;

	if (len < sizeof(value))
		return -EINVAL;

	if (copy_from_user(&value, buf, sizeof(value)))
		return -EFAULT;

	if (value < 0 || value > 0xFF)
		return -EINVAL;

	if (data->status != CS1237_STATUS_READY)
		return -EAGAIN;

	data->config.value = cs1237_setup(CS1237_REG_WR, value);
	if (data->config.reserved)
		return -EIO;

	data->config.value = cs1237_setup(CS1237_REG_RD, 0);
	dev_info(&data->pdev->dev, "cfg %#X refo %d speed %d pga %d ch %d\n", \
		 data->config.value,
		 data->config.refo_off,
		 data->config.speed,
		 data->config.pga,
		 data->config.channel);

	return sizeof(value);
}

static ssize_t cs1237_read(struct file *file, char __user *buf,
			   size_t len, loff_t *ppos)
{
	struct cs1237_data *data = file->private_data;
	int32_t value;

	if (len < sizeof(value))
		return -EINVAL;

	if (!(file->f_flags & O_NONBLOCK))
		wait_event_interruptible(data->wait, data->status == CS1237_STATUS_READY);

	if (!cs1237_read_adc(&value))
		return -EIO;

	if (copy_to_user(buf, &value, sizeof(value)))
		return -EFAULT;

	return sizeof(value);
}

static unsigned int cs1237_poll(struct file *file, poll_table *wait)
{
	struct cs1237_data *data = file->private_data;
	unsigned int mask = 0;

	poll_wait(file, &data->wait, wait);

	if (data->status == CS1237_STATUS_READY)
		mask |= POLLIN | POLLRDNORM;	/* readable */

	return mask;
}

#ifdef CONFIG_IIO
static int cs1237_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long mask)
{
	struct cs1237_data *data = iio_priv(indio_dev);	

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!cs1237_read_adc(val))
			return -EAGAIN;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = data->config.speed;
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static const struct iio_info cs1237_info = {
	.read_raw = cs1237_read_raw,
};
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
static void cs1237_remove(struct platform_device *pdev)
#else
static int cs1237_remove(struct platform_device *pdev)
#endif

{
	struct cs1237_data *data = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "remove\n");

	tasklet_disable(&data->tasklet);
	tasklet_kill(&data->tasklet);
	
	misc_deregister(&cs1237_device);

#ifdef CONFIG_IIO
	iio_device_unregister(data->indio_dev);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
	return 0;
#endif
}

static int cs1237_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np;
	struct cs1237_data *data;
#ifdef CONFIG_IIO
	struct iio_dev *indio_dev;
#endif

	np = of_find_compatible_node(pdev->dev.of_node, NULL, "cs1237");
	if (!np) {
		dev_err(&pdev->dev, "no device tree node\n");
		return -ENODEV;
	}
	dev_info(&pdev->dev, "compatible: %s\n", np->name);

#ifdef CONFIG_IIO
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (!indio_dev) {
		dev_err(&pdev->dev, "failed to allocate iio device\n");
		return -ENOMEM;
	}
	data = iio_priv(indio_dev);
	data->indio_dev = indio_dev;
#else
	data = devm_kmalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
#endif
	data->pdev = pdev;
	data->status = CS1237_STATUS_IDLE;
	data->dlock = 26;

	platform_set_drvdata(pdev, data);

	ret = of_property_read_u32(np, "gpio-sck", &data->dclk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get gpio-sck\n");
		return ret;
	}

	ret = of_property_read_u32(np, "gpio-mosi", &data->dio);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get gpio-mosi\n");
		return ret;
	}
	dev_info(&pdev->dev, "dclk: %d, dio: %d\n", data->dclk, data->dio);

	ret = of_property_read_u32(np, "ref-voltage", &data->ref);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get ref-voltage, set default 3300 mV\n");
		data->ref = 3300000;
	}

	ret = cs1237_gpio_request(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request gpio\n");
		return ret;
	}

#ifdef CONFIG_IIO
	/* register iio device */
	data->channels[0].type = IIO_VOLTAGE;
	data->channels[0].indexed = 1;
	data->channels[0].channel = 0;
	data->channels[0].address = 0;
	data->channels[0].info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	data->channels[0].info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = "cs1237";
	indio_dev->info = &cs1237_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = data->channels;
	indio_dev->num_channels = CS1237_NUM_CHANNELS;
	indio_dev->available_scan_masks = (unsigned long *)BIT(0);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register iio device\n");
		return ret;
	}
#endif

	/* register misc device */
	ret = misc_register(&cs1237_device);
	if (ret) {
		dev_err(&pdev->dev, "failed to register misc device\n");
#ifdef CONFIG_IIO
		goto unregister_iio;
#else
		goto free_data;
#endif
	}
	dev_set_drvdata(cs1237_device.this_device, pdev);

	spin_lock_init(&data->slock);
	init_waitqueue_head(&data->wait);
	tasklet_init(&data->tasklet, cs1237_tasklet_handler, (unsigned long)data);

	data->status = CS1237_STATUS_PREPARE;
	ret = devm_request_irq(&pdev->dev, data->irq, cs1237_irq_handler,
			       IRQF_TRIGGER_FALLING, "cs1237", data);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq %d\n", data->irq);
		goto unregister_misc;
	}

	data->config.value = cs1237_setup(CS1237_REG_WR, 0x2C);
	if (data->config.reserved) {
		dev_err(&pdev->dev, "can not setup cs1237\n");
		ret = -EIO;
		goto unregister_misc;
	}

	data->config.value = cs1237_setup(CS1237_REG_RD, 0);
	dev_info(&pdev->dev, "cfg %#X refo %d speed %d pga %d ch %d\n", \
		 data->config.value,
		 data->config.refo_off,
		 data->config.speed,
		 data->config.pga,
		 data->config.channel);
	goto done;
unregister_misc:
	misc_deregister(&cs1237_device);
#ifdef CONFIG_IIO
unregister_iio:
	iio_device_unregister(indio_dev);
#else
free_data:
	devm_kfree(&pdev->dev, data);
#endif
done:
	if (ret)
		dev_err(&pdev->dev, "failed to probe %d\n", ret);
	return ret;
}

static const struct platform_device_id cs1237_id_table[] = {
	{ "anycubic", 0 },
	{},
};

static const struct of_device_id cs1237_of_match[] = {
	{ .compatible = "cs1237", },
	{},
};

static struct platform_driver cs1237_driver = {
	.probe = cs1237_probe,
	.remove = cs1237_remove,
	.id_table = cs1237_id_table,
	.driver = {
		.name = "cs1237",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cs1237_of_match),
	},
};

module_platform_driver(cs1237_driver);

MODULE_DESCRIPTION("CS1237 ADC Driver");
MODULE_AUTHOR("Zhengzhuorui <zhengzhuorui@anycubic.com>");
MODULE_LICENSE("GPL");
