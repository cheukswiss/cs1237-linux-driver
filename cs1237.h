#ifndef _CHIPSEA_CS1237_H_
#define _CHIPSEA_CS1237_H_

#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/ioctl.h>

#define CS1237_REG_WR			0x65
#define CS1237_REG_RD			0x56

#define CS1237_REG_REFO_OFF		BIT(6)
#define CS1237_REG_SLEEP_SEL		BIT(5) | BIT(4)
#define CS1237_REG_PGA_SEL		BIT(3) | BIT(2)
#define CS1237_REG_CH_SEL		BIT(1) | BIT(0)

typedef enum {
	REF_OUTPUT_ON	= 0,
	REF_OUTPUT_OFF	= 1,
	REF_OUTPUT_MAX
} ref_output_e;

typedef enum {
	SPEED_SEL_10HZ = 0,
	SPEED_SEL_40HZ,
	SPEED_SEL_640HZ,
	SPEED_SEL_1280HZ,
	SPEED_SEL_MAX
} speed_sel_e;

typedef enum {
	PGA_SEL_1 = 0,
	PGA_SEL_2,
	PGA_SEL_64,
	PGA_SEL_128,
	PGA_SEL_MAX
} pga_sel_e;

typedef enum {
	CH_SEL_A = 0,
	CH_SEL_RESERVED,
	CH_SEL_TEMP,
	CH_SEL_SC,
	CH_SEL_MAX
} channel_sel_e;

typedef enum {
	STATUS_OK		= 0,
	STATUS_ERR,
	STATUS_LOW_POWER,
	STATUS_RESET,
	STATUS_READY,
	STATUS_MAX
} status_e;

typedef union {
	struct {
		uint8_t		channel		: 2;
		uint8_t		pga		: 2;
		uint8_t		speed		: 2;
		uint8_t		refo_off	: 1;
		uint8_t		reserved	: 1;
	};
	uint8_t			value;
} cs1237_config_t;

#define CS1237_IOC_MAGIC		'k'
#define CS1237_IOC_SET_CONFIG		_IOW(CS1237_IOC_MAGIC, 1, cs1237_config_t)
#define CS1237_IOC_GET_CONFIG		_IOR(CS1237_IOC_MAGIC, 2, cs1237_config_t)
#define CS1237_IOC_GET_ADC		_IOR(CS1237_IOC_MAGIC, 3, int32_t)

#endif /* _CHIPSEA_CS1237_H_ */