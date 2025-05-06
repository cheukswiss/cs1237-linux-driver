# CS1237 ADC Driver

CS1237 is a high-precision, low-power analog-to-digital converter with one differential input channel and a built-in temperature sensor and high-precision oscillator. The MCU can communicate with and configure CS1237 through 2-wire SPI interfaces, SCLK, and DRDY, such as the channel, PGA, and output rate selection.

## Device Tree Example

```
anycubic {
	compatible = "anycubic";

	cs1237@0 {
	compatible = "cs1237";
	gpio-sck = <&gpio 0 GPIO_ACTIVE_HIGH>;
	gpio-mosi = <&gpio 1 GPIO_ACTIVE_HIGH>;
	ref-voltage = <3300000>; /* 3.3V */
	};
};
```

## Chip features
- Built-in crystal oscillator
- Integrated temperature sensor
- Power down function
- **2-wire SPI** interface with a maximum rate of `1.1 MHz`

## ADC functional features

- 24 bits with no missing codes
- PGA amplification time options: `1x`, `2x`, `64x`, and `128x`
- 1~24-bit differential input with no missing codes. 20-bit ENOB (5 V)/19.5 bits (3.3 V), when PGA = 128x
- **Effective noise**: PGA = 128x, 10 Hz: 30 nV
- **INL**: Less than 0.0015%
- Output rate options: `10 Hz`, `40 Hz`, `640 Hz`, and `1.28 kHz`
- Short circuit function

## Technical Documentation

**Product manual(1):** [DS_CS1237_v1.1](https://en.chipsea.com/api/uploadfileFront?content=https://chipsea-obs.obs.cn-south-1.myhuaweicloud.com:443/uploads/DS_CS1237_V1.1_1668155164.pdf&title=DS_CS1237_V1.1)
