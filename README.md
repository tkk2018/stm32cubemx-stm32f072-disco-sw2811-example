# stm32cubemx-stm32f072-disco-sw2811-example
An example of SW2811 with STM32F072-DISCO board by using the STM32CubeMX

## Specifications

### STM32CubeMX
**Software version**<br />
>4.19.0

**STM32CubeF0 version**<br />
>1.7.0

### SW2811
The data transmission time (TH + TL = 1.25u±600ns)<br />

| Logic | Description | Time | Floating |
| --- | --- | --- | --- |
| T0H | 0 code, high level time | 0.3µs | ±0.15µs |
| T0L | 0 code, low level time | 0.9µs | ±0.15µs |
| T1H | 1 code, high level time | 0.6µs | ±0.15µs |
| T1L | 1 code, low level time | 0.6µs | ±0.15µs |
| RES | Reset code, low level time | 80µ | ±0.15µs |


## References
http://fabioangeletti.altervista.org/blog/stm32-interface-ws2812b/?doing_wp_cron=1530408536.5406301021575927734375<br />
