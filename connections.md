
# Connections


Ground, vcc, and battery +ve are connected via a three-strip piece of veroboard. 

The battery is connected via some scraps of copper (held on with tape) and a [switch](https://cpc.farnell.com/eao/09-03290-01/slide-switch-spdt-vert/dp/SW03106?CMP=TREML007-005) to the JST connector that small Li-Pos are often sold with. 



A from schematic. E: can it do PWM. C: the normal Arduino name.  B: the Pro Micro board stencil.  A: the chip pin, from the schematic and matching up with C/B.   E,D,C,B from [here](https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide/hardware-overview-pro-micro). 


|             |   E  | D  |C |B  |  A              | A          |B  | C   | D   |E    |                                                      |
|-------------|------|----|--|---|-----------------|------------|---|-----|-----|-----|------------------------------------------------------|
|             |      | TX |1 |TCO| PD3(TX)         |            |RAW|     |     |     | battery connector +ve                                |
|             |      | RX |0 |RXI| PD2(RX)         |            |GND|     |     |     | battery connector -ve / NRF24L01+ -ve / neopixel -ve |
|             |      |    |  |GND|                 |            |RST|     |     |     |                                                      |
| MPU9150 GND |      |    |  |GND|                 |            |VCC|     |     |     | MPU9150 +ve / NRF24L01+ +ve / neopixel +ve           |
| MPU9150 SDA |      |SDA |2 |2  | PD1(SDA)        |  PF4(ADC4) |A3 | 21  | A3  |     |                                                      |
| MPU9150 SCL |  PWM |SCL |3 |3  | PD0(SCL)        |  PF5(ADC5) |A2 | 20  | A2  |     |                                                      |
|             |      | A6 |4 |4  | PD4(ADC8)D4/A6  |  PF6(ADC6) |A1 | 19  | A1  |     | neopixel data                                        |
|             |  PWM |    |5 |5  | PC6             |  PF7(ADC7) |A0 | 18  | A0  |     | NRF24L01+ CE                                         |
|             |  PWM | A7 |6 |6  | PD7(ADC10)D6#/A7|  PB1(SCK)  |15 | 15  | SCLK|     | NRF24L01+ SCK                                        |
| MPU9150 INT |      |    |7 |7  | PE6 D7          |  PB3(MISO) |14 | 14  | MISO|     | NRF24L01+ MISO                                       |
|             |      | A8 |8 |8  | PB4(ADC11)      |  PB2(MOSI) |16 | 16  | MOSI|     | NRF24L01+ MOSI                                       |
|             |  PWM | A9 |9 |9  | PB5(ADC12)D9#/A9|  PB6(ADC13)|10 | 10  | A10 | PWM | NRF24L01+ CSN                                        |
     

<code>
NRF24L01+ from top of board: 

     2  4 6 8   VCC CS MOSI IRQ
    [1] 3 5 7   GND CE SCK  MISO

from bottom:

     VCC  2   [1] GND
   CSN 10 4    3  CE 9
 MOSI 16  6    5  SCK 15
          8    7  MISO 14
</code>
