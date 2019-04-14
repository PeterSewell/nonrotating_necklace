# Non-rotating necklace

<p>
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5264.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5267.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5261.optimised.gif?raw=true" height="200">
</p>


## What is it?

This <em>non-rotating necklace</em> uses sensors to determine its orientation with respect to magnetic North and illuminates four coloured blobs (on a circular LED strip) at the four cardinal points: North, East, South, and West.  The coloured blobs thus stay more-or-less stable as the wearer rotates - contrary to one's usual expectation that the two would rotate together. 

Additionally: 
- each coloured blob slowly changes colour, following a random walk in a region of colour space;
- the light intensity is scaled with the wearer's rotation speed, becoming brighter if they spin;
- each blob is actually a (mild) colour gradient, not a uniform colour, and the direction of the gradient flips depending on whether the wearer is rotating clockwise or anticlockwise;
- the sensor data and reconstructed angle can be sent by short-range radio to a receiver, to be displayed live or recorded on a laptop. 

## The hardware

The electronics consists of:
- a [Sparkfun Pro Micro 3.3v/8MHz Arduino](https://www.sparkfun.com/products/12587) processor (much cheaper clones are available);
- an MPU-9150 board, containing a three-axis accelerometer, gyroscope, and magnetometer;
- an nRF24L01P+ 2.4Gz Transceiver board, for short-range radio; and
- half an [Adafruit NeoPixel Digital RGB LED Strip 144 LED - 1m White](https://www.adafruit.com/product/1507), ie 72 pixels; and
- a Fenix ARB-L16-700U 3.6v 700mAh Li-ion battery, with built-in micro-USB charging socket.

The first three are wired up so they fold into a tight package (insulated with tape):

<p>
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5212.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5221.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5222.JPG?raw=true" height="200">
</p>


E,D,C,B from https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide/hardware-overview-pro-micro
A from schematic

E: can it do PWM
C: the normal Arduino name
B: the Pro Micro board stencil
A: the chip pin, from the schematic and matching up with C/B



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
     


NRF24L01+ from top of board: 

     2  4 6 8   VCC CS MOSI IRQ
    [1] 3 5 7   GND CE SCK  MISO

from bottom:

     VCC  2   [1] GND
   CSN 10 4    3  CE 9
 MOSI 16  6    5  SCK 15
          8    7  MISO 14


<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5244.JPG?raw=true" height="200">
</p>





https://cpc.farnell.com/eao/09-03290-01/slide-switch-spdt-vert/dp/SW03106?CMP=TREML007-005



<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5231.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5245.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5247.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5248.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5264.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5271.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5290.JPG?raw=true" height="200">



