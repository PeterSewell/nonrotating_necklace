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
- an nRF24L01P+ 2.4Gz Transceiver board, for short-range radio; 
- half an [Adafruit NeoPixel Digital RGB LED Strip 144 LED - 1m White](https://www.adafruit.com/product/1507), ie 72 pixels; and
- a Fenix ARB-L16-700U 3.6v 700mAh Li-ion battery, with built-in micro-USB charging socket.

The first three are wired up so they fold into a tight package (insulated with tape):

<p>
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5212.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5221.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5222.JPG?raw=true" height="200">
</p>

and then that and the battery are cable-tied to the necklace structure:
<p>
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5244.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5245.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5248.JPG?raw=true" height="200">
</p>
See the [connections](./connections.md).



The necklace structure is a 500mm length of [Stainless Steel Knitted
Wire Mesh Tubing - Hop Filter Mesh - 22mm Diameter (from The Mesh
Company)](https://www.wireandstuff.co.uk/products/Hop-Filter-Mesh---Stainless-Steel-Knitted-Tubing---22mm-Width-79.html),
with the ends reinforced with some 0.5 and 1.0mm stainless wire, and a
hook from 2mm stainless wire.  The neopixel strip is just loose within
this, insulated at the open end with clear tape, and held with a cable
tie on its connecting wires at one end.

The diffusing shell is a piece of silk satin, nominally white but
actually cream coloured, cut from a 0.2m length from John Lewis (506
10321 barcode 2986 9855).  Overall 580mm long, 130mm wide for 95mm,
97mm wide for the main part, with a diagonal intermediate part, with
fray-check solution applied to the edges before cutting. Hand-sewn
into a tube
















<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5231.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5247.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5264.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5271.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5290.JPG?raw=true" height="200">



