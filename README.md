# Non-rotating necklace

<p>
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5264.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5267.JPG?raw=true" height="200">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5261.optimised.gif?raw=true" height="200">
</p>

<p>

[![Fibonacci RMI Java EE](http://img.youtube.com/vi/nX_inqaAzOI/0.jpg)](https://www.youtube.com/watch?v=nX_inqaAzOI&feature=youtu.be&hd=1 "RMI Fibonacci Java")


[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/8gVy3Wga34s/0.jpg)](https://www.youtube.com/watch?v=8gVy3Wga34s)


<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5271.JPG?raw=true" height="190">


<img src="https://www.youtube.com/watch?v=8gVy3Wga34s&feature=youtu.be" height="190">
</p>


## What is it?

This <em>non-rotating necklace</em> uses sensors to determine its orientation with respect to magnetic North and illuminates four coloured blobs (on a circular LED strip) at the four cardinal points: North, East, South, and West.  The coloured blobs thus stay more-or-less stable as the wearer rotates - interestingly contrary to one's usual expectation that the two would rotate together, and especially accentuating dance movement. 


Additionally: 
- each coloured blob slowly changes colour, following a random walk in a region of colour space;
- the light intensity is scaled with the wearer's rotation speed, becoming brighter if they spin;
- each blob is actually a (mild) colour gradient, not a uniform colour, and the direction of the gradient flips depending on whether the wearer is rotating clockwise or anticlockwise;
- the sensor data and reconstructed angle can be sent by short-range radio to a receiver, to be displayed live or recorded on a laptop. 


## How it works

### Orientation estimation

The sensors measure the current linear acceleration, gyroscope
rotation rate, and magnetic field, each in three axes.  

We assume the wearer will be largely vertical, with the necklace not
necessarily horizontal, but more-or-less stable in the reference frame
of the wearer's shoulders.  In normal use, as the wearer moves, the
acceleration vector is dominated by gravity.  To estimate which way is
up, in the device reference frame, we take a smoothed
(low-pass-filtered) version of the acceleration data.  We project the
gyroscope and magnetic field data onto this horizontal plane, which
also greatly simplifies the mathematics and computation needed,
compared to the general 3d orientation estimation problem.

The magnetic field data gives an absolute orientation with respect to
the Earth's magnetic field, but with a very noisy signal.  On the
other hand, the gyroscope gives reasonably good data about the
rotation rate, which one can integrate to get an orientation estimate,
but it will tend to drift with time as errors accumulate.  We fuse the
two with a complementary filter, choosing the time constant
empirically to balance reducing jitter from the magnetometer noise
against giving a fast response to quick rotation.  Additionally, when
the angular rotation rate is small (less than two pixels/second), we
smooth the resulting orientation estimate.

### Display

Meanwhile, each cardinal-point region of LEDs has a base colour from a
random walk within a fixed RGB cuboid.  Each random walk has a current
position, velocity, and duration (in cycles). When the duration
reaches 0, a new random velocity is chosen. The walk bounces off the
internal faces of the RGB cuboid.  Each region is three pixels wide,
anti-aliased onto four pixels of the strip.  One end is the base
colour, and subsequent pixels are shifted by a factor of the current
pixel colour velocity - this gives each region a slowly changing
colour and a slowly changing colour gradient.  The gradient direction
is flipped based on the direction of rotation (to give a mild visual
highlight to changes of rotation direction), slightly offset from zero
to avoid flicker at rest.  The regions have brightness scaled by the
angular velocity (above 0.2 rad/s).

The overall brightness is good for normal or dim artificial lighting;
clearly visible but not dazzling, and well-diffused without the
individual pixel LED points being obtrusive.  In overcast outdoor
light, it's visible but rather subdued.

The refresh rate, for a main loop including reading the sensor data,
computing the orientation estimate, and refreshing the display, is a
comfortable 40-50 Hz.  Reading the magnetometer data takes a long time
(around 21ms) and is not needed at high frequency, so is only done
once every four cycles.  Printing data to the serial link slows this
down considerably, while sending data over the radio link has little
effect.


### Power

Power is from a Fenix ARB-L16-700U 3.6v 700mAh Li-ion battery, which
conveniently has a built-in micro-USB charging socket - so the
necklace can connected either to the battery or to the Pro Micro
micro-USB socket (but not both at once!).  Power to the neopixel strip
is taken from the Pro Micro regulated power, which will be less
efficient than the battery supply, but makes it conveniently
comparable when connected to the 5v micro-USB supply.  Adafruit
estimate around 20mA per full-brightness single-colour pixel, at 5v,
but the measured power here is much less, around 8mA per full-scale
single-colour pixel.  Running all pixels full-power would still vastly
exceed the Pro Micro regulator (1730mA vs 500mA max), and also give
tiny battery life - but that would also be uncomfortably bright for
non-daytime use.  Measured power, running on the battery, is around
65mA before the neopixels start up, 85mA steady state, and up to 220mA when
rotating.  That should give battery life of 3 to 8 hours.
Power consumption is not notably affected by whether the radio is used. 


### Radio

One can see the sensor and orientation estimate data simply by
printing it to the Arduino serial link, but it's much more convenient
to be able to do this wirelessly.  The nRF24L01P+ transceiver gives a
cheap and easy way to do this, sending 32-byte packets to a similar
receiver attached to another Arduino, itself connected via USB to a
laptop.


### Client

<em>Visualising</em> the data is essential for development, and also
 interesting to see the sensor properties and the quantitivae
 parameters of movement in use.  The client software is a simple
 application that reads the incoming data from a serial device (either
 directly connected to the necklace USB, or indirectly via the radio
 and a receiving Arduino), and uses gnuplot to draw various graphs
 dynamically.  It can also record and replay data streams. 



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
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5244.JPG?raw=true" width="250">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5245.JPG?raw=true" width="250">
<img src="https://github.com/PeterSewell/nonrotating_necklace/blob/master/media/DSD_5248.JPG?raw=true" width="250">
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
into a tube.  We tried various other diffusers, including other John
Lewis fabrics and Rosco quiet light grid cloth (E462) (which looks
promising but is only sold in long rolls); this is a good balance
between diffusion and light loss, and feels good on the skin.


















