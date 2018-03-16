
![Wurstradar](https://github.com/dpiegdon/wurstradar/blob/master/pictures/logo.jpg?raw=true)

Sausage radar. Simple doppler radar that fits into a sausage can.
Named after "Wurstblinker" from the german movie [Werner](https://www.youtube.com/watch?v=n31j2_dkaCo),
as it also is meant to be mounted on a motorcycle as an alternative to the speedometer.

![DataFlow](https://github.com/dpiegdon/wurstradar/blob/master/pictures/dataflow.png?raw=true)

![Prototype](https://github.com/dpiegdon/wurstradar/blob/master/pictures/photos/prototype.jpg?raw=true)

![PCB](https://github.com/dpiegdon/wurstradar/blob/master/pictures/photos/pcb.jpg?raw=true)


Required hardware
=================

![TestSetup](https://github.com/dpiegdon/wurstradar/blob/master/pictures/photos/testsetup.jpg?raw=true)

RSM2650 Radar movement alarm unit (this actuall is an Innosent IPS-265.pdf)

 * `http://www.produktinfo.conrad.com/datenblaetter/500000-524999/506343-da-01-en-RADARBEWEGUNGSM__MOD__STEREO_4_75__5_25V.pdf`
 * `https://www.innosent.de/fileadmin/media/dokumente/datasheets/IPS-265.pdf`

1bitsy STM32F415 on a tiny breakout board

 * `https://1bitsy.org/`

Used Simpson Voltmeter 5V as analog dial.

 * `https://www.google.de/search?q=simpson+5v+ac&tbm=isch`

Generic powerbrick to 5V

 * e.g. TI TPSM84205

A JTAG debugger.

 * e.g. Black Magic Probe


Required software
=================

Only `arm-none-eabi-gcc`.


Compiling
=========

Clone first: `git clone --recursive https://github.com/dpiegdon/wurstradar.git`

Then build libopencm3: `cd wurstradar/libopencm3 && make`

Then build the firmware: `cd ../src && make` - 
you can set the measurement angle and its proper doppler frequency via
commandline: `make DOPPLER_HZ_PER_POINT1_KPH=314 MEASUREMENT_DEGREE=45`

Flash it to the 1bitsy. E.g. with a Black Magic Probe.


Team
====

 * David Owczarek - `fixme` - hardware

 * David R. Piegdon - @dpiegdon `dgit at piegdon dot de` - concept, firmware, hardware and assembly

 * Florian Schwanse - `florian at schwanse dot de` - 3d printed parts and case

 * Frank Terbeck - @ft `ft at bewatermyfriend dot org` - concept and hardware

 * Iqbal Maraqa - `iqbal underscore maraqa at hotmail dot com` - analog dial face

 * Manoel Brunnen - @mbrunnen `manoel dot brunnen at gmail dot com` - firmware

 * Martin Gritzan - @mgritz `martin dot gritzan at gmail dot com` - firmware

 * Möffi ... - `spsm at mailbox dot org` - 3d printed parts


