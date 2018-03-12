
Wurstradar
==========

Sausage radar. Neat doppler radar that fits into a sausage can.


Required hardware
=================

RSM2650 Radar movement alarm unit (this actuall is an Innosent IPS-265.pdf)

 * `http://www.produktinfo.conrad.com/datenblaetter/500000-524999/506343-da-01-en-RADARBEWEGUNGSM__MOD__STEREO_4_75__5_25V.pdf`
 * `https://www.innosent.de/fileadmin/media/dokumente/datasheets/IPS-265.pdf`

1bitsy STM32F415 on a tiny breakout board

 * `https://1bitsy.org/`

Used Simpson Voltmeter 5V as analog dial.

 * `https://www.google.de/search?q=simpson+5v+ac&tbm=isch`

Generic powerbrick to 5V

 * e.g. TI TPSM84205


Compiling
=========

Build libopencm3 first (`cd libopencm3; make`), then build in `src`.
You can set the measurement angle and its proper doppler frequency via
commandline: `make DOPPLER_HZ_PER_POINT1_KPH=314 MEASUREMENT_DEGREE=45`

