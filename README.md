# ADS126X
Evaluate performance of 32-bit ADC from Texas Instruments - ADS1263 or ADS1262 on small signal measurements (microVolt/NanoVolt)


It is ongoing project for personal curiocity and the performance evaluation on small signal measurement will be kept updated in the following weeks until it will be finished. The absulate accuracy is not evaluted here which requires some precise equipment out of my reach.
 - Now only initial data has been collected under the directory of [data](data), which shows ADS1263 under test with default setting (20sps, gain 1, FIR) and shorted AN0 and AN1 to the internal 2.5V reference gives 385nVrms noise, 23.6bits ENOB, consist with the performance provided in its datasheet. Further processing collected data with runing average 100 seconds window gives 4.3nVrms noise, 400 seconds window gives 1.7nVrms noise.

Document on hardware and other related will be uploaded later.

The AD126X driver has been built based on the work of [dinamitemic](https://github.com/dinamitemic/ADS126X)
