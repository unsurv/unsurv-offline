# unsurv-offline

Currently available on [crowdsupply](https://www.crowdsupply.com/unsurv-technologies/unsurv-offline)

This project is part of the unsurv framework to fight privacy invasive technologies in the offline world. unsurv-offline logs your encounters with surveillance cameras and creates a daily report for you via NFC to review in the unsurv-companion Android app.

<img src="https://github.com/unsurv/unsurv-offline/blob/master/images/first.jpeg" alt="main PCB"
	title="unsurv-offline" width="480" height="360" />

Demo: https://vimeo.com/497241039



This device can easily be adapted to different use cases via the Arduino IDE. Arduino library + board integration are planned.

List of features

- ESP32 D4
- ublox M8C GNSS receiver
- RF430CL330H NFC 
- BMA400 accelerometer
- SD card reader
- MCP73831 single cell LiPo charging
- CH340 USB to Serial

Software:
- [unsurv-offline](https://github.com/unsurv/unsurv-offline/tree/master/software/unsurv-offline)
- [other examples](https://github.com/unsurv/unsurv-offline/tree/master/software/examples) (tap to log location, location logging + step counting)

Hardware:
- [PCB design](https://github.com/unsurv/unsurv-offline/tree/master/unsurv_offline_pcb/main) (KiCad files)
- [case design](https://github.com/unsurv/unsurv-offline/tree/master/case) (FreeCAD + STL files)

Special thanks to:

- SparkFun for maintaining the [ublox library](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library)
- [awong1900](https://github.com/awong1900/RF430CL330H_Shield) for his work on the RF430
- [kriswiner](https://github.com/kriswiner/BMA400) for his BMA400 library 
