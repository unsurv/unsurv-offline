
RF430CL330H_Shield (DNFC Tag)
==================

The DNFC tag is reprogrammable with a microcontroller with an I2C interface thanks to TI's RF430CL330H dynamic NFC transponder IC.

## Info
- Indiegogo - [info](https://www.indiegogo.com/projects/dnfc-tag-nfc-that-interacts-with-microcontroller)
- DNC Tag - [Buy](http://www.elecfreaks.com/store/dnfc-tag-p-745.html?zenid=5854397ad30b2f1aeee5a31489913af8)

![nfc tag](http://www.elecfreaks.com/store/images/DNFC%20TAG-01.jpg)

## Start up:
- connect Arduino 2560/UNO with NFC Tag, see [Connect Guide](https://github.com/awong1900/RF430CL330H_Shield/blob/master/README.md#connect-guide).
- make this library to arduino IDE
- use example to test your Tag

## NFC flower demo
use NFC to water flower, and take the temperature, humidity, soil moisture.
- Github Arduino Code - [water system](https://github.com/nfcwormhole/NFC-Flower/tree/master/Arduino/nfc_flower)
- Github Android App Code - [water system](https://github.com/nfcwormhole/NFC-Flower/tree/master/Android/NFC_Flower)
- Google Play download- [water system](https://play.google.com/store/apps/details?id=com.flower.nfcaction)


## Connect Guide

```
//                                            (Host/Tester)
//                  DNFC Tag                 Arduino 2560/UNO
//                  (Slave)                      Master
//             _________________            _________________
//     3.3V<--|VCC           SDA|<-------->|P20(2560)/A4(UNO)|
//            |                 |    I2C   |                 |
//            |              SCL|<-------->|P21(2560)/A5(UNO)|
//            |                 |          |                 |
//      GND<--|GND          /RST|<---------|P4               |
//            |             INTO|--------->|P3(INT1)         |
//            |                 |          |                 |
//            |                 |          |                 |
//            |                 |          |                 |
//            |_________________|          |_________________|
```
## Wiki & Forum

- [wiki](http://www.elecfreaks.com/wiki/index.php?title=Dynamic_NFC_Tag) - More resource about this NFC Tag
- [Forum](http://www.elecfreaks.com/forum/) - Ask questions and share your feedback

[![Analytics](https://ga-beacon.appspot.com/UA-61411142-2/RF430CL330H_Shield)](https://github.com/igrigorik/ga-beacon)
