# solarRouter
Solar router based upon Linky messages to pilot a dimmer connected to an electrical heating water tank

Some links related to Linky

https://hallard.me/demystifier-la-teleinfo/

http://hallard.me/pitinfov12/

https://gammatroniques.fr/connaitre-sa-consommation-electrique-avec-home-assistant/

https://morbret.monsite-orange.fr/page-59f35f96b4860.html

Documentation Enedis for Linky

https://www.enedis.fr/media/2035/download

This version of code was tested with an allogen lamp of 500W.

Next step is to test on a real electrical water tank.

Note that esp32 is used because it has several UARTS that helps to solve issues in reception of Linky messages (e.g. The library <AltSoftSerial.h> looses characters while it is not the case with <HardwareSerial.h>)
