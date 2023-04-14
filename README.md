# PZEM-V2-1PHASE-MPP-ASYNC
PZEM V2/V3-1PHASE MPP-ASYNC
This is my own imlementation alghoritm of reading PZEM V2 sensor (without library) for working in AM enviroment https://sites.google.com/site/mppsuite/Home
All alghoritms that presents in internet is based on the same Serial reading technic that include "waiting till read"
That blocks CPU for a while making the device difficult to read over the network and generally slow down.
That was quite important for succsessfull and stable work with AM Server software https://sites.google.com/site/mppsuite/Home
So , therefore here the asynchronous alghoritm is represented. It read the device every 250 ms and send command every 500ms.
You can change these parameters from device's web page, marked as READ_S and COMMAND_S
