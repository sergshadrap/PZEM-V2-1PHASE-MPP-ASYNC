# PZEM-V2-1PHASE-MPP-ASYNC
PZEM V2/V3-1PHASE MPP-ASYNC
This is my own imlementation alghoritm of reading PZEM V2 sensor (without library) for working in AM enviroment https://sites.google.com/site/mppsuite/Home
All alghoritms that presents in internet is based on the same Serial reading technic that include "waiting till read"
That blocks CPU for a while making the device difficult to read/aproach over the network and generally slowing it down.
That was quite important for succsessfull and stable work with AM Server software https://sites.google.com/site/mppsuite/Home
So , therefore here the asynchronous alghoritm is represented. It read the device as fast as it ready.
You can change  parameters from device's web page,   COMMAND_S - the delay between quering Pzem with new command. 
Period - how fast the device refresh values from device and reporting it to AM server
