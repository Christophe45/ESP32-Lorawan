# ESP32-Lorawan
ESP32 Lorawan beehive connected

This project is to make a connected hive that can control the weight, the internal and external temperature of the hive as well as the humidity. Data recovery is done with a Lora RFM95 module and reception of this data through a gateway connected to TTN. The data received on TTN is then sent to Node-red and sent back to Blynk's servers.


![alt text](https://github.com/Christophe45/ESP32-Lorawan/blob/master/pictures/projet.JPG)

The schematic of the Lorawan is:
![alt text](https://github.com/Christophe45/ESP32-Lorawan/blob/master/pictures/schema1.JPG)

PCB 3D :

![alt text](https://github.com/Christophe45/ESP32-Lorawan/blob/master/pictures/loranode-esp32-V3.jpg)

New version on test with a GPS module NOE6NV2 and an accelerometer MPU6050 used for alarm detection when the hive is moved.

schematic:
![alt text](https://github.com/Christophe45/ESP32-Lorawan/blob/master/pictures/SchemaV3.JPG)

PCB 3D :

![alt text](https://github.com/Christophe45/ESP32-Lorawan/blob/master/pictures/loranode-esp32-V3-GPS.jpg)

![alt text](https://github.com/Christophe45/ESP32-Lorawan/blob/master/pictures/balance.jpg)
