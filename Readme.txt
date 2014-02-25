This is a self-project about developing an embedded system that can gather information about an in-flight RC airplane and transmit the data back to the ground station, which is my laptop for now. The flight information and corresponding sensors are as the following:

    attitude   <----->  MPU6050 motion sensor
    altitude   <----->  BMP085 barometer
    airspeed   <----->  MPXV7002 differential pressure sensor
    position   <----->  Ublox Neo-6M GPS
    ground speed <--->  Ublox Neo-6M GPS
    battery    <----->  Current/Voltage sensor

This embedded system is based on the STM32F4-Discovery board, and the RT-Thread real-time OS. Information about the RT-Thread OS can be found on Wikipedia or here: http://en.rt-thread.org/

[2013-12] I decided to use the MAVLink Protocol when transmitting data back to the Ground Station. More Info about MAVLink can be found here: http://qgroundcontrol.org/mavlink/start
