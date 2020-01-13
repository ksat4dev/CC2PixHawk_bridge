# CC2PixHawk_bridge
An inter-process communication is to be established using already existing light framework [1] for platform-independent communication.

In my example, the two platforms include - an Intel Upboard for high-level computing and PixHawk Flight Controller running PX4 for low-level control. 

Although, ZeroMQ libraries have been used to establish communication between opti_sub.cpp and SerialPort.cpp, the serial port is not able to receive data due to integration issues (needs debugging)

However, opti_sub.cpp and a test application (subscriber.cpp) are able to exchange (pose) messages.  



Reference to source and original low-level ASIO Framework

[1] https://github.com/flyingmachines/litefw/serial_bridge
[2] https://objectcomputing.com/resources/publications/mnb/multi-platform-serial-interfacing-using-boost-a-gps-sensor-and-opendds-part-i



