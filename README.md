# mc3479-linux-driver
Linux device driver for MEMSIC MC3479 accelerometer.

The MC3479 is a small form factor, integrated digital output 3-axis accelerometer with a feature set optimized for cell phones and consumer product motion sensing.

Development Status
------------------
First goal is to create a driver that works through SPI interface of the sensor. 

| Goal      | Status |
| ----------- | ----------- |
| SPI Read and Write Registers                                                |      ![](https://geps.dev/progress/100)       |
| Read from XYZ output registers and do conversion                            |      ![](https://geps.dev/progress/100)       |
| Write iio_info functions to make single shot read/writes from user space    |      ![](https://geps.dev/progress/0)         |
| Create triggered buffers and do triggered continuous sampling               |      ![](https://geps.dev/progress/0)         |
