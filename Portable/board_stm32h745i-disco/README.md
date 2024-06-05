# Develop with STM32CubeIDE

1. This project can be imported to the STM32CubeIDE. Create new project (CTRL+N) and select **Makefile Project with Existing Code**
2. Enter the name, and location of this directory, and select MCU ARM GCC
3. Click **Finish**

The project should be loaded and able to compile. Then you can use it as usual.

## Basic interaction from Linux

The basic configuration of the CAN interface in Linux is done by `ip` command. We need to bring the interface down, then configure it and then bring it up.
This can be done by following the code:

```
dbitrate=2000000
bitrate=1000000
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 type can bitrate ${bitrate} fd on dbitrate ${dbitrate}
sudo ip link set can1 type can bitrate ${bitrate} fd on dbitrate ${dbitrate}
sudo ip link set can0 up
sudo ip link set can1 up
```

Then we should be able to send/receive data so we can simulate the traffic by sending custom frames using `cansend` utility,
or by replaying the log using `canplayer`, or generating random traffic using `cangen`.

Some examples of sending data are below:

- `cansend can0 213##311223344`
- `canplayer -I candump.log`
- `cangen can0 -f -b -g 5`

To display sent data use `candump can0` (it can contain more than one interface so we can use `candump can0 can1` and track the traffic of two interfaces).

## STM32H745I-DISCO issues

After flashing the board may not be able to connect with the ST-Link GDB server from the IDE. To resolve this we need to delete the firmware from FLASH memory.
This can be done using *STM32CubeProgrammer* by using the GUI or CLI interface. The CLI command looks like this `STM32_Programmer_CLI -c port=SWD mode=POWERDOWN -e All`.
The **POWERDOWN** option is crucial here. Sometimes deleting only the first sector is enough so in the command replace `All` with `0` and the flashing should be fixed too.
