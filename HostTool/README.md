# Motor Host Tool

Simple Python serial host for the motor driver UART protocol.

## Features

- Connect to a serial port
- Send `MOTOR_CMD_SET`
- Pack and send motor module parameters with CRC16
- Send `SET + TRANSMIT` in one click

## Protocol

The firmware expects:

```text
master_id(2, big-endian)
cmd(1)
slave_id(2, big-endian)
data_len(2, big-endian)
data(payload)
crc16(2, little-endian)
```

The parameter payload layout matches `MotorModuleStroageData_t` in
`User/Application/Inc/motor_module_manager.h`:

```text
uint16 slave_id
uint16 master_id
float   kp
float   ki
float   kd
float   maxout
float   integral_limit
float   deadband
float   lpf
float   radio
uint8   multiplier
uint32  ppr
uint8   freq
uint16  crc
```

## Run

Install dependency:

```bash
pip install pyserial
```

Start the tool:

```bash
python motor_host_tool.py
```
