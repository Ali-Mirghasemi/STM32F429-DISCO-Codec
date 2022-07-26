# STM32F429-DISCO-Codec
Codec example for STM32F429 microcontroller

## Packet

| HEADER (8x Byte) | DATA (N Byte) | FOOTER (4x Byte) |
|------------------|---------------|------------------|

## Header

| First Sign (2x Byte) | Packet Size (4x Byte) | Second Sign (2x Byte) |
|----------------------|-----------------------|-----------------------|

## Commands

| Type    | Code | Data Size | Description        |
|:-------:|:----:|:---------:|:------------------:|
| Error   | FF   | 5         | Return error code  |
| Unknown | 00   | 1         | Unknown Command    |
| Set Led | 01   | 3         | set state of led   |
| Get Led | 02   | 3         | return leds states |
| Ok      | 80   | 1         | successful command |

#### Command Frame

| Type (1x Byte) | Params (Nx Byte) |
|----------------|------------------|

#### Set LED
For change led state
| Type (1x Byte) | LED (1x Byte) | State (1x Byte) |
|----------------|---------------|-----------------|

- Ex:
    - Set LED0:     33 CC 00 00 00 03 55 AA 01 00 01 CC 33 AA 55
    - Clear LED1:   33 CC 00 00 00 03 55 AA 01 01 00 CC 33 AA 55

#### Get LED

Request: 
| Type (1x Byte) | 
|----------------|

Response:
| Type (1x Byte) | LED0 (1x Byte) | LED1 (1x Byte) |
|----------------|----------------|----------------|


- Ex:
    - Req LEDs:     33 CC 00 00 00 01 55 AA 02 CC 33 AA 55
    - Res LEDs:     33 CC 00 00 00 03 55 AA 02 01 CC 33 AA 55


#### Error

| Type (1x Byte) | Code (4x Byte) |
|----------------|----------------|

- Ex:
    - 33 CC 00 00 00 05 55 AA FF 00 00 00 FF CC 33 AA 55




