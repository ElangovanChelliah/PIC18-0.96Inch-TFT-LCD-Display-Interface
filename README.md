# PIC18-0.96Inch-TFT-LCD-Display-Interface

This project demonstrates a simple hardware design for 0.96 inch RGB TFT IPS Display and PIC18F26K40 Interface, the given circuit schematics can be integrated to TFT IPS (0.96') required projects like medical devices, gaming console, information display, remote controller, music player, interactive button, coffee machine, vending machine. The aim of the project is to design, develop and to test the TFT functionality with PIC18F26K40 combined with custom I2C circuit design feasible for OLED involved future projects.

### Softwares used
`Altium Designer` `MPLAB X IDE` `Proteus` `Tear Term` 
### Skills
`Circuit Design` `Schematic Capture` `PCB Layout` `Debugging & Testing` `Embedded C` `Embedded Product Development`

System Design:
<table>
  <tr>
    <th>Component</th>
    <th>Specification</th>
  </tr>
  <tr>
    <td>MCU</td>
    <td>PIC16F1615</td>
  </tr>
  <tr>
    <td>OLED</td>
    <td>1.3 Inch</td>
  </tr>
  <tr>
    <td>Pixel</td>
    <td>128x64</td>
  </tr>	
  <tr>
    <td>Driver</td>
    <td>SH1106</td>
  </tr>
    <tr>
    <td>Interface</td>
    <td>I2C</td>
  </tr>
  <tr>
    <td>Oscillator</td>
    <td>INTSOC</td>
  </tr>
    <tr>
    <td>Internal Clock</td>
    <td>16MhZ</td>
  </tr>
 </table>

### Hardware Design

The designed schematics uses a seperate 662K 3.3V voltage regulator for module testing, the integrated applications can use the same schematics connected to the common voltage source of 3.3V, Altium designer is used to layout the circuit. The schematics can be transformed to desired layouts like module or custom hardware. 

<img src="https://github.com/ElangovanChelliah/PIC16-1.3Inch-OLED-Interface/blob/7bf8ae4249119983b670ed0e5f937535a9344fe7/Schematics.png" width="600">

<img src="https://github.com/ElangovanChelliah/PIC16-1.3Inch-OLED-Interface/blob/fb46c0077c1c229d86bac8b4fa5d2c523bebb977/1.3%20Inch%20OLED.png" width="300">

### Pin Diagram

<img src="https://github.com/ElangovanChelliah/PIC16-1.3Inch-OLED-Interface/blob/b93f52e42c382b0f1b97d97665717313e876241a/Pin%20Diagram.png" width="400">

### Software Design
MPLAB X IDE is used to design and develop firmware and software, copy the project files [here](https://github.com/ElangovanChelliah/PIC16-1.3Inch-OLED-Interface/tree/3b928b85792c1b3a9fc59a058e2cbee1c41ed1a5/Firmware), and execute the same to get the desired output, from the example in this project a library is created and LOGO is used to demonstrate the functionality.

Example:
```c
//****sh1106.c****//
#include "myOLED_SH1106.h"
#include "../mcc_generated_files/mcc.h"

//TEST LOGO
static unsigned char Frame[] = {
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,							
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	    
        0xFF,0x7F,0xFF,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x7F,0xFF,0x7F,							
        0x0E,0x00,0x18,0x00,0x30,0x00,0x60,0x00,0xC0,0x00,0x80,0x01,0x00,0x07,0xFF,0x7F,							
        0xFF,0x7F,0x0C,0x00,0x18,0x00,0x30,0x00,0x60,0x00,0xC0,0x00,0x80,0x01,0x00,0x03,							
        0x00,0x06,0x00,0x0C,0x00,0x18,0xFF,0x7F,0xFF,0x7F,0x00,0x00,0xE0,0x03,0xF8,0x0F,							
        0x0C,0x18,0x06,0x30,0x03,0x60,0x01,0x40,0x01,0x40,0x01,0x40,0x01,0x40,0x01,0x40,							
        0x01,0x40,0x03,0x60,0x06,0x30,0x0C,0x18,0xF8,0x07,0xE0,0x03,0x00,0x00,0x00,0x00,							
        0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0x07,0x00,0x87,0x01,0x87,0x07,0x87,0x0F,0xFF,0x1F,	
        0xFF,0x79,0xFE,0x70,0x00,0x60,0x00,0x00,0x00,0x70,0x00,0x7E,0xE0,0x7F,0xFC,0x0F,							
        0xFF,0x01,0x3F,0x00,0x3F,0x00,0xFC,0x01,0xF0,0x0F,0xC0,0x7F,0x00,0x7E,0x00,0x70,							
        0x00,0x00,0x00,0x00,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0x07,0x70,0x07,0x70,0x07,0x70,							
        0x07,0x70,0xFE,0x7F,0xFE,0x3F,0xF8,0x0F,0x00,0x00,0x00,0x00,0xFF,0x7F,0xFF,0x7F,							
        0xFF,0x7F,0x00,0x00,0x00,0x00,0x00,0x70,0x00,0x7E,0xF0,0x7F,0xFC,0x0F,0xFF,0x01,							
        0x3F,0x00,0x3F,0x00,0xFC,0x01,0xF0,0x0F,0xC0,0x7F,0x00,0x7E,0x00,0x70,0x00,0x40,
    };

```

### SH1106 Library for I2C

```c
//****sh1106.c****//

void I2C_Start(void){
    SCL = 0;
	__delay_us(1);
	SDA = 1;
	SCL = 1;
	__delay_us(1);
	SDA = 0;
	SCL = 0;
}

void I2C_Stop(void){
    SCL = 0;
	SDA = 0;
	SCL = 1;
	__delay_us(1);
	SDA = 1;
}

bool SH1106_Write_1byte(uint8_t D){
    uint8_t i;
    bool SH1106_Ack = 0;
    uint8_t Data = D;
    for(i=0;i<8;i++){
        SDA = (bool)(Data&0x80);
        NOP();
        SCL = 1;
        NOP();NOP();
        SCL = 0;
        Data = Data<<1;
    }
    SDA = 1;
    NOP();NOP();
    SCL = 1;
    NOP();NOP();
    SH1106_Ack = SDA_Ack;
    SCL = 0;
    return SH1106_Ack;
}

void SH1106_Write_Command(uint8_t Command){
    I2C_Start();
    SH1106_Write_1byte(SH1106_ADD_Write);
    SH1106_Write_1byte(SH1106_Command);
    SH1106_Write_1byte(Command);
    I2C_Stop();
}

void SH1106_Write_Data(uint8_t Data){
    I2C_Start();
    SH1106_Write_1byte(SH1106_ADD_Write);
    SH1106_Write_1byte(SH1106_Data);
    SH1106_Write_1byte(Data);
    I2C_Stop();
}

void SH1106_Clear_Screen(uint8_t disp_data){
    uint8_t page_index = 0, colume_index = 0;
    for(page_index=0;page_index<8;page_index++){
        SH1106_Write_Command(0xB0 + page_index);
        SH1106_Write_Command(0x10);
        SH1106_Write_Command(0x02);
        for(colume_index=0;colume_index<128;colume_index++){
            SH1106_Write_Data(disp_data);
        }
    }
}

void SH1106_init(void){
    I2C_Stop();
    SH1106_Write_Command(0xAE);
    SH1106_Write_Command(0xD5);
    SH1106_Write_Command(0x80);
    SH1106_Write_Command(0xAD);
    SH1106_Write_Command(0x8B);
    SH1106_Write_Command(0x32);
    SH1106_Write_Command(0x81);
    SH1106_Write_Command(0x80);
    SH1106_Write_Command(0xDA);
    SH1106_Write_Command(0x12);
    SH1106_Write_Command(0xD3);
    SH1106_Write_Command(0x00);
    SH1106_Write_Command(0xA1);
    SH1106_Write_Command(0xC8);
    SH1106_Write_Command(0xA6);
    SH1106_Write_Command(0xAF);
}

void SH1106_Set_Pos(uint8_t x, uint8_t y){
    SH1106_Write_Command(0xB0 + y);
    //設置列位址高4位
    //10H-17H    0 0 0 1 A7 A6 A5 A4
    SH1106_Write_Command((((x+2)&0xF0)>>4) | 0x10);
    //設置列位址低4位
    //00H-0FH    0 0 0 0 A3 A2 A1 A0
    SH1106_Write_Command((x+2)&0x0F);
}

void SH1106_Display(void){
    uint8_t page_index = 0, colume_index = 0;
    for(page_index=3;page_index<5;page_index++){
        SH1106_Write_Command(0xB0 + page_index);
        SH1106_Write_Command(0x10);
        SH1106_Write_Command(0x02);
        for(colume_index=0;colume_index<128;colume_index++){
            if(page_index == 3)
                SH1106_Write_Data(Frame[colume_index*2]);
            else
                SH1106_Write_Data(Frame[colume_index*2+1]);
        }
    }
}
```


### Output

Example:

This is a simple example for displaying the example font, initialize the library, initialize the display, clear the display and display the custom font.
```c
//****main.c****//
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/pin_manager.h"
#include "SH1106_Lib/myOLED_SH1106.h"

void main(void)
{
    //initialize the device
    SYSTEM_Initialize();

    __delay_ms(500);
    
    SH1106_init();
    SH1106_Clear_Screen(0x00);
    SH1106_Display(char);
}

```

The SH1106 I2C 1.3 Inch OLED can work successfully and can display graphics.

<img src="https://github.com/ElangovanChelliah/PIC16-1.3Inch-OLED-Interface/blob/aaa293a598f0cbdf646f21cc152dbaaec0948218/Output.jpg" width="1100">
