# PIC18-0.96Inch-TFT-LCD-Display-Interface

This project demonstrates a simple hardware design for 0.96 inch RGB TFT IPS Display and PIC18F26K40 Interface, the given circuit schematics can be integrated to TFT IPS (0.96') required projects like medical devices, gaming console, information display, remote controller, music player, interactive button, coffee machine, vending machine. The project aims to design, develop, and to test the TFT functionality with PIC18F26K40 combined with custom SPI circuit design feasible for TFT LCD-involved future projects.

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
    <td>PIC18F26K40</td>
  </tr>
  <tr>
    <td>TFT IPS</td>
    <td>0.96Inch</td>
  </tr>
  <tr>
    <td>Pixel</td>
    <td>80xRGBx160(TFT)</td>
  </tr>	
  <tr>
    <td>Driver</td>
    <td>ST7735S</td>
  </tr>
    <tr>
    <td>Interface</td>
    <td>SPI</td>
  </tr>
  <tr>
    <td>Oscillator</td>
    <td>INTSOC</td>
  </tr>
    <tr>
    <td>Internal Clock</td>
    <td>64MhZ</td>
  </tr>
 </table>

### Hardware Design

The designed schematics use a direct circuit connection as per the pin diagram with a voltage regulator for module testing, the integrated applications can use the same schematics connected to the common voltage source of 3.3V, the varied voltage ranging from 3V to 3.6V results in different peak brightness, a PWM signal also can be used to control the brightness, Altium designer is used to layout the circuit. The schematics can be transformed to desired layouts like module or custom hardware. 

Will be updated soon

<img src="https://github.com/ElangovanChelliah/PIC16-1.3Inch-OLED-Interface/blob/7bf8ae4249119983b670ed0e5f937535a9344fe7/Schematics.pn" width="600">

<img src="https://github.com/ElangovanChelliah/PIC16-1.3Inch-OLED-Interface/blob/fb46c0077c1c229d86bac8b4fa5d2c523bebb977/1.3%20Inch%20OLED.pn" width="300">

### Pin Diagram

Will be updated soon

<img src="https://github.com/ElangovanChelliah/PIC16-1.3Inch-OLED-Interface/blob/b93f52e42c382b0f1b97d97665717313e876241a/Pin%20Diagram.pn" width="400">

### Software Design
MPLAB X IDE is used to design and develop firmware and software, copy the project files [here](https://github.com/ElangovanChelliah/PIC16-1.3Inch-OLED-Interface/tree/3b928b85792c1b3a9fc59a058e2cbee1c41ed1a5/Firmware), and execute the same to get the desired output, from the example in this project a library is created and LOGO is used to demonstrate the functionality.

Example:
Will be updated soon

```c
void ST7735S_Init(void){
        
    //Set BackLight OFF
    //BackLight_SetLow(); - Uncomment if MCC Configuration: Start High
    //Set PWM to 0
    BackLight_SetHigh();
    
    //Haptic Initialize
    ELAN_SetLow();
    //LED Initialize
    LED2_SetHigh();
    
    //Set Low
    __delay_ms(300);
    ELAN_SetHigh();
    LED2_SetLow();      
    
	//RESET 
    SPI_RST = 1; 
    __delay_us(50);
	SPI_RST = 0;
    __delay_us(100);
	SPI_RST = 1;
    __delay_us(50);

	//TFT START WRITE (CS=0), Initialize Sequence Begin      
    StartWrite();
    
    //Sleep out 
    SPI_0096A_WrCmd(0x11);                            //Sleep out	

    //RESET CANCEL -> Normal Operation
    __delay_us(120);

	//ST7735S Frame Rate Setting in normal mode: FOSC/ (((RTNA*2)+40)*(LINE+FPA+BPA+2))=80  
 	SPI_0096A_WrCmd(0XB1); 							  // FOSC=850KHz
	SPI_0096A_WriteData(0X05);                        // RTNA=5
	SPI_0096A_WriteData(0X3C);  //20180612            // FPA=58
	SPI_0096A_WriteData(0X3C);  //20180612            // BPA=58

	//ST7735S Frame Rate Setting in idle mode: FOSC/ (((RTNB*2)+40)*(LINE+FPB+BPB+2))=80  
 	SPI_0096A_WrCmd(0XB2);                            // FOSC=850KHz
	SPI_0096A_WriteData(0X05);                        // RTNB=5
	SPI_0096A_WriteData(0X3C);   //20180612           // FPB=58
	SPI_0096A_WriteData(0X3C);   //20180612           // BPB=58

	//ST7735S Frame Rate Setting in PARITAL mode (dot inversion): FOSC/ (((RTNC*2)+40)*(LINE+FPC+BPC+2))=80
	//ST7735S Frame Rate Setting in PARITAL mode (column inversion): FOSC/ (((RTNC*2)+40)*(LINE+FPC+BPC+2))=80
	SPI_0096A_WrCmd(0XB3);                            // FOSC=850KHz
	SPI_0096A_WriteData(0X05);                        // RTNC=5
	SPI_0096A_WriteData(0X3C);   //20180612           // FPC=58
	SPI_0096A_WriteData(0X3C);   //20180612           // BPC=58
	SPI_0096A_WriteData(0X05);                        // RTND=5
	SPI_0096A_WriteData(0X3C);   //20180612           // FPD=58
	SPI_0096A_WriteData(0X3C);   //20180612           // BPD=58

	//ST7735S Display Inversion Control
	SPI_0096A_WrCmd(0XB4);                            // Dot inversion: 20184019 modify from SITRONIX initial code
	SPI_0096A_WriteData( 0X07);                       // 0xB4[2]=Inversion setting in normal mode
                                                      // 0xB4[1]=Inversion setting in idle mode
                                                      // 0xB4[0]=Inversion setting in partial mode
	//ST7735S Power on Sequence
	SPI_0096A_WrCmd(0XC0);                            // power control 1
	SPI_0096A_WriteData(0XE9);   //20180612           // {Par.3[0],Par.1[4:0]}=VRHP[5:0]=2¡÷GVDD=4.6, Par.1[7:5]=AVDD [2:0]=6¡÷AVDD=5.1
    SPI_0096A_WriteData(0X09);   //20180612           // {Par.3[1],Par.2[4:0]}=VRHN[5:0]=2¡÷GVCL=-4.6                                               
	SPI_0096A_WriteData(0X04);                        // Par.3[7:6]=MODE[1:0]=2X 
	
	SPI_0096A_WrCmd(0XC1);                            // power control 2
	SPI_0096A_WriteData(0XC5);   //20180612           // Par.1 [1:0]=VGHBT[1:0]=0¡÷VGH=2*AVDD+VGH25-0.5
	                                                  // Par.1 [3:2]=VGLSEL[1:0]=0¡÷VGL=-7.5
                                                      // Par.1 [7:6]=VGLSEL[1:0]=3¡÷VGH25=2.4	
    
    SPI_0096A_WrCmd(0XC2);                            // power control 3
	SPI_0096A_WriteData(0X0D);  //20180612            // (SITRONIX initial) {Par.1 [7:6],Par.2[7:0]=DCA[9:0]=000000000'b¡÷Booster set up cycle BCLK/1 BCLK/3 BCLK/1 BCLK/1 BCLK/1 in normal mode
	SPI_0096A_WriteData(0X00);                        // Par.1[5:3]=SAPA[2:0]=001'b¡÷ OP current is small in normal mode
                                                      // Par.1[2:0]=APA[2:0]=001'b¡÷ OP current is Large in normal mode	
	
    SPI_0096A_WrCmd(0XC3);                            // power control 4
	SPI_0096A_WriteData(0X8D); //20180612             // (SITRONIX initial){Par.1 [7:6],Par.2[7:0]=DCB[9:0]=1001101010'b¡÷Booster set up cycle BCLK/2 BCLK/1 BCLK/2 BCLK/2 BCLK/2 in idle mode
	SPI_0096A_WriteData(0X6A);                        // Par.1[5:3]=SAPB[2:0]=001'b¡÷ OP current is small in idle mode
                                                      // Par.1[2:0]=APB[2:0]=011'b¡÷ OP current is Medium in idle mode
	
    SPI_0096A_WrCmd(0XC4);							  // power control 5
	SPI_0096A_WriteData(0X8D); //20180612             // (SITRONIX initial){Par.1 [7:6],Par.2[7:0]=DCC[9:0]=1011101110'b¡÷Booster set up cycle BCLK/2 BCLK/2 BCLK/2 BCLK/4 BCLK/2 in partial mode
	SPI_0096A_WriteData(0XEE);                        // Par.1[5:3]=SAPC[2:0]=001'b¡÷ OP current is small in partial mode                                                
	
    // ST7735S VCOM   	
    SPI_0096A_WrCmd(0XC5);                             // VCOM setting value
   	SPI_0096A_WriteData(0X15); //20180612              // 0XC5[5:0]=010010'b ¡÷ VCOM=-0.875
	
    // ST7735 Memory data access control: add from SITRONIX initial code
    SPI_0096A_WrCmd(0X36);                             // VCOM setting value
   	SPI_0096A_WriteData(0XC8);    
	
    //ST7735 Display Inversion on
	SPI_0096A_WrCmd(0X21);
	
    // ST7735 Gamma Sequence 
	SPI_0096A_WrCmd(0XE0);                             // Gamma setting value (Positive Polarity)
	SPI_0096A_WriteData(0X07); 	//20180612			   // Par.1[5:0]=VRF0P[5:0]=000011'b (Variable Resistor VRHP)
	SPI_0096A_WriteData(0X0E);  //20180612        	   // Par.2[5:0]=VOS0P[5:0]=011011'b (Variable Resistor VRLP)
	SPI_0096A_WriteData(0X08); 	//20180612			   // Par.3[5:0]=PK0P[5:0]=010010'b  (Voltage of V3 gray scale)	
	SPI_0096A_WriteData(0X07);	//20180612             // Par.4[5:0]=PK1P[5:0]=010001'b  (Voltage of V4 gray scale)	
	SPI_0096A_WriteData(0X10);  //20180612             // Par.5[5:0]=PK2P[5:0]=111111'b  (Voltage of V12 gray scale)
	SPI_0096A_WriteData(0X07);	//20180612			   // Par.6[5:0]=PK3P[5:0]=111010'b  (Voltage of V20 gray scale)
	SPI_0096A_WriteData(0X02);  //20180612			   // Par.7[5:0]=PK4P[5:0]=111010'b  (Voltage of V28 gray scale)	
	SPI_0096A_WriteData(0X07);	//20180612			   // Par.8[5:0]=PK5P[5:0]=110100'b  (Voltage of V36 gray scale)
	SPI_0096A_WriteData(0X09); 	//20180612			   // Par.9[5:0]=PK6P[5:0]=101111'b  (Voltage of V44 gray scale)
	SPI_0096A_WriteData(0X0F);  //20180612             // Par.10[5:0]=PK7P[5:0]=101011'b  (Voltage of V52 gray scale)
	SPI_0096A_WriteData(0X25);  //20180612             // Par.11[5:0]=PK8P[5:0]=110000'b  (Voltage of V56 gray scale)
	SPI_0096A_WriteData(0X36);	//20180612			   // Par.12[5:0]=PK9P[5:0]=111010'b  (Voltage of V60 gray scale)
	SPI_0096A_WriteData(0X00); 						   // Par.13[5:0]=SELV0P[5:0]=000000'b  (Voltage of V0 gray scale)
	SPI_0096A_WriteData(0X08);	//20180612			   // Par.14[5:0]=SELV1P[5:0]=000001'b  (Voltage of V1 gray scale)
	SPI_0096A_WriteData(0X04);  //20180612             // Par.15[5:0]=SELV62P[5:0]=000010'b  (Voltage of V62 gray scale)
	SPI_0096A_WriteData(0X10);	//20180612			   // Par.16[5:0]=SELV63P[5:0]=001001'b  (Voltage of V63 gray scale)

	SPI_0096A_WrCmd(0XE1);                             // Gamma setting value (Negative Polarity)
	SPI_0096A_WriteData(0X0A); 	//20180612			   // Par.1[5:0]=VRF0N[5:0]=000011'b    (Variable Resistor VRHN)
	SPI_0096A_WriteData(0X0D);  //20180612 			   // Par.2[5:0]=VOS0N[5:0]=011011'b    (Variable Resistor VRLN)
	SPI_0096A_WriteData(0X08); 	//20180612			   // Par.3[5:0]=PK0N[5:0]=010010'b     (Voltage of V3 gray scale)	
	SPI_0096A_WriteData(0X07);	//20180612			   // Par.4[5:0]=PK1N[5:0]=010001'b     (Voltage of V4 gray scale)	
	SPI_0096A_WriteData(0X0F);  //20180612             // Par.5[5:0]=PK2N[5:0]=110010'b     (Voltage of V12 gray scale)
	SPI_0096A_WriteData(0X07);	//20180612			   // Par.6[5:0]=PK3N[5:0]=101111'b     (Voltage of V20 gray scale)
	SPI_0096A_WriteData(0X02);  //20180612			   // Par.7[5:0]=PK4N[5:0]=101010'b     (Voltage of V28 gray scale)
	SPI_0096A_WriteData(0X07);	//20180612			   // Par.8[5:0]=PK5N[5:0]=101111'b     (Voltage of V36 gray scale)
	SPI_0096A_WriteData(0X09); 	//20180612			   // Par.9[5:0]=PK6N[5:0]=101110'b     (Voltage of V44 gray scale)
	SPI_0096A_WriteData(0X0F);  //20180612             // Par.10[5:0]=PK7N[5:0]=101100'b    (Voltage of V52 gray scale)
	SPI_0096A_WriteData(0X25);  //20180612             // Par.11[5:0]=PK8N[5:0]=111001'b    (Voltage of V56 gray scale)
	SPI_0096A_WriteData(0X35);						   // Par.12[5:0]=PK9N[5:0]=111111'b    (Voltage of V60 gray scale)
	SPI_0096A_WriteData(0X00); 						   // Par.13[5:0]=SELV0N[5:0]=000000'b  (Voltage of V0 gray scale)
	SPI_0096A_WriteData(0X09);	//20180612			   // Par.14[5:0]=SELV1N[5:0]=000000'b  (Voltage of V1 gray scale)
	SPI_0096A_WriteData(0X04);  //20180612             // Par.15[5:0]=SELV62N[5:0]=000001'b (Voltage of V62 gray scale)
	SPI_0096A_WriteData(0X10);	//20180612			   // Par.16[5:0]=SELV63N[5:0]=001001'b (Voltage of V63 gray scale)

	SPI_0096A_WrCmd(0XFC);                             // Enable Gate power save mode
	SPI_0096A_WriteData(0XC0);                         // 0XFC[7:6]=GCV_Enable[1:0]=10'b¡÷ Gate Pump Clock Frequency disable
                                                       // 0XFC[3:2]=CLK_Variable[1:0]=11'b¡÷ Save Power Ability is Large 
    SPI_0096A_WrCmd(0X3A);
	SPI_0096A_WriteData(0X05);                         // 65K Mode
	
	SPI_0096A_WrCmd(0X2A);
	SPI_0096A_WriteData(0X00);                         // 65K Mode
	SPI_0096A_WriteData(0X1A);                         // 65K Mode
	SPI_0096A_WriteData(0X00);                         // 65K Mode
	SPI_0096A_WriteData(0X69);                         // 65K Mode

	SPI_0096A_WrCmd(0X2B);
	SPI_0096A_WriteData(0X00);                         // 6
	SPI_0096A_WriteData(0X01);                         // 
	SPI_0096A_WriteData(0X00);                         //
	SPI_0096A_WriteData(0XA0);                         // 

	SPI_0096A_WrCmd(0X29);                             // Display on
	SPI_0096A_WrCmd(0x2C); 
    
    //TFT END WRITE (CS=1), Initialize Sequence End  
    EndWrite();
    
    //Fill Screen Before Back Light Turn ON, (Back Light is turned ON after FULL_ON_SPI to avoid seeing glitches and fill write) 
    FillScreen(GREEN);                          //NOTE: Gimmick Fill 
    
    //Turn ON TFT LED
    //BackLight_SetHigh();
    BackLight_SetLow();

    //End of Initialize Sequence
}

```

### ST7735S Library for SPI
Will be updated soon

```c

void DrawPixel(uint8_t x, uint8_t y, unsigned int color) {
  if((x < _width) && (y < _height)) {
    StartWrite();
    SetAddrWindow(x, y, 1, 1);
    SPI_0096A_WriteData(color >> 8);
    SPI_0096A_WriteData(color & 0xFF);
    EndWrite();
  }
}

void FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, unsigned int color) {
  uint16_t px;
  if(w && h) {                            // Nonzero width and height?  
    uint8_t hi = color >> 8, lo = color;
    if((x >= _width) || (y >= _height))
      return;
    if((x + w - 1) >= _width)  
      w = _width  - x;
    if((y + h - 1) >= _height) 
      h = _height - y;
    StartWrite();
    SetAddrWindow(x, y, w, h);
    px = (uint16_t)w * h;
    while (px--) {
      SPI_0096A_WriteData(hi);
      SPI_0096A_WriteData(lo);
    }
    EndWrite();
  }
}

void FillScreen(unsigned int color) {
    FillRect(0, 0, _width, _height, color);
}

void DrawHLine(uint8_t x, uint8_t y, uint8_t w, unsigned int color) {
  if( (x < _width) && (y < _height) && w) {   
    uint8_t hi = color >> 8, lo = color;
    
    if((x + w - 1) >= _width)  
      w = _width  - x;
    StartWrite();
    SetAddrWindow(x, y, w, 1);
    while (w--) {
      SPI_0096A_WriteData(hi);
      SPI_0096A_WriteData(lo);
    }
    EndWrite();
  }
}

void DrawVLine(uint8_t x, uint8_t y, uint8_t h, unsigned int color) {
  if( (x < _width) && (y < _height) && h) {  
    uint8_t hi = color >> 8, lo = color;
    if((y + h - 1) >= _height) 
      h = _height - y;
    StartWrite();
    SetAddrWindow(x, y, 1, h);
    while (h--) {
      SPI_0096A_WriteData(hi);
      SPI_0096A_WriteData(lo);
    }
    EndWrite();
  }
}

void InvertDisplay(bool i) {
    StartWrite();
    SPI_0096A_WrCmd(i ? ST7735_INVON : ST7735_INVOFF);
    EndWrite();
}

void PushColor(uint16_t color) {
    uint8_t hi = color >> 8, lo = color;
    StartWrite();
    SPI_0096A_WriteData(hi);
    SPI_0096A_WriteData(lo);
    EndWrite();
}

//END___________________________________________________________________________

//Address & OffSet Configuration
void SetAddrWindow(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
  x += _xstart;
  y += _ystart;

  SPI_0096A_WrCmd(ST7735_CASET); // Column addr set
  SPI_0096A_WriteData(0);
  SPI_0096A_WriteData(x);
  SPI_0096A_WriteData(0);
  SPI_0096A_WriteData(x+w-1);

  SPI_0096A_WrCmd(ST7735_RASET); // Row addr set
  SPI_0096A_WriteData(0);
  SPI_0096A_WriteData(y);
  SPI_0096A_WriteData(0);
  SPI_0096A_WriteData(y+h-1);

  SPI_0096A_WrCmd(ST7735_RAMWR); // write to RAM
}
                                        
//SPI Write Command RS=0, CS=0
void SPI_0096A_WrCmd(unsigned char dat){
	SPI_RS = 0;
	Write_byte_0096A(dat);
}

//SPI Write Data RS=1, CS=1
void SPI_0096A_WriteData(unsigned char dat){
	SPI_RS = 1;    
	Write_byte_0096A(dat);
}

//SPI Write Byte / Function(HEX Code); 
void Write_byte_0096A(unsigned char dat){
    
	unsigned char i;
	for(i=0;i<8;i++){
		if((dat<<i)&ST_CMD_DELAY)
		{
			SPI_SDA=1;
		}
		else
		{
			SPI_SDA=0;
		}		
        __delay_us(1);
		SPI_SCK=1;
        __delay_us(1);
		SPI_SCK=0;
	}
}

//Start SPI
void StartWrite(void) {
    SPI_CS = 0;
}

//End SPI
void EndWrite(void) {
    SPI_CS = 1;
}

//Welcome Swipe
void Welcome_Swipe(void){
    
    unsigned int COLOR = BLACK;
    unsigned int CLEAR = GREEN;
    Bluetooth(COLOR);
    Battery(COLOR);
    
    Battery_L1(COLOR);
    Battery_L2(COLOR);
    Battery_L3(COLOR);
    
//    Eight(COLOR);
    Neutral(COLOR);
}

```


### Output

Example:

This is a simple example for displaying the example font, initialize the library, initialize the display, clear the display and display the custom font.
```c
//****main.c****//
Will be updated soon
}

```

<img src="https://github.com/ElangovanChelliah/PIC18-0.96Inch-TFT-LCD-Display-Interface/blob/0948c7bd6216744c7700121284c22d76c24141b0/Output_1.jpg" width="1100">
