## ESP32 I2S Simulator

This Simulator demonstrates the function of the I2S modul in 8-Bit Input capture mode, as used in the camera application.  

It can help to understand the details of the functioning, step by step  (or clock by clock ;)  

There must be no camera connected to the ESP32!  
All "input signals" are simulated by writing values to the PortPins output register, which in turn are used as input signals.  

Its a basic terminal application.  


**Menue**   
l-list state   	= list current state info  
c-clock1		= do exactly 1 clock (pclk input), this fetches one Byte from the 8Bit input Bus.  
s-set SampleMode = this resets the state and switches to a selected SampleMode. (there are 4 samplemodes in the I2S-module)  
i-set inport Data = preset the 8Bit input Bus to desired Data. After each clock this data is incremented by one.  
                   (NOTE: some pins esp32 are input-only and cannot be used to set output data. They read as 0 always)  

I2Ssim>  


**State List**   
+++ClockCnt:8  IntCnt:1  DataPort:0x09 SampleMode:1  
DMAsize:16 inlen:16 eof:1 owner:0  -  Dmabuffer-BytesPerSample:2  
DMA-Buffer[0-15]:  
0x02 0x00 0x01 0x00 0x04 0x00 0x03 0x00 0x06 0x00 0x05 0x00 0x08 0x00 0x07 0x00   


- It shows the amount of clocks done, amount of interrupts happened (one after 16Bytes are filled), current input PortData, current samplemode.  
- The configured DMA descriptor as 16 Bytes. (amount of received bytes, current flags: owner, eof - are updated when interrupt happens)  
- and the 16 Bytes in the DMA buffer, which shows where the fetched databytes are positioned.  

The interrupt (in_done) happens after the configured 16 Bytes are filled.  Its only 16Bytes because its a demo for learning!  

Its good for examining the behavior of data-capture, DMA transfer and interrupt in different SampleModes, step by step.  
