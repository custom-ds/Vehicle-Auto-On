# Vehicle Auto-On #

## Source ##

### Configuring the Dev Environment ###
Install Platform IO in Visual Studio Code

Use the jtag2updi



https://dev.to/dariocasciato/attiny85-programming-your-complete-guide-with-arduino-and-platformio-37do

https://www.electronics-lab.com/project/programming-attiny10-platform-io-ide/


### Programming ###

With the Arduion programmer board loaded with the jtag2updi software, and the Pin 6 from the Arduino connected to the Reset
line on the ATTiny404, you can Build and Program directly inside of the Visual Studio Code.

To build, go to the bottom-left part of the screen to the Status Bar area of VSCode, and there should be some little icons
that Platform IO installed. The Checkmark icon is the Build option, and will compile/build the code that is written.

Once it compiles successfully, there is a second icon that is a right-arrow to click, which will utilize the Arduino
programmer to program the ATTiny via UPDI.