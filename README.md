# Arduino_Drone
Files for Arduino nano drone hardware and software:

-Multiwii files are arduino libraries for drone/plane/hobby builds. This specific one is for a QUADX utilizing MPU6050 
 flight PID controller.
 
-Quadcop_onboard_revised is the altium CAD files for the onboard PCB, Gerber files included
    -utilizes 2 difference amplifiers to downregulate the battery voltage for NRf24 RX antenna and Arduino micro           
     controllers. New revs comming.
     
-Receiver is the arduino code for the receiver. Converts antenna data from SPI to pulse position modulated signal that 
 flight controller arduino interperets.

-Transmitter rev2 is the altium cad files for transmitter PCB (hand held remote with joysticks), Gerber files included
    -utilizes 2 double axis joystick potentiometers to send flight data through arduino to NRF24 TX antenna.

-Transmitter_state_maching is the arduino code for the transmitter. Impliments a simple state machine that uses buttons on  joysticks to fire interrupts and control states (start, hover, joystick control).
    -new rev comming to fix debounce issues.
    
please reach out if you're using my project as a basis for building your arduino drone. I'm happy to answer questions and hear helpful criticisms.

Big credit to ELECTRONOOBS' youtube page for providing foundational basis for my project.
