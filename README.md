## General
This project is responsible for handling RFID cards and wireless authorization for Nucleo STM32 F303RE microcontroller. Getting RFID card or pressing remote button results in sending 5V signal to the relay and opening doors.
The microcontroller is connected with RC522 13.56 Mhz RFID card listener, nRF24L01 2.4GHz wireless communication antenna, speaker and 5V relay.

### RFID
RFID IDs are programmable by sending command to microcontroller through USART communication port (I've created my own communication protocol), or by pressing button in the right combination - then, the RFID programming mode turns on for 30 seconds, and when antenna reads RFID ID, it saves it to the flash memory. All IDs are stored in flash memory and stay here even after power outage.
When RFID 13.56 MHz card is detected near antenna, antenna reads card's ID and checks if this ID is already saved in flash. If it's saved, then 5V signal is sent for the relay which can for e.g. open the electric strike of doors. If program doesn't find the ID in flash, it indicates it playing suitable sound on the speaker and doesn't send the signal for the relay.

### NRF24L01 Wireless module
Nucleo microcontroller's nrf24l01 module is working most time as a receiver. It detects signal on 2.4GHz band, and if that signal is a combination of characters that is programmed in code, then the signal is sent to the relay. Once in a while it changes state from receiver to transmitter, and it transmit signal for remotes to reset LED states according to relay state (if it's open or not). After that it turns to the receiver again.
Remotes are created separately with help of Arduino. These are sending signals for the receiver when button is pressed, and also have leds which indicate the nucleo relay's state.

### Communication protocol
Program has implemented my own communication protocol, which allows to send commands through USART communication port and do couple of things. To send a command, you have to code it by putting '<' char at the start of command and '>' at the end. Command formula is <TRM_RCV_COMMAND>, where TRM in transmitter name (you can put your own name, it's important to put there 3 letters), RCV is receiver and it has to be NCL, and Command is command you want to get done. Commands are:
* OPEN - sending signal for a relay,
* OPENPERM - sending permanent signal for relay, until powers gets off or nucleo gets command for close,
* CLOSE - stopping signal,
* FLASHSAVE - turns on RFID flash save mode,
* FLASHCLEAR - clears flash memory,
* CHANGETIMEOPEN:XXX - changes time for which signal is sent after OPEN command or card detection,
* SOUND - turns sound on/off.

## Technologies
* Language: C
* Microcontroller: Nucleo STM32 F303RE
* IDE: STM32CubeIDE 1.4.0
* Connected modules / parts: RC522 RFID, NRF24L01 Wireless module, Grove speaker, 5V relay.
