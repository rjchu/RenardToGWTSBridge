// Renard to IR/Mouse Ear Transmission Tool for Arduino
// (C) 2013 Joni Chu with code contributions from Jonathan Fether and DIYC user Mat er daddy.
// This product is not endorsed, authorized, or prepared by the manufacturer of the ears.
// v0.2 NO WARRANTY EXPRESSED OR IMPLIED
// 
/*

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 The license document is available at: http://www.gnu.org/licenses/gpl-2.0.html
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 
 */
// 
// Preface/Thanks:
// This code is based in large part (read - lots of copying and pasting) on code from doityourselfchristmas.com users 
// Jon Fether and "Mat er daddy" who I thank for their contributions and posts on this topic as well as Timon and 
// everyone else that's contributed to the "Glow with *OUR* Show" effort. Thanks also go out to Dirk for 
// his help with using XBee's for transmitting and receiving data to the Arduino and other hardware assistance.
// 
// Documentation (sort of):
// I. Assumptions:
//    1. The input serial data stream is Renard (via Vixen or some other equivalent) and that individual RGB values 
//      are sent in on channels 1-3 for the left ear and channels 4-6 for the right ear.
//    2. The input serial data comes in on Serial1, 8N1 @ 2,400 baud. Currently I'm testing serial receive code below  
//      with a XBee radio as the receiver with another paired XBee radio in an Explorer board as the transmitter attached 
//      to the Vixen computer. Wiring is such that (for pictures see posting on DIYC):
//        XBee pin 1 -> Arduino 3.3v
//        XBee pin 2 -> Arduino Serial1 RX
//        XBee pin 3 -> Arduino Serial2 TX (not used)
//        XBee pin 10 -> Arduino GND
//    3. IR LED emitter and cooresponding valued resistor attached to pin 40 and GND. If you want to use a different pin, 
//      adjust the value of IRLedPin accordingly. 
// II. How's it work?
//    The incoming Renard serial stream is decoded into RGB values ranging from 0-255. These are then hashed into buckets 
//    numbered 0-4 which are used as indices into a 3D array of GwtS codes in the 1-bit color pallet published on DIYC. 
//    This is NOT a granular color pallet but is intended to be a v1.0-ish solution so please don't flame me with how you would 
//    have done it better, instead, please feel free to implement your solution and share it. I'd love to use it! :) The 
//    mapped color values are then sent to the IR transmission logic prefaced with 0x91 and 0x0E for both ears or 0x94 and 0x0E
//    for colors intended for the right ear which indicate the command and pallet and is then appended with the checksum byte as 
//    calculated based on the 3-5 command bytes being sent.
//
// To Do List:
// - Write some docs and/or add better code comments
// - Clean things up a bit more and remove copy/paste artifacts
// - Add Renard start addressing code so the transmitter channels can be inserted anywhere in the Renard controller chain
// - Add a 4th channel for each of the two ears that, based on intensity level, adjust command sequence to include fade-in, 
//   fade-out, etc vs the instance on/off that's currently (only) supported
// - Anything else? Let me know!


// ****** Configurable items below this point ******
#define IRledPin 40
bool debug = true;
// ****** Nothing beyond this point is configurable unless you know what you're doing ******

bool sync;
int i = 0;
uint8_t lRedLast   = 0;
uint8_t lGreenLast = 0;
uint8_t lBlueLast  = 0;
uint8_t rRedLast   = 0;
uint8_t rGreenLast = 0;
uint8_t rBlueLast  = 0;
byte codeLookup[5][5][5] = {
  {
   {0x1D, 0x1D, 0x04, 0x04, 0x04},
   {0x04, 0x04, 0x02, 0x02, 0x02},
   {0x02, 0x02, 0x01, 0x01, 0x01},
   {0x01, 0x01, 0x1A, 0x1A, 0x1A},
   {0x1A, 0x17, 0x17, 0x17, 0x16}
  },
  {
   {0x16, 0x16, 0x16, 0x16, 0x07}, 
   {0x07, 0x07, 0x07, 0x07, 0x07},
   {0x07, 0x07, 0x07, 0x07, 0x07},
   {0x07, 0x07, 0x07, 0x08, 0x08},
   {0x08, 0x08, 0x08, 0x08, 0x08}
  },
  { 
   {0x08, 0x08, 0x08, 0x08, 0x08},
   {0x08, 0x08, 0x08, 0x08, 0x08},
   {0x08, 0x08, 0x08, 0x08, 0x08},
   {0x08, 0x08, 0x08, 0x08, 0x06},
   {0x06, 0x06, 0x06, 0x06, 0x06}
  },
  {
   {0x06, 0x06, 0x06, 0x06, 0x06},
   {0x06, 0x06, 0x06, 0x06, 0x06},
   {0x06, 0x06, 0x06, 0x06, 0x06},
   {0x06, 0x06, 0x00, 0x00, 0x00},
   {0x00, 0x00, 0x00, 0x00, 0x00}
  },
  {
   {0x15, 0x0D, 0x0D, 0x0C, 0x0A},
   {0x10, 0x10, 0x10, 0x10, 0x10},
   {0x11, 0x11, 0x11, 0x11, 0x11},
   {0x0F, 0x0F, 0x05, 0x05, 0x05},
   {0x12, 0x12, 0x1B, 0x1B, 0x1C}
  }
}; 
 
void setup() {
  delay(10);
  Serial1.begin(2400);
  delay(10);
  Serial.begin(19200);
  pinMode(IRledPin, OUTPUT); 
  pinMode(13, OUTPUT); // Clock Timing Monitor Pin
  sync = false;
}
 
void wait_for_serial() {
    while ( ! Serial1.available() > 0 ) { }
}

unsigned char calc_crc(unsigned char *data, unsigned char length) {
  // Courtesy of rjchu and Timon - A godsend
  unsigned char crc = 0;
  while(length--) {
    crc ^= *data++;
    unsigned n = 8; 
    do crc = (crc & 1) ? (crc >> 1) ^ 0x8C : crc >> 1; 
    while(--n);
  }
  return crc;
} 
 
void pulseIR(long microsecs, int hilo) {
  // we'll count down from the number of microseconds we are told to wait
  while (microsecs > 0) {
    // 38 kHz is about 13 microseconds high and 13 microseconds low
    digitalWrite(IRledPin, hilo);  // this takes about 5 microseconds to happen
    delayMicroseconds(8);         // hang out for 8 microseconds
    digitalWrite(IRledPin, LOW);   // this also takes about 5 microseconds
    delayMicroseconds(8);         // hang out for 8 microseconds
    // so 26 microseconds altogether
    microsecs -= 26;
  }
}

void sendbyte(byte b) {
  // Send 8-N-1 data as the mouse ears communicate in.
  // Data consists of 1 Start Bit, 8 Data Bits, 1 Stop Bit, and NO parity bit
  // 1 bit is 417 microseconds @ 2400 baud
  pulseIR(400, HIGH); // Start bit
  byte i=0;
  while(i<8) {
    pulseIR(400, (b>>(i++)&1)?LOW:HIGH); // Data Bits
  }
  pulseIR(400, LOW); // Stop bit
}

byte bytefromhex(char hexed[2]) {
  // This can be massaged if desired to allow better work. I'm just going to use a cheap ASCII offset.
  // This isn't "safe" or "pretty" programming but works for this purpose.
  if(hexed[1]>'9') hexed[1] -= 7; // (7 is the offset from 'A' to ':' which follows '9')
  if(hexed[0]>'9') hexed[0] -= 7;
  return (byte)(((hexed[0] - '0')<<4) + hexed[1] - '0');
}

int renardReadBytes( uint8_t *bytes, uint8_t bytes_size ) {
  int in_byte = 0;
  int bytes_read;
 
  for ( bytes_read = 0; bytes_read < bytes_size; ) {
    wait_for_serial();
    in_byte = Serial1.read();
    switch (in_byte) {
      case(0x7E): // We saw the sync byte, start over!
        sync = true;
        return bytes_read;
 
      case(0x7D): // Skip the pad byte
        continue;
 
      case(0x7F): // Escape character, we need to read one more byte to get our actual data
        wait_for_serial();
        in_byte = Serial1.read();
        switch (in_byte) {
            case(0x2F): // renard wants an 0x7D
              in_byte = 0x7D;
            case(0x30): // renard wants an 0x7E
              in_byte = 0x7E;
            case(0x31): // renard wants an 0x7F
              in_byte = 0x7F;
        }
      }
      bytes[bytes_read++] = in_byte;
  }
  return bytes_read;
}
 
int renardRead( uint8_t *bytes, uint8_t byte_count ) {
  int in_byte = 0;
 
  while ( ! sync ) {
    wait_for_serial();
    in_byte = Serial1.read();
    if ( in_byte == 0x7E ) // Sync byte signifies start of packet
      sync = true;
  }
 
  if ( sync ) {
    sync = false;
    wait_for_serial();
    in_byte = Serial1.read();
    if ( in_byte == 0x80 ) { // Read from here 
      return renardReadBytes(bytes, byte_count);
    }
  }
  return 0;
}
 
void loop() {
  byte cmdbuf[32];
  int cmdcount = 0;
  unsigned char checksum = 0;
  uint8_t bytes[6], bytes_read;
  bytes_read = renardRead(&bytes[0], 6);
  if ( bytes_read == 6 ) {
    uint8_t lRed   = floor(bytes[0]/51);
    uint8_t lGreen = floor(bytes[1]/51);
    uint8_t lBlue  = floor(bytes[2]/51);
    uint8_t rRed   = floor(bytes[3]/51);
    uint8_t rGreen = floor(bytes[4]/51);
    uint8_t rBlue  = floor(bytes[5]/51);
    if (lRed > 4) lRed = 4;
    if (lGreen > 4) lGreen = 4;
    if (lBlue > 4) lBlue = 4;
    if (rRed > 4) rRed = 4;
    if (rGreen > 4) rGreen = 4;
    if (rBlue > 4) rBlue = 4;
    if (debug) {
      Serial.print("Left RGB Values are ");
      Serial.print(lRed);
      Serial.print(" ");
      Serial.print(lGreen);
      Serial.print(" ");
      Serial.println(lBlue);
      Serial.print("Right RGB Values are ");
      Serial.print(rRed);
      Serial.print(" ");
      Serial.print(rGreen);
      Serial.print(" ");
      Serial.println(rBlue);
    }
    char tempArray[2];
    if ((lRed == rRed) && (lGreen == rGreen) && (lBlue == rBlue)) {
      tempArray[0] = '9';
      tempArray[1] = '1';
      cmdbuf[0] = bytefromhex(tempArray); //0x91;
      tempArray[0] = '0';
      tempArray[1] = 'E';
      cmdbuf[1] = bytefromhex(tempArray); //0x0E;
      cmdbuf[2] = codeLookup[lRed][lGreen][lBlue]; // Color code for both ears
      cmdcount = 3;
      checksum = calc_crc(cmdbuf, cmdcount);
    }
    else {
      if (debug) {
        Serial.println("*** ABOVE IS DIFFERENT ***");
      }
      tempArray[0] = '9';
      tempArray[1] = '4';
      cmdbuf[0] = bytefromhex(tempArray); //0x91;
      tempArray[0] = '0';
      tempArray[1] = 'E';
      cmdbuf[1] = bytefromhex(tempArray); //0x0E;
      cmdbuf[2] = codeLookup[lRed][lGreen][lBlue]; // Color code for the left ear
      tempArray[0] = '0';
      tempArray[1] = 'E';
      cmdbuf[3] = bytefromhex(tempArray); //0x0E;
      cmdbuf[4] = codeLookup[rRed][rGreen][rBlue]; // Add 0x80 to color code for the right ear
      tempArray[0] = '8';
      tempArray[1] = '0';
      cmdbuf[4] = cmdbuf[4] + bytefromhex(tempArray);
      cmdcount = 6;
      checksum = calc_crc(cmdbuf, cmdcount);
    }
    i=0;
    while (i < cmdcount) {
      sendbyte(cmdbuf[i++]);
    }
    sendbyte(checksum);
  }
}



