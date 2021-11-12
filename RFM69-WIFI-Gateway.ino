#include <Adafruit_SleepyDog.h>
//#include <MemoryFree.h>
#include <SPI.h>

#include <WiFiNINA.h>
//#include <WiFi101.h>
#include <WiFiUDP.h>
#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;	 // your network SSID (name)
char pass[] = SECRET_PASS;	 // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS; // the WiFi radio's status

long prevWifiCheckMillis = 0;
long wifiInterval = 300000; // Check the wifi connection every minutes

// Radio Includes and variables:

//#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define RF69_FREQ 915.0

#define MY_ADDRESS 1

// Feather M0 w/Radio
#define RFM69_CS 8
#define RFM69_IRQ 3
#define RFM69_RST 4

// Feather M0 w/Radio Featherwing
//#define RFM69_CS      10   // "B"
//#define RFM69_RST     11   // "A"
//#define RFM69_IRQ     6    // "D"
//#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )

//#define RELAY 		  12

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_IRQ);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0; // packet counter, we increment per xmission

uint32_t localCounter = 1;

struct EvenDataPacket
{
	uint32_t counter;
	float batteryVoltage;
	uint8_t cubeID;
	uint8_t side;
} eventData;

// Dont put this on the stack:
uint8_t eventBuffer[sizeof(eventData)];
uint8_t from;
uint8_t len = sizeof(eventData);

// -------------- UDP Ethernet Variables ----------------------

#define SPIWIFI SPI		// The SPI port
#define SPIWIFI_SS 13	// Chip select pin
#define ESP32_RESETN 12 // Reset pin
#define SPIWIFI_ACK 11	// a.k.a BUSY or READY pin
#define ESP32_GPIO0 10

///#define WIFI_SHIELD_CS 8

//byte mac[] = {
//  0x98, 0x76, 0xB6, 0x10, 0xb6, 0x53
//};

IPAddress ip(10, 10, 212, 101); // will change when moved to new VLAN
//IPAddress ip(192, 168, 0, 101);		// will change when moved to new VLAN
unsigned int localPort = 8888; // local port to listen on

byte gateway[] = {10, 10, 212, 5}; // was .5 or .15
byte subnet[] = {255, 255, 255, 0};
byte dnsServer[] = {10, 10, 100, 20}; // byte dnsServer[] = {10, 10, 100, 20};
IPAddress splunkIp(10, 10, 100, 150);
unsigned int splunkPort = 4514;
String pingHost = "10.10.212.129";
int pingResult;

// buffers for receiving and sending data
char packetBuffer[255];							//buffer to hold incoming packet,
const char AckBuffer[] = "acknowledged";		// a string to send back
const char DenyBuffer[] = "Error, no release."; // a string to send back
const char pigDrop[] = "pigDrop";				// command string to compare rx to
const char cEquals[] = "c=";
const char tEquals[] = " t=";
const char sEquals[] = " s=";
const char gEquals[] = " g=";
const char bEquals[] = " b=";
const char fEquals[] = " f=";

char ReplyBuffer[] = "acknowledged"; // a string to send back

WiFiUDP Udp;

void selectRadio()
{
	//  digitalWrite(SPIWIFI_SS, HIGH);
	//delay(100);
	digitalWrite(RFM69_CS, LOW);
	delay(100);
}

void selectWiFi()
{
	digitalWrite(RFM69_CS, HIGH);
	//delay(100);
	//  digitalWrite(SPIWIFI_SS, LOW);
	delay(100);
}

void sendDispenseEvent()
{
	//eventData.side = 1;
	eventData.counter = localCounter;
	localCounter++;
	eventData.cubeID = 0;
	eventData.side = 61;
	Serial.print(F("About to send transmission number: "));
	Serial.println(eventData.counter);
	sendEventData();
	//eventData.side = 0;
}

//RF communication
void sendEventData()
{
	rf69.send((uint8_t *)&eventData, sizeof(eventData));
	rf69.waitPacketSent();
}

void setupRadio()
{
	//--Radio Setup--//
	selectRadio();
	pinMode(RFM69_RST, OUTPUT);
	digitalWrite(RFM69_RST, LOW);

	Serial.println(F("Feather Addressed RFM69 TX Test!"));
	Serial.println();

	// manual reset
	digitalWrite(RFM69_RST, HIGH);
	delay(50);
	digitalWrite(RFM69_RST, LOW);
	delay(50);

	if (!rf69_manager.init())
	{
		Serial.println(F("RFM69 radio init failed"));
		while (1)
			;
	}
	Serial.println(F("RFM69 radio init OK!"));
	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
	// No encryption
	if (!rf69.setFrequency(RF69_FREQ))
	{
		Serial.println(F("setFrequency failed"));
	}

	// If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
	// ishighpowermodule flag set like this:
	rf69.setTxPower(20, true); // range from 14-20 for power, 2nd arg must be true for 69HCW

	// The encryption key has to be the same as the one in the server
	uint8_t key[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
					 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
	rf69.setEncryptionKey(key);

	eventData.cubeID = 0; // 1.Music, 2.Animals, 3.Weather, 50.Penny, 51.Penny, 60.Loan Desk
	eventData.side = 0;
	eventData.batteryVoltage = 0;
	eventData.counter = 0;
}

void setupWiFi()
{
	//Configure pins for Adafruit ATWINC1500 Feather

	// Set up the pins!
	WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);

	// check for the WiFi module:
	while (WiFi.status() == WL_NO_MODULE)
	{
		Serial.println("Communication with WiFi module failed!");
		// don't continue
		delay(1000);
	}
	String fv = WiFi.firmwareVersion();
	Serial.println(fv);
	if (fv < "1.0.0")
	{
		Serial.println("Please upgrade the firmware");
		while (1)
			delay(10);
	}
	Serial.println("Firmware OK");

	// print your MAC address:
	byte mac[6];
	WiFi.macAddress(mac);
	Serial.print("MAC: ");
	printMacAddress(mac);

	// attempt to connect to WiFi network:
	while (status != WL_CONNECTED)
	{
		Serial.print(F("Attempting to connect to WPA SSID: "));
		Serial.println(ssid);

		WiFi.config(ip, dnsServer, gateway, subnet);
		// Connect to WPA/WPA2 network:
		status = WiFi.begin(ssid, pass);

		// wait 10 seconds for connection:
		for (int i = 0; i > 10; i++)
		{
			delay(1000);
			Watchdog.reset();
		}
	}

	// you're connected now, so print out the data:
	Serial.print(F("You're connected to the network"));
	printCurrentNet();
	printWiFiData();

	Udp.begin(localPort);
	Udp.beginPacket(splunkIp, splunkPort);
	Udp.write("c=0 reconnect=1");
	Udp.endPacket();
}

void setup()
{
	Serial.begin(9600);
	delay(5000);
	// while (!Serial);

	//pinMode(WIFI_SHIELD_CS, OUTPUT);

	setupRadio();
	pinMode(ESP32_RESETN, OUTPUT);
	digitalWrite(ESP32_RESETN, LOW);
	delay(500);
	digitalWrite(ESP32_RESETN, HIGH);

	setupWiFi();

	Serial.print(F("Splunk IP: "));
	Serial.println(splunkIp);
	Serial.print(F("Splunk Port: "));
	Serial.println(splunkPort);

	//	Serial.print(F(" FreeRAM= ")); //F function does the same and is now a built in library, in IDE > 1.0.0
	//	Serial.println(freeMemory(), DEC);  // print how much RAM is available.

	//delay(10000);

	//	selectWiFi();

	Serial.print(F("Pinging "));
	Serial.print(pingHost);
	Serial.print(F(": "));

	pingResult = WiFi.ping(pingHost);

	if (pingResult >= 0)
	{
		Serial.print(F("SUCCESS! RTT = "));
		Serial.print(pingResult);
		Serial.println(F(" ms"));
	}
	else
	{
		Serial.print(F("FAILED! Error code: "));
		Serial.println(pingResult);
	}

	Serial.println(F("Sending reboot=1"));
	Udp.beginPacket(splunkIp, splunkPort);
	Udp.write("c=0 reboot=1");
	Udp.endPacket();

	// First a normal example of using the watchdog timer.
	// Enable the watchdog by calling Watchdog.enable() as below.
	// This will turn on the watchdog timer with a ~4 second timeout
	// before reseting the Arduino. The estimated actual milliseconds
	// before reset (in milliseconds) is returned.
	// Make sure to reset the watchdog before the countdown expires or
	// the Arduino will reset!
	int countdownMS = Watchdog.enable(4000);
	Serial.print(F("Enabled the watchdog with max countdown of "));
	Serial.print(countdownMS, DEC);
	Serial.println(F(" milliseconds!"));
	Serial.println();

} // END SETUP //

void loop()
{
	//selectWiFi();
	unsigned long currentMillis = millis();
	if (currentMillis - prevWifiCheckMillis >= wifiInterval)
	{
		prevWifiCheckMillis = currentMillis;
		selectWiFi();
		//printCurrentNet();

		Serial.print(F("Pinging "));
		Serial.print(pingHost);
		Serial.print(F(": "));

		pingResult = WiFi.ping(pingHost, 128);

		if (pingResult >= 0)
		{
			Serial.print(F("SUCCESS! RTT = "));
			Serial.print(pingResult);
			Serial.println(F(" ms"));
		}
		else
		{
			Serial.print(F("FAILED! Error code: "));
			Serial.println(pingResult);
			Serial.println(F("Resetting WiFi..."));
			pinMode(ESP32_RESETN, OUTPUT);
			digitalWrite(ESP32_RESETN, LOW);
			delay(500);
			digitalWrite(ESP32_RESETN, HIGH);
			setupWiFi();
		}
	}

	receiveFromCube();

	readUDP();

	delay(200);
	Watchdog.reset();
	//  Serial.println(F("loop"));
} // END LOOP //

void readUDP()
{
	selectWiFi();
	// if there's data available, read a packet
	int packetSize = Udp.parsePacket();
	if (packetSize)
	{
		/*		Serial.print(F("Received packet of size "));
		Serial.println(packetSize);
		Serial.print(F("From "));
		IPAddress remote = Udp.remoteIP();
		for (int i = 0; i < 4; i++)
		{
			Serial.print(remote[i], DEC);
			if (i < 3)
			{
				Serial.print(F("."));
			}
		}
		Serial.print(F(", port "));
		Serial.println(Udp.remotePort());
*/
		// read the packet into packetBufffer
		Udp.read(packetBuffer, 255);
		Serial.println(F("Contents:"));
		Serial.println(packetBuffer);

		if (memcmp_P(packetBuffer, pigDrop, 7) == 0)
		{
			Serial.println(F("Pig Drop Command Recieved - Release the Pig Eggs!"));
			selectRadio();
			sendDispenseEvent();

			selectWiFi();
			// send a reply, to the IP address and port that sent us the packet we received
			Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
			Udp.write(AckBuffer, sizeof(AckBuffer));
			Udp.endPacket();

			// send a "pigDrop" message to Splunk
			if (Udp.beginPacket(splunkIp, splunkPort) == 1)
			{
				Serial.println(F("Packet begun"));
			}
			else
			{
				Serial.println(F("ERROR BEGINNING PACKET!"));
			}
			Udp.write(pigDrop, sizeof(pigDrop));

			//Udp.write("A", 1);

			if (Udp.endPacket() == 1)
			{
				Serial.println(F("Packet ended/sent successfully"));
			}
			else
			{
				Serial.println(F("ERROR ENDING/SENDING PACKET!"));
			}
		}

		// Added by Jeff to clear out buffer.
		for (int ii = 0; ii < 255; ii++) // changed max from 24 to 64 5/9/19 Jeff
		{
			packetBuffer[ii] = 0;
		}
	}
}

/*void readUDP()
{
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
  }
}*/

void receiveFromCube()
{
	selectRadio();
	if (rf69.recv((uint8_t *)&eventData, &len) && len == sizeof(eventData))
	{
		char buf[16];
		//String stringBuf = "-00.000";

		Serial.print(F("Received: "));
		Serial.print(F("c="));
		Serial.print(itoa(eventData.cubeID, buf, 10));
		Serial.print(F(" t="));
		Serial.print(itoa(eventData.counter, buf, 10));
		Serial.print(F(" s="));
		Serial.print(itoa(eventData.side, buf, 10));
		Serial.print(F(" g="));
		Serial.print(rf69.lastRssi(), DEC);
		Serial.print(F(" b="));
		Serial.print(eventData.batteryVoltage);

		//		Serial.print(F(" FreeRAM=")); //F function does the same and is now a built in library, in IDE > 1.0.0
		//		Serial.print(freeMemory(), DEC);  // print how much RAM is available.

		if ((eventData.cubeID == 80) || (eventData.cubeID == 42) || (eventData.cubeID == 85) || (eventData.cubeID == 86) || (eventData.cubeID == 90)) // Only Maze and Turntable forwarded to Splunk
		{
			selectWiFi();
			Udp.beginPacket(splunkIp, splunkPort);
			Udp.write(cEquals);
			Udp.write(itoa(eventData.cubeID, buf, 10));
			Udp.write(tEquals);
			Udp.write(itoa(eventData.counter, buf, 10));
			if ((eventData.cubeID < 49) || (eventData.cubeID > 59)) // If not coin
			{
				Udp.write(sEquals);
				Udp.write(itoa(eventData.side, buf, 10));
			}
			Udp.write(gEquals);
			Udp.write(itoa(rf69.lastRssi(), buf, 10));
			if (eventData.batteryVoltage > 0) // If battery powered
			{
				Udp.write(bEquals);
				ftoa(eventData.batteryVoltage, buf, 3);
				Udp.write(buf);
			}
			//			Udp.write(fEquals);
			//			Udp.write(itoa(freeMemory(), buf, 10));
			if (Udp.endPacket() == 1)
			{
				Serial.print(F(" Packet sent"));
			}
			else
			{
				Serial.print(F(" Error sending UDP"));
			}
		}
		Serial.println(F(""));
	}
}

// ftoa from http://www.ars-informatica.ca/eclectic/ftoa-convert-a-floating-point-number-to-a-character-array-on-the-arduino/
void ftoa(float f, char *str, uint8_t precision)
{
	uint8_t i, j, divisor = 1;
	int8_t log_f;
	int32_t int_digits = (int)f; //store the integer digits
	float decimals;
	char s1[12];

	memset(str, 0, sizeof(str));
	memset(s1, 0, 10);

	if (f < 0)
	{				  //if a negative number
		str[0] = '-'; //start the char array with '-'
		f = abs(f);	  //store its positive absolute value
	}
	log_f = ceil(log10(f)); //get number of digits before the decimal
	if (log_f > 0)
	{ //log value > 0 indicates a number > 1
		if (log_f == precision)
		{					 //if number of digits = significant figures
			f += 0.5;		 //add 0.5 to round up decimals >= 0.5
			itoa(f, s1, 10); //itoa converts the number to a char array
			strcat(str, s1); //add to the number string
		}
		else if ((log_f - precision) > 0)
		{						   //if more integer digits than significant digits
			i = log_f - precision; //count digits to discard
			divisor = 10;
			for (j = 0; j < i; j++)
				divisor *= 10; //divisor isolates our desired integer digits
			f /= divisor;	   //divide
			f += 0.5;		   //round when converting to int
			int_digits = (int)f;
			int_digits *= divisor; //and multiply back to the adjusted value
			itoa(int_digits, s1, 10);
			strcat(str, s1);
		}
		else
		{							  //if more precision specified than integer digits,
			itoa(int_digits, s1, 10); //convert
			strcat(str, s1);		  //and append
		}
	}

	else
	{ //decimal fractions between 0 and 1: leading 0
		s1[0] = '0';
		strcat(str, s1);
	}

	if (log_f < precision)
	{						   //if precision exceeds number of integer digits,
		decimals = f - (int)f; //get decimal value as float
		strcat(str, ".");	   //append decimal point to char array

		i = precision - log_f; //number of decimals to read
		for (j = 0; j < i; j++)
		{					//for each,
			decimals *= 10; //multiply decimals by 10
			if (j == (i - 1))
				decimals += 0.5;		 //and if it's the last, add 0.5 to round it
			itoa((int)decimals, s1, 10); //convert as integer to character array
			strcat(str, s1);			 //append to string
			decimals -= (int)decimals;	 //and remove, moving to the next
		}
	}
}

void printWiFiData()
{
	// print your WiFi shield's IP address:
	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: ");
	Serial.println(ip);
	Serial.println(ip);

	// print your MAC address:
	byte mac[6];
	WiFi.macAddress(mac);
	Serial.print("MAC address: ");
	printMacAddress(mac);
}

void printCurrentNet()
{
	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print the MAC address of the router you're attached to:
	byte bssid[6];
	WiFi.BSSID(bssid);
	Serial.print("BSSID: ");
	printMacAddress(bssid);

	// print the received signal strength:
	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):");
	Serial.println(rssi);

	// print the encryption type:
	byte encryption = WiFi.encryptionType();
	Serial.print("Encryption Type:");
	Serial.println(encryption, HEX);
	Serial.println();
}

void printMacAddress(byte mac[])
{
	for (int i = 5; i >= 0; i--)
	{
		if (mac[i] < 16)
		{
			Serial.print("0");
		}
		Serial.print(mac[i], HEX);
		if (i > 0)
		{
			Serial.print(":");
		}
	}
	Serial.println();
}
