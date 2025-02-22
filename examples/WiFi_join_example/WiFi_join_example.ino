#include <CYW4343W_t4.h>
#include "secrets.h"

#define VERSION "1.00"

W4343WCard wifiCard;

void setup()
{
  Serial.begin(115200);
  // wait for serial port to connect.
  while (!Serial) {}

  Serial.printf("\nZerowi network join test v" VERSION "\n");

  Serial.printf("CPU speed: %ld MHz\n", F_CPU_ACTUAL / 1'000'000);
  pinMode(13, OUTPUT); // For debugging. Temporary usage.
    
  //////////////////////////////////////////
  //Begin parameters: 
  //SDIO1 (false), SDIO2 (true)
  //WL_REG_ON pin 
  //WL_IRQ pin (-1 to ignore)
  //EXT_LPO pin (optional, -1 to ignore)
  //////////////////////////////////////////
  if (wifiCard.begin(true, 33, 34, -1) == true) { 
    Serial.println("initialization done");

    wifiCard.getMACAddress();
    wifiCard.getFirmwareVersion();
  } else {
    Serial.println("initialization failed!");
  }

  Serial.println("Setup complete");

  waitforInput();

  // Use "secrets.h" to set MY_SSID, MY_PASSPHRASE, SECURITY.
  wifiCard.JoinNetworks(MY_SSID, MY_PASSPHRASE, SECURITY);

}

void loop() {}

void waitforInput()
{
  Serial.println("Press anykey to join a network...");
  while (Serial.read() == -1) ;
  while (Serial.read() != -1) ;
}
