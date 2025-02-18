#include <CYW4343W_t4.h>

W4343WCard wifiCard;

void setup()
{
  Serial.begin(115200);
  // wait for serial port to connect.
  while (!Serial) {}

  Serial.printf("CPU speed: %ld MHz\n", F_CPU_ACTUAL / 1'000'000);

  //////////////////////////////////////////
  //Begin parameters: 
  //SDIO1 (false), SDIO2 (true)
  //WL_REG_ON pin 
  //EXT_LPO pin (optional)
  //////////////////////////////////////////
  if (wifiCard.begin(false, 27) == true) { 
    Serial.println("initialization done");

    wifiCard.getMACAddress();
    wifiCard.getFirmwareVersion();
    wifiCard.ScanNetworks();

  } else {
    Serial.println("initialization failed!");
  }

  Serial.println("Setup complete");
}

void loop() {}