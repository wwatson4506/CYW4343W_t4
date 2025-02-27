# Converts a firmware bin file to a c header file
# Change input and output files
def firmware_bin_convert():
    with open("brcmfmac43430-sdio.bin", "rb") as f:    
        # Read firmware bin file
        firmware_bin = f.read()

    with open("brcmfmac43430-sdio-armbian.c", "w") as f:
        # Write header
        f.write("#include <Arduino.h>\n\n")
        f.write(f"#define FIRMWARE_LEN {len(firmware_bin)}\n\n")
        
        f.write("PROGMEM const unsigned char firmware_bin[] = {\n")

        # Write data
        for i, byte in enumerate(firmware_bin):
            if i % 16 == 0:
                f.write("    ")
            f.write(f"0x{byte:02X}, ")
            if i % 16 == 15 or i == len(firmware_bin) - 1:
                f.write("\n")
        
        # Write footer
        f.write("};\n")

# Converts a firmware CLM file to a c header file
# Change input and output files
def firmware_clm_convert():
    with open("cyfmac43430-sdio.1DX.clm_blob", "rb") as f:    
        # Read firmware clm blob file
        firmware_clm = f.read()

    with open("cyfmac43430-sdio-1DX-clm_blob.c", "w") as f:
        # Write header
        f.write("#include <Arduino.h>\n\n")
        f.write(f"#define FIRMWARE_CLM_LEN {len(firmware_clm)}\n\n")
        
        f.write("PROGMEM const unsigned char firmware_clm[] = {\n")

        # Write data
        for i, byte in enumerate(firmware_clm):
            if i % 16 == 0:
                f.write("    ")
            f.write(f"0x{byte:02X}, ")
            if i % 16 == 15 or i == len(firmware_clm) - 1:
                f.write("\n")
        
        # Write footer
        f.write("};\n")

firmware_clm_convert()