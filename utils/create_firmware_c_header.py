# Converts a firmware bin file to a c header file
# Change input and output files

with open("cypress-fmac-v5.10.9-2022_0909/cyfmac43430-sdio.bin", "rb") as f:    
    # Read firmware bin file
    firmware_bin = f.read()

with open("cypress-fmac-v5.10.9-2022_0909/cyfmac43430_fmac-sdio.c", "w") as f:
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
