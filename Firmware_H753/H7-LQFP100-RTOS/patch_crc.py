import sys
import struct

APP_BASE = 0x08020000
CRC_ADDR = 0x080FF000

def compute_crc(data):
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(32):
            if crc & 0x80000000:
                crc = (crc << 1) ^ 0x04C11DB7
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF
    return crc

def main(bin_path):
    with open(bin_path, 'rb') as f:
        data = bytearray(f.read())

    crc_offset = CRC_ADDR - APP_BASE
    final_size = crc_offset + 4

    if len(data) < final_size:
        padding = final_size - len(data)
        print(f"Extending binary by {padding} bytes to reach CRC location.")
        data += b'\x00' * padding

    # Zero the CRC field before computing CRC
    data[crc_offset:crc_offset + 4] = b'\x00' * 4

    # Calculate CRC over range [0, crc_offset)
    crc_data = data[:crc_offset]
    # Optional 4-byte alignment (only if your bootloader requires it)
    pad_len = (4 - (len(crc_data) % 4)) % 4
    crc_data += b'\x00' * pad_len

    crc = compute_crc(crc_data)
    print(f"Patching CRC32: 0x{crc:08X} at offset 0x{crc_offset:X}")

    # Patch it into the correct location
    data[crc_offset:crc_offset + 4] = struct.pack('<I', crc)

    with open(bin_path, 'wb') as f:
        f.write(data)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python patch_crc_append.py firmware.bin")
        sys.exit(1)

    main(sys.argv[1])
