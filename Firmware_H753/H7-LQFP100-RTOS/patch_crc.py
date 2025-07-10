import sys
import struct

def reflect_byte(byte):
    return int('{:08b}'.format(byte)[::-1], 2)

def stm32_crc(data: bytes) -> int:
    poly = 0x04C11DB7
    crc = 0x00000000

    for b in data:
        b = reflect_byte(b)  # Byte-wise inversion
        crc ^= (b << 24)
        for _ in range(8):
            if (crc & 0x80000000):
                crc = ((crc << 1) ^ poly) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF

    return ~crc & 0xFFFFFFFF  # Output inversion

def patch_crc(elf_path):
    from elftools.elf.elffile import ELFFile

    with open(elf_path, 'rb+') as f:
        elf = ELFFile(f)
        symtab = elf.get_section_by_name('.symtab')
        sym = symtab.get_symbol_by_name('firmware_crc_placeholder')[0]
        crc_addr = sym['st_value']

        # Find file offset of CRC placeholder
        for segment in elf.iter_segments():
            if segment['p_vaddr'] <= crc_addr < segment['p_vaddr'] + segment['p_memsz']:
                offset = segment['p_offset'] + (crc_addr - segment['p_vaddr'])
                break
        else:
            print("CRC address not found in any segment")
            return

        # Read everything up to the placeholder
        f.seek(0)
        data = f.read(offset)

        crc = stm32_crc(data)
        print(f"STM32-matching CRC: 0x{crc:08X}")

        # Patch the CRC into the ELF
        f.seek(offset)
        f.write(struct.pack('<I', crc))
        print("Patched CRC.")

if __name__ == "__main__":
    patch_crc(sys.argv[1])
