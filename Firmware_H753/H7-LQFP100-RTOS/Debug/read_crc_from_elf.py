from elftools.elf.elffile import ELFFile
import sys
import struct

def read_crc_from_elf(elf_path):
    with open(elf_path, 'rb') as f:
        elf = ELFFile(f)
        symtab = elf.get_section_by_name('.symtab')
        sym = symtab.get_symbol_by_name('firmware_crc_placeholder')[0]
        crc_addr = sym['st_value']

        for segment in elf.iter_segments():
            if segment['p_vaddr'] <= crc_addr < segment['p_vaddr'] + segment['p_memsz']:
                offset = segment['p_offset'] + (crc_addr - segment['p_vaddr'])
                f.seek(offset)
                raw = f.read(4)
                crc = struct.unpack('<I', raw)[0]
                print(f"CRC value at 0x{crc_addr:08X}: 0x{crc:08X}")
                return

        print("Symbol not found in any segment")

if __name__ == '__main__':
    read_crc_from_elf(sys.argv[1])
