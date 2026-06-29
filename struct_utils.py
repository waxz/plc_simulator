import struct

import struct
from typing import List, Tuple, get_type_hints, get_origin, get_args

# Mapping of standard Python types to struct format characters
TYPE_MAP = {
    float: 'f',    # 32-bit float
    int: 'i',      # 4-byte standard signed integer
    bool: '?',     # 1-byte boolean
}

class ConveyorPayload:
    bitmap: bytes
    floats: List[float]
    
    def __init__(self, bitmap: bytes, floats: List[float]):
        self.bitmap = bitmap
        self.floats = floats

def generate_format_string(cls, list_lengths: dict, endian: str = 'little') -> str:
    """
    Dynamically generates the struct format string based on class annotations
    and the dynamic array dimensions passed via list_lengths.
    """
    prefix = '>' if endian.lower() == 'big' else '<'
    format_chars = []
    
    hints = get_type_hints(cls)
    
    for field_name, field_type in hints.items():
        # Bytes fields are treated as raw blocks and skipped in the struct formatting
        if field_type is bytes:
            continue
            
        origin = get_origin(field_type)
        
        # Correctly handle List types using get_origin
        if origin is list or origin is List:
            inner_type = get_args(field_type)[0]
            fmt_char = TYPE_MAP.get(inner_type, 'f')
            # Look up how many elements this list is expected to hold
            length = list_lengths.get(field_name, 0)
            format_chars.append(f"{length}{fmt_char}")
            
        elif field_type in TYPE_MAP:
            format_chars.append(TYPE_MAP[field_type])
            
    return f"{prefix}{''.join(format_chars)}"

def pack_dynamic(payload: ConveyorPayload, endian: str = 'little') -> bytes:
    """Packs any annotated object into a byte array dynamically."""
    # Build list length map from the actual runtime fields
    list_lengths = {'floats': len(payload.floats)}
    fmt = generate_format_string(ConveyorPayload, list_lengths, endian)
    
    packed_floats = struct.pack(fmt, *payload.floats)
    return payload.bitmap + packed_floats

def unpack_dynamic(cls, data_bytes: bytes, list_lengths: dict, bitmap_len_bytes: int = 1, endian: str = 'little'):
    """Unpacks a byte array back into an annotated class object dynamically."""
    # 1. Generate format string cleanly without using inaccurate dummy objects
    fmt = generate_format_string(cls, list_lengths, endian)
    
    # 2. Extract the bitmap prefix based on the true expected length
    extracted_bitmap = data_bytes[:bitmap_len_bytes]
    
    # 3. Unpack remaining fields with the correct offset shift [1]
    unpacked_values = struct.unpack_from(fmt, data_bytes, offset=bitmap_len_bytes)
    
    return cls(bitmap=extracted_bitmap, floats=list(unpacked_values))



def string_to_bitmap(bit_string: str, alignment: str = 'left') -> bytes:
    bit_int = int(bit_string, 2)
    num_bytes = (len(bit_string) + 7) // 8
    remainder = len(bit_string) % 8
    if remainder != 0 and alignment.lower() == 'left':
        bit_int <<= (8 - remainder)
    return bit_int.to_bytes(num_bytes, byteorder='big')

def pack_data(bitmap_bytes, floats_list, endian='little'):
    prefix = '>' if endian.lower() == 'big' else '<'
    format_str = f'{prefix}{len(floats_list)}f'
    floats_bytes = struct.pack(format_str, *floats_list)
    return bitmap_bytes + floats_bytes

def unpack_data(data_bytes: bytes, num_floats: int, bitmap_len_bytes: int = 1, endian: str = 'little'):
    """Extracts the bitmap bytes and the list of floats from a combined byte array."""
    # 1. Extract the raw bitmap bytes from the front of the array
    bitmap_bytes = data_bytes[:bitmap_len_bytes]
    
    # 2. Determine endianness prefix for struct unpacking
    prefix = '>' if endian.lower() == 'big' else '<'
    format_str = f'{prefix}{num_floats}f'
    
    # 3. Unpack the floats starting immediately after the bitmap bytes
    # struct.unpack_from avoids manual slicing of the byte stream
    floats_tuple = struct.unpack_from(format_str, data_bytes, offset=bitmap_len_bytes)
    
    return bitmap_bytes, list(floats_tuple)

def print_byte_by_byte(data: bytes):
    print(f"{'Byte Index':<12} | {'Hex Value':<10} | {'Bit Representation (8-bit)':<25}")
    print("-" * 55)
    for i, byte_value in enumerate(data):
        hex_str = f"0x{byte_value:02x}"
        print(f"Byte {i:<7} | {hex_str:<10} | {byte_value:08b}")

if __name__ == "__main__":
    # 1. Create original data
    bitmap_input = string_to_bitmap('0101', alignment='left')
    floats_input = [3.14, 2.71]
    
    print("--- PACKING DATA (LITTLE ENDIAN) ---")
    packed_stream = pack_data(bitmap_input, floats_input, endian='little')
    print_byte_by_byte(packed_stream)
    
    print("\n--- UNPACKING DATA ---")
    # 2. Extract data back out
    # We specify we expect 2 floats and that our bitmap occupies 1 byte
    unpacked_bitmap, unpacked_floats = unpack_data(
        packed_stream, 
        num_floats=len(floats_input), 
        bitmap_len_bytes=len(bitmap_input), 
        endian='little'
    )
    
    print(f"Extracted Bitmap (Hex): 0x{unpacked_bitmap.hex()}")
    # Using round() just to clean up floating point precision display artifacts
    print(f"Extracted Floats:       {[round(f, 2) for f in unpacked_floats]}")
     # Simulated function output from your previous step ('0101' left-aligned)
    bitmap_data = b'\x50\x30\x12' 
    floats = [12904027136.0, -192.18, -0.0,3.5]
    original_data = ConveyorPayload(bitmap=bitmap_data, floats=floats)
    
    # 1. Pack Data
    packed_stream = pack_dynamic(original_data, endian='big')
    print("Packed Stream Hex:", packed_stream.hex())
    
    # 2. Unpack Data
    unpacked_data = unpack_dynamic(
        ConveyorPayload, 
        packed_stream, 
        list_lengths={'floats': len(floats)}, 
        bitmap_len_bytes=len(bitmap_data), 
        endian='big'
    )
    
    print("\n--- Unpacked Verification ---")
    print("Bitmap (Hex):", unpacked_data.bitmap.hex())
    print("Floats:      ", [round(f, 2) for f in unpacked_data.floats])

