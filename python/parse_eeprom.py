import struct


def get_uint8(data: bytearray, offset: int) -> int:
    return struct.unpack('<B', data[offset:offset + 1])[0]


def get_sint8(data: bytearray, offset: int) -> int:
    return struct.unpack('<b', data[offset:offset + 1])[0]


def get_uint16(data: bytearray, offset: int) -> int:
    return struct.unpack('<H', data[offset:offset + 2])[0]


def get_sint16(data: bytearray, offset: int) -> int:
    return struct.unpack('<h', data[offset:offset + 2])[0]


def get_float(data: bytearray, offset: int) -> float:
    return struct.unpack('<f', data[offset:offset + 4])[0]


def get_n_uint16(data: bytearray, offset: int, n: int) -> list:
    return [get_uint16(data, offset + i * 2) for i in range(n)]


def get_n_sint16(data: bytearray, offset: int, n: int) -> list:
    return [get_sint16(data, offset + i * 2) for i in range(n)]


def get_uint32(data: bytearray, offset: int) -> int:
    return struct.unpack('<I', data[offset:offset + 4])[0]


# sensor no 2 is one with different focal lenght
sensor_number = 2
with open(f"Sensor{sensor_number}_dump.txt", 'r') as file:
    hex_content = ''.join(file.read().split('\n')[3:])

bytes_array = bytearray.fromhex(hex_content)

print(bytes_array)

print("PixCmin: ", get_float(bytes_array, 0x0000))
print("PixCmax: ", get_float(bytes_array, 0x0004))
print("gradScale: ", get_uint8(bytes_array, 0x0008))
print("TableNumber: ", get_uint16(bytes_array, 0x000B))
print("epsilon: ", get_uint8(bytes_array, 0x000D))

print("MBIT(calib): ", get_uint8(bytes_array, 0x001A))
print("BIAS(calib): ", get_uint8(bytes_array, 0x001B))
print("CLK(calib): ", get_uint8(bytes_array, 0x001C))
print("BPA(calib): ", get_uint8(bytes_array, 0x001D))
print("PU(calib): ", get_uint8(bytes_array, 0x001E))

print("Arraytype: ", get_uint8(bytes_array, 0x0022))
print("VDDTH1: ", get_uint16(bytes_array, 0x0026))
print("VDDTH2: ", get_uint16(bytes_array, 0x0028))

print("PTAT-gradient: ", get_float(bytes_array, 0x0034))
print("PTAT-offset: ", get_float(bytes_array, 0x0038))
print("PTAT(Th1): ", get_uint16(bytes_array, 0x003C))
print("PTAT(Th2): ", get_uint16(bytes_array, 0x003E))

print("VddScGrad: ", get_uint8(bytes_array, 0x004E))
print("VddScOff: ", get_uint8(bytes_array, 0x004F))

print("GlobalOff: ", get_uint8(bytes_array, 0x0054))
print("GlobalGain: ", get_uint16(bytes_array, 0x0055))

print("MBIT(user): ", get_uint8(bytes_array, 0x0060))
print("BIAS(user): ", get_uint8(bytes_array, 0x0061))
print("CLK(user): ", get_uint8(bytes_array, 0x0062))
print("BPA(user): ", get_uint8(bytes_array, 0x0063))
print("PU(user): ", get_uint8(bytes_array, 0x0064))

print("DeviceID: ", get_uint32(bytes_array, 0x0074))
print("NrOfDefPix: ", get_uint8(bytes_array, 0x007F))

print("DeadPixAdr: ", get_n_uint16(bytes_array, 0x0080, 8 * 3))

print("DeadPixMask: ")

print("VddCompGrad_ij: ", get_n_sint16(bytes_array, 0x0340, 8 * 32))
print("VddCompOff_ij: ", get_n_sint16(bytes_array, 0x0540, 8 * 32))

print("ThGrad_ij: ", get_n_sint16(bytes_array, 0x0740, 8 * 128))
print("ThOff_ij: ", get_n_sint16(bytes_array, 0x0F40, 8 * 128))

print("P_ij: ", get_n_uint16(bytes_array, 0x1740, 8 * 128))
