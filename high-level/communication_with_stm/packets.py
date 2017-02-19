import struct
from cmd_list import REVERSED_CMD_LIST
import numpy as np

SYNC = 0xFA
ADDR_SEND = 0xAF
ADDR_RECV = 0xFA


CRC_POLY = np.uint32(0x4C11DB7)
CONST_FF = np.uint32(0xFFFFFFFF)
CONST_80 = np.uint32(0x80000000)
CONST_1 = np.uint32(1)

# data must be of bytearray type
def crc(data):
    crc = CONST_FF
    for byte in data:
        crc = crc ^ np.uint32(byte)
        for i in range(32):
            to_poly = crc & CONST_80 != 0
            crc = (crc << CONST_1) & CONST_FF
            if to_poly:
                crc = crc ^ CRC_POLY
    return crc

def cut_crc(msg_crc):
    return struct.pack("<I", msg_crc)[:2]

def encode_params(params):
    res = bytearray()
    for param in params:
        if isinstance(param, str):
            res += param
        elif isinstance(param, int):
            if not 0<= param <= 255:
                raise ValueError("Can't encode int: %d" % param)
            #res += chr(param)
            res += struct.pack('<B', param)
        elif isinstance(param, float):
            # TODO check
            res += struct.pack('<f', param)
        else:
            raise ValueError("Unexpected parameter type: %d" % type(param))
    return res

def encode_packet(cmd_id, params):
    enc_params = encode_params(params)
    msg = bytearray()
    msg += chr(SYNC) + chr(ADDR_SEND) + chr(0) + chr(cmd_id) + enc_params
    msg[2] = len(msg) + 2
    msg += cut_crc(crc(msg))
    return msg


def decode_params(cmd, params):
    if cmd == 'getCurentCoordinates' or cmd == 'getCurrentSpeed':
        return [
            struct.unpack('<f', params[i*4:(i+1)*4])[0]  # TODO check correctness
            for i in range(3)
        ]

    #
    # if cmd in ['closeCubeCollector', 'getADCPinState']:  # One value answer!!
    #    return struct.unpack('>B', params)[0] # TODO check correctness
    if (len(params)>1):
        return str(params[:-1])  # Denis x00 deletetion
    else:
        return ord(params) # for char answer


def decode_packet(data):
    #print ([i for i in data])
    if data[0] != SYNC or data[1] != ADDR_RECV:
        raise ValueError('Wrong packet header: %s' % data)
    msg_len = data[2]
    if len(data) != msg_len:
        raise ValueError('Length missmatch, expected %d, got %d' %
            (msg_len, len(data)))
    cmd = data[3]
    if cmd not in REVERSED_CMD_LIST:
        raise ValueError('Unexpected command: %02x' % cmd)
    rev_cmd = REVERSED_CMD_LIST[cmd]
    msg_crc = cut_crc(crc(data[:-2]))
    if data[-2:] != msg_crc:
        raise ValueError('CRC missmatch')
    params = data[4:-2]
    data = decode_params(rev_cmd, params)
    res = {'cmd': rev_cmd}
    if data is not None:
        res['data'] = data
    return res
