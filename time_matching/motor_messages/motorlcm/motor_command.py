"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class motor_command(object):
    __slots__ = ["motor_enable"]

    __typenames__ = ["boolean"]

    __dimensions__ = [None]

    def __init__(self):
        self.motor_enable = False

    def encode(self):
        buf = BytesIO()
        buf.write(motor_command._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">b", self.motor_enable))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != motor_command._get_packed_fingerprint():
            raise ValueError("Decode error")
        return motor_command._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = motor_command()
        self.motor_enable = bool(struct.unpack('b', buf.read(1))[0])
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if motor_command in parents: return 0
        tmphash = (0xb5e57d0c17f3aa60) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if motor_command._packed_fingerprint is None:
            motor_command._packed_fingerprint = struct.pack(">Q", motor_command._get_hash_recursive([]))
        return motor_command._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", motor_command._get_packed_fingerprint())[0]
