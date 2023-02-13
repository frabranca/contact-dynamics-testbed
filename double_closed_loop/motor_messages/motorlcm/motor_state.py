"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class motor_state(object):
    __slots__ = ["position"]

    __typenames__ = ["double"]

    __dimensions__ = [None]

    def __init__(self):
        self.position = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(motor_state._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">d", self.position))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != motor_state._get_packed_fingerprint():
            raise ValueError("Decode error")
        return motor_state._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = motor_state()
        self.position = struct.unpack(">d", buf.read(8))[0]
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if motor_state in parents: return 0
        tmphash = (0xe743566f594023fd) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if motor_state._packed_fingerprint is None:
            motor_state._packed_fingerprint = struct.pack(">Q", motor_state._get_hash_recursive([]))
        return motor_state._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

