"""Packet type for Root Robot"""

from struct import pack
from crc8 import crc8


class Packet():
    """
    Root communication packet type
    """
    PACKET_LEN = 20
    PAYLOAD_LEN = 16

    def __init__(self, dev, cmd, inc, payload=bytes(PAYLOAD_LEN), crc=None):
        """
        Create a Packet.

        Parameters
        ----------

        dev, cmd, and id are mandatory

        payload type must be bytes with length <= Packet.PAYLOAD_LEN
        shorter payloads will be padded

        crc will be calculated if not supplied

        Usage
        -----

        Access the raw bytes with the Packet.bytes property

        A Packet may also be created from an existing byte array of
        length == self.PACKET_LEN via Packet.from_bytes()

        Packet.check_crc() will return True if the stored crc matches a
        calculated crc

        """
        self.dev = dev
        self.cmd = cmd
        self.inc = inc
        assert len(payload) <= self.PAYLOAD_LEN, "invalid payload length"
        self.payload = payload + bytes(self.PAYLOAD_LEN - len(payload))
        self._crc = crc

    @classmethod
    def from_bytes(cls, raw_bytes):
        assert len(raw_bytes) == cls.PACKET_LEN, "invalid packet len"
        return Packet(
            raw_bytes[0],  # device
            raw_bytes[1],  # command
            raw_bytes[2],  # id
            payload=raw_bytes[3:19],
            crc=raw_bytes[19])

    def check_crc(self):
        """check if computed crc matches stored crc"""
        return False if self._crc is None else self._crc == crc8(
            self.packet).digest()[0]  # compare ints

    @property
    def packet(self):
        """19 bytes of packet minus crc"""
        packet = pack('3B', self.dev, self.cmd, self.inc) + self.payload
        assert len(packet) == self.PACKET_LEN - 1
        return packet

    @property
    def bytes(self):
        """20 raw bytes of full packet with crc attached"""
        packet = self.packet
        packet += crc8(packet).digest()  # return byte
        assert len(packet) == self.PACKET_LEN, "invalid packet length"
        return packet

    @property
    def crc(self):
        """return stored crc or calculate new crc"""
        if self._crc is None:
            self._crc = crc8(self.packet).digest()[0]  # return int
        return self._crc
