import enum
from typing import Tuple, Iterator

import av

Packet = Tuple[bool, int, bytes]


class H264_NAL_TYPE(enum.Enum):
    UNSPECIFIED = 0
    SLICE = 1
    DPA = 2
    DPB = 3
    DPC = 4
    IDR_SLICE = 5
    SEI = 6
    SPS = 7
    PPS = 8
    AUD = 9
    END_SEQUENCE = 10
    END_STREAM = 11
    FILLER_DATA = 12
    SPS_EXT = 13
    PREFIX = 14
    SUB_SPS = 15
    DPS = 16
    AUXILIARY_SLICE = 19
    EXTEN_SLICE = 20
    DEPTH_EXTEN_SLICE = 21


H64_KEYFRAME_BEGIN_TYPES = {
    H264_NAL_TYPE.IDR_SLICE.value, H264_NAL_TYPE.SPS.value,
    H264_NAL_TYPE.PPS.value
}


def nalu_unit_type(header: int) -> int:
    return header & 31


def parse_nalu(stream: bytes) -> Iterator[Tuple[int, int]]:
    pos = 0
    while pos >= 0:
        pos = stream.find(b'\x00\x00\x01', pos)
        if pos >= 0 and pos + 3 < len(stream):
            yield pos, nalu_unit_type(stream[pos + 3])
            pos += 4


def is_keyframe(stream: bytes) -> bool:
    for i, (p, u) in enumerate(parse_nalu(stream)):
        if i > 1:
            break
        if u in H64_KEYFRAME_BEGIN_TYPES:
            return True
    return False


class H264Parser:

    def __init__(self):
        self.codec = av.CodecContext.create("h264", "r")
        self.pts = 0

    def parse(self, stream: bytes) -> Iterator[Packet]:
        for packet in self.codec.parse(stream):
            bs = bytes(packet)
            value = is_keyframe(bs), self.pts, bs
            self.pts += 1
            yield value
