"""Strict packed CollisionData reader for the repository's MORAI NetworkModule.

Layout source: src/common/morai_network/lib/define/CollisionData.py (5 records).
Wire header/length and empty-slot convention must be checked against a capture
from 25.S4.MolitComp03. This module never sends simulator commands.
"""

import math
import struct

HEADER = struct.Struct("<15s i 3i i i")
RECORD = struct.Struct("<hh6f")
PACKET_SIZE = HEADER.size + 5*RECORD.size + 2


def parse_collision_packet(packet, expected_header=b"#CollisionData$"):
    if len(packet) != PACKET_SIZE:
        raise ValueError("CollisionData expected {} bytes; got {}".format(PACKET_SIZE,len(packet)))
    header,length,_,_,_,sec,nsec = HEADER.unpack_from(packet)
    if header != expected_header or length != 148 or packet[-2:] != b"\r\n":
        raise ValueError("Unrecognized CollisionData header/data_length/footer")
    if sec < 0 or not 0 <= nsec < 1_000_000_000:
        raise ValueError("Invalid CollisionData timestamp")
    objects = []
    for i in range(5):
        record = RECORD.unpack_from(packet,HEADER.size+i*RECORD.size)
        obj_type,obj_id,*coordinates = record
        if not all(math.isfinite(v) for v in coordinates):
            raise ValueError("Nonfinite CollisionData coordinate")
        if all(v == 0 for v in record):
            continue
        objects.append({"key":"{}:{}".format(obj_type,obj_id),"type":obj_type,"id":obj_id,
                        "position":coordinates[:3],"global_offset":coordinates[3:]})
    return {"packet_timestamp_sec":sec+nsec/1e9,"objects":objects,
            "keys":sorted({v["key"] for v in objects}),"at_capacity":len(objects)==5}
