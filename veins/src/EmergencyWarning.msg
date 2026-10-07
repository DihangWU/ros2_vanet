// SPDX-License-Identifier: GPL-3.0-or-later
// See lisence/THIRD_PARTY_NOTICES.md and lisence/GPL-3.0.txt.
import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;
class EmergencyWarning extends inet::FieldsChunk
{
    string eventId;
    double sourceTime;
    double sendTime;
    double x;
    double y;
    double speed;
}
