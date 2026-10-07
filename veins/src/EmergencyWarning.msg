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
