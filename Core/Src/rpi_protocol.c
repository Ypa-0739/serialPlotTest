#include "rpi_protocol.h"

#include <stddef.h>

#define RPI_HEADER_BODY_SIZE 5U
#define RPI_FIXED_FRAME_SIZE 9U

static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    uint8_t bit;
    crc ^= (uint16_t)byte << 8;
    for (bit = 0U; bit < 8U; ++bit) {
        crc = (crc & 0x8000U) != 0U ?
              (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

uint16_t RpiProtocol_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    if (data == NULL && length != 0U) return 0U;
    for (index = 0U; index < length; ++index) crc = crc16_update(crc, data[index]);
    return crc;
}

uint16_t RpiProtocol_Encode(uint8_t message_type,
                            uint8_t sequence,
                            const uint8_t *payload,
                            uint16_t payload_length,
                            uint8_t *output,
                            uint16_t output_capacity)
{
    uint16_t crc;
    uint16_t index;
    uint16_t total_length = (uint16_t)(RPI_FIXED_FRAME_SIZE + payload_length);
    if (output == NULL || (payload == NULL && payload_length != 0U) ||
        payload_length > RPI_PROTOCOL_MAX_PAYLOAD || output_capacity < total_length) {
        return 0U;
    }
    output[0] = RPI_PROTOCOL_SOF_1;
    output[1] = RPI_PROTOCOL_SOF_2;
    output[2] = RPI_PROTOCOL_VERSION;
    output[3] = message_type;
    output[4] = sequence;
    output[5] = (uint8_t)(payload_length & 0xFFU);
    output[6] = (uint8_t)(payload_length >> 8);
    for (index = 0U; index < payload_length; ++index) output[7U + index] = payload[index];
    crc = RpiProtocol_Crc16(&output[2], (uint16_t)(5U + payload_length));
    output[7U + payload_length] = (uint8_t)(crc & 0xFFU);
    output[8U + payload_length] = (uint8_t)(crc >> 8);
    return total_length;
}

void RpiProtocol_ParserReset(RpiProtocolParser *parser)
{
    if (parser == NULL) return;
    parser->state = RPI_PARSE_SOF_1;
    parser->header_index = 0U;
    parser->payload_index = 0U;
    parser->received_crc = 0U;
    parser->running_crc = 0xFFFFU;
}

void RpiProtocol_ParserInit(RpiProtocolParser *parser,
                            RpiFrameCallback callback,
                            void *callback_context)
{
    if (parser == NULL) return;
    parser->valid_frames = 0U;
    parser->crc_errors = 0U;
    parser->format_errors = 0U;
    parser->callback = callback;
    parser->callback_context = callback_context;
    RpiProtocol_ParserReset(parser);
}

void RpiProtocol_FeedByte(RpiProtocolParser *parser, uint8_t byte)
{
    uint16_t payload_length;
    if (parser == NULL) return;
    switch (parser->state) {
    case RPI_PARSE_SOF_1:
        if (byte == RPI_PROTOCOL_SOF_1) parser->state = RPI_PARSE_SOF_2;
        break;
    case RPI_PARSE_SOF_2:
        if (byte == RPI_PROTOCOL_SOF_2) {
            parser->state = RPI_PARSE_HEADER;
            parser->header_index = 0U;
            parser->running_crc = 0xFFFFU;
        } else if (byte != RPI_PROTOCOL_SOF_1) {
            parser->state = RPI_PARSE_SOF_1;
        }
        break;
    case RPI_PARSE_HEADER:
        parser->header[parser->header_index++] = byte;
        parser->running_crc = crc16_update(parser->running_crc, byte);
        if (parser->header_index == RPI_HEADER_BODY_SIZE) {
            payload_length = (uint16_t)parser->header[3] |
                             ((uint16_t)parser->header[4] << 8);
            if (parser->header[0] != RPI_PROTOCOL_VERSION ||
                payload_length > RPI_PROTOCOL_MAX_PAYLOAD) {
                ++parser->format_errors;
                RpiProtocol_ParserReset(parser);
                break;
            }
            parser->frame.version = parser->header[0];
            parser->frame.message_type = parser->header[1];
            parser->frame.sequence = parser->header[2];
            parser->frame.payload_length = payload_length;
            parser->payload_index = 0U;
            parser->state = payload_length == 0U ? RPI_PARSE_CRC_LOW : RPI_PARSE_PAYLOAD;
        }
        break;
    case RPI_PARSE_PAYLOAD:
        parser->frame.payload[parser->payload_index++] = byte;
        parser->running_crc = crc16_update(parser->running_crc, byte);
        if (parser->payload_index == parser->frame.payload_length) parser->state = RPI_PARSE_CRC_LOW;
        break;
    case RPI_PARSE_CRC_LOW:
        parser->received_crc = byte;
        parser->state = RPI_PARSE_CRC_HIGH;
        break;
    case RPI_PARSE_CRC_HIGH:
        parser->received_crc |= (uint16_t)byte << 8;
        if (parser->received_crc == parser->running_crc) {
            ++parser->valid_frames;
            if (parser->callback != NULL) parser->callback(&parser->frame, parser->callback_context);
        } else {
            ++parser->crc_errors;
        }
        RpiProtocol_ParserReset(parser);
        break;
    default:
        ++parser->format_errors;
        RpiProtocol_ParserReset(parser);
        break;
    }
}

void RpiProtocol_QueueReset(RpiFrameQueue *queue)
{
    if (queue == 0) return;
    queue->head = 0U;
    queue->tail = 0U;
    queue->count = 0U;
    queue->urgent_ready = 0U;
    queue->dropped = 0U;
}

uint8_t RpiProtocol_QueuePush(RpiFrameQueue *queue,
                              const RpiFrame *frame,
                              uint8_t urgent)
{
    if (queue == 0 || frame == 0) return 0U;
    if (urgent) {
        /* STOP 是接收屏障：清除旧命令，处理前拒绝后续普通帧。 */
        queue->dropped += queue->count;
        queue->head = 0U;
        queue->tail = 0U;
        queue->count = 0U;
        queue->urgent_frame = *frame;
        queue->urgent_ready = 1U;
        return 1U;
    }
    if (queue->urgent_ready || queue->count >= RPI_PROTOCOL_QUEUE_CAPACITY) {
        queue->dropped++;
        return 0U;
    }
    queue->frames[queue->head] = *frame;
    queue->head = (uint8_t)((queue->head + 1U) % RPI_PROTOCOL_QUEUE_CAPACITY);
    queue->count++;
    return 1U;
}

uint8_t RpiProtocol_QueuePop(RpiFrameQueue *queue, RpiFrame *frame)
{
    if (queue == 0 || frame == 0) return 0U;
    if (queue->urgent_ready) {
        *frame = queue->urgent_frame;
        queue->urgent_ready = 0U;
        return 1U;
    }
    if (queue->count == 0U) return 0U;
    *frame = queue->frames[queue->tail];
    queue->tail = (uint8_t)((queue->tail + 1U) % RPI_PROTOCOL_QUEUE_CAPACITY);
    queue->count--;
    return 1U;
}

void RpiProtocol_TxQueueReset(RpiTxQueue *queue)
{
    if (queue == 0) return;
    queue->head = 0U;
    queue->tail = 0U;
    queue->count = 0U;
    queue->urgent_ready = 0U;
    queue->telemetry_ready = 0U;
    queue->dropped_critical = 0U;
    queue->replaced_telemetry = 0U;
}

static void copy_encoded_frame(RpiEncodedFrame *frame,
                               const uint8_t *bytes,
                               uint16_t length)
{
    uint16_t index;
    frame->length = length;
    for (index = 0U; index < length; ++index) frame->bytes[index] = bytes[index];
}

uint8_t RpiProtocol_TxQueuePush(RpiTxQueue *queue,
                                const uint8_t *bytes,
                                uint16_t length,
                                uint8_t priority)
{
    if (queue == 0 || bytes == 0 || length == 0U ||
        length > RPI_PROTOCOL_MAX_FRAME) return 0U;
    /* 2=urgent safety latch, 1=control FIFO, 0=latest telemetry. */
    if (priority >= 2U) {
        copy_encoded_frame(&queue->urgent_frame, bytes, length);
        queue->urgent_ready = 1U;
        return 1U;
    }
    if (priority == 0U) {
        if (queue->telemetry_ready) queue->replaced_telemetry++;
        copy_encoded_frame(&queue->telemetry_frame, bytes, length);
        queue->telemetry_ready = 1U;
        return 1U;
    }
    if (queue->count >= RPI_PROTOCOL_TX_CAPACITY) {
        queue->dropped_critical++;
        return 0U;
    }
    copy_encoded_frame(&queue->frames[queue->head], bytes, length);
    queue->head = (uint8_t)((queue->head + 1U) % RPI_PROTOCOL_TX_CAPACITY);
    queue->count++;
    return 1U;
}

uint8_t RpiProtocol_TxQueuePop(RpiTxQueue *queue, RpiEncodedFrame *frame)
{
    if (queue == 0 || frame == 0) return 0U;
    if (queue->urgent_ready) {
        *frame = queue->urgent_frame;
        queue->urgent_ready = 0U;
        return 1U;
    }
    if (queue->count != 0U) {
        *frame = queue->frames[queue->tail];
        queue->tail = (uint8_t)((queue->tail + 1U) % RPI_PROTOCOL_TX_CAPACITY);
        queue->count--;
        return 1U;
    }
    if (queue->telemetry_ready) {
        *frame = queue->telemetry_frame;
        queue->telemetry_ready = 0U;
        return 1U;
    }
    return 0U;
}
