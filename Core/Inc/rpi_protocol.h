#ifndef RPI_PROTOCOL_H
#define RPI_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "rpi_protocol_generated.h"

#define RPI_PROTOCOL_MAX_FRAME     (2U + 5U + RPI_PROTOCOL_MAX_PAYLOAD + 2U)
#define RPI_PROTOCOL_SOF_1         0xA5U
#define RPI_PROTOCOL_SOF_2         0x5AU
#define RPI_PROTOCOL_QUEUE_CAPACITY 4U
#define RPI_PROTOCOL_TX_CAPACITY    8U

typedef struct {
    uint8_t version;
    uint8_t message_type;
    uint8_t sequence;
    uint16_t payload_length;
    uint8_t payload[RPI_PROTOCOL_MAX_PAYLOAD];
} RpiFrame;

typedef void (*RpiFrameCallback)(const RpiFrame *frame, void *context);

typedef struct {
    RpiFrame frames[RPI_PROTOCOL_QUEUE_CAPACITY];
    RpiFrame urgent_frame;
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint8_t urgent_ready;
    uint32_t dropped;
} RpiFrameQueue;

typedef struct {
    uint16_t length;
    uint8_t bytes[RPI_PROTOCOL_MAX_FRAME];
} RpiEncodedFrame;

typedef struct {
    RpiEncodedFrame frames[RPI_PROTOCOL_TX_CAPACITY];
    RpiEncodedFrame urgent_frame;
    RpiEncodedFrame telemetry_frame;
    uint8_t head;
    uint8_t tail;
    uint8_t count;
    uint8_t urgent_ready;
    uint8_t telemetry_ready;
    uint32_t dropped_critical;
    uint32_t replaced_telemetry;
} RpiTxQueue;

typedef enum {
    RPI_PARSE_SOF_1 = 0,
    RPI_PARSE_SOF_2,
    RPI_PARSE_HEADER,
    RPI_PARSE_PAYLOAD,
    RPI_PARSE_CRC_LOW,
    RPI_PARSE_CRC_HIGH
} RpiParserState;

typedef struct {
    RpiParserState state;
    uint8_t header[5];
    uint8_t header_index;
    uint16_t payload_index;
    uint16_t received_crc;
    uint16_t running_crc;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t format_errors;
    RpiFrame frame;
    RpiFrameCallback callback;
    void *callback_context;
} RpiProtocolParser;

uint16_t RpiProtocol_Crc16(const uint8_t *data, uint16_t length);
uint16_t RpiProtocol_Encode(uint8_t message_type,
                            uint8_t sequence,
                            const uint8_t *payload,
                            uint16_t payload_length,
                            uint8_t *output,
                            uint16_t output_capacity);
void RpiProtocol_ParserInit(RpiProtocolParser *parser,
                            RpiFrameCallback callback,
                            void *callback_context);
void RpiProtocol_ParserReset(RpiProtocolParser *parser);
void RpiProtocol_FeedByte(RpiProtocolParser *parser, uint8_t byte);
void RpiProtocol_QueueReset(RpiFrameQueue *queue);
uint8_t RpiProtocol_QueuePush(RpiFrameQueue *queue,
                              const RpiFrame *frame,
                              uint8_t urgent);
uint8_t RpiProtocol_QueuePop(RpiFrameQueue *queue, RpiFrame *frame);
void RpiProtocol_TxQueueReset(RpiTxQueue *queue);
uint8_t RpiProtocol_TxQueuePush(RpiTxQueue *queue,
                                const uint8_t *bytes,
                                uint16_t length,
                                uint8_t priority);
uint8_t RpiProtocol_TxQueuePop(RpiTxQueue *queue, RpiEncodedFrame *frame);

#ifdef __cplusplus
}
#endif

#endif
