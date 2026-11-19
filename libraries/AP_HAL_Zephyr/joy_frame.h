/* COPY of wc-hardware fw/joystick/logic/joy_frame.h @ e785750 (frame spec v1, docs/JOYSTICK-FRAME.md): keep in sync */
/* Joystick UART frame codec, spec: docs/JOYSTICK-FRAME.md (v1). Pure C, no platform code. */
#ifndef JOY_FRAME_H
#define JOY_FRAME_H
#include <stddef.h>
#include <stdint.h>

#define JOY_SYNC0 0xA5
#define JOY_SYNC1 0x5A
#define JOY_TYPE_STATE  0x01
#define JOY_TYPE_STATUS 0x81
#define JOY_STATE_LEN   14
#define JOY_STATUS_LEN  4
#define JOY_MAX_LEN     32
#define JOY_STATE_FRAME_BYTES  (4 + JOY_STATE_LEN + 2)
#define JOY_STATUS_FRAME_BYTES (4 + JOY_STATUS_LEN + 2)

enum { JOY_F_GRIP = 1u << 0, JOY_F_ARMED = 1u << 1, JOY_F_FAULT = 1u << 2, JOY_F_CALIBRATED = 1u << 3,
       JOY_F_ZEROING = 1u << 4, JOY_F_MODE_REJECTED = 1u << 5, JOY_F_OVERLOAD = 1u << 6 };
enum { JOY_MODE_FORWARD = 0, JOY_MODE_BACK = 1, JOY_MODE_PIVOT = 2 };
enum { JOY_FAULT_NONE = 0, JOY_FAULT_SAT_A, JOY_FAULT_SAT_B, JOY_FAULT_STALE_A, JOY_FAULT_STALE_B,
       JOY_FAULT_NOISE_A, JOY_FAULT_NOISE_B, JOY_FAULT_ZERO_RANGE, JOY_FAULT_NO_CAL, JOY_FAULT_BOOT };
enum { JOY_S_ACTIVE = 1u << 0, JOY_S_ESTOP_OPEN = 1u << 1, JOY_S_HOLD = 1u << 2, JOY_S_OVERVOLT = 1u << 3,
       JOY_S_NEED_NEUTRAL = 1u << 4, JOY_S_REMOTE_STOP = 1u << 5 };

typedef struct {
    uint8_t seq, flags, mode, fault;
    int16_t fx_cN, fy_cN;          /* 0.01 N */
    int16_t throttle, steering;    /* -1000..1000 */
    uint8_t age_ms;
} joy_state_t;

typedef struct {
    uint8_t last_seq, state;
} joy_status_t;

uint16_t joy_crc16(const uint8_t *data, size_t len);
size_t joy_encode_state(const joy_state_t *s, uint8_t out[JOY_STATE_FRAME_BYTES]);
size_t joy_encode_status(const joy_status_t *s, uint8_t out[JOY_STATUS_FRAME_BYTES]);
int joy_decode_state(const uint8_t *payload, size_t len, joy_state_t *s);   /* 0 ok */
int joy_decode_status(const uint8_t *payload, size_t len, joy_status_t *s); /* 0 ok */

/* byte-wise receiver: feed bytes, returns the TYPE of a complete good frame or 0 */
typedef struct {
    uint8_t buf[4 + JOY_MAX_LEN + 2];   /* bytes not yet consumed */
    size_t n;
    uint8_t frame[4 + JOY_MAX_LEN + 2]; /* last good frame, valid until the next good frame */
    uint32_t crc_errors, resyncs;
} joy_parser_t;
void joy_parser_init(joy_parser_t *p);
int joy_parser_push(joy_parser_t *p, uint8_t byte);
static inline const uint8_t *joy_parser_payload(const joy_parser_t *p) { return &p->frame[4]; }
static inline size_t joy_parser_len(const joy_parser_t *p) { return p->frame[3]; }

#endif
