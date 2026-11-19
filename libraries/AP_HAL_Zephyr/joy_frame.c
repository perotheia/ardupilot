/* COPY of wc-hardware fw/joystick/logic/joy_frame.c @ e785750 (frame spec v1, docs/JOYSTICK-FRAME.md): keep in sync */
#include "joy_frame.h"
#include <string.h>

uint16_t joy_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;               /* CRC-16/CCITT-FALSE */
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static size_t finish(uint8_t *out, uint8_t type, uint8_t len)
{
    out[0] = JOY_SYNC0; out[1] = JOY_SYNC1; out[2] = type; out[3] = len;
    put16(&out[4 + len], joy_crc16(&out[2], 2u + len));
    return 4u + len + 2u;
}

size_t joy_encode_state(const joy_state_t *s, uint8_t out[JOY_STATE_FRAME_BYTES])
{
    uint8_t *p = &out[4];
    p[0] = s->seq; p[1] = s->flags; p[2] = s->mode; p[3] = s->fault;
    put16(&p[4], (uint16_t)s->fx_cN); put16(&p[6], (uint16_t)s->fy_cN);
    put16(&p[8], (uint16_t)s->throttle); put16(&p[10], (uint16_t)s->steering);
    p[12] = s->age_ms; p[13] = 0;
    return finish(out, JOY_TYPE_STATE, JOY_STATE_LEN);
}

size_t joy_encode_status(const joy_status_t *s, uint8_t out[JOY_STATUS_FRAME_BYTES])
{
    uint8_t *p = &out[4];
    p[0] = s->last_seq; p[1] = s->state; p[2] = 0; p[3] = 0;
    return finish(out, JOY_TYPE_STATUS, JOY_STATUS_LEN);
}

int joy_decode_state(const uint8_t *p, size_t len, joy_state_t *s)
{
    if (len != JOY_STATE_LEN) return -1;
    s->seq = p[0]; s->flags = p[1]; s->mode = p[2]; s->fault = p[3];
    s->fx_cN = (int16_t)get16(&p[4]); s->fy_cN = (int16_t)get16(&p[6]);
    s->throttle = (int16_t)get16(&p[8]); s->steering = (int16_t)get16(&p[10]);
    s->age_ms = p[12];
    if (s->mode > JOY_MODE_PIVOT || s->throttle < -1000 || s->throttle > 1000 ||
        s->steering < -1000 || s->steering > 1000) return -2;
    return 0;
}

int joy_decode_status(const uint8_t *p, size_t len, joy_status_t *s)
{
    if (len != JOY_STATUS_LEN) return -1;
    s->last_seq = p[0]; s->state = p[1];
    return 0;
}

void joy_parser_init(joy_parser_t *p) { memset(p, 0, sizeof *p); }

/* drop the first byte of the buffer and re-scan it for a new start */
static void resync(joy_parser_t *p)
{
    p->resyncs++;
    size_t i = 1;
    while (i < p->n && p->buf[i] != JOY_SYNC0) i++;
    memmove(p->buf, &p->buf[i], p->n - i);
    p->n -= i;
}

int joy_parser_push(joy_parser_t *p, uint8_t byte)
{
    p->buf[p->n++] = byte;
    for (;;) {
        if (p->n >= 1 && p->buf[0] != JOY_SYNC0) { resync(p); continue; }
        if (p->n >= 2 && p->buf[1] != JOY_SYNC1) { resync(p); continue; }
        if (p->n >= 4 && p->buf[3] > JOY_MAX_LEN) { resync(p); continue; }
        if (p->n < 4) return 0;
        size_t total = 4u + p->buf[3] + 2u;
        if (p->n < total) return 0;
        uint16_t want = get16(&p->buf[total - 2]);
        if (joy_crc16(&p->buf[2], 2u + p->buf[3]) != want) {
            p->crc_errors++;
            resync(p);
            if (p->n == 0) return 0;
            continue;
        }
        int type = p->buf[2];
        memcpy(p->frame, p->buf, total);           /* hand the frame out, keep any remainder */
        memmove(p->buf, &p->buf[total], p->n - total);
        p->n -= total;
        return type;
    }
}
