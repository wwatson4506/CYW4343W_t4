#ifndef IOCTL_H
#define IOCTL_H

#ifdef __cplusplus
extern "C" {
#endif


#include <Arduino.h>

#include "../whd/whd_types.h"
#include "../whd/whd_events.h"
#include "../whd/whd_wlioctl.h"

// IOCTL commands
#define IOCTL_UP                    2
#define IOCTL_SET_SCAN_CHANNEL_TIME 0xB9

#define IOCTL_POLL_MSEC     2
#define SCAN_CHAN_TIME      40
#define SCANTYPE_ACTIVE     0
#define SCANTYPE_PASSIVE    1


#define IOCTL_WAIT_USEC     2000
#define IOCTL_MAX_BLKLEN    256
#define SSID_MAXLEN         32

#define EVENT_SET_SSID      0
#define EVENT_JOIN          1
#define EVENT_AUTH          3
#define EVENT_LINK          16
#define EVENT_MAX           208
#define SET_EVENT(msk, e)   msk[e/8] |= 1 << (e & 7)

typedef struct {
    int32_t num;
    const char * str;
} EVT_STR;
#define EVT(e)      {e, #e}

#define NO_EVTS     {EVT(-1)}
#define ESCAN_EVTS  {EVT(WLC_E_ESCAN_RESULT), EVT(-1)}
#define JOIN_EVTS   {EVT(WLC_E_SET_SSID), EVT(WLC_E_LINK), EVT(WLC_E_AUTH), \
        EVT(WLC_E_DEAUTH_IND), EVT(WLC_E_DISASSOC_IND), EVT(WLC_E_PSK_SUP), EVT(-1)}

#pragma pack(1)
typedef struct {
    uint32_t version;
    uint16_t action;
    uint16_t sync_id;
             //brcmf_ssid_le
    uint32_t ssidlen;
    uint8_t  ssid[SSID_MAXLEN];
             //brcmf_scan_params_le
    uint8_t bssid[6];
    int8_t  bss_type;
    int8_t  scan_type;
    int32_t nprobes;
    int32_t active_time;
    int32_t passive_time;
    int32_t home_time;
    uint16_t nchans;
    uint16_t nssids;
    uint8_t  chans[14][2],
             ssids[1][SSID_MAXLEN];
} SCAN_PARAMS;

// Event structures
typedef struct {
    whd_event_eth_hdr_t   hdr;
    struct whd_event_msg  msg;
    uint8_t data[1];
} ETH_EVENT;

typedef struct {
    uint8_t pad[10];
    whd_event_ether_header_t eth_hdr;
    union {
        ETH_EVENT event;
        uint8_t data[1];
    };
} ETH_EVENT_FRAME;

typedef struct {
    uint8_t  seq,       // sdpcm_sw_header
             chan,
             nextlen,
             hdrlen,
             flow,
             credit,
             reserved[2];
    uint32_t cmd;       // cdc_header
    uint16_t outlen,
             inlen;
    uint32_t flags,
             status;
    uint8_t data[IOCTL_MAX_BLKLEN];
} IOCTL_CMD;

typedef struct {
    uint16_t len;
    uint8_t  reserved1,
             flags,
             reserved2[2],
             pad[2];
} IOCTL_GLOM_HDR;

typedef struct {
    IOCTL_GLOM_HDR glom_hdr;
    IOCTL_CMD  cmd;
} IOCTL_GLOM_CMD;

typedef struct
{
    uint16_t len,           // sdpcm_header.frametag
             notlen;
    union 
    {
        IOCTL_CMD cmd;
        IOCTL_GLOM_CMD glom_cmd;
    };
} IOCTL_MSG;

typedef struct {
    uint16_t len,       // sdpcm_header.frametag
             notlen;
    uint8_t  seq,       // sdpcm_sw_header
             chan,
             nextlen,
             hdrlen,
             flow,
             credit,
             reserved[2];
} IOCTL_EVENT_HDR;

// Escan result event (excluding 12-byte IOCTL header)
typedef struct {
    uint8_t pad[10];
    whd_event_t event;
    wl_escan_result_t escan;
  } escan_result;
#pragma pack()

extern char ioctl_event_hdr_fields[];
extern int txglom;

#ifdef __cplusplus
}
#endif

#endif // IOCTL_H