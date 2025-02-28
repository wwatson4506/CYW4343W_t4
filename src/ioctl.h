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
#define MAX_CHUNK_LEN       400
#define IOCTL_MAX_BLKLEN    512
#define SSID_MAXLEN         32

#define EVENT_SET_SSID      0
#define EVENT_JOIN          1
#define EVENT_AUTH          3
#define EVENT_LINK          16
#define EVENT_MAX           208
#define SET_EVENT(msk, e)   msk[e/8] |= 1 << (e & 7)

#define DL_BEGIN			0x0002
#define DL_END				0x0004
#define DL_TYPE_CLM		    2

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

struct brcmf_dload_data_le {
	uint16_t flag;
	uint16_t dload_type;
	uint32_t len;
	uint32_t crc;
	uint8_t data[1];
};

struct brcmf_ssid_le {
	uint32_t SSID_len;
	uint8_t SSID[SSID_MAXLEN];
};

struct brcmf_scan_params_le {
    struct brcmf_ssid_le ssid_le;
    uint8_t bssid[6];
    int8_t  bss_type;
    int8_t  scan_type;
    int32_t nprobes;
    int32_t active_time;
    int32_t passive_time;
    int32_t home_time;
    uint16_t nchans;
    uint16_t nssids;
    uint16_t channel_list[1];   // channel list (not used)
    //uint8_t  chans[14][2],
    //         ssids[1][SSID_MAXLEN];
};

struct brcmf_scan_params_v2_le {
	uint16_t version;		/* structure version */
	uint16_t length;		/* structure length */
	struct brcmf_ssid_le ssid_le;
	uint8_t bssid[6];
	int8_t bss_type;
	uint8_t pad;
	uint32_t scan_type;
	int32_t nprobes;
	int32_t active_time;
    int32_t passive_time;
    int32_t home_time;
    uint16_t nchans;
    uint16_t nssids;
    uint8_t  chans[14][2],
             ssids[1][SSID_MAXLEN];
};

struct brcmf_escan_params_le {
	uint32_t version;
	uint16_t action;
	uint16_t _;
	union {
		struct brcmf_scan_params_le params_le;
		struct brcmf_scan_params_v2_le params_v2_le;
	};
};

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
    uint8_t seq,      
            chan,
            nextlen,
            hdrlen,
            flow,
            credit,
            reserved[2];
} sdpcm_sw_header;

typedef struct {
    sdpcm_sw_header sw_header;
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
    uint16_t        len,       // sdpcm_header.frametag
                    notlen;
    sdpcm_sw_header sw_header;
} sdpcm_header_t;

// Escan result event (excluding 12-byte IOCTL header)
typedef struct {
    uint8_t pad[10];
    whd_event_t event;
    wl_escan_result_t escan;
  } escan_result;
#pragma pack()

extern char ioctl_event_hdr_fields[];

#ifdef __cplusplus
}
#endif

#endif // IOCTL_H
