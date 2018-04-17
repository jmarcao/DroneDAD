/*
Application Code
*/

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif


/** Wi-Fi AP Settings. */
#define MAIN_WLAN_SSID                       "ForTheAlliance" //"AirPennNet-Device" /**< Destination SSID */
#define MAIN_WLAN_AUTH                       M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                        "measuretape25" //"penn1740wifi" /**< Password for Destination SSID */

/** IP address parsing. */
#define IPV4_BYTE(val, index)                ((val >> (index * 8)) & 0xFF)

/** Content URI for download. */
#define URL_METADATA                   "https://www.seas.upenn.edu/~jmarcao/ddad.meta"
#define URL_LATEST_FIRMWARE            "https://www.seas.upenn.edu/~jmarcao/ddad_fw.bin"

// Buffer len for metadata
#define METADATA_VERSION_LENGTH 6
#define MAX_METADATA_BUFFER_LEN 1024
// Buffer len for firmware
#define MAX_FIRMWARE_BUFFER_LEN 4096

/** Maximum size for packet buffer. */
#define MAIN_BUFFER_MAX_SIZE                 (1446)
/** Maximum file name length. */
#define MAIN_MAX_FILE_NAME_LENGTH            (250)
/** Maximum file extension length. */
#define MAIN_MAX_FILE_EXT_LENGTH             (8)
/** Output format with '0'. */
#define MAIN_ZERO_FMT(SZ)                    (SZ == 4) ? "%04d" : (SZ == 3) ? "%03d" : (SZ == 2) ? "%02d" : "%d"

typedef enum {
	NOT_READY = 0, /*!< Not ready. */
	STORAGE_READY = 0x01, /*!< Storage is ready. */
	WIFI_CONNECTED = 0x02, /*!< Wi-Fi is connected. */
	GET_REQUESTED = 0x04, /*!< GET request is sent. */
	DOWNLOADING = 0x08, /*!< Running to download. */
	COMPLETED = 0x10, /*!< Download completed. */
	CANCELED = 0x20 /*!< Download canceled. */
} download_state;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */
