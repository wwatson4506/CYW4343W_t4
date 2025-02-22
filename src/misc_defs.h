#define ALIGN_UINT(val, align) (((val) + (align) - 1) & ~((align) - 1))
#define CYW43_WRITE_BYTES_PAD(len) ALIGN_UINT((len), 64)

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define SWAP16(x) ((x&0xff)<<8 | (x&0xff00)>>8)
#define SWAP32(x) ((x&0xff)<<24 | (x&0xff00)<<8 | (x&0xff0000)>>8 | (x&0xff000000)>>24)

#define LED_PIN         10

#define USE_DEBUG_COLORS

#if defined (USE_DEBUG_COLORS)
//Foreground: reset = 0, black = 30, red = 31, green = 32, yellow = 33, blue = 34, magenta = 35, cyan = 36, and white = 37
//Background: reset = 0, black = 40, red = 41, green = 42, yellow = 43, blue = 44, magenta = 45, cyan = 46, and white = 47
#define SER_RED "\033[1;31m"
#define SER_GREEN "\033[1;32m"
#define SER_YELLOW "\033[1;33m"
#define SER_MAGENTA "\033[1;35m"
#define SER_CYAN "\033[1;36m"
#define SER_WHITE "\033[1;37m"
#define SER_RESET "\033[1;0m"

#define SER_TRACE "\033[38;2;182;222;215m"
#define SER_INFO "\033[38;2;200;200;200m"
#define SER_WARN "\033[38;2;221;230;112m"
#define SER_ERROR "\033[38;2;255;105;82m"
#define SER_USER "\033[38;2;55;255;28m"
#define SER_GREY "\033[38;2;128;128;128m"

#else
#define SER_RED ""
#define SER_GREEN ""
#define SER_YELLOW ""
#define SER_MAGENTA ""
#define SER_CYAN ""
#define SER_RESET ""

#define SER_TRACE ""
#define SER_INFO ""
#define SER_WARN ""
#define SER_ERROR ""
#define SER_USER ""
#define SER_GREY ""
#endif //USE_DEBUG_COLORS

#define COUNTRY         "US"
#define COUNTRY_REV     -1

// EOF
