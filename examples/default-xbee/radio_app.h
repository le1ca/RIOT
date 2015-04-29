#ifndef radio_app_h
#define radio_app_h

#include <stdint.h>

// width specifiers for floats
typedef float  float32_t;
typedef double float64_t;

// sample name field length
#define RA_NAME_LEN 8

// signed and unsigned ints
#define RA_FMT_u64 0x01
#define RA_FMT_i64 0x02
#define RA_FMT_u32 0x03
#define RA_FMT_i32 0x04
#define RA_FMT_u16 0x05
#define RA_FMT_i16 0x06
#define RA_FMT_u8  0x07
#define RA_FMT_i8  0x08

// floats
#define RA_FMT_f64 0x11
#define RA_FMT_f32 0x12

// strings
#define RA_FMT_s8  0x21
#define RA_FMT_s4  0x22
#define RA_FMT_s2  0x23

// helper for accessing different types of values
typedef union {
    uint64_t  u64;
    int64_t   i64;
    uint32_t  u32;
    int32_t   i32;
    uint16_t  u16;
    int16_t   i16;
    uint8_t   u8;
    int8_t    i8;
    
    float32_t f32;
    float64_t f64;
    
    char      s8[8];
    char      s4[4];
    char      s2[2];
} radio_app_val;

typedef struct __attribute__ ((__packed__)) radio_app_samp_s {
    uint8_t type;
    radio_app_val value;
} radio_app_samp;

typedef struct __attribute__ ((__packed__)) radio_app_sset_s {
    radio_app_samp samples[0];
} radio_app_sset;

typedef struct __attribute__ ((__packed__)) radio_app_pkt_s {
    uint8_t sset_ct; // sample set count
    uint8_t samp_ct; // samples per set
    char name_map[0][RA_NAME_LEN];
    radio_app_sset sets[0];
} radio_app_pkt;

#endif
