#include "radio_app.h"
#include <string.h>
#include <stdio.h>

uint16_t app_build_pkt(void *buffer, uint16_t b_len, uint8_t sset_ct, uint8_t samp_ct, char *names[], radio_app_sset *ssets, uint16_t sslen);
uint16_t app_append_sample(void *buf, uint16_t b_len, uint8_t type, void *value);
int8_t   app_get_type_len(uint8_t t);
void     app_decode_pkt(void *buffer, uint16_t len);

int8_t app_get_type_len(uint8_t t){

    switch(t){
        
        case RA_FMT_u64:
        case RA_FMT_i64:
        case RA_FMT_f64:
        case RA_FMT_s8:
            return 8;
        
        case RA_FMT_u32:
        case RA_FMT_i32:
        case RA_FMT_f32:
        case RA_FMT_s4:
            return 4;
            
        case RA_FMT_u16:
        case RA_FMT_i16:
        case RA_FMT_s2:
            return 2;
            
        case RA_FMT_u8:
        case RA_FMT_i8:
            return 1;
        
        default:
            return -1;
    }
    
}

uint16_t app_build_pkt(void *buffer, uint16_t b_len, uint8_t sset_ct, uint8_t samp_ct, char *names[], radio_app_sset *ssets, uint16_t sslen){
    int i;
    radio_app_pkt  *p  = buffer;
    radio_app_sset *ss = 0;
    
    // check for sufficient length
    if(b_len < sslen + (2*sizeof(uint8_t)) + (samp_ct*RA_NAME_LEN)){
        return 0;
    }
    
    // set fields
    p->sset_ct = sset_ct;
    p->samp_ct = samp_ct;
    
    // copy names
    for(i = 0; i < p->samp_ct; i++){
        uint8_t len = strlen(names[i]);
        len = (len > RA_NAME_LEN) ? RA_NAME_LEN : len;
        memcpy(p->name_map[i], names[i], len);
    }
    
    // load sample sets
    ss = (void*) buffer + (2*sizeof(uint8_t)) + (samp_ct*RA_NAME_LEN);
    memcpy(ss, ssets, sslen);
    
    // return size
    return sslen + (2*sizeof(uint8_t)) + (samp_ct*RA_NAME_LEN);
}

uint16_t app_append_sample(void *buf, uint16_t b_len, uint8_t type, void *value){
    radio_app_samp *loc = buf;
    int8_t len = app_get_type_len(type);
    
    if(len < 0)
        printf("[ra] invalid type %02x\n", type);
        
    loc->type = type;
    memcpy((void*) &loc->value, value, len);
    
    return sizeof(uint8_t) + len;
}
