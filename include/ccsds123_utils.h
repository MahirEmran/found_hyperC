#ifndef CCSDS123_UTILS_H
#define CCSDS123_UTILS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CCSDS123_MAX_PATH_LEN 4096

int64_t ccsds123_clip_i64(int64_t x, int64_t minv, int64_t maxv);
int64_t ccsds123_sign_i64(int64_t x);
int64_t ccsds123_sign_positive_i64(int64_t x);
int64_t ccsds123_modulo_star_i64(int64_t x, int64_t R);
int64_t ccsds123_floor_div_i64(int64_t a, int64_t b);

int64_t *ccsds123_alloc_i64(size_t n);
size_t ccsds123_idx3(int y, int x, int z, int x_size, int z_size);
size_t ccsds123_idx4(int y, int x, int z, int c, int x_size, int z_size, int c_size);

typedef struct {
    uint8_t *data;
    size_t bit_len;
    size_t cap_bits;
} BitWriter;

void ccsds123_bw_init(BitWriter *bw);
void ccsds123_bw_free(BitWriter *bw);
int ccsds123_bw_reserve_bits(BitWriter *bw, size_t extra_bits);
void ccsds123_bw_append_bit(BitWriter *bw, int bit);
void ccsds123_bw_append_bits_u64(BitWriter *bw, uint64_t value, int bits);
void ccsds123_bw_append_bits_str(BitWriter *bw, const char *bits);
void ccsds123_bw_append_from_bw(BitWriter *dst, const BitWriter *src);
void ccsds123_bw_pad_to_byte(BitWriter *bw);
int ccsds123_bw_write_to_file(BitWriter *bw, const char *path);

#ifdef __cplusplus
}
#endif

#endif
