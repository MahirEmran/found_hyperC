#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ccsds123_utils.h"

int64_t ccsds123_clip_i64(int64_t x, int64_t minv, int64_t maxv) {
    if (x < minv) return minv;
    if (x > maxv) return maxv;
    return x;
}

int64_t ccsds123_sign_i64(int64_t x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

int64_t ccsds123_sign_positive_i64(int64_t x) {
    return (x >= 0) ? 1 : -1;
}

int64_t ccsds123_modulo_star_i64(int64_t x, int64_t R) {
    if (R == 64) {
        uint64_t mod = (uint64_t)(x + (int64_t)(1ULL << 63));
        return (int64_t)(mod - (1ULL << 63));
    }
    int64_t mod = ((x + ((int64_t)1 << (R - 1))) % ((int64_t)1 << R));
    if (mod < 0) mod += ((int64_t)1 << R);
    return mod - ((int64_t)1 << (R - 1));
}

int64_t ccsds123_floor_div_i64(int64_t a, int64_t b) {
    int64_t q = a / b;
    int64_t r = a % b;
    if (r != 0 && ((r > 0) != (b > 0))) {
        q -= 1;
    }
    return q;
}

int64_t *ccsds123_alloc_i64(size_t n) {
    return (int64_t *)calloc(n, sizeof(int64_t));
}

size_t ccsds123_idx3(int y, int x, int z, int x_size, int z_size) {
    return ((size_t)y * (size_t)x_size + (size_t)x) * (size_t)z_size + (size_t)z;
}

size_t ccsds123_idx4(int y, int x, int z, int c, int x_size, int z_size, int c_size) {
    return (((size_t)y * (size_t)x_size + (size_t)x) * (size_t)z_size + (size_t)z) * (size_t)c_size + (size_t)c;
}

void ccsds123_bw_init(BitWriter *bw) {
    bw->data = NULL;
    bw->bit_len = 0;
    bw->cap_bits = 0;
}

void ccsds123_bw_free(BitWriter *bw) {
    free(bw->data);
    bw->data = NULL;
    bw->bit_len = 0;
    bw->cap_bits = 0;
}

int ccsds123_bw_reserve_bits(BitWriter *bw, size_t extra_bits) {
    size_t needed = bw->bit_len + extra_bits;
    if (needed <= bw->cap_bits) return 0;

    size_t new_cap_bits = bw->cap_bits ? bw->cap_bits : 1024;
    while (new_cap_bits < needed) new_cap_bits *= 2;

    size_t new_cap_bytes = (new_cap_bits + 7) / 8;
    size_t old_cap_bytes = (bw->cap_bits + 7) / 8;

    uint8_t *new_data = (uint8_t *)realloc(bw->data, new_cap_bytes);
    if (!new_data) return -1;
    if (new_cap_bytes > old_cap_bytes) {
        memset(new_data + old_cap_bytes, 0, new_cap_bytes - old_cap_bytes);
    }
    bw->data = new_data;
    bw->cap_bits = new_cap_bits;
    return 0;
}

void ccsds123_bw_append_bit(BitWriter *bw, int bit) {
    if (ccsds123_bw_reserve_bits(bw, 1) != 0) return;
    size_t byte_index = bw->bit_len / 8;
    int bit_index = 7 - (int)(bw->bit_len % 8); /* MSB-first */
    if (bit) bw->data[byte_index] |= (uint8_t)(1u << bit_index);
    bw->bit_len += 1;
}

void ccsds123_bw_append_bits_u64(BitWriter *bw, uint64_t value, int bits) {
    for (int i = bits - 1; i >= 0; i--) {
        int bit = (int)((value >> i) & 1u);
        ccsds123_bw_append_bit(bw, bit);
    }
}

void ccsds123_bw_append_bits_str(BitWriter *bw, const char *bits) {
    for (const char *p = bits; *p; p++) {
        ccsds123_bw_append_bit(bw, *p == '1');
    }
}

void ccsds123_bw_append_from_bw(BitWriter *dst, const BitWriter *src) {
    for (size_t bi = 0; bi < src->bit_len; bi++) {
        int bit = (src->data[bi / 8] >> (7 - (bi % 8))) & 1;
        ccsds123_bw_append_bit(dst, bit);
    }
}

void ccsds123_bw_pad_to_byte(BitWriter *bw) {
    size_t fill = (8 - (bw->bit_len % 8)) % 8;
    for (size_t i = 0; i < fill; i++) {
        ccsds123_bw_append_bit(bw, 0);
    }
}

int ccsds123_bw_write_to_file(BitWriter *bw, const char *path) {
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    size_t bytes = (bw->bit_len + 7) / 8;
    if (bytes > 0) {
        if (fwrite(bw->data, 1, bytes, f) != bytes) {
            fclose(f);
            return -1;
        }
    }
    fclose(f);
    return 0;
}
