#ifndef HC_COMPRESS_H
#define HC_COMPRESS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int hc_compress(const char *input_raw, const char *output_dir);
int hc_decompress(const char *input_bitstream, const char *output_dir);
int hc_decompress_from_buffer(const char *input_bitstream, const char *output_dir,
                              const uint8_t *input_buf, size_t input_len);

#ifdef __cplusplus
}
#endif

#endif
