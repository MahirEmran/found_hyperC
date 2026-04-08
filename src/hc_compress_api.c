#include "hc_compress.h"
#include "ccsds123_internal.h"

int hc_compress(const char *input_raw, const char *output_dir) {
    if (!input_raw || !output_dir) return -1;
    if (ccsds123_ensure_dir(output_dir) != 0) return -1;
    return ccsds123_compress_one_image(input_raw, output_dir, 0, 0, 0, 0, "");
}

int hc_decompress(const char *input_bitstream, const char *output_dir) {
    if (!input_bitstream || !output_dir) return -1;
    if (ccsds123_ensure_dir(output_dir) != 0) return -1;
    return ccsds123_decompress_one_image(input_bitstream, output_dir);
}

int hc_decompress_from_buffer(const char *input_bitstream, const char *output_dir,
                              const uint8_t *input_buf, size_t input_len) {
    if (!input_bitstream || !output_dir || (!input_buf && input_len != 0)) return -1;
    if (ccsds123_ensure_dir(output_dir) != 0) return -1;
    return ccsds123_decompress_with_buffer(input_bitstream, output_dir, input_buf, input_len);
}
