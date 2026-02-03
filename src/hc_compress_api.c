#include "hc_compress.h"
#include "ccsds123_internal.h"

int hc_compress(const char *input_raw, const char *output_dir) {
    if (!input_raw || !output_dir) return -1;
    if (ccsds123_ensure_dir(output_dir) != 0) return -1;
    return ccsds123_compress_one_image(input_raw, output_dir, 0, 0, 0, 0, "");
}
