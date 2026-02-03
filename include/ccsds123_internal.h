#ifndef CCSDS123_INTERNAL_H
#define CCSDS123_INTERNAL_H

#include "ccsds123_utils.h"
#include "ccsds123_io.h"

#ifdef __cplusplus
extern "C" {
#endif

int ccsds123_compress_one_image(const char *raw_path, const char *output_root, int ael,
                                int override_x, int override_y, int override_z, const char *override_dtype);
int ccsds123_ends_with_raw(const char *name);

#ifdef __cplusplus
}
#endif

#endif
