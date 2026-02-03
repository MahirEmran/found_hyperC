#ifndef CCSDS123_IO_H
#define CCSDS123_IO_H

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

int ccsds123_ensure_dir(const char *path);
int ccsds123_parse_raw_filename(const char *path, char *dtype_out, int *z_out, int *y_out, int *x_out);
int64_t ccsds123_read_sample(FILE *f, const char *dtype);
int ccsds123_load_raw_bip(const char *path, const char *dtype, int z, int y, int x, int64_t **out);
void ccsds123_build_output_folder_path(const char *output_root, const char *raw_path, int ael, char *out_dir);
int ccsds123_get_file_size(const char *path, long long *out_size);

#ifdef __cplusplus
}
#endif

#endif
