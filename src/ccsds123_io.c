#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "ccsds123_io.h"
#include "ccsds123_utils.h"

int ccsds123_ensure_dir(const char *path) {
    struct stat st;
    if (stat(path, &st) == 0) {
        if (S_ISDIR(st.st_mode)) return 0;
        fprintf(stderr, "Output path exists and is not a directory: %s\n", path);
        return -1;
    }
    if (mkdir(path, 0755) != 0) {
        perror("mkdir");
        return -1;
    }
    return 0;
}

int ccsds123_parse_raw_filename(const char *path, char *dtype_out, int *z_out, int *y_out, int *x_out) {
    const char *base = strrchr(path, '/');
    base = base ? base + 1 : path;

    char name[CCSDS123_MAX_PATH_LEN];
    strncpy(name, base, sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';

    char *dot = strrchr(name, '.');
    if (!dot || strcmp(dot, ".raw") != 0) return -1;
    *dot = '\0';

    char *last_dash = strrchr(name, '-');
    if (!last_dash) return -1;
    char *dims = last_dash + 1;

    int z = 0, y = 0, x = 0;
    if (sscanf(dims, "%dx%dx%d", &z, &y, &x) != 3) return -1;

    *last_dash = '\0';
    char *dtype = strrchr(name, '-');
    if (!dtype) return -1;
    dtype++;

    strncpy(dtype_out, dtype, 15);
    dtype_out[15] = '\0';
    *z_out = z;
    *y_out = y;
    *x_out = x;
    return 0;
}

int64_t ccsds123_read_sample(FILE *f, const char *dtype) {
    if (strcmp(dtype, "u8be") == 0 || strcmp(dtype, "u8le") == 0 || strcmp(dtype, "rgb8") == 0) {
        uint8_t v; if (fread(&v, 1, 1, f) != 1) return 0; return v;
    }
    if (strcmp(dtype, "s8be") == 0 || strcmp(dtype, "s8le") == 0) {
        int8_t v; if (fread(&v, 1, 1, f) != 1) return 0; return v;
    }
    if (strstr(dtype, "16") != NULL) {
        uint8_t b[2]; if (fread(b, 1, 2, f) != 2) return 0;
        int be = (dtype[3] == 'b');
        uint16_t v = be ? (uint16_t)((b[0] << 8) | b[1]) : (uint16_t)((b[1] << 8) | b[0]);
        if (dtype[0] == 's') return (int16_t)v;
        return v;
    }
    if (strstr(dtype, "32") != NULL) {
        uint8_t b[4]; if (fread(b, 1, 4, f) != 4) return 0;
        int be = (dtype[3] == 'b');
        uint32_t v = be ? ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3]
                        : ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | b[0];
        if (dtype[0] == 's') return (int32_t)v;
        return v;
    }
    if (strstr(dtype, "64") != NULL) {
        uint8_t b[8]; if (fread(b, 1, 8, f) != 8) return 0;
        int be = (dtype[3] == 'b');
        uint64_t v = 0;
        if (be) {
            for (int i = 0; i < 8; i++) v = (v << 8) | b[i];
        } else {
            for (int i = 7; i >= 0; i--) v = (v << 8) | b[i];
        }
        if (dtype[0] == 's') return (int64_t)v;
        return (int64_t)v;
    }
    return 0;
}

int ccsds123_load_raw_bip(const char *path, const char *dtype, int z, int y, int x, int64_t **out) {
    FILE *f = fopen(path, "rb");
    if (!f) return -1;

    size_t n = (size_t)x * (size_t)y * (size_t)z;
    int64_t *buf = (int64_t *)malloc(sizeof(int64_t) * n);
    if (!buf) { fclose(f); return -1; }

    /* file is BSQ: z, y, x */
    for (int zi = 0; zi < z; zi++) {
        for (int yi = 0; yi < y; yi++) {
            for (int xi = 0; xi < x; xi++) {
                int64_t v = ccsds123_read_sample(f, dtype);
                size_t idx = ccsds123_idx3(yi, xi, zi, x, z); /* convert to BIP */
                buf[idx] = v;
            }
        }
    }

    fclose(f);
    *out = buf;
    return 0;
}

void ccsds123_build_output_folder_path(const char *output_root, const char *raw_path, int ael, char *out_dir) {
    const char *base = strrchr(raw_path, '/');
    base = base ? base + 1 : raw_path;
    char stem[CCSDS123_MAX_PATH_LEN];
    strncpy(stem, base, sizeof(stem) - 1);
    stem[sizeof(stem) - 1] = '\0';
    char *dot = strrchr(stem, '.');
    if (dot) *dot = '\0';
    snprintf(out_dir, CCSDS123_MAX_PATH_LEN, "%s/%s_ael%d", output_root, stem, ael);
}

int ccsds123_get_file_size(const char *path, long long *out_size) {
    struct stat st;
    if (stat(path, &st) != 0) return -1;
    *out_size = (long long)st.st_size;
    return 0;
}
