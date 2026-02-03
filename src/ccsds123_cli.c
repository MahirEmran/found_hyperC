#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ccsds123_internal.h"

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <input_raw_file> <output_dir> [ael] [--x N --y N --z N --dtype STR]\n", argv[0]);
        return 2;
    }

    const char *input_file = argv[1];
    const char *output_dir = argv[2];
    int ael = 0;
    int argi = 3;
    if (argi < argc && strncmp(argv[argi], "--", 2) != 0) {
        ael = atoi(argv[argi]);
        argi++;
    }
    if (ael < 0) {
        fprintf(stderr, "AEL must be >= 0\n");
        return 2;
    }

    int override_x = 0;
    int override_y = 0;
    int override_z = 0;
    const char *override_dtype = "";

    for (int i = argi; i < argc; i++) {
        if (strcmp(argv[i], "--x") == 0 && i + 1 < argc) {
            override_x = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--y") == 0 && i + 1 < argc) {
            override_y = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--z") == 0 && i + 1 < argc) {
            override_z = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--dtype") == 0 && i + 1 < argc) {
            override_dtype = argv[++i];
        } else {
            fprintf(stderr, "Unknown arg: %s\n", argv[i]);
            return 2;
        }
    }

    if (!ccsds123_ends_with_raw(input_file)) {
        fprintf(stderr, "Input must be a .raw file: %s\n", input_file);
        return 2;
    }

    if (ccsds123_ensure_dir(output_dir) != 0) return 1;

    printf("[ccsds123_native] %s (AEL=%d)\n", input_file, ael);
    if (ccsds123_compress_one_image(input_file, output_dir, ael, override_x, override_y, override_z, override_dtype) != 0) {
        fprintf(stderr, "Failed: %s\n", input_file);
        return 1;
    }

    char out_dir[CCSDS123_MAX_PATH_LEN];
    ccsds123_build_output_folder_path(output_dir, input_file, ael, out_dir);

    char bitstream_path[CCSDS123_MAX_PATH_LEN];
    snprintf(bitstream_path, sizeof(bitstream_path), "%s/z-output-bitstream.bin", out_dir);

    long long in_size = 0, out_size = 0;
    if (ccsds123_get_file_size(input_file, &in_size) == 0 && ccsds123_get_file_size(bitstream_path, &out_size) == 0 && out_size > 0) {
        double factor = (double)in_size / (double)out_size;
        printf("Compression factor: %.4f (input %lld bytes, output %lld bytes)\n", factor, in_size, out_size);
    } else {
        fprintf(stderr, "Warning: could not compute compression factor.\n");
    }

    printf("Done.\n");
    return 0;
}
