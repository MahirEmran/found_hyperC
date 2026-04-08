#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "ccsds123_utils.h"
#include "ccsds123_io.h"

/*
 * Native CCSDS 123.0-B-2 encoder port.
 * Single-file build to minimize includes.
 */

#define MAX_PATH_LEN CCSDS123_MAX_PATH_LEN
#ifndef CCSDS123_NO_HEAP_WORKSPACE_BYTES
#define CCSDS123_NO_HEAP_WORKSPACE_BYTES (96u * 1024u * 1024u)
#endif

typedef struct {
    uint8_t *base;
    size_t cap;
    size_t offset;
    int enabled;
} NoHeapWorkspace;

static uint8_t g_no_heap_workspace[CCSDS123_NO_HEAP_WORKSPACE_BYTES];
static NoHeapWorkspace g_workspace = {0};

static void workspace_begin_no_heap(void);
static void workspace_end_no_heap(void);
static void *workspace_alloc(size_t size);
static void *workspace_calloc(size_t count, size_t size);
static void workspace_free(void *ptr);
static int64_t *alloc_i64_local(size_t n);

static void bw_init_local(BitWriter *bw);
static void bw_free_local(BitWriter *bw);
static int bw_reserve_bits_local(BitWriter *bw, size_t extra_bits);
static void bw_append_bit_local(BitWriter *bw, int bit);
static void bw_append_bits_u64_local(BitWriter *bw, uint64_t value, int bits);
static void bw_append_bits_str_local(BitWriter *bw, const char *bits);
static void bw_append_from_bw_local(BitWriter *dst, const BitWriter *src);
static void bw_pad_to_byte_local(BitWriter *bw);
static int bw_write_to_file_local(BitWriter *bw, const char *path);

#define clip_i64 ccsds123_clip_i64
#define sign_i64 ccsds123_sign_i64
#define sign_positive_i64 ccsds123_sign_positive_i64
#define modulo_star_i64 ccsds123_modulo_star_i64
#define floor_div_i64 ccsds123_floor_div_i64

#define alloc_i64 alloc_i64_local
#define idx3 ccsds123_idx3
#define idx4 ccsds123_idx4

#define bw_init bw_init_local
#define bw_free bw_free_local
#define bw_reserve_bits bw_reserve_bits_local
#define bw_append_bit bw_append_bit_local
#define bw_append_bits_u64 bw_append_bits_u64_local
#define bw_append_bits_str bw_append_bits_str_local
#define bw_append_from_bw bw_append_from_bw_local
#define bw_pad_to_byte bw_pad_to_byte_local
#define bw_write_to_file bw_write_to_file_local

#define ensure_dir ccsds123_ensure_dir
#define parse_raw_filename ccsds123_parse_raw_filename
#define read_sample ccsds123_read_sample
#define load_raw_bip ccsds123_load_raw_bip
#define write_raw_bsq ccsds123_write_raw_bsq
#define build_output_folder_path ccsds123_build_output_folder_path
#define build_output_filename ccsds123_build_output_filename
#define build_decompressed_filename ccsds123_build_decompressed_filename
#define get_file_size ccsds123_get_file_size

static int build_out_path(const char *out_dir, const char *file_name, char *path, size_t path_len);

/* ---------------- Header and enums ---------------- */

typedef enum { SAMPLE_UNSIGNED = 0, SAMPLE_SIGNED = 1 } SampleType;
typedef enum { LARGE_D_SMALL = 0, LARGE_D_LARGE = 1 } LargeDFlag;
typedef enum { ORDER_BI = 0, ORDER_BSQ = 1 } SampleEncodingOrder;
typedef enum { ENTROPY_BA = 2 } EntropyCoderType;
typedef enum { QF_LOSSLESS = 0, QF_ABS = 1, QF_REL = 2, QF_ABS_REL = 3 } QuantizerFidelityControlMethod;
typedef enum { SR_NOT_INCLUDED = 0, SR_INCLUDED = 1 } SampleRepresentativeFlag;
typedef enum { PRED_FULL = 0, PRED_REDUCED = 1 } PredictionMode;
typedef enum { WEO_ALL_ZERO = 0, WEO_NOT_ALL_ZERO = 1 } WeightExponentOffsetFlag;
typedef enum { LS_WIDE_NEIGHBOR = 0, LS_NARROW_NEIGHBOR = 1, LS_WIDE_COLUMN = 2, LS_NARROW_COLUMN = 3 } LocalSumType;
typedef enum { WEO_TABLE_NOT_INCLUDED = 0, WEO_TABLE_INCLUDED = 1 } WeightExponentOffsetTableFlag;
typedef enum { WEIGHT_INIT_DEFAULT = 0, WEIGHT_INIT_CUSTOM = 1 } WeightInitMethod;
typedef enum { WEIGHT_INIT_TABLE_NOT_INCLUDED = 0, WEIGHT_INIT_TABLE_INCLUDED = 1 } WeightInitTableFlag;
typedef enum { PEU_NOT_USED = 0, PEU_USED = 1 } PeriodicErrorUpdatingFlag;
typedef enum { ELA_BAND_INDEPENDENT = 0, ELA_BAND_DEPENDENT = 1 } ErrorLimitAssignmentMethod;
typedef enum { BVDF_BAND_INDEPENDENT = 0, BVDF_BAND_DEPENDENT = 1 } BandVaryingDampingFlag;
typedef enum { DAMP_NOT_INCLUDED = 0, DAMP_INCLUDED = 1 } DampingTableFlag;
typedef enum { BVOF_BAND_INDEPENDENT = 0, BVOF_BAND_DEPENDENT = 1 } BandVaryingOffsetFlag;
typedef enum { OFFSET_NOT_INCLUDED = 0, OFFSET_INCLUDED = 1 } OffsetTableFlag;
typedef enum { ACCU_INIT_TABLE_NOT_INCLUDED = 0, ACCU_INIT_TABLE_INCLUDED = 1 } AccumulatorInitTableFlag;
typedef enum { RESTRICTED_UNRESTRICTED = 0, RESTRICTED_RESTRICTED = 1 } RestrictedCodeOptionsFlag;

typedef struct {
    /* Image metadata */
    int user_defined_data;
    int x_size;
    int y_size;
    int z_size;
    SampleType sample_type;
    LargeDFlag large_d_flag;
    int dynamic_range; /* encoded D%16 */
    SampleEncodingOrder sample_encoding_order;
    int sub_frame_interleaving_depth;
    int output_word_size;
    EntropyCoderType entropy_coder_type;
    QuantizerFidelityControlMethod quantizer_fidelity_control_method;

    /* Predictor metadata */
    SampleRepresentativeFlag sample_representative_flag;
    int prediction_bands_num;
    PredictionMode prediction_mode;
    WeightExponentOffsetFlag weight_exponent_offset_flag;
    LocalSumType local_sum_type;
    int register_size;
    int weight_component_resolution;
    int weight_update_change_interval;
    int weight_update_initial_parameter;
    int weight_update_final_parameter;
    WeightExponentOffsetTableFlag weight_exponent_offset_table_flag;
    WeightInitMethod weight_init_method;
    WeightInitTableFlag weight_init_table_flag;
    int weight_init_resolution;

    int weight_init_table_value;
    int64_t *weight_init_table; /* [z][c] */
    int weight_exponent_offset_value;
    int64_t *weight_exponent_offset_table; /* [z][c] */

    /* Quantization */
    PeriodicErrorUpdatingFlag periodic_error_updating_flag;
    int error_update_period_exponent;

    ErrorLimitAssignmentMethod absolute_error_limit_assignment_method;
    int absolute_error_limit_bit_depth;
    int absolute_error_limit_value;
    int64_t *absolute_error_limit_table; /* [z] */
    int64_t *periodic_absolute_error_limit_table; /* [y_period][z] */

    ErrorLimitAssignmentMethod relative_error_limit_assignment_method;
    int relative_error_limit_bit_depth;
    int relative_error_limit_value;
    int64_t *relative_error_limit_table; /* [z] */
    int64_t *periodic_relative_error_limit_table; /* [y_period][z] */

    /* Sample representative */
    int sample_representative_resolution;
    BandVaryingDampingFlag band_varying_damping_flag;
    DampingTableFlag damping_table_flag;
    int fixed_damping_value;
    BandVaryingOffsetFlag band_varying_offset_flag;
    OffsetTableFlag damping_offset_table_flag;
    int fixed_offset_value;
    int64_t *damping_table_array; /* [z] */
    int64_t *damping_offset_table_array; /* [z] */

    /* Entropy coder metadata */
    int unary_length_limit;
    int rescaling_counter_size;
    int initial_count_exponent;

    int accumulator_init_constant;
    AccumulatorInitTableFlag accumulator_init_table_flag;
    int64_t *accumulator_init_table; /* [z] */

    int block_size;
    RestrictedCodeOptionsFlag restricted_code_options_flag;
    int reference_sample_interval;

    BitWriter header_bitstream;
    BitWriter optional_tables_bitstream;
} Header;

/* ---------------- Image Constants ---------------- */

typedef struct {
    int dynamic_range_bits;
    int64_t dynamic_range;
    int64_t lower_sample_limit;
    int64_t upper_sample_limit;
    int64_t middle_sample_value;
} ImageConstants;

typedef struct {
    const uint8_t *data;
    size_t bit_len;
    size_t bit_pos;
} BitReader;

static void br_init(BitReader *br, const uint8_t *data, size_t byte_len) {
    br->data = data;
    br->bit_len = byte_len * 8;
    br->bit_pos = 0;
}

static size_t br_bits_remaining(const BitReader *br) {
    return (br->bit_pos <= br->bit_len) ? (br->bit_len - br->bit_pos) : 0;
}

static int br_read_bit(BitReader *br, int *bit_out) {
    if (br_bits_remaining(br) < 1) return -1;
    *bit_out = (br->data[br->bit_pos / 8] >> (7 - (br->bit_pos % 8))) & 1;
    br->bit_pos += 1;
    return 0;
}

static int br_read_bits_u64(BitReader *br, int bits, uint64_t *value_out) {
    if (bits < 0 || bits > 64) return -1;
    if ((size_t)bits > br_bits_remaining(br)) return -1;

    uint64_t value = 0;
    for (int i = 0; i < bits; i++) {
        int bit = 0;
        if (br_read_bit(br, &bit) != 0) return -1;
        value = (value << 1) | (uint64_t)bit;
    }
    *value_out = value;
    return 0;
}

static int br_align_to_byte(BitReader *br) {
    size_t fill = (8 - (br->bit_pos % 8)) % 8;
    if (fill > br_bits_remaining(br)) return -1;
    br->bit_pos += fill;
    return 0;
}

static int br_read_unary_zeroes(BitReader *br, int64_t *zero_count_out) {
    int64_t count = 0;
    for (;;) {
        int bit = 0;
        if (br_read_bit(br, &bit) != 0) return -1;
        if (bit != 0) break;
        count++;
    }
    *zero_count_out = count;
    return 0;
}

static int64_t decode_twos_complement_u64(uint64_t value, int bits) {
    if (bits <= 0) return 0;
    if (bits >= 64) return (int64_t)value;
    if ((value & (1ULL << (bits - 1))) != 0) {
        return (int64_t)(value - (1ULL << bits));
    }
    return (int64_t)value;
}

static size_t align_up_size(size_t value, size_t alignment) {
    size_t rem = value % alignment;
    if (rem == 0) return value;
    return value + (alignment - rem);
}

static void workspace_begin_no_heap(void) {
    g_workspace.base = g_no_heap_workspace;
    g_workspace.cap = CCSDS123_NO_HEAP_WORKSPACE_BYTES;
    g_workspace.offset = 0;
    g_workspace.enabled = 1;
}

static void workspace_end_no_heap(void) {
    g_workspace.offset = 0;
    g_workspace.enabled = 0;
}

static void *workspace_alloc(size_t size) {
    if (size == 0) size = 1;

    if (!g_workspace.enabled) {
        return malloc(size);
    }

    size_t aligned = align_up_size(g_workspace.offset, sizeof(void *));
    if (aligned > g_workspace.cap || size > (g_workspace.cap - aligned)) {
        return NULL;
    }

    void *ptr = g_workspace.base + aligned;
    g_workspace.offset = aligned + size;
    return ptr;
}

static void *workspace_calloc(size_t count, size_t size) {
    if (size != 0 && count > (SIZE_MAX / size)) return NULL;
    size_t bytes = count * size;
    void *ptr = workspace_alloc(bytes);
    if (!ptr) return NULL;
    memset(ptr, 0, bytes);
    return ptr;
}

static void workspace_free(void *ptr) {
    if (!ptr) return;

    if (!g_workspace.enabled) {
        free(ptr);
        return;
    }

    /* In no-heap mode allocations come from the static workspace and are released
       by resetting the workspace at the end of the call. */
    if (ptr < (void *)g_workspace.base || ptr >= (void *)(g_workspace.base + g_workspace.cap)) {
        free(ptr);
    }
}

static int64_t *alloc_i64_local(size_t n) {
    return (int64_t *)workspace_calloc(n, sizeof(int64_t));
}

static void bw_init_local(BitWriter *bw) {
    bw->data = NULL;
    bw->bit_len = 0;
    bw->cap_bits = 0;
}

static void bw_free_local(BitWriter *bw) {
    workspace_free(bw->data);
    bw->data = NULL;
    bw->bit_len = 0;
    bw->cap_bits = 0;
}

static int bw_reserve_bits_local(BitWriter *bw, size_t extra_bits) {
    size_t needed = bw->bit_len + extra_bits;
    if (needed <= bw->cap_bits) return 0;

    size_t new_cap_bits = bw->cap_bits ? bw->cap_bits : 1024;
    while (new_cap_bits < needed) new_cap_bits *= 2;

    size_t new_cap_bytes = (new_cap_bits + 7) / 8;
    size_t old_cap_bytes = (bw->cap_bits + 7) / 8;

    uint8_t *new_data = (uint8_t *)workspace_alloc(new_cap_bytes);
    if (!new_data) return -1;

    if (bw->data && old_cap_bytes > 0) memcpy(new_data, bw->data, old_cap_bytes);
    if (new_cap_bytes > old_cap_bytes) memset(new_data + old_cap_bytes, 0, new_cap_bytes - old_cap_bytes);

    workspace_free(bw->data);
    bw->data = new_data;
    bw->cap_bits = new_cap_bits;
    return 0;
}

static void bw_append_bit_local(BitWriter *bw, int bit) {
    if (bw_reserve_bits_local(bw, 1) != 0) return;
    size_t byte_index = bw->bit_len / 8;
    int bit_index = 7 - (int)(bw->bit_len % 8);
    if (bit) bw->data[byte_index] |= (uint8_t)(1u << bit_index);
    bw->bit_len += 1;
}

static void bw_append_bits_u64_local(BitWriter *bw, uint64_t value, int bits) {
    for (int i = bits - 1; i >= 0; i--) {
        int bit = (int)((value >> i) & 1u);
        bw_append_bit_local(bw, bit);
    }
}

static void bw_append_bits_str_local(BitWriter *bw, const char *bits) {
    for (const char *p = bits; *p; p++) {
        bw_append_bit_local(bw, *p == '1');
    }
}

static void bw_append_from_bw_local(BitWriter *dst, const BitWriter *src) {
    for (size_t bi = 0; bi < src->bit_len; bi++) {
        int bit = (src->data[bi / 8] >> (7 - (bi % 8))) & 1;
        bw_append_bit_local(dst, bit);
    }
}

static void bw_pad_to_byte_local(BitWriter *bw) {
    size_t fill = (8 - (bw->bit_len % 8)) % 8;
    for (size_t i = 0; i < fill; i++) {
        bw_append_bit_local(bw, 0);
    }
}

static int bw_write_to_file_local(BitWriter *bw, const char *path) {
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    size_t bytes = (bw->bit_len + 7) / 8;
    if (bytes > 0 && fwrite(bw->data, 1, bytes, f) != bytes) {
        fclose(f);
        return -1;
    }
    fclose(f);
    return 0;
}

/* -------------- Header helpers -------------- */

static int header_get_x_size(const Header *h) { return (h->x_size == 0) ? 65536 : h->x_size; }
static int header_get_y_size(const Header *h) { return (h->y_size == 0) ? 65536 : h->y_size; }
static int header_get_z_size(const Header *h) { return (h->z_size == 0) ? 65536 : h->z_size; }

static int header_get_dynamic_range_bits(const Header *h) {
    int bits = h->dynamic_range;
    if (bits == 0) bits = 16;
    if (h->large_d_flag == LARGE_D_LARGE) bits += 16;
    return bits;
}

static int header_get_absolute_error_limit_bit_depth_value(const Header *h) {
    return h->absolute_error_limit_bit_depth + 16 * (h->absolute_error_limit_bit_depth == 0);
}

static int header_get_relative_error_limit_bit_depth_value(const Header *h) {
    return h->relative_error_limit_bit_depth + 16 * (h->relative_error_limit_bit_depth == 0);
}

static void header_set_dynamic_range(Header *h, int dynamic_range) {
    h->large_d_flag = (dynamic_range > 16) ? LARGE_D_LARGE : LARGE_D_SMALL;
    h->dynamic_range = dynamic_range % 16;
}

static void header_init_defaults(Header *h) {
    memset(h, 0, sizeof(*h));

    h->user_defined_data = 0;
    h->x_size = 0;
    h->y_size = 0;
    h->z_size = 0;
    h->sample_type = SAMPLE_UNSIGNED;
    h->large_d_flag = LARGE_D_SMALL;
    h->dynamic_range = 0;
    h->sample_encoding_order = ORDER_BI;
    h->sub_frame_interleaving_depth = 1;
    h->output_word_size = 0;
    h->entropy_coder_type = ENTROPY_BA;
    h->quantizer_fidelity_control_method = QF_ABS_REL;

    h->sample_representative_flag = SR_INCLUDED;
    h->prediction_bands_num = 3;
    h->prediction_mode = PRED_REDUCED;
    h->weight_exponent_offset_flag = WEO_ALL_ZERO;
    h->local_sum_type = LS_NARROW_COLUMN;
    h->register_size = 0;
    h->weight_component_resolution = 15;
    h->weight_update_change_interval = 2;
    h->weight_update_initial_parameter = 5;
    h->weight_update_final_parameter = 10;
    h->weight_exponent_offset_table_flag = WEO_TABLE_NOT_INCLUDED;
    h->weight_init_method = WEIGHT_INIT_DEFAULT;
    h->weight_init_table_flag = WEIGHT_INIT_TABLE_NOT_INCLUDED;
    h->weight_init_resolution = 0;

    h->weight_init_table_value = 0;
    h->weight_init_table = NULL;
    h->weight_exponent_offset_value = 0;
    h->weight_exponent_offset_table = NULL;

    h->periodic_error_updating_flag = PEU_NOT_USED;
    h->error_update_period_exponent = 0;

    h->absolute_error_limit_assignment_method = ELA_BAND_INDEPENDENT;
    h->absolute_error_limit_bit_depth = 5;
    h->absolute_error_limit_value = 4;
    h->absolute_error_limit_table = NULL;
    h->periodic_absolute_error_limit_table = NULL;

    h->relative_error_limit_assignment_method = ELA_BAND_INDEPENDENT;
    h->relative_error_limit_bit_depth = 7;
    h->relative_error_limit_value = 16;
    h->relative_error_limit_table = NULL;
    h->periodic_relative_error_limit_table = NULL;

    h->sample_representative_resolution = 3;
    h->band_varying_damping_flag = BVDF_BAND_INDEPENDENT;
    h->damping_table_flag = DAMP_NOT_INCLUDED;
    h->fixed_damping_value = 3;
    h->band_varying_offset_flag = BVOF_BAND_INDEPENDENT;
    h->damping_offset_table_flag = OFFSET_NOT_INCLUDED;
    h->fixed_offset_value = 7;
    h->damping_table_array = NULL;
    h->damping_offset_table_array = NULL;

    h->unary_length_limit = 18;
    h->rescaling_counter_size = 2;
    h->initial_count_exponent = 1;
    h->accumulator_init_constant = 0;
    h->accumulator_init_table_flag = ACCU_INIT_TABLE_NOT_INCLUDED;
    h->accumulator_init_table = NULL;

    h->block_size = 2;
    h->restricted_code_options_flag = RESTRICTED_UNRESTRICTED;
    h->reference_sample_interval = 0;

    bw_init(&h->header_bitstream);
    bw_init(&h->optional_tables_bitstream);
}

static void header_free(Header *h) {
    if (!h) return;
    workspace_free(h->weight_init_table);
    h->weight_init_table = NULL;
    workspace_free(h->weight_exponent_offset_table);
    h->weight_exponent_offset_table = NULL;

    workspace_free(h->absolute_error_limit_table);
    h->absolute_error_limit_table = NULL;
    workspace_free(h->periodic_absolute_error_limit_table);
    h->periodic_absolute_error_limit_table = NULL;
    workspace_free(h->relative_error_limit_table);
    h->relative_error_limit_table = NULL;
    workspace_free(h->periodic_relative_error_limit_table);
    h->periodic_relative_error_limit_table = NULL;

    workspace_free(h->damping_table_array);
    h->damping_table_array = NULL;
    workspace_free(h->damping_offset_table_array);
    h->damping_offset_table_array = NULL;
    workspace_free(h->accumulator_init_table);
    h->accumulator_init_table = NULL;

    bw_free(&h->header_bitstream);
    bw_free(&h->optional_tables_bitstream);
}

static void header_init_weight_init_table(Header *h) {
    int z = header_get_z_size(h);
    int c = h->prediction_bands_num + (h->prediction_mode == PRED_FULL ? 3 : 0);
    size_t n = (size_t)z * (size_t)c;
    h->weight_init_table = alloc_i64(n);
}

static void header_init_weight_exponent_offset_table(Header *h) {
    int z = header_get_z_size(h);
    int c = h->prediction_bands_num + (h->prediction_mode == PRED_FULL ? 1 : 0);
    size_t n = (size_t)z * (size_t)c;
    h->weight_exponent_offset_table = alloc_i64(n);
}

static void header_init_absolute_error_limit_table(Header *h) {
    int z = header_get_z_size(h);
    h->absolute_error_limit_table = alloc_i64((size_t)z);
}

static void header_init_periodic_absolute_error_limit_table(Header *h) {
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);
    int period = 1 << h->error_update_period_exponent;
    int rows = (y + period - 1) / period;
    h->periodic_absolute_error_limit_table = alloc_i64((size_t)rows * (size_t)z);
}

static void header_init_periodic_relative_error_limit_table(Header *h) {
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);
    int period = 1 << h->error_update_period_exponent;
    int rows = (y + period - 1) / period;
    h->periodic_relative_error_limit_table = alloc_i64((size_t)rows * (size_t)z);
}

static void header_init_relative_error_limit_table(Header *h) {
    int z = header_get_z_size(h);
    h->relative_error_limit_table = alloc_i64((size_t)z);
}

static void header_init_damping_table(Header *h) {
    int z = header_get_z_size(h);
    h->damping_table_array = alloc_i64((size_t)z);
}

static void header_init_damping_offset_table(Header *h) {
    int z = header_get_z_size(h);
    h->damping_offset_table_array = alloc_i64((size_t)z);
}

static void header_init_accumulator_init_table(Header *h) {
    int z = header_get_z_size(h);
    h->accumulator_init_table = alloc_i64((size_t)z);
}

static void header_set_damping_table_default(Header *h);
static void header_set_damping_offset_table_default(Header *h);
static void header_set_accumulator_init_table_default(Header *h);

static void header_set_absolute_error_limit_table_default(Header *h) {
    header_init_absolute_error_limit_table(h);
    int z = header_get_z_size(h);
    int bit_depth = header_get_absolute_error_limit_bit_depth_value(h);
    for (int i = 0; i < z; i++) {
        if (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
            h->absolute_error_limit_table[i] = h->absolute_error_limit_value;
        } else {
            int64_t v = i;
            int64_t maxv = ((int64_t)1 << bit_depth) - 1;
            if (v > maxv) v = maxv;
            h->absolute_error_limit_table[i] = v;
        }
    }
}

static void header_set_relative_error_limit_table_default(Header *h) {
    header_init_relative_error_limit_table(h);
    int z = header_get_z_size(h);
    int bit_depth = header_get_relative_error_limit_bit_depth_value(h);
    int64_t maxv = ((int64_t)1 << bit_depth) - 1;
    for (int i = 0; i < z; i++) {
        if (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
            h->relative_error_limit_table[i] = h->relative_error_limit_value;
        } else {
            int64_t v = h->relative_error_limit_value + 2 * i;
            if (v > maxv) v = maxv;
            h->relative_error_limit_table[i] = v;
        }
    }
}

static void header_set_periodic_absolute_error_limit_table_default(Header *h) {
    header_init_periodic_absolute_error_limit_table(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);
    int period = 1 << h->error_update_period_exponent;
    int rows = (y + period - 1) / period;
    int bit_depth = header_get_absolute_error_limit_bit_depth_value(h);
    int64_t mod = ((int64_t)1 << bit_depth) - 1;
    for (int i = 0; i < rows; i++) {
        for (int zi = 0; zi < z; zi++) {
            if (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
                h->periodic_absolute_error_limit_table[i * z + zi] = (i * z) % mod;
            } else {
                h->periodic_absolute_error_limit_table[i * z + zi] = (i * z + zi) % mod;
            }
        }
    }
}

static void header_set_periodic_relative_error_limit_table_default(Header *h) {
    header_init_periodic_relative_error_limit_table(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);
    int period = 1 << h->error_update_period_exponent;
    int rows = (y + period - 1) / period;
    int bit_depth = header_get_relative_error_limit_bit_depth_value(h);
    int64_t mod = ((int64_t)1 << bit_depth) - 1;
    for (int i = 0; i < rows; i++) {
        for (int zi = 0; zi < z; zi++) {
            if (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
                h->periodic_relative_error_limit_table[i * z + zi] = (i * z) % mod;
            } else {
                h->periodic_relative_error_limit_table[i * z + zi] = (i * z + zi) % mod;
            }
        }
    }
}

static void header_init_tables_default(Header *h) {
    if (h->weight_init_method == WEIGHT_INIT_CUSTOM) {
        header_init_weight_init_table(h);
        int z = header_get_z_size(h);
        int c = h->prediction_bands_num + (h->prediction_mode == PRED_FULL ? 3 : 0);
        for (int zi = 0; zi < z; zi++) {
            int bands = (zi < h->prediction_bands_num) ? zi : h->prediction_bands_num;
            int limit = bands + (h->prediction_mode == PRED_FULL ? 3 : 0);
            for (int j = 0; j < limit; j++) {
                h->weight_init_table[(size_t)zi * c + j] = h->weight_init_table_value;
            }
        }
    }

    if (h->weight_exponent_offset_flag == WEO_NOT_ALL_ZERO) {
        header_init_weight_exponent_offset_table(h);
        int z = header_get_z_size(h);
        int c = h->prediction_bands_num + (h->prediction_mode == PRED_FULL ? 1 : 0);
        for (int zi = 0; zi < z; zi++) {
            int bands = (zi < h->prediction_bands_num) ? zi : h->prediction_bands_num;
            int limit = bands + (h->prediction_mode == PRED_FULL ? 1 : 0);
            for (int j = 0; j < limit; j++) {
                h->weight_exponent_offset_table[(size_t)zi * c + j] = h->weight_exponent_offset_value;
            }
        }
    }

    if (h->quantizer_fidelity_control_method == QF_ABS || h->quantizer_fidelity_control_method == QF_ABS_REL) {
        if (h->periodic_error_updating_flag == PEU_NOT_USED) {
            header_set_absolute_error_limit_table_default(h);
        } else {
            header_set_periodic_absolute_error_limit_table_default(h);
        }
    }

    if (h->quantizer_fidelity_control_method == QF_REL || h->quantizer_fidelity_control_method == QF_ABS_REL) {
        if (h->periodic_error_updating_flag == PEU_NOT_USED) {
            header_set_relative_error_limit_table_default(h);
        } else {
            header_set_periodic_relative_error_limit_table_default(h);
        }
    }

    header_set_damping_table_default(h);
    header_set_damping_offset_table_default(h);
    header_set_accumulator_init_table_default(h);
}

static void header_set_damping_table_default(Header *h) {
    header_init_damping_table(h);
    int z = header_get_z_size(h);
    if (h->band_varying_damping_flag == BVDF_BAND_INDEPENDENT) {
        for (int i = 0; i < z; i++) h->damping_table_array[i] = h->fixed_damping_value;
    } else {
        int64_t maxv = ((int64_t)1 << h->sample_representative_resolution) - 1;
        for (int i = 0; i < z; i++) h->damping_table_array[i] = i % maxv;
    }
}

static void header_set_damping_offset_table_default(Header *h) {
    header_init_damping_offset_table(h);
    int z = header_get_z_size(h);
    if (h->band_varying_offset_flag == BVOF_BAND_INDEPENDENT) {
        for (int i = 0; i < z; i++) h->damping_offset_table_array[i] = h->fixed_offset_value;
    } else {
        int64_t maxv = ((int64_t)1 << h->sample_representative_resolution) - 1;
        for (int i = 0; i < z; i++) h->damping_offset_table_array[i] = i % maxv;
    }
}

static void header_set_accumulator_init_table_default(Header *h) {
    header_init_accumulator_init_table(h);
    int z = header_get_z_size(h);
    if (h->accumulator_init_constant == 15) {
        int maxv = header_get_dynamic_range_bits(h) - 2;
        if (maxv > 14) maxv = 14;
        for (int i = 0; i < z; i++) h->accumulator_init_table[i] = i % maxv;
    } else {
        for (int i = 0; i < z; i++) h->accumulator_init_table[i] = h->accumulator_init_constant;
    }
}

/* -------------- Header encoding -------------- */

static void header_encode_essential(const Header *h, BitWriter *bw) {
    bw_append_bits_u64(bw, (uint64_t)h->user_defined_data, 8);
    bw_append_bits_u64(bw, (uint64_t)h->x_size, 16);
    bw_append_bits_u64(bw, (uint64_t)h->y_size, 16);
    bw_append_bits_u64(bw, (uint64_t)h->z_size, 16);
    bw_append_bits_u64(bw, (uint64_t)h->sample_type, 1);
    bw_append_bits_u64(bw, 0, 1);
    bw_append_bits_u64(bw, (uint64_t)h->large_d_flag, 1);
    bw_append_bits_u64(bw, (uint64_t)h->dynamic_range, 4);
    bw_append_bits_u64(bw, (uint64_t)h->sample_encoding_order, 1);
    bw_append_bits_u64(bw, (uint64_t)h->sub_frame_interleaving_depth, 16);
    bw_append_bits_u64(bw, 0, 2);
    bw_append_bits_u64(bw, (uint64_t)h->output_word_size, 3);
    bw_append_bits_u64(bw, (uint64_t)h->entropy_coder_type, 2);
    bw_append_bits_u64(bw, 0, 1);
    bw_append_bits_u64(bw, (uint64_t)h->quantizer_fidelity_control_method, 2);
    bw_append_bits_u64(bw, 0, 2);
    bw_append_bits_u64(bw, 0, 4);
}

static void header_encode_predictor_primary(const Header *h, BitWriter *header_bw, BitWriter *optional_bw) {
    bw_append_bits_u64(header_bw, 0, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->sample_representative_flag, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->prediction_bands_num, 4);
    bw_append_bits_u64(header_bw, (uint64_t)h->prediction_mode, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->weight_exponent_offset_flag, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->local_sum_type, 2);
    bw_append_bits_u64(header_bw, (uint64_t)h->register_size, 6);
    bw_append_bits_u64(header_bw, (uint64_t)h->weight_component_resolution, 4);
    bw_append_bits_u64(header_bw, (uint64_t)h->weight_update_change_interval, 4);
    bw_append_bits_u64(header_bw, (uint64_t)h->weight_update_initial_parameter, 4);
    bw_append_bits_u64(header_bw, (uint64_t)h->weight_update_final_parameter, 4);
    bw_append_bits_u64(header_bw, (uint64_t)h->weight_exponent_offset_table_flag, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->weight_init_method, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->weight_init_table_flag, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->weight_init_resolution, 5);

    if (h->weight_init_method == WEIGHT_INIT_CUSTOM && h->weight_init_table) {
        int z = header_get_z_size(h);
        int c = h->prediction_bands_num + (h->prediction_mode == PRED_FULL ? 3 : 0);
        for (int zi = 0; zi < z; zi++) {
            int bands = (zi < h->prediction_bands_num) ? zi : h->prediction_bands_num;
            int limit = bands + (h->prediction_mode == PRED_FULL ? 3 : 0);
            for (int j = 0; j < limit; j++) {
                int64_t num = h->weight_init_table[(size_t)zi * c + j];
                int64_t enc = num;
                if (num < 0) enc = num + ((int64_t)1 << h->weight_init_resolution);
                if (h->weight_init_table_flag == WEIGHT_INIT_TABLE_INCLUDED) {
                    bw_append_bits_u64(header_bw, (uint64_t)enc, h->weight_init_resolution);
                } else {
                    bw_append_bits_u64(optional_bw, (uint64_t)enc, h->weight_init_resolution);
                }
            }
        }
        if (h->weight_init_table_flag == WEIGHT_INIT_TABLE_INCLUDED) bw_pad_to_byte(header_bw);
        else bw_pad_to_byte(optional_bw);
    }

    if (h->weight_exponent_offset_flag == WEO_NOT_ALL_ZERO && h->weight_exponent_offset_table) {
        int z = header_get_z_size(h);
        int c = h->prediction_bands_num + (h->prediction_mode == PRED_FULL ? 1 : 0);
        for (int zi = 0; zi < z; zi++) {
            int bands = (zi < h->prediction_bands_num) ? zi : h->prediction_bands_num;
            int limit = bands + (h->prediction_mode == PRED_FULL ? 1 : 0);
            for (int j = 0; j < limit; j++) {
                int64_t num = h->weight_exponent_offset_table[(size_t)zi * c + j];
                int64_t enc = num;
                if (num < 0) enc = num + 16;
                if (h->weight_exponent_offset_table_flag == WEO_TABLE_INCLUDED) {
                    bw_append_bits_u64(header_bw, (uint64_t)enc, 4);
                } else {
                    bw_append_bits_u64(optional_bw, (uint64_t)enc, 4);
                }
            }
        }
        if (h->weight_exponent_offset_table_flag == WEO_TABLE_INCLUDED) bw_pad_to_byte(header_bw);
        else bw_pad_to_byte(optional_bw);
    }
}

static void header_encode_quantization_update(const Header *h, BitWriter *bw) {
    bw_append_bits_u64(bw, 0, 1);
    bw_append_bits_u64(bw, (uint64_t)h->periodic_error_updating_flag, 1);
    bw_append_bits_u64(bw, 0, 2);
    bw_append_bits_u64(bw, (uint64_t)h->error_update_period_exponent, 4);
}

static void header_encode_quantization_abs(const Header *h, BitWriter *bw) {
    bw_append_bits_u64(bw, 0, 1);
    bw_append_bits_u64(bw, (uint64_t)h->absolute_error_limit_assignment_method, 1);
    bw_append_bits_u64(bw, 0, 2);
    bw_append_bits_u64(bw, (uint64_t)h->absolute_error_limit_bit_depth, 4);
    if (h->periodic_error_updating_flag == PEU_NOT_USED) {
        int bit_depth = header_get_absolute_error_limit_bit_depth_value(h);
        if (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
            bw_append_bits_u64(bw, (uint64_t)h->absolute_error_limit_value, bit_depth);
        } else {
            int z = header_get_z_size(h);
            for (int i = 0; i < z; i++) {
                bw_append_bits_u64(bw, (uint64_t)h->absolute_error_limit_table[i], bit_depth);
            }
        }
        bw_pad_to_byte(bw);
    }
}

static void header_encode_quantization_rel(const Header *h, BitWriter *bw) {
    bw_append_bits_u64(bw, 0, 1);
    bw_append_bits_u64(bw, (uint64_t)h->relative_error_limit_assignment_method, 1);
    bw_append_bits_u64(bw, 0, 2);
    bw_append_bits_u64(bw, (uint64_t)h->relative_error_limit_bit_depth, 4);
    if (h->periodic_error_updating_flag == PEU_NOT_USED) {
        int bit_depth = header_get_relative_error_limit_bit_depth_value(h);
        if (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
            bw_append_bits_u64(bw, (uint64_t)h->relative_error_limit_value, bit_depth);
        } else {
            int z = header_get_z_size(h);
            for (int i = 0; i < z; i++) {
                bw_append_bits_u64(bw, (uint64_t)h->relative_error_limit_table[i], bit_depth);
            }
        }
        bw_pad_to_byte(bw);
    }
}

static void header_encode_sample_representative(const Header *h, BitWriter *header_bw, BitWriter *optional_bw) {
    bw_append_bits_u64(header_bw, 0, 5);
    bw_append_bits_u64(header_bw, (uint64_t)h->sample_representative_resolution, 3);
    bw_append_bits_u64(header_bw, 0, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->band_varying_damping_flag, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->damping_table_flag, 1);
    bw_append_bits_u64(header_bw, 0, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->fixed_damping_value, 4);
    bw_append_bits_u64(header_bw, 0, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->band_varying_offset_flag, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->damping_offset_table_flag, 1);
    bw_append_bits_u64(header_bw, 0, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->fixed_offset_value, 4);

    if (h->band_varying_damping_flag == BVDF_BAND_DEPENDENT) {
        BitWriter temp; bw_init(&temp);
        int z = header_get_z_size(h);
        for (int i = 0; i < z; i++) {
            bw_append_bits_u64(&temp, (uint64_t)h->damping_table_array[i], h->sample_representative_resolution);
        }
        bw_pad_to_byte(&temp);
        if (h->damping_table_flag == DAMP_INCLUDED) {
            bw_append_from_bw(header_bw, &temp);
        } else {
            bw_append_from_bw(optional_bw, &temp);
        }
        bw_free(&temp);
    }

    if (h->band_varying_offset_flag == BVOF_BAND_DEPENDENT) {
        BitWriter temp; bw_init(&temp);
        int z = header_get_z_size(h);
        for (int i = 0; i < z; i++) {
            bw_append_bits_u64(&temp, (uint64_t)h->damping_offset_table_array[i], h->sample_representative_resolution);
        }
        bw_pad_to_byte(&temp);
        if (h->damping_offset_table_flag == OFFSET_INCLUDED) {
            bw_append_from_bw(header_bw, &temp);
        } else {
            bw_append_from_bw(optional_bw, &temp);
        }
        bw_free(&temp);
    }
}

static void header_encode_entropy(const Header *h, BitWriter *header_bw, BitWriter *optional_bw) {
    (void)optional_bw;
    bw_append_bits_u64(header_bw, 0, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->block_size, 2);
    bw_append_bits_u64(header_bw, (uint64_t)h->restricted_code_options_flag, 1);
    bw_append_bits_u64(header_bw, (uint64_t)h->reference_sample_interval, 12);
}

static void header_build_bitstreams(Header *h) {
    bw_free(&h->header_bitstream);
    bw_free(&h->optional_tables_bitstream);
    bw_init(&h->header_bitstream);
    bw_init(&h->optional_tables_bitstream);

    header_encode_essential(h, &h->header_bitstream);

    /* Supplementary tables omitted when count is 0 */

    header_encode_predictor_primary(h, &h->header_bitstream, &h->optional_tables_bitstream);

    if (h->quantizer_fidelity_control_method != QF_LOSSLESS) {
        if (h->sample_encoding_order != ORDER_BSQ) {
            header_encode_quantization_update(h, &h->header_bitstream);
        }
        if (h->quantizer_fidelity_control_method != QF_REL) {
            header_encode_quantization_abs(h, &h->header_bitstream);
        }
        if (h->quantizer_fidelity_control_method != QF_ABS) {
            header_encode_quantization_rel(h, &h->header_bitstream);
        }
    }

    if (h->sample_representative_flag == SR_INCLUDED) {
        header_encode_sample_representative(h, &h->header_bitstream, &h->optional_tables_bitstream);
    }

    header_encode_entropy(h, &h->header_bitstream, &h->optional_tables_bitstream);
}

static int header_decode_essential(Header *h, BitReader *br) {
    uint64_t value = 0;

    if (br_read_bits_u64(br, 8, &value) != 0) return -1;
    h->user_defined_data = (int)value;
    if (br_read_bits_u64(br, 16, &value) != 0) return -1;
    h->x_size = (int)value;
    if (br_read_bits_u64(br, 16, &value) != 0) return -1;
    h->y_size = (int)value;
    if (br_read_bits_u64(br, 16, &value) != 0) return -1;
    h->z_size = (int)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->sample_type = (SampleType)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->large_d_flag = (LargeDFlag)value;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->dynamic_range = (int)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->sample_encoding_order = (SampleEncodingOrder)value;
    if (br_read_bits_u64(br, 16, &value) != 0) return -1;
    h->sub_frame_interleaving_depth = (int)value;
    if (br_read_bits_u64(br, 2, &value) != 0) return -1;
    if (br_read_bits_u64(br, 3, &value) != 0) return -1;
    h->output_word_size = (int)value;
    if (br_read_bits_u64(br, 2, &value) != 0) return -1;
    h->entropy_coder_type = (EntropyCoderType)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 2, &value) != 0) return -1;
    h->quantizer_fidelity_control_method = (QuantizerFidelityControlMethod)value;
    if (br_read_bits_u64(br, 2, &value) != 0) return -1;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;

    if (h->entropy_coder_type != ENTROPY_BA) {
        fprintf(stderr, "Only block-adaptive entropy coding is supported.\n");
        return -1;
    }

    return 0;
}

static int header_decode_predictor_primary(Header *h, BitReader *br) {
    uint64_t value = 0;

    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->sample_representative_flag = (SampleRepresentativeFlag)value;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->prediction_bands_num = (int)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->prediction_mode = (PredictionMode)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->weight_exponent_offset_flag = (WeightExponentOffsetFlag)value;
    if (br_read_bits_u64(br, 2, &value) != 0) return -1;
    h->local_sum_type = (LocalSumType)value;
    if (br_read_bits_u64(br, 6, &value) != 0) return -1;
    h->register_size = (int)value;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->weight_component_resolution = (int)value;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->weight_update_change_interval = (int)value;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->weight_update_initial_parameter = (int)value;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->weight_update_final_parameter = (int)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->weight_exponent_offset_table_flag = (WeightExponentOffsetTableFlag)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->weight_init_method = (WeightInitMethod)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->weight_init_table_flag = (WeightInitTableFlag)value;
    if (br_read_bits_u64(br, 5, &value) != 0) return -1;
    h->weight_init_resolution = (int)value;

    if (h->weight_init_method == WEIGHT_INIT_CUSTOM) {
        if (h->weight_init_table_flag != WEIGHT_INIT_TABLE_INCLUDED) {
            fprintf(stderr, "External weight-init tables are not supported.\n");
            return -1;
        }
        header_init_weight_init_table(h);
        if (!h->weight_init_table) return -1;

        int z = header_get_z_size(h);
        int c = h->prediction_bands_num + (h->prediction_mode == PRED_FULL ? 3 : 0);
        for (int zi = 0; zi < z; zi++) {
            int bands = (zi < h->prediction_bands_num) ? zi : h->prediction_bands_num;
            int limit = bands + (h->prediction_mode == PRED_FULL ? 3 : 0);
            for (int j = 0; j < limit; j++) {
                if (br_read_bits_u64(br, h->weight_init_resolution, &value) != 0) return -1;
                h->weight_init_table[(size_t)zi * c + j] = decode_twos_complement_u64(value, h->weight_init_resolution);
            }
        }
        if (br_align_to_byte(br) != 0) return -1;
    }

    if (h->weight_exponent_offset_flag == WEO_NOT_ALL_ZERO) {
        if (h->weight_exponent_offset_table_flag != WEO_TABLE_INCLUDED) {
            fprintf(stderr, "External weight exponent offset tables are not supported.\n");
            return -1;
        }
        header_init_weight_exponent_offset_table(h);
        if (!h->weight_exponent_offset_table) return -1;

        int z = header_get_z_size(h);
        int c = h->prediction_bands_num + (h->prediction_mode == PRED_FULL ? 1 : 0);
        for (int zi = 0; zi < z; zi++) {
            int bands = (zi < h->prediction_bands_num) ? zi : h->prediction_bands_num;
            int limit = bands + (h->prediction_mode == PRED_FULL ? 1 : 0);
            for (int j = 0; j < limit; j++) {
                if (br_read_bits_u64(br, 4, &value) != 0) return -1;
                h->weight_exponent_offset_table[(size_t)zi * c + j] = decode_twos_complement_u64(value, 4);
            }
        }
        if (br_align_to_byte(br) != 0) return -1;
    }

    return 0;
}

static int header_decode_quantization_update(Header *h, BitReader *br) {
    uint64_t value = 0;

    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->periodic_error_updating_flag = (PeriodicErrorUpdatingFlag)value;
    if (br_read_bits_u64(br, 2, &value) != 0) return -1;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->error_update_period_exponent = (int)value;
    return 0;
}

static int header_decode_quantization_abs(Header *h, BitReader *br) {
    uint64_t value = 0;

    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->absolute_error_limit_assignment_method = (ErrorLimitAssignmentMethod)value;
    if (br_read_bits_u64(br, 2, &value) != 0) return -1;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->absolute_error_limit_bit_depth = (int)value;

    if (h->periodic_error_updating_flag == PEU_NOT_USED) {
        int bit_depth = header_get_absolute_error_limit_bit_depth_value(h);
        if (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
            if (br_read_bits_u64(br, bit_depth, &value) != 0) return -1;
            h->absolute_error_limit_value = (int)value;
        } else {
            int z = header_get_z_size(h);
            header_init_absolute_error_limit_table(h);
            if (!h->absolute_error_limit_table) return -1;
            for (int zi = 0; zi < z; zi++) {
                if (br_read_bits_u64(br, bit_depth, &value) != 0) return -1;
                h->absolute_error_limit_table[zi] = (int64_t)value;
            }
        }
        if (br_align_to_byte(br) != 0) return -1;
    }

    return 0;
}

static int header_decode_quantization_rel(Header *h, BitReader *br) {
    uint64_t value = 0;

    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->relative_error_limit_assignment_method = (ErrorLimitAssignmentMethod)value;
    if (br_read_bits_u64(br, 2, &value) != 0) return -1;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->relative_error_limit_bit_depth = (int)value;

    if (h->periodic_error_updating_flag == PEU_NOT_USED) {
        int bit_depth = header_get_relative_error_limit_bit_depth_value(h);
        if (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
            if (br_read_bits_u64(br, bit_depth, &value) != 0) return -1;
            h->relative_error_limit_value = (int)value;
        } else {
            int z = header_get_z_size(h);
            header_init_relative_error_limit_table(h);
            if (!h->relative_error_limit_table) return -1;
            for (int zi = 0; zi < z; zi++) {
                if (br_read_bits_u64(br, bit_depth, &value) != 0) return -1;
                h->relative_error_limit_table[zi] = (int64_t)value;
            }
        }
        if (br_align_to_byte(br) != 0) return -1;
    }

    return 0;
}

static int header_decode_sample_representative(Header *h, BitReader *br) {
    uint64_t value = 0;

    if (br_read_bits_u64(br, 5, &value) != 0) return -1;
    if (br_read_bits_u64(br, 3, &value) != 0) return -1;
    h->sample_representative_resolution = (int)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->band_varying_damping_flag = (BandVaryingDampingFlag)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->damping_table_flag = (DampingTableFlag)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->fixed_damping_value = (int)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->band_varying_offset_flag = (BandVaryingOffsetFlag)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->damping_offset_table_flag = (OffsetTableFlag)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 4, &value) != 0) return -1;
    h->fixed_offset_value = (int)value;

    if (h->band_varying_damping_flag == BVDF_BAND_DEPENDENT) {
        if (h->damping_table_flag != DAMP_INCLUDED) {
            fprintf(stderr, "External damping tables are not supported.\n");
            return -1;
        }
        header_init_damping_table(h);
        if (!h->damping_table_array) return -1;
        for (int zi = 0; zi < header_get_z_size(h); zi++) {
            if (br_read_bits_u64(br, h->sample_representative_resolution, &value) != 0) return -1;
            h->damping_table_array[zi] = (int64_t)value;
        }
        if (br_align_to_byte(br) != 0) return -1;
    }

    if (h->band_varying_offset_flag == BVOF_BAND_DEPENDENT) {
        if (h->damping_offset_table_flag != OFFSET_INCLUDED) {
            fprintf(stderr, "External damping offset tables are not supported.\n");
            return -1;
        }
        header_init_damping_offset_table(h);
        if (!h->damping_offset_table_array) return -1;
        for (int zi = 0; zi < header_get_z_size(h); zi++) {
            if (br_read_bits_u64(br, h->sample_representative_resolution, &value) != 0) return -1;
            h->damping_offset_table_array[zi] = (int64_t)value;
        }
        if (br_align_to_byte(br) != 0) return -1;
    }

    return 0;
}

static int header_decode_entropy(Header *h, BitReader *br) {
    uint64_t value = 0;

    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    if (br_read_bits_u64(br, 2, &value) != 0) return -1;
    h->block_size = (int)value;
    if (br_read_bits_u64(br, 1, &value) != 0) return -1;
    h->restricted_code_options_flag = (RestrictedCodeOptionsFlag)value;
    if (br_read_bits_u64(br, 12, &value) != 0) return -1;
    h->reference_sample_interval = (int)value;
    return 0;
}

static int header_prepare_runtime_tables(Header *h) {
    if (h->weight_init_method == WEIGHT_INIT_CUSTOM && !h->weight_init_table) {
        fprintf(stderr, "Missing weight initialization table.\n");
        return -1;
    }

    if (h->weight_exponent_offset_flag == WEO_NOT_ALL_ZERO && !h->weight_exponent_offset_table) {
        fprintf(stderr, "Missing weight exponent offset table.\n");
        return -1;
    }

    if (h->quantizer_fidelity_control_method == QF_ABS || h->quantizer_fidelity_control_method == QF_ABS_REL) {
        if (h->periodic_error_updating_flag == PEU_NOT_USED) {
            if (!h->absolute_error_limit_table) header_set_absolute_error_limit_table_default(h);
        } else if (!h->periodic_absolute_error_limit_table) {
            header_init_periodic_absolute_error_limit_table(h);
        }
    }

    if (h->quantizer_fidelity_control_method == QF_REL || h->quantizer_fidelity_control_method == QF_ABS_REL) {
        if (h->periodic_error_updating_flag == PEU_NOT_USED) {
            if (!h->relative_error_limit_table) header_set_relative_error_limit_table_default(h);
        } else if (!h->periodic_relative_error_limit_table) {
            header_init_periodic_relative_error_limit_table(h);
        }
    }

    if (!h->damping_table_array) {
        if (h->band_varying_damping_flag == BVDF_BAND_INDEPENDENT) header_set_damping_table_default(h);
        else {
            fprintf(stderr, "Missing damping table.\n");
            return -1;
        }
    }

    if (!h->damping_offset_table_array) {
        if (h->band_varying_offset_flag == BVOF_BAND_INDEPENDENT) header_set_damping_offset_table_default(h);
        else {
            fprintf(stderr, "Missing damping offset table.\n");
            return -1;
        }
    }

    if (!h->accumulator_init_table) header_set_accumulator_init_table_default(h);
    return 0;
}

static int header_parse_from_bitstream(Header *h, BitReader *br) {
    if (header_decode_essential(h, br) != 0) return -1;
    if (header_decode_predictor_primary(h, br) != 0) return -1;

    if (h->quantizer_fidelity_control_method != QF_LOSSLESS) {
        if (h->sample_encoding_order != ORDER_BSQ) {
            if (header_decode_quantization_update(h, br) != 0) return -1;
        }
        if (h->quantizer_fidelity_control_method != QF_REL) {
            if (header_decode_quantization_abs(h, br) != 0) return -1;
        }
        if (h->quantizer_fidelity_control_method != QF_ABS) {
            if (header_decode_quantization_rel(h, br) != 0) return -1;
        }
    }

    if (h->sample_representative_flag == SR_INCLUDED) {
        if (header_decode_sample_representative(h, br) != 0) return -1;
    }

    if (header_decode_entropy(h, br) != 0) return -1;
    return header_prepare_runtime_tables(h);
}


/* ---------------- Predictor ---------------- */

typedef struct {
    Header *header;
    ImageConstants *image_constants;
    int64_t *image_sample; /* [y][x][z] */

    int local_difference_values_num;
    int64_t *spectral_bands_used; /* [z] */

    int weight_component_resolution;
    int weight_update_change_interval;
    int weight_update_initial_parameter;
    int weight_update_final_parameter;
    int64_t *weight_update_scaling_exponent; /* [t] */
    double *weight_exponent_offset; /* [z][c] */
    int64_t weight_min;
    int64_t weight_max;
    int register_size;

    int64_t *absolute_error_limits; /* [y][z] */
    int64_t *relative_error_limits; /* [y][z] */

    /* arrays */
    int64_t *local_sum; /* [y][x][z] */
    int64_t *local_difference_vector; /* [y][x][z][c] */
    int64_t *weight_vector; /* [y][x][z][c] */
    int64_t *predicted_central_local_difference; /* [y][x][z] */
    int64_t *high_resolution_predicted_sample_value; /* [y][x][z] */
    int64_t *double_resolution_predicted_sample_value; /* [y][x][z] */
    int64_t *predicted_sample_value; /* [y][x][z] */
    int64_t *prediction_residual; /* [y][x][z] */
    int64_t *maximum_error; /* [y][x][z] */
    int64_t *quantizer_index; /* [y][x][z] */
    int64_t *clipped_quantizer_bin_center; /* [y][x][z] */
    int64_t *double_resolution_sample_representative; /* [y][x][z] */
    int64_t *sample_representative; /* [y][x][z] */
    int64_t *double_resolution_prediction_error; /* [y][x][z] */
    int64_t *scaled_prediction_endpoint_difference; /* [y][x][z] */
    int64_t *mapped_quantizer_index; /* [y][x][z] */
} Predictor;

static void predictor_free(Predictor *p) {
    if (!p) return;
    workspace_free(p->spectral_bands_used);
    p->spectral_bands_used = NULL;
    workspace_free(p->weight_update_scaling_exponent);
    p->weight_update_scaling_exponent = NULL;
    workspace_free(p->weight_exponent_offset);
    p->weight_exponent_offset = NULL;

    workspace_free(p->absolute_error_limits);
    p->absolute_error_limits = NULL;
    workspace_free(p->relative_error_limits);
    p->relative_error_limits = NULL;

    workspace_free(p->local_sum);
    p->local_sum = NULL;
    workspace_free(p->local_difference_vector);
    p->local_difference_vector = NULL;
    workspace_free(p->weight_vector);
    p->weight_vector = NULL;
    workspace_free(p->predicted_central_local_difference);
    p->predicted_central_local_difference = NULL;
    workspace_free(p->high_resolution_predicted_sample_value);
    p->high_resolution_predicted_sample_value = NULL;
    workspace_free(p->double_resolution_predicted_sample_value);
    p->double_resolution_predicted_sample_value = NULL;
    workspace_free(p->predicted_sample_value);
    p->predicted_sample_value = NULL;
    workspace_free(p->prediction_residual);
    p->prediction_residual = NULL;
    workspace_free(p->maximum_error);
    p->maximum_error = NULL;
    workspace_free(p->quantizer_index);
    p->quantizer_index = NULL;
    workspace_free(p->clipped_quantizer_bin_center);
    p->clipped_quantizer_bin_center = NULL;
    workspace_free(p->double_resolution_sample_representative);
    p->double_resolution_sample_representative = NULL;
    workspace_free(p->sample_representative);
    p->sample_representative = NULL;
    workspace_free(p->double_resolution_prediction_error);
    p->double_resolution_prediction_error = NULL;
    workspace_free(p->scaled_prediction_endpoint_difference);
    p->scaled_prediction_endpoint_difference = NULL;
    workspace_free(p->mapped_quantizer_index);
    p->mapped_quantizer_index = NULL;
}

static void predictor_init(Predictor *p, Header *h, ImageConstants *ic, int64_t *image_sample) {
    memset(p, 0, sizeof(*p));
    p->header = h;
    p->image_constants = ic;
    p->image_sample = image_sample;
}

static int predictor_init_constants(Predictor *p) {
    Header *h = p->header;
    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);

    p->local_difference_values_num = h->prediction_bands_num + (h->prediction_mode == PRED_FULL ? 3 : 0);

    p->spectral_bands_used = alloc_i64(z);
    if (!p->spectral_bands_used) return -1;
    for (int zi = 0; zi < z; zi++) {
        p->spectral_bands_used[zi] = (zi < h->prediction_bands_num) ? zi : h->prediction_bands_num;
    }

    p->weight_component_resolution = h->weight_component_resolution + 4;
    p->weight_update_change_interval = 1 << (h->weight_update_change_interval + 4);
    p->weight_update_initial_parameter = h->weight_update_initial_parameter - 6;
    p->weight_update_final_parameter = h->weight_update_final_parameter - 6;

    p->weight_update_scaling_exponent = alloc_i64((size_t)x * (size_t)y);
    if (!p->weight_update_scaling_exponent) return -1;
    for (int t = 0; t < x * y; t++) {
        int64_t base = p->weight_update_initial_parameter + floor_div_i64((int64_t)(t - x), p->weight_update_change_interval);
        base = clip_i64(base, p->weight_update_initial_parameter, p->weight_update_final_parameter);
        base += p->image_constants->dynamic_range_bits - p->weight_component_resolution;
        p->weight_update_scaling_exponent[t] = base;
    }

    if (p->local_difference_values_num > 0) {
        size_t n = (size_t)z * (size_t)p->local_difference_values_num;
        p->weight_exponent_offset = (double *)workspace_calloc(n, sizeof(double));
        if (!p->weight_exponent_offset) return -1;
        if (h->weight_exponent_offset_flag == WEO_NOT_ALL_ZERO && h->weight_exponent_offset_table) {
            if (h->prediction_mode == PRED_FULL) {
                for (int zi = 0; zi < z; zi++) {
                    for (int i = 0; i < 3; i++) {
                        p->weight_exponent_offset[(size_t)zi * p->local_difference_values_num + i] = (double)h->weight_exponent_offset_table[(size_t)zi * (h->prediction_bands_num + 1)];
                    }
                    for (int i = 3; i < p->local_difference_values_num; i++) {
                        p->weight_exponent_offset[(size_t)zi * p->local_difference_values_num + i] = (double)h->weight_exponent_offset_table[(size_t)zi * (h->prediction_bands_num + 1) + (i - 2)];
                    }
                }
            } else {
                for (int zi = 0; zi < z; zi++) {
                    for (int i = 0; i < p->local_difference_values_num; i++) {
                        p->weight_exponent_offset[(size_t)zi * p->local_difference_values_num + i] = (double)h->weight_exponent_offset_table[(size_t)zi * (h->prediction_bands_num) + i];
                    }
                }
            }
        }
    }

    p->weight_min = -((int64_t)1 << (p->weight_component_resolution + 2));
    p->weight_max = ((int64_t)1 << (p->weight_component_resolution + 2)) - 1;
    p->register_size = (h->register_size == 0) ? 64 : h->register_size;

    p->absolute_error_limits = alloc_i64((size_t)y * (size_t)z);
    p->relative_error_limits = alloc_i64((size_t)y * (size_t)z);
    if (!p->absolute_error_limits || !p->relative_error_limits) return -1;

    for (int yi = 0; yi < y; yi++) {
        for (int zi = 0; zi < z; zi++) {
            p->absolute_error_limits[yi * z + zi] = -1;
            p->relative_error_limits[yi * z + zi] = -1;
        }
    }

    if (h->periodic_error_updating_flag == PEU_NOT_USED && h->quantizer_fidelity_control_method != QF_LOSSLESS) {
        if (h->quantizer_fidelity_control_method != QF_REL) {
            for (int yi = 0; yi < y; yi++) {
                for (int zi = 0; zi < z; zi++) {
                    p->absolute_error_limits[yi * z + zi] = h->absolute_error_limit_table[zi];
                }
            }
        }
        if (h->quantizer_fidelity_control_method != QF_ABS) {
            for (int yi = 0; yi < y; yi++) {
                for (int zi = 0; zi < z; zi++) {
                    p->relative_error_limits[yi * z + zi] = h->relative_error_limit_table[zi];
                }
            }
        }
    } else if (h->periodic_error_updating_flag == PEU_USED) {
        int period = 1 << h->error_update_period_exponent;
        for (int yi = 0; yi < y; yi++) {
            int i = yi / period;
            if (h->quantizer_fidelity_control_method != QF_REL) {
                for (int zi = 0; zi < z; zi++) {
                    p->absolute_error_limits[yi * z + zi] = h->periodic_absolute_error_limit_table[i * z + zi];
                }
            }
            if (h->quantizer_fidelity_control_method != QF_ABS) {
                for (int zi = 0; zi < z; zi++) {
                    p->relative_error_limits[yi * z + zi] = h->periodic_relative_error_limit_table[i * z + zi];
                }
            }
        }
    }

    return 0;
}

static int predictor_init_arrays(Predictor *p) {
    Header *h = p->header;
    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);
    int c = p->local_difference_values_num;

    size_t n3 = (size_t)x * (size_t)y * (size_t)z;
    size_t n4 = n3 * (size_t)c;

    p->local_sum = alloc_i64(n3);
    p->local_difference_vector = alloc_i64(n4);
    p->weight_vector = alloc_i64(n4);
    p->predicted_central_local_difference = alloc_i64(n3);
    p->high_resolution_predicted_sample_value = alloc_i64(n3);
    p->double_resolution_predicted_sample_value = alloc_i64(n3);
    p->predicted_sample_value = alloc_i64(n3);
    p->prediction_residual = alloc_i64(n3);
    p->maximum_error = alloc_i64(n3);
    p->quantizer_index = alloc_i64(n3);
    p->clipped_quantizer_bin_center = alloc_i64(n3);
    p->double_resolution_sample_representative = alloc_i64(n3);
    p->sample_representative = alloc_i64(n3);
    p->double_resolution_prediction_error = alloc_i64(n3);
    p->scaled_prediction_endpoint_difference = alloc_i64(n3);
    if (!p->mapped_quantizer_index) {
        p->mapped_quantizer_index = alloc_i64(n3);
    }

    if (!p->local_sum || !p->local_difference_vector || !p->weight_vector ||
        !p->predicted_central_local_difference || !p->high_resolution_predicted_sample_value ||
        !p->double_resolution_predicted_sample_value || !p->predicted_sample_value ||
        !p->prediction_residual || !p->maximum_error || !p->quantizer_index ||
        !p->clipped_quantizer_bin_center || !p->double_resolution_sample_representative ||
        !p->sample_representative || !p->double_resolution_prediction_error ||
        !p->scaled_prediction_endpoint_difference || !p->mapped_quantizer_index) {
        return -1;
    }

    return 0;
}

static void predictor_calculate_local_sum(Predictor *p, int x, int y, int z, int t) {
    if (t == 0) return;
    Header *h = p->header;
    int x_size = header_get_x_size(h);

    size_t idx = idx3(y, x, z, x_size, header_get_z_size(h));
    if (h->local_sum_type == LS_WIDE_NEIGHBOR) {
        if (y > 0 && x > 0 && x < x_size - 1) {
            p->local_sum[idx] =
                p->sample_representative[idx3(y, x - 1, z, x_size, header_get_z_size(h))] +
                p->sample_representative[idx3(y - 1, x - 1, z, x_size, header_get_z_size(h))] +
                p->sample_representative[idx3(y - 1, x, z, x_size, header_get_z_size(h))] +
                p->sample_representative[idx3(y - 1, x + 1, z, x_size, header_get_z_size(h))];
        } else if (y == 0 && x > 0) {
            p->local_sum[idx] = p->sample_representative[idx3(y, x - 1, z, x_size, header_get_z_size(h))] * 4;
        } else if (y > 0 && x == 0) {
            p->local_sum[idx] =
                (p->sample_representative[idx3(y - 1, x, z, x_size, header_get_z_size(h))] +
                 p->sample_representative[idx3(y - 1, x + 1, z, x_size, header_get_z_size(h))]) * 2;
        } else if (y > 0 && x == x_size - 1) {
            p->local_sum[idx] =
                p->sample_representative[idx3(y, x - 1, z, x_size, header_get_z_size(h))] +
                p->sample_representative[idx3(y - 1, x - 1, z, x_size, header_get_z_size(h))] +
                p->sample_representative[idx3(y - 1, x, z, x_size, header_get_z_size(h))] * 2;
        }
    } else if (h->local_sum_type == LS_NARROW_NEIGHBOR) {
        if (y > 0 && x > 0 && x < x_size - 1) {
            p->local_sum[idx] =
                p->sample_representative[idx3(y - 1, x - 1, z, x_size, header_get_z_size(h))] +
                p->sample_representative[idx3(y - 1, x, z, x_size, header_get_z_size(h))] * 2 +
                p->sample_representative[idx3(y - 1, x + 1, z, x_size, header_get_z_size(h))];
        } else if (y == 0 && x > 0 && z > 0) {
            p->local_sum[idx] = p->sample_representative[idx3(y, x - 1, z - 1, x_size, header_get_z_size(h))] * 4;
        } else if (y > 0 && x == 0) {
            p->local_sum[idx] =
                (p->sample_representative[idx3(y - 1, x, z, x_size, header_get_z_size(h))] +
                 p->sample_representative[idx3(y - 1, x + 1, z, x_size, header_get_z_size(h))]) * 2;
        } else if (y > 0 && x == x_size - 1) {
            p->local_sum[idx] =
                (p->sample_representative[idx3(y - 1, x - 1, z, x_size, header_get_z_size(h))] +
                 p->sample_representative[idx3(y - 1, x, z, x_size, header_get_z_size(h))]) * 2;
        } else if (y == 0 && x > 0 && z == 0) {
            p->local_sum[idx] = p->image_constants->middle_sample_value * 4;
        }
    } else if (h->local_sum_type == LS_WIDE_COLUMN) {
        if (y > 0) {
            p->local_sum[idx] = p->sample_representative[idx3(y - 1, x, z, x_size, header_get_z_size(h))] * 4;
        } else if (y == 0 && x > 0) {
            p->local_sum[idx] = p->sample_representative[idx3(y, x - 1, z, x_size, header_get_z_size(h))] * 4;
        }
    } else if (h->local_sum_type == LS_NARROW_COLUMN) {
        if (y > 0) {
            p->local_sum[idx] = p->sample_representative[idx3(y - 1, x, z, x_size, header_get_z_size(h))] * 4;
        } else if (y == 0 && x > 0 && z > 0) {
            p->local_sum[idx] = p->sample_representative[idx3(y, x - 1, z - 1, x_size, header_get_z_size(h))] * 4;
        } else if (y == 0 && x > 0 && z == 0) {
            p->local_sum[idx] = p->image_constants->middle_sample_value * 4;
        }
    }
}

static void predictor_calculate_local_difference_vector(Predictor *p, int x, int y, int z, int t) {
    if (t == 0) return;
    Header *h = p->header;
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);
    int offset = 0;

    if (h->prediction_mode == PRED_FULL) {
        if (x > 0 && y > 0) {
            p->local_difference_vector[idx4(y, x, z, 0, x_size, z_size, p->local_difference_values_num)] =
                4 * p->sample_representative[idx3(y - 1, x, z, x_size, z_size)] - p->local_sum[idx3(y, x, z, x_size, z_size)];
            p->local_difference_vector[idx4(y, x, z, 1, x_size, z_size, p->local_difference_values_num)] =
                4 * p->sample_representative[idx3(y, x - 1, z, x_size, z_size)] - p->local_sum[idx3(y, x, z, x_size, z_size)];
            p->local_difference_vector[idx4(y, x, z, 2, x_size, z_size, p->local_difference_values_num)] =
                4 * p->sample_representative[idx3(y - 1, x - 1, z, x_size, z_size)] - p->local_sum[idx3(y, x, z, x_size, z_size)];
        } else if (x == 0 && y > 0) {
            for (int i = 0; i < 3; i++) {
                p->local_difference_vector[idx4(y, x, z, i, x_size, z_size, p->local_difference_values_num)] =
                    4 * p->sample_representative[idx3(y - 1, x, z, x_size, z_size)] - p->local_sum[idx3(y, x, z, x_size, z_size)];
            }
        } else {
            for (int i = 0; i < 3; i++) {
                p->local_difference_vector[idx4(y, x, z, i, x_size, z_size, p->local_difference_values_num)] = 0;
            }
        }
        offset += 3;
    }

    if (z > 0 && p->spectral_bands_used[z] > 0) {
        p->local_difference_vector[idx4(y, x, z, offset, x_size, z_size, p->local_difference_values_num)] =
            4 * p->sample_representative[idx3(y, x, z - 1, x_size, z_size)] -
            p->local_sum[idx3(y, x, z - 1, x_size, z_size)];
        for (int i = 1; i < p->spectral_bands_used[z]; i++) {
            p->local_difference_vector[idx4(y, x, z, offset + i, x_size, z_size, p->local_difference_values_num)] =
                p->local_difference_vector[idx4(y, x, z - 1, offset + i - 1, x_size, z_size, p->local_difference_values_num)];
        }
    }
}

static void predictor_init_weights(Predictor *p, int x, int y, int z) {
    Header *h = p->header;
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);

    if (h->weight_init_method == WEIGHT_INIT_DEFAULT) {
        int offset = 0;
        if (h->prediction_mode == PRED_FULL) {
            for (int i = 0; i < 3; i++) {
                p->weight_vector[idx4(y, x, z, i, x_size, z_size, p->local_difference_values_num)] = 0;
            }
            offset += 3;
        }
        if (z > 0 && p->spectral_bands_used[z] > 0) {
            p->weight_vector[idx4(y, x, z, offset, x_size, z_size, p->local_difference_values_num)] =
                ((int64_t)1 << p->weight_component_resolution) * 7 / 8;
            for (int i = 1; i < p->spectral_bands_used[z]; i++) {
                p->weight_vector[idx4(y, x, z, offset + i, x_size, z_size, p->local_difference_values_num)] =
                    p->weight_vector[idx4(y, x, z, offset + i - 1, x_size, z_size, p->local_difference_values_num)] / 8;
            }
        }
    } else {
        int c = p->local_difference_values_num;
        int w_res = h->weight_init_resolution;
        for (int i = 0; i < c; i++) {
            int64_t val = h->weight_init_table[(size_t)z * c + i];
            p->weight_vector[idx4(y, x, z, i, x_size, z_size, p->local_difference_values_num)] =
                ((int64_t)1 << (p->weight_component_resolution + 3 - w_res)) * val +
                (int64_t)ceil((double)((1LL << (p->weight_component_resolution + 2 - w_res)) - 1));
        }
    }
}

static void predictor_calculate_weight_vector(Predictor *p, int x, int y, int z, int t) {
    if (t == 0) return;
    if (t == 1) {
        predictor_init_weights(p, x, y, z);
        return;
    }

    Header *h = p->header;
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);

    int prev_y = y;
    int prev_x = x - 1;
    if (prev_x < 0) {
        prev_y -= 1;
        prev_x = x_size - 1;
    }

    for (int i = 0; i < p->local_difference_values_num; i++) {
        double term = (double)sign_positive_i64(p->double_resolution_prediction_error[idx3(prev_y, prev_x, z, x_size, z_size)]) *
            (double)p->local_difference_vector[idx4(prev_y, prev_x, z, i, x_size, z_size, p->local_difference_values_num)] *
            pow(2.0, -(double)(p->weight_update_scaling_exponent[t - 1] + p->weight_exponent_offset[(size_t)z * p->local_difference_values_num + i]));
        int64_t delta = floor_div_i64((int64_t)floor(term) + 1, 2);
        int64_t prev = p->weight_vector[idx4(prev_y, prev_x, z, i, x_size, z_size, p->local_difference_values_num)];
        p->weight_vector[idx4(y, x, z, i, x_size, z_size, p->local_difference_values_num)] =
            clip_i64(prev + delta, p->weight_min, p->weight_max);
    }
}

static void predictor_calculate_predicted_central_local_difference(Predictor *p, int x, int y, int z, int t) {
    if (t == 0) return;
    Header *h = p->header;
    if (h->prediction_mode == PRED_REDUCED && z == 0) {
        p->predicted_central_local_difference[idx3(y, x, z, header_get_x_size(h), header_get_z_size(h))] = 0;
        return;
    }
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);
    int64_t sum = 0;
    for (int i = 0; i < p->local_difference_values_num; i++) {
        sum += p->weight_vector[idx4(y, x, z, i, x_size, z_size, p->local_difference_values_num)] *
            p->local_difference_vector[idx4(y, x, z, i, x_size, z_size, p->local_difference_values_num)];
    }
    p->predicted_central_local_difference[idx3(y, x, z, x_size, z_size)] = sum;
}

static void predictor_calculate_prediction(Predictor *p, int x, int y, int z, int t) {
    Header *h = p->header;
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);
    size_t idx = idx3(y, x, z, x_size, z_size);

    if (t > 0) {
        int64_t tmp = p->predicted_central_local_difference[idx] +
            ((int64_t)1 << p->weight_component_resolution) *
            (p->local_sum[idx] - 4 * p->image_constants->middle_sample_value);
        tmp = modulo_star_i64(tmp, p->register_size);
        tmp += ((int64_t)1 << (p->weight_component_resolution + 2)) * p->image_constants->middle_sample_value +
               ((int64_t)1 << (p->weight_component_resolution + 1));
        int64_t minv = ((int64_t)1 << (p->weight_component_resolution + 2)) * p->image_constants->lower_sample_limit;
        int64_t maxv = ((int64_t)1 << (p->weight_component_resolution + 2)) * p->image_constants->upper_sample_limit +
                       ((int64_t)1 << (p->weight_component_resolution + 1));
        p->high_resolution_predicted_sample_value[idx] = clip_i64(tmp, minv, maxv);
        p->double_resolution_predicted_sample_value[idx] = p->high_resolution_predicted_sample_value[idx] >> (p->weight_component_resolution + 1);
    } else if (t == 0 && h->prediction_bands_num > 0 && z > 0) {
        p->double_resolution_predicted_sample_value[idx] = 2 * p->image_sample[idx3(y, x, z - 1, x_size, z_size)];
    } else {
        p->double_resolution_predicted_sample_value[idx] = 2 * p->image_constants->middle_sample_value;
    }

    p->predicted_sample_value[idx] = p->double_resolution_predicted_sample_value[idx] / 2;
}

static void predictor_calculate_maximum_error(Predictor *p, int x, int y, int z) {
    Header *h = p->header;
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);
    size_t idx = idx3(y, x, z, x_size, z_size);

    if (h->quantizer_fidelity_control_method == QF_LOSSLESS) {
        p->maximum_error[idx] = 0;
    } else if (h->quantizer_fidelity_control_method == QF_ABS) {
        p->maximum_error[idx] = p->absolute_error_limits[y * z_size + z];
    } else if (h->quantizer_fidelity_control_method == QF_REL) {
        p->maximum_error[idx] = (int64_t)floor((double)p->relative_error_limits[y * z_size + z] *
            (double)p->predicted_sample_value[idx] / (double)p->image_constants->dynamic_range);
    } else {
        int64_t abs_e = p->absolute_error_limits[y * z_size + z];
        int64_t rel_e = (int64_t)floor((double)p->relative_error_limits[y * z_size + z] *
            (double)p->predicted_sample_value[idx] / (double)p->image_constants->dynamic_range);
        p->maximum_error[idx] = (abs_e < rel_e) ? abs_e : rel_e;
    }
}

static void predictor_calculate_quantization(Predictor *p, int x, int y, int z, int t) {
    int x_size = header_get_x_size(p->header);
    int z_size = header_get_z_size(p->header);
    size_t idx = idx3(y, x, z, x_size, z_size);

    p->prediction_residual[idx] = p->image_sample[idx] - p->predicted_sample_value[idx];

    if (t == 0) {
        p->quantizer_index[idx] = p->prediction_residual[idx];
        return;
    }

    int64_t numerator = llabs(p->prediction_residual[idx]) + p->maximum_error[idx];
    int64_t denom = 2 * p->maximum_error[idx] + 1;
    int64_t q = numerator / denom;
    p->quantizer_index[idx] = sign_i64(p->prediction_residual[idx]) * q;
}

static void predictor_calculate_sample_representative(Predictor *p, int x, int y, int z, int t) {
    Header *h = p->header;
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);
    size_t idx = idx3(y, x, z, x_size, z_size);

    if (t == 0) {
        p->sample_representative[idx] = p->image_sample[idx];
        return;
    }

    if (p->maximum_error[idx] == 0) {
        p->clipped_quantizer_bin_center[idx] = p->image_sample[idx];
    } else {
        int64_t val = p->predicted_sample_value[idx] +
            p->quantizer_index[idx] * (2 * p->maximum_error[idx] + 1);
        p->clipped_quantizer_bin_center[idx] = clip_i64(val, p->image_constants->lower_sample_limit, p->image_constants->upper_sample_limit);
    }

    if (h->damping_table_array[z] == 0 && h->damping_offset_table_array[z] == 0) {
        p->double_resolution_sample_representative[idx] = 2 * p->clipped_quantizer_bin_center[idx];
        p->sample_representative[idx] = p->clipped_quantizer_bin_center[idx];
    } else {
        int64_t term1 = 4 * (((int64_t)1 << h->sample_representative_resolution) - h->damping_table_array[z]);
        int64_t term2 = p->clipped_quantizer_bin_center[idx] * ((int64_t)1 << p->weight_component_resolution) -
            sign_i64(p->quantizer_index[idx]) * p->maximum_error[idx] * h->damping_offset_table_array[z] *
            ((int64_t)1 << (p->weight_component_resolution - h->sample_representative_resolution));
        int64_t term3 = h->damping_table_array[z] * p->high_resolution_predicted_sample_value[idx] -
            h->damping_table_array[z] * ((int64_t)1 << (p->weight_component_resolution + 1));
        int64_t denom = (int64_t)1 << (p->weight_component_resolution + h->sample_representative_resolution + 1);
        p->double_resolution_sample_representative[idx] = floor_div_i64(term1 * term2 + term3, denom);
        p->sample_representative[idx] = (p->double_resolution_sample_representative[idx] + 1) / 2;
    }
}

static void predictor_calculate_prediction_error(Predictor *p, int x, int y, int z) {
    int x_size = header_get_x_size(p->header);
    int z_size = header_get_z_size(p->header);
    size_t idx = idx3(y, x, z, x_size, z_size);

    p->double_resolution_prediction_error[idx] =
        2 * p->clipped_quantizer_bin_center[idx] - p->double_resolution_predicted_sample_value[idx];
}

static void predictor_calculate_mapped_quantizer_index(Predictor *p, int x, int y, int z, int t) {
    Header *h = p->header;
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);
    size_t idx = idx3(y, x, z, x_size, z_size);

    if (t == 0) {
        int64_t v1 = p->predicted_sample_value[idx3(0, 0, z, x_size, z_size)] - p->image_constants->lower_sample_limit;
        int64_t v2 = p->image_constants->upper_sample_limit - p->predicted_sample_value[idx3(0, 0, z, x_size, z_size)];
        p->scaled_prediction_endpoint_difference[idx] = (v1 < v2) ? v1 : v2;
    } else {
        int64_t v1 = (p->predicted_sample_value[idx] - p->image_constants->lower_sample_limit + p->maximum_error[idx]) /
            (2 * p->maximum_error[idx] + 1);
        int64_t v2 = (p->image_constants->upper_sample_limit - p->predicted_sample_value[idx] + p->maximum_error[idx]) /
            (2 * p->maximum_error[idx] + 1);
        p->scaled_prediction_endpoint_difference[idx] = (v1 < v2) ? v1 : v2;
    }

    int64_t term = ((p->double_resolution_predicted_sample_value[idx] % 2) == 0 ? 1 : -1) * p->quantizer_index[idx];

    if (llabs(p->quantizer_index[idx]) > p->scaled_prediction_endpoint_difference[idx]) {
        p->mapped_quantizer_index[idx] = llabs(p->quantizer_index[idx]) + p->scaled_prediction_endpoint_difference[idx];
    } else if (0 <= term && term <= p->scaled_prediction_endpoint_difference[idx]) {
        p->mapped_quantizer_index[idx] = 2 * llabs(p->quantizer_index[idx]);
    } else {
        p->mapped_quantizer_index[idx] = 2 * llabs(p->quantizer_index[idx]) - 1;
    }
}

static int predictor_run(Predictor *p) {
    if (predictor_init_constants(p) != 0) {
        predictor_free(p);
        return -1;
    }
    if (predictor_init_arrays(p) != 0) {
        predictor_free(p);
        return -1;
    }

    Header *h = p->header;
    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);

    for (int yi = 0; yi < y; yi++) {
        for (int xi = 0; xi < x; xi++) {
            int t = xi + yi * x;
            for (int zi = 0; zi < z; zi++) {
                predictor_calculate_local_sum(p, xi, yi, zi, t);
                predictor_calculate_local_difference_vector(p, xi, yi, zi, t);
                predictor_calculate_weight_vector(p, xi, yi, zi, t);
                predictor_calculate_predicted_central_local_difference(p, xi, yi, zi, t);
                predictor_calculate_prediction(p, xi, yi, zi, t);
                predictor_calculate_maximum_error(p, xi, yi, zi);
                predictor_calculate_quantization(p, xi, yi, zi, t);
                predictor_calculate_sample_representative(p, xi, yi, zi, t);
                predictor_calculate_prediction_error(p, xi, yi, zi);
                predictor_calculate_mapped_quantizer_index(p, xi, yi, zi, t);
            }
        }
    }

    return 0;
}

static void predictor_compute_endpoint_differences(Predictor *p, int x, int y, int z, int t,
                                                   int64_t *lower_diff_out, int64_t *upper_diff_out) {
    int x_size = header_get_x_size(p->header);
    int z_size = header_get_z_size(p->header);
    size_t idx = idx3(y, x, z, x_size, z_size);

    int64_t lower_diff = 0;
    int64_t upper_diff = 0;
    if (t == 0) {
        lower_diff = p->predicted_sample_value[idx] - p->image_constants->lower_sample_limit;
        upper_diff = p->image_constants->upper_sample_limit - p->predicted_sample_value[idx];
    } else {
        int64_t denom = 2 * p->maximum_error[idx] + 1;
        lower_diff = (p->predicted_sample_value[idx] - p->image_constants->lower_sample_limit + p->maximum_error[idx]) / denom;
        upper_diff = (p->image_constants->upper_sample_limit - p->predicted_sample_value[idx] + p->maximum_error[idx]) / denom;
    }

    *lower_diff_out = lower_diff;
    *upper_diff_out = upper_diff;
}

static int predictor_reconstruct_sample(Predictor *p, int x, int y, int z, int t) {
    int x_size = header_get_x_size(p->header);
    int z_size = header_get_z_size(p->header);
    size_t idx = idx3(y, x, z, x_size, z_size);
    int64_t mapped = p->mapped_quantizer_index[idx];
    int64_t lower_diff = 0;
    int64_t upper_diff = 0;

    predictor_compute_endpoint_differences(p, x, y, z, t, &lower_diff, &upper_diff);
    p->scaled_prediction_endpoint_difference[idx] = (lower_diff < upper_diff) ? lower_diff : upper_diff;

    int64_t s = p->scaled_prediction_endpoint_difference[idx];
    int64_t q = 0;
    int parity_sign = ((p->double_resolution_predicted_sample_value[idx] % 2) == 0) ? 1 : -1;

    if (mapped > 2 * s) {
        int64_t magnitude = mapped - s;
        if (upper_diff > lower_diff) q = magnitude;
        else if (lower_diff > upper_diff) q = -magnitude;
        else return -1;
    } else if ((mapped & 1) == 0) {
        q = (int64_t)parity_sign * (mapped / 2);
    } else {
        q = -(int64_t)parity_sign * ((mapped + 1) / 2);
    }

    p->quantizer_index[idx] = q;

    int64_t reconstructed = 0;
    if (t == 0 || p->maximum_error[idx] == 0) {
        reconstructed = p->predicted_sample_value[idx] + q;
    } else {
        reconstructed = p->predicted_sample_value[idx] + q * (2 * p->maximum_error[idx] + 1);
    }
    reconstructed = clip_i64(reconstructed, p->image_constants->lower_sample_limit, p->image_constants->upper_sample_limit);

    p->prediction_residual[idx] = reconstructed - p->predicted_sample_value[idx];
    p->clipped_quantizer_bin_center[idx] = reconstructed;
    p->image_sample[idx] = reconstructed;
    return 0;
}

static int predictor_inverse_run(Predictor *p) {
    if (predictor_init_constants(p) != 0) {
        predictor_free(p);
        return -1;
    }
    if (predictor_init_arrays(p) != 0) {
        predictor_free(p);
        return -1;
    }

    Header *h = p->header;
    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);

    for (int yi = 0; yi < y; yi++) {
        for (int xi = 0; xi < x; xi++) {
            int t = xi + yi * x;
            for (int zi = 0; zi < z; zi++) {
                predictor_calculate_local_sum(p, xi, yi, zi, t);
                predictor_calculate_local_difference_vector(p, xi, yi, zi, t);
                predictor_calculate_weight_vector(p, xi, yi, zi, t);
                predictor_calculate_predicted_central_local_difference(p, xi, yi, zi, t);
                predictor_calculate_prediction(p, xi, yi, zi, t);
                predictor_calculate_maximum_error(p, xi, yi, zi);
                if (predictor_reconstruct_sample(p, xi, yi, zi, t) != 0) {
                    predictor_free(p);
                    return -1;
                }
                predictor_calculate_sample_representative(p, xi, yi, zi, t);
                predictor_calculate_prediction_error(p, xi, yi, zi);
            }
        }
    }

    return 0;
}

/* ---------------- Encoders ---------------- */

/* Block-adaptive encoder */

typedef struct {
    Header *header;
    ImageConstants *image_constants;
    int64_t *mapped_quantizer_index;

    int block_size;
    int reference_sample_interval;
    int id_bits;
    int segment_size;
    int max_sample_split_bits;
    int periodic_error_update_values_num;

    int64_t *blocks; /* [num_blocks][block_size] flattened */
    size_t blocks_count;
    int64_t *zero_block_count;

    BitWriter bitstream;
} BlockAdaptiveEncoder;

static void ba_free(BlockAdaptiveEncoder *enc) {
    if (!enc) return;
    workspace_free(enc->blocks);
    enc->blocks = NULL;
    workspace_free(enc->zero_block_count);
    enc->zero_block_count = NULL;
    bw_free(&enc->bitstream);
}

static void ba_init(BlockAdaptiveEncoder *enc, Header *h, ImageConstants *ic, int64_t *mqi) {
    memset(enc, 0, sizeof(*enc));
    enc->header = h;
    enc->image_constants = ic;
    enc->mapped_quantizer_index = mqi;
    enc->segment_size = 64;
    bw_init(&enc->bitstream);
}

static int ba_init_constants(BlockAdaptiveEncoder *enc) {
    Header *h = enc->header;
    int block_sizes[4] = {8, 16, 32, 64};
    enc->block_size = block_sizes[h->block_size];
    enc->reference_sample_interval = h->reference_sample_interval + ((h->reference_sample_interval == 0) ? (1 << 12) : 0);

    int id_bits_lower = (h->restricted_code_options_flag == RESTRICTED_RESTRICTED) ? 1 : 3;
    enc->id_bits = (int)fmax(ceil(log2((double)enc->image_constants->dynamic_range_bits)), id_bits_lower);

    if (h->restricted_code_options_flag == RESTRICTED_RESTRICTED) {
        enc->max_sample_split_bits = (enc->image_constants->dynamic_range_bits <= 2) ? -1 : 1;
    } else {
        if (enc->image_constants->dynamic_range_bits <= 8) enc->max_sample_split_bits = 5;
        else if (enc->image_constants->dynamic_range_bits <= 16) enc->max_sample_split_bits = 13;
        else enc->max_sample_split_bits = 29;
    }

    enc->periodic_error_update_values_num = 0;
    if (h->periodic_error_updating_flag == PEU_USED) {
        int updates = (header_get_y_size(h) + (1 << h->error_update_period_exponent) - 1) / (1 << h->error_update_period_exponent);
        int errors_per_update = 0;
        if (h->quantizer_fidelity_control_method != QF_ABS) {
            errors_per_update += (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) ? 1 : header_get_z_size(h);
        }
        if (h->quantizer_fidelity_control_method != QF_REL) {
            errors_per_update += (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) ? 1 : header_get_z_size(h);
        }
        enc->periodic_error_update_values_num = updates * errors_per_update;
    }

    return 0;
}

static int ba_build_blocks(BlockAdaptiveEncoder *enc) {
    Header *h = enc->header;
    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);

    size_t values_to_encode = (size_t)x * (size_t)y * (size_t)z + (size_t)enc->periodic_error_update_values_num;
    enc->blocks_count = (values_to_encode + enc->block_size - 1) / enc->block_size;
    enc->blocks = alloc_i64(enc->blocks_count * enc->block_size);
    enc->zero_block_count = alloc_i64(enc->blocks_count);
    if (!enc->blocks || !enc->zero_block_count) return -1;

    size_t index = 0;
    if (h->sample_encoding_order == ORDER_BI) {
        for (int yi = 0; yi < y; yi++) {
            if (yi % (1 << h->error_update_period_exponent) == 0 && h->periodic_error_updating_flag == PEU_USED) {
                int period_index = yi / (1 << h->error_update_period_exponent);
                if (h->quantizer_fidelity_control_method != QF_REL) {
                    if (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
                        enc->blocks[index++] = h->periodic_absolute_error_limit_table[period_index * z];
                    } else {
                        for (int zi = 0; zi < z; zi++) enc->blocks[index++] = h->periodic_absolute_error_limit_table[period_index * z + zi];
                    }
                }
                if (h->quantizer_fidelity_control_method != QF_ABS) {
                    if (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
                        enc->blocks[index++] = h->periodic_relative_error_limit_table[period_index * z];
                    } else {
                        for (int zi = 0; zi < z; zi++) enc->blocks[index++] = h->periodic_relative_error_limit_table[period_index * z + zi];
                    }
                }
            }

            for (int i = 0; i < (z + h->sub_frame_interleaving_depth - 1) / h->sub_frame_interleaving_depth; i++) {
                for (int xi = 0; xi < x; xi++) {
                    int z_start = i * h->sub_frame_interleaving_depth;
                    int z_end = z_start + h->sub_frame_interleaving_depth;
                    if (z_end > z) z_end = z;
                    for (int zi = z_start; zi < z_end; zi++) {
                        enc->blocks[index++] = enc->mapped_quantizer_index[idx3(yi, xi, zi, x, z)];
                    }
                }
            }
        }
    } else {
        for (int zi = 0; zi < z; zi++) {
            for (int yi = 0; yi < y; yi++) {
                for (int xi = 0; xi < x; xi++) {
                    enc->blocks[index++] = enc->mapped_quantizer_index[idx3(yi, xi, zi, x, z)];
                }
            }
        }
        /* pad remaining with zeros */
        while (index < enc->blocks_count * (size_t)enc->block_size) {
            enc->blocks[index++] = 0;
        }
    }

    return 0;
}

static int ba_encode_no_compression(BlockAdaptiveEncoder *enc, int block_index, BitWriter *out) {
    bw_init(out);
    for (int i = 0; i < enc->id_bits; i++) bw_append_bit(out, 1);
    int dyn_bits = enc->image_constants->dynamic_range_bits;
    size_t base = (size_t)block_index * enc->block_size;
    for (int i = 0; i < enc->block_size; i++) {
        bw_append_bits_u64(out, (uint64_t)enc->blocks[base + i], dyn_bits);
    }
    return 0;
}

static int ba_encode_second_extension(BlockAdaptiveEncoder *enc, int block_index, BitWriter *out) {
    bw_init(out);
    for (int i = 0; i < enc->id_bits; i++) bw_append_bit(out, 0);
    bw_append_bit(out, 1);
    int dyn_bits = enc->image_constants->dynamic_range_bits;
    size_t base = (size_t)block_index * enc->block_size;
    int limit = enc->block_size * dyn_bits;
    for (int i = 0; i < enc->block_size; i += 2) {
        int64_t d0 = enc->blocks[base + i];
        int64_t d1 = enc->blocks[base + i + 1];
        int64_t transformed = (d0 + d1) * (d0 + d1 + 1) / 2 + d1;
        if (transformed >= limit) {
            bw_free(out);
            bw_init(out);
            for (int k = 0; k < enc->id_bits + 1 + (enc->block_size + 1) * dyn_bits; k++) bw_append_bit(out, 0);
            return 0;
        }
        for (int64_t z = 0; z < transformed; z++) bw_append_bit(out, 0);
        bw_append_bit(out, 1);
    }
    return 0;
}

static int ba_encode_sample_splitting(BlockAdaptiveEncoder *enc, int block_index, int k, BitWriter *out) {
    bw_init(out);
    int dyn_bits = enc->image_constants->dynamic_range_bits;
    int limit = enc->block_size * dyn_bits;
    bw_append_bits_u64(out, (uint64_t)(k + 1), enc->id_bits);
    size_t base = (size_t)block_index * enc->block_size;

    size_t fs_len = 0;
    for (int i = 0; i < enc->block_size; i++) {
        int64_t val = enc->blocks[base + i];
        int64_t zeroes = (k == 0) ? val : (val >> k);
        if (zeroes > limit || (int)fs_len > limit) {
            bw_free(out);
            bw_init(out);
            for (int t = 0; t < 2 * (enc->block_size + 1) * dyn_bits; t++) bw_append_bit(out, 0);
            return 0;
        }
        for (int64_t z = 0; z < zeroes; z++) bw_append_bit(out, 0);
        bw_append_bit(out, 1);
        fs_len += (size_t)zeroes + 1;
    }

    if (k > 0) {
        for (int i = 0; i < enc->block_size; i++) {
            uint64_t low = (uint64_t)enc->blocks[base + i] & ((1ULL << k) - 1);
            bw_append_bits_u64(out, low, k);
        }
    }
    return 0;
}

static void ba_append_bitwriter(BitWriter *dst, BitWriter *src) {
    bw_append_from_bw(dst, src);
}

static void ba_encode_block(BlockAdaptiveEncoder *enc, int block_index) {
    int start_of_segment = ((block_index % enc->reference_sample_interval) % enc->segment_size) == 0;
    int zero_block = 1;
    size_t base = (size_t)block_index * enc->block_size;
    for (int i = 0; i < enc->block_size; i++) {
        if (enc->blocks[base + i] != 0) { zero_block = 0; break; }
    }

    if (zero_block) {
        if (start_of_segment) {
            enc->zero_block_count[block_index] = 1;
            if (block_index == 0) return;
        } else {
            enc->zero_block_count[block_index] = enc->zero_block_count[block_index - 1] + 1;
            return;
        }
    }

    if (block_index > 0 && enc->zero_block_count[block_index - 1] > 0) {
        int64_t count = enc->zero_block_count[block_index - 1];
        for (int i = 0; i < enc->id_bits + 1; i++) bw_append_bit(&enc->bitstream, 0);
        if (count <= 4) {
            for (int i = 0; i < count - 1; i++) bw_append_bit(&enc->bitstream, 0);
            bw_append_bit(&enc->bitstream, 1);
        } else if (start_of_segment) {
            bw_append_bits_str(&enc->bitstream, "00001");
        } else {
            for (int i = 0; i < count; i++) bw_append_bit(&enc->bitstream, 0);
            bw_append_bit(&enc->bitstream, 1);
        }
    }
    if (start_of_segment && zero_block) return;

    BitWriter best; bw_init(&best);
    BitWriter cand; bw_init(&cand);
    size_t best_len = (size_t)-1;

    ba_encode_no_compression(enc, block_index, &cand);
    if (cand.bit_len < best_len) { bw_free(&best); best = cand; best_len = cand.bit_len; } else { bw_free(&cand); }

    ba_encode_second_extension(enc, block_index, &cand);
    if (cand.bit_len < best_len) { bw_free(&best); best = cand; best_len = cand.bit_len; } else { bw_free(&cand); }

    for (int k = 0; k <= enc->max_sample_split_bits; k++) {
        ba_encode_sample_splitting(enc, block_index, k, &cand);
        if (cand.bit_len < best_len) { bw_free(&best); best = cand; best_len = cand.bit_len; } else { bw_free(&cand); }
    }

    ba_append_bitwriter(&enc->bitstream, &best);
    bw_free(&best);
}

static int ba_run(BlockAdaptiveEncoder *enc) {
    if (ba_init_constants(enc) != 0) return -1;
    if (ba_build_blocks(enc) != 0) return -1;

    for (size_t i = 0; i < enc->blocks_count; i++) {
        ba_encode_block(enc, (int)i);
    }

    if (enc->blocks_count > 0 && enc->zero_block_count[enc->blocks_count - 1] > 0) {
        int64_t count = enc->zero_block_count[enc->blocks_count - 1];
        for (int i = 0; i < enc->id_bits + 1; i++) bw_append_bit(&enc->bitstream, 0);
        if (count <= 4) {
            for (int i = 0; i < count - 1; i++) bw_append_bit(&enc->bitstream, 0);
            bw_append_bit(&enc->bitstream, 1);
        } else {
            bw_append_bits_str(&enc->bitstream, "00001");
        }
    }

    return 0;
}

typedef struct {
    Header *header;
    ImageConstants *image_constants;
    int block_size;
    int reference_sample_interval;
    int id_bits;
    int segment_size;
    int max_sample_split_bits;
    int periodic_error_update_values_num;

    int64_t *blocks;
    size_t blocks_count;
} BlockAdaptiveDecoder;

static void bad_free(BlockAdaptiveDecoder *dec) {
    if (!dec) return;
    workspace_free(dec->blocks);
    dec->blocks = NULL;
}

static void bad_init(BlockAdaptiveDecoder *dec, Header *h, ImageConstants *ic) {
    memset(dec, 0, sizeof(*dec));
    dec->header = h;
    dec->image_constants = ic;
    dec->segment_size = 64;
}

static int bad_init_constants(BlockAdaptiveDecoder *dec) {
    Header *h = dec->header;
    int block_sizes[4] = {8, 16, 32, 64};
    if (h->block_size < 0 || h->block_size > 3) return -1;

    dec->block_size = block_sizes[h->block_size];
    dec->reference_sample_interval = h->reference_sample_interval + ((h->reference_sample_interval == 0) ? (1 << 12) : 0);

    int id_bits_lower = (h->restricted_code_options_flag == RESTRICTED_RESTRICTED) ? 1 : 3;
    dec->id_bits = (int)fmax(ceil(log2((double)dec->image_constants->dynamic_range_bits)), id_bits_lower);

    if (h->restricted_code_options_flag == RESTRICTED_RESTRICTED) {
        dec->max_sample_split_bits = (dec->image_constants->dynamic_range_bits <= 2) ? -1 : 1;
    } else {
        if (dec->image_constants->dynamic_range_bits <= 8) dec->max_sample_split_bits = 5;
        else if (dec->image_constants->dynamic_range_bits <= 16) dec->max_sample_split_bits = 13;
        else dec->max_sample_split_bits = 29;
    }

    dec->periodic_error_update_values_num = 0;
    if (h->periodic_error_updating_flag == PEU_USED) {
        int updates = (header_get_y_size(h) + (1 << h->error_update_period_exponent) - 1) / (1 << h->error_update_period_exponent);
        int errors_per_update = 0;
        if (h->quantizer_fidelity_control_method != QF_ABS) {
            errors_per_update += (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) ? 1 : header_get_z_size(h);
        }
        if (h->quantizer_fidelity_control_method != QF_REL) {
            errors_per_update += (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) ? 1 : header_get_z_size(h);
        }
        dec->periodic_error_update_values_num = updates * errors_per_update;
    }

    return 0;
}

static int bad_alloc_blocks(BlockAdaptiveDecoder *dec) {
    Header *h = dec->header;
    size_t values_to_decode = (size_t)header_get_x_size(h) * (size_t)header_get_y_size(h) * (size_t)header_get_z_size(h)
                            + (size_t)dec->periodic_error_update_values_num;
    dec->blocks_count = (values_to_decode + dec->block_size - 1) / dec->block_size;
    dec->blocks = alloc_i64(dec->blocks_count * (size_t)dec->block_size);
    return dec->blocks ? 0 : -1;
}

static int bad_decode_no_compression(BlockAdaptiveDecoder *dec, BitReader *br, size_t block_index) {
    int dyn_bits = dec->image_constants->dynamic_range_bits;
    size_t base = block_index * (size_t)dec->block_size;
    for (int i = 0; i < dec->block_size; i++) {
        uint64_t value = 0;
        if (br_read_bits_u64(br, dyn_bits, &value) != 0) return -1;
        dec->blocks[base + i] = (int64_t)value;
    }
    return 0;
}

static int bad_inverse_pairing(int64_t transformed, int64_t *d0_out, int64_t *d1_out) {
    if (transformed < 0) return -1;

    int64_t w = (int64_t)floor((sqrt(8.0 * (double)transformed + 1.0) - 1.0) / 2.0);
    while (((w + 1) * (w + 2)) / 2 <= transformed) w++;
    while ((w * (w + 1)) / 2 > transformed) w--;

    int64_t t = (w * (w + 1)) / 2;
    int64_t d1 = transformed - t;
    int64_t d0 = w - d1;
    if (d0 < 0 || d1 < 0) return -1;

    *d0_out = d0;
    *d1_out = d1;
    return 0;
}

static int bad_decode_second_extension(BlockAdaptiveDecoder *dec, BitReader *br, size_t block_index) {
    size_t base = block_index * (size_t)dec->block_size;
    for (int i = 0; i < dec->block_size; i += 2) {
        int64_t transformed = 0;
        int64_t d0 = 0;
        int64_t d1 = 0;
        if (br_read_unary_zeroes(br, &transformed) != 0) return -1;
        if (bad_inverse_pairing(transformed, &d0, &d1) != 0) return -1;
        dec->blocks[base + i] = d0;
        dec->blocks[base + i + 1] = d1;
    }
    return 0;
}

static int bad_decode_sample_splitting(BlockAdaptiveDecoder *dec, BitReader *br, size_t block_index, int k) {
    size_t base = block_index * (size_t)dec->block_size;
    int64_t highs[64];
    if (dec->block_size > 64) return -1;

    for (int i = 0; i < dec->block_size; i++) {
        if (br_read_unary_zeroes(br, &highs[i]) != 0) {
            fprintf(stderr, "Unary read failed in sample-splitting block %zu sample %d at bit %zu\n",
                    block_index, i, br->bit_pos);
            return -1;
        }
    }

    for (int i = 0; i < dec->block_size; i++) {
        uint64_t low = 0;
        if (k > 0 && br_read_bits_u64(br, k, &low) != 0) {
            fprintf(stderr, "Low-bit read failed in sample-splitting block %zu sample %d at bit %zu\n",
                    block_index, i, br->bit_pos);
            return -1;
        }
        dec->blocks[base + i] = (highs[i] << k) | (int64_t)low;
    }
    return 0;
}

static size_t bad_special_zero_run_count(const BlockAdaptiveDecoder *dec, size_t block_index) {
    size_t in_reference = block_index % (size_t)dec->reference_sample_interval;
    size_t segment_remaining = (size_t)dec->segment_size - (in_reference % (size_t)dec->segment_size);
    size_t reference_remaining = (size_t)dec->reference_sample_interval - in_reference;
    size_t remaining = dec->blocks_count - block_index;

    size_t count = segment_remaining;
    if (reference_remaining < count) count = reference_remaining;
    if (remaining < count) count = remaining;
    return count;
}

static int bad_decode_zero_run(BlockAdaptiveDecoder *dec, BitReader *br, size_t block_index, size_t *run_count_out) {
    int64_t zeros_before_one = 0;
    if (br_read_unary_zeroes(br, &zeros_before_one) != 0) return -1;

    if (zeros_before_one <= 3) {
        *run_count_out = (size_t)(zeros_before_one + 1);
    } else if (zeros_before_one == 4) {
        *run_count_out = bad_special_zero_run_count(dec, block_index);
    } else {
        *run_count_out = (size_t)zeros_before_one;
    }

    if (*run_count_out == 0 || block_index + *run_count_out > dec->blocks_count) return -1;

    return 0;
}

static int bad_decode_blocks(BlockAdaptiveDecoder *dec, BitReader *br) {
    if (bad_init_constants(dec) != 0) return -1;
    if (bad_alloc_blocks(dec) != 0) return -1;

    const uint64_t all_ones = (dec->id_bits >= 64) ? UINT64_MAX : ((1ULL << dec->id_bits) - 1ULL);

    for (size_t block_index = 0; block_index < dec->blocks_count;) {
        uint64_t code = 0;
        if (br_read_bits_u64(br, dec->id_bits, &code) != 0) {
            fprintf(stderr, "Failed reading block code at block %zu\n", block_index);
            return -1;
        }

        if (code == all_ones) {
            if (bad_decode_no_compression(dec, br, block_index) != 0) {
                fprintf(stderr, "Failed no-compression block at block %zu\n", block_index);
                return -1;
            }
            block_index++;
            continue;
        }

        if (code == 0) {
            int next_bit = 0;
            if (br_read_bit(br, &next_bit) != 0) {
                fprintf(stderr, "Failed reading code extension at block %zu\n", block_index);
                return -1;
            }
            if (next_bit == 1) {
                if (bad_decode_second_extension(dec, br, block_index) != 0) {
                    fprintf(stderr, "Failed second-extension block at block %zu\n", block_index);
                    return -1;
                }
                block_index++;
                continue;
            }

            size_t zero_run = 0;
            if (bad_decode_zero_run(dec, br, block_index, &zero_run) != 0) {
                fprintf(stderr, "Failed zero-run at block %zu\n", block_index);
                return -1;
            }
            block_index += zero_run;
            continue;
        }

        int k = (int)code - 1;
        if (k < 0 || k > dec->max_sample_split_bits) {
            fprintf(stderr, "Invalid sample-splitting code %d at block %zu\n", k, block_index);
            return -1;
        }
        if (bad_decode_sample_splitting(dec, br, block_index, k) != 0) {
            fprintf(stderr, "Failed sample-splitting block at block %zu with k=%d\n", block_index, k);
            return -1;
        }
        block_index++;
    }

    return 0;
}

static int bad_unpack_values(BlockAdaptiveDecoder *dec, int64_t *mapped_quantizer_index, size_t mapped_len) {
    Header *h = dec->header;
    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);
    size_t needed = (size_t)x * (size_t)y * (size_t)z;
    if (!mapped_quantizer_index || mapped_len < needed) return -1;

    size_t index = 0;
    if (h->sample_encoding_order == ORDER_BI) {
        for (int yi = 0; yi < y; yi++) {
            if (yi % (1 << h->error_update_period_exponent) == 0 && h->periodic_error_updating_flag == PEU_USED) {
                int period_index = yi / (1 << h->error_update_period_exponent);
                if (h->quantizer_fidelity_control_method != QF_REL) {
                    if (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
                        int64_t value = dec->blocks[index++];
                        for (int zi = 0; zi < z; zi++) h->periodic_absolute_error_limit_table[period_index * z + zi] = value;
                    } else {
                        for (int zi = 0; zi < z; zi++) h->periodic_absolute_error_limit_table[period_index * z + zi] = dec->blocks[index++];
                    }
                }
                if (h->quantizer_fidelity_control_method != QF_ABS) {
                    if (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
                        int64_t value = dec->blocks[index++];
                        for (int zi = 0; zi < z; zi++) h->periodic_relative_error_limit_table[period_index * z + zi] = value;
                    } else {
                        for (int zi = 0; zi < z; zi++) h->periodic_relative_error_limit_table[period_index * z + zi] = dec->blocks[index++];
                    }
                }
            }

            for (int i = 0; i < (z + h->sub_frame_interleaving_depth - 1) / h->sub_frame_interleaving_depth; i++) {
                for (int xi = 0; xi < x; xi++) {
                    int z_start = i * h->sub_frame_interleaving_depth;
                    int z_end = z_start + h->sub_frame_interleaving_depth;
                    if (z_end > z) z_end = z;
                    for (int zi = z_start; zi < z_end; zi++) {
                        mapped_quantizer_index[idx3(yi, xi, zi, x, z)] = dec->blocks[index++];
                    }
                }
            }
        }
    } else {
        if (h->periodic_error_updating_flag == PEU_USED) {
            fprintf(stderr, "BSQ periodic error updates are not supported.\n");
            return -1;
        }
        for (int zi = 0; zi < z; zi++) {
            for (int yi = 0; yi < y; yi++) {
                for (int xi = 0; xi < x; xi++) {
                    mapped_quantizer_index[idx3(yi, xi, zi, x, z)] = dec->blocks[index++];
                }
            }
        }
    }

    return 0;
}

static void image_constants_init(ImageConstants *ic, const Header *h) {
    ic->dynamic_range_bits = header_get_dynamic_range_bits(h);
    ic->dynamic_range = (int64_t)1 << ic->dynamic_range_bits;

    if (h->sample_type == SAMPLE_UNSIGNED) {
        ic->lower_sample_limit = 0;
        ic->upper_sample_limit = ((int64_t)1 << ic->dynamic_range_bits) - 1;
        ic->middle_sample_value = (int64_t)1 << (ic->dynamic_range_bits - 1);
    } else {
        ic->lower_sample_limit = -((int64_t)1 << (ic->dynamic_range_bits - 1));
        ic->upper_sample_limit = ((int64_t)1 << (ic->dynamic_range_bits - 1)) - 1;
        ic->middle_sample_value = 0;
    }
}

typedef struct {
    Header *h;
    ImageConstants *ic;
    int x;
    int y;
    int z;
    int c;

    int *spectral_bands_used;      /* [z] */
    double *weight_exponent_offset;/* [z][c] */

    int weight_component_resolution;
    int weight_update_change_interval;
    int weight_update_initial_parameter;
    int weight_update_final_parameter;
    int64_t weight_min;
    int64_t weight_max;
    int register_size;

    int64_t *prev_row_sample_representative; /* [x][z] */
    int64_t *curr_row_sample_representative; /* [x][z] */
    int64_t *weight_state;                   /* [z][c] */
    int64_t *prev_local_difference;          /* [z][c] */
    int64_t *prev_prediction_error;          /* [z] */
    int64_t *local_difference_curr;          /* [c] */
    int64_t *local_difference_prev_z;        /* [c] */
} PredictorStreamState;

static size_t row_idx(int x, int z, int z_size) {
    return (size_t)x * (size_t)z_size + (size_t)z;
}

static int predictor_stream_init(PredictorStreamState *s, Header *h, ImageConstants *ic) {
    memset(s, 0, sizeof(*s));
    s->h = h;
    s->ic = ic;
    s->x = header_get_x_size(h);
    s->y = header_get_y_size(h);
    s->z = header_get_z_size(h);
    s->c = h->prediction_bands_num + (h->prediction_mode == PRED_FULL ? 3 : 0);

    s->weight_component_resolution = h->weight_component_resolution + 4;
    s->weight_update_change_interval = 1 << (h->weight_update_change_interval + 4);
    s->weight_update_initial_parameter = h->weight_update_initial_parameter - 6;
    s->weight_update_final_parameter = h->weight_update_final_parameter - 6;
    s->weight_min = -((int64_t)1 << (s->weight_component_resolution + 2));
    s->weight_max = ((int64_t)1 << (s->weight_component_resolution + 2)) - 1;
    s->register_size = (h->register_size == 0) ? 64 : h->register_size;

    s->spectral_bands_used = (int *)workspace_calloc((size_t)s->z, sizeof(int));
    s->weight_exponent_offset = (double *)workspace_calloc((size_t)s->z * (size_t)(s->c > 0 ? s->c : 1), sizeof(double));
    s->prev_row_sample_representative = alloc_i64((size_t)s->x * (size_t)s->z);
    s->curr_row_sample_representative = alloc_i64((size_t)s->x * (size_t)s->z);
    s->weight_state = alloc_i64((size_t)s->z * (size_t)(s->c > 0 ? s->c : 1));
    s->prev_local_difference = alloc_i64((size_t)s->z * (size_t)(s->c > 0 ? s->c : 1));
    s->prev_prediction_error = alloc_i64((size_t)s->z);
    s->local_difference_curr = alloc_i64((size_t)(s->c > 0 ? s->c : 1));
    s->local_difference_prev_z = alloc_i64((size_t)(s->c > 0 ? s->c : 1));

    if (!s->spectral_bands_used || !s->weight_exponent_offset ||
        !s->prev_row_sample_representative || !s->curr_row_sample_representative ||
        !s->weight_state || !s->prev_local_difference || !s->prev_prediction_error ||
        !s->local_difference_curr || !s->local_difference_prev_z) {
        return -1;
    }

    for (int zi = 0; zi < s->z; zi++) {
        s->spectral_bands_used[zi] = (zi < h->prediction_bands_num) ? zi : h->prediction_bands_num;
    }

    if (s->c > 0 && h->weight_exponent_offset_flag == WEO_NOT_ALL_ZERO && h->weight_exponent_offset_table) {
        if (h->prediction_mode == PRED_FULL) {
            for (int zi = 0; zi < s->z; zi++) {
                for (int i = 0; i < 3; i++) {
                    s->weight_exponent_offset[(size_t)zi * (size_t)s->c + (size_t)i] =
                        (double)h->weight_exponent_offset_table[(size_t)zi * (size_t)(h->prediction_bands_num + 1)];
                }
                for (int i = 3; i < s->c; i++) {
                    s->weight_exponent_offset[(size_t)zi * (size_t)s->c + (size_t)i] =
                        (double)h->weight_exponent_offset_table[(size_t)zi * (size_t)(h->prediction_bands_num + 1) + (size_t)(i - 2)];
                }
            }
        } else {
            for (int zi = 0; zi < s->z; zi++) {
                for (int i = 0; i < s->c; i++) {
                    s->weight_exponent_offset[(size_t)zi * (size_t)s->c + (size_t)i] =
                        (double)h->weight_exponent_offset_table[(size_t)zi * (size_t)h->prediction_bands_num + (size_t)i];
                }
            }
        }
    }

    return 0;
}

static void predictor_stream_get_error_limits(const PredictorStreamState *s, int yi, int zi,
                                              int64_t *abs_out, int64_t *rel_out) {
    Header *h = s->h;
    int z = s->z;
    *abs_out = -1;
    *rel_out = -1;

    if (h->periodic_error_updating_flag == PEU_NOT_USED) {
        if (h->quantizer_fidelity_control_method != QF_REL) {
            if (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) *abs_out = h->absolute_error_limit_value;
            else *abs_out = h->absolute_error_limit_table[zi];
        }
        if (h->quantizer_fidelity_control_method != QF_ABS) {
            if (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) *rel_out = h->relative_error_limit_value;
            else *rel_out = h->relative_error_limit_table[zi];
        }
        return;
    }

    {
        int period = 1 << h->error_update_period_exponent;
        int pi = yi / period;
        if (h->quantizer_fidelity_control_method != QF_REL) {
            if (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) *abs_out = h->periodic_absolute_error_limit_table[pi * z];
            else *abs_out = h->periodic_absolute_error_limit_table[pi * z + zi];
        }
        if (h->quantizer_fidelity_control_method != QF_ABS) {
            if (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) *rel_out = h->periodic_relative_error_limit_table[pi * z];
            else *rel_out = h->periodic_relative_error_limit_table[pi * z + zi];
        }
    }
}

static int64_t predictor_stream_local_sum(const PredictorStreamState *s, int yi, int xi, int zi, int t) {
    Header *h = s->h;
    if (t == 0) return 0;

    const int64_t *prev = s->prev_row_sample_representative;
    const int64_t *curr = s->curr_row_sample_representative;
    int x_size = s->x;
    int z_size = s->z;

    if (h->local_sum_type == LS_WIDE_NEIGHBOR) {
        if (yi > 0 && xi > 0 && xi < x_size - 1) {
            return curr[row_idx(xi - 1, zi, z_size)] +
                   prev[row_idx(xi - 1, zi, z_size)] +
                   prev[row_idx(xi, zi, z_size)] +
                   prev[row_idx(xi + 1, zi, z_size)];
        } else if (yi == 0 && xi > 0) {
            return curr[row_idx(xi - 1, zi, z_size)] * 4;
        } else if (yi > 0 && xi == 0) {
            return (prev[row_idx(xi, zi, z_size)] +
                    prev[row_idx(xi + 1, zi, z_size)]) * 2;
        } else if (yi > 0 && xi == x_size - 1) {
            return curr[row_idx(xi - 1, zi, z_size)] +
                   prev[row_idx(xi - 1, zi, z_size)] +
                   prev[row_idx(xi, zi, z_size)] * 2;
        }
    } else if (h->local_sum_type == LS_NARROW_NEIGHBOR) {
        if (yi > 0 && xi > 0 && xi < x_size - 1) {
            return prev[row_idx(xi - 1, zi, z_size)] +
                   prev[row_idx(xi, zi, z_size)] * 2 +
                   prev[row_idx(xi + 1, zi, z_size)];
        } else if (yi == 0 && xi > 0 && zi > 0) {
            return curr[row_idx(xi - 1, zi - 1, z_size)] * 4;
        } else if (yi > 0 && xi == 0) {
            return (prev[row_idx(xi, zi, z_size)] +
                    prev[row_idx(xi + 1, zi, z_size)]) * 2;
        } else if (yi > 0 && xi == x_size - 1) {
            return (prev[row_idx(xi - 1, zi, z_size)] +
                    prev[row_idx(xi, zi, z_size)]) * 2;
        } else if (yi == 0 && xi > 0 && zi == 0) {
            return s->ic->middle_sample_value * 4;
        }
    } else if (h->local_sum_type == LS_WIDE_COLUMN) {
        if (yi > 0) return prev[row_idx(xi, zi, z_size)] * 4;
        if (yi == 0 && xi > 0) return curr[row_idx(xi - 1, zi, z_size)] * 4;
    } else if (h->local_sum_type == LS_NARROW_COLUMN) {
        if (yi > 0) return prev[row_idx(xi, zi, z_size)] * 4;
        if (yi == 0 && xi > 0 && zi > 0) return curr[row_idx(xi - 1, zi - 1, z_size)] * 4;
        if (yi == 0 && xi > 0 && zi == 0) return s->ic->middle_sample_value * 4;
    }

    return 0;
}

static int predictor_stream_process(PredictorStreamState *s, int64_t *image_sample,
                                    int64_t *mapped_quantizer_index, int inverse) {
    Header *h = s->h;
    ImageConstants *ic = s->ic;
    int x = s->x;
    int y = s->y;
    int z = s->z;
    int c = s->c;

    if (!image_sample || (!inverse && !mapped_quantizer_index) || (inverse && !mapped_quantizer_index)) return -1;

    for (int yi = 0; yi < y; yi++) {
        if (yi > 0) {
            int64_t *tmp = s->prev_row_sample_representative;
            s->prev_row_sample_representative = s->curr_row_sample_representative;
            s->curr_row_sample_representative = tmp;
            memset(s->curr_row_sample_representative, 0, (size_t)x * (size_t)z * sizeof(int64_t));
        }

        for (int xi = 0; xi < x; xi++) {
            int t = xi + yi * x;
            int64_t scaling_exponent = 0;
            if (t > 1) {
                int64_t base = s->weight_update_initial_parameter +
                    floor_div_i64((int64_t)(t - x), s->weight_update_change_interval);
                base = clip_i64(base, s->weight_update_initial_parameter, s->weight_update_final_parameter);
                scaling_exponent = base + ic->dynamic_range_bits - s->weight_component_resolution;
            }
            if (c > 0) memset(s->local_difference_prev_z, 0, (size_t)c * sizeof(int64_t));
            int64_t local_sum_prev_z = 0;

            for (int zi = 0; zi < z; zi++) {
                size_t idx = idx3(yi, xi, zi, x, z);
                int64_t local_sum = predictor_stream_local_sum(s, yi, xi, zi, t);
                int64_t *ld = s->local_difference_curr;
                int64_t *wz = s->weight_state + (size_t)zi * (size_t)(c > 0 ? c : 1);
                if (c > 0) memset(ld, 0, (size_t)c * sizeof(int64_t));

                int offset = 0;
                if (h->prediction_mode == PRED_FULL && c > 0) {
                    if (xi > 0 && yi > 0) {
                        ld[0] = 4 * s->prev_row_sample_representative[row_idx(xi, zi, z)] - local_sum;
                        ld[1] = 4 * s->curr_row_sample_representative[row_idx(xi - 1, zi, z)] - local_sum;
                        ld[2] = 4 * s->prev_row_sample_representative[row_idx(xi - 1, zi, z)] - local_sum;
                    } else if (xi == 0 && yi > 0) {
                        int64_t v = 4 * s->prev_row_sample_representative[row_idx(xi, zi, z)] - local_sum;
                        ld[0] = v; ld[1] = v; ld[2] = v;
                    }
                    offset = 3;
                }

                if (zi > 0 && s->spectral_bands_used[zi] > 0 && c > 0) {
                    ld[offset] = 4 * s->curr_row_sample_representative[row_idx(xi, zi - 1, z)] - local_sum_prev_z;
                    for (int i = 1; i < s->spectral_bands_used[zi]; i++) {
                        ld[offset + i] = s->local_difference_prev_z[offset + i - 1];
                    }
                }

                if (t == 1 && c > 0) {
                    if (h->weight_init_method == WEIGHT_INIT_DEFAULT) {
                        memset(wz, 0, (size_t)c * sizeof(int64_t));
                        int woff = (h->prediction_mode == PRED_FULL) ? 3 : 0;
                        if (zi > 0 && s->spectral_bands_used[zi] > 0) {
                            wz[woff] = ((int64_t)1 << s->weight_component_resolution) * 7 / 8;
                            for (int i = 1; i < s->spectral_bands_used[zi]; i++) {
                                wz[woff + i] = wz[woff + i - 1] / 8;
                            }
                        }
                    } else {
                        for (int i = 0; i < c; i++) {
                            int64_t val = h->weight_init_table[(size_t)zi * (size_t)c + (size_t)i];
                            int shift1 = s->weight_component_resolution + 3 - h->weight_init_resolution;
                            int shift2 = s->weight_component_resolution + 2 - h->weight_init_resolution;
                            wz[i] = ((int64_t)1 << shift1) * val + (((int64_t)1 << shift2) - 1);
                        }
                    }
                } else if (t > 1 && c > 0) {
                    int64_t *prev_ld = s->prev_local_difference + (size_t)zi * (size_t)c;
                    for (int i = 0; i < c; i++) {
                        double term =
                            (double)sign_positive_i64(s->prev_prediction_error[zi]) *
                            (double)prev_ld[i] *
                            pow(2.0, -(double)(scaling_exponent + s->weight_exponent_offset[(size_t)zi * (size_t)c + (size_t)i]));
                        int64_t delta = floor_div_i64((int64_t)floor(term) + 1, 2);
                        wz[i] = clip_i64(wz[i] + delta, s->weight_min, s->weight_max);
                    }
                }

                int64_t predicted_center = 0;
                if (t > 0 && !(h->prediction_mode == PRED_REDUCED && zi == 0) && c > 0) {
                    for (int i = 0; i < c; i++) predicted_center += wz[i] * ld[i];
                }

                int64_t high_pred = 0;
                int64_t double_pred = 0;
                if (t > 0) {
                    int64_t tmp = predicted_center +
                        ((int64_t)1 << s->weight_component_resolution) * (local_sum - 4 * ic->middle_sample_value);
                    tmp = modulo_star_i64(tmp, s->register_size);
                    tmp += ((int64_t)1 << (s->weight_component_resolution + 2)) * ic->middle_sample_value +
                           ((int64_t)1 << (s->weight_component_resolution + 1));
                    {
                        int64_t minv = ((int64_t)1 << (s->weight_component_resolution + 2)) * ic->lower_sample_limit;
                        int64_t maxv = ((int64_t)1 << (s->weight_component_resolution + 2)) * ic->upper_sample_limit +
                                       ((int64_t)1 << (s->weight_component_resolution + 1));
                        high_pred = clip_i64(tmp, minv, maxv);
                    }
                    double_pred = high_pred >> (s->weight_component_resolution + 1);
                } else if (h->prediction_bands_num > 0 && zi > 0) {
                    double_pred = 2 * image_sample[idx3(yi, xi, zi - 1, x, z)];
                } else {
                    double_pred = 2 * ic->middle_sample_value;
                }
                int64_t predicted_sample = double_pred / 2;

                int64_t abs_e = -1, rel_e = -1;
                predictor_stream_get_error_limits(s, yi, zi, &abs_e, &rel_e);

                int64_t max_error = 0;
                if (h->quantizer_fidelity_control_method == QF_LOSSLESS) max_error = 0;
                else if (h->quantizer_fidelity_control_method == QF_ABS) max_error = abs_e;
                else if (h->quantizer_fidelity_control_method == QF_REL) {
                    max_error = (int64_t)floor((double)rel_e * (double)predicted_sample / (double)ic->dynamic_range);
                } else {
                    int64_t rel_calc = (int64_t)floor((double)rel_e * (double)predicted_sample / (double)ic->dynamic_range);
                    max_error = (abs_e < rel_calc) ? abs_e : rel_calc;
                }

                int64_t q = 0;
                int64_t reconstructed = 0;
                int64_t clipped_bin = 0;
                if (!inverse) {
                    int64_t residual = image_sample[idx] - predicted_sample;
                    if (t == 0) q = residual;
                    else {
                        int64_t numerator = llabs(residual) + max_error;
                        int64_t denom = 2 * max_error + 1;
                        q = sign_i64(residual) * (numerator / denom);
                    }
                    reconstructed = image_sample[idx];
                } else {
                    int64_t mapped = mapped_quantizer_index[idx];
                    int64_t lower_diff = 0;
                    int64_t upper_diff = 0;
                    if (t == 0) {
                        lower_diff = predicted_sample - ic->lower_sample_limit;
                        upper_diff = ic->upper_sample_limit - predicted_sample;
                    } else {
                        int64_t denom = 2 * max_error + 1;
                        lower_diff = (predicted_sample - ic->lower_sample_limit + max_error) / denom;
                        upper_diff = (ic->upper_sample_limit - predicted_sample + max_error) / denom;
                    }
                    {
                        int64_t sdiff = (lower_diff < upper_diff) ? lower_diff : upper_diff;
                        int parity_sign = ((double_pred % 2) == 0) ? 1 : -1;
                        if (mapped > 2 * sdiff) {
                            int64_t magnitude = mapped - sdiff;
                            if (upper_diff > lower_diff) q = magnitude;
                            else if (lower_diff > upper_diff) q = -magnitude;
                            else return -1;
                        } else if ((mapped & 1) == 0) {
                            q = (int64_t)parity_sign * (mapped / 2);
                        } else {
                            q = -(int64_t)parity_sign * ((mapped + 1) / 2);
                        }
                    }
                    if (t == 0 || max_error == 0) reconstructed = predicted_sample + q;
                    else reconstructed = predicted_sample + q * (2 * max_error + 1);
                    reconstructed = clip_i64(reconstructed, ic->lower_sample_limit, ic->upper_sample_limit);
                    image_sample[idx] = reconstructed;
                }

                if (t == 0) {
                    clipped_bin = reconstructed;
                } else if (max_error == 0) {
                    clipped_bin = reconstructed;
                } else {
                    int64_t val = predicted_sample + q * (2 * max_error + 1);
                    clipped_bin = clip_i64(val, ic->lower_sample_limit, ic->upper_sample_limit);
                }

                int64_t sample_rep = 0;
                int64_t double_sample_rep = 0;
                if (t == 0) {
                    sample_rep = reconstructed;
                    double_sample_rep = 2 * sample_rep;
                } else if (h->damping_table_array[zi] == 0 && h->damping_offset_table_array[zi] == 0) {
                    double_sample_rep = 2 * clipped_bin;
                    sample_rep = clipped_bin;
                } else {
                    int64_t term1 = 4 * (((int64_t)1 << h->sample_representative_resolution) - h->damping_table_array[zi]);
                    int64_t term2 = clipped_bin * ((int64_t)1 << s->weight_component_resolution) -
                        sign_i64(q) * max_error * h->damping_offset_table_array[zi] *
                        ((int64_t)1 << (s->weight_component_resolution - h->sample_representative_resolution));
                    int64_t term3 = h->damping_table_array[zi] * high_pred -
                        h->damping_table_array[zi] * ((int64_t)1 << (s->weight_component_resolution + 1));
                    int64_t denom = (int64_t)1 << (s->weight_component_resolution + h->sample_representative_resolution + 1);
                    double_sample_rep = floor_div_i64(term1 * term2 + term3, denom);
                    sample_rep = (double_sample_rep + 1) / 2;
                }

                if (!inverse) {
                    int64_t lower_diff = 0;
                    int64_t upper_diff = 0;
                    int64_t sdiff = 0;
                    int64_t mapped = 0;
                    int64_t term = 0;
                    if (t == 0) {
                        lower_diff = predicted_sample - ic->lower_sample_limit;
                        upper_diff = ic->upper_sample_limit - predicted_sample;
                    } else {
                        int64_t denom = 2 * max_error + 1;
                        lower_diff = (predicted_sample - ic->lower_sample_limit + max_error) / denom;
                        upper_diff = (ic->upper_sample_limit - predicted_sample + max_error) / denom;
                    }
                    sdiff = (lower_diff < upper_diff) ? lower_diff : upper_diff;
                    term = ((double_pred % 2) == 0 ? 1 : -1) * q;

                    if (llabs(q) > sdiff) mapped = llabs(q) + sdiff;
                    else if (0 <= term && term <= sdiff) mapped = 2 * llabs(q);
                    else mapped = 2 * llabs(q) - 1;
                    mapped_quantizer_index[idx] = mapped;
                }

                s->curr_row_sample_representative[row_idx(xi, zi, z)] = sample_rep;
                if (c > 0) {
                    memcpy(s->prev_local_difference + (size_t)zi * (size_t)c, ld, (size_t)c * sizeof(int64_t));
                    memcpy(s->local_difference_prev_z, ld, (size_t)c * sizeof(int64_t));
                }
                if (t > 0) {
                    s->prev_prediction_error[zi] = 2 * clipped_bin - double_pred;
                }
                local_sum_prev_z = local_sum;
                (void)double_sample_rep;
            }
        }
    }

    return 0;
}

static int header_config_from_dims(Header *h, int x, int y, int z, const char *dtype) {
    if (x <= 0 || y <= 0 || z <= 0) return -1;
    h->x_size = x % 65536;
    h->y_size = y % 65536;
    h->z_size = z % 65536;

    if (dtype[0] == 'u' || strncmp(dtype, "rgb", 3) == 0) h->sample_type = SAMPLE_UNSIGNED;
    else if (dtype[0] == 's') h->sample_type = SAMPLE_SIGNED;
    else return -1;

    int bits = 0;
    for (size_t i = 0; i < strlen(dtype); i++) {
        if (dtype[i] >= '0' && dtype[i] <= '9') {
            bits = atoi(&dtype[i]);
            break;
        }
    }
    if (bits > 0) header_set_dynamic_range(h, bits);
    return 0;
}

static int make_output_folder(const char *output_root, const char *raw_path, int ael, char *out_dir) {
    build_output_folder_path(output_root, raw_path, ael, out_dir);
    return ensure_dir(out_dir);
}

static int build_out_path(const char *out_dir, const char *file_name, char *path, size_t path_len) {
    int written = snprintf(path, path_len, "%s/%s", out_dir, file_name);
    if (written < 0 || (size_t)written >= path_len) {
        fprintf(stderr, "Output path too long: %s/%s\n", out_dir, file_name);
        return -1;
    }
    return 0;
}


static int write_bitstream_with_header(const char *out_dir, const char *out_file_name, Header *h, BitWriter *payload) {
    BitWriter full; bw_init(&full);
    header_build_bitstreams(h);

    for (size_t bi = 0; bi < h->header_bitstream.bit_len; bi++) {
        int bit = (h->header_bitstream.data[bi / 8] >> (7 - (bi % 8))) & 1;
        bw_append_bit(&full, bit);
    }
    for (size_t bi = 0; bi < payload->bit_len; bi++) {
        int bit = (payload->data[bi / 8] >> (7 - (bi % 8))) & 1;
        bw_append_bit(&full, bit);
    }

    int word_bits = 8 * (h->output_word_size + 8 * (h->output_word_size == 0));
    int fill = (word_bits - (full.bit_len % word_bits)) % word_bits;
    for (int i = 0; i < fill; i++) bw_append_bit(&full, 0);

    char path[MAX_PATH_LEN];
    if (build_out_path(out_dir, out_file_name, path, sizeof(path)) != 0) {
        bw_free(&full);
        return -1;
    }
    int rc = bw_write_to_file(&full, path);
    bw_free(&full);
    return rc;
}

static int load_file_bytes(const char *path, uint8_t **data_out, size_t *len_out) {
    FILE *f = fopen(path, "rb");
    if (!f) return -1;

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return -1;
    }
    long size = ftell(f);
    if (size < 0) {
        fclose(f);
        return -1;
    }
    if (fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return -1;
    }

    uint8_t *data = NULL;
    if (size > 0) {
        data = (uint8_t *)workspace_alloc((size_t)size);
        if (!data) {
            fclose(f);
            return -1;
        }
        if (fread(data, 1, (size_t)size, f) != (size_t)size) {
            workspace_free(data);
            fclose(f);
            return -1;
        }
    }

    fclose(f);
    *data_out = data;
    *len_out = (size_t)size;
    return 0;
}

static int header_infer_output_dtype(const Header *h, char *dtype_out, size_t dtype_len) {
    const char *prefix = (h->sample_type == SAMPLE_UNSIGNED) ? "u" : "s";
    int bits = header_get_dynamic_range_bits(h);
    int storage_bits = 0;

    if (bits <= 8) storage_bits = 8;
    else if (bits <= 16) storage_bits = 16;
    else if (bits <= 32) storage_bits = 32;
    else if (bits <= 64) storage_bits = 64;
    else return -1;

    int written = snprintf(dtype_out, dtype_len, "%s%dbe", prefix, storage_bits);
    return (written < 0 || (size_t)written >= dtype_len) ? -1 : 0;
}

static int decompress_one_image(const char *bitstream_path, const char *output_root,
                                const uint8_t *bitstream_buf, size_t bitstream_len) {
    Header h;
    header_init_defaults(&h);

    int result = -1;
    uint8_t *owned_bitstream_buf = NULL;
    const uint8_t *bitstream_data = bitstream_buf;
    int64_t *image_sample = NULL;
    int64_t *mapped_quantizer_index = NULL;
    Predictor pred;
    int pred_ready = 0;
    size_t sample_count = 0;

    if (!bitstream_data) {
        if (load_file_bytes(bitstream_path, &owned_bitstream_buf, &bitstream_len) != 0) {
            fprintf(stderr, "Failed reading bitstream: %s\n", bitstream_path);
            goto cleanup;
        }
        bitstream_data = owned_bitstream_buf;
    }

    if (!bitstream_data || bitstream_len == 0) {
        fprintf(stderr, "Bitstream is empty: %s\n", bitstream_path);
        goto cleanup;
    }

    BitReader br;
    br_init(&br, bitstream_data, bitstream_len);
    if (header_parse_from_bitstream(&h, &br) != 0) {
        fprintf(stderr, "Failed parsing CCSDS123 header: %s\n", bitstream_path);
        goto cleanup;
    }

    ImageConstants ic;
    image_constants_init(&ic, &h);

    int x = header_get_x_size(&h);
    int y = header_get_y_size(&h);
    int z = header_get_z_size(&h);
    sample_count = (size_t)x * (size_t)y * (size_t)z;

    image_sample = (int64_t *)workspace_alloc(sizeof(int64_t) * sample_count);
    mapped_quantizer_index = alloc_i64(sample_count);
    if (!image_sample || !mapped_quantizer_index) goto cleanup;
    memset(image_sample, 0, sizeof(int64_t) * sample_count);

    {
        BlockAdaptiveDecoder dec;
        bad_init(&dec, &h, &ic);
        if (bad_decode_blocks(&dec, &br) != 0) {
            fprintf(stderr, "Block-adaptive decoder failed for %s\n", bitstream_path);
            bad_free(&dec);
            goto cleanup;
        }
        if (bad_unpack_values(&dec, mapped_quantizer_index, sample_count) != 0) {
            fprintf(stderr, "Failed unpacking decoded values for %s\n", bitstream_path);
            bad_free(&dec);
            goto cleanup;
        }
        bad_free(&dec);
    }

    if (g_workspace.enabled) {
        PredictorStreamState ps;
        if (predictor_stream_init(&ps, &h, &ic) != 0 ||
            predictor_stream_process(&ps, image_sample, mapped_quantizer_index, 1) != 0) {
            fprintf(stderr, "Inverse predictor (streaming) failed for %s\n", bitstream_path);
            goto cleanup;
        }
    } else {
        predictor_init(&pred, &h, &ic, image_sample);
        pred.mapped_quantizer_index = mapped_quantizer_index;
        mapped_quantizer_index = NULL; /* ownership transferred to Predictor */
        if (predictor_inverse_run(&pred) != 0) {
            fprintf(stderr, "Inverse predictor failed for %s\n", bitstream_path);
            goto cleanup;
        }
        pred_ready = 1;
    }

    char dtype[16] = {0};
    if (header_infer_output_dtype(&h, dtype, sizeof(dtype)) != 0) {
        fprintf(stderr, "Failed inferring output dtype for %s\n", bitstream_path);
        goto cleanup;
    }

    char out_dir[MAX_PATH_LEN];
    if (make_output_folder(output_root, bitstream_path, 0, out_dir) != 0) goto cleanup;

    char out_file_name[MAX_PATH_LEN];
    if (build_decompressed_filename(bitstream_path, dtype, z, y, x, out_file_name, sizeof(out_file_name)) != 0) {
        fprintf(stderr, "Output file name too long: %s\n", bitstream_path);
        goto cleanup;
    }

    char out_path[MAX_PATH_LEN];
    if (build_out_path(out_dir, out_file_name, out_path, sizeof(out_path)) != 0) goto cleanup;

    if (write_raw_bsq(out_path, dtype, z, y, x, image_sample, sample_count) != 0) {
        fprintf(stderr, "Failed writing decompressed image: %s\n", out_path);
        goto cleanup;
    }

    result = 0;

cleanup:
    if (pred_ready) predictor_free(&pred);
    workspace_free(mapped_quantizer_index);
    workspace_free(image_sample);
    workspace_free(owned_bitstream_buf);
    header_free(&h);
    return result;
}

static int compress_one_image(const char *raw_path, const char *output_root, int ael,
                              int override_x, int override_y, int override_z, const char *override_dtype,
                              int64_t *image_sample_buf, size_t image_sample_len) {
    Header h;
    header_init_defaults(&h);
    int result = -1;
    int64_t *image_sample = image_sample_buf;
    int owns_image_sample = 0;
    Predictor pred;
    int pred_ready = 0;
    size_t image_sample_count = 0;
    char dtype[16] = {0};
    int z = 0, y = 0, x = 0;
    int parsed = (parse_raw_filename(raw_path, dtype, &z, &y, &x) == 0);

    if (override_x > 0) x = override_x;
    if (override_y > 0) y = override_y;
    if (override_z > 0) z = override_z;
    if (override_dtype && override_dtype[0] != '\0') {
        strncpy(dtype, override_dtype, sizeof(dtype) - 1);
        dtype[sizeof(dtype) - 1] = '\0';
    }

    if (x <= 0 || y <= 0) {
        if (parsed) {
            /* parsed x/y already set above */
        } else {
            fprintf(stderr, "Need --x/--y (filename has no dimensions): %s\n", raw_path);
            goto cleanup;
        }
    }
    if (z <= 0) z = 1;
    if (dtype[0] == '\0') {
        strncpy(dtype, "u8be", sizeof(dtype) - 1);
        dtype[sizeof(dtype) - 1] = '\0';
    }

    if (header_config_from_dims(&h, x, y, z, dtype) != 0) goto cleanup;

    /* apply AEL override */
    h.quantizer_fidelity_control_method = QF_ABS;
    h.periodic_error_updating_flag = PEU_NOT_USED;
    h.absolute_error_limit_value = ael;
    header_init_tables_default(&h);

    char out_dir[MAX_PATH_LEN];
    if (make_output_folder(output_root, raw_path, ael, out_dir) != 0) goto cleanup;

    char out_file_name[MAX_PATH_LEN];
    if (build_output_filename(raw_path, out_file_name, sizeof(out_file_name)) != 0) {
        fprintf(stderr, "Output file name too long: %s\n", raw_path);
        goto cleanup;
    }

    /* load image */
    /* dtype/x/y/z already resolved above */
    image_sample_count = (size_t)x * (size_t)y * (size_t)z;
    if (!image_sample) {
        image_sample = (int64_t *)workspace_alloc(sizeof(int64_t) * image_sample_count);
        if (!image_sample) goto cleanup;
        owns_image_sample = 1;
    } else if (image_sample_len < image_sample_count) {
        fprintf(stderr, "Provided image buffer is too small for %zu samples.\n", image_sample_count);
        goto cleanup;
    }

    if (load_raw_bip(raw_path, dtype, z, y, x, image_sample, image_sample_count) != 0) goto cleanup;

    ImageConstants ic;
    image_constants_init(&ic, &h);

    predictor_init(&pred, &h, &ic, image_sample);
    if (g_workspace.enabled) {
        PredictorStreamState ps;
        pred.mapped_quantizer_index = alloc_i64(image_sample_count);
        if (!pred.mapped_quantizer_index ||
            predictor_stream_init(&ps, &h, &ic) != 0 ||
            predictor_stream_process(&ps, image_sample, pred.mapped_quantizer_index, 0) != 0) {
            fprintf(stderr, "Predictor (streaming) failed for %s\n", raw_path);
            goto cleanup;
        }
    } else if (predictor_run(&pred) != 0) {
        fprintf(stderr, "Predictor failed for %s\n", raw_path);
        goto cleanup;
    }
    pred_ready = 1;

    {
        BlockAdaptiveEncoder enc;
        ba_init(&enc, &h, &ic, pred.mapped_quantizer_index);
        if (ba_run(&enc) != 0) {
            fprintf(stderr, "Block-adaptive encoder failed for %s\n", raw_path);
            ba_free(&enc);
            goto cleanup;
        }
        if (write_bitstream_with_header(out_dir, out_file_name, &h, &enc.bitstream) != 0) {
            fprintf(stderr, "Failed writing bitstream for %s\n", raw_path);
            ba_free(&enc);
            goto cleanup;
        }
        ba_free(&enc);
    }

    result = 0;

cleanup:
    if (pred_ready) predictor_free(&pred);
    if (owns_image_sample) workspace_free(image_sample);
    header_free(&h);
    return result;
}

static int ends_with_raw(const char *name) {
    size_t n = strlen(name);
    if (n < 4) return 0;
    const char *ext = name + n - 4;
    return (ext[0] == '.' || ext[0] == '.') &&
           (ext[1] == 'r' || ext[1] == 'R') &&
           (ext[2] == 'a' || ext[2] == 'A') &&
           (ext[3] == 'w' || ext[3] == 'W');
}

int ccsds123_compress_one_image(const char *raw_path, const char *output_root, int ael,
                                int override_x, int override_y, int override_z, const char *override_dtype) {
    return compress_one_image(raw_path, output_root, ael, override_x, override_y, override_z, override_dtype,
                              NULL, 0);
}

int ccsds123_compress_with_buffer(const char *raw_path, const char *output_root, int ael,
                                            int override_x, int override_y, int override_z, const char *override_dtype,
                                            int64_t *image_sample_buf, size_t image_sample_len) {
    int rc = -1;
    if (image_sample_buf) workspace_begin_no_heap();
    rc = compress_one_image(raw_path, output_root, ael, override_x, override_y, override_z, override_dtype,
                            image_sample_buf, image_sample_len);
    if (image_sample_buf) workspace_end_no_heap();
    return rc;
}

int ccsds123_decompress_one_image(const char *bitstream_path, const char *output_root) {
    return decompress_one_image(bitstream_path, output_root, NULL, 0);
}

int ccsds123_decompress_with_buffer(const char *bitstream_path, const char *output_root,
                                    const uint8_t *bitstream_buf, size_t bitstream_len) {
    int rc = -1;
    if (bitstream_buf) workspace_begin_no_heap();
    rc = decompress_one_image(bitstream_path, output_root, bitstream_buf, bitstream_len);
    if (bitstream_buf) workspace_end_no_heap();
    return rc;
}

int ccsds123_ends_with_raw(const char *name) {
    return ends_with_raw(name);
}

// #ifndef UNIT_TEST
// int main(int argc, char **argv) {
//     if (argc < 3) {
//         fprintf(stderr, "Usage: %s <input_raw_file> <output_dir> [ael] [--x N --y N --z N --dtype STR]\n", argv[0]);
//         return 2;
//     }

//     const char *input_file = argv[1];
//     const char *output_dir = argv[2];
//     int ael = 0;
//     int argi = 3;
//     if (argi < argc && strncmp(argv[argi], "--", 2) != 0) {
//         ael = atoi(argv[argi]);
//         argi++;
//     }
//     if (ael < 0) {
//         fprintf(stderr, "AEL must be >= 0\n");
//         return 2;
//     }

//     int override_x = 0;
//     int override_y = 0;
//     int override_z = 0;
//     const char *override_dtype = "";

//     for (int i = argi; i < argc; i++) {
//         if (strcmp(argv[i], "--x") == 0 && i + 1 < argc) {
//             override_x = atoi(argv[++i]);
//         } else if (strcmp(argv[i], "--y") == 0 && i + 1 < argc) {
//             override_y = atoi(argv[++i]);
//         } else if (strcmp(argv[i], "--z") == 0 && i + 1 < argc) {
//             override_z = atoi(argv[++i]);
//         } else if (strcmp(argv[i], "--dtype") == 0 && i + 1 < argc) {
//             override_dtype = argv[++i];
//         } else {
//             fprintf(stderr, "Unknown arg: %s\n", argv[i]);
//             return 2;
//         }
//     }

//     if (!ends_with_raw(input_file)) {
//         fprintf(stderr, "Input must be a .raw file: %s\n", input_file);
//         return 2;
//     }

//     if (ensure_dir(output_dir) != 0) return 1;

//     printf("[ccsds123.0-b-2] %s (AEL=%d)\n", input_file, ael);
//     if (compress_one_image(input_file, output_dir, ael, override_x, override_y, override_z, override_dtype) != 0) {
//         fprintf(stderr, "Failed: %s\n", input_file);
//         return 1;
//     }

//     char out_dir[MAX_PATH_LEN];
//     build_output_folder_path(output_dir, input_file, ael, out_dir);

//     char bitstream_path[MAX_PATH_LEN];
//     if (build_out_path(out_dir, "output.bin", bitstream_path, sizeof(bitstream_path)) != 0) {
//         fprintf(stderr, "Warning: could not compute compression factor.\n");
//         printf("Done compressing.\n");
//         return 0;
//     }

//     long long in_size = 0, out_size = 0;
//     if (get_file_size(input_file, &in_size) == 0 && get_file_size(bitstream_path, &out_size) == 0 && out_size > 0) {
//         double factor = (double)in_size / (double)out_size;
//         printf("Compression factor: %.4f (input %lld bytes, output %lld bytes)\n", factor, in_size, out_size);
//     } else {
//         fprintf(stderr, "Warning: could not compute compression factor.\n");
//     }

//     printf("Done compressing.\n");
//     return 0;
// }
// #endif
