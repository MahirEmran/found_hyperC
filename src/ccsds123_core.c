#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <dirent.h>
#include <errno.h>
#include <ctype.h>

#include "ccsds123_utils.h"
#include "ccsds123_io.h"

/*
 * Native CCSDS 123.0-B-2 encoder port.
 * Single-file build to minimize includes.
 */

#define MAX_PATH_LEN CCSDS123_MAX_PATH_LEN
#define MAX_TABLE_ENTRIES 257
#define MAX_TABLE_STR 512

#include "hybrid_tables_data.h"
#define code_table_input hybrid_code_table_input
#define code_table_output hybrid_code_table_output
#define flush_table_prefix hybrid_flush_table_prefix
#define flush_table_word hybrid_flush_table_word

#define clip_i64 ccsds123_clip_i64
#define sign_i64 ccsds123_sign_i64
#define sign_positive_i64 ccsds123_sign_positive_i64
#define modulo_star_i64 ccsds123_modulo_star_i64
#define floor_div_i64 ccsds123_floor_div_i64

#define alloc_i64 ccsds123_alloc_i64
#define idx3 ccsds123_idx3
#define idx4 ccsds123_idx4

#define bw_init ccsds123_bw_init
#define bw_free ccsds123_bw_free
#define bw_reserve_bits ccsds123_bw_reserve_bits
#define bw_append_bit ccsds123_bw_append_bit
#define bw_append_bits_u64 ccsds123_bw_append_bits_u64
#define bw_append_bits_str ccsds123_bw_append_bits_str
#define bw_append_from_bw ccsds123_bw_append_from_bw
#define bw_pad_to_byte ccsds123_bw_pad_to_byte
#define bw_write_to_file ccsds123_bw_write_to_file

#define ensure_dir ccsds123_ensure_dir
#define parse_raw_filename ccsds123_parse_raw_filename
#define read_sample ccsds123_read_sample
#define load_raw_bip ccsds123_load_raw_bip
#define build_output_folder_path ccsds123_build_output_folder_path
#define get_file_size ccsds123_get_file_size

static int build_out_path(const char *out_dir, const char *file_name, char *path, size_t path_len);

/* ---------------- Header and enums ---------------- */

typedef enum { SAMPLE_UNSIGNED = 0, SAMPLE_SIGNED = 1 } SampleType;
typedef enum { LARGE_D_SMALL = 0, LARGE_D_LARGE = 1 } LargeDFlag;
typedef enum { ORDER_BI = 0, ORDER_BSQ = 1 } SampleEncodingOrder;
typedef enum { ENTROPY_SA = 0, ENTROPY_HYBRID = 1, ENTROPY_BA = 2 } EntropyCoderType;
typedef enum { QF_LOSSLESS = 0, QF_ABS = 1, QF_REL = 2, QF_ABS_REL = 3 } QuantizerFidelityControlMethod;
typedef enum { TABLE_UNSIGNED = 0, TABLE_SIGNED = 1, TABLE_FLOAT = 2 } TableType;
typedef enum { TSTRUCT_0D = 0, TSTRUCT_1D = 1, TSTRUCT_2D_ZX = 2, TSTRUCT_2D_YX = 3 } TableStructure;
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
    TableType table_type;
    int table_purpose;
    TableStructure table_structure;
    int user_defined_data;
    BitWriter table_data_subblock; /* stored as bitstream */
} SupplementaryInformationTable;

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
    int supplementary_information_table_count;
    SupplementaryInformationTable *supp_tables;

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

/* ---------------- Hybrid Tables ---------------- */

static const int input_symbol_limit[16] = {12,10,8,6,6,4,4,4,2,2,2,2,2,2,2,0};
static const int threshold_table[16] = {303336,225404,166979,128672,95597,69670,50678,34898,23331,14935,9282,5510,3195,1928,1112,408};

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
    h->entropy_coder_type = ENTROPY_HYBRID;
    h->quantizer_fidelity_control_method = QF_ABS_REL;
    h->supplementary_information_table_count = 0;
    h->supp_tables = NULL;

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
    free(h->weight_init_table);
    h->weight_init_table = NULL;
    free(h->weight_exponent_offset_table);
    h->weight_exponent_offset_table = NULL;

    free(h->absolute_error_limit_table);
    h->absolute_error_limit_table = NULL;
    free(h->periodic_absolute_error_limit_table);
    h->periodic_absolute_error_limit_table = NULL;
    free(h->relative_error_limit_table);
    h->relative_error_limit_table = NULL;
    free(h->periodic_relative_error_limit_table);
    h->periodic_relative_error_limit_table = NULL;

    free(h->damping_table_array);
    h->damping_table_array = NULL;
    free(h->damping_offset_table_array);
    h->damping_offset_table_array = NULL;
    free(h->accumulator_init_table);
    h->accumulator_init_table = NULL;

    if (h->supp_tables) {
        for (int i = 0; i < h->supplementary_information_table_count; i++) {
            bw_free(&h->supp_tables[i].table_data_subblock);
        }
        free(h->supp_tables);
        h->supp_tables = NULL;
    }

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
    bw_append_bits_u64(bw, (uint64_t)h->supplementary_information_table_count, 4);
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
    if (h->entropy_coder_type == ENTROPY_SA) {
        bw_append_bits_u64(header_bw, (uint64_t)h->unary_length_limit, 5);
        bw_append_bits_u64(header_bw, (uint64_t)h->rescaling_counter_size, 3);
        bw_append_bits_u64(header_bw, (uint64_t)h->initial_count_exponent, 3);
        bw_append_bits_u64(header_bw, (uint64_t)h->accumulator_init_constant, 4);
        bw_append_bits_u64(header_bw, (uint64_t)h->accumulator_init_table_flag, 1);

        if (h->accumulator_init_constant == 15) {
            BitWriter temp; bw_init(&temp);
            int z = header_get_z_size(h);
            for (int i = 0; i < z; i++) {
                bw_append_bits_u64(&temp, (uint64_t)h->accumulator_init_table[i], 4);
            }
            bw_pad_to_byte(&temp);
            if (h->accumulator_init_table_flag == ACCU_INIT_TABLE_INCLUDED) {
                bw_append_from_bw(header_bw, &temp);
            } else {
                bw_append_from_bw(optional_bw, &temp);
            }
            bw_free(&temp);
        }
    } else if (h->entropy_coder_type == ENTROPY_HYBRID) {
        bw_append_bits_u64(header_bw, (uint64_t)h->unary_length_limit, 5);
        bw_append_bits_u64(header_bw, (uint64_t)h->rescaling_counter_size, 3);
        bw_append_bits_u64(header_bw, (uint64_t)h->initial_count_exponent, 3);
        bw_append_bits_u64(header_bw, 0, 5);
    } else {
        bw_append_bits_u64(header_bw, 0, 1);
        bw_append_bits_u64(header_bw, (uint64_t)h->block_size, 2);
        bw_append_bits_u64(header_bw, (uint64_t)h->restricted_code_options_flag, 1);
        bw_append_bits_u64(header_bw, (uint64_t)h->reference_sample_interval, 12);
    }
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
    free(p->spectral_bands_used);
    p->spectral_bands_used = NULL;
    free(p->weight_update_scaling_exponent);
    p->weight_update_scaling_exponent = NULL;
    free(p->weight_exponent_offset);
    p->weight_exponent_offset = NULL;

    free(p->absolute_error_limits);
    p->absolute_error_limits = NULL;
    free(p->relative_error_limits);
    p->relative_error_limits = NULL;

    free(p->local_sum);
    p->local_sum = NULL;
    free(p->local_difference_vector);
    p->local_difference_vector = NULL;
    free(p->weight_vector);
    p->weight_vector = NULL;
    free(p->predicted_central_local_difference);
    p->predicted_central_local_difference = NULL;
    free(p->high_resolution_predicted_sample_value);
    p->high_resolution_predicted_sample_value = NULL;
    free(p->double_resolution_predicted_sample_value);
    p->double_resolution_predicted_sample_value = NULL;
    free(p->predicted_sample_value);
    p->predicted_sample_value = NULL;
    free(p->prediction_residual);
    p->prediction_residual = NULL;
    free(p->maximum_error);
    p->maximum_error = NULL;
    free(p->quantizer_index);
    p->quantizer_index = NULL;
    free(p->clipped_quantizer_bin_center);
    p->clipped_quantizer_bin_center = NULL;
    free(p->double_resolution_sample_representative);
    p->double_resolution_sample_representative = NULL;
    free(p->sample_representative);
    p->sample_representative = NULL;
    free(p->double_resolution_prediction_error);
    p->double_resolution_prediction_error = NULL;
    free(p->scaled_prediction_endpoint_difference);
    p->scaled_prediction_endpoint_difference = NULL;
    free(p->mapped_quantizer_index);
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
        p->weight_exponent_offset = (double *)calloc(n, sizeof(double));
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
    p->mapped_quantizer_index = alloc_i64(n3);

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

/* ---------------- Encoders ---------------- */

/* Sample-Adaptive Encoder */

typedef struct {
    Header *header;
    ImageConstants *image_constants;
    int64_t *mapped_quantizer_index;

    int unary_length_limit;
    int rescaling_counter_size;
    int initial_count_exponent;

    int64_t *accumulator_init_parameter_1;
    int64_t *accumulator_init_parameter_2;

    int64_t *accumulator; /* [y][x][z] */
    int64_t *counter; /* [y][x] */
    int64_t *variable_length_code; /* [y][x][z] */

    BitWriter bitstream;
} SampleAdaptiveEncoder;

static void sa_free(SampleAdaptiveEncoder *enc) {
    if (!enc) return;
    free(enc->accumulator_init_parameter_1);
    enc->accumulator_init_parameter_1 = NULL;
    free(enc->accumulator_init_parameter_2);
    enc->accumulator_init_parameter_2 = NULL;
    free(enc->accumulator);
    enc->accumulator = NULL;
    free(enc->counter);
    enc->counter = NULL;
    free(enc->variable_length_code);
    enc->variable_length_code = NULL;
    bw_free(&enc->bitstream);
}

static void sa_init(SampleAdaptiveEncoder *enc, Header *h, ImageConstants *ic, int64_t *mqi) {
    memset(enc, 0, sizeof(*enc));
    enc->header = h;
    enc->image_constants = ic;
    enc->mapped_quantizer_index = mqi;
    bw_init(&enc->bitstream);
}

static int sa_init_constants(SampleAdaptiveEncoder *enc) {
    Header *h = enc->header;
    enc->unary_length_limit = h->unary_length_limit + 32 * (h->unary_length_limit == 0);
    enc->rescaling_counter_size = h->rescaling_counter_size + 4;
    enc->initial_count_exponent = h->initial_count_exponent + 8 * (h->initial_count_exponent == 0);

    int z = header_get_z_size(h);
    enc->accumulator_init_parameter_2 = alloc_i64((size_t)z);
    enc->accumulator_init_parameter_1 = alloc_i64((size_t)z);
    if (!enc->accumulator_init_parameter_1 || !enc->accumulator_init_parameter_2) return -1;

    for (int i = 0; i < z; i++) {
        enc->accumulator_init_parameter_2[i] = h->accumulator_init_table[i];
        if (enc->accumulator_init_parameter_2[i] <= 30 - enc->image_constants->dynamic_range_bits) {
            enc->accumulator_init_parameter_1[i] = enc->accumulator_init_parameter_2[i];
        } else {
            enc->accumulator_init_parameter_1[i] = 2 * enc->accumulator_init_parameter_2[i] +
                enc->image_constants->dynamic_range_bits - 30;
        }
    }
    return 0;
}

static int sa_init_arrays(SampleAdaptiveEncoder *enc) {
    Header *h = enc->header;
    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);

    size_t n3 = (size_t)x * (size_t)y * (size_t)z;
    size_t n2 = (size_t)x * (size_t)y;

    enc->accumulator = alloc_i64(n3);
    enc->counter = alloc_i64(n2);
    enc->variable_length_code = alloc_i64(n3);
    if (!enc->accumulator || !enc->counter || !enc->variable_length_code) return -1;

    if (y > 1 || x > 1) {
        int x_t1 = (x == 1) ? 0 : 1;
        int y_t1 = (x == 1) ? 1 : 0;
        enc->counter[y_t1 * x + x_t1] = (int64_t)1 << enc->initial_count_exponent;
        enc->accumulator[idx3(y_t1, x_t1, 0, x, z)] =
            (int64_t)floor((3 * ((int64_t)1 << (enc->accumulator_init_parameter_1[0] + 6)) - 49) *
                enc->counter[y_t1 * x + x_t1] / (double)(1 << 7));
    }

    return 0;
}

static void sa_find_code_length(SampleAdaptiveEncoder *enc, int x, int y, int z) {
    Header *h = enc->header;
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);
    size_t idx = idx3(y, x, z, x_size, z_size);

    int64_t counter = enc->counter[y * x_size + x];
    int64_t acc = enc->accumulator[idx];

    if (2 * counter > acc + counter * 49 / (1 << 7)) {
        enc->variable_length_code[idx] = 0;
    } else {
        int64_t v_int = (acc + counter * 49 / (1 << 7)) / counter;
        double v = log2((double)v_int);
        int64_t k = (int64_t)fmin(v, enc->image_constants->dynamic_range_bits - 2);
        enc->variable_length_code[idx] = k;
    }
}

static void sa_gpo2(BitWriter *bw, int64_t j, int64_t k, int dyn_bits, int unary_limit) {
    int64_t zeros = (k == 0) ? j : (j >> k);
    if (zeros < unary_limit) {
        for (int64_t i = 0; i < zeros; i++) bw_append_bit(bw, 0);
        bw_append_bit(bw, 1);
        if (k > 0) {
            bw_append_bits_u64(bw, (uint64_t)j & ((1ULL << k) - 1), (int)k);
        }
    } else {
        for (int i = 0; i < unary_limit; i++) bw_append_bit(bw, 0);
        bw_append_bits_u64(bw, (uint64_t)j, dyn_bits);
    }
}

static void sa_encode_error_limits(SampleAdaptiveEncoder *enc, int y) {
    Header *h = enc->header;
    int z = header_get_z_size(h);
    int period_index = y / (1 << h->error_update_period_exponent);

    if (h->quantizer_fidelity_control_method != QF_REL) {
        int bit_depth = header_get_absolute_error_limit_bit_depth_value(h);
        if (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
            bw_append_bits_u64(&enc->bitstream, (uint64_t)h->periodic_absolute_error_limit_table[period_index * z], bit_depth);
        } else {
            for (int zi = 0; zi < z; zi++) {
                bw_append_bits_u64(&enc->bitstream, (uint64_t)h->periodic_absolute_error_limit_table[period_index * z + zi], bit_depth);
            }
        }
    }

    if (h->quantizer_fidelity_control_method != QF_ABS) {
        int bit_depth = header_get_relative_error_limit_bit_depth_value(h);
        if (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
            bw_append_bits_u64(&enc->bitstream, (uint64_t)h->periodic_relative_error_limit_table[period_index * z], bit_depth);
        } else {
            for (int zi = 0; zi < z; zi++) {
                bw_append_bits_u64(&enc->bitstream, (uint64_t)h->periodic_relative_error_limit_table[period_index * z + zi], bit_depth);
            }
        }
    }
}

static void sa_encode_sample(SampleAdaptiveEncoder *enc, int x, int y, int z) {
    Header *h = enc->header;
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);
    size_t idx = idx3(y, x, z, x_size, z_size);

    if (y == 0 && x == 0) {
        bw_append_bits_u64(&enc->bitstream, (uint64_t)enc->mapped_quantizer_index[idx], enc->image_constants->dynamic_range_bits);
        return;
    }

    int prev_y = y;
    int prev_x = x - 1;
    if (prev_x < 0) {
        prev_y -= 1;
        prev_x = x_size - 1;
    }
    int t = y * x_size + x;

    if (t == 1) {
        /* no op */
    } else if (enc->counter[prev_y * x_size + prev_x] == ((int64_t)1 << enc->rescaling_counter_size) - 1) {
        enc->counter[y * x_size + x] = (enc->counter[prev_y * x_size + prev_x] + 1) / 2;
        enc->accumulator[idx] = (enc->accumulator[idx3(prev_y, prev_x, z, x_size, z_size)] + enc->mapped_quantizer_index[idx3(prev_y, prev_x, z, x_size, z_size)] + 1) / 2;
    } else {
        enc->counter[y * x_size + x] = enc->counter[prev_y * x_size + prev_x] + 1;
        enc->accumulator[idx] = enc->accumulator[idx3(prev_y, prev_x, z, x_size, z_size)] + enc->mapped_quantizer_index[idx3(prev_y, prev_x, z, x_size, z_size)];
    }

    sa_find_code_length(enc, x, y, z);
    sa_gpo2(&enc->bitstream, enc->mapped_quantizer_index[idx], enc->variable_length_code[idx], enc->image_constants->dynamic_range_bits, enc->unary_length_limit);
}

static int sa_run(SampleAdaptiveEncoder *enc) {
    if (sa_init_constants(enc) != 0) return -1;
    if (sa_init_arrays(enc) != 0) return -1;

    Header *h = enc->header;
    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);

    if (h->sample_encoding_order == ORDER_BI) {
        for (int yi = 0; yi < y; yi++) {
            if (h->periodic_error_updating_flag == PEU_USED &&
                (yi % (1 << h->error_update_period_exponent) == 0)) {
                sa_encode_error_limits(enc, yi);
            }
            for (int i = 0; i < (z + h->sub_frame_interleaving_depth - 1) / h->sub_frame_interleaving_depth; i++) {
                for (int xi = 0; xi < x; xi++) {
                    int z_start = i * h->sub_frame_interleaving_depth;
                    int z_end = z_start + h->sub_frame_interleaving_depth;
                    if (z_end > z) z_end = z;
                    for (int zi = z_start; zi < z_end; zi++) {
                        sa_encode_sample(enc, xi, yi, zi);
                    }
                }
            }
        }
    } else {
        for (int zi = 0; zi < z; zi++) {
            for (int yi = 0; yi < y; yi++) {
                for (int xi = 0; xi < x; xi++) {
                    sa_encode_sample(enc, xi, yi, zi);
                }
            }
        }
    }

    return 0;
}

/* Hybrid encoder */

typedef struct {
    Header *header;
    ImageConstants *image_constants;
    int64_t *mapped_quantizer_index;

    int unary_length_limit;
    int rescaling_counter_size;
    int initial_count_exponent;

    int64_t *accumulator; /* [y][x][z] */
    int64_t *counter; /* [y][x] */

    BitWriter bitstream;
} HybridEncoder;

static void hyb_free(HybridEncoder *enc) {
    if (!enc) return;
    free(enc->accumulator);
    enc->accumulator = NULL;
    free(enc->counter);
    enc->counter = NULL;
    bw_free(&enc->bitstream);
}

static void hyb_init(HybridEncoder *enc, Header *h, ImageConstants *ic, int64_t *mqi) {
    memset(enc, 0, sizeof(*enc));
    enc->header = h;
    enc->image_constants = ic;
    enc->mapped_quantizer_index = mqi;
    bw_init(&enc->bitstream);
}

static int hyb_init_constants(HybridEncoder *enc) {
    Header *h = enc->header;
    enc->unary_length_limit = h->unary_length_limit + 32 * (h->unary_length_limit == 0);
    enc->rescaling_counter_size = h->rescaling_counter_size + 4;
    enc->initial_count_exponent = h->initial_count_exponent + 8 * (h->initial_count_exponent == 0);
    return 0;
}

static int hyb_init_arrays(HybridEncoder *enc) {
    Header *h = enc->header;
    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);

    size_t n3 = (size_t)x * (size_t)y * (size_t)z;
    size_t n2 = (size_t)x * (size_t)y;

    enc->accumulator = alloc_i64(n3);
    enc->counter = alloc_i64(n2);
    if (!enc->accumulator || !enc->counter) return -1;

    enc->counter[0] = (int64_t)1 << enc->initial_count_exponent;
    for (int zi = 0; zi < z; zi++) {
        enc->accumulator[idx3(0, 0, zi, x, z)] = 4 * enc->counter[0];
    }
    return 0;
}

static void hyb_append_codeword(BitWriter *bw, const char *codeword) {
    /* codeword like 8'h29 */
    const char *hpos = strstr(codeword, "'h");
    if (!hpos) return;
    int width = atoi(codeword);
    const char *hex = hpos + 2;
    char binbuf[1024];
    binbuf[0] = '\0';

    char temp[8];
    for (const char *p = hex; *p; p++) {
        char c = *p;
        int v = 0;
        if (c >= '0' && c <= '9') v = c - '0';
        else if (c >= 'a' && c <= 'f') v = 10 + (c - 'a');
        else if (c >= 'A' && c <= 'F') v = 10 + (c - 'A');
        else continue;
        for (int i = 3; i >= 0; i--) {
            temp[3 - i] = ((v >> i) & 1) ? '1' : '0';
        }
        temp[4] = '\0';
        strncat(binbuf, temp, sizeof(binbuf) - strlen(binbuf) - 1);
    }

    int binlen = (int)strlen(binbuf);
    if (binlen < width) {
        int pad = width - binlen;
        for (int i = 0; i < pad; i++) bw_append_bit(bw, 0);
        bw_append_bits_str(bw, binbuf);
    } else {
        const char *start = binbuf + (binlen - width);
        bw_append_bits_str(bw, start);
    }
}

static void hyb_reverse_gpo2(BitWriter *bw, int64_t j, int64_t k, int dyn_bits, int unary_limit) {
    int64_t zeros = (k == 0) ? j : (j >> k);
    if (zeros < unary_limit) {
        if (k == 0) {
            bw_append_bit(bw, 1);
            for (int64_t i = 0; i < zeros; i++) bw_append_bit(bw, 0);
        } else {
            bw_append_bits_u64(bw, (uint64_t)j & ((1ULL << k) - 1), (int)k);
            bw_append_bit(bw, 1);
            for (int64_t i = 0; i < zeros; i++) bw_append_bit(bw, 0);
        }
    } else {
        bw_append_bits_u64(bw, (uint64_t)j, dyn_bits);
        for (int i = 0; i < unary_limit; i++) bw_append_bit(bw, 0);
    }
}

static void hyb_encode_error_limits(HybridEncoder *enc, int y) {
    Header *h = enc->header;
    int z = header_get_z_size(h);
    int period_index = y / (1 << h->error_update_period_exponent);

    if (h->quantizer_fidelity_control_method != QF_REL) {
        int bit_depth = header_get_absolute_error_limit_bit_depth_value(h);
        if (h->absolute_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
            bw_append_bits_u64(&enc->bitstream, (uint64_t)h->periodic_absolute_error_limit_table[period_index * z], bit_depth);
        } else {
            for (int zi = 0; zi < z; zi++) {
                bw_append_bits_u64(&enc->bitstream, (uint64_t)h->periodic_absolute_error_limit_table[period_index * z + zi], bit_depth);
            }
        }
    }

    if (h->quantizer_fidelity_control_method != QF_ABS) {
        int bit_depth = header_get_relative_error_limit_bit_depth_value(h);
        if (h->relative_error_limit_assignment_method == ELA_BAND_INDEPENDENT) {
            bw_append_bits_u64(&enc->bitstream, (uint64_t)h->periodic_relative_error_limit_table[period_index * z], bit_depth);
        } else {
            for (int zi = 0; zi < z; zi++) {
                bw_append_bits_u64(&enc->bitstream, (uint64_t)h->periodic_relative_error_limit_table[period_index * z + zi], bit_depth);
            }
        }
    }
}

static void hyb_encode_sample(HybridEncoder *enc, int x, int y, int z, char active_prefix[16][MAX_TABLE_STR]) {
    Header *h = enc->header;
    int x_size = header_get_x_size(h);
    int z_size = header_get_z_size(h);
    size_t idx = idx3(y, x, z, x_size, z_size);

    if (y == 0 && x == 0) {
        bw_append_bits_u64(&enc->bitstream, (uint64_t)enc->mapped_quantizer_index[idx], enc->image_constants->dynamic_range_bits);
        return;
    }

    int prev_y = y;
    int prev_x = x - 1;
    if (prev_x < 0) {
        prev_y -= 1;
        prev_x = x_size - 1;
    }

    if (enc->counter[prev_y * x_size + prev_x] == ((int64_t)1 << enc->rescaling_counter_size) - 1) {
        enc->counter[y * x_size + x] = (enc->counter[prev_y * x_size + prev_x] + 1) / 2;
        enc->accumulator[idx] = (enc->accumulator[idx3(prev_y, prev_x, z, x_size, z_size)] + 4 * enc->mapped_quantizer_index[idx] + 1) / 2;
        int lsb = (int)(enc->accumulator[idx3(prev_y, prev_x, z, x_size, z_size)] & 1);
        bw_append_bit(&enc->bitstream, lsb);
    } else {
        enc->counter[y * x_size + x] = enc->counter[prev_y * x_size + prev_x] + 1;
        enc->accumulator[idx] = enc->accumulator[idx3(prev_y, prev_x, z, x_size, z_size)] + 4 * enc->mapped_quantizer_index[idx];
    }

    if (enc->accumulator[idx] * (1 << 14) >= (int64_t)threshold_table[0] * enc->counter[y * x_size + x]) {
        int64_t v_int = (enc->accumulator[idx] + enc->counter[y * x_size + x] * 49 / (1 << 5)) / enc->counter[y * x_size + x];
        int64_t k = (int64_t)fmin(
            log2((double)v_int) - 2,
            fmax(enc->image_constants->dynamic_range_bits - 2, 2)
        );
        if (k < 2) k = 2;
        hyb_reverse_gpo2(&enc->bitstream, enc->mapped_quantizer_index[idx], k, enc->image_constants->dynamic_range_bits, enc->unary_length_limit);
    } else {
        int code_index = -2;
        for (int i = 15; i >= 0; i--) {
            if (enc->accumulator[idx] * (1 << 14) < (int64_t)enc->counter[y * x_size + x] * threshold_table[i]) {
                code_index = i;
                break;
            }
        }
        char input_symbol = 'F';
        if (enc->mapped_quantizer_index[idx] <= input_symbol_limit[code_index]) {
            int val = (int)enc->mapped_quantizer_index[idx];
            input_symbol = (val < 10) ? ('0' + val) : ('A' + (val - 10));
        } else {
            input_symbol = 'X';
            int64_t residual = enc->mapped_quantizer_index[idx] - input_symbol_limit[code_index] - 1;
            hyb_reverse_gpo2(&enc->bitstream, residual, 0, enc->image_constants->dynamic_range_bits, enc->unary_length_limit);
        }

        size_t len = strlen(active_prefix[code_index]);
        if (len + 1 < MAX_TABLE_STR) {
            active_prefix[code_index][len] = input_symbol;
            active_prefix[code_index][len + 1] = '\0';
        }

        for (int i = 0; i < MAX_TABLE_ENTRIES && code_table_input[code_index][i]; i++) {
            if (strcmp(code_table_input[code_index][i], active_prefix[code_index]) == 0) {
                hyb_append_codeword(&enc->bitstream, code_table_output[code_index][i]);
                active_prefix[code_index][0] = '\0';
                break;
            }
        }
    }
}

static void hyb_encode_image_tail(HybridEncoder *enc, char active_prefix[16][MAX_TABLE_STR]) {
    Header *h = enc->header;
    int z = header_get_z_size(h);

    for (int i = 0; i < 16; i++) {
        int matched = 0;
        for (int j = 0; j < MAX_TABLE_ENTRIES && flush_table_prefix[i][j]; j++) {
            if (strcmp(flush_table_prefix[i][j], active_prefix[i]) == 0) {
                hyb_append_codeword(&enc->bitstream, flush_table_word[i][j]);
                matched = 1;
                break;
            }
        }
        if (!matched) {
            /* if empty prefix, first entry is used */
            if (flush_table_word[i][0]) hyb_append_codeword(&enc->bitstream, flush_table_word[i][0]);
        }
    }

    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    for (int zi = 0; zi < z; zi++) {
        int64_t acc = enc->accumulator[idx3(y - 1, x - 1, zi, x, z)];
        int bits = 2 + enc->image_constants->dynamic_range_bits + enc->rescaling_counter_size;
        bw_append_bits_u64(&enc->bitstream, (uint64_t)acc, bits);
    }
    bw_append_bit(&enc->bitstream, 1);
}

static int hyb_run(HybridEncoder *enc) {
#if defined(HYBRID_TABLES_EXTERNAL)
    if (hyb_load_tables_once() != 0) return -1;
#endif
    if (hyb_init_constants(enc) != 0) return -1;
    if (hyb_init_arrays(enc) != 0) return -1;

    Header *h = enc->header;
    int x = header_get_x_size(h);
    int y = header_get_y_size(h);
    int z = header_get_z_size(h);

    char active_prefix[16][MAX_TABLE_STR];
    for (int i = 0; i < 16; i++) active_prefix[i][0] = '\0';

    if (h->sample_encoding_order == ORDER_BI) {
        for (int yi = 0; yi < y; yi++) {
            if (h->periodic_error_updating_flag == PEU_USED &&
                (yi % (1 << h->error_update_period_exponent) == 0)) {
                hyb_encode_error_limits(enc, yi);
            }
            for (int i = 0; i < (z + h->sub_frame_interleaving_depth - 1) / h->sub_frame_interleaving_depth; i++) {
                for (int xi = 0; xi < x; xi++) {
                    int z_start = i * h->sub_frame_interleaving_depth;
                    int z_end = z_start + h->sub_frame_interleaving_depth;
                    if (z_end > z) z_end = z;
                    for (int zi = z_start; zi < z_end; zi++) {
                        hyb_encode_sample(enc, xi, yi, zi, active_prefix);
                    }
                }
            }
        }
    } else {
        for (int zi = 0; zi < z; zi++) {
            for (int yi = 0; yi < y; yi++) {
                for (int xi = 0; xi < x; xi++) {
                    hyb_encode_sample(enc, xi, yi, zi, active_prefix);
                }
            }
        }
    }

    hyb_encode_image_tail(enc, active_prefix);
    return 0;
}

/* Block-adaptive encoder omitted in this minimal native port (not used in default config). */

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
    free(enc->blocks);
    enc->blocks = NULL;
    free(enc->zero_block_count);
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


static int write_bitstream_with_header(const char *out_dir, Header *h, BitWriter *payload) {
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
    if (build_out_path(out_dir, "output.bin", path, sizeof(path)) != 0) {
        bw_free(&full);
        return -1;
    }
    int rc = bw_write_to_file(&full, path);
    bw_free(&full);
    return rc;
}

static int compress_one_image(const char *raw_path, const char *output_root, int ael,
                              int override_x, int override_y, int override_z, const char *override_dtype) {
    Header h;
    header_init_defaults(&h);
    int result = -1;
    int64_t *image_sample = NULL;
    Predictor pred;
    int pred_ready = 0;
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

    /* load image */
    /* dtype/x/y/z already resolved above */

    if (load_raw_bip(raw_path, dtype, z, y, x, &image_sample) != 0) goto cleanup;

    ImageConstants ic;
    image_constants_init(&ic, &h);

    predictor_init(&pred, &h, &ic, image_sample);
    if (predictor_run(&pred) != 0) {
        fprintf(stderr, "Predictor failed for %s\n", raw_path);
        goto cleanup;
    }
    pred_ready = 1;

    if (h.entropy_coder_type == ENTROPY_HYBRID) {
        HybridEncoder enc;
        hyb_init(&enc, &h, &ic, pred.mapped_quantizer_index);
        if (hyb_run(&enc) != 0) {
            fprintf(stderr, "Hybrid encoder failed for %s\n", raw_path);
            hyb_free(&enc);
            goto cleanup;
        }
        if (write_bitstream_with_header(out_dir, &h, &enc.bitstream) != 0) {
            fprintf(stderr, "Failed writing bitstream for %s\n", raw_path);
            hyb_free(&enc);
            goto cleanup;
        }
        hyb_free(&enc);
    } else if (h.entropy_coder_type == ENTROPY_SA) {
        SampleAdaptiveEncoder enc;
        sa_init(&enc, &h, &ic, pred.mapped_quantizer_index);
        if (sa_run(&enc) != 0) {
            fprintf(stderr, "Sample-adaptive encoder failed for %s\n", raw_path);
            sa_free(&enc);
            goto cleanup;
        }
        if (write_bitstream_with_header(out_dir, &h, &enc.bitstream) != 0) {
            fprintf(stderr, "Failed writing bitstream for %s\n", raw_path);
            sa_free(&enc);
            goto cleanup;
        }
        sa_free(&enc);
    } else {
        BlockAdaptiveEncoder enc;
        ba_init(&enc, &h, &ic, pred.mapped_quantizer_index);
        if (ba_run(&enc) != 0) {
            fprintf(stderr, "Block-adaptive encoder failed for %s\n", raw_path);
            ba_free(&enc);
            goto cleanup;
        }
        if (write_bitstream_with_header(out_dir, &h, &enc.bitstream) != 0) {
            fprintf(stderr, "Failed writing bitstream for %s\n", raw_path);
            ba_free(&enc);
            goto cleanup;
        }
        ba_free(&enc);
    }

    result = 0;

cleanup:
    if (pred_ready) predictor_free(&pred);
    free(image_sample);
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
    return compress_one_image(raw_path, output_root, ael, override_x, override_y, override_z, override_dtype);
}

int ccsds123_ends_with_raw(const char *name) {
    return ends_with_raw(name);
}

#ifndef UNIT_TEST
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

    if (!ends_with_raw(input_file)) {
        fprintf(stderr, "Input must be a .raw file: %s\n", input_file);
        return 2;
    }

    if (ensure_dir(output_dir) != 0) return 1;

    printf("[ccsds123.0-b-2] %s (AEL=%d)\n", input_file, ael);
    if (compress_one_image(input_file, output_dir, ael, override_x, override_y, override_z, override_dtype) != 0) {
        fprintf(stderr, "Failed: %s\n", input_file);
        return 1;
    }

    char out_dir[MAX_PATH_LEN];
    build_output_folder_path(output_dir, input_file, ael, out_dir);

    char bitstream_path[MAX_PATH_LEN];
    if (build_out_path(out_dir, "output.bin", bitstream_path, sizeof(bitstream_path)) != 0) {
        fprintf(stderr, "Warning: could not compute compression factor.\n");
        printf("Done compressing.\n");
        return 0;
    }

    long long in_size = 0, out_size = 0;
    if (get_file_size(input_file, &in_size) == 0 && get_file_size(bitstream_path, &out_size) == 0 && out_size > 0) {
        double factor = (double)in_size / (double)out_size;
        printf("Compression factor: %.4f (input %lld bytes, output %lld bytes)\n", factor, in_size, out_size);
    } else {
        fprintf(stderr, "Warning: could not compute compression factor.\n");
    }

    printf("Done compressing.\n");
    return 0;
}
#endif
