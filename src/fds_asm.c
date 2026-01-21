/*
 * fds_asm.c - Voyager FDS Assembler
 *
 * Assembles FDS assembly source into binary.
 * Based on Figure 2 of JPL Memo MJS 2.64A, 7 October 1974.
 *
 * Instruction encoding reference (Figure 2):
 *
 * SIMPLE ADDRESS INSTRUCTIONS (12-bit address):
 *   JMP addr    0000 aaaa aaaa aaaa    Jump
 *   SRB addr    0001 aaaa aaaa aaaa    Save RB, Jump (call)
 *   EXC addr    0010 aaaa aaaa aaaa    Execute instruction at addr
 *   WAT data    0011 dddd dddd dddd    Wait (cycle count)
 *   MLD addr    0100 aaaa aaaa aaaa    Load Memory -> RA
 *   MRD addr    0101 aaaa aaaa aaaa    Store RA -> Memory
 *   ABS addr    1000 aaaa aaaa aaaa    Absolute entry (indirect jump)
 *
 * ALU OPERATIONS (opcode 1001):
 *   ADD         1001 00bb b00r rrrr    RA <- RA + RB
 *   LXR         1001 00bb b01r rrrr    RA <- RA XOR RB
 *   AND         1001 00bb b10r rrrr    RA <- RA AND RB
 *   LOR         1001 00bb b11r rrrr    RA <- RA OR RB
 *   SUB         1001 01bb b0rr rrrr    RA <- RA - RB
 *
 * SKIP INSTRUCTIONS (opcode 1001 11xx):
 *   SKP  [mem]  1001 1100 000m mmmm    Skip on Positive
 *   ISP  [mem]  1001 1100 001m mmmm    Inc & Skip on Positive
 *   DSP  [mem]  1001 1100 010m mmmm    Dec & Skip on Positive
 *   SKZ  [mem]  1001 1100 011m mmmm    Skip on Zero
 *   ISZ  [mem]  1001 1100 100m mmmm    Inc & Skip on Zero
 *   DSZ  [mem]  1001 1100 101m mmmm    Dec & Skip on Zero
 *   SKO         1001 1110 xxxx xxxx    Skip on Overflow
 *   SKC         1001 1100 1xxx xxxx    Skip on Carry
 *
 *   NOTE: mem field may be 7-bit address into special registers (F80-FFF)
 *   or may operate on RA. Need FF43 docs to verify.
 *
 * SHIFT OPERATIONS:
 *   SRS cnt     1001 01cc c1rr rrrr    Short Right Shift
 *   SLS cnt     1001 01cc c0rr rrrr    Short Left Shift  (verify)
 *   SRR cnt     1001 01cc c1rr rrrr    Short Right Rotate (verify)
 *   ARS         1110 xxxx xxxx xxxx    Short Arith Right Shift
 *   LRS         1110 xxxx xxxx xxxx    Long Right Shift
 *   LLS         1101 xxxx xxxx xxxx    Long Left Shift
 *   LRR         1100 xxxx xxxx xxxx    Long Right Rotate
 *
 * AUTO-INDEX MEMORY (opcode 1010):
 *   AML mp      1010 0xxx xxmm mmmm    Auto Index Memory Load
 *   AMR mp      1010 1xxx xxmm mmmm    Auto Index Memory Read
 *
 * COMPARE/CONDITIONAL (opcode 1011):
 *   MCX         1011 0xxx xxxx xxxx    Conditional Mod Index
 *   SKE         1011 1xxx xxxx xxxx    Skip if Equal
 *
 * I/O OPERATIONS (opcode 0110, 0111):
 *   SWI         0110 xxxx xxxx xxxx    Serial Data In
 *   SWO         0111 0xxx xxxx xxxx    Serial Data Out
 *   PWD         0111 1xxx xxxx xxxx    Parallel Word Transfer
 *
 * DISCRETE OUTPUT (opcode 1111):
 *   OUT         1111 xxxx xxxx xxxx    Discrete Output
 *
 * SLC (opcode 1001 10xx):
 *   SLC         1001 10xx xxxx xxxx    Skip on Line Count
 */

#include "../include/fds.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* ============================================================================
 * ASSEMBLER CONSTANTS
 * ============================================================================ */

#define MAX_LINE_LEN     256
#define MAX_LABELS       256
#define MAX_PROGRAM_SIZE 4096

/* ============================================================================
 * INSTRUCTION TABLE
 *
 * Based on Figure 2, JPL Memo MJS 2.64A
 * ============================================================================ */

typedef enum {
    ARG_NONE,        /* No argument */
    ARG_ADDR12,      /* 12-bit address */
    ARG_ADDR7,       /* 7-bit address (special registers) */
    ARG_REG,         /* Register number */
    ARG_COUNT,       /* Shift count */
    ARG_DATA,        /* Immediate data */
} arg_type_t;

typedef struct {
    const char *mnemonic;
    uint16_t opcode;     /* Base opcode (bits to set) */
    uint16_t mask;       /* Mask for opcode bits */
    arg_type_t arg_type;
    int cycles;          /* From Figure 2 */
    const char *desc;
} instr_def_t;

/*
 * Instruction definitions from Figure 2.
 * Format: { mnemonic, base_opcode, mask, arg_type, cycles, description }
 *
 * The mask indicates which bits are fixed by the opcode.
 * Remaining bits are filled by arguments.
 */
static const instr_def_t instructions[] = {
    /* Simple address instructions */
    { "JMP",  0x0000, 0xF000, ARG_ADDR12, 2,    "Jump" },
    { "SRB",  0x1000, 0xF000, ARG_ADDR12, 3,    "Save RB, Jump (call)" },
    { "EXC",  0x2000, 0xF000, ARG_ADDR12, 2,    "Execute at address" },
    { "WAT",  0x3000, 0xF000, ARG_DATA,   4097, "Wait" },
    { "MLD",  0x4000, 0xF000, ARG_ADDR12, 4,    "Load Memory to RA" },
    { "MRD",  0x5000, 0xF000, ARG_ADDR12, 4,    "Store RA to Memory" },
    { "ABS",  0x8000, 0xF000, ARG_ADDR12, 3,    "Absolute entry (indirect jump)" },

    /* ALU operations - simple forms (operate on RA, RB) */
    /* Full encoding: 1001 00bb b[op]r rrrr - we simplify to no args */
    { "ADD",  0x9000, 0xFFFF, ARG_NONE,   6,    "RA <- RA + RB" },
    { "LXR",  0x9010, 0xFFFF, ARG_NONE,   6,    "RA <- RA XOR RB" },
    { "AND",  0x9020, 0xFFFF, ARG_NONE,   6,    "RA <- RA AND RB" },
    { "LOR",  0x9030, 0xFFFF, ARG_NONE,   6,    "RA <- RA OR RB" },
    { "SUB",  0x9400, 0xFFFF, ARG_NONE,   6,    "RA <- RA - RB" },

    /* Skip instructions */
    /* Encoding: 1001 11[type]m mmmm mmmm */
    /* For now, simplified to operate on RA (mem field = 0) */
    { "SKP",  0x9C00, 0xFFFF, ARG_NONE,   3,    "Skip on Positive" },
    { "ISP",  0x9C10, 0xFFFF, ARG_NONE,   4,    "Inc & Skip on Positive" },
    { "DSP",  0x9C20, 0xFFFF, ARG_NONE,   4,    "Dec & Skip on Positive" },
    { "SKZ",  0x9C30, 0xFFFF, ARG_NONE,   3,    "Skip on Zero" },
    { "ISZ",  0x9C40, 0xFFFF, ARG_NONE,   4,    "Inc & Skip on Zero" },
    { "DSZ",  0x9C50, 0xFFFF, ARG_NONE,   4,    "Dec & Skip on Zero" },
    { "SKO",  0x9E00, 0xFF00, ARG_NONE,   3,    "Skip on Overflow" },
    { "SKC",  0x9C80, 0xFF80, ARG_NONE,   3,    "Skip on Carry" },

    /* Skip on Line Count */
    { "SLC",  0x9800, 0xFC00, ARG_DATA,   6,    "Skip on Line Count" },

    /* Auto-index memory operations */
    { "AML",  0xA000, 0xF800, ARG_ADDR7,  6,    "Auto Index Memory Load" },
    { "AMR",  0xA800, 0xF800, ARG_ADDR7,  6,    "Auto Index Memory Read" },

    /* Conditional/compare */
    { "MCX",  0xB000, 0xF800, ARG_DATA,   4,    "Conditional Mod Index" },
    { "SKE",  0xB800, 0xF800, ARG_DATA,   3,    "Skip if Equal" },

    /* Long shifts - encoding needs verification from docs */
    { "LRR",  0xC000, 0xF000, ARG_DATA,   9,    "Long Right Rotate" },
    { "LLS",  0xD000, 0xF000, ARG_DATA,   9,    "Long Left Shift" },
    { "LRS",  0xE000, 0xF000, ARG_DATA,   9,    "Long Right Shift" },
    { "ARS",  0xE000, 0xF000, ARG_DATA,   6,    "Short Arith Right Shift" },

    /* Short shifts - encoded within 1001 space, need verification */
    { "SRS",  0x9480, 0xFFC0, ARG_COUNT,  6,    "Short Right Shift" },
    { "SLS",  0x9440, 0xFFC0, ARG_COUNT,  6,    "Short Left Shift" },
    { "SRR",  0x94C0, 0xFFC0, ARG_COUNT,  6,    "Short Right Rotate" },

    /* I/O operations - complex encoding, simplified here */
    { "SWI",  0x6000, 0xF000, ARG_DATA,   11,   "Serial Data In" },
    { "SWO",  0x7000, 0xF800, ARG_DATA,   11,   "Serial Data Out" },
    { "PWD",  0x7800, 0xF800, ARG_DATA,   4,    "Parallel Word Transfer" },

    /* Discrete output */
    { "OUT",  0xF000, 0xF000, ARG_DATA,   3,    "Discrete Output" },

    /* Memory banking (special OUT instructions - per MJS77-4-2006-1A) */
    { "SETJU", 0xF001, 0xFFFF, ARG_NONE,  3,    "Set Jump Upper (next JMP to upper 4K)" },
    { "SETJD", 0xF002, 0xFFFF, ARG_NONE,  3,    "Set Jump Down (JMPs to lower 4K)" },
    { "SETAU", 0xF003, 0xFFFF, ARG_NONE,  3,    "Set Address Upper (data refs upper 4K)" },
    { "SETAD", 0xF004, 0xFFFF, ARG_NONE,  3,    "Set Address Down (data refs lower 4K)" },

    /* End marker */
    { NULL, 0, 0, ARG_NONE, 0, NULL }
};

/* ============================================================================
 * LABEL TABLE
 * ============================================================================ */

typedef struct {
    char name[32];
    uint16_t addr;
    bool defined;
} label_t;

typedef struct {
    label_t labels[MAX_LABELS];
    int count;
} label_table_t;

/* Forward reference for second pass */
typedef struct {
    uint16_t addr;         /* Address of instruction */
    char label[32];        /* Label referenced */
    int line_num;          /* Source line for error reporting */
} forward_ref_t;

/* ============================================================================
 * ASSEMBLER STATE
 * ============================================================================ */

typedef struct {
    uint16_t code[MAX_PROGRAM_SIZE];
    uint16_t pc;               /* Current program counter */
    uint16_t origin;           /* Origin address */
    label_table_t labels;
    forward_ref_t forwards[MAX_LABELS];
    int forward_count;
    int line_num;
    fds_asm_error_t error;
    char error_msg[256];
} asm_state_t;

/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */

static void set_error(asm_state_t *state, fds_asm_error_t err, const char *msg)
{
    state->error = err;
    snprintf(state->error_msg, sizeof(state->error_msg),
             "Line %d: %s", state->line_num, msg);
}

/* Skip whitespace */
static const char *skip_ws(const char *p)
{
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    return p;
}

/* Parse identifier (label or mnemonic) */
static const char *parse_ident(const char *p, char *buf, size_t buflen)
{
    size_t i = 0;
    while (*p && (isalnum((unsigned char)*p) || *p == '_') && i < buflen - 1) {
        buf[i++] = toupper((unsigned char)*p);
        p++;
    }
    buf[i] = '\0';
    return p;
}

/* Parse number (hex or decimal) */
static const char *parse_number(const char *p, uint16_t *value, bool *valid)
{
    *valid = false;
    p = skip_ws(p);

    if (*p == '\0' || *p == ';') {
        return p;
    }

    char *end;
    long val;

    if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
        /* Hex: 0xNNN */
        val = strtol(p, &end, 16);
    } else if (p[0] == '$') {
        /* Hex: $NNN */
        val = strtol(p + 1, &end, 16);
        if (end == p + 1) {
            return p;  /* No digits after $ */
        }
    } else if (isdigit((unsigned char)*p)) {
        /* Decimal or hex with H suffix */
        val = strtol(p, &end, 10);
        if (*end == 'H' || *end == 'h') {
            /* It was hex, re-parse */
            val = strtol(p, &end, 16);
            end++;  /* Skip the H */
        }
    } else {
        return p;  /* Not a number */
    }

    if (end != p) {
        *value = (uint16_t)(val & 0xFFFF);
        *valid = true;
        return end;
    }
    return p;
}

/* Look up label */
static int find_label(asm_state_t *state, const char *name)
{
    for (int i = 0; i < state->labels.count; i++) {
        if (strcmp(state->labels.labels[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

/* Add or update label */
static int add_label(asm_state_t *state, const char *name, uint16_t addr, bool define)
{
    int idx = find_label(state, name);

    if (idx >= 0) {
        /* Existing label */
        if (define) {
            if (state->labels.labels[idx].defined) {
                set_error(state, FDS_ASM_ERR_DUPLICATE_LABEL, "Duplicate label");
                return -1;
            }
            state->labels.labels[idx].addr = addr;
            state->labels.labels[idx].defined = true;
        }
        return idx;
    }

    /* New label */
    if (state->labels.count >= MAX_LABELS) {
        set_error(state, FDS_ASM_ERR_SYNTAX, "Too many labels");
        return -1;
    }

    idx = state->labels.count++;
    strncpy(state->labels.labels[idx].name, name,
            sizeof(state->labels.labels[idx].name) - 1);
    state->labels.labels[idx].addr = addr;
    state->labels.labels[idx].defined = define;

    return idx;
}

/* Find instruction by mnemonic */
static const instr_def_t *find_instruction(const char *mnemonic)
{
    for (int i = 0; instructions[i].mnemonic != NULL; i++) {
        if (strcmp(instructions[i].mnemonic, mnemonic) == 0) {
            return &instructions[i];
        }
    }
    return NULL;
}

/* ============================================================================
 * ASSEMBLER CORE
 * ============================================================================ */

/* Assemble one line */
static int assemble_line(asm_state_t *state, const char *line)
{
    const char *p = line;
    char token[64];

    p = skip_ws(p);

    /* Empty line or comment */
    if (*p == '\0' || *p == ';' || *p == '*') {
        return 0;
    }

    /* Check for label (identifier followed by :) */
    p = parse_ident(p, token, sizeof(token));

    if (token[0] != '\0') {
        const char *after_ident = skip_ws(p);

        if (*after_ident == ':') {
            /* It's a label definition */
            if (add_label(state, token, state->pc, true) < 0) {
                return -1;
            }
            p = after_ident + 1;
            p = skip_ws(p);

            /* Check for instruction on same line */
            if (*p == '\0' || *p == ';') {
                return 0;  /* Label only */
            }

            /* Parse mnemonic */
            p = parse_ident(p, token, sizeof(token));
        } else {
            /* It's a mnemonic (no label) */
            /* token already has the mnemonic */
        }
    } else {
        /* No identifier at start */
        return 0;
    }

    /* Handle directives */
    if (strcmp(token, "ORG") == 0) {
        uint16_t addr;
        bool valid;
        p = parse_number(p, &addr, &valid);
        if (!valid) {
            set_error(state, FDS_ASM_ERR_INVALID_OPERAND, "ORG requires address");
            return -1;
        }
        if (state->pc == 0) {
            state->origin = addr;
        }
        state->pc = addr;
        return 0;
    }

    if (strcmp(token, "DW") == 0 || strcmp(token, "DATA") == 0) {
        /* Data word */
        uint16_t value;
        bool valid;
        p = parse_number(p, &value, &valid);
        if (!valid) {
            set_error(state, FDS_ASM_ERR_INVALID_OPERAND, "DW requires value");
            return -1;
        }
        if (state->pc >= MAX_PROGRAM_SIZE) {
            set_error(state, FDS_ASM_ERR_ADDRESS_RANGE, "Program too large");
            return -1;
        }
        state->code[state->pc++] = value;
        return 0;
    }

    if (strcmp(token, "DS") == 0 || strcmp(token, "RES") == 0) {
        /* Reserve space */
        uint16_t count;
        bool valid;
        p = parse_number(p, &count, &valid);
        if (!valid) {
            count = 1;
        }
        state->pc += count;
        return 0;
    }

    if (strcmp(token, "END") == 0) {
        return 1;  /* End of source */
    }

    /* Look up instruction */
    const instr_def_t *instr = find_instruction(token);
    if (instr == NULL) {
        set_error(state, FDS_ASM_ERR_UNKNOWN_MNEMONIC, token);
        return -1;
    }

    /* Parse operand if needed */
    uint16_t operand = 0;
    p = skip_ws(p);

    if (instr->arg_type != ARG_NONE) {
        /* Try parsing as number first */
        uint16_t value;
        bool valid;
        const char *num_end = parse_number(p, &value, &valid);

        if (valid) {
            operand = value;
            p = num_end;
        } else {
            /* Try parsing as label */
            char label[32];
            p = parse_ident(p, label, sizeof(label));

            if (label[0] == '\0') {
                set_error(state, FDS_ASM_ERR_INVALID_OPERAND,
                          "Expected operand");
                return -1;
            }

            /* Look up label or add forward reference */
            int idx = find_label(state, label);
            if (idx >= 0 && state->labels.labels[idx].defined) {
                operand = state->labels.labels[idx].addr;
            } else {
                /* Forward reference */
                if (idx < 0) {
                    add_label(state, label, 0, false);
                }
                if (state->forward_count >= MAX_LABELS) {
                    set_error(state, FDS_ASM_ERR_SYNTAX,
                              "Too many forward references");
                    return -1;
                }
                state->forwards[state->forward_count].addr = state->pc;
                snprintf(state->forwards[state->forward_count].label,
                         sizeof(state->forwards[state->forward_count].label),
                         "%s", label);
                state->forwards[state->forward_count].line_num = state->line_num;
                state->forward_count++;
                operand = 0;  /* Will be patched later */
            }
        }
    }

    /* Validate operand range */
    switch (instr->arg_type) {
    case ARG_ADDR12:
        if (operand > 0x0FFF) {
            set_error(state, FDS_ASM_ERR_ADDRESS_RANGE,
                      "Address out of 12-bit range");
            return -1;
        }
        break;
    case ARG_ADDR7:
        if (operand > 0x007F) {
            set_error(state, FDS_ASM_ERR_ADDRESS_RANGE,
                      "Address out of 7-bit range");
            return -1;
        }
        break;
    case ARG_COUNT:
        if (operand > 0x003F) {
            set_error(state, FDS_ASM_ERR_ADDRESS_RANGE,
                      "Count out of range");
            return -1;
        }
        break;
    default:
        break;
    }

    /* Encode instruction */
    uint16_t word = instr->opcode;

    switch (instr->arg_type) {
    case ARG_ADDR12:
        word |= (operand & 0x0FFF);
        break;
    case ARG_ADDR7:
        word |= (operand & 0x007F);
        break;
    case ARG_COUNT:
        word |= (operand & 0x003F);
        break;
    case ARG_DATA:
        word |= (operand & (~instr->mask));
        break;
    case ARG_REG:
        /* Register operand - encoded in low bits */
        word |= (operand & 0x001F);
        break;
    case ARG_NONE:
        break;
    }

    /* Store instruction */
    if (state->pc >= MAX_PROGRAM_SIZE) {
        set_error(state, FDS_ASM_ERR_ADDRESS_RANGE, "Program too large");
        return -1;
    }
    state->code[state->pc++] = word;

    return 0;
}

/* Resolve forward references */
static int resolve_forwards(asm_state_t *state)
{
    for (int i = 0; i < state->forward_count; i++) {
        int idx = find_label(state, state->forwards[i].label);

        if (idx < 0 || !state->labels.labels[idx].defined) {
            state->line_num = state->forwards[i].line_num;
            char msg[64];
            snprintf(msg, sizeof(msg), "Undefined label: %s",
                     state->forwards[i].label);
            set_error(state, FDS_ASM_ERR_UNDEFINED_LABEL, msg);
            return -1;
        }

        uint16_t addr = state->labels.labels[idx].addr;
        uint16_t instr_addr = state->forwards[i].addr;

        /* Patch the instruction - assumes 12-bit address in low bits */
        state->code[instr_addr] = (state->code[instr_addr] & 0xF000) |
                                  (addr & 0x0FFF);
    }
    return 0;
}

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

fds_asm_error_t fds_assemble(const char *source, fds_program_t *out)
{
    if (source == NULL || out == NULL) {
        return FDS_ASM_ERR_SYNTAX;
    }

    asm_state_t state = {0};
    state.error = FDS_ASM_OK;

    /* Parse line by line */
    const char *p = source;
    char line[MAX_LINE_LEN];

    while (*p) {
        state.line_num++;

        /* Extract line */
        int i = 0;
        while (*p && *p != '\n' && i < MAX_LINE_LEN - 1) {
            line[i++] = *p++;
        }
        line[i] = '\0';

        if (*p == '\n') {
            p++;
        }

        /* Assemble line */
        int result = assemble_line(&state, line);
        if (result < 0) {
            fprintf(stderr, "%s\n", state.error_msg);
            return state.error;
        }
        if (result > 0) {
            break;  /* END directive */
        }
    }

    /* Resolve forward references */
    if (resolve_forwards(&state) < 0) {
        fprintf(stderr, "%s\n", state.error_msg);
        return state.error;
    }

    /* Copy output */
    out->origin = state.origin;
    out->count = state.pc - state.origin;
    out->code = malloc(out->count * sizeof(uint16_t));
    if (out->code == NULL) {
        return FDS_ASM_ERR_IO;
    }
    memcpy(out->code, &state.code[state.origin], out->count * sizeof(uint16_t));

    return FDS_ASM_OK;
}

fds_asm_error_t fds_assemble_file(const char *filename, fds_program_t *out)
{
    FILE *f = fopen(filename, "r");
    if (f == NULL) {
        return FDS_ASM_ERR_IO;
    }

    /* Read entire file */
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *source = malloc(size + 1);
    if (source == NULL) {
        fclose(f);
        return FDS_ASM_ERR_IO;
    }

    size_t read = fread(source, 1, size, f);
    source[read] = '\0';
    fclose(f);

    fds_asm_error_t result = fds_assemble(source, out);
    free(source);

    return result;
}

void fds_program_free(fds_program_t *prog)
{
    if (prog != NULL && prog->code != NULL) {
        free(prog->code);
        prog->code = NULL;
        prog->count = 0;
    }
}

const char *fds_asm_error_str(fds_asm_error_t err)
{
    switch (err) {
    case FDS_ASM_OK:                  return "OK";
    case FDS_ASM_ERR_SYNTAX:          return "Syntax error";
    case FDS_ASM_ERR_UNKNOWN_MNEMONIC: return "Unknown mnemonic";
    case FDS_ASM_ERR_INVALID_OPERAND: return "Invalid operand";
    case FDS_ASM_ERR_UNDEFINED_LABEL: return "Undefined label";
    case FDS_ASM_ERR_DUPLICATE_LABEL: return "Duplicate label";
    case FDS_ASM_ERR_ADDRESS_RANGE:   return "Address out of range";
    case FDS_ASM_ERR_IO:              return "I/O error";
    default:                          return "Unknown error";
    }
}
