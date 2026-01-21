/*
 * fds.h - Voyager Flight Data Subsystem (FDS) Processor
 *
 * Based on JPL Interoffice Memo MJS 2.64A
 * "MJS FDS Processor Architecture and Instruction Set"
 * J. Wooddell, 7 October 1974
 *
 * This processor flew on Voyager 1 and Voyager 2.
 * Launched 1977. Still operating 2026. Your laptop battery didn't.
 *
 * Currently 24+ billion kilometres away. Response time: 44 hours.
 * Still faster than some enterprise software deployments.
 */

#ifndef FDS_H
#define FDS_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * PROCESSOR CONSTANTS
 *
 * These numbers got humanity to interstellar space. Your smartwatch has
 * better specs but can't find Neptune.
 * ============================================================================ */

#define FDS_MEMORY_SIZE      8192    /* 8K x 16-bit words. All of it. For everything. */
#define FDS_MEMORY_BANK      4096    /* 4K words per bank */
#define FDS_WORD_BITS        16
#define FDS_ADDR_BITS        12      /* 12-bit program address within bank */
#define FDS_ADDR_BITS_EXT    13      /* 13-bit extended address (8K total) */

/* Clock timing - glacial by modern standards, eternal by space standards */
#define FDS_CLOCK_HZ         806400  /* 806.4 kHz. Not a typo. kHz. */
#define FDS_MASTER_OSC_HZ    4838400 /* 4.8384 MHz master oscillator */
#define FDS_MEM_ACCESS_HZ    403200  /* 403.2 kHz memory access rate */
#define FDS_CYCLE_US         2.48    /* microseconds per instruction cycle */
#define FDS_INTERRUPT_MS     2.5     /* For cosmic ray error recovery. Space is rude. */
#define FDS_INTERRUPT_CYCLES 1008    /* Cycles between interrupts (2.5ms / 2.48us) */

/* ============================================================================
 * MEMORY MAP (from FDS Memory Map diagram)
 * ============================================================================ */

/* Program areas */
#define FDS_ADDR_CRS_PROG    0x000   /* CRS Program (70 words) */
#define FDS_ADDR_PRA_PROG    0x046   /* PRA Program (200 words) */
#define FDS_ADDR_LECP_PROG   0x10E   /* LECP Program (150 words) */
#define FDS_ADDR_PPS_PROG    0x1A4   /* PPS Program (50 words) */
#define FDS_ADDR_PLS_PROG    0x1D6   /* PLS Program (300 words) */
#define FDS_ADDR_UVS_PROG    0x302   /* UVS Program (50 words) */
#define FDS_ADDR_MAG_PROG    0x334   /* MAG Program (200 words) */
#define FDS_ADDR_ISS_PROG    0x3FC   /* ISS Program (200 words) */
#define FDS_ADDR_IRIS_PROG   0x4C4   /* IRIS Program (70 words) */
#define FDS_ADDR_UVP_PROG    0x50A   /* UVP Program (150 words) */
#define FDS_ADDR_CCS_CMD     0x5A0   /* CCS Command Program (100 words) */
#define FDS_ADDR_ENG_PROG    0x604   /* ENG Program (100 words) */
#define FDS_ADDR_INTERRUPT   0x668   /* Interrupt Program (40 words) */
#define FDS_ADDR_DMA_GOLAY   0x690   /* DMA & Golay Coder Control (50 words) */

/* Data areas */
#define FDS_ADDR_ENG_IDENT   0x6C2   /* Engineering Identifiers (130 words) */
#define FDS_ADDR_ISS_PARAMS  0x744   /* ISS Picture Parameters (200 words) */
#define FDS_ADDR_SPARE1      0x80C   /* Spare (500 words) */
#define FDS_ADDR_SPARE2      0x9FC   /* Spare (176 words) */
#define FDS_ADDR_ISS_BUFFER  0xAB0   /* ISS Data Buffer (300 words) */
#define FDS_ADDR_PLS_TEMP    0xBDC   /* PLS Temporary Storage (400 words) */
#define FDS_ADDR_PRA_SNAP    0xD6C   /* PRA Snapshot Buffer (200 words) */
#define FDS_ADDR_MAG_PPS_TMP 0xE34   /* MAG & PPS Temporary (50 words) */
#define FDS_ADDR_GEN_SCI_BUF 0xE66   /* General Science Buffer (280 words) */

/* Special registers - top 128 memory locations */
#define FDS_ADDR_SPECIAL_REG 0xF80   /* F80-FFF: Special Registers (128 words) */
#define FDS_NUM_SPECIAL_REG  128

/* ============================================================================
 * REGISTER DEFINITIONS
 *
 * Per the memo:
 * - GR1-GR16:  16 general registers
 * - MP1-MP32:  32 memory pointers
 * - IR1-IR8:   8 index registers
 * - GR1, MP1, and IR1 all refer to the SAME register (it's complicated)
 * - 73 additional registers available as counters
 * - All reside in top 128 memory locations (F80-FFF)
 *
 * The naming scheme makes perfect sense if you were there in 1974.
 * ============================================================================ */

/* General Registers (GR1-GR16) - offsets from F80 */
#define FDS_REG_GR1          0x00    /* Also MP1 and IR1 */
#define FDS_REG_GR2          0x01
#define FDS_REG_GR3          0x02
#define FDS_REG_GR4          0x03
#define FDS_REG_GR5          0x04
#define FDS_REG_GR6          0x05
#define FDS_REG_GR7          0x06
#define FDS_REG_GR8          0x07
#define FDS_REG_GR9          0x08
#define FDS_REG_GR10         0x09
#define FDS_REG_GR11         0x0A
#define FDS_REG_GR12         0x0B
#define FDS_REG_GR13         0x0C
#define FDS_REG_GR14         0x0D
#define FDS_REG_GR15         0x0E
#define FDS_REG_GR16         0x0F

/* Memory Pointers (MP1-MP32) - MP1 overlaps GR1 */
#define FDS_REG_MP1          0x00    /* Same as GR1, IR1 */
/* MP2-MP32 at subsequent locations */

/* Index Registers (IR1-IR8) - IR1 overlaps GR1, MP1 */
#define FDS_REG_IR1          0x00    /* Same as GR1, MP1 */
/* IR2-IR8 at subsequent locations */

/* Line count register (for SLC instruction) */
#define FDS_REG_LINE_COUNT   0x7F    /* Likely location - verify in docs */

/* ============================================================================
 * INSTRUCTION ENCODING (from Figure 2)
 *
 * 16-bit instruction word format varies by instruction type.
 * Bits numbered 16-1 in original doc (1-indexed), we use 15-0 (0-indexed).
 *
 * Opcode is typically bits 15-12 (top 4 bits).
 * ============================================================================ */

/* Opcode field extraction */
#define FDS_OPCODE_MASK      0xF000
#define FDS_OPCODE_SHIFT     12

/* Primary opcodes (bits 15-12) - per JPL MJS 2.64A Figure 2 */
typedef enum {
    FDS_OP_JMP  = 0x0,   /* 0000 - Jump */
    FDS_OP_SRB  = 0x1,   /* 0001 - Save RB Register */
    FDS_OP_EXC  = 0x2,   /* 0010 - Execute */
    FDS_OP_WAT  = 0x3,   /* 0011 - Wait */
    FDS_OP_MLD  = 0x4,   /* 0100 - Load Memory */
    FDS_OP_MRD  = 0x5,   /* 0101 - Read Memory */
    FDS_OP_SWI  = 0x6,   /* 0110 - Serial Data In */
    FDS_OP_SWO  = 0x7,   /* 0111 - Serial Data Out (bit11=0) / Parallel (bit11=1) */
    FDS_OP_ABS  = 0x8,   /* 1000 - Absolute Entry */
    FDS_OP_ALU  = 0x9,   /* 1001 - ALU ops (ADD/LXR/AND/LOR/SUB/SLC/SKP/shifts) */
    FDS_OP_AUTO = 0xA,   /* 1010 - Auto Index Memory (AML bit11=0, AMR bit11=1) */
    FDS_OP_MCX_SKE = 0xB,  /* 1011 - MCX (bit11=0) or SKE (bit11=1) */
    FDS_OP_LRR  = 0xC,   /* 1100 - Long Right Rotate */
    FDS_OP_LLS  = 0xD,   /* 1101 - Long Left Shift */
    FDS_OP_ARS_LRS = 0xE,  /* 1110 - Arithmetic Right Shift (short/long) */
    FDS_OP_OUT  = 0xF,   /* 1111 - Discrete Output */
} fds_opcode_t;

/* ALU sub-operations (when opcode = 1001) */
/* Bits 11-10 and lower bits determine specific operation */
typedef enum {
    FDS_ALU_ADD = 0,     /* Add */
    FDS_ALU_LXR,         /* Exclusive OR */
    FDS_ALU_AND,         /* And */
    FDS_ALU_LOR,         /* Inclusive OR */
    FDS_ALU_SUB,         /* Subtract */
} fds_alu_op_t;

/* Skip sub-operations */
typedef enum {
    FDS_SKIP_POS,        /* SKP - Skip on Positive */
    FDS_SKIP_INC_POS,    /* ISP - Increment & Skip on Positive */
    FDS_SKIP_DEC_POS,    /* DSP - Decrement & Skip on Positive */
    FDS_SKIP_ZERO,       /* SKZ - Skip on Zero */
    FDS_SKIP_INC_ZERO,   /* ISZ - Increment & Skip on Zero */
    FDS_SKIP_DEC_ZERO,   /* DSZ - Decrement & Skip on Zero */
    FDS_SKIP_OVERFLOW,   /* SKO - Skip on Overflow */
    FDS_SKIP_CARRY,      /* SKC - Skip on Carry */
} fds_skip_op_t;

/* Shift sub-operations */
typedef enum {
    FDS_SHIFT_SRS,       /* Short Right Shift */
    FDS_SHIFT_SLS,       /* Short Left Shift */
    FDS_SHIFT_SRR,       /* Short Right Rotate */
    FDS_SHIFT_ARS,       /* Short Arithmetic Right Shift */
    FDS_SHIFT_LRS,       /* Long Right Shift */
    FDS_SHIFT_LLS,       /* Long Left Shift */
    FDS_SHIFT_LRR,       /* Long Right Rotate */
} fds_shift_op_t;

/* ============================================================================
 * CPU STATE
 * ============================================================================ */

/* Status/flag bits */
typedef struct {
    bool carry;          /* Carry flag */
    bool overflow;       /* Overflow flag */
    bool zero;           /* Zero flag (derived from RA) */
    bool positive;       /* Positive flag (derived from RA) */
    bool halted;         /* CPU halted (WAT or error) */
    bool interrupt;      /* Interrupt pending */
} fds_flags_t;

/* Memory banking state (per MJS77-4-2006-1A section 4.2.5.1)
 * Because 4K at a time wasn't enough. So they added complexity.
 * The JPL solution to running out of address bits. */
typedef struct {
    bool jump_upper;     /* SETJU: next JMP goes to upper 4K */
    bool addr_upper;     /* SETAU: absolute addresses in upper 4K */
} fds_bank_t;

/* Main CPU state structure */
typedef struct {
    /* Program-visible registers */
    uint16_t ra;         /* Register A (16-bit working register) */
    uint16_t rb;         /* Register B (16-bit working register) */
    uint16_t ib;         /* Instruction Buffer (16-bit) */
    uint16_t pr;         /* Program Register / Address (12-bit, use lower 12) */

    /* Internal state */
    fds_flags_t flags;   /* Status flags */
    fds_bank_t bank;     /* Memory banking state */
    uint64_t cycles;     /* Cycle counter (for timing) */
    uint64_t cycles_to_interrupt;  /* Cycles until next 2.5ms interrupt */

    /* Memory - 8K x 16-bit words (two 4K banks) */
    /* Top 128 words of each bank (F80-FFF, 1F80-1FFF) are special registers */
    uint16_t memory[FDS_MEMORY_SIZE];

} fds_cpu_t;

/* ============================================================================
 * I/O SUBSYSTEM (per MJS77-4-2006-1A Functional Requirements)
 *
 * The FDS interfaces with:
 * - CCS (Computer Command Subsystem) - commands from Earth
 * - MDS (Modulation Demodulation Subsystem) - telemetry output
 * - DSS (Data Storage Subsystem) - tape recorder
 * - DMA channels (4 total) - high speed block transfer
 *   - MDS, DSS, ISS, PRA channels @ 115.2 kbps each
 *   - Total DMA bandwidth: 379+ kbps
 * - Science Instruments via serial/parallel I/O:
 *   - ISS: Imaging Science Subsystem (NA + WA cameras)
 *   - PRA: Planetary Radio Astronomy
 *   - MAG: Magnetometer
 *   - PLS: Plasma Science
 *   - LECP: Low Energy Charged Particles
 *   - CRS: Cosmic Ray Subsystem
 *   - UVS: Ultraviolet Spectrometer (has FP compression)
 *   - IRIS: Infrared Interferometer Spectrometer
 *   - PPS: Photopolarimeter Subsystem (has FP compression)
 *   - PWS: Plasma Wave Subsystem
 * ============================================================================ */

/* DMA channel definitions (4 channels per MJS77-4-2006-1A) */
#define FDS_DMA_CHANNELS     4
#define FDS_DMA_RATE_BPS     115200   /* 115.2 kbps per channel. Per CHANNEL. */
#define FDS_DMA_TOTAL_BPS    379000   /* Enough to stream Jupiter at very low resolution */

typedef enum {
    FDS_DMA_MDS = 0,     /* Modulation/Demodulation - phone home */
    FDS_DMA_DSS = 1,     /* Data Storage - the tape recorder (yes, tape) */
    FDS_DMA_ISS = 2,     /* Imaging Science - the good stuff */
    FDS_DMA_PRA = 3,     /* Planetary Radio Astronomy - listening to Jupiter hum */
} fds_dma_channel_t;

/* DMA channel state */
typedef struct {
    bool     enabled;        /* Channel on/off */
    bool     direction;      /* 0=input (I/O->mem), 1=output (mem->I/O) */
    uint16_t address;        /* Current memory address (auto-incremented) */
    uint16_t count;          /* Words remaining (for simulation) */
    uint16_t data;           /* Data buffer for current transfer */
    bool     request;        /* DMA request pending from I/O device */
    uint64_t last_transfer;  /* Cycle count of last transfer (for timing) */
} fds_dma_state_t;

/* I/O channel identifiers */
typedef enum {
    FDS_IO_CCS,          /* Computer Command Subsystem */
    FDS_IO_MDS,          /* Modulation/Demodulation (telemetry) */
    FDS_IO_DSS,          /* Data Storage Subsystem (tape) */
    FDS_IO_ISS_NA,       /* Imaging - Narrow Angle camera */
    FDS_IO_ISS_WA,       /* Imaging - Wide Angle camera */
    FDS_IO_PRA,          /* Planetary Radio Astronomy */
    FDS_IO_MAG,          /* Magnetometer */
    FDS_IO_PLS,          /* Plasma Science */
    FDS_IO_LECP,         /* Low Energy Charged Particles */
    FDS_IO_CRS,          /* Cosmic Ray Subsystem */
    FDS_IO_UVS,          /* UV Spectrometer (FP compressed) */
    FDS_IO_IRIS,         /* IR Spectrometer */
    FDS_IO_PPS,          /* Photopolarimeter (FP compressed) */
    FDS_IO_PWS,          /* Plasma Wave Subsystem */
    FDS_IO_GOLAY_A,      /* Golay Coder A */
    FDS_IO_GOLAY_B,      /* Golay Coder B */
    FDS_IO_RS_CODER,     /* Reed-Solomon Coder */
    FDS_IO_ENG,          /* Engineering telemetry tree */
    FDS_IO_COUNT
} fds_io_channel_t;

/* Serial I/O parameters (per MJS 2.64A) */
#define FDS_SERIAL_CLOCK_HZ  14400    /* 14.4 kHz shift clock */
#define FDS_ENG_RATE_BPS     1200     /* Engineering data rate */
#define FDS_ENG_MEASUREMENTS 243      /* Total engineering measurements */

/* I/O channel counts (per MJS 2.64A) */
#define FDS_SERIAL_INPUTS    32       /* 32 serial data inputs (5-bit select) */
#define FDS_SERIAL_OUTPUTS   16       /* 16 serial data outputs (4-bit select) */
#define FDS_PARALLEL_INPUTS  8        /* 8 parallel data inputs */
#define FDS_PARALLEL_OUTPUTS 10       /* 10 parallel data outputs */
#define FDS_DISCRETE_OUTPUTS 512      /* 5-bit subcode for 512 discretes */

/*
 * I/O instruction encoding (12-bit operand field):
 *
 * SWI (0110 xxxx xxxx xxxx) - Serial Word In:
 *   Bits 0-4:  Serial input channel (0-31)
 *   Bits 5-7:  Control (shift count, etc.)
 *   Bits 8-11: Extended control
 *   RA <- serial data from channel
 *
 * SWO (0111 0xxx xxxx xxxx) - Serial Word Out:
 *   Bits 0-3:  Serial output channel (0-15)
 *   Bits 4-7:  Control (shift count, etc.)
 *   Bits 8-10: Extended control
 *   Serial output <- RA
 *
 * PWD (0111 1xxx xxxx xxxx) - Parallel Word Transfer:
 *   Bit 10:    Direction (0=input, 1=output)
 *   Bits 0-3:  Parallel channel (0-7 for input, 0-9 for output)
 *   Bits 4-9:  Control
 *   RA <-> parallel channel
 */
#define FDS_SWI_CHANNEL_MASK   0x001F   /* 5 bits for serial input channel */
#define FDS_SWO_CHANNEL_MASK   0x000F   /* 4 bits for serial output channel */
#define FDS_PWD_CHANNEL_MASK   0x000F   /* 4 bits for parallel channel */
#define FDS_PWD_DIRECTION_BIT  0x0400   /* Bit 10: 0=input, 1=output */

/* Special OUT instruction codes for memory banking (per MJS77-4-2006-1A) */
#define FDS_OUT_SETJU        0x01     /* Set Jump Up - next JMP to upper 4K */
#define FDS_OUT_SETJD        0x02     /* Set Jump Down - JMPs to lower 4K */
#define FDS_OUT_SETAU        0x03     /* Set Address Up - data refs upper 4K */
#define FDS_OUT_SETAD        0x04     /* Set Address Down - data refs lower 4K */

/*
 * DMA Control via OUT instruction (per MJS77-4-2006-1A section 4.2.5.3)
 *
 * DMA channels provide high-speed block transfer through:
 *   Channel 0: MDS (Modulation/Demodulation - telemetry output)
 *   Channel 1: DSS (Data Storage Subsystem - tape recorder)
 *   Channel 2: ISS (Imaging Science Subsystem)
 *   Channel 3: PRA (Planetary Radio Astronomy)
 *
 * Control codes (channel number in bits 0-1):
 *   OUT $08+ch: Enable DMA channel
 *   OUT $0C+ch: Disable DMA channel
 *   OUT $10+ch: Set DMA direction to OUTPUT (memory -> I/O)
 *   OUT $14+ch: Set DMA direction to INPUT (I/O -> memory)
 *   OUT $18+ch: Load DMA address from RA (auto-increment base)
 *
 * DMA transfers occur during "access windows" within instruction execution.
 * Priority: MDS > DSS > ISS > PRA
 */
#define FDS_OUT_DMA_ENABLE   0x08     /* + channel: Enable DMA channel */
#define FDS_OUT_DMA_DISABLE  0x0C     /* + channel: Disable DMA channel */
#define FDS_OUT_DMA_DIR_OUT  0x10     /* + channel: Set direction to output */
#define FDS_OUT_DMA_DIR_IN   0x14     /* + channel: Set direction to input */
#define FDS_OUT_DMA_ADDR     0x18     /* + channel: Load address from RA */

#define FDS_OUT_DMA_CH_MASK  0x03     /* Mask for channel number in OUT code */

/* I/O callback type for extensible I/O handling
 * channel: identifies the I/O channel (serial in, serial out, parallel, etc.)
 * port: the specific port/subaddress within that channel
 * For serial I/O, port is the channel number (0-31 for input, 0-15 for output)
 * For parallel I/O, port is the parallel port number (0-7 in, 0-9 out)
 */
typedef uint16_t (*fds_io_read_fn)(fds_io_channel_t channel, uint16_t port);
typedef void (*fds_io_write_fn)(fds_io_channel_t channel, uint16_t port, uint16_t data);

/* Serial I/O port state (for simulation) */
typedef struct {
    uint16_t data;            /* Current data value */
    bool     ready;           /* Data ready for reading */
    bool     busy;            /* Port is busy (transfer in progress) */
} fds_serial_port_t;

/* Parallel I/O port state */
typedef struct {
    uint16_t data;            /* Current data value */
    bool     ready;           /* Data ready for transfer */
} fds_parallel_port_t;

/* I/O subsystem state */
typedef struct {
    /* Callbacks for external I/O handling */
    fds_io_read_fn read;
    fds_io_write_fn write;
    void *user_data;          /* For callbacks to access external state */

    /* DMA channel states */
    fds_dma_state_t dma[FDS_DMA_CHANNELS];

    /* Serial I/O ports (directly accessible for simulation) */
    fds_serial_port_t serial_in[FDS_SERIAL_INPUTS];    /* 32 serial inputs */
    fds_serial_port_t serial_out[FDS_SERIAL_OUTPUTS];  /* 16 serial outputs */

    /* Parallel I/O ports */
    fds_parallel_port_t parallel_in[FDS_PARALLEL_INPUTS];   /* 8 parallel inputs */
    fds_parallel_port_t parallel_out[FDS_PARALLEL_OUTPUTS]; /* 10 parallel outputs */

    /* SLC (Skip on Line Count) register */
    uint16_t line_count;      /* Line count register for SLC instruction */
} fds_io_t;

/* ============================================================================
 * API FUNCTIONS
 * ============================================================================ */

/* CPU lifecycle */
int  fds_cpu_init(fds_cpu_t *cpu);
void fds_cpu_reset(fds_cpu_t *cpu);

/* I/O subsystem */
void fds_io_init(fds_io_t *io);
void fds_io_set_callbacks(fds_io_t *io, fds_io_read_fn read, fds_io_write_fn write, void *user_data);
void fds_io_set_serial_input(fds_io_t *io, uint8_t channel, uint16_t data);
uint16_t fds_io_get_serial_output(fds_io_t *io, uint8_t channel);
void fds_io_set_parallel_input(fds_io_t *io, uint8_t channel, uint16_t data);
uint16_t fds_io_get_parallel_output(fds_io_t *io, uint8_t channel);

/* DMA subsystem */
void fds_dma_enable(fds_io_t *io, fds_dma_channel_t ch, bool enable);
void fds_dma_set_direction(fds_io_t *io, fds_dma_channel_t ch, bool output);
void fds_dma_set_address(fds_io_t *io, fds_dma_channel_t ch, uint16_t addr);
void fds_dma_request(fds_io_t *io, fds_dma_channel_t ch, uint16_t data);  /* For input */
uint16_t fds_dma_get_output(fds_io_t *io, fds_dma_channel_t ch);          /* For output */
void fds_dma_service(fds_cpu_t *cpu, fds_io_t *io);  /* Service pending DMA requests */

/* Execution */
int  fds_cpu_step(fds_cpu_t *cpu, fds_io_t *io);      /* Execute one instruction */
int  fds_cpu_run(fds_cpu_t *cpu, fds_io_t *io, uint64_t max_cycles);

/* Memory access */
uint16_t fds_mem_read(fds_cpu_t *cpu, uint16_t addr);
void     fds_mem_write(fds_cpu_t *cpu, uint16_t addr, uint16_t value);

/* Program loading */
int  fds_load_binary(fds_cpu_t *cpu, const uint16_t *data, uint16_t addr, uint16_t count);
int  fds_load_file(fds_cpu_t *cpu, const char *filename, uint16_t addr);

/* Register access (special registers at F80-FFF) */
uint16_t fds_reg_read(fds_cpu_t *cpu, uint8_t reg);   /* reg = 0-127 */
void     fds_reg_write(fds_cpu_t *cpu, uint8_t reg, uint16_t value);

/* Debugging */
void fds_cpu_dump(const fds_cpu_t *cpu);
const char *fds_disassemble(uint16_t instruction, char *buf, size_t buflen);

/* ============================================================================
 * ASSEMBLER TYPES
 * ============================================================================ */

/* Assembler error codes */
typedef enum {
    FDS_ASM_OK = 0,
    FDS_ASM_ERR_SYNTAX,
    FDS_ASM_ERR_UNKNOWN_MNEMONIC,
    FDS_ASM_ERR_INVALID_OPERAND,
    FDS_ASM_ERR_UNDEFINED_LABEL,
    FDS_ASM_ERR_DUPLICATE_LABEL,
    FDS_ASM_ERR_ADDRESS_RANGE,
    FDS_ASM_ERR_IO,
} fds_asm_error_t;

/* Assembled output */
typedef struct {
    uint16_t *code;      /* Assembled words */
    uint16_t count;      /* Number of words */
    uint16_t origin;     /* Start address */
} fds_program_t;

/* Assembler API */
fds_asm_error_t fds_assemble(const char *source, fds_program_t *out);
fds_asm_error_t fds_assemble_file(const char *filename, fds_program_t *out);
void fds_program_free(fds_program_t *prog);
const char *fds_asm_error_str(fds_asm_error_t err);

/* Disassembler API */
const char *fds_disassemble(uint16_t word, char *buf, size_t buflen);
void fds_disassemble_program(const uint16_t *code, uint16_t origin, uint16_t count);

#endif /* FDS_H */
