/*
 * fds_cpu.c - Voyager FDS Processor Emulator
 *
 * Emulates the Flight Data Subsystem processor from Voyager 1 & 2.
 * Based on JPL Memo MJS 2.64A, 7 October 1974.
 */

#include "../include/fds.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * INTERNAL HELPERS
 * ============================================================================ */

/* Extract opcode from instruction (bits 15-12) */
static inline uint8_t get_opcode(uint16_t instr)
{
    return (instr >> 12) & 0x0F;
}

/* Extract 12-bit address from instruction (bits 11-0) */
static inline uint16_t get_address(uint16_t instr)
{
    return instr & 0x0FFF;
}

/* Get effective address for data access with banking (MLD, MRD, SRB, EXC) */
static inline uint16_t get_data_address(fds_cpu_t *cpu, uint16_t addr)
{
    /* Special registers (F80-FFF) always in lower bank */
    if (addr >= FDS_ADDR_SPECIAL_REG) {
        return addr;
    }
    /* Apply address banking if SETAU was used */
    if (cpu->bank.addr_upper) {
        return addr + FDS_MEMORY_BANK;  /* Add 0x1000 for upper bank */
    }
    return addr;
}

/* Get effective address for jump with banking */
static inline uint16_t get_jump_address(fds_cpu_t *cpu, uint16_t addr)
{
    if (cpu->bank.jump_upper) {
        return addr + FDS_MEMORY_BANK;  /* Add 0x1000 for upper bank */
    }
    return addr;
}

/* Update zero and positive flags based on RA */
static void update_flags(fds_cpu_t *cpu)
{
    cpu->flags.zero = (cpu->ra == 0);
    cpu->flags.positive = ((cpu->ra & 0x8000) == 0) && (cpu->ra != 0);
}

/* ============================================================================
 * MEMORY ACCESS
 * ============================================================================ */

uint16_t fds_mem_read(fds_cpu_t *cpu, uint16_t addr)
{
    if (addr >= FDS_MEMORY_SIZE) {
        return 0;  /* Out of range */
    }
    return cpu->memory[addr];
}

void fds_mem_write(fds_cpu_t *cpu, uint16_t addr, uint16_t value)
{
    if (addr >= FDS_MEMORY_SIZE) {
        return;  /* Out of range */
    }
    cpu->memory[addr] = value;
}

/* Special register access (F80-FFF mapped to memory) */
uint16_t fds_reg_read(fds_cpu_t *cpu, uint8_t reg)
{
    if (reg >= FDS_NUM_SPECIAL_REG) {
        return 0;
    }
    return cpu->memory[FDS_ADDR_SPECIAL_REG + reg];
}

void fds_reg_write(fds_cpu_t *cpu, uint8_t reg, uint16_t value)
{
    if (reg >= FDS_NUM_SPECIAL_REG) {
        return;
    }
    cpu->memory[FDS_ADDR_SPECIAL_REG + reg] = value;
}

/* ============================================================================
 * CPU LIFECYCLE
 * ============================================================================ */

int fds_cpu_init(fds_cpu_t *cpu)
{
    if (cpu == NULL) {
        return -1;
    }
    memset(cpu, 0, sizeof(fds_cpu_t));
    return 0;
}

void fds_cpu_reset(fds_cpu_t *cpu)
{
    if (cpu == NULL) {
        return;
    }

    cpu->ra = 0;
    cpu->rb = 0;
    cpu->ib = 0;
    cpu->pr = 0;
    cpu->cycles = 0;
    cpu->cycles_to_interrupt = FDS_INTERRUPT_CYCLES;

    cpu->flags.carry = false;
    cpu->flags.overflow = false;
    cpu->flags.zero = true;
    cpu->flags.positive = false;
    cpu->flags.halted = false;
    cpu->flags.interrupt = false;

    /* Memory banking - start in lower 4K */
    cpu->bank.jump_upper = false;
    cpu->bank.addr_upper = false;

    /* Memory preserved on reset - only registers cleared */
}

/* ============================================================================
 * I/O SUBSYSTEM
 * ============================================================================ */

void fds_io_init(fds_io_t *io)
{
    if (io == NULL) {
        return;
    }
    memset(io, 0, sizeof(fds_io_t));
}

void fds_io_set_callbacks(fds_io_t *io, fds_io_read_fn read, fds_io_write_fn write, void *user_data)
{
    if (io == NULL) {
        return;
    }
    io->read = read;
    io->write = write;
    io->user_data = user_data;
}

void fds_io_set_serial_input(fds_io_t *io, uint8_t channel, uint16_t data)
{
    if (io == NULL || channel >= FDS_SERIAL_INPUTS) {
        return;
    }
    io->serial_in[channel].data = data;
    io->serial_in[channel].ready = true;
}

uint16_t fds_io_get_serial_output(fds_io_t *io, uint8_t channel)
{
    if (io == NULL || channel >= FDS_SERIAL_OUTPUTS) {
        return 0;
    }
    io->serial_out[channel].ready = false;
    return io->serial_out[channel].data;
}

void fds_io_set_parallel_input(fds_io_t *io, uint8_t channel, uint16_t data)
{
    if (io == NULL || channel >= FDS_PARALLEL_INPUTS) {
        return;
    }
    io->parallel_in[channel].data = data;
    io->parallel_in[channel].ready = true;
}

uint16_t fds_io_get_parallel_output(fds_io_t *io, uint8_t channel)
{
    if (io == NULL || channel >= FDS_PARALLEL_OUTPUTS) {
        return 0;
    }
    io->parallel_out[channel].ready = false;
    return io->parallel_out[channel].data;
}

/* ============================================================================
 * DMA SUBSYSTEM
 *
 * Per MJS77-4-2006-1A: Four DMA channels provide high-speed data transfer
 * through MDS, DSS, ISS, and PRA I/O units. Addresses auto-increment.
 * DMA "access windows" occur at least once during each instruction.
 * Priority: MDS (0) > DSS (1) > ISS (2) > PRA (3)
 * ============================================================================ */

void fds_dma_enable(fds_io_t *io, fds_dma_channel_t ch, bool enable)
{
    if (io == NULL || ch >= FDS_DMA_CHANNELS) {
        return;
    }
    io->dma[ch].enabled = enable;
    if (!enable) {
        io->dma[ch].request = false;
    }
}

void fds_dma_set_direction(fds_io_t *io, fds_dma_channel_t ch, bool output)
{
    if (io == NULL || ch >= FDS_DMA_CHANNELS) {
        return;
    }
    io->dma[ch].direction = output;
}

void fds_dma_set_address(fds_io_t *io, fds_dma_channel_t ch, uint16_t addr)
{
    if (io == NULL || ch >= FDS_DMA_CHANNELS) {
        return;
    }
    io->dma[ch].address = addr & 0x1FFF;  /* 13-bit address for 8K memory */
}

void fds_dma_request(fds_io_t *io, fds_dma_channel_t ch, uint16_t data)
{
    if (io == NULL || ch >= FDS_DMA_CHANNELS) {
        return;
    }
    if (io->dma[ch].enabled && !io->dma[ch].direction) {
        /* Input mode: I/O device has data to write to memory */
        io->dma[ch].data = data;
        io->dma[ch].request = true;
    }
}

uint16_t fds_dma_get_output(fds_io_t *io, fds_dma_channel_t ch)
{
    if (io == NULL || ch >= FDS_DMA_CHANNELS) {
        return 0;
    }
    /* Return the last data read from memory for output */
    return io->dma[ch].data;
}

/*
 * Service pending DMA requests.
 * Called during instruction execution "access windows".
 * Processes channels in priority order (0 = highest).
 */
void fds_dma_service(fds_cpu_t *cpu, fds_io_t *io)
{
    if (cpu == NULL || io == NULL) {
        return;
    }

    /* Process channels in priority order */
    for (int ch = 0; ch < FDS_DMA_CHANNELS; ch++) {
        fds_dma_state_t *dma = &io->dma[ch];

        if (!dma->enabled) {
            continue;
        }

        if (dma->direction) {
            /* Output mode: memory -> I/O device */
            /* Read from memory and make available to I/O device */
            dma->data = fds_mem_read(cpu, dma->address);
            dma->address = (dma->address + 1) & 0x1FFF;
            dma->last_transfer = cpu->cycles;

            /* Notify via callback if available */
            if (io->write != NULL) {
                fds_io_channel_t io_ch;
                switch (ch) {
                case FDS_DMA_MDS: io_ch = FDS_IO_MDS; break;
                case FDS_DMA_DSS: io_ch = FDS_IO_DSS; break;
                case FDS_DMA_ISS: io_ch = FDS_IO_ISS_NA; break;
                case FDS_DMA_PRA: io_ch = FDS_IO_PRA; break;
                default: io_ch = FDS_IO_MDS; break;
                }
                io->write(io_ch, 0xFFFF, dma->data);  /* 0xFFFF = DMA transfer */
            }
        } else if (dma->request) {
            /* Input mode with pending request: I/O device -> memory */
            fds_mem_write(cpu, dma->address, dma->data);
            dma->address = (dma->address + 1) & 0x1FFF;
            dma->request = false;
            dma->last_transfer = cpu->cycles;
        }
    }
}

/* ============================================================================
 * PROGRAM LOADING
 * ============================================================================ */

int fds_load_binary(fds_cpu_t *cpu, const uint16_t *data, uint16_t addr, uint16_t count)
{
    if (cpu == NULL || data == NULL) {
        return -1;
    }
    if (addr + count > FDS_MEMORY_SIZE) {
        return -1;  /* Would overflow memory */
    }

    memcpy(&cpu->memory[addr], data, count * sizeof(uint16_t));
    return 0;
}

int fds_load_file(fds_cpu_t *cpu, const char *filename, uint16_t addr)
{
    if (cpu == NULL || filename == NULL) {
        return -1;
    }

    FILE *f = fopen(filename, "rb");
    if (f == NULL) {
        return -1;
    }

    /* Read 16-bit words until EOF or memory full */
    uint16_t offset = 0;
    while (addr + offset < FDS_MEMORY_SIZE) {
        uint8_t buf[2];
        if (fread(buf, 1, 2, f) != 2) {
            break;
        }
        /* Big-endian word order (verify against actual FDS) */
        cpu->memory[addr + offset] = (buf[0] << 8) | buf[1];
        offset++;
    }

    fclose(f);
    return offset;  /* Return words loaded */
}

/* ============================================================================
 * INSTRUCTION EXECUTION
 * ============================================================================ */

/* Execute a single instruction. Returns cycles consumed, or -1 on error. */
int fds_cpu_step(fds_cpu_t *cpu, fds_io_t *io)
{
    if (cpu == NULL) {
        return -1;
    }
    if (cpu->flags.halted) {
        return 0;
    }

    /* Fetch instruction */
    cpu->ib = fds_mem_read(cpu, cpu->pr);
    uint16_t instr = cpu->ib;
    uint8_t opcode = get_opcode(instr);
    uint16_t addr = get_address(instr);

    /* Default: advance to next instruction */
    uint16_t next_pr = (cpu->pr + 1) & 0x0FFF;
    int cycles = 2;  /* Default cycle count */

    switch (opcode) {

    /* --------------------------------------------------------------------
     * JMP (0000) - Jump
     * Cycles: 2
     * If SETJU was used, jumps to upper 4K bank
     * -------------------------------------------------------------------- */
    case FDS_OP_JMP:
        next_pr = get_jump_address(cpu, addr);
        cycles = 2;
        break;

    /* --------------------------------------------------------------------
     * SRB (0001) - Save RB Register, then Jump
     * Cycles: 3
     * Saves (PR+1) to RB, then jumps to address (subroutine call)
     * Uses addr_upper for the target address
     * -------------------------------------------------------------------- */
    case FDS_OP_SRB:
        cpu->rb = (cpu->pr + 1) & 0x1FFF;  /* Save return address (13-bit) */
        next_pr = get_data_address(cpu, addr);
        cycles = 3;
        break;

    /* --------------------------------------------------------------------
     * EXC (0010) - Execute
     * Cycles: 2
     * Execute instruction at address, then continue
     * Uses addr_upper for the target address
     * -------------------------------------------------------------------- */
    case FDS_OP_EXC:
        /* Fetch and execute instruction at addr, then return here */
        {
            uint16_t saved_pr = cpu->pr;
            cpu->pr = get_data_address(cpu, addr);
            fds_cpu_step(cpu, io);  /* Execute one instruction at addr */
            next_pr = (saved_pr + 1) & 0x1FFF;
        }
        cycles = 2;  /* Plus cycles of executed instruction */
        break;

    /* --------------------------------------------------------------------
     * WAT (0011) - Wait
     * Cycles: 4097 (or until interrupt)
     * -------------------------------------------------------------------- */
    case FDS_OP_WAT:
        cpu->flags.halted = true;  /* Will resume on interrupt */
        cycles = 4097;
        break;

    /* --------------------------------------------------------------------
     * MLD (0100) - Load Memory
     * Cycles: 4
     * RA <- Memory[addr]
     * Uses addr_upper for the target address
     * -------------------------------------------------------------------- */
    case FDS_OP_MLD:
        cpu->ra = fds_mem_read(cpu, get_data_address(cpu, addr));
        update_flags(cpu);
        cycles = 4;
        break;

    /* --------------------------------------------------------------------
     * MRD (0101) - Read Memory (Store)
     * Cycles: 4
     * Memory[addr] <- RA
     * Uses addr_upper for the target address
     * -------------------------------------------------------------------- */
    case FDS_OP_MRD:
        fds_mem_write(cpu, get_data_address(cpu, addr), cpu->ra);
        cycles = 4;
        break;

    /* --------------------------------------------------------------------
     * SWI (0110) - Serial Word In
     * Cycles: 11
     * RA <- serial data from channel
     * Encoding: 0110 xxxx xxCCC CCCC
     *   Bits 0-4: Serial input channel (0-31)
     *   Bits 5-11: Control/extended parameters
     * -------------------------------------------------------------------- */
    case FDS_OP_SWI:
        {
            uint8_t channel = instr & FDS_SWI_CHANNEL_MASK;

            if (io != NULL) {
                /* Try callback first */
                if (io->read != NULL) {
                    cpu->ra = io->read(FDS_IO_CCS, channel);
                }
                /* Fall back to direct port access */
                else if (channel < FDS_SERIAL_INPUTS) {
                    cpu->ra = io->serial_in[channel].data;
                    io->serial_in[channel].ready = false;
                }
            } else {
                /* No I/O subsystem - read 0 */
                cpu->ra = 0;
            }
            update_flags(cpu);
            cycles = 11;
        }
        break;

    /* --------------------------------------------------------------------
     * SWO/PWD (0111) - Serial Word Out / Parallel Word Transfer
     * Bit 11 distinguishes: 1 = PWD (parallel), 0 = SWO (serial)
     * Cycles: 4 (PWD) or 11 (SWO)
     *
     * SWO encoding: 0111 0xxx xCCC CCCC
     *   Bits 0-3: Serial output channel (0-15)
     *   Bits 4-10: Control/extended parameters
     *
     * PWD encoding: 0111 1xxx Dxxx CCCC
     *   Bit 10: Direction (0=input to RA, 1=output from RA)
     *   Bits 0-3: Parallel channel (0-7 for input, 0-9 for output)
     * -------------------------------------------------------------------- */
    case FDS_OP_SWO:
        if (instr & 0x0800) {
            /* PWD - Parallel Word Transfer */
            uint8_t channel = instr & FDS_PWD_CHANNEL_MASK;
            bool output = (instr & FDS_PWD_DIRECTION_BIT) != 0;

            if (io != NULL) {
                if (output) {
                    /* Output: RA -> parallel out port */
                    if (io->write != NULL) {
                        io->write(FDS_IO_MDS, channel, cpu->ra);
                    } else if (channel < FDS_PARALLEL_OUTPUTS) {
                        io->parallel_out[channel].data = cpu->ra;
                        io->parallel_out[channel].ready = true;
                    }
                } else {
                    /* Input: parallel in port -> RA */
                    if (io->read != NULL) {
                        cpu->ra = io->read(FDS_IO_MDS, channel);
                    } else if (channel < FDS_PARALLEL_INPUTS) {
                        cpu->ra = io->parallel_in[channel].data;
                        io->parallel_in[channel].ready = false;
                    }
                    update_flags(cpu);
                }
            } else if (!output) {
                cpu->ra = 0;
                update_flags(cpu);
            }
            cycles = 4;
        } else {
            /* SWO - Serial Word Out */
            uint8_t channel = instr & FDS_SWO_CHANNEL_MASK;

            if (io != NULL) {
                /* Try callback first */
                if (io->write != NULL) {
                    io->write(FDS_IO_MDS, channel, cpu->ra);
                }
                /* Fall back to direct port access */
                else if (channel < FDS_SERIAL_OUTPUTS) {
                    io->serial_out[channel].data = cpu->ra;
                    io->serial_out[channel].ready = true;
                }
            }
            cycles = 11;
        }
        break;

    /* --------------------------------------------------------------------
     * ABS (1000) - Absolute Entry
     * Cycles: 3
     * Jump using address from memory location
     * -------------------------------------------------------------------- */
    case FDS_OP_ABS:
        next_pr = fds_mem_read(cpu, addr) & 0x0FFF;
        cycles = 3;
        break;

    /* --------------------------------------------------------------------
     * ALU Operations (1001)
     * Sub-operation determined by bits 11-8
     * -------------------------------------------------------------------- */
    case FDS_OP_ALU:
        {
            uint8_t subop = (instr >> 8) & 0x0F;

            /* Bits 11-10 = 00: Arithmetic (ADD, LXR, AND, LOR, SUB) */
            if ((subop & 0x0C) == 0x00) {
                uint8_t aluop = (instr >> 4) & 0x03;
                /* Source is typically RB or memory, dest is RA */
                uint16_t operand = cpu->rb;
                uint32_t result;

                switch (aluop) {
                case 0: /* ADD */
                    result = (uint32_t)cpu->ra + operand;
                    cpu->flags.carry = (result > 0xFFFF);
                    cpu->flags.overflow = (((cpu->ra ^ result) & (operand ^ result)) >> 15) & 1;
                    cpu->ra = result & 0xFFFF;
                    break;
                case 1: /* LXR - Exclusive OR */
                    cpu->ra = cpu->ra ^ operand;
                    break;
                case 2: /* AND */
                    cpu->ra = cpu->ra & operand;
                    break;
                case 3: /* LOR - Inclusive OR */
                    cpu->ra = cpu->ra | operand;
                    break;
                }
                cycles = 6;
            }
            /* Bits 11-10 = 01, bit 9 = 0: SUB */
            else if ((subop & 0x0E) == 0x04) {
                uint32_t result = (uint32_t)cpu->ra - cpu->rb;
                cpu->flags.carry = (cpu->ra < cpu->rb);
                cpu->ra = result & 0xFFFF;
                cycles = 6;
            }
            /* Bits 11-10 = 10: SLC (Skip on Line Count) */
            else if ((subop & 0x0C) == 0x08) {
                /* Skip if line count matches */
                /* TODO: Implement line count register */
                cycles = 6;
            }
            /* Bits 11-10 = 11: Skip instructions */
            else if ((subop & 0x0C) == 0x0C) {
                uint8_t skip_type = (instr >> 4) & 0x07;
                bool skip = false;

                switch (skip_type) {
                case 0: /* SKP - Skip on Positive */
                    skip = cpu->flags.positive;
                    cycles = 3;
                    break;
                case 1: /* ISP - Increment & Skip on Positive */
                    cpu->ra++;
                    update_flags(cpu);
                    skip = cpu->flags.positive;
                    cycles = 4;
                    break;
                case 2: /* DSP - Decrement & Skip on Positive */
                    cpu->ra--;
                    update_flags(cpu);
                    skip = cpu->flags.positive;
                    cycles = 4;
                    break;
                case 3: /* SKZ - Skip on Zero */
                    skip = cpu->flags.zero;
                    cycles = 3;
                    break;
                case 4: /* ISZ - Increment & Skip on Zero */
                    cpu->ra++;
                    update_flags(cpu);
                    skip = cpu->flags.zero;
                    cycles = 4;
                    break;
                case 5: /* DSZ - Decrement & Skip on Zero */
                    cpu->ra--;
                    update_flags(cpu);
                    skip = cpu->flags.zero;
                    cycles = 4;
                    break;
                case 6: /* SKO - Skip on Overflow */
                    skip = cpu->flags.overflow;
                    cpu->flags.overflow = false;  /* Clear after test */
                    cycles = 3;
                    break;
                case 7: /* SKC - Skip on Carry */
                    skip = cpu->flags.carry;
                    cpu->flags.carry = false;  /* Clear after test */
                    cycles = 3;
                    break;
                }

                if (skip) {
                    next_pr = (cpu->pr + 2) & 0x0FFF;
                }
            }

            update_flags(cpu);
        }
        break;

    /* --------------------------------------------------------------------
     * AML/AMR (1010) - Auto Index Memory Load/Read
     * Cycles: 6
     * Memory access with auto-increment/decrement of index register
     * -------------------------------------------------------------------- */
    case FDS_OP_AUTO:
        {
            bool is_read = (instr & 0x0800) != 0;  /* Bit 11: 0=Load, 1=Read(Store) */
            uint8_t mp_reg = (instr >> 4) & 0x1F;  /* Memory pointer register */
            uint16_t mp_addr = FDS_ADDR_SPECIAL_REG + mp_reg;
            uint16_t effective_addr = fds_mem_read(cpu, mp_addr);

            if (is_read) {
                /* AMR: Store RA to memory, then increment pointer */
                fds_mem_write(cpu, effective_addr, cpu->ra);
            } else {
                /* AML: Load from memory to RA, then increment pointer */
                cpu->ra = fds_mem_read(cpu, effective_addr);
                update_flags(cpu);
            }

            /* Auto-increment the memory pointer */
            fds_mem_write(cpu, mp_addr, (effective_addr + 1) & 0xFFFF);
            cycles = 6;
        }
        break;

    /* --------------------------------------------------------------------
     * Short Shift Operations (1011)
     * Cycles: 4-6 depending on shift count
     * Operates on RA only (16-bit)
     * -------------------------------------------------------------------- */
    case FDS_OP_SHIFT_SHORT:
        {
            uint8_t shift_type = (instr >> 8) & 0x03;  /* Bits 9-8 */
            uint8_t shift_count = instr & 0x0F;        /* Bits 3-0 */
            if (shift_count == 0) shift_count = 16;    /* 0 means 16 */

            switch (shift_type) {
            case 0: /* SRS - Short Right Shift */
                cpu->ra >>= shift_count;
                break;
            case 1: /* SLS - Short Left Shift */
                cpu->ra <<= shift_count;
                break;
            case 2: /* SRR - Short Right Rotate */
                cpu->ra = (cpu->ra >> shift_count) | (cpu->ra << (16 - shift_count));
                break;
            case 3: /* ARS - Short Arithmetic Right Shift */
                {
                    int16_t signed_ra = (int16_t)cpu->ra;
                    signed_ra >>= shift_count;
                    cpu->ra = (uint16_t)signed_ra;
                }
                break;
            }
            update_flags(cpu);
            cycles = 4 + (shift_count / 4);  /* Approximate cycle count */
        }
        break;

    /* --------------------------------------------------------------------
     * Long Shift Operations (1100)
     * Cycles: 6-11 depending on shift count
     * Operates on RA:RB as 32-bit value (RA=high, RB=low)
     * -------------------------------------------------------------------- */
    case FDS_OP_SHIFT_LONG:
        {
            uint8_t shift_type = (instr >> 8) & 0x03;  /* Bits 9-8 */
            uint8_t shift_count = instr & 0x1F;        /* Bits 4-0 */
            if (shift_count == 0) shift_count = 32;    /* 0 means 32 */

            uint32_t combined = ((uint32_t)cpu->ra << 16) | cpu->rb;

            switch (shift_type) {
            case 0: /* LRS - Long Right Shift (Arithmetic) */
                {
                    int32_t signed_val = (int32_t)combined;
                    signed_val >>= shift_count;
                    combined = (uint32_t)signed_val;
                }
                break;
            case 1: /* LLS - Long Left Shift */
                combined <<= shift_count;
                break;
            case 2: /* LRR - Long Right Rotate */
                combined = (combined >> shift_count) | (combined << (32 - shift_count));
                break;
            default:
                /* Reserved */
                break;
            }

            cpu->ra = (combined >> 16) & 0xFFFF;
            cpu->rb = combined & 0xFFFF;
            update_flags(cpu);
            cycles = 6 + (shift_count / 4);  /* Approximate cycle count */
        }
        break;

    /* --------------------------------------------------------------------
     * MCX (1101) - Conditionally Modify Index
     * Cycles: 2
     * -------------------------------------------------------------------- */
    case FDS_OP_MCX:
        /* TODO: Implement conditional index modification */
        cycles = 2;
        break;

    /* --------------------------------------------------------------------
     * SKE (1110) - Skip if Equal
     * Cycles: 2
     * Skip next instruction if index register equals value
     * -------------------------------------------------------------------- */
    case FDS_OP_SKE:
        {
            uint8_t index_reg = (instr >> 8) & 0x07;  /* Which index register */
            uint8_t compare_val = instr & 0xFF;       /* Value to compare */
            uint16_t ir_addr = FDS_ADDR_SPECIAL_REG + index_reg;
            uint16_t ir_value = fds_mem_read(cpu, ir_addr) & 0xFF;

            if (ir_value == compare_val) {
                next_pr = (cpu->pr + 2) & 0x0FFF;  /* Skip next instruction */
            }
            cycles = 2;
        }
        break;

    /* --------------------------------------------------------------------
     * OUT (1111) - Discrete Output
     * Cycles: 3
     * Output discrete signals to I/O
     * Special codes for:
     *   - Memory banking (SETJU, SETJD, SETAU, SETAD)
     *   - DMA control (enable, disable, direction, address)
     * -------------------------------------------------------------------- */
    case FDS_OP_OUT:
        {
            uint8_t out_code = instr & 0x1F;  /* 5-bit discrete code */

            /* Check for memory banking codes (01-04) */
            if (out_code >= FDS_OUT_SETJU && out_code <= FDS_OUT_SETAD) {
                switch (out_code) {
                case FDS_OUT_SETJU:
                    cpu->bank.jump_upper = true;
                    break;
                case FDS_OUT_SETJD:
                    cpu->bank.jump_upper = false;
                    break;
                case FDS_OUT_SETAU:
                    cpu->bank.addr_upper = true;
                    break;
                case FDS_OUT_SETAD:
                    cpu->bank.addr_upper = false;
                    break;
                }
            }
            /* Check for DMA enable (08-0B) */
            else if (out_code >= FDS_OUT_DMA_ENABLE &&
                     out_code < FDS_OUT_DMA_ENABLE + FDS_DMA_CHANNELS) {
                uint8_t ch = out_code & FDS_OUT_DMA_CH_MASK;
                fds_dma_enable(io, ch, true);
            }
            /* Check for DMA disable (0C-0F) */
            else if (out_code >= FDS_OUT_DMA_DISABLE &&
                     out_code < FDS_OUT_DMA_DISABLE + FDS_DMA_CHANNELS) {
                uint8_t ch = out_code & FDS_OUT_DMA_CH_MASK;
                fds_dma_enable(io, ch, false);
            }
            /* Check for DMA direction output (10-13) */
            else if (out_code >= FDS_OUT_DMA_DIR_OUT &&
                     out_code < FDS_OUT_DMA_DIR_OUT + FDS_DMA_CHANNELS) {
                uint8_t ch = out_code & FDS_OUT_DMA_CH_MASK;
                fds_dma_set_direction(io, ch, true);  /* Output: mem->I/O */
            }
            /* Check for DMA direction input (14-17) */
            else if (out_code >= FDS_OUT_DMA_DIR_IN &&
                     out_code < FDS_OUT_DMA_DIR_IN + FDS_DMA_CHANNELS) {
                uint8_t ch = out_code & FDS_OUT_DMA_CH_MASK;
                fds_dma_set_direction(io, ch, false);  /* Input: I/O->mem */
            }
            /* Check for DMA address load (18-1B) */
            else if (out_code >= FDS_OUT_DMA_ADDR &&
                     out_code < FDS_OUT_DMA_ADDR + FDS_DMA_CHANNELS) {
                uint8_t ch = out_code & FDS_OUT_DMA_CH_MASK;
                fds_dma_set_address(io, ch, cpu->ra);  /* Address from RA */
            }
            /* Other discrete outputs - pass to I/O subsystem */
            else if (io != NULL && io->write != NULL) {
                io->write(FDS_IO_CCS, out_code, cpu->ra);
            }
        }
        cycles = 3;
        break;

    default:
        /* Unknown opcode - halt */
        cpu->flags.halted = true;
        return -1;
    }

    /* Update program counter */
    cpu->pr = next_pr;
    cpu->cycles += cycles;

    /* Service DMA channels during "access window" */
    fds_dma_service(cpu, io);

    return cycles;
}

/* Handle the 2.5ms periodic interrupt */
static void fds_handle_interrupt(fds_cpu_t *cpu)
{
    /* Per MJS77-4-2006-1A: Program address forced to zero every 2.5ms */
    cpu->pr = 0;
    cpu->flags.halted = false;  /* Resume if halted by WAT */

    /* Reset memory banking on interrupt */
    cpu->bank.jump_upper = false;
    cpu->bank.addr_upper = false;

    /* Reset interrupt counter */
    cpu->cycles_to_interrupt = FDS_INTERRUPT_CYCLES;
}

/* Run for up to max_cycles, or until halted */
int fds_cpu_run(fds_cpu_t *cpu, fds_io_t *io, uint64_t max_cycles)
{
    if (cpu == NULL) {
        return -1;
    }

    uint64_t start_cycles = cpu->cycles;

    while ((cpu->cycles - start_cycles) < max_cycles) {
        /* Check for 2.5ms interrupt */
        if (cpu->cycles_to_interrupt == 0) {
            fds_handle_interrupt(cpu);
        }

        if (cpu->flags.halted) {
            /* WAT instruction - just count down to interrupt */
            uint64_t wait_cycles = cpu->cycles_to_interrupt;
            if (wait_cycles > (max_cycles - (cpu->cycles - start_cycles))) {
                wait_cycles = max_cycles - (cpu->cycles - start_cycles);
            }
            cpu->cycles += wait_cycles;
            cpu->cycles_to_interrupt -= wait_cycles;
            continue;
        }

        int result = fds_cpu_step(cpu, io);
        if (result < 0) {
            return -1;
        }

        /* Decrement interrupt counter */
        if (cpu->cycles_to_interrupt >= (uint64_t)result) {
            cpu->cycles_to_interrupt -= result;
        } else {
            cpu->cycles_to_interrupt = 0;
        }
    }

    return (int)(cpu->cycles - start_cycles);
}

/* ============================================================================
 * DEBUGGING
 * ============================================================================ */

void fds_cpu_dump(const fds_cpu_t *cpu)
{
    if (cpu == NULL) {
        return;
    }

    printf("FDS CPU State:\n");
    printf("  PR:  %03X    IB:  %04X\n", cpu->pr, cpu->ib);
    printf("  RA:  %04X   RB:  %04X\n", cpu->ra, cpu->rb);
    printf("  Flags: %s%s%s%s%s\n",
           cpu->flags.carry ? "C" : "-",
           cpu->flags.overflow ? "V" : "-",
           cpu->flags.zero ? "Z" : "-",
           cpu->flags.positive ? "P" : "-",
           cpu->flags.halted ? "H" : "-");
    printf("  Cycles: %llu\n", (unsigned long long)cpu->cycles);
}

/* Disassemble a single instruction */
const char *fds_disassemble(uint16_t instr, char *buf, size_t buflen)
{
    static char static_buf[64];
    if (buf == NULL) {
        buf = static_buf;
        buflen = sizeof(static_buf);
    }

    uint8_t opcode = get_opcode(instr);
    uint16_t addr = get_address(instr);

    switch (opcode) {
    case FDS_OP_JMP:
        snprintf(buf, buflen, "JMP  %03X", addr);
        break;
    case FDS_OP_SRB:
        snprintf(buf, buflen, "SRB  %03X", addr);
        break;
    case FDS_OP_EXC:
        snprintf(buf, buflen, "EXC  %03X", addr);
        break;
    case FDS_OP_WAT:
        snprintf(buf, buflen, "WAT  %03X", addr);
        break;
    case FDS_OP_MLD:
        snprintf(buf, buflen, "MLD  %03X", addr);
        break;
    case FDS_OP_MRD:
        snprintf(buf, buflen, "MRD  %03X", addr);
        break;
    case FDS_OP_SWI:
        {
            uint8_t ch = instr & FDS_SWI_CHANNEL_MASK;
            snprintf(buf, buflen, "SWI  CH%d", ch);
        }
        break;
    case FDS_OP_SWO:
        if (instr & 0x0800) {
            /* PWD - show direction and channel */
            uint8_t ch = instr & FDS_PWD_CHANNEL_MASK;
            bool out = (instr & FDS_PWD_DIRECTION_BIT) != 0;
            snprintf(buf, buflen, "PWD  %s,CH%d", out ? "OUT" : "IN", ch);
        } else {
            /* SWO - show channel */
            uint8_t ch = instr & FDS_SWO_CHANNEL_MASK;
            snprintf(buf, buflen, "SWO  CH%d", ch);
        }
        break;
    case FDS_OP_ABS:
        snprintf(buf, buflen, "ABS  %03X", addr);
        break;
    case FDS_OP_ALU:
        {
            uint8_t subop = (instr >> 8) & 0x0F;
            if ((subop & 0x0C) == 0x00) {
                uint8_t aluop = (instr >> 4) & 0x03;
                const char *ops[] = {"ADD", "LXR", "AND", "LOR"};
                snprintf(buf, buflen, "%s", ops[aluop]);
            } else if ((subop & 0x0E) == 0x04) {
                snprintf(buf, buflen, "SUB");
            } else if ((subop & 0x0C) == 0x0C) {
                uint8_t skip_type = (instr >> 4) & 0x07;
                const char *skips[] = {"SKP", "ISP", "DSP", "SKZ", "ISZ", "DSZ", "SKO", "SKC"};
                snprintf(buf, buflen, "%s", skips[skip_type]);
            } else {
                snprintf(buf, buflen, "ALU? %04X", instr);
            }
        }
        break;
    case FDS_OP_AUTO:
        snprintf(buf, buflen, "%s  MP%d",
                 (instr & 0x0800) ? "AMR" : "AML",
                 (instr >> 4) & 0x1F);
        break;
    case FDS_OP_SHIFT_SHORT:
        {
            uint8_t shift_type = (instr >> 8) & 0x03;
            uint8_t shift_count = instr & 0x0F;
            if (shift_count == 0) shift_count = 16;
            const char *ops[] = {"SRS", "SLS", "SRR", "ARS"};
            snprintf(buf, buflen, "%s  %d", ops[shift_type], shift_count);
        }
        break;
    case FDS_OP_SHIFT_LONG:
        {
            uint8_t shift_type = (instr >> 8) & 0x03;
            uint8_t shift_count = instr & 0x1F;
            if (shift_count == 0) shift_count = 32;
            const char *ops[] = {"LRS", "LLS", "LRR", "???"};
            snprintf(buf, buflen, "%s  %d", ops[shift_type], shift_count);
        }
        break;
    case FDS_OP_MCX:
        snprintf(buf, buflen, "MCX  %03X", addr);
        break;
    case FDS_OP_SKE:
        snprintf(buf, buflen, "SKE  IR%d,%02X", (instr >> 8) & 0x07, instr & 0xFF);
        break;
    case FDS_OP_OUT:
        {
            uint8_t out_code = instr & 0x1F;
            const char *dma_ch[] = {"MDS", "DSS", "ISS", "PRA"};

            /* Memory banking codes */
            if (out_code == FDS_OUT_SETJU) {
                snprintf(buf, buflen, "SETJU");
            } else if (out_code == FDS_OUT_SETJD) {
                snprintf(buf, buflen, "SETJD");
            } else if (out_code == FDS_OUT_SETAU) {
                snprintf(buf, buflen, "SETAU");
            } else if (out_code == FDS_OUT_SETAD) {
                snprintf(buf, buflen, "SETAD");
            }
            /* DMA enable (08-0B) */
            else if (out_code >= FDS_OUT_DMA_ENABLE &&
                     out_code < FDS_OUT_DMA_ENABLE + FDS_DMA_CHANNELS) {
                uint8_t ch = out_code & FDS_OUT_DMA_CH_MASK;
                snprintf(buf, buflen, "DMAEN %s", dma_ch[ch]);
            }
            /* DMA disable (0C-0F) */
            else if (out_code >= FDS_OUT_DMA_DISABLE &&
                     out_code < FDS_OUT_DMA_DISABLE + FDS_DMA_CHANNELS) {
                uint8_t ch = out_code & FDS_OUT_DMA_CH_MASK;
                snprintf(buf, buflen, "DMADI %s", dma_ch[ch]);
            }
            /* DMA direction output (10-13) */
            else if (out_code >= FDS_OUT_DMA_DIR_OUT &&
                     out_code < FDS_OUT_DMA_DIR_OUT + FDS_DMA_CHANNELS) {
                uint8_t ch = out_code & FDS_OUT_DMA_CH_MASK;
                snprintf(buf, buflen, "DMAOU %s", dma_ch[ch]);
            }
            /* DMA direction input (14-17) */
            else if (out_code >= FDS_OUT_DMA_DIR_IN &&
                     out_code < FDS_OUT_DMA_DIR_IN + FDS_DMA_CHANNELS) {
                uint8_t ch = out_code & FDS_OUT_DMA_CH_MASK;
                snprintf(buf, buflen, "DMAIN %s", dma_ch[ch]);
            }
            /* DMA address load (18-1B) */
            else if (out_code >= FDS_OUT_DMA_ADDR &&
                     out_code < FDS_OUT_DMA_ADDR + FDS_DMA_CHANNELS) {
                uint8_t ch = out_code & FDS_OUT_DMA_CH_MASK;
                snprintf(buf, buflen, "DMAAD %s", dma_ch[ch]);
            }
            else {
                snprintf(buf, buflen, "OUT  %02X", out_code);
            }
        }
        break;
    default:
        snprintf(buf, buflen, "???  %04X", instr);
        break;
    }

    return buf;
}

/* Disassemble a program and print to stdout */
void fds_disassemble_program(const uint16_t *code, uint16_t origin, uint16_t count)
{
    char buf[64];

    printf("\n; FDS Disassembly at $%03X\n", origin);
    printf("; ========================\n\n");

    for (uint16_t i = 0; i < count; i++) {
        uint16_t addr = origin + i;
        uint16_t word = code[i];
        fds_disassemble(word, buf, sizeof(buf));
        printf("$%03X:  %04X    %s\n", addr, word, buf);
    }
    printf("\n");
}
