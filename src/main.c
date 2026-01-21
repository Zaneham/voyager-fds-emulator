/*
 * main.c - Voyager FDS Emulator Test Harness
 *
 * Simple test program to validate the FDS emulator.
 *
 * If this works, you can pretend you're NASA.
 * If it doesn't, at least you're closer to home than Voyager.
 */

#include "../include/fds.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Simple test program: count down from 10 to 0
 *
 * Note: DSZ decrements RA, so we must store back to memory each loop.
 * TODO: Verify if real FDS DSZ operates on memory location directly.
 */
static uint16_t test_countdown[] = {
    0x4010,  /* 000: MLD  010    ; Load initial value (10) into RA */
    0x9C50,  /* 001: DSZ         ; Decrement RA, skip if zero */
    0x0001,  /* 002: JMP  001    ; Not zero, loop back to DSZ */
    0x5011,  /* 003: MRD  011    ; Store final RA (0) to 011 */
    0x3000,  /* 004: WAT  000    ; Halt */
    0x0000,  /* 005: (unused) */
    0x0000,  /* 006: (unused) */
    0x0000,  /* 007: (unused) */
    0x0000,  /* 008: (unused) */
    0x0000,  /* 009: (unused) */
    0x0000,  /* 00A: (unused) */
    0x0000,  /* 00B: (unused) */
    0x0000,  /* 00C: (unused) */
    0x0000,  /* 00D: (unused) */
    0x0000,  /* 00E: (unused) */
    0x0000,  /* 00F: (unused) */
    0x000A,  /* 010: Initial value = 10 */
    0x0000,  /* 011: Result storage */
};

/* TODO: More test programs once instruction encoding verified */

static void print_usage(const char *prog)
{
    printf("Voyager FDS Emulator & Assembler\n");
    printf("Based on JPL Memo MJS 2.64A, 7 October 1974\n");
    printf("The computer that's leaving the solar system. On purpose.\n\n");
    printf("Usage: %s [options] [file]\n", prog);
    printf("\nOptions:\n");
    printf("  -a, --assemble   Assemble .fds source file\n");
    printf("  -o FILE          Output file for assembly (default: out.bin)\n");
    printf("  -r, --run        Run after assembling\n");
    printf("  -t, --test       Run built-in test program\n");
    printf("  -d, --debug      Enable debug output\n");
    printf("  -s, --step       Single-step mode\n");
    printf("  -h, --help       Show this help\n");
    printf("\nExamples:\n");
    printf("  %s -t                    Run built-in test\n", prog);
    printf("  %s -a prog.fds -o prog.bin   Assemble source\n", prog);
    printf("  %s -a prog.fds -r        Assemble and run\n", prog);
    printf("  %s prog.bin              Load and debug binary\n", prog);
    printf("\n");
}

static void run_test_program(bool debug, bool step_mode)
{
    fds_cpu_t cpu;
    char disasm[64];

    printf("=== FDS Emulator Test: Countdown ===\n\n");

    /* Initialize CPU */
    fds_cpu_init(&cpu);
    fds_cpu_reset(&cpu);

    /* Load test program */
    fds_load_binary(&cpu, test_countdown,
                    0, sizeof(test_countdown) / sizeof(test_countdown[0]));

    printf("Program loaded. Starting execution...\n\n");

    if (debug) {
        fds_cpu_dump(&cpu);
        printf("\n");
    }

    /* Execute */
    int max_steps = 1000;  /* Safety limit */
    int steps = 0;

    while (!cpu.flags.halted && steps < max_steps) {
        if (debug || step_mode) {
            printf("[%03X] %04X  %s\n",
                   cpu.pr,
                   cpu.memory[cpu.pr],
                   fds_disassemble(cpu.memory[cpu.pr], disasm, sizeof(disasm)));
        }

        (void)fds_cpu_step(&cpu, NULL);
        steps++;

        if (debug || step_mode) {
            printf("       RA=%04X RB=%04X [%s%s%s%s]\n",
                   cpu.ra, cpu.rb,
                   cpu.flags.carry ? "C" : "-",
                   cpu.flags.overflow ? "V" : "-",
                   cpu.flags.zero ? "Z" : "-",
                   cpu.flags.positive ? "P" : "-");

            if (step_mode) {
                printf("Press Enter to continue...");
                getchar();
            }
        }
    }

    printf("\n=== Execution Complete ===\n");
    printf("Steps: %d\n", steps);
    printf("Cycles: %llu\n", (unsigned long long)cpu.cycles);
    fds_cpu_dump(&cpu);

    /* Check result */
    printf("\nCounter value at 011: %04X\n", cpu.memory[0x011]);
    if (cpu.memory[0x011] == 0x0000) {
        printf("TEST PASSED: Countdown reached zero.\n");
        printf("You are now qualified to work on spacecraft computers.\n");
        printf("(This is not legally binding.)\n");
    } else {
        printf("TEST FAILED: Expected 0000.\n");
        printf("Voyager would be disappointed, but it's too far away to care.\n");
    }
}

static void run_interactive(fds_cpu_t *cpu)
{
    char line[256];
    char disasm[64];

    printf("FDS Interactive Mode. Commands:\n");
    printf("  s, step    - Single step\n");
    printf("  r, run     - Run until halt\n");
    printf("  d, dump    - Dump CPU state\n");
    printf("  m ADDR     - Show memory at ADDR (hex)\n");
    printf("  q, quit    - Exit (Voyager cannot do this, it's committed)\n\n");

    while (1) {
        printf("fds> ");
        fflush(stdout);

        if (fgets(line, sizeof(line), stdin) == NULL) {
            break;
        }

        /* Strip newline */
        line[strcspn(line, "\n")] = 0;

        if (strcmp(line, "q") == 0 || strcmp(line, "quit") == 0) {
            break;
        }
        else if (strcmp(line, "s") == 0 || strcmp(line, "step") == 0) {
            if (cpu->flags.halted) {
                printf("CPU is halted.\n");
            } else {
                printf("[%03X] %04X  %s\n",
                       cpu->pr,
                       cpu->memory[cpu->pr],
                       fds_disassemble(cpu->memory[cpu->pr], disasm, sizeof(disasm)));
                fds_cpu_step(cpu, NULL);
                printf("       RA=%04X RB=%04X\n", cpu->ra, cpu->rb);
            }
        }
        else if (strcmp(line, "r") == 0 || strcmp(line, "run") == 0) {
            fds_cpu_run(cpu, NULL, 100000);
            fds_cpu_dump(cpu);
        }
        else if (strcmp(line, "d") == 0 || strcmp(line, "dump") == 0) {
            fds_cpu_dump(cpu);
        }
        else if (strncmp(line, "m ", 2) == 0) {
            unsigned int addr;
            if (sscanf(line + 2, "%x", &addr) == 1 && addr < FDS_MEMORY_SIZE) {
                printf("%03X: %04X\n", addr, cpu->memory[addr]);
            } else {
                printf("Invalid address.\n");
            }
        }
        else if (line[0] != '\0') {
            printf("Unknown command: %s\n", line);
        }
    }
}

/* Write assembled program to binary file */
static int write_binary(const fds_program_t *prog, const char *filename)
{
    FILE *f = fopen(filename, "wb");
    if (f == NULL) {
        return -1;
    }

    /* Write as big-endian 16-bit words */
    for (uint16_t i = 0; i < prog->count; i++) {
        uint8_t buf[2];
        buf[0] = (prog->code[i] >> 8) & 0xFF;
        buf[1] = prog->code[i] & 0xFF;
        fwrite(buf, 1, 2, f);
    }

    fclose(f);
    return 0;
}

/* Print assembled listing */
static void print_listing(const fds_program_t *prog)
{
    char disasm[64];
    printf("\n=== Assembly Listing ===\n");
    printf("Origin: %03X\n", prog->origin);
    printf("Size:   %d words\n\n", prog->count);
    printf("ADDR  CODE  DISASSEMBLY\n");
    printf("----  ----  -----------\n");

    for (uint16_t i = 0; i < prog->count; i++) {
        uint16_t addr = prog->origin + i;
        uint16_t word = prog->code[i];
        printf("%03X:  %04X  %s\n", addr, word,
               fds_disassemble(word, disasm, sizeof(disasm)));
    }
    printf("\n");
}

int main(int argc, char *argv[])
{
    bool run_test = false;
    bool assemble = false;
    bool run_after = false;
    bool debug = false;
    bool step_mode = false;
    const char *filename = NULL;
    const char *outfile = "out.bin";

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--test") == 0) {
            run_test = true;
        }
        else if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--assemble") == 0) {
            assemble = true;
        }
        else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--run") == 0) {
            run_after = true;
        }
        else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            outfile = argv[++i];
        }
        else if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--debug") == 0) {
            debug = true;
        }
        else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--step") == 0) {
            step_mode = true;
        }
        else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        else if (argv[i][0] != '-') {
            filename = argv[i];
        }
        else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return 1;
        }
    }

    if (run_test) {
        run_test_program(debug, step_mode);
        return 0;
    }

    /* Assemble mode */
    if (assemble) {
        if (filename == NULL) {
            fprintf(stderr, "Error: No source file specified\n");
            return 1;
        }

        printf("Assembling: %s\n", filename);

        fds_program_t prog = {0};
        fds_asm_error_t err = fds_assemble_file(filename, &prog);

        if (err != FDS_ASM_OK) {
            fprintf(stderr, "Assembly failed: %s\n", fds_asm_error_str(err));
            return 1;
        }

        printf("Assembly successful: %d words at origin %03X\n",
               prog.count, prog.origin);

        if (debug) {
            print_listing(&prog);
        }

        /* Write output file */
        if (write_binary(&prog, outfile) < 0) {
            fprintf(stderr, "Failed to write: %s\n", outfile);
            fds_program_free(&prog);
            return 1;
        }
        printf("Output written to: %s\n", outfile);

        /* Run after assembly if requested */
        if (run_after) {
            printf("\n=== Running Program ===\n");
            fds_cpu_t cpu;
            fds_cpu_init(&cpu);
            fds_cpu_reset(&cpu);
            fds_load_binary(&cpu, prog.code, prog.origin, prog.count);
            cpu.pr = prog.origin;

            if (step_mode) {
                run_interactive(&cpu);
            } else {
                fds_cpu_run(&cpu, NULL, 100000);
                fds_cpu_dump(&cpu);
            }
        }

        fds_program_free(&prog);
        return 0;
    }

    /* Load binary and run interactive */
    if (filename != NULL) {
        fds_cpu_t cpu;
        fds_cpu_init(&cpu);
        fds_cpu_reset(&cpu);

        int loaded = fds_load_file(&cpu, filename, 0);
        if (loaded < 0) {
            fprintf(stderr, "Failed to load: %s\n", filename);
            return 1;
        }
        printf("Loaded %d words from %s\n", loaded, filename);

        run_interactive(&cpu);
        return 0;
    }

    /* No arguments - show usage */
    print_usage(argv[0]);
    return 0;
}
