# Voyager FDS Emulator

An emulator for the computer that's leaving the solar system. On purpose.

## What Is This?

This is an emulator for the Flight Data Subsystem (FDS) used on Voyager 1 and Voyager 2. Voyager 1 launched September 5, 1977. Voyager 2 launched sixteen days earlier on August 20, 1977, because space mission naming conventions are not required to make sense.

Both spacecraft are still operating. Voyager 1 is currently about 24 billion kilometres away and has entered interstellar space. It takes over 22 hours to send a message there. If you email NASA now asking them to restart the computer, you'll get the reply sometime tomorrow. Assuming Voyager feels like replying.

The FDS was designed by JPL engineers in the early 1970s. The architecture memo is dated October 7, 1974. The computer uses an 806.4 kHz clock because apparently that was the correct frequency for leaving the solar system. The memory uses 8,192 16-bit words of CMOS RAM, organised into two 4K banks, because even interstellar spacecraft have to deal with memory banking.

The FDS handles all the science data. Every picture of Jupiter's Great Red Spot. Every reading from Saturn's rings. Every measurement of the heliopause. All processed by a computer running at under 1 MHz with less memory than this README file.

It's been running for 48 years. Your phone struggles to make it through the day.

## Features

- **Full instruction set** (36+ instructions, more than enough to reach Neptune)
- **8K x 16-bit CMOS memory** (with banking, because 4K at a time is plenty) (it wasn't, that's why there's banking)
- **4 DMA channels** (MDS, DSS, ISS, PRA - telemetry, tape, imaging, radio astronomy)
- **Serial I/O** (32 inputs, 16 outputs - for all those instruments)
- **Parallel I/O** (8 inputs, 10 outputs - for when serial is too slow at 115.2 kbps)
- **2.5ms periodic interrupt** (for error recovery, because space is full of cosmic rays and they're very rude)
- **Assembler and disassembler** (the original engineers used punch cards, you get text files, you're welcome)

## Architecture

```
Clock:           806.4 kHz (about 400,000 instructions per second)
Cycle Time:      2.48 microseconds (glacial by modern standards, eternal by space standards)
Memory:          8,192 x 16-bit words (two 4K banks)
Registers:       RA, RB (working), 128 special registers (because 2 wasn't enough)
I/O:             32 serial in, 16 serial out, 18 parallel, 4 DMA channels
Location:        Originally: Cape Canaveral. Currently: interstellar space.
Signal Delay:    22+ hours (latency that makes dial-up look good)
Weight:          You can lift it, but it's in interstellar space, so good luck.
```

## Building

```bash
gcc -O2 -o fds src/main.c src/fds_cpu.c src/fds_asm.c -Iinclude
```

Works with any C compiler from the last forty years. If you're using something older than Voyager itself, I have questions. Specifically: how did you get it running, and can I see it?

## Usage

```bash
# Run the built-in test
./fds -t

# Assemble and run a program
./fds -a program.fds -r

# Assemble with listing
./fds -a program.fds -d

# Interactive mode
./fds -a program.fds -r -s
```

### Interactive Commands

| Command | Description |
|---------|-------------|
| `s` | Step one instruction |
| `r` | Run until halt |
| `d` | Dump CPU state |
| `m <addr>` | Show memory at hex address |
| `q` | Quit (Voyager cannot do this, it's committed) |

## Instruction Set

The FDS has a proper 1970s instruction set. Simple. Elegant. Designed before complexity was invented.

```
Memory:      JMP, SRB, EXC, MLD, MRD, ABS, AML, AMR
Arithmetic:  ADD, SUB, AND, LOR, LXR
Shifts:      SRS, SLS, SRR, ARS, LRS, LLS, LRR (short and long)
Skip:        SKP, SKZ, SKO, SKC, ISP, ISZ, DSP, DSZ, SKE, SLC
I/O:         SWI, SWO, PWD, OUT
Control:     WAT (wait), MCX (modify index)
Banking:     SETJU, SETJD, SETAU, SETAD
DMA:         DMAEN, DMADI, DMAOU, DMAIN, DMAAD
```

There's no multiply instruction. The FDS uses repeated addition. If you want to multiply two numbers, you add one of them to itself many times, like it's 1974. Which, when this was designed, it was.

## Memory Banking

The FDS has 8K of memory but only 12-bit addresses. Rather than admit defeat, JPL engineers invented a banking scheme:

- `SETAU` - Address upper bank (data accesses go to upper 4K)
- `SETAD` - Address down (data accesses go to lower 4K)
- `SETJU` - Jump upper (next jump goes to upper 4K)
- `SETJD` - Jump down (jumps go to lower 4K)

Special registers (F80-FFF) always live in the lower bank because some things need to be predictable. The rest of the address space is chaos. Organised chaos, but chaos nonetheless.

## DMA Channels

Four DMA channels, each running at 115.2 kbps:

| Channel | Name | Purpose |
|---------|------|---------|
| 0 | MDS | Modulation/Demodulation - telemetry home |
| 1 | DSS | Data Storage - the tape recorder |
| 2 | ISS | Imaging Science - the cameras |
| 3 | PRA | Planetary Radio Astronomy - listening to Jupiter |

Priority goes MDS > DSS > ISS > PRA. Telemetry home is more important than taking pictures. Fair enough, really. If you can't tell Earth what you saw, you may as well have not seen it.

## Project Structure

```
voyager-fds-emulator/
├── src/
│   ├── main.c          # Entry point, CLI handling
│   ├── fds_cpu.c       # CPU emulation, instruction execution
│   └── fds_asm.c       # Assembler and disassembler
├── include/
│   └── fds.h           # Definitions, opcodes, structures
├── tests/              # Example FDS assembly programs
│   ├── countdown.fds
│   ├── banking_test.fds
│   ├── io_test.fds
│   └── dma_test.fds
└── docs/               # Original JPL documentation (1974-1977)
    ├── MS 87-08 B37 FF41.pdf
    ├── MS 87-08 B37 FF42.pdf
    └── MS 87-08 B37 FF43.pdf
```

## Documentation Sources

This emulator was built from original JPL documentation that survived in university archives and filing cabinets:

- **MJS 2.64A** "MJS FDS Processor Architecture and Instruction Set" (October 1974) - J. Wooddell
- **MJS77-4-2006-1A** "FDS Functional Requirements" (1977) - The full I/O specification
- **MS 87-08 B37 FF41-43** - Scanned documents from the JPL archive

MJS stood for Mariner Jupiter-Saturn. They renamed it Voyager later because that sounds better in press releases. The documentation wasn't updated. You work with what you have. The archives weren't expecting anyone to come looking 50 years later.

## Known Limitations

- Cannot actually communicate with Voyager (NASA controls the Deep Space Network and they don't share)
- Tape recorder simulation is stubbed (Voyager 2's tape recorder is the only one still working anyway)
- No Golden Record emulation (you'll have to provide your own whale songs) (and greetings in 55 languages)
- Cannot simulate leaving the solar system (this is a limitation of your location, not our software)
- Does not include the radioisotope thermoelectric generators (bring your own plutonium-238)

## The Golden Record

Voyager carries a golden phonograph record with sounds and images from Earth. It includes greetings in 55 languages, music from Bach to Chuck Berry, and whale songs. The FDS has nothing to do with playing it. That's handled by a separate system with its own stylus and instructions etched on the cover.

If aliens find Voyager, they get the record. If they somehow connect to the FDS instead, they get 8K of CMOS RAM running 1970s assembly language. I'm hoping they find the record first. If they don't, I apologise on behalf of humanity for the memory banking scheme.

## Why This Matters

Voyager is the furthest human-made object from Earth. It's been operating for nearly half a century with computers that would be outperformed by a modern washing machine. The engineers who built this didn't have the luxury of pushing a fix later. They got it right the first time. And the second time. And every time since, for 48 years, 24 billion kilometres from the nearest reboot.

When Voyager 1 crossed into interstellar space in 2012, the FDS was there processing the data. When Voyager 2's heater failed and it got too cold, the FDS kept running. When cosmic rays flip bits in memory, the 2.5ms interrupt handler catches the errors. When the RTGs finally run out sometime in the 2030s, the FDS will still be working. It just won't have power.

We have faster computers now. They require weekly security patches and crash if you look at them funny.

## Related Projects

If you've enjoyed emulating the computers leaving our solar system, you might also appreciate:

- **[Viking Mars Rover Emulator](https://github.com/Zaneham/Viking-Marsrover-emulator)** - The computers that landed on Mars. Voyager went further but Viking got there first. Sibling rivalry at its finest.

- **[Setun-70 Emulator](https://github.com/Zaneham/setun70-emulator)** - Soviet ternary computer. Because sometimes binary isn't weird enough.

- **[Minuteman Emulator](https://github.com/Zaneham/Minuteman-computer-emulator)** - The D-17B/D-37C guidance computers. Same era, different trajectory. Minuteman goes up, Voyager went sideways.

- **[HAL/S LSP](https://github.com/Zaneham/hals-lsp)** - Language server for the Space Shuttle's programming language. Real-time, redundant, and very particular about type safety.

- **[JOVIAL LSP](https://github.com/Zaneham/Jovial-LSP)** - Language server for the US military's favourite programming language. Jules' Own Version of the International Algebraic Language. Not a joke. That's actually what it stands for.

## Acknowledgements

Thank you to Brent Adams and the Special Collections team at Wichita State University Libraries. They responded to a stranger's email asking about Voyager documentation, found the materials, and made them available. This emulator wouldn't exist without their help.

## Licence

Apache 2.0. See [LICENSE](LICENSE) for details.

## Contact

Questions? Found a bug? Have original Voyager flight software on a 9-track tape that's been sitting in a basement since 1977?

zanehambly@gmail.com

Response time faster than a round-trip signal to Voyager. Currently running at 44+ hours and increasing by 61,000 kilometres per day. Light speed is a hard limit. Email is not.

Available to discuss interstellar computing over coffee. Has opinions about 806.4 kHz clock selection. Will explain memory banking until you ask me to stop. Knows more about 1970s spacecraft computers than is strictly healthy or socially acceptable.

---

*"Voyager's last software update was in 1990. Thirty-five years of continuous operation, 24 billion kilometres from the nearest patch. We can't get a printer driver to survive a Windows update."*
