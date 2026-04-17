# Complete Guide to the GTH Transceiver IP in Your ARINC 818 Design

> **Target Audience**: Someone with no prior transceiver knowledge.
> **FPGA Board**: Xilinx KCU105 (Kintex UltraScale XCKU040)
> **Application**: ARINC 818 (ADVB) video link at **4.25 Gbps**
> **Key Documents**: UG576 (GTH User Guide), PG182 (Transceiver Wizard Guide), UG917 (KCU105 Board Guide)
>
> **This document is a FRESH REWRITE** with corrected reset sequence details,
> verified against UG576, PG182, and the actual Verilog code you provided.

---

## Table of Contents

1.  [What Is a Transceiver and Why Do You Need One?](#1-what-is-a-transceiver-and-why-do-you-need-one)
2.  [The Big Picture — System Architecture](#2-the-big-picture--system-architecture)
3.  [Inside the GTH — The Two Halves (PMA and PCS)](#3-inside-the-gth--the-two-halves-pma-and-pcs)
4.  [Clocking — Every Clock in Your Design](#4-clocking--every-clock-in-your-design)
5.  [**The Reset Sequence — The Most Important Section**](#5-the-reset-sequence--the-most-important-section)
6.  [8b/10b Encoding](#6-8b10b-encoding)
7.  [Comma Alignment](#7-comma-alignment)
8.  [The Elastic Buffer](#8-the-elastic-buffer)
9.  [The DRP Interface](#9-the-drp-interface)
10. [Loopback Modes](#10-loopback-modes)
11. [Module-by-Module Code Walkthrough](#11-module-by-module-code-walkthrough)
12. [**Your Specific Problem: "PMA TX Reset Done but Digital Part Stuck"**](#12-your-specific-problem-pma-tx-reset-done-but-digital-part-stuck)
13. [Hardware Debugging Checklist](#13-hardware-debugging-checklist)
14. [Code Bugs Found](#14-code-bugs-found)
15. [Glossary](#15-glossary)

---

## 1. What Is a Transceiver and Why Do You Need One?

### The Problem

Your ARINC 818 design generates a **1024×768 image at 24 bits/pixel**. This image
must be sent as a serial stream over a single fibre-optic cable at
**4.25 Gbps** (4.25 billion bits per second). Normal FPGA I/O pins can only
toggle at a few hundred MHz — nowhere near fast enough.

### The Solution

A **GTH Transceiver** is a piece of **hardened silicon** — actual analog and
digital circuitry baked directly into the FPGA chip (NOT built from
programmable logic). It can:

| Capability              | What It Means                                                              |
| ----------------------- | -------------------------------------------------------------------------- |
| **Serialize**           | Take your 32-bit parallel data and convert it to one serial bit-stream     |
| **Deserialize**         | Take a serial bit-stream and convert it back to 32-bit parallel data       |
| **Run at very high speeds** | Up to 16.3 Gbps on KCU105 (you use 4.25 Gbps)                         |
| **Encode/Decode**       | Apply 8b/10b line coding automatically in hardware                         |
| **Recover the clock**   | Extract a clock from the incoming serial data (CDR)                        |
| **Drive fibre optics**  | Its electrical output directly drives an SFP+ optical module               |

### Simple Analogy

Think of it like a **postal sorting machine**:

- Your FPGA logic writes addresses on 32 envelopes simultaneously (parallel data).
- The transceiver takes those 32 envelopes, lines them up single-file, and shoots
  them down a conveyor belt at incredible speed (serial data on fibre).
- On the receiving end, another transceiver catches the envelopes, groups them
  back into batches of 32, and delivers them to the receiving logic.

---

## 2. The Big Picture — System Architecture

This diagram shows how everything connects based on your actual code.
The code you provided has this module hierarchy:

```
arinc818_top  (System Top Level — NOT shown in your paste but referenced)
  ├── IBUFDS_GTE3              — GT reference clock buffer
  ├── clk_wiz_0 (PLL)          — Generates 25 MHz, 100 MHz, 106.25 MHz
  ├── image_pattern_gen         — Test colour bars
  ├── arinc818_core             — TX/RX FIFOs + ADVB framer/deframer
  └── arinc818_trans_top        — ★ THE TRANSCEIVER SUBSYSTEM (your pasted code)
       ├── IBUF                 — Buffers the reset input
       ├── arinc818_trans_init  — Initialization retry state machine
       └── arinc818_trans_wrapper — Wrapper around the GT IP
            └── arinc818_trans  — ★ THE ACTUAL XILINX GT IP CORE
                 ├── GTHE3_CHANNEL primitive  (the physical silicon)
                 ├── Reset Controller Helper Block (automated reset FSM)
                 └── User Clocking Helper Blocks (BUFG_GT for TX/RX clocks)
```

```
┌─────────────────────────────────────────────────────────────────────┐
│                     arinc818_top (System Top)                       │
│                                                                     │
│  ┌──────────────┐   ┌──────────────────────────────────────────┐   │
│  │ clk_wiz_0    │   │         arinc818_core                    │   │
│  │ (PLL)        │   │                                          │   │
│  │ 25MHz ───────────►  async    advbtx       advbrx   async   │   │
│  │ 100MHz ──────────►  FIFO ──► (framer) ◄── (defrm) ◄─FIFO  │   │
│  │ 106.25MHz    │   └──────┬──────────────────────┬────────────┘   │
│  └──────────────┘          │  advb_txdata[31:0]   │ advb_rxdata    │
│                            │  advb_txcharisk[3:0] │ advb_rxcharisk │
│  ┌──────────────┐   ┌──────▼──────────────────────▼────────────┐   │
│  │ IBUFDS_GTE3  │   │       arinc818_trans_top                 │   │
│  │ (GT ref clk  │   │  ┌─────────────┐  ┌──────────────────┐  │   │
│  │  buffer)     │───►  │ _trans_init │  │ _trans_wrapper   │  │   │
│  │              │   │  │ (retry FSM) │──► │ _trans (GT IP)│  │   │
│  └──────────────┘   │  └─────────────┘  │  └──────────────┘│  │   │
│                     │                    └──────────┬───────┘  │   │
│                     └───────────────────────────────┼──────────┘   │
│                                      sfp_tx_p/n ◄──┘              │
│                                      sfp_rx_p/n ──►               │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                           ┌────────▼────────┐
                           │   SFP+ Module   │
                           │ (Fibre Optic)   │
                           └─────────────────┘
```

**KEY POINT**: The module you NEVER modify is `arinc818_trans` — that is the
Vivado-generated IP core. Everything else (`_top`, `_wrapper`, `_init`) is
scaffolding code around it.

---

## 3. Inside the GTH — The Two Halves (PMA and PCS)

The GTH transceiver is split into two **completely different** sub-blocks.
Understanding this split is **essential** for understanding your reset problem.

### 3.1 PMA — Physical Media Attachment (The Analog Half)

The PMA is the **analog** circuitry. Think of it as the transceiver's "muscles."
It handles raw electrical signals — things that operate at the full 4.25 GHz
serial rate.

```
┌─────────────────────────── PMA (ANALOG) ───────────────────────────┐
│                                                                     │
│  TX Side:                            RX Side:                      │
│  ┌────────────┐  ┌────────────┐     ┌────────────┐  ┌──────────┐  │
│  │ Serializer │─►│ TX Driver  │──►  │ RX Analog  │─►│   CDR    │  │
│  │ (parallel  │  │ (amplifier │ SFP │ Buffer /   │  │(Clock &  │  │
│  │  to serial)│  │  to wire)  │     │ Equalizer  │  │ Data     │  │
│  └────────────┘  └────────────┘     └────────────┘  │ Recovery)│  │
│       ▲                                              └────┬─────┘  │
│       │                                                   │        │
│  From PCS                                           ┌─────▼─────┐  │
│  (digital side)                                     │Deserializer│ │
│                                                     │(serial to  │ │
│                                                     │ parallel)  │ │
│                                                     └─────┬─────┘  │
│                                                           │        │
│                                                      To PCS        │
│                                                     (digital side) │
└─────────────────────────────────────────────────────────────────────┘
```

| PMA Component     | What It Does | Everyday Analogy |
| ----------------- | ------------ | ---------------- |
| **CPLL**          | Multiplies the 106.25 MHz reference clock up to 4.25 GHz for the serializer | A gear system that makes a wheel spin 40x faster |
| **Serializer**    | Converts 20 or 40 parallel bits into one high-speed serial stream | Loading individual letters onto a conveyor belt one-by-one |
| **TX Driver**     | Amplifies the serial signal for the wire/fibre | A megaphone making your voice louder |
| **RX Equalizer**  | Receives and cleans up the weak incoming signal | Noise-cancelling headphones |
| **CDR**           | Figures out the clock from the incoming data transitions (there is NO separate clock wire!) | A drummer figuring out the tempo by listening to the music |
| **Deserializer**  | Converts the serial stream back into 20 or 40 parallel bits | Taking letters off the belt and grouping them into words |

**KEY SIGNAL**: `TXPMARESETDONE` / `RXPMARESETDONE` — These signals tell you
that **only the PMA (analog) half** has completed its internal reset. The digital
half (PCS) might still be resetting!

### 3.2 PCS — Physical Coding Sublayer (The Digital Half)

The PCS is the **digital** logic. Think of it as the transceiver's "brain."
It handles encoding, decoding, alignment, and buffering — things that operate
at the user clock rate (106.25 MHz for your 32-bit data path).

```
┌─────────────────────────── PCS (DIGITAL) ──────────────────────────┐
│                                                                     │
│  TX Side:                            RX Side:                      │
│  ┌────────────┐  ┌────────────┐     ┌────────────┐  ┌──────────┐  │
│  │  8b/10b    │─►│  TX Buffer │──►  │ RX Elastic │─►│  8b/10b  │  │
│  │  Encoder   │  │  (FIFO)    │ PMA │ Buffer     │  │  Decoder │  │
│  └────────────┘  └────────────┘     │ (FIFO)     │  └──────────┘  │
│       ▲                             └────────────┘       │         │
│       │                                  ▲               │         │
│  Your 32-bit                        ┌────┘               ▼         │
│  parallel data                 ┌────┴───────┐     Your 32-bit     │
│  + K-char flags                │   Comma    │     parallel data    │
│                                │ Alignment  │     + K-char flags   │
│                                │   Logic    │                      │
│                                └────────────┘                      │
└─────────────────────────────────────────────────────────────────────┘
```

| PCS Component        | What It Does | Everyday Analogy |
| -------------------- | ------------ | ---------------- |
| **8b/10b Encoder**   | Converts every 8-bit byte into a 10-bit symbol for DC balance and error detection | Adding area codes to phone numbers |
| **8b/10b Decoder**   | Converts 10-bit symbols back to 8-bit bytes | Stripping area codes off |
| **TX Buffer**        | Small FIFO absorbing timing differences between your clock and the serializer clock | A waiting room — people arrive at different times but are seen in order |
| **RX Elastic Buffer** | Absorbs clock frequency differences between sender and receiver | A variable-speed conveyor belt at baggage claim |
| **Comma Alignment**  | Finds the K28.5 pattern to lock onto byte boundaries | Finding the capital letter at the start of a sentence when there are no spaces |

**KEY SIGNAL**: `TXRESETDONE` / `RXRESETDONE` — These tell you that
**the ENTIRE channel** (PMA + PCS together) has completed its reset.

### 3.3 The Critical Difference You MUST Understand

> ⚠️ **THIS IS THE KEY TO YOUR PROBLEM** ⚠️

| Signal | What It Means | Scope |
| ------ | ------------- | ----- |
| `TXPMARESETDONE` | "The analog part (PMA) has finished resetting" | PMA only (analog) |
| `TXRESETDONE` | "The ENTIRE channel (PMA + PCS) has finished resetting" | PMA + PCS (full) |
| `gtwiz_reset_tx_done_out` | "The wizard's reset controller confirms TX is fully ready" | Full TX path verified |

**The relationship — they happen in this ORDER:**

```
TXPMARESETDONE first  ──►  then TXRESETDONE  ──►  then gtwiz_reset_tx_done_out
   (analog done)             (analog+digital)         (wizard confirms all done)
```

If your manager says "PMA TX reset is done but the digital part is not":
- `TXPMARESETDONE` = 1 ✅ (analog OK)
- `TXRESETDONE` = 0 ❌ (PCS/digital NOT completing)
- `gtwiz_reset_tx_done_out` = 0 ❌ (wizard reports NOT done)

We explain exactly why this happens and how to fix it in
[Section 12](#12-your-specific-problem-pma-tx-reset-done-but-digital-part-stuck).

### 3.4 Known Hardware Behavior: The TXPMARESETDONE "Double-Pulse"

This is a **documented Xilinx behavior** (not a bug in your code):

`TXPMARESETDONE` (and `RXPMARESETDONE`) are known to **toggle multiple times**
during the reset sequence. They go HIGH, then LOW, then HIGH again before
finally settling at a stable HIGH:

```
TXPMARESETDONE: ──────┐     ┌──┐  ┌══════════════════
                      └─────┘  └──┘
                        ↑         ↑         ↑
                     First      Drops    Final stable
                     pulse      back     assertion
                     (false)    to 0     (real)
```

**Why this matters for your design:** Your code uses `TXPMARESETDONE` to
control the BUFG_GT reset:

```verilog
assign userclk_tx_reset_in = ~(&txpmaresetdone_out);
```

During the double-pulse, the BUFG_GT will:
1. Enable (clock starts) — first pulse
2. Disable (clock STOPS) — drop back to 0
3. Enable again (clock restarts) — final assertion

The Xilinx reset controller is **designed to handle this** through its internal
logic. But it means these signals are NOT reliable for use as general-purpose
"ready" indicators. **Always use `gtwiz_reset_tx_done_out` instead.**

---

## 4. Clocking — Every Clock in Your Design

### 4.1 Clock Map

```
                  ┌─────────────────────────┐
sysclk_p/n ──────► clk_wiz_0 (PLL)         │
(board osc.)     │                          │
                 │  clk_out1 = 106.25 MHz ──├──► mgt_clk106M25 (NOT used to
                 │                          │    drive GTH — available if needed)
                 │  clk_out2 = 25 MHz ──────├──► drp_clk25M ──► DRP clock AND
                 │                          │    free-running clock for reset
                 │                          │    controller AND init state machine
                 │  clk_out3 = 100 MHz ─────├──► clk_100M ──► image_pattern_gen,
                 │                          │    TX FIFO write side, RX FIFO read
                 │  clk_out4 ───────────────├──► (unused)
                 │                          │
                 │  locked ─────────────────├──► drp_clk25M_rst = locked
                 │                          │    rst_n = locked & !cpureset
                 └──────────────────────────┘

mgtrefclk0_x0y2_p/n ──► IBUFDS_GTE3 ──► mgtrefclk0_x0y2_int
(106.25 MHz differential)                      │
                                               ▼
                                        ┌──────────────┐
                                        │  GTH CPLL    │
                                        │ (inside the  │
                                        │  transceiver)│
                                        │ cplllock_out │
                                        └──────┬───────┘
                                               │
                                     ┌─────────┼─────────┐
                                     │                    │
                                  TXOUTCLK            RXOUTCLK
                                     │                    │
                                 ┌───▼───┐           ┌───▼───┐
                                 │BUFG_GT│           │BUFG_GT│
                                 └───┬───┘           └───┬───┘
                                     │                    │
                              tx_userclk2          rx_userclk2
                              (106.25 MHz)         (106.25 MHz)
                                     │                    │
                              To advbtx &          To advbrx &
                              TX FIFO rclk         RX FIFO wclk
```

### 4.2 Each Clock Explained

| Clock                  | Frequency   | Source | Purpose |
| ---------------------- | ----------- | ------ | ------- |
| `sysclk_p/n`          | (board osc.) | KCU105 oscillator | Input to PLL (`clk_wiz_0`) |
| `mgtrefclk0_x0y2_p/n` | **106.25 MHz** | External SMA or Si570 | **Reference clock for the CPLL** — determines serial line rate |
| `drp_clk25M`          | 25 MHz | `clk_wiz_0` output 2 | Free-running clock for: DRP, reset controller, init SM |
| `clk_100M`            | 100 MHz | `clk_wiz_0` output 3 | Image generator, TX FIFO write, RX FIFO read |
| `tx_userclk2`         | **106.25 MHz** | GTH TXOUTCLK via BUFG_GT | **TX logic clock** — runs advbtx and TX FIFO read |
| `rx_userclk2`         | **106.25 MHz** | GTH RXOUTCLK via BUFG_GT | **RX logic clock** — runs advbrx and RX FIFO write |

### 4.3 Why 106.25 MHz?

```
ARINC 818 wire rate = 4.25 Gbps
With 8b/10b: Symbol Rate = 4.25 Gbps / 10 bits = 425 Msymbols/sec
32-bit data path = 4 symbols per clock
User Clock = 425 / 4 = 106.25 MHz
```

### 4.4 The CPLL (Channel PLL)

The **Channel PLL** is a clock synthesizer **physically inside** the GTH.
Each channel has its own dedicated CPLL.

```
              ┌───────────────── CPLL ─────────────────────┐
              │                                             │
Ref Clock ────► Phase        VCO         Output            │
106.25 MHz   │ Detector ──► (2-6.25 GHz) ──► Dividers ────├──► Serial Clock
              │    ▲                                        │    (4.25 GHz)
              │    └──── Feedback Divider ◄─────────────────│
              │                                             │
              │ cplllock ──────────────────────────────────│──► To your logic
              └─────────────────────────────────────────────┘
```

**`cplllock_out`**: HIGH means CPLL has locked. **The reset controller will NOT
proceed until CPLL lock is HIGH.** If `cplllock_out` stays LOW:
1. Is the 106.25 MHz reference clock actually present? (Measure with scope)
2. Is it connected to the correct MGTREFCLK pins?
3. Is the IBUFDS_GTE3 buffer correctly instantiated?

### 4.5 The BUFG_GT and User Clocking Helper Block

```
GTH TXOUTCLK ──► BUFG_GT ──► tx_userclk2 (TX fabric logic)
GTH RXOUTCLK ──► BUFG_GT ──► rx_userclk2 (RX fabric logic)
```

**Rules:**
- You CANNOT use a regular BUFG or PLL/MMCM to buffer TXOUTCLK/RXOUTCLK
- Only BUFG_GT is allowed
- The Transceiver Wizard generates this automatically

**From your code:**
```verilog
// TX BUFG_GT held in reset until TX PMA is done
assign userclk_tx_reset_in[0:0] = ~(&txpmaresetdone_out);
// When txpmaresetdone = 0 → reset = 1 → BUFG_GT in reset → no clock
// When txpmaresetdone = 1 → reset = 0 → BUFG_GT runs → clock flows

// RX BUFG_GT held in reset until RX PMA is done
assign userclk_rx_reset_in = ~(&rxpmaresetdone_out);
```

---

## 5. The Reset Sequence — The Most Important Section

This is the **#1 most important** section for understanding your problem.

### 5.1 Why Is Reset So Complicated?

The GTH has many sub-blocks that must be initialized in a specific order:

1. The silicon must power up (GTPOWERGOOD)
2. The CPLL must lock onto the reference clock
3. The analog circuits (PMA) must calibrate
4. The user clocks must start and stabilize
5. The fabric must signal "I'm ready" (TXUSERRDY/RXUSERRDY)
6. The digital circuits (PCS) must be released from reset
7. For RX: CDR must lock, comma alignment must succeed, elastic buffer must sync

Xilinx provides a **Reset Controller Helper Block** (inside the generated IP)
to handle this entire sequence automatically.

> **IMPORTANT (verified from PG182):**
>
> After device configuration, the reset controller **internally holds ALL PLL
> and datapath resources in reset until GTPOWERGOOD is HIGH** from all
> transceiver channels. It then **automatically runs one full reset-all
> sequence**. This means you do NOT need to explicitly pulse
> `gtwiz_reset_all_in` for the first initialization — it happens
> automatically once the silicon is powered up.
>
> However, if the PLL **loses lock** after `gtwiz_reset_tx_done_out` has
> asserted, the done signal **drops but does NOT auto-restart the reset**.
> User intervention (or your init module's retry logic) is required to
> re-trigger a reset.

### 5.2 The Three Layers of Reset Logic in Your Design

```
Layer 3: arinc818_trans_init (YOUR code)
   │      "The retry manager"
   │      Adds timeout + automatic retry if TX or RX don't complete in time
   │
   ▼
Layer 2: Reset Controller Helper Block (INSIDE the arinc818_trans IP)
   │      "The sequencer"
   │      Automatically drives GTTXRESET, TXUSERRDY, GTRXRESET, RXUSERRDY
   │      in the correct sequence with proper timing
   │
   ▼
Layer 1: GTHE3_CHANNEL primitive (THE PHYSICAL SILICON)
         "The hardware"
         Responds to reset signals, produces PMARESETDONE, RESETDONE, etc.
```

### 5.3 The Reset Controller's INTERNAL State Machine (from PG182)

This is what happens **inside** the Xilinx-generated IP (`arinc818_trans`)
when `gtwiz_reset_all_in` is asserted. **This was not explained correctly
in the previous document.**

#### TX Reset State Machine — All 6 States

```
     gtwiz_reset_all_in = 1
                │
                ▼
┌─────────────────────────────────────────────────────────────┐
│  State 1: ST_RESET_TX_PLL                                   │
│                                                              │
│  What the reset controller does:                             │
│    - Asserts CPLL reset = 1  (forces PLL to lose lock)       │
│    - Asserts GTTXRESET = 1   (holds TX channel in reset)     │
│    - De-asserts TXUSERRDY = 0 (tells GT: fabric NOT ready)   │
│                                                              │
│  In plain English:                                           │
│    "Reset the clock PLL and tell the transceiver             │
│     that nobody is ready yet"                                │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  State 2: ST_RESET_TX_DATAPATH                              │
│                                                              │
│  What the reset controller does:                             │
│    - Releases CPLL reset = 0  (PLL starts trying to lock)    │
│    - Keeps GTTXRESET = 1      (TX still held in reset)       │
│    - Keeps TXUSERRDY = 0      (fabric still not ready)       │
│                                                              │
│  In plain English:                                           │
│    "Let the PLL start locking, but keep everything else      │
│     in reset while we wait"                                  │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  State 3: ST_RESET_TX_WAIT_LOCK                             │
│                                                              │
│  What the reset controller waits for:                        │
│    cplllock = 1 (the CPLL has locked onto the ref clock)     │
│                                                              │
│  In plain English:                                           │
│    "Wait until the PLL has successfully locked onto           │
│     the 106.25 MHz reference clock"                          │
│                                                              │
│  ⚠️ GETS STUCK HERE IF:                                      │
│    - Reference clock is not present                          │
│    - Reference clock is wrong frequency                      │
│    - IBUFDS_GTE3 not connected or wrong pins                 │
│                                                              │
│  HOW TO CHECK: Monitor cplllock_out with ILA                 │
└────────────────────────┬────────────────────────────────────┘
                         │ CPLL locked!
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  State 4: ST_RESET_TX_WAIT_USERRDY                          │
│                                                              │
│  What the reset controller does:                             │
│    Step A: Releases GTTXRESET = 0                            │
│      → This starts the PMA internal reset sequence           │
│      → PMA takes some time to calibrate internally           │
│      → When PMA is done: TXPMARESETDONE goes HIGH            │
│        (⚠️ may double-pulse! see Section 3.4)                │
│                                                              │
│    Step B: TXPMARESETDONE releases the BUFG_GT               │
│      → Because your code has:                                │
│        userclk_tx_reset_in = ~txpmaresetdone_out             │
│      → tx_userclk2 starts toggling at 106.25 MHz             │
│                                                              │
│    Step C: The clocking helper reports clock active           │
│      → gtwiz_userclk_tx_active_out = 1                       │
│                                                              │
│    Step D: Reset controller asserts TXUSERRDY = 1            │
│      → This tells the GT primitive: "The fabric side         │
│        is ready — user clocks are stable, you can             │
│        finish your PCS initialization"                        │
│                                                              │
│  ⚠️ GETS STUCK HERE IF:                                      │
│    - BUFG_GT is not producing a valid clock                  │
│    - TXPMARESETDONE double-pulse causes clock instability    │
│    - gtwiz_userclk_tx_active_out never goes HIGH             │
│                                                              │
│  HOW TO CHECK: Monitor gtwiz_userclk_tx_active_out           │
│  If it stays 0, the BUFG_GT clock is not starting            │
└────────────────────────┬────────────────────────────────────┘
                         │ User clock active + TXUSERRDY asserted
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  State 5: ST_RESET_TX_WAIT_RESETDONE                        │
│                                                              │
│  What the reset controller waits for:                        │
│    TXRESETDONE = 1 (from the GTHE3_CHANNEL primitive)        │
│                                                              │
│  In plain English:                                           │
│    "Wait for the ENTIRE transceiver channel —                │
│     both PMA (analog) AND PCS (digital) — to confirm         │
│     that all internal reset sequences are complete"           │
│                                                              │
│  ★★★ THIS IS "THE DIGITAL PART" YOUR MANAGER MENTIONED ★★★  │
│                                                              │
│  TXPMARESETDONE = 1 at this point means the PMA (analog)     │
│  is done. But the PCS (digital) inside the channel still     │
│  needs to finish its own initialization. TXRESETDONE goes    │
│  HIGH only when BOTH PMA and PCS have completed.             │
│                                                              │
│  If TXPMARESETDONE = 1 but TXRESETDONE = 0:                  │
│    → The PCS (digital part) is stuck!                        │
│    → See Section 12 for root cause analysis                   │
│                                                              │
│  ⚠️ GETS STUCK HERE IF:                                      │
│    - PCS can't complete initialization                       │
│    - TXUSERRDY was asserted before clock was truly stable    │
│    - Internal PCS counters haven't reached target values     │
│    - DRP register misconfiguration from IP generation        │
└────────────────────────┬────────────────────────────────────┘
                         │ TXRESETDONE = 1!
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  State 6: ST_RESET_TX_IDLE                                  │
│                                                              │
│  What the reset controller does:                             │
│    - Asserts gtwiz_reset_tx_done_out = 1                     │
│                                                              │
│  In plain English:                                           │
│    "TX is fully initialized and ready to send data!"          │
│                                                              │
│  ⚠️ WILL LEAVE THIS STATE IF:                                │
│    - CPLL loses lock → done goes to 0                        │
│    - A new reset is requested                                │
│    NOTE: The FSM does NOT auto-restart. User must trigger.    │
└─────────────────────────────────────────────────────────────┘
```

#### RX Reset State Machine — Same Structure

The RX path follows the **same 6-state pattern** but with RX-specific signals:

```
State 1: ST_RESET_RX_PLL           → Reset CPLL, assert GTRXRESET, deassert RXUSERRDY
State 2: ST_RESET_RX_DATAPATH      → Release CPLL reset, keep GTRXRESET
State 3: ST_RESET_RX_WAIT_LOCK     → Wait for CPLL lock
State 4: ST_RESET_RX_WAIT_USERRDY  → Release GTRXRESET, wait for RX clock active,
                                      assert RXUSERRDY
State 5: ST_RESET_RX_WAIT_RESETDONE → Wait for RXRESETDONE
State 6: ST_RESET_RX_IDLE          → Assert gtwiz_reset_rx_done_out = 1
```

**Key difference for RX**: The RX path additionally requires:
- The CDR to lock onto incoming data (needs valid data at SFP RX)
- Comma alignment to succeed (needs K28.5 characters in the stream)
- The elastic buffer to initialize
- `RXSYNCDONE` to assert (elastic buffer is synchronized)

This is why **RX can take much longer than TX** and may **never** complete if
there is no cable plugged in or no transmitter sending data.

### 5.4 The "Reset All" Supervisory State Machine

When `gtwiz_reset_all_in` is pulsed, a supervisory FSM coordinates TX and RX:

> **NOTE (from PG182):** The reset-all state machine is initiated by the
> **falling edge** of the synchronized `gtwiz_reset_all_in` input — not the
> rising edge. This means the sequence begins when `gtwiz_reset_all_in`
> transitions from HIGH to LOW.

```
gtwiz_reset_all_in = 1
     │
     ▼
Start TX reset  ──►  Wait for gtwiz_reset_tx_done_out = 1
                           │
                       TX done!
                           │
                    Start RX reset  ──►  Wait for gtwiz_reset_rx_done_out = 1
                                              │
                                          RX done!
                                              │
                                         BOTH DONE!
```

**TX resets first, then RX.** The TX clock must be available before the RX
path can synchronize (RX needs a running clock infrastructure).

### 5.5 Complete Reset Timeline (What You Should See in Hardware)

```
TIME ──────────────────────────────────────────────────────────────────►

Power On
  │
  │  GTPOWERGOOD = 0 → 1  (silicon powered, takes a few µs)
  │
  │  PLL (clk_wiz_0) locks → locked = 1
  │     drp_clk25M_rst = 1
  │     IBUF(!drp_clk25M_rst) → reset_all_buf_in = 0
  │     reset_all_in goes LOW (reset de-asserted — controller starts)
  │
  │  Reset Controller begins the TX reset sequence:
  │     ├── CPLL starts locking...
  │     │     cplllock_out: 0 → 1  (a few hundred µs)
  │     │
  │     ├── GTTXRESET goes 1 → 0 (PMA starts calibrating)
  │     │     txpmaresetdone_out: 0 → 1  ★ (may double-pulse!)
  │     │
  │     │     BUFG_GT releases → tx_userclk2 starts (106.25 MHz)
  │     │     gtwiz_userclk_tx_active_out: 0 → 1
  │     │
  │     │     Reset Controller asserts TXUSERRDY = 1
  │     │     PCS completes initialization...
  │     │     TXRESETDONE: 0 → 1
  │     │
  │     │  ★★★ gtwiz_reset_tx_done_out: 0 → 1  (TX READY!) ★★★
  │     │
  │     ├── Now RX reset starts:
  │     │     GTRXRESET goes 1 → 0
  │     │     rxpmaresetdone_out: 0 → 1  (may double-pulse)
  │     │     BUFG_GT releases → rx_userclk2 starts
  │     │     RXUSERRDY = 1
  │     │     CDR locks... comma alignment... elastic buffer sync...
  │     │     RXSYNCDONE: 0 → 1
  │     │     RXRESETDONE: 0 → 1
  │     │
  │     │  ★★★ gtwiz_reset_rx_done_out: 0 → 1  (RX READY!) ★★★
  │
  │  Init module sees both done:
  │     → Enters ST_MONITOR
  │     → init_done_out = 1
  │
  ▼  LINK IS UP!
```

### 5.6 The Init State Machine (`arinc818_trans_init`)

This module wraps around the Reset Controller to add **automatic retry logic**.

**State Machine (verified from your code):**

```
          reset_all_sync asserted (power-up or PLL loss)
                     │
                     ▼
             ┌───────────────┐
             │   ST_START    │  sm_init_active set to 1
             │   (2'd0)     │  Clear timer, remove all reset outputs
             └───────┬───────┘
                     │ (if sm_init_active = 1)
                     ▼
             ┌───────────────┐
             │  ST_TX_WAIT   │  Timer counts from 0
             │   (2'd1)     │  Waiting for gtwiz_reset_tx_done_out
             └───┬───────┬──┘
  tx_init_done   │       │ Timer hits 30ms = 750,000 cycles
  BEFORE timeout │       │ AND tx_init_done_sync still = 0
                 │       │ ──► reset_all_out = 1 (FULL reset!)
                 │       │ ──► retry_ctr + 1
                 │       │ ──► back to ST_START
                 ▼       │
             ┌───────────────┐
             │  ST_RX_WAIT   │  Timer restarts from 0
             │   (2'd2)     │  Waiting for gtwiz_reset_rx_done_out
             └───┬───────┬──┘
  rx_timer_sat   │       │ Timer hits 130ms = 3,250,000 cycles
  AND rx_init_   │       │ AND rx_init_done_sync still = 0
  done_sync = 1  │       │ ──► reset_rx_out = 1 (RX-only reset)
                 │       │ ──► retry_ctr + 1
                 │       │ ──► back to ST_START
                 ▼       │
             ┌───────────────┐
             │  ST_MONITOR   │  init_done_out = 1 (LINK IS UP!)
             │   (2'd3)     │
             └───┬───────┬──┘
      Link OK    │       │ rx_init_done_sync drops to 0 (link lost!)
      (stay here)│       │ ──► reset_rx_out = 1
                 │       │ ──► init_done_out = 0
                 ▼       │ ──► back to ST_START
           [keep monitoring]
```

**Timer calculations:**

| Timer | Duration | Cycles at 25 MHz | Purpose |
| ----- | -------- | ---------------- | ------- |
| TX | 30,000 µs = 30 ms | 750,000 | Max wait for TX init |
| RX | 130,000 µs = 130 ms | 3,250,000 | Max wait for RX init |

**What happens in a retry cycle:**

```
1. Init module: "TX didn't complete in 30ms!"
   → reset_all_out = 1 (pulse)
   → reset_all_in = reset_all_buf_in OR reset_all_init_in = 1
   → Reset Controller receives reset_all_in = 1
   → EVERYTHING resets: CPLL, TX, RX all restart from zero
   → retry_ctr_out increments

2. reset_all_out goes back to 0
   → reset_all_in goes back to 0 (if PLL still locked)
   → Reset Controller begins full sequence again
   → New 30ms timer starts in ST_TX_WAIT

3. If this keeps failing → retry_ctr_out counts up
   → Saturates at 15 (doesn't stop retrying, just counter maxes)
   → retry_ctr = 15 means "15+ retries — something is seriously wrong"
```

### 5.7 How Reset Signals Connect (from your code)

```
                    ┌──────────────────┐
drp_clk25M_rst ────► IBUF             │
(= locked from     │ .I(!drp_clk25M_  │──► reset_all_buf_in
 PLL)              │      rst)        │       │
                   └──────────────────┘       │
                                              ▼
reset_all_init_in  ─────────────────────► [ OR ] ──► reset_all_in
(from init block)                                       │
                                                        ▼
                                   ┌────────────────────────────────┐
                                   │     arinc818_trans_wrapper     │
                                   │  ┌──────────────────────────┐ │
                                   │  │  Reset Controller        │ │
                                   │  │  Helper Block            │ │
                                   │  │  (inside arinc818_trans) │ │
                                   │  │                          │ │
                                   │  │  gtwiz_reset_tx_done ────│─┼─► reset_tx_done_out
                                   │  │  gtwiz_reset_rx_done ────│─┼─► reset_rx_done_out
                                   │  └──────────────────────────┘ │
                                   └────────────────────────────────┘
                                              │           │
                                              ▼           ▼
                                   ┌─────────────────────────────────┐
                                   │     arinc818_trans_init         │
                                   │  tx_init_done_in ◄── reset_tx_done_out
                                   │  rx_init_done_in ◄── reset_rx_done_out
                                   │                                 │
                                   │  reset_all_out ─────► reset_all_init_in
                                   │               (FEEDBACK LOOP! ↑ goes back to OR)
                                   │  reset_rx_out ──────► reset_rx_datapath_in
                                   └─────────────────────────────────┘
```

### 5.8 Synchronizers in the Init Module

| Instance | Module | Purpose |
| -------- | ------ | ------- |
| `reset_synchronizer_reset_all_inst` | `reset_synchronizer` | Syncs `reset_all_in` into `drp_clk25M`. 5 FFs with **async reset** |
| `bit_synchronizer_tx_init_done_inst` | `bit_synchronizer` | Syncs `tx_init_done_in` (TXUSRCLK2 domain) into `drp_clk25M`. 5 FFs |
| `bit_synchronizer_rx_init_done_inst` | `bit_synchronizer` | Syncs `rx_init_done_in` (RXUSRCLK2 domain) into `drp_clk25M`. 5 FFs |

**Why 5 flip-flops?** For ultra-high reliability. Each additional FF multiplies
the MTBF (Mean Time Between Failures) for metastability by a large factor.
The `(* ASYNC_REG = "TRUE" *)` attribute tells Vivado to place them close
together for minimal routing delay.

**Reset synchronizer vs bit synchronizer:**
- `reset_synchronizer`: Has asynchronous set (reset asserts instantly, without needing a clock edge). Reset removal is synchronous (aligned to clock). Used for reset signals.
- `bit_synchronizer`: Purely synchronous (no async behavior). Used for status signals.

---

## 6. 8b/10b Encoding

### 6.1 What Is It?

8b/10b converts every 8-bit byte into a 10-bit "symbol." Costs 25% bandwidth but gives:

| Benefit | Explanation |
| ------- | ----------- |
| **DC Balance** | Roughly equal 1s and 0s, preventing voltage drift |
| **Guaranteed transitions** | Enough 0-to-1 and 1-to-0 changes for CDR lock |
| **Error detection** | Invalid 10-bit codes are immediately detectable |
| **Control characters** | Reserved K-codes for framing |

### 6.2 D-codes vs K-codes

| Type | Set by | Examples | Used for |
| ---- | ------ | -------- | -------- |
| **D-codes** | `txcharisk = 0` | D0.0, D10.2, D31.7 | Pixel data |
| **K-codes** | `txcharisk = 1` | **K28.5**, K27.7 (SOF), K29.7 (EOF) | Framing, idle |

### 6.3 How It Works in Your Code

```verilog
assign tx8b10ben_in = 1'b1;    // TX encoding ON
assign rx8b10ben_in = 1'b1;    // RX decoding ON

assign txctrl2_in = {4'b0, advb_txcharisk};
// [7:4] = 0000 (unused padding — GTH port is 8 bits wide)
// [3:0] = advb_txcharisk (one K-flag per byte of 32-bit word)

assign advb_rxcharisk = rxctrl0_out[3:0];
// rxctrl0_out is 16 bits, only [3:0] matter for 32-bit data path
```

**How 4 K-char bits map to your 32-bit data:**
```
advb_txdata:     [  Byte 3  |  Byte 2  |  Byte 1  |  Byte 0  ]
advb_txcharisk:  [   bit 3  |  bit 2   |  bit 1   |  bit 0   ]

bit 0 = 1 → Byte 0 is a K-code (e.g., K28.5)
bit 0 = 0 → Byte 0 is normal data
```

---

## 7. Comma Alignment

### 7.1 The Problem

Serial bitstream has no byte boundaries visible:
```
...0111110000101100101000111011100010110...
```

### 7.2 The Solution

K28.5's bit pattern (`0011111010` / `1100000101`) can NEVER occur within or
across valid data symbols. Receiver scans for it and aligns accordingly.

### 7.3 Configuration (hardcoded in wrapper)

```verilog
.rxcommadeten_in    (1'b1)    // Enable comma detection (REQUIRED for 8b/10b)
.rxmcommaalignen_in (1'b1)    // Align on minus-disparity K28.5
.rxpcommaalignen_in (1'b1)    // Align on plus-disparity K28.5
```

### 7.4 Status Signals

| Signal | Meaning |
| ------ | ------- |
| `rxbyteisaligned_out` | HIGH = receiver locked onto byte boundaries |
| `rxbyterealign_out` | Pulses when receiver had to re-align |
| `rxcommadet_out` | Pulses each time a comma is detected |

---

## 8. The Elastic Buffer

### 8.1 Why It Exists

TX clock and RX clock are never exactly the same frequency. Even 1 PPM
difference eventually causes data loss. The elastic buffer compensates.

### 8.2 How It Works

```
Incoming data ──► [Elastic Buffer FIFO] ──► To your logic
(at CDR clock)                              (at rx_userclk2)

Buffer too full  → delete an IDLE character
Buffer too empty → insert an IDLE character
```

### 8.3 Status Signals

| Signal | Meaning |
| ------ | ------- |
| `txbufstatus_out[1]` | TX buffer overflow (data corruption!) |
| `txbufstatus_out[0]` | TX buffer underflow (data corruption!) |
| `rxbufstatus_out` | RX buffer fill level |

If `txbufstatus_out` is ever non-zero, you have a clocking problem.

---

## 9. The DRP Interface

The **Dynamic Reconfiguration Port** — a register interface for runtime config.
Your code ties it all off (unused):

```verilog
assign drpaddr_in = 9'b0;
assign drpdi_in   = 16'b0;
assign drpen_in   = 1'b0;
assign drpwe_in   = 1'b0;
// drpclk_in connected to drp_clk25M (25 MHz)
```

Normal for a fixed-configuration design.

---

## 10. Loopback Modes

```verilog
assign loopback = 3'b000;  // Normal operation
```

| `loopback[2:0]` | Mode | What Happens |
| --------------- | ---- | ------------ |
| `000` | Normal | TX to SFP, RX from SFP |
| `001` | Near-end PCS | TX PCS loops to RX PCS (digital) |
| `010` | Near-end PMA | TX PMA loops to RX PMA (full analog) |
| `100` | Far-end PMA | Remote data echoed at PMA |
| `110` | Far-end PCS | Remote data echoed at PCS |

**For debugging**: Set `loopback = 3'b010` to test without cable/SFP. If this
works, your GT config is correct and the issue is physical.

---

## 11. Module-by-Module Code Walkthrough

### 11.1 `arinc818_trans_top` — Transceiver Subsystem Top

Every assignment explained:

```verilog
// ── User Clocking Reset ──
assign userclk_tx_reset_in = ~(&txpmaresetdone_out);
// Hold BUFG_GT in reset until PMA TX is done

assign userclk_rx_reset_in = ~(&rxpmaresetdone_out);
// Hold BUFG_GT in reset until PMA RX is done

// ── Manual Reset Control ──
assign reset_tx_pll_and_datapath_in = 1'b0;  // Never manually reset TX PLL
assign reset_rx_pll_and_datapath_in = 1'b0;  // Never manually reset RX PLL
assign reset_tx_datapath_in         = 1'b0;  // Never manually reset TX datapath
// reset_rx_datapath_in driven by init module (for retry)

// ── 8b/10b ──
assign tx8b10ben_in = 1'b1;  // TX 8b/10b encoding ON
assign rx8b10ben_in = 1'b1;  // RX 8b/10b decoding ON

// ── Loopback ──
assign loopback = 3'b000;    // Normal operation

// ── TX Data ──
assign txctrl0_in = 16'b0;                    // Unused in 8b/10b
assign txctrl1_in = 16'b0;                    // Unused in 8b/10b
assign txctrl2_in = {4'b0, advb_txcharisk};  // K-char flags

// ── RX K-chars ──
assign advb_rxcharisk = rxctrl0_out[3:0];    // Which RX bytes were K-codes

// ── DRP (unused) ──
assign drpaddr_in = 9'b0;
assign drpdi_in   = 16'b0;
assign drpen_in   = 1'b0;
assign drpwe_in   = 1'b0;
```

### 11.2 Reset Input Logic

```verilog
IBUF ibuf_reset_all_inst (
    .I (!drp_clk25M_rst),     // drp_clk25M_rst = locked (from PLL)
    .O (reset_all_buf_in)     // Output = !locked
);                             //   PLL NOT locked → 1 (reset active)
                               //   PLL IS locked  → 0 (reset released)

assign reset_all_in = reset_all_buf_in || reset_all_init_in;
// Reset active when: PLL not locked OR init module requesting retry
```

### 11.3 Init Module Instantiation

```verilog
arinc818_trans_init gth_init_inst (
    .clk_freerun_in  (drp_clk25M),          // 25 MHz free-running clock
    .reset_all_in    (reset_all_in),         // Master reset input
    .tx_init_done_in (reset_tx_done_out),    // gtwiz_reset_tx_done_out
    .rx_init_done_in (reset_rx_done_out),    // gtwiz_reset_rx_done_out
    .rx_data_good_in (sm_link),              // ⚠️ BUG: NEVER DECLARED (see Sec 14)
    .reset_all_out   (reset_all_init_in),    // Trigger full reset retry
    .reset_rx_out    (reset_rx_datapath_in), // Trigger RX-only reset retry
    .init_done_out   (init_done_in),         // Initialization complete status
    .retry_ctr_out   (init_retry_ctr_in)     // Debug retry counter
);
```

### 11.4 `arinc818_trans_wrapper`

Sits between `_trans_top` and the GT IP. Its jobs:
1. Pass-through most signals
2. Expose optional ports (CPLLLOCK, GTPOWERGOOD, RXSYNCDONE)
3. Hardcode comma alignment (all three enables = 1)
4. Include functions file for channel index calculation

### 11.5 `reset_synchronizer`

```verilog
// 5 flip-flops with ASYNCHRONOUS reset
always @(posedge clk_in, posedge rst_in) begin
    if (rst_in) begin
        // Instant assertion — no clock needed
        {rst_in_out, rst_in_sync3, rst_in_sync2, rst_in_sync1, rst_in_meta} <= 5'b11111;
    end else begin
        // Synchronous removal — through pipeline
        rst_in_meta  <= 1'b0;
        rst_in_sync1 <= rst_in_meta;
        rst_in_sync2 <= rst_in_sync1;
        rst_in_sync3 <= rst_in_sync2;
        rst_in_out   <= rst_in_sync3;
    end
end
assign rst_out = rst_in_out;
```

**Why async assert, sync de-assert?**
- Assert: Happens immediately (no need to wait for clock) — reset propagates instantly
- De-assert: Goes through 5 FF pipeline, aligned to clock — prevents metastability

### 11.6 `bit_synchronizer`

```verilog
// 5 flip-flops, purely synchronous
always @(posedge clk_in) begin
    i_in_meta  <= i_in;           // Capture (may be metastable!)
    i_in_sync1 <= i_in_meta;      // Resolution stage 1
    i_in_sync2 <= i_in_sync1;     // Resolution stage 2
    i_in_sync3 <= i_in_sync2;     // Resolution stage 3
    i_in_out   <= i_in_sync3;     // Output — safe to use
end
assign o_out = i_in_out;
```

Cost: 5 clock cycles of latency for a safely synchronized signal.

---

## 12. Your Specific Problem: "PMA TX Reset Done but Digital Part Stuck"

### 12.1 What You Are Seeing

Your manager's observation translates to:
- `TXPMARESETDONE` = 1 ✅ — the PMA (analog) part has completed its reset
- `gtwiz_reset_tx_done_out` = 0 ❌ — the full TX reset has NOT completed
- This means the PCS (digital) part is stuck

The Reset Controller is stuck in **State 4 or State 5** (see Section 5.3).

### 12.2 Possible Root Causes (Most Likely First)

#### Cause 1: TX User Clock Not Starting (State 4 stuck)

The BUFG_GT needs TXPMARESETDONE to release. TXPMARESETDONE double-pulses.
If the clocking helper block doesn't properly settle,
`gtwiz_userclk_tx_active_out` never goes HIGH, and the reset controller is stuck
waiting for the user clock.

**Check:** Probe `gtwiz_userclk_tx_active_out` in ILA. If it's 0, the clock
isn't starting.

#### Cause 2: PCS Can't Complete (State 5 stuck)

The PCS has internal initialization counters that need a running, stable user
clock. If the user clock frequency is wrong or had glitches during init, PCS
may never finish.

**Check:** If `gtwiz_userclk_tx_active_out` = 1 but `gtwiz_reset_tx_done_out`
is still 0, the PCS is stuck.

#### Cause 3: Init Module Retry Loop

If the TX reset takes longer than 30ms, the init module timeout fires:
- `reset_all_out` pulses → full reset → starts over → times out again → infinite loop

**Check:** Monitor `retry_ctr_out`. If it's incrementing continuously, you're
in a retry loop. The init module never gives the reset controller enough time.

#### Cause 4: Free-Running Clock Wrong Frequency

The reset controller's internal timers assume 25 MHz. If `drp_clk25M` deviates
significantly, internal timeouts fail.

**Check:** Verify `drp_clk25M` = 25 MHz with scope or ILA.

#### Cause 5: CPLL Lock Instability

CPLL locks initially, reset controller proceeds, then CPLL briefly loses lock.
Reset controller detects lock loss → `gtwiz_reset_tx_done_out` drops to 0.
From outside it looks like TX never completed.

**Check:** Monitor `cplllock_out` for any brief dropouts. Trigger ILA on
falling edge of `cplllock_out`.

### 12.3 Recommended Debugging Procedure

#### Step 1: Add an ILA

In Vivado, add an ILA IP core on the `drp_clk25M` clock domain.
Probe these signals:

**Must-probe (minimum set):**

| Signal | What to Look For |
| ------ | ---------------- |
| `cplllock_out` | Should go 1 and STAY 1 |
| `txpmaresetdone_out` | Should go 1 (may blink — that's normal) |
| `reset_tx_done_out` | **KEY: Must go 1** |
| `reset_rx_done_out` | Must go 1 (needs cable for normal mode) |
| `reset_all_in` | Should pulse 1 once, then stay 0 |
| `reset_all_init_in` | If pulsing repeatedly = retry loop |

**Nice-to-have:**

| Signal | What to Look For |
| ------ | ---------------- |
| `rxpmaresetdone_out` | Should go 1 |
| `userclk_tx_active_out` | Must go 1 for State 4 to proceed |
| `userclk_rx_active_out` | Must go 1 for RX State 4 |
| `rxbyteisaligned_out` | Should go 1 (comma alignment OK) |
| `rxsyncdone_out` | Should go 1 (elastic buffer OK) |
| `txbufstatus_out` | Should be 00 |

#### Step 2: Trigger and Analyze

```
Trigger: cplllock_out rising edge (0 → 1)
Depth: Maximum
Window: 25% pre-trigger, 75% post-trigger
```

Expected healthy waveform:
```
cplllock_out:        ___/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
txpmaresetdone_out:  _______/‾‾\__/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾  (double-pulse OK)
reset_tx_done_out:   _______________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
reset_all_in:        ‾‾‾‾\_________________________________
reset_all_init_in:   ___________________________________
```

Unhealthy — retry loop:
```
cplllock_out:        ___/‾‾‾‾‾‾‾‾‾‾‾‾‾‾\_/‾‾‾‾‾‾‾‾‾‾‾‾  (intermittent!)
reset_tx_done_out:   __________________________________  (never goes high!)
reset_all_in:        ‾‾‾‾\____/‾\_____/‾\______  (pulsing repeatedly!)
reset_all_init_in:   ________/‾\______/‾\______  (init module retrying!)
```

#### Step 3: Try Loopback Mode

Change your code:
```verilog
assign loopback = 3'b010;  // Near-end PMA loopback
```

Re-synthesize and test. If both `reset_tx_done_out` and `reset_rx_done_out`
go HIGH in loopback, your transceiver config is correct and the problem is
physical (cable, SFP, or reference clock).

#### Step 4: Verify Reference Clock

Use an oscilloscope to verify:
1. 106.25 MHz is present on the correct MGTREFCLK pins
2. Correct frequency (within ±100 ppm)
3. Clean differential waveform (LVDS or LVPECL)
4. Connected to the right Quad

If using KCU105 Si570, it must be programmed to 106.25 MHz via I2C first!

#### Step 5: Check for Vivado Warnings

Look in the synthesis and implementation logs for:
- Critical warnings about GT IP
- Timing failures on GT-related paths
- Placement issues for the GT channel

---

## 13. Hardware Debugging Checklist

```
Step 1: Program FPGA
  │
  ▼
Step 2: Check PLL lock (clk_wiz_0 locked)
  │    → NOT locked: sysclk missing or wrong frequency
  │
  ▼
Step 3: Check cplllock_out
  │    → NOT locked: 106.25 MHz ref clock missing
  │    → Measure with oscilloscope
  │
  ▼
Step 4: Check txpmaresetdone_out
  │    → NOT asserted: CPLL issue or fundamental GT problem
  │
  ▼
Step 5: Check gtwiz_reset_tx_done_out (NOT txpmaresetdone!)
  │    → txpma=1 but tx_done=0: ★ YOUR PROBLEM — see Section 12 ★
  │
  ▼
Step 6: Try loopback = 3'b010
  │    → Works: GT config OK, issue is physical
  │    → Still stuck: GT IP config problem
  │
  ▼
Step 7: Check rxpmaresetdone_out
  │    → May not assert without cable (CDR needs data)
  │
  ▼
Step 8: Check gtwiz_reset_rx_done_out
  │    → Needs cable + SFP + valid transmitter
  │
  ▼
Step 9: Check rxbyteisaligned_out
  │    → 0 = comma alignment failing
  │
  ▼
Step 10: Monitor retry_ctr_out
         → Incrementing = init module retrying repeatedly
```

---

## 14. Code Bugs Found

### Bug 1: Undeclared Signal `sm_link`

```verilog
.rx_data_good_in (sm_link),   // sm_link is NEVER DECLARED!
```

`sm_link` is connected to `rx_data_good_in` of the init module but never
declared as a wire or driven to any value.

**Impact:** Vivado creates an implicit wire. After synthesis it defaults to 0.
Looking at the init state machine, `rx_data_good_in` is synchronized but
**never actually checked** in any state transition (the FSM only checks
`rx_init_done_sync`). So this has **no functional effect** in the current code
— but is a latent bug.

**Fix:**
```verilog
// Option A: Declare and drive it
wire sm_link;
assign sm_link = 1'b1;

// Option B: Tie off at instantiation
.rx_data_good_in (1'b1),
```

### Bug 2: Undeclared `init_done_in` and `init_retry_ctr_in`

```verilog
.init_done_out   (init_done_in),     // Never declared
.retry_ctr_out   (init_retry_ctr_in) // Never declared
```

**Impact:** No functional effect (output-only), but generates synthesis warnings.

**Fix:** Declare them:
```verilog
wire        init_done_in;
wire [3:0]  init_retry_ctr_in;
```

### Bug 3: Duplicate `bit_synchronizer` Module Definition

In the code you pasted, `bit_synchronizer` is defined **twice**. This causes
a synthesis error ("module already defined").

**Fix:** Remove one duplicate definition. (May be a copy-paste issue in your
message only — verify your actual source files.)

### Bug 4: Syntax Error at End of Wrapper

```verilog
endmodule.    // The period (.) after endmodule is a syntax error
```

**Fix:** Change to just `endmodule`

---

## 15. Glossary

| Term | Full Name | Plain-English Meaning |
| ---- | --------- | --------------------- |
| **GTH** | Gigabit Transceiver High-perf | Type of transceiver in Kintex UltraScale |
| **CPLL** | Channel PLL | Clock multiplier for one GT channel |
| **QPLL** | Quad PLL | Clock multiplier shared by 4 GT channels |
| **PMA** | Physical Media Attachment | **Analog** half: serializer, driver, CDR |
| **PCS** | Physical Coding Sublayer | **Digital** half: 8b/10b, elastic buffer, comma align |
| **CDR** | Clock and Data Recovery | Extracts clock from serial data |
| **8b/10b** | 8-to-10 bit encoding | Line coding for DC balance and error detection |
| **K-code** | Control Character | Special 8b/10b symbols for framing |
| **K28.5** | Comma Character | Specific K-code for byte-boundary detection |
| **DRP** | Dynamic Reconfiguration Port | Register interface for runtime config |
| **BUFG_GT** | GT Global Buffer | Special clock buffer for GT clocks to fabric |
| **IBUFDS_GTE3** | GT Diff Input Buffer | Special buffer for GT reference clock inputs |
| **TXOUTCLK** | TX Output Clock | Raw TX clock from GTH (before BUFG_GT) |
| **TXUSRCLK2** | TX User Clock 2 | Your TX fabric logic clock (after BUFG_GT) |
| **TXPMARESETDONE** | TX PMA Reset Done | PMA (analog only) completed reset |
| **TXRESETDONE** | TX Reset Done | Entire channel (PMA+PCS) completed reset |
| **gtwiz_reset_tx_done_out** | Wizard TX Done | Reset controller confirms TX fully ready |
| **TXUSERRDY** | TX User Ready | Internal signal: fabric tells GT "I'm ready" |
| **GTTXRESET** | GT TX Reset | Internal signal: asserts TX reset |
| **GTPOWERGOOD** | GT Power Good | Silicon is powered and ready |
| **ILA** | Integrated Logic Analyzer | Vivado on-chip debug tool |
| **PPM** | Parts Per Million | Tiny frequency difference measurement |
| **SFP+** | Small Form-factor Pluggable | Fibre optic module |
| **ADVB** | ARINC 818 Digital Video Bus | Avionics video protocol |

---

## Quick Reference: Critical Signal Monitor Table

| Signal | Expected | If WRONG |
| ------ | -------- | -------- |
| `locked` (PLL) | HIGH | System clock missing |
| `cplllock_out` | HIGH | Ref clock (106.25 MHz) missing |
| `txpmaresetdone_out` | HIGH (may blink) | CPLL issue or GT stuck |
| **`gtwiz_reset_tx_done_out`** | **HIGH** | **PCS stuck — Section 12** |
| `rxpmaresetdone_out` | HIGH | No incoming data |
| **`gtwiz_reset_rx_done_out`** | **HIGH** | **RX not init — check cable/SFP** |
| `rxbyteisaligned_out` | HIGH | Comma detection failing |
| `txbufstatus_out` | 2'b00 | Clocking problem! |
| `retry_ctr_out` | Low/0 | High = init retrying |

---

> **THE SINGLE MOST IMPORTANT DEBUGGING TIP:**
>
> Get the **106.25 MHz reference clock** correct and stable FIRST.
> If `cplllock_out` is not HIGH, nothing else will work.
>
> After that, use an ILA to watch `gtwiz_reset_tx_done_out` and
> `gtwiz_reset_rx_done_out`. These are your definitive "transceiver ready"
> indicators — **NOT** `TXPMARESETDONE`.

---

*Document verified against UG576 (GTH User Guide), PG182 (Transceiver Wizard Guide), and the provided Verilog source code.*

*Key correction from previous version: Properly distinguishes TXPMARESETDONE (PMA-only, analog) from
TXRESETDONE/gtwiz_reset_tx_done_out (full channel PMA+PCS). Documents the internal 6-state reset
controller FSM and the TXPMARESETDONE double-pulse behavior. These were missing or incorrect before.*
