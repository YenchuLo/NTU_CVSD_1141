# HW1 - ALU (Arithmetic Logic Unit) Design

## Project Overview

This project is Homework 1 for the Computer Vision and System Design course (CVSD 114-1) at National Taiwan University, Department of Electrical Engineering. It implements an Arithmetic Logic Unit (ALU) that supports multiple arithmetic and logic operations.

## Project Structure

```
HW1/
├── 00_TESTBED/          # Testbench and test patterns
│   ├── hidden/         # Hidden test patterns
│   ├── pattern/        # Public test patterns
│   └── testbench.v     # Testbench file
├── 01_RTL/             # RTL design files
│   ├── alu.v           # Main ALU design file
│   ├── rtl.f           # RTL file list
│   ├── 01_run          # Execution script
│   └── 99_clean        # Cleanup script
├── b11901092_score.txt # Test score report
└── README.md           # This file
```

> **Note**: The assignment specification PDFs are not included in this repository due to potential license restrictions. All necessary information for understanding and using this design is documented in this README.

## Design Specifications

### Parameters
- `INST_W = 4`: Instruction width (4 bits)
- `INT_W = 6`: Integer part bit width
- `FRAC_W = 10`: Fractional part bit width
- `DATA_W = 16`: Data width (INT_W + FRAC_W)

### Data Format
- Uses fixed-point representation
- Format: `[6-bit integer].[10-bit fraction]`
- Most significant bit is the sign bit

### Supported Instructions

| Opcode | Instruction Name | Description | Latency |
|--------|------------------|-------------|---------|
| `0000` | Signed Addition | Signed addition with saturation overflow handling. Detects positive/negative overflow and saturates to max/min values. | 1 cycle |
| `0001` | Signed Subtraction | Signed subtraction with saturation overflow handling. Detects positive/negative overflow and saturates to max/min values. | 1 cycle |
| `0010` | Signed MAC | Signed multiply-accumulate operation. Takes two cycles: first cycle accumulates the product, second cycle outputs the rounded result. | 2 cycles |
| `0011` | Taylor Expansion of Sin | Computes sin(x) using Taylor series: sin(x) ≈ x - x³/6 + x⁵/120. Input is 12-bit fixed-point, output is rounded to 16-bit. | 1 cycle |
| `0100` | Binary to Gray Code | Converts binary to Gray code using XOR operation: G[i] = B[i] ⊕ B[i+1] for i < 15, G[15] = B[15]. | 1 cycle |
| `0101` | LRCW | Left rotate with carry and wrap. Rotates `data_b` left by the number of bits specified in `data_a[3:0]`, with carry bit wrapping. | 16 cycles |
| `0110` | Right Rotation | Right rotation operation. Rotates `data_a` right by the number of bits specified in `data_b[3:0]`. | Variable |
| `0111` | Count Leading Zeros | Counts the number of leading zeros in `data_a`. Returns 16 if all bits are zero. | Variable |
| `1000` | Reverse Match4 | Reverse 4-bit pattern matching. Checks if 4-bit patterns in `data_a` match reversed patterns in `data_b`. | 1 cycle |
| `1001` | Matrix Transpose | 8x8 matrix transpose. Requires 8 cycles to input the matrix (2 bits per element), then outputs 8 rows. | 8 input + 8 output cycles |

## State Machine Design

The ALU uses a Finite State Machine (FSM) to control the computation flow:

- **S_IDLE (2'b00)**: Idle state, waiting for input
- **S_BUSY (2'b01)**: Busy state, executing operation
- **S_OUTPUT (2'b10)**: Output state, outputting result

## Interface Signals

### Input Signals
- `i_clk`: Clock signal
- `i_rst_n`: Reset signal (active low)
- `i_in_valid`: Input valid signal
- `i_inst[3:0]`: Instruction opcode
- `i_data_a[15:0]`: Operand A
- `i_data_b[15:0]`: Operand B

### Output Signals
- `o_busy`: Busy signal
- `o_out_valid`: Output valid signal
- `o_data[15:0]`: Operation result

## Usage

### Compilation and Simulation

```bash
cd 01_RTL
./01_run I0    # Run public test pattern I0
./01_run I1    # Run public test pattern I1
# ... and so on
./01_run H0    # Run hidden test pattern H0
# ... and so on
```

### Clean Temporary Files

```bash
cd 01_RTL
./99_clean
```

## Test Results

According to `b11901092_score.txt`, this design passes all tests:

- **Public Test Patterns**: I0 ~ I9 (10/10 PASS)
- **Hidden Test Patterns**: H0 ~ H4 (5/5 PASS)
- **File Check**: Submitted on time
- **Hierarchy Check**: PASS
- **Spyglass Check**: PASS
- **Final Score**: 100.00

## Design Features

1. **Overflow Handling**: Addition and subtraction operations include complete overflow detection and saturation handling
   - Positive overflow: saturates to `0x7FFF` (maximum positive value)
   - Negative overflow: saturates to `0x8000` (minimum negative value)
   
2. **Fixed-Point Arithmetic**: Supports fixed-point format arithmetic operations
   - Format: Q6.10 (6 integer bits, 10 fractional bits)
   - Sign-magnitude representation with two's complement
   
3. **Taylor Expansion**: Implements sin function approximation using Taylor series
   - Formula: sin(x) ≈ x - x³/6 + x⁵/120
   - Uses Booth multiplier for efficient multiplication
   - Input range: 12-bit fixed-point, output: 16-bit with rounding
   
4. **State Machine Control**: Uses FSM to manage computation flow, ensuring correct timing control
   - Three states: IDLE, BUSY, OUTPUT
   - Handles multi-cycle operations (MAC, rotation, matrix transpose)
   
5. **Matrix Operations**: Supports 8x8 matrix transpose operation
   - Matrix elements are 2-bit values
   - Requires 8 cycles for input, produces 8 cycles of output
   
6. **Booth Multiplier**: Implements signed multiplication using Booth's algorithm
   - Supports 16-bit × 16-bit multiplication
   - Handles sign extension correctly

## Notes

### Timing Requirements
- All operations use signed number representation (two's complement)
- Input data must be provided when `i_in_valid` is high
- New inputs should **not** be provided when `o_busy` is high
- Output results are valid when `o_out_valid` is high
- The design uses a handshaking protocol: `i_in_valid` → `o_busy` → `o_out_valid`

### Instruction-Specific Notes
- **MAC (0010)**: Requires two consecutive cycles. First cycle accumulates, second cycle outputs rounded result.
- **Right Rotation (0110)**: Latency depends on rotation amount (0-15 cycles).
- **Count Leading Zeros (0111)**: Latency depends on input value (0-16 cycles).
- **Matrix Transpose (1001)**: 
  - Input: 8 cycles, each cycle provides 2 bits for each of 8 matrix elements
  - Output: 8 cycles, each cycle outputs one row of the transposed matrix
  - Matrix size: 8×8, element width: 2 bits

### Implementation Details
- Reset is active low (`i_rst_n`)
- All registers are reset to zero on reset
- The design uses a two-stage pipeline: combinational logic for computation, sequential logic for state management
- MAC operation uses a 36-bit accumulator for intermediate calculations

## Author Information

- Student ID: b11901092
- Course: NTU CVSD 114-1

## Dependencies

- **Simulator**: VCS (Synopsys) or compatible Verilog simulator
- **Linter**: Spyglass (for design rule checking)
- **Language**: Verilog-2001 with SystemVerilog extensions

## License

This project is for educational purposes only. The RTL design and testbench are provided as-is for learning and reference.

> **Disclaimer**: The assignment specification PDFs are not included in this repository. Please refer to the course materials for the original assignment requirements.
