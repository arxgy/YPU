# YPU

#### Overview

A toy CPU based on tomasulo algorithm, supporting part of RV32I Instruction set.



#### Technical Features

- Tomasulo architecture with Reorder Buffer.
- Hardware Speculation and precise interruption, supported by 2-bit saturating counter predictor of 1024 entries.
- Directly-mapped instruction cache of 2048 entries.
- Reorder buffer of 16 entries.
- Well performance with relatively less hardware overhead.
- Pass all testcases on simulation and FPGA.
- Due to positive delay on 100MHz, YPU is quite possible to support higher frequencies. **(not test yet)**



#### Performance Table

| Testcases           | tak      | magic    | superloop | basicopt | bulgarian | queens   | Pi   |
| :------------------ | -------- | -------- | --------- | -------- | --------- | -------- | ---- |
| **Time on FPGA /s** | 0.074187 | 0.043482 | 0.033457  | 0.028221 | 1.228735  | 1.200035 | >3   |



#### Acknowledgement

Thanks Sirius, Pioooooo, XOR-op, happypig for answering questions and giving suggestions.

Thanks Lhtie for playing Overwatch with me, which helped me a lot on releasing pressures.

And at last, thanks Diana for no reasons.



