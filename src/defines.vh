// SPDX-FileCopyrightText: 2024 Roman Penyaev
// SPDX-License-Identifier: BSD-2-Clause

`ifndef PIO_DEFINES
`define PIO_DEFINES

//
// Actions
//
`define NONE           0
`define INSTR          1
`define PEND           2
`define PULL           3
`define PUSH           4
`define GRPS           5
`define EN             6
`define DIV            7
`define SIDES          8
`define IMM            9
`define SHIFT          10
// IRQ read registers
`define RD_IRQ         11
`define RD_INTR        12
`define RD_IRQ0_INTE   13
`define RD_IRQ0_INTF   14
`define RD_IRQ0_INTS   15
`define RD_IRQ1_INTE   16
`define RD_IRQ1_INTF   17
`define RD_IRQ1_INTS   18
// IRQ write registers
`define WR_IRQ         19
`define WR_IRQ_FORCE   20
`define WR_IRQ0_INTE   21
`define WR_IRQ0_INTF   22
`define WR_IRQ1_INTE   23
`define WR_IRQ1_INTF   24
// Rest
`define IN_SYNC_BYPASS 25

`endif //  `ifndef PIO_DEFINES
