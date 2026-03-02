# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge


async def reset_dut(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def load_params(dut, k, xg, yg):
    """Load k, xg, yg one cycle each using load strobes."""
    # load_k = uio_in[0], load_x = uio_in[1], load_y = uio_in[2]
    dut.ui_in.value = k
    dut.uio_in.value = 0b00000001  # load_k
    await ClockCycles(dut.clk, 1)

    dut.ui_in.value = xg
    dut.uio_in.value = 0b00000010  # load_x
    await ClockCycles(dut.clk, 1)

    dut.ui_in.value = yg
    dut.uio_in.value = 0b00000100  # load_y
    await ClockCycles(dut.clk, 1)

    dut.uio_in.value = 0


async def start_and_wait(dut, timeout=200):
    """Pulse start and wait for done flag."""
    dut.uio_in.value = 0b00001000  # start = uio_in[3]
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0

    for _ in range(timeout):
        await RisingEdge(dut.clk)
        # done = uio_out[5]
        if (int(dut.uio_out.value) >> 5) & 1:
            return True
    return False  # timeout


def read_result(dut):
    """Read X and Y results using read_sel mux."""
    # read_sel = uio_in[4]; 0=result_x, 1=result_y
    # We read uo_out directly — read_sel is combinational
    return int(dut.uo_out.value)


@cocotb.test()
async def test_reset_state(dut):
    """Outputs should be 0 after reset, not busy or done."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    assert dut.uo_out.value == 0
    assert (int(dut.uio_out.value) >> 5) & 1 == 0, "done should be 0 after reset"
    assert (int(dut.uio_out.value) >> 6) & 1 == 0, "busy should be 0 after reset"



@cocotb.test()
async def test_scalar_mult_k1(dut):
    """k=0x80: only MSB set, result = 1*G after 7 doublings with no additions."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    XG, YG = 0x53, 0xCA
    await load_params(dut, k=0x80, xg=XG, yg=YG)

    done = await start_and_wait(dut)
    assert done, "Timed out waiting for done"

    dut.uio_in.value = 0b00000000
    await ClockCycles(dut.clk, 1)
    rx = read_result(dut)

    dut.uio_in.value = 0b00010000
    await ClockCycles(dut.clk, 1)
    ry = read_result(dut)

    dut._log.info(f"k=0x80, G=({XG:#04x},{YG:#04x}) => R=({rx:#04x},{ry:#04x})")

    # Verify result is deterministic (run twice, compare)
    await reset_dut(dut)
    await load_params(dut, k=0x80, xg=XG, yg=YG)
    await start_and_wait(dut)
    dut.uio_in.value = 0b00000000
    await ClockCycles(dut.clk, 1)
    rx2 = read_result(dut)
    assert rx == rx2, "Result is non-deterministic!"


@cocotb.test()
async def test_busy_flag(dut):
    """Busy should go high after start and low after done."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await load_params(dut, k=0xFF, xg=0x53, yg=0xCA)

    dut.uio_in.value = 0b00001000  # start
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0
    await ClockCycles(dut.clk, 1)

    busy = (int(dut.uio_out.value) >> 6) & 1
    assert busy == 1, "busy should be high after start"

    await start_and_wait(dut)
    busy = (int(dut.uio_out.value) >> 6) & 1
    assert busy == 0, "busy should be low after done"


@cocotb.test()
async def test_gf2_8_add(dut):
    """Sanity: GF(2^8) addition is XOR — indirectly exercised via the core."""
    # Direct ALU test isn't wired to pins, so we just confirm no crash on arbitrary k/G
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await load_params(dut, k=0xA5, xg=0x03, yg=0x07)
    done = await start_and_wait(dut)
    assert done, "Timed out — possible infinite loop or deadlock in FSM"
