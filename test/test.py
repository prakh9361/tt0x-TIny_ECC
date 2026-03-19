# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

# =========================================================================
# PYTHON GOLDEN MODEL FOR GF(2^8) ECC
# =========================================================================
POLY = 0x11B
CURVE_A = 0x20
CURVE_B = 0x01  

def gf_add(a, b):
    return a ^ b

def gf_mult(a, b):
    p = 0
    for _ in range(8):
        if b & 1:
            p ^= a
        carry = a & 0x80
        a <<= 1
        if carry:
            a ^= POLY
        b >>= 1
    return p

def gf_inv(a):
    if a == 0: return 0
    for i in range(1, 256):
        if gf_mult(a, i) == 1:
            return i
    return 0

def is_point_on_curve(x, y):
    """Check if (x, y) satisfies y^2 + xy = x^3 + ax^2 + b"""
    if x == 0 and y == 0:
        return False # (0,0) is our marker for Point at Infinity, but not a valid affine point here
    y_sq = gf_mult(y, y)
    xy = gf_mult(x, y)
    left = gf_add(y_sq, xy)
    
    x_sq = gf_mult(x, x)
    x_cb = gf_mult(x_sq, x)
    ax_sq = gf_mult(CURVE_A, x_sq)
    right = gf_add(gf_add(x_cb, ax_sq), CURVE_B)
    
    return left == right

def find_valid_base_point():
    """Finds the first valid (x,y) point on the curve to use for testing."""
    for x in range(1, 256):
        for y in range(1, 256):
            if is_point_on_curve(x, y):
                return x, y
    raise ValueError("No valid points found on this curve!")


def point_double(x1, y1):
    # If the point is already at infinity (0,0) 
    # OR if the point has a vertical tangent (X=0)
    if x1 == 0: 
        return 0, 0

    lam = gf_add(x1, gf_mult(y1, gf_inv(x1)))
    lam_sq = gf_mult(lam, lam)
    x3 = gf_add(gf_add(lam_sq, lam), CURVE_A)
    
    x1_sq = gf_mult(x1, x1)
    lam_x3 = gf_mult(lam, x3)
    y3 = gf_add(gf_add(x1_sq, lam_x3), x3)
    return x3, y3

def point_add(x1, y1, x2, y2):
    # If point is at infinity (represented by x=0, y=0 in this test structure)
    if x1 == 0 and y1 == 0: return x2, y2
    if x2 == 0 and y2 == 0: return x1, y1
    
    # If points are inverses of each other (x1 == x2, y1 != y2 or y1 == x1 + y2) -> return Infinity
    if x1 == x2:
        if y1 == gf_add(x2, y2):
            return 0, 0
        else:
            return point_double(x1, y1)

    lam = gf_mult(gf_add(y1, y2), gf_inv(gf_add(x1, x2)))
    lam_sq = gf_mult(lam, lam)
    x3 = gf_add(gf_add(gf_add(gf_add(lam_sq, lam), x1), x2), CURVE_A)
    
    x1_plus_x3 = gf_add(x1, x3)
    lam_x1_x3 = gf_mult(lam, x1_plus_x3)
    y3 = gf_add(gf_add(lam_x1_x3, x3), y1)
    return x3, y3

def ecc_scalar_mult_golden(k, xg, yg):
    if k == 0: return 0, 0
    msb = 7
    while msb >= 0 and not (k & (1 << msb)):
        msb -= 1
    if msb < 0: return 0, 0

    xr, yr = xg, yg
    for i in range(msb - 1, -1, -1):
        xr, yr = point_double(xr, yr)
        # If doubling hit infinity, propagate it safely
        if xr == 0 and yr == 0:
            pass # Stays at infinity
            
        if k & (1 << i):
            xr, yr = point_add(xr, yr, xg, yg)
            
    return xr, yr

# =========================================================================
# COCOTB HARDWARE DRIVERS
# =========================================================================

async def reset_dut(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

async def load_params(dut, k, xg, yg):
    dut.ui_in.value = k
    dut.uio_in.value = 0b00000001
    await ClockCycles(dut.clk, 1)

    dut.ui_in.value = xg
    dut.uio_in.value = 0b00000010
    await ClockCycles(dut.clk, 1)

    dut.ui_in.value = yg
    dut.uio_in.value = 0b00000100
    await ClockCycles(dut.clk, 1)

    dut.uio_in.value = 0

async def start_and_wait(dut, timeout=1000):
    dut.uio_in.value = 0b00001000
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0

    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if (int(dut.uio_out.value) >> 5) & 1:
            return True
    return False

def read_result(dut):
    return int(dut.uo_out.value)

# =========================================================================
# TESTS
# =========================================================================

@cocotb.test()
async def test_invalid_curve_point(dut):
    """Test that the hardware correctly rejects points that do not lie on the elliptic curve."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Inject a mathematically invalid point (guaranteed to not equal B=1)
    # Note: 0x00, 0x00 is invalid on b=1
    INVALID_X, INVALID_Y = 0x00, 0x00 
    
    await load_params(dut, k=5, xg=INVALID_X, yg=INVALID_Y)
    done = await start_and_wait(dut)
    
    assert done, "Hardware timeout waiting for invalid point validation"
    
    # Check if hardware caught the invalid point and asserted the error flag
    error_flag = (int(dut.uio_out.value) >> 7) & 1
    assert error_flag == 1, f"Hardware ACCEPTED an invalid point ({INVALID_X:#04x}, {INVALID_Y:#04x})! Error flag was not raised."
    
    dut._log.info("SUCCESS: Hardware successfully rejected the invalid curve point!")


@cocotb.test()
async def test_poi_k_zero(dut):
    """Test the Point at Infinity case where scalar k = 0."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    XG, YG = find_valid_base_point()

    # K=0 represents the Point at Infinity
    await load_params(dut, k=0, xg=XG, yg=YG)
    done = await start_and_wait(dut)
    
    assert done, "Hardware timeout waiting for k=0 operation"
    
    # Read Status Flags (uio_out[7] is status_error)
    error_flag = (int(dut.uio_out.value) >> 7) & 1
    assert error_flag == 1, "Hardware failed to raise ERROR flag for Point at Infinity (k=0)"
    
    # Verify outputs are scrubbed to 0
    dut.uio_in.value = 0b00000000 # Read X
    await ClockCycles(dut.clk, 1)
    assert read_result(dut) == 0, "Result X should be 0 when returning Point at Infinity"

    dut._log.info("SUCCESS: Point at Infinity correctly raises error flag and returns 0!")


@cocotb.test()
async def test_exhaustive_keys(dut):
    """Exhaustively test all 256 possible scalar keys (k) against the golden model."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Automatically find a valid point for the configured curve
    XG, YG = find_valid_base_point()
    dut._log.info(f"Using Valid Base Point on Curve B={CURVE_B:#04x}: XG={XG:#04x}, YG={YG:#04x}")

    for k in range(1, 256):
        # 1. Compute expected result in Python
        expected_x, expected_y = ecc_scalar_mult_golden(k, XG, YG)

        # 2. Compute result in Verilog Hardware
        await load_params(dut, k=k, xg=XG, yg=YG)
        done = await start_and_wait(dut)
        
        assert done, f"Hardware timeout for k={k:#04x}"
        
        # Hardware Error Flag Check for intermediate POI
        error_flag = (int(dut.uio_out.value) >> 7) & 1
        
        # Read X
        dut.uio_in.value = 0b00000000
        await ClockCycles(dut.clk, 1)
        hw_x = read_result(dut)

        # Read Y
        dut.uio_in.value = 0b00010000
        await ClockCycles(dut.clk, 1)
        hw_y = read_result(dut)

        # 3. Exhaustively Compare
        if expected_x == 0 and expected_y == 0:
            assert error_flag == 1, f"Expected ERROR flag for POI at k={k:#04x}"
            assert hw_x == 0 and hw_y == 0, f"Expected 0,0 output for POI at k={k:#04x}"
        else:
            assert error_flag == 0, f"Unexpected ERROR flag raised at k={k:#04x}"
            assert hw_x == expected_x, f"X Mismatch at k={k:#04x}! HW: {hw_x:#04x}, Expected: {expected_x:#04x}"
            assert hw_y == expected_y, f"Y Mismatch at k={k:#04x}! HW: {hw_y:#04x}, Expected: {expected_y:#04x}"
        
    dut._log.info("SUCCESS: All 255 K scalar operations completely match the Python golden model!")
