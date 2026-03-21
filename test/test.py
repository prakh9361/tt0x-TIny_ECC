# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

# =========================================================================
# GOLDEN MODEL
# Curve: y^2 + xy = x^3 + CURVE_A*x^2 + CURVE_B over GF(2^8) with poly 0x11B
# Parameters: a=0x20, b=0x01, n=113, G=(143,41), h=2
# =========================================================================
POLY    = 0x11B
CURVE_A = 0x20
CURVE_B = 0x01
GX      = 143
GY      = 41
N       = 113

def gf_mul(a, b):
    p = 0
    for _ in range(8):
        if b & 1:
            p ^= a
        carry = a & 0x80
        a = (a << 1) & 0xFF
        if carry:
            a ^= 0x1B
        b >>= 1
    return p

def gf_inv(a):
    if a == 0:
        return 0
    for i in range(1, 256):
        if gf_mul(a, i) == 1:
            return i
    return 0

def point_double(x1, y1):
    if x1 == 0:
        return 0, 0
    lam   = x1 ^ gf_mul(y1, gf_inv(x1))
    x3    = gf_mul(lam, lam) ^ lam ^ CURVE_A
    x1sq  = gf_mul(x1, x1)
    y3    = x1sq ^ gf_mul(lam, x3) ^ x3
    return x3, y3

def point_add(x1, y1, x2, y2):
    if x1 == 0 and y1 == 0: return x2, y2
    if x2 == 0 and y2 == 0: return x1, y1
    if x1 == x2:
        if y1 == (x2 ^ y2):
            return 0, 0
        return point_double(x1, y1)
    lam = gf_mul(y1 ^ y2, gf_inv(x1 ^ x2))
    x3  = gf_mul(lam, lam) ^ lam ^ x1 ^ x2 ^ CURVE_A
    y3  = gf_mul(lam, x1 ^ x3) ^ x3 ^ y1
    return x3, y3

def scalar_mult(k, xg, yg):
    if k == 0:
        return 0, 0
    msb = 7
    while msb >= 0 and not (k & (1 << msb)):
        msb -= 1
    if msb < 0:
        return 0, 0
    xr, yr = xg, yg
    for i in range(msb - 1, -1, -1):
        xr, yr = point_double(xr, yr)
        if k & (1 << i):
            xr, yr = point_add(xr, yr, xg, yg)
    return xr, yr

# =========================================================================
# HARDWARE DRIVERS
# =========================================================================

async def reset_dut(dut):
    dut.ena.value    = 1
    dut.ui_in.value  = 0
    dut.uio_in.value = 0
    dut.rst_n.value  = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 2)

async def load_params(dut, k, xg, yg):
    dut.ui_in.value  = k
    dut.uio_in.value = 0b00000001   # load_k
    await ClockCycles(dut.clk, 1)

    dut.ui_in.value  = xg
    dut.uio_in.value = 0b00000010   # load_x
    await ClockCycles(dut.clk, 1)

    dut.ui_in.value  = yg
    dut.uio_in.value = 0b00000100   # load_y
    await ClockCycles(dut.clk, 1)

    dut.uio_in.value = 0

async def start_and_wait(dut, timeout=2000):
    dut.uio_in.value = 0b00001000   # start
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = 0

    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if (int(dut.uio_out.value) >> 5) & 1:   # done flag
            return True
    return False

async def read_xy(dut):
    dut.uio_in.value = 0b00000000   # read_sel=0 -> X
    await ClockCycles(dut.clk, 1)
    hw_x = int(dut.uo_out.value)

    dut.uio_in.value = 0b00010000   # read_sel=1 -> Y
    await ClockCycles(dut.clk, 1)
    hw_y = int(dut.uo_out.value)

    dut.uio_in.value = 0
    return hw_x, hw_y

def error_flag(dut):
    return (int(dut.uio_out.value) >> 7) & 1

# =========================================================================
# TEST 1 — k=0 gives point at infinity (hits SCAN_BIT, no valid MSB found)
# =========================================================================
@cocotb.test()
async def test_k_zero(dut):
    """k=0 must raise error (SCAN_BIT finds no set bit)."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    await load_params(dut, k=0, xg=GX, yg=GY)
    done = await start_and_wait(dut)

    assert done, "Timeout for k=0"
    assert error_flag(dut) == 1, "No error raised for k=0"

    hw_x, hw_y = await read_xy(dut)
    assert hw_x == 0 and hw_y == 0, f"Expected (0,0) for k=0, got ({hw_x},{hw_y})"
    dut._log.info("PASS: k=0 correctly returns error and (0,0)")

# =========================================================================
# TEST 2 — exhaustive scalar multiplication k=1..112
# =========================================================================
@cocotb.test()
async def test_exhaustive_scalar_mult(dut):
    """Test all valid private keys k=1..n-1 against golden model using G=(143,41)."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    dut._log.info(f"Curve: a={CURVE_A:#04x}, b={CURVE_B:#04x}, n={N}")
    dut._log.info(f"Base point G=({GX:#04x}, {GY:#04x})")

    for k in range(1, N):
        exp_x, exp_y = scalar_mult(k, GX, GY)

        await load_params(dut, k=k, xg=GX, yg=GY)
        done = await start_and_wait(dut)
        assert done, f"Timeout at k={k}"

        hw_x, hw_y = await read_xy(dut)
        ef = error_flag(dut)

        if exp_x == 0 and exp_y == 0:
            assert ef == 1,                         f"k={k}: expected error for POI"
            assert hw_x == 0 and hw_y == 0,         f"k={k}: expected (0,0) for POI"
        else:
            assert ef == 0,  f"k={k}: unexpected error flag"
            assert hw_x == exp_x, f"k={k}: X mismatch HW={hw_x:#04x} expected={exp_x:#04x}"
            assert hw_y == exp_y, f"k={k}: Y mismatch HW={hw_y:#04x} expected={exp_y:#04x}"

    dut._log.info(f"PASS: all {N-1} scalar multiplications match golden model")

# =========================================================================
# TEST 3 — Diffie-Hellman shared secret
# =========================================================================
@cocotb.test()
async def test_diffie_hellman(dut):
    """Full DH key exchange: Alice and Bob derive the same shared secret."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    alice_priv = 42
    bob_priv   = 37

    await load_params(dut, k=alice_priv, xg=GX, yg=GY)
    done = await start_and_wait(dut)
    assert done, "Timeout computing Alice public key"
    assert error_flag(dut) == 0, "Error computing Alice public key"
    alice_pub_x, alice_pub_y = await read_xy(dut)
    dut._log.info(f"Alice public key: ({alice_pub_x:#04x}, {alice_pub_y:#04x})")

    await load_params(dut, k=bob_priv, xg=GX, yg=GY)
    done = await start_and_wait(dut)
    assert done, "Timeout computing Bob public key"
    assert error_flag(dut) == 0, "Error computing Bob public key"
    bob_pub_x, bob_pub_y = await read_xy(dut)
    dut._log.info(f"Bob public key:   ({bob_pub_x:#04x}, {bob_pub_y:#04x})")

    await load_params(dut, k=alice_priv, xg=bob_pub_x, yg=bob_pub_y)
    done = await start_and_wait(dut)
    assert done, "Timeout computing Alice shared secret"
    assert error_flag(dut) == 0, "Error computing Alice shared secret"
    alice_secret_x, alice_secret_y = await read_xy(dut)
    dut._log.info(f"Alice secret:     ({alice_secret_x:#04x}, {alice_secret_y:#04x})")

    await load_params(dut, k=bob_priv, xg=alice_pub_x, yg=alice_pub_y)
    done = await start_and_wait(dut)
    assert done, "Timeout computing Bob shared secret"
    assert error_flag(dut) == 0, "Error computing Bob shared secret"
    bob_secret_x, bob_secret_y = await read_xy(dut)
    dut._log.info(f"Bob secret:       ({bob_secret_x:#04x}, {bob_secret_y:#04x})")

    assert alice_secret_x == bob_secret_x, \
        f"DH X mismatch: Alice={alice_secret_x:#04x}, Bob={bob_secret_x:#04x}"
    assert alice_secret_y == bob_secret_y, \
        f"DH Y mismatch: Alice={alice_secret_y:#04x}, Bob={bob_secret_y:#04x}"

    gm_alice_x, gm_alice_y = scalar_mult(alice_priv, bob_pub_x, bob_pub_y)
    assert alice_secret_x == gm_alice_x and alice_secret_y == gm_alice_y, \
        "DH result does not match golden model"

    dut._log.info("PASS: DH shared secrets match")

# =========================================================================
# TEST 4 — public key as base point
# =========================================================================
@cocotb.test()
async def test_peer_public_key_as_base(dut):
    """scalar_mult(k, peer_pub) == expected shared secret."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset_dut(dut)

    my_priv   = 15
    peer_priv = 23

    peer_pub_x, peer_pub_y = scalar_mult(peer_priv, GX, GY)
    dut._log.info(f"Peer public key: ({peer_pub_x:#04x}, {peer_pub_y:#04x})")

    await load_params(dut, k=my_priv, xg=peer_pub_x, yg=peer_pub_y)
    done = await start_and_wait(dut)
    assert done, "Timeout computing shared secret with peer public key"
    assert error_flag(dut) == 0, "Error when using peer public key as base"

    hw_x, hw_y = await read_xy(dut)
    exp_x, exp_y = scalar_mult(my_priv, peer_pub_x, peer_pub_y)

    assert hw_x == exp_x, f"X mismatch: HW={hw_x:#04x} expected={exp_x:#04x}"
    assert hw_y == exp_y, f"Y mismatch: HW={hw_y:#04x} expected={exp_y:#04x}"
    dut._log.info(f"PASS: shared secret ({hw_x:#04x}, {hw_y:#04x}) matches golden model")
