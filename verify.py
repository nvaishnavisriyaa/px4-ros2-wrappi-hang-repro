"""Evidence for the unbounded loop in px4_ros2::wrapPi, with no compiler needed.

Every floating point step below is rounded to IEEE-754 binary32 with struct,
so the arithmetic matches what C++ does for wrapPi<float>. The one place C and
Python disagree is non finite input, and that is handled explicitly.

Run:
    python verify.py
"""

import math
import random
import struct
import sys

PI = math.pi
TWO_PI = 2.0 * PI
SEED = 20261010


def f32(value):
    """Round a Python float to the nearest binary32, the way C++ does."""
    return struct.unpack("f", struct.pack("f", value))[0]


def next_f32(value):
    bits = struct.unpack("I", struct.pack("f", value))[0]
    return struct.unpack("f", struct.pack("I", bits + 1))[0]


def prev_f32(value):
    bits = struct.unpack("I", struct.pack("f", value))[0]
    return struct.unpack("f", struct.pack("I", bits - 1))[0]


F_PI = f32(PI)
F_TWO_PI = f32(TWO_PI)


def shipped(angle, budget=30_000_000):
    """px4_ros2::wrapPi<float> as it is today.

    Returns (value, iterations). A value of None means it never returns:
    iterations is -1 when an iteration stopped changing the value, and -2
    when the budget ran out.
    """
    count = 0
    while angle >= PI:
        nxt = f32(angle - TWO_PI)
        if nxt == angle:
            return None, -1
        angle = nxt
        count += 1
        if count > budget:
            return None, -2
    while angle < -PI:
        nxt = f32(angle + TWO_PI)
        if nxt == angle:
            return None, -1
        angle = nxt
        count += 1
        if count > budget:
            return None, -2
    return angle, count


def upstream(angle):
    """matrix::wrap_floating(angle, -pi, pi) from PX4-Matrix, in binary32.

    C floor() passes infinity and NaN straight through, and inf - inf is NaN.
    """
    low, high = f32(-PI), F_PI
    if low <= angle < high:
        return angle
    span = f32(high - low)
    inverse = f32(1.0 / span)
    scaled = f32(f32(angle - low) * inverse)
    if math.isnan(scaled) or math.isinf(scaled):
        return math.nan
    wraps = float(math.floor(scaled))
    return f32(angle - f32(span * wraps))


def proposed(angle):
    """The replacement: std::remainder, then keep the half open range."""
    if math.isnan(angle) or math.isinf(angle):
        return math.nan  # C: remainder of an infinity is NaN
    out = f32(math.remainder(angle, F_TWO_PI))
    if out >= F_PI:
        out = f32(out - F_TWO_PI)
    return out


def in_range(value):
    return math.isnan(value) or (-PI <= value < PI)


def heading(text):
    print()
    print(text)
    print("-" * len(text))


def part_1_threshold():
    heading("1. Where the loop stops making progress")
    print("The loop subtracts 2*pi. Once 2*pi falls below half the spacing")
    print("between neighbouring floats, the result rounds back to the same")
    print("value and the loop runs forever.")
    print()

    exponent = 20
    while exponent < 40:
        value = f32(2.0**exponent)
        moves = f32(value - TWO_PI) != value
        spacing = f32(2.0 ** (exponent - 23))
        print(
            f"  2^{exponent:<3} = {value:>22.0f}   spacing {spacing:>10g}"
            f"   progresses: {moves}"
        )
        if not moves:
            break
        exponent += 1

    value = f32(2.0**27)
    while f32(value - TWO_PI) != value:
        value = next_f32(value)
    print()
    print(
        f"  exact boundary: {value:.0f} never returns,"
        f" {prev_f32(value):.0f} still does"
    )
    return value


def part_2_costs():
    heading("2. What each input costs, wrapPi<float>")
    print(f"  {'input':<40}{'outcome':<34}")
    cases = [
        ("1e3", f32(1e3)),
        ("1e4", f32(1e4)),
        ("1e7", f32(1e7)),
        ("134217728 (2^27)", f32(134217728.0)),
        ("134217744", f32(134217744.0)),
        ("1e9", f32(1e9)),
        ("FLT_MAX, e.g. uninitialised memory", f32(3.4e38)),
        ("+inf, e.g. a divide by zero", math.inf),
        ("-inf", -math.inf),
        ("NaN", math.nan),
    ]
    for label, value in cases:
        if math.isnan(value):
            print(f"  {label:<40}{'returns NaN, no hang':<34}")
            continue
        out, count = shipped(value)
        if count == -1:
            text = "NEVER RETURNS"
        elif count == -2:
            text = "over 30,000,000 iterations"
        else:
            text = f"{out:.5f} after {count:,} iterations"
        print(f"  {label:<40}{text:<34}")


def part_3_equivalence():
    heading("3. Is the replacement behaviour preserving")
    random.seed(SEED)

    same_up = same_new = considered = 0
    for _ in range(200_000):
        value = f32(random.uniform(-PI, PI))
        if not (f32(-PI) <= value < F_PI):
            continue
        considered += 1
        out, _ = shipped(value)
        same_up += upstream(value) == out
        same_new += proposed(value) == out
    print(f"  {considered:,} random inputs already inside [-pi, pi)")
    print(f"    upstream closed form bit identical to the loop: {same_up:,}")
    print(f"    proposed remainder   bit identical to the loop: {same_new:,}")

    print()
    random.seed(SEED + 1)
    worst_up = worst_new = 0.0
    out_up = out_new = 0
    total = 0
    for _ in range(200_000):
        value = f32(random.uniform(-1000.0, 1000.0))
        out, _ = shipped(value)
        if out is None:
            continue
        total += 1
        a, b = upstream(value), proposed(value)
        out_up += not in_range(a)
        out_new += not in_range(b)
        worst_up = max(worst_up, min(abs(out - a), abs(abs(out - a) - TWO_PI)))
        worst_new = max(worst_new, min(abs(out - b), abs(abs(out - b) - TWO_PI)))
    print(f"  {total:,} random inputs in [-1000, 1000] rad")
    print(
        f"    upstream closed form: {out_up} outside [-pi, pi),"
        f" worst gap {worst_up:.2e} rad"
    )
    print(
        f"    proposed remainder  : {out_new} outside [-pi, pi),"
        f" worst gap {worst_new:.2e} rad"
    )


def show(value):
    if math.isnan(value):
        return "nan"
    return f"{value:.5f}" + ("" if in_range(value) else "  OUT OF RANGE")


def part_4_side_by_side():
    heading("4. The three implementations on the inputs that matter")
    print(
        f"  {'input':<12}{'shipped loop':<28}"
        f"{'upstream closed form':<24}{'proposed':<20}"
    )
    print("  " + "-" * 82)
    for label, value in [
        ("1e4", f32(1e4)),
        ("1e7", f32(1e7)),
        ("1e9", f32(1e9)),
        ("FLT_MAX", f32(3.4e38)),
        ("+inf", math.inf),
        ("NaN", math.nan),
    ]:
        out, count = shipped(value)
        if count == -1:
            left = "NEVER RETURNS"
        elif count == -2:
            left = "over 30,000,000 iters"
        else:
            left = f"{out:.5f} ({count:,} iters)"
        print(
            f"  {label:<12}{left:<28}"
            f"{show(upstream(value)):<24}{show(proposed(value)):<20}"
        )

    print()
    print("  For any |angle| past about 1e7 no implementation can give a")
    print("  meaningful answer, because the spacing between floats is already")
    print("  wider than 2*pi. The difference is that the shipped loop hangs,")
    print("  while the other two return at once.")


def proposed_double(angle):
    if math.isnan(angle) or math.isinf(angle):
        return math.nan
    out = math.remainder(angle, TWO_PI)
    if out >= PI:
        out -= TWO_PI
    return out


def part_5_existing_tests():
    heading("5. Does the replacement still pass the repo's own tests")
    print("  From px4_ros2_cpp/test/unit/utils/geometry.cpp,")
    print("  TEST(Geometry, wrapPi). Tolerance 1e-3.")
    print()

    checks = [
        ("wrapPi(M_PI_2)", proposed_double(PI / 2), PI / 2),
        ("wrapPi(-M_PI_2)", proposed_double(-PI / 2), -PI / 2),
        ("wrapPi(4*M_PI)", proposed_double(4 * PI), 0.0),
        ("wrapPi(-11.f*M_PI/4.f)", proposed(f32(-11.0 * PI / 4.0)), -3.0 * PI / 4.0),
        ("wrapPi(11.f*M_PI/4.f)", proposed(f32(11.0 * PI / 4.0)), 3.0 * PI / 4.0),
        ("fabs(wrapPi(-11.0*M_PI))", abs(proposed_double(-11.0 * PI)), PI),
    ]
    failures = 0
    for name, got, want in checks:
        ok = abs(got - want) < 1e-3
        failures += not ok
        print(
            f"  {'PASS' if ok else 'FAIL'}  {name:<26}"
            f" want {want:>9.5f}  got {got:>9.5f}"
        )
    print()
    if failures == 0:
        print("  All existing assertions pass.")
    else:
        print("  Some assertions fail.")
    return failures


def exact_wrap(value):
    """Exact value mod 2*pi into [-pi, pi), using 50 digit pi.

    This is the reference the three implementations are scored against. The
    float is converted exactly, so the only approximation is pi itself.
    """
    from decimal import Decimal, getcontext

    getcontext().prec = 60
    pi = Decimal("3.14159265358979323846264338327950288419716939937511")
    two_pi = 2 * pi
    remainder = Decimal(value) - (
        Decimal(value) / two_pi
    ).to_integral_value(rounding="ROUND_HALF_EVEN") * two_pi
    while remainder >= pi:
        remainder -= two_pi
    while remainder < -pi:
        remainder += two_pi
    return float(remainder)


def angular_error_degrees(reference, got):
    gap = abs(reference - got) % TWO_PI
    return math.degrees(min(gap, TWO_PI - gap))


def part_6_accuracy():
    heading("6. Accuracy, scored against an exact reference")
    print("  The loop rounds once per iteration, so its error grows with the")
    print("  number of iterations. This shows up long before it hangs.")
    print()
    print(
        f"  {'input':<8}{'exact':>11}{'shipped':>11}{'off by':>10}"
        f"{'proposed':>11}{'off by':>10}"
    )
    print("  " + "-" * 61)
    for label, value in [
        ("1e2", f32(1e2)),
        ("1e3", f32(1e3)),
        ("1e4", f32(1e4)),
        ("1e5", f32(1e5)),
        ("1e6", f32(1e6)),
    ]:
        reference = exact_wrap(value)
        loop_out, _ = shipped(value)
        new_out = proposed(value)
        if loop_out is None:
            loop_text, loop_err = f"{'hangs':>11}", f"{'-':>10}"
        else:
            loop_text = f"{loop_out:>11.5f}"
            loop_err = f"{angular_error_degrees(reference, loop_out):>9.3f}d"
        print(
            f"  {label:<8}{reference:>11.5f}{loop_text}{loop_err}"
            f"{new_out:>11.5f}"
            f"{angular_error_degrees(reference, new_out):>9.3f}d"
        )
    print()
    print("  At 1e5 rad the shipped loop is off by about 100 degrees, which is")
    print("  a wrong heading rather than a slow one.")


def main():
    print("px4_ros2::wrapPi, unbounded loop")
    print("=" * 64)
    print("Simulated in exact IEEE-754 binary32. No compiler required.")
    part_1_threshold()
    part_2_costs()
    part_3_equivalence()
    part_4_side_by_side()
    failures = part_5_existing_tests()
    part_6_accuracy()
    print()
    print("=" * 64)
    print("Done." if failures == 0 else "Done, with failing assertions.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
