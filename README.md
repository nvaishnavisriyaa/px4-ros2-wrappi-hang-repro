# px4_ros2::wrapPi never returns for a large or infinite angle

`wrapPi` in [Auterion/px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib)
reduces an angle by subtracting `2*pi` in a loop that has no bound. Two things
go wrong:

1. For any single precision `|angle|` at or above **134217744**, and for
   **either infinity**, the loop never finishes. The subtraction rounds back to
   the same value, so no iteration makes progress.
2. Long before that, the answer is already wrong. The loop rounds once per
   iteration, so the error grows with the iteration count. At `1e5` rad the
   result is off by about **100 degrees**.

The function sits in `px4_ros2_cpp/include/px4_ros2/utils/geometry.hpp` and is
reached straight from caller supplied values, so neither case needs anything
unusual inside the library to trigger.

This repository holds the measurements and a patch. Nothing here needs ROS 2,
Eigen, or a flight controller.

Reported upstream as [Auterion/px4-ros2-interface-lib#228](https://github.com/Auterion/px4-ros2-interface-lib/issues/228).

## The code

[`geometry.hpp` lines 69 to 77](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_cpp/include/px4_ros2/utils/geometry.hpp#L69-L77):

```cpp
template <typename Type>
Type wrapPi(Type angle)
{
  while (angle >= M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}
```

## Why it never finishes

Floats are spaced further apart as they get larger. Once that spacing is more
than twice `2*pi`, subtracting `2*pi` rounds straight back to the value you
started from, and the loop condition never changes.

| magnitude | spacing between neighbouring floats | subtraction still moves |
| --- | --- | --- |
| 2^24 | 2 | yes |
| 2^26 | 8 | yes |
| 2^27 | 16 | yes |
| 2^28 | 32 | **no** |

The exact boundary is `134217744`. One float below it, `134217728`, still
returns, but only after 18,092,333 iterations.

## What each input does today

| input to `wrapPi<float>` | result |
| --- | --- |
| `1e3` | returns after 159 iterations |
| `1e4` | returns after 1,592 iterations |
| `1e7` | returns after 1,583,682 iterations |
| `134217728` | returns after 18,092,333 iterations |
| `134217744` | **never returns** |
| `1e9` | **never returns** |
| `FLT_MAX`, for example uninitialised memory | **never returns** |
| `+inf`, for example a divide by zero | **never returns** |
| `-inf` | **never returns** |
| `NaN` | returns `NaN`, no hang |

## The accuracy problem

Each iteration rounds, and the errors add up. Scored against a 50 digit
reference:

| input | exact answer | shipped loop | off by | with the patch | off by |
| --- | --- | --- | --- | --- | --- |
| `1e2` | -0.53096 | -0.53097 | 0.000 deg | -0.53097 | 0.000 deg |
| `1e3` | 0.97354 | 0.97261 | 0.053 deg | 0.97351 | 0.002 deg |
| `1e4` | -2.83101 | -2.85748 | 1.517 deg | -2.83129 | 0.016 deg |
| `1e5` | 3.10584 | -1.40434 | **101.586 deg** | 3.10305 | 0.159 deg |
| `1e6` | -0.35756 | 2.72261 | **176.481 deg** | -0.38539 | 1.594 deg |

A heading that is wrong by 100 degrees is a different kind of problem from a
slow one, and it starts at inputs far smaller than the hang threshold.

## How a caller reaches it

`angle` is not always an internal value that the library already trusts:

- [`geodesic.cpp:155`](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_cpp/src/utils/geodesic.cpp#L155)
  `globalPositionFromHeadingAndDist(position, heading, dist)` runs
  `heading = wrapPi(heading)` on the `float` argument exactly as passed in.
- [`frame_conversion.hpp:84`](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_cpp/include/px4_ros2/utils/frame_conversion.hpp#L84)
  `yawNedToEnu` and `yawEnuToNed` wrap a caller supplied yaw.
- `px4_ros2_py` exports the function to Python as `wrap_pi`
  ([`geometry.cpp:19`](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/px4_ros2_py/src/px4_ros2/utils/geometry.cpp#L19)).
  `wrap_pi(float("inf"))` spins inside pybind11 while holding the GIL, so the
  interpreter stops responding and Ctrl+C cannot be delivered. The process has
  to be killed.

Inside a flight mode's update loop, a hang is worse than a wrong number,
because the mode stops publishing setpoints at all.

## Upstream PX4 does not have this problem

PX4 itself wraps angles with a closed form in
[`PX4-Matrix/matrix/helper_functions.hpp`](https://github.com/PX4/PX4-Matrix/blob/master/matrix/helper_functions.hpp),
which has no loop and runs in constant time:

```cpp
template<typename Floating>
Floating wrap_floating(Floating x, Floating low, Floating high) {
    if (low <= x && x < high) {
        return x;
    }
    const auto range = high - low;
    const auto inv_range = Floating(1) / range;
    const auto num_wraps = floor((x - low) * inv_range);
    return x - range * num_wraps;
}
```

So this is a divergence from the implementation the flight controller on the
other end of the link already uses.

## The patch

`fix.patch` replaces the loop with `std::remainder`, which is exact, runs in
constant time, and already returns `NaN` for a non finite input, so no extra
branch is needed:

```cpp
angle = std::remainder(angle, static_cast<Type>(2.0 * M_PI));

// std::remainder yields [-pi, pi]; the documented range is [-pi, pi).
if (angle >= static_cast<Type>(M_PI)) {
  angle -= static_cast<Type>(2.0 * M_PI);
}

return angle;
```

13 insertions and 5 deletions in one file, most of it comment.

`std::remainder` was chosen over PX4's closed form because the closed form
subtracts two nearly equal large numbers, which cancels. For `1e9` it returns
`-64.0`, outside the documented `[-pi, pi)`. `std::remainder` stays inside the
range for every finite input.

### Does it change behaviour

| check | result |
| --- | --- |
| 200,000 random inputs already in `[-pi, pi)` | bit for bit identical to today |
| 200,000 random inputs in `[-1000, 1000]` rad | all inside `[-pi, pi)`, worst gap from today 9.11e-04 rad |
| the repo's six existing `wrapPi` assertions | all pass |
| `NaN` in | `NaN` out, same as today |
| `+inf` or `-inf` in | `NaN` out, instead of hanging |

The six assertions include `fabs(wrapPi(-11.0 * M_PI))`, which lands exactly on
the half open boundary. `std::remainder` picks the even multiple there and
returns `+pi`, and the guard above brings it to `-pi`, so the absolute value is
still `pi` and the assertion holds.

## Reproducing

Python only, no compiler. This is the full evidence above:

```bash
python verify.py
```

Saved output is in `RESULTS.txt`. The script rounds every step to IEEE-754
binary32 with `struct`, so the arithmetic matches `wrapPi<float>`. The one
place C and Python differ is non finite input, and that is handled explicitly.

To run the real C++ instead, no ROS 2 or Eigen needed:

```bash
g++ -O2 -std=c++17 -o wrap_pi_demo wrap_pi_demo.cpp && ./wrap_pi_demo
```

That copies `wrapPi` out of the header unchanged and compares it with the
patched version. It is safe by default: the shipped function is run through a
variant that counts iterations and stops as soon as one stops changing the
value, so it reports the stall rather than locking up the terminal. To watch
the real thing hang:

```bash
./wrap_pi_demo --hang
```

To apply the patch to a checkout:

```bash
git apply fix.patch
```

## A note on how this was measured

There was no C++ compiler on the machine used to produce `RESULTS.txt`, so the
numbers there come from the Python model of binary32 arithmetic rather than
from a compiled run. The arithmetic is not in doubt, since every step is
`(float)((double)angle - 6.283185307179586)` rounded to binary32, but
`wrap_pi_demo.cpp` is included so the same table can be confirmed natively.

## Files

| file | what it is |
| --- | --- |
| `verify.py` | all six measurements, no compiler needed |
| `RESULTS.txt` | saved output of `verify.py` |
| `wrap_pi_demo.cpp` | standalone C++ check, safe by default |
| `fix.patch` | the one file patch |

## Licence

BSD 3-Clause, matching px4-ros2-interface-lib.
