// Reproduces the unbounded loop in px4_ros2::wrapPi.
//
// No ROS 2 and no Eigen needed. Build and run:
//
//     g++ -O2 -std=c++17 -o wrap_pi_demo wrap_pi_demo.cpp && ./wrap_pi_demo
//
// By default nothing hangs. The shipped function is run through a version that
// counts iterations and stops as soon as an iteration stops changing the
// value, so it reports the stall instead of locking up your terminal.
//
// To see the real hang, run:
//
//     ./wrap_pi_demo --hang
//
// That calls the shipped function unchanged and never returns. Use Ctrl+C.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

// ---------------------------------------------------------------------------
// Copied verbatim from px4_ros2_cpp/include/px4_ros2/utils/geometry.hpp
// ---------------------------------------------------------------------------
template <typename Type>
Type wrapPiShipped(Type angle)
{
  while (angle >= M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// ---------------------------------------------------------------------------
// The same function with a counter added. The arithmetic is identical; the
// only change is that it gives up when an iteration leaves the value alone.
// ---------------------------------------------------------------------------
enum class Outcome { Returned, Stalled, OverBudget };

template <typename Type>
Outcome wrapPiInstrumented(Type angle, long budget, Type* out, long* iterations)
{
  long n = 0;
  while (angle >= M_PI) {
    const Type next = angle - static_cast<Type>(2.0 * M_PI);
    if (next == angle) {
      *iterations = n;
      return Outcome::Stalled;
    }
    angle = next;
    if (++n > budget) {
      *iterations = n;
      return Outcome::OverBudget;
    }
  }
  while (angle < -M_PI) {
    const Type next = angle + static_cast<Type>(2.0 * M_PI);
    if (next == angle) {
      *iterations = n;
      return Outcome::Stalled;
    }
    angle = next;
    if (++n > budget) {
      *iterations = n;
      return Outcome::OverBudget;
    }
  }
  *out = angle;
  *iterations = n;
  return Outcome::Returned;
}

// ---------------------------------------------------------------------------
// The proposed replacement.
// ---------------------------------------------------------------------------
template <typename Type>
Type wrapPiFixed(Type angle)
{
  angle = std::remainder(angle, static_cast<Type>(2.0 * M_PI));
  if (angle >= static_cast<Type>(M_PI)) {
    angle -= static_cast<Type>(2.0 * M_PI);
  }
  return angle;
}

// ---------------------------------------------------------------------------

static bool inDocumentedRange(float v)
{
  return std::isnan(v) || (v >= -static_cast<float>(M_PI) && v < static_cast<float>(M_PI));
}

static void row(const char* label, float input)
{
  float shipped = 0.0F;
  long iterations = 0;
  const Outcome outcome = wrapPiInstrumented<float>(input, 30000000L, &shipped, &iterations);

  char shipped_text[64];
  if (outcome == Outcome::Stalled) {
    std::snprintf(shipped_text, sizeof(shipped_text), "NEVER RETURNS");
  } else if (outcome == Outcome::OverBudget) {
    std::snprintf(shipped_text, sizeof(shipped_text), "> %ld iterations", iterations);
  } else {
    std::snprintf(shipped_text, sizeof(shipped_text), "%.5f  (%ld iters)", shipped, iterations);
  }

  const float fixed = wrapPiFixed<float>(input);
  char fixed_text[64];
  std::snprintf(fixed_text, sizeof(fixed_text), "%.5f%s", fixed,
                inDocumentedRange(fixed) ? "" : "  OUT OF RANGE");

  std::printf("  %-10s %-28s %s\n", label, shipped_text, fixed_text);
}

int main(int argc, char** argv)
{
  if (argc > 1 && std::strcmp(argv[1], "--hang") == 0) {
    std::printf("Calling the shipped wrapPi with infinity. This will not return.\n");
    std::printf("Press Ctrl+C.\n");
    std::fflush(stdout);
    const float result = wrapPiShipped<float>(std::numeric_limits<float>::infinity());
    std::printf("unreachable, got %f\n", result);
    return 0;
  }

  std::printf("px4_ros2::wrapPi<float>, shipped against proposed\n\n");
  std::printf("  %-10s %-28s %s\n", "input", "shipped", "proposed");
  std::printf("  %-10s %-28s %s\n", "-----", "-------", "--------");

  row("1e3", 1.0e3F);
  row("1e4", 1.0e4F);
  row("1e7", 1.0e7F);
  row("134217728", 134217728.0F);
  row("134217744", 134217744.0F);
  row("1e9", 1.0e9F);
  row("FLT_MAX", std::numeric_limits<float>::max());
  row("+inf", std::numeric_limits<float>::infinity());
  row("-inf", -std::numeric_limits<float>::infinity());
  row("NaN", std::numeric_limits<float>::quiet_NaN());

  std::printf("\nThe repo's existing unit tests, tolerance 1e-3\n\n");
  struct Check {
    const char* name;
    double got;
    double want;
  };
  const Check checks[] = {
      {"wrapPi(M_PI_2)", wrapPiFixed<double>(M_PI_2), M_PI_2},
      {"wrapPi(-M_PI_2)", wrapPiFixed<double>(-M_PI_2), -M_PI_2},
      {"wrapPi(4*M_PI)", wrapPiFixed<double>(4 * M_PI), 0.0},
      {"wrapPi(-11.f*M_PI/4.f)", wrapPiFixed<float>(-11.0F * static_cast<float>(M_PI) / 4.0F),
       -3.0 * M_PI / 4.0},
      {"wrapPi(11.f*M_PI/4.f)", wrapPiFixed<float>(11.0F * static_cast<float>(M_PI) / 4.0F),
       3.0 * M_PI / 4.0},
      {"fabs(wrapPi(-11.0*M_PI))", std::fabs(wrapPiFixed<double>(-11.0 * M_PI)), M_PI},
  };

  int failures = 0;
  for (const Check& check : checks) {
    const bool ok = std::fabs(check.got - check.want) < 1e-3;
    if (!ok) {
      ++failures;
    }
    std::printf("  %-4s %-26s want %9.5f  got %9.5f\n", ok ? "PASS" : "FAIL", check.name,
                check.want, check.got);
  }

  std::printf("\n%s\n", failures == 0 ? "All existing assertions pass." : "Some assertions fail.");
  return failures == 0 ? 0 : 1;
}
