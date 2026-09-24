#pragma once
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* tiny test framework: CHECK*, then TEST_MAIN(fn, fn, ...) */
static int test_failures;

#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); test_failures++; } } while (0)
#define CHECK_NEAR(a, b, tol) do { double _a = (a), _b = (b); if (!(fabs(_a - _b) <= (tol))) { \
    printf("  FAIL %s:%d: %s = %g, expected %g +/- %g\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); test_failures++; } } while (0)

typedef void (*test_fn)(void);
typedef struct { const char *name; test_fn fn; } test_case;
#define T(fn) {#fn, fn}

static inline int run_tests(const test_case *t, int n)
{
    for (int i = 0; i < n; i++) {
        int before = test_failures;
        t[i].fn();
        printf("%s %s\n", test_failures == before ? "PASS" : "FAIL", t[i].name);
    }
    return test_failures ? 1 : 0;
}
