/*
 *  sub_barcode.c
 */
#include "rc_prelude.h"
#include "sub_barcode.h"
#include "drv_ir.h"
#include "rc_config.h"
#include "rc_event.h"

/*
 *  Code 39 encodes one character as nine elements, five bars and four
 *  spaces, of which exactly three are wide. Characters are separated by
 *  a narrow inter-character space, giving ten elements per cycle.
 *
 *  We collect ten widths, classify each as narrow or wide against the
 *  midpoint of the min and max in the window, pack the result into a
 *  nine bit pattern and look it up.
 */
/* How many bar/space widths make up one character (see the block comment
 * above). Used as the size of the sliding "pattern" we try to classify. */
#define ELEMENTS        (9U)
/* How many widths we keep buffered at once. It's one more than ELEMENTS
 * so that after a failed decode we can slide the window forward by one
 * element (shift_window()) and retry, rather than throwing everything
 * away and waiting for a fresh set of 9. */
#define WINDOW          (10U)

/* Reject anything implausible: a real element at working speed is tens
 * of milliseconds. These bounds throw out interrupt noise and long
 * stretches of blank track. "UL" suffix = "unsigned long" (a 32-bit
 * unsigned constant), matching the uint32_t widths these get compared
 * against. */
#define ELEM_MIN_US     (800UL)
#define ELEM_MAX_US     (200000UL)

/* A `struct` bundles several related pieces of data together under one
 * name. Here, one barcode table entry = one printable character plus the
 * 9-bit wide/narrow pattern that represents it. */
typedef struct {
    char     symbol;
    /* uint16_t is a 16-bit unsigned integer — plenty of room for a 9-bit
     * pattern (values 0..511). Bit 8 (counting from 0) holds whether the
     * FIRST of the 9 elements was wide; bit 0 holds the LAST. 1 = wide,
     * 0 = narrow. See classify() below for exactly how this is built. */
    uint16_t pattern;   /* bit 8 is the first element, 1 means wide */
} code39_t;

/*
 *  Standard Code 39 patterns, most significant bit first, for the '*'
 *  guard that delimits every message, the four command letters the brief
 *  uses (A..D), and 'Z' because the printed sample sheet in this folder
 *  carries one. Each 9-bit value is the elements in scan order
 *  bar,space,bar,space,bar,space,bar,space,bar with 1 = wide, so exactly
 *  three bits are set in every entry.
 *
 *  To add a character, take its bars and spaces from the Code 39 table
 *  (5 bars, 4 spaces, wide = 1) and interleave them: b1 s1 b2 s2 b3 s3
 *  b4 s4 b5. For 'A' the bars are 10001 and the spaces 0010, giving
 *  1 0 0 0 0 1 0 0 1 = 0x109.
 *
 *  TODO Buddy 3: add more letters only if your track uses them.
 */
static const code39_t table[] = {
    { '*', 0x094U },    /* bars 00110, spaces 1000 -> 0 1001 0100 */
    { 'A', 0x109U },    /* bars 10001, spaces 0010 -> 1 0000 1001 */
    { 'B', 0x049U },    /* bars 01001, spaces 0010 -> 0 0100 1001 */
    { 'C', 0x148U },    /* bars 11000, spaces 0010 -> 1 0100 1000 */
    { 'D', 0x019U },    /* bars 00101, spaces 0010 -> 0 0001 1001 */
    { 'Z', 0x0D0U }     /* bars 01100, spaces 1000 -> 0 1101 0000 */
};

/* Number of entries in `table`, computed at compile time rather than
 * hard-coded: sizeof(table) is the table's total size in bytes, and
 * dividing by the size of a single entry gives the element count. This
 * way, adding rows to `table` above automatically keeps TABLE_N correct
 * — nobody has to remember to update a count by hand. */
#define TABLE_N     (sizeof(table) / sizeof(table[0]))

/* sliding buffer of the most recent bar/space widths, in microseconds */
static uint32_t widths[WINDOW];
/* how many entries in `widths` are currently valid */
static uint32_t n_widths;
/* decoding enabled? (see sub_barcode_arm) */
static bool     armed;
/* most recently decoded character, for telemetry/logging */
static char     last_symbol;
/* a "function pointer" — the caller-supplied function to invoke on a successful
 * decode */
static sub_barcode_cb_t user_cb;
/* opaque context pointer passed back to user_cb untouched, so the caller can
 * tell which car/instance called it */
static void    *user_ctx;

/* ------------------------------------------------------------------ *
 *  Map a decoded character to a navigation command. This mapping comes
 *  straight from the table in the project brief.
 * ------------------------------------------------------------------ */

/* A `switch` is a multi-way if/else keyed on one value — here, which
 * character was decoded — that's usually clearer to read than a chain of
 * `if (c == 'A') ... else if (c == 'B') ...`. `default:` catches anything
 * not explicitly listed above it. */
static rc_nav_cmd_t symbol_to_cmd(char c)
{
    switch (c) {
    case 'A':
        return RC_CMD_TURN_LEFT;
    case 'B':
        return RC_CMD_TURN_RIGHT;
    case 'C':
        return RC_CMD_GO_STRAIGHT;
    case 'D':
        return RC_CMD_U_TURN;
    default:
        return RC_CMD_NONE;
    }
}

/* ------------------------------------------------------------------ *
 *  Try to decode the oldest nine elements in the window.
 * ------------------------------------------------------------------ */

/*
 * Looks at the oldest 9 widths currently buffered and tries to turn them
 * into a 9-bit wide/narrow pattern. Step by step, this is doing:
 *
 *   1. Find the shortest and longest element seen in this window
 *      (`min_w`/`max_w`). We don't know the car's speed in advance, so we
 *      can't compare against a fixed millisecond threshold — instead we
 *      figure out "short" and "long" *relative to each other*, which is
 *      the "ratio-based decoding" the barcode standard depends on.
 *   2. Sanity check: real Code 39 elements are wide-to-narrow by roughly
 *      3:1 (at least 2:1). If the widest element here isn't at least
 *      double the narrowest, this window isn't sitting on a real
 *      character (e.g. we're mid-way through one, or looking at noise) —
 *      bail out early.
 *   3. Pick the midpoint between shortest and longest as a threshold, and
 *      classify each of the 9 elements as wide (above the midpoint) or
 *      narrow (at/below it).
 *   4. Pack those 9 yes/no answers into a single 9-bit number (`pattern`)
 *      so the whole character can be compared and looked up as one value
 *      instead of nine. This is called "bit-packing": each bit position
 *      in `pattern` records the wide/narrow verdict for one element.
 *   5. Final sanity check: exactly 3 of the 9 elements must be wide in
 *      any legal Code 39 character. If the count doesn't match, the
 *      classification was wrong (bad threshold, noisy read, etc.) and we
 *      reject it rather than return a bogus character.
 *
 * Returns false (and leaves *pattern_out untouched) if this window
 * doesn't look like a valid character; the caller then slides the window
 * by one element and tries again (see on_edge() below).
 */
static bool classify(uint16_t *pattern_out)
{
    /* start at the largest possible value so the first real width always beats
     * it */
    uint32_t min_w = 0xFFFFFFFFUL;
    /* start at zero so the first real width always beats it */
    uint32_t max_w = 0U;
    uint32_t mid;
    uint16_t pattern = 0U;
    uint32_t wide_count = 0U;
    uint32_t i;

    for (i = 0U; i < ELEMENTS; i++) {
        if (widths[i] < min_w) {
            min_w = widths[i];
        }
        if (widths[i] > max_w) {
            max_w = widths[i];
        }
    }

    /* A valid Code 39 character has a wide:narrow ratio of at least 2.
     * If the spread is smaller than that we are not looking at one. */
    if (max_w < (min_w * 2U)) {
        return false;
    }

    mid = (min_w + max_w) / 2U;

    for (i = 0U; i < ELEMENTS; i++) {
        /* `<<=` is the left-shift-and-assign operator: `pattern <<= 1`
         * means "shift every bit in pattern one place to the left,
         * making room for a new bit at the bottom" — equivalent to
         * `pattern = pattern << 1`. Doing this once per element, in
         * order, is how the 9 separate wide/narrow verdicts end up
         * packed into the 9 low bits of one number, most-significant
         * (first element) bit first. */
        pattern <<= 1;
        if (widths[i] > mid) {
            /* `|=` is bitwise-OR-and-assign: `pattern |= 1U` sets just
             * the lowest bit to 1 (marking "this element was wide")
             * without disturbing any of the bits already shifted in
             * above. */
            pattern |= 1U;
            wide_count++;
        }
    }

    /* Exactly three of the nine elements are wide in every Code 39
     * character. This one check throws out most misreads. */
    if (wide_count != 3U) {
        return false;
    }

    *pattern_out = pattern;
    return true;
}

/* Searches `table` for a matching pattern. `out` and `reversed` are
 * "output parameters": because C functions can only directly return one
 * value, passing in pointers lets this function hand back two results
 * (the decoded character, and whether it needed reversing) by writing
 * through those pointers into the caller's own variables. */
static bool lookup(uint16_t pattern, char *out, bool *reversed)
{
    uint32_t i;
    uint16_t rev = 0U;
    uint32_t b;

    for (i = 0U; i < TABLE_N; i++) {
        if (table[i].pattern == pattern) {
            *out      = table[i].symbol;
            *reversed = false;
            return true;
        }
    }

    /*
     *  The car can meet a barcode from either end, so also try the
     *  pattern reversed. Without this the decoder silently fails half
     *  the time and it looks like a sensor problem.
     */
    /* Bit-reverse `pattern` into `rev` (flip the order of its 9 bits),
     * so a character read back-to-front still matches the table. Reading
     * this loop: `1U << b` is "the number 1 shifted left by b places",
     * i.e. a mask with only bit `b` set (1, 2, 4, 8, ...). `pattern & mask`
     * (bitwise AND) tests whether that specific bit is set in `pattern`.
     * We walk b from 0 (the original pattern's last element) upward,
     * shifting that bit into `rev` from the bottom each time — which
     * builds `rev` with the elements in the opposite order to `pattern`. */
    for (b = 0U; b < ELEMENTS; b++) {
        rev <<= 1;
        if ((pattern & (1U << b)) != 0U) {
            rev |= 1U;
        }
    }

    for (i = 0U; i < TABLE_N; i++) {
        if (table[i].pattern == rev) {
            *out      = table[i].symbol;
            *reversed = true;
            return true;
        }
    }
    return false;
}

/* Drops the oldest width and shifts everything else down by one slot, so
 * the window "slides" forward by one element instead of being cleared
 * entirely. Used after a failed classify()/lookup() so we retry starting
 * one element later, in case we were simply mis-aligned with the true
 * start of the character rather than looking at garbage. */
static void shift_window(void)
{
    uint32_t i;

    for (i = 1U; i < n_widths; i++) {
        widths[i - 1U] = widths[i];
    }
    n_widths--;
}

/* ------------------------------------------------------------------ *
 *  Edge callback. Dispatcher task context, fast lane.
 * ------------------------------------------------------------------ */

/* Called automatically (event bus callback, see TEAM_GUIDE.md §1.2)
 * every time drv_ir.c's barcode ISR records one more bar/space edge and
 * publishes its width. This is the "collect widths, then try to decode"
 * loop that drives the whole barcode module. */
static void on_edge(const rc_event_t *evt, void *ctx)
{
    uint32_t   w = evt->u.bar_edge.width_us;
    uint16_t   pattern;
    char       symbol;
    bool       reversed;
    rc_event_t out;

    (void)ctx;

    if (!armed) {
        return;
    }

    if ((w < ELEM_MIN_US) || (w > ELEM_MAX_US)) {
        n_widths = 0U;              /* implausible, restart the window */
        return;
    }

    /* Append the new width to the buffer, making room by sliding out the
     * oldest one first if we're already full. */
    if (n_widths >= WINDOW) {
        shift_window();
    }
    widths[n_widths] = w;
    n_widths++;

    /* Not enough elements buffered yet to attempt a decode — wait for
     * more edges to arrive. */
    if (n_widths < ELEMENTS) {
        return;
    }

    if (!classify(&pattern)) {
        shift_window();             /* slide by one and try again */
        return;
    }

    if (!lookup(pattern, &symbol, &reversed)) {
        shift_window();
        return;
    }

    /* Successful decode: reset the window so we start collecting the
     * *next* character fresh, remember the symbol for sub_barcode_last(),
     * and tell the rest of the car about it two ways — publish an event
     * for anyone subscribed (e.g. sub_nav), and also call the specific
     * callback function (if any) someone registered directly via
     * sub_barcode_on_decode(). */
    n_widths    = 0U;
    last_symbol = symbol;

    out.id = RC_EVT_BARCODE_DECODED;
    out.u.barcode.symbol   = symbol;
    out.u.barcode.command  = symbol_to_cmd(symbol);
    out.u.barcode.reversed = reversed;
    (void)rc_event_publish(&out);

    /* user_cb is a function pointer (see its declaration above) — it
     * might be NULL if nobody registered a callback via
     * sub_barcode_on_decode(), so we must check before calling through
     * it or the car would crash trying to "call" address zero. */
    if (user_cb != NULL) {
        user_cb(symbol, symbol_to_cmd(symbol), user_ctx);
    }
}

/* ------------------------------------------------------------------ */

/* Call once at boot. Clears the width buffer, starts disarmed (see
 * sub_barcode_arm below), and subscribes on_edge() to fire for every
 * RC_EVT_BARCODE_EDGE event published by drv_ir.c's barcode ISR. */
rc_result_t sub_barcode_init(void)
{
    {
        uint32_t i;
        for (i = 0U; i < WINDOW; i++) {
            widths[i] = 0U;
        }
    }
    n_widths    = 0U;
    armed       = false;
    /* '\0' is the "nul" character: C's usual way to say "no value here" */
    last_symbol = '\0';

    if (rc_event_subscribe(RC_EVT_BARCODE_EDGE, RC_LANE_FAST,
                           on_edge, NULL) < 0) {
        return RC_ERR_NOSPACE;
    }
    return RC_OK;
}

/* Turns decoding on/off, and forwards that to the driver so the
 * underlying interrupt (see drv_ir.c) is masked while off too — see
 * drv_ir_barcode_enable's comment for why: leaving it always-on would let
 * ordinary track noise get decoded as if it were a real barcode. */
rc_result_t sub_barcode_arm(bool on)
{
    armed    = on;
    n_widths = 0U;
    return drv_ir_barcode_enable(on);
}

/* Registers a function to call every time a character is successfully
 * decoded. `cb` is a function pointer (see sub_barcode_cb_t's typedef in
 * sub_barcode.h) and `ctx` is an arbitrary pointer stored alongside it
 * and handed back unchanged when `cb` is called — a common C idiom for
 * giving a plain function some "memory" of which caller registered it,
 * without needing objects/classes. */
rc_result_t sub_barcode_on_decode(sub_barcode_cb_t cb, void *ctx)
{
    user_cb  = cb;
    user_ctx = ctx;
    return RC_OK;
}

/* Returns the most recently decoded character, mainly for telemetry
 * (Buddy 1's module) to report on what the car has read so far. */
char sub_barcode_last(void)
{
    return last_symbol;
}
