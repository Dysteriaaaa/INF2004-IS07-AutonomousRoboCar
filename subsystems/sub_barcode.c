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
#define ELEMENTS        (9U)
#define WINDOW          (10U)

/* Reject anything implausible: a real element at working speed is tens
 * of milliseconds. These bounds throw out interrupt noise and long
 * stretches of blank track. */
#define ELEM_MIN_US     (800UL)
#define ELEM_MAX_US     (200000UL)

typedef struct {
    char     symbol;
    uint16_t pattern;   /* bit 8 is the first element, 1 means wide */
} code39_t;

/*
 *  TODO Buddy 3: this table carries only the four characters the brief
 *  asks for, plus the '*' guard that delimits a Code 39 message. Add the
 *  rest of the alphabet if your track uses it. Patterns are the standard
 *  Code 39 encodings, most significant bit first.
 */
static const code39_t table[] = {
    { '*', 0x094U },    /* guard, 0 1001 0100 */
    { 'A', 0x0A1U },
    { 'B', 0x0A1U },    /* TODO: placeholder, replace with the real pattern */
    { 'C', 0x0C1U },    /* TODO: placeholder */
    { 'D', 0x091U }     /* TODO: placeholder */
};

#define TABLE_N     (sizeof(table) / sizeof(table[0]))

static uint32_t widths[WINDOW];
static uint32_t n_widths;
static bool     armed;
static char     last_symbol;
static sub_barcode_cb_t user_cb;
static void    *user_ctx;

/* ------------------------------------------------------------------ *
 *  Map a decoded character to a navigation command. This mapping comes
 *  straight from the table in the project brief.
 * ------------------------------------------------------------------ */

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

static bool classify(uint16_t *pattern_out)
{
    uint32_t min_w = 0xFFFFFFFFUL;
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
        pattern <<= 1;
        if (widths[i] > mid) {
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

    if (n_widths >= WINDOW) {
        shift_window();
    }
    widths[n_widths] = w;
    n_widths++;

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

    n_widths    = 0U;
    last_symbol = symbol;

    out.id = RC_EVT_BARCODE_DECODED;
    out.u.barcode.symbol   = symbol;
    out.u.barcode.command  = symbol_to_cmd(symbol);
    out.u.barcode.reversed = reversed;
    (void)rc_event_publish(&out);

    if (user_cb != NULL) {
        user_cb(symbol, symbol_to_cmd(symbol), user_ctx);
    }
}

/* ------------------------------------------------------------------ */

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
    last_symbol = '\0';

    if (rc_event_subscribe(RC_EVT_BARCODE_EDGE, RC_LANE_FAST,
                           on_edge, NULL) < 0) {
        return RC_ERR_NOSPACE;
    }
    return RC_OK;
}

rc_result_t sub_barcode_arm(bool on)
{
    armed    = on;
    n_widths = 0U;
    return drv_ir_barcode_enable(on);
}

rc_result_t sub_barcode_on_decode(sub_barcode_cb_t cb, void *ctx)
{
    user_cb  = cb;
    user_ctx = ctx;
    return RC_OK;
}

char sub_barcode_last(void)
{
    return last_symbol;
}
