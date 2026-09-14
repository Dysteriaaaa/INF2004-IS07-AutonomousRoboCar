/*
 *  sub_telemetry.c
 */
#include "rc_prelude.h"   /* project-wide basics: fixed-width int types (uint32_t etc.), bool, common macros */
#include <tm/tmonitor.h>  /* RTOS "target monitor" console I/O -- gives us tm_printf(), the debug-console print function */

#include "sub_telemetry.h"
#include "rc_fmt.h"       /* rc_snprintf()/rc_strlen(): this project's own tiny stand-ins for the C standard library's snprintf/strlen (embedded builds often avoid the full libc) */
#include "sub_terrain.h"  /* Buddy 4's module -- gives us sub_terrain_motion_class()/sub_terrain_max_peak_mm() to report */
#include "sub_barcode.h"  /* Buddy 3's module -- gives us sub_barcode_last() to report the last decoded barcode char */
#include "drv_motor.h"    /* Buddy 2's driver -- gives us drv_motor_get() to report current commanded motor duty */
#include "drv_encoder.h"  /* Buddy 2's driver (included for future use / shared types) */
#include "rc_config.h"    /* board-wide tunable constants, e.g. RC_PERIOD_TELEM_MS, RC_PRI_TELEMETRY, RC_STACK_SZ */
#include "rc_event.h"     /* the event bus: rc_event_subscribe(), rc_event_dropped(), RC_LANE_* lane IDs */
#include "rc_time.h"      /* rc_time_ms(): milliseconds-since-boot clock, used for timestamps */

/* #define creates a plain text-substitution constant -- the preprocessor
 * replaces every later use of TOPIC_MAX with the number (48) before the
 * compiler even runs. Using named constants like this instead of writing
 * bare numbers ("magic numbers") makes it obvious what the number means
 * and gives one place to change it.
 * TOPIC_MAX: size (in bytes) of the local buffer that holds one MQTT-style
 * topic string, e.g. "car/01/state". Used in send() below when building
 * the topic with rc_snprintf(); must be big enough for TOPIC_BASE + the
 * longest leaf name ("obstacle", "heartbeat", etc.) plus a slash and the
 * terminating '\0' (C strings are terminated by a zero byte, not a length
 * field, so every buffer holding text needs room for that extra byte). */
#define TOPIC_MAX       (48)
/* PAYLOAD_MAX: size (in bytes) of the local buffer that holds one JSON
 * message body before it's sent. Used by publish_state(), publish_heartbeat()
 * and sub_telemetry_publish_event() below. If a message ever needs to grow
 * (more fields), this is the first thing to check/raise so rc_snprintf()
 * doesn't silently truncate it. */
#define PAYLOAD_MAX     (192)

/*
 *  Topic design. One base, one leaf per message class, which keeps a
 *  broker subscription simple: car/+/state subscribes to every car's
 *  state, car/01/# to everything from this one.
 *
 *  TOPIC_BASE: the common prefix for every topic this car publishes to
 *  (car number "01" -- would change if multiple cars shared one broker).
 *  It's a C string literal (text in double quotes); #define-ing it here
 *  means every function below builds its topic as TOPIC_BASE "/leafname"
 *  instead of retyping "car/01" everywhere.
 */
#define TOPIC_BASE      "car/01"

/* --- Module-level (file-scope) state below ---
 * `static` on a variable at file scope means it is private to this .c
 * file: no other .c file can see or link against it directly, even
 * though it lives for the whole lifetime of the program (unlike a local
 * variable inside a function, which disappears when the function
 * returns). This is C's way of keeping a module's internal state
 * encapsulated when there are no classes/objects to do it for you. */

/* Pointer to the currently active transport (see sub_telemetry_sink_t in
 * the header). NULL until sub_telemetry_init() sets it to the console
 * sink. Everything in this file sends messages *through* whatever this
 * currently points at -- that's the whole "pluggable transport" trick. */
static const sub_telemetry_sink_t *sink;
static ID       telem_tskid;   /* the RTOS's handle/ID for the background telemetry task created in sub_telemetry_init() */
static uint32_t seq;           /* monotonically increasing message counter, stamped into every "state" message so a listener can spot gaps/reordering */
static uint32_t tx_count;      /* how many messages have been sent successfully so far -- reported in the heartbeat */
static uint32_t tx_fail;       /* how many send attempts have failed so far (sink down, etc.) -- reported in the heartbeat */
static sub_telemetry_cmd_cb_t cmd_cb;  /* the callback registered via sub_telemetry_on_command(), or NULL if none yet */
static void    *cmd_ctx;               /* the opaque context pointer that comes back unchanged whenever cmd_cb is eventually called */

/* Cached state, updated by callbacks, serialised by the telemetry task.
 * Each field is written by exactly one callback and read by one task, so
 * no lock is needed for a word-sized value on this architecture.
 *
 * `volatile` tells the compiler "this value can change at any time due
 * to something outside the normal flow of this function (here: another
 * task's callback running), so never cache it in a register or optimise
 * away a re-read of it." Without volatile, an aggressive compiler could
 * legally read st_speed_l once and reuse a stale copy. These fields are
 * the module's little "mailbox" of the latest known sensor values, kept
 * up to date by on_odometry()/on_line() below and read out whenever
 * publish_state() builds the next status message. */
static volatile int32_t  st_speed_l;   /* left wheel speed, mm/s, signed (negative = reversing) -- last value from an RC_EVT_ODOMETRY event */
static volatile int32_t  st_speed_r;   /* right wheel speed, mm/s, signed -- same source */
static volatile uint32_t st_dist_mm;   /* average of left+right distance travelled, in millimetres -- see on_odometry() below */
static volatile bool     st_line_l;    /* true if the left line sensor currently reports "on the black line" */
static volatile bool     st_line_r;    /* true if the right line sensor currently reports "on the black line" */

/* ------------------------------------------------------------------ *
 *  Console sink. Always available, needs no radio, works from the first
 *  day of integration.
 * ------------------------------------------------------------------ */

/* The four functions below are the console sink's implementation of the
 * sub_telemetry_sink_t "interface" from the header -- their names don't
 * matter to the rest of the program, only their addresses do, once they
 * are packed into the console_sink struct further down. `static` here
 * means these helper functions are private to this file too -- nothing
 * outside needs to call console_open()/console_publish()/etc. directly,
 * only through the function-pointer struct. */

/* Nothing to set up for the console -- it's always ready, so this just
 * reports success. */
static rc_result_t console_open(void)
{
    return RC_OK;
}

/* Prints one topic+payload pair to the debug console (the same UART/USB
 * console you see boot logs on). `(void)len;` is a common C idiom for
 * "yes, I know I'm not using this parameter, don't warn me about it" --
 * len isn't needed here because tm_printf() can find the end of the
 * payload itself (it's a plain null-terminated C string), but the sink
 * interface always passes it so *other* transports (e.g. a raw UDP
 * socket) that can't infer a length from the bytes alone still get it.
 * The cast `(UB *)` converts the string's type to whatever the RTOS's
 * tm_printf() expects (UB = "unsigned byte", the kernel's own typedef);
 * it doesn't change the bytes, just satisfies the compiler's type
 * checking for this particular API. */
static rc_result_t console_publish(const char *topic, const char *payload,
                                   uint16_t len)
{
    (void)len;
    tm_printf((UB *)"[telem] %s %s\n", topic, payload);
    return RC_OK;
}

/* Nothing to tear down for the console either. */
static rc_result_t console_close(void)
{
    return RC_OK;
}

/* The console is always considered "up" -- there's no cable to unplug. */
static bool console_is_up(void)
{
    return true;
}

/* Builds the actual sub_telemetry_sink_t value for the console transport,
 * filling each function-pointer field with the address of the matching
 * function above (in C, writing a function's name without parentheses
 * gives you a pointer to it). This is the concrete "plug" that gets
 * handed to sub_telemetry_set_sink() -- or used by default at boot. */
static const sub_telemetry_sink_t console_sink = {
    "console", console_open, console_publish, console_close, console_is_up
};

/* Public accessor so other files can get a pointer to the one shared
 * console_sink instance without it needing to be a global (non-static)
 * variable. Returns its address (&console_sink), not a copy. */
const sub_telemetry_sink_t *sub_telemetry_console_sink(void)
{
    return &console_sink;
}

/* ------------------------------------------------------------------ *
 *  Framing
 *
 *  JSON, because it is readable during debugging and every broker tool
 *  understands it. It is not the most compact choice. If the resource
 *  efficiency part of the assessment bites, swap the body of these
 *  functions for a packed binary struct and keep the topics; nothing
 *  else in the tree cares.
 * ------------------------------------------------------------------ */

/* The shared "mailroom": every outgoing message funnels through this one
 * function. `leaf` is the last part of the topic (e.g. "state"), `payload`
 * is the already-built JSON text. It glues TOPIC_BASE and leaf together,
 * bails out early if there's no working transport, and hands the result
 * off to whichever sink is currently installed -- keeping the counting
 * (tx_count/tx_fail) in one place so callers don't have to repeat it. */
static rc_result_t send(const char *leaf, const char *payload)
{
    char topic[TOPIC_MAX];   /* local (stack) buffer -- exists only while send() is running */
    rc_result_t res;

    /* `sink == NULL` guards against calling through an uninitialised
     * pointer (before sub_telemetry_init() runs). `!sink->is_up()` asks
     * the active transport whether it's actually usable right now (e.g.
     * WiFi still connected). Either way, count it as a failure and give
     * up without touching the transport. */
    if ((sink == NULL) || !sink->is_up()) {
        tx_fail++;
        return RC_ERR_STATE;
    }

    /* rc_snprintf: this project's bounded string-formatting helper (like
     * the standard C sprintf, but it also takes sizeof(topic) so it can
     * never write past the end of the buffer, which is what makes the
     * fixed-size TOPIC_MAX buffer safe). "%s/%s" glues TOPIC_BASE and
     * leaf together with a "/" in between, e.g. "car/01" + "state" ->
     * "car/01/state". `(void)` on the front just discards the return
     * value (the number of characters that would have been written) --
     * another "yes, I meant to ignore this" marker. */
    (void)rc_snprintf(topic, sizeof(topic), "%s/%s", TOPIC_BASE, leaf);

    /* sink->publish(...) -- this is calling *through* a function
     * pointer: "whichever function the active sink's `publish` field
     * points to, call it now with these arguments." This is what makes
     * the transport swappable -- this line never changes no matter which
     * sink is installed. rc_strlen() is this project's strlen() stand-in
     * (returns a string's length not counting the terminating '\0');
     * the cast to uint16_t matches the `len` parameter's type in the
     * sink interface. */
    res = sink->publish(topic, payload, (uint16_t)rc_strlen(payload));
    if (res == RC_OK) {
        tx_count++;
    } else {
        tx_fail++;
    }
    return res;
}

/* Builds and sends the main "what's happening right now" status message.
 * Called once per telemetry task tick (see telemetry_task() below). */
static void publish_state(void)
{
    char buf[PAYLOAD_MAX];

    /* Hand-builds one JSON (JavaScript Object Notation -- a plain-text,
     * human-readable way to represent {"key":value} data, widely
     * understood by tools/brokers) object by printing directly into
     * `buf` with rc_snprintf. Each %-placeholder pulls in one live
     * value; the (unsigned long)/(long)/(int)/(unsigned int) casts exist
     * because rc_snprintf's %lu/%ld/%d/%u format specifiers expect
     * exactly those C types, and the source variables (uint32_t,
     * int32_t, etc.) don't always match bit-for-bit on this platform --
     * the cast just relabels the value's type for the call, it doesn't
     * change the number. */
    (void)rc_snprintf(buf, sizeof(buf),
        "{\"seq\":%lu,\"t\":%lu,\"spd_l\":%ld,\"spd_r\":%ld,"
        "\"dist\":%lu,\"m_l\":%d,\"m_r\":%d,\"line\":\"%c%c\","
        "\"cls\":%d,\"peak\":%u,\"bc\":\"%c\"}",
        (unsigned long)seq,                /* message sequence number, see `seq` above */
        (unsigned long)rc_time_ms(),       /* milliseconds since boot -- a lightweight timestamp */
        (long)st_speed_l,                  /* left wheel speed, mm/s (from Buddy 2 via RC_EVT_ODOMETRY) */
        (long)st_speed_r,                  /* right wheel speed, mm/s */
        (unsigned long)st_dist_mm,         /* average distance travelled, mm */
        (int)drv_motor_get(RC_SIDE_LEFT),  /* current commanded left motor duty (Buddy 2's driver) */
        (int)drv_motor_get(RC_SIDE_RIGHT), /* current commanded right motor duty */
        st_line_l ? '1' : '0',             /* '?:' is the ternary/conditional operator: "if st_line_l is true use '1', else use '0'" -- a compact inline if/else that produces a value */
        st_line_r ? '1' : '0',
        (int)sub_terrain_motion_class(),   /* Buddy 4's current motion classification (stationary/turning/climbing/...) */
        (unsigned int)sub_terrain_max_peak_mm(), /* Buddy 4's tallest hump seen so far, mm */
        (sub_barcode_last() != '\0') ? sub_barcode_last() : '-'); /* last decoded barcode char (Buddy 3), or '-' if none yet ('\0' is the null byte C uses to mean "no character") */

    seq++;                    /* bump the sequence counter for the *next* message */
    (void)send("state", buf); /* publishes to topic "car/01/state" */
}

/* Builds and sends a less-frequent "is the link healthy" summary --
 * called every 8th tick by telemetry_task() below (roughly every 2
 * seconds at the default period). */
static void publish_heartbeat(void)
{
    char buf[PAYLOAD_MAX];

    (void)rc_snprintf(buf, sizeof(buf),
        "{\"up\":%lu,\"tx\":%lu,\"fail\":%lu,"
        "\"drop_fast\":%lu,\"drop_slow\":%lu,\"sink\":\"%s\"}",
        (unsigned long)rc_time_ms(),   /* uptime in ms */
        (unsigned long)tx_count,       /* messages sent successfully so far */
        (unsigned long)tx_fail,        /* messages that failed to send so far */
        /* rc_event_dropped(): asks the event bus how many events were
         * dropped (never delivered, e.g. because a subscriber's queue
         * was full) on each lane -- RC_LANE_FAST is the high-priority
         * "steer/react now" lane, RC_LANE_SLOW the lower-priority
         * "logging/telemetry can wait" lane this module itself uses
         * (see §0.2 of TEAM_GUIDE.md for the fast/slow lane concept). */
        (unsigned long)rc_event_dropped(RC_LANE_FAST),
        (unsigned long)rc_event_dropped(RC_LANE_SLOW),
        (sink != NULL) ? sink->name : "none");  /* name of the currently active transport, or "none" if not yet set */

    (void)send("status", buf);  /* publishes to topic "car/01/status" */
}

/* ------------------------------------------------------------------ *
 *  Event subscriptions. Slow lane throughout: telemetry must never get
 *  in front of the control path.
 * ------------------------------------------------------------------ */

/* These on_*() functions are event-bus subscriber callbacks: they are
 * registered against a specific event ID in sub_telemetry_init() below
 * (via rc_event_subscribe()), and the event bus calls them automatically
 * whenever that kind of event is published anywhere in the program --
 * this module never calls Buddy 2/3's code directly, it just reacts to
 * their announcements. See TEAM_GUIDE.md §0.2 for the publish/subscribe
 * idea. Every subscriber callback has the same shape: a pointer to the
 * event data, and the `ctx` pointer that was supplied at subscribe time
 * (unused here, hence the `(void)ctx;` "I know, ignore it" line).
 * `evt->u.odometry.speed_l_mm_s` -- `evt` is a pointer to a struct, so
 * `->` (instead of `.`) is used to reach into the struct it points at;
 * `u` is a "union" field (one block of memory reinterpreted differently
 * depending on evt->id -- odometry data here, line data in on_line(),
 * etc.), which is how one generic rc_event_t type carries many different
 * payload shapes without wasting memory on all of them at once. */

/* Fires on RC_EVT_ODOMETRY (published by Buddy 2's motion code). Just
 * copies the latest speed/distance numbers into this module's cached
 * state (st_speed_l etc.) so publish_state() can report them later --
 * it does no other work, keeping this callback fast. */
static void on_odometry(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    st_speed_l = evt->u.odometry.speed_l_mm_s;
    st_speed_r = evt->u.odometry.speed_r_mm_s;
    /* Average of both wheels' distance; `/ 2U` -- the `U` suffix marks
     * the literal 2 as "unsigned", matching the unsigned fields being
     * divided, which avoids a signed/unsigned mismatch warning. */
    st_dist_mm = (evt->u.odometry.dist_l_mm + evt->u.odometry.dist_r_mm) / 2U;
}

/* Fires on RC_EVT_LINE_SAMPLE (published by Buddy 3's line sensor code).
 * Caches whether each line sensor currently sees the black line. */
static void on_line(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    st_line_l = evt->u.line.on_line_l;
    st_line_r = evt->u.line.on_line_r;
}

/* Shared handler for every "noteworthy, send it now" event (barcode
 * decoded, hump finished, obstacle profile ready, impact detected -- see
 * the subscriptions in sub_telemetry_init()). Rather than waiting for
 * the next scheduled tick like publish_state()/publish_heartbeat() do,
 * it immediately hands the event off to sub_telemetry_publish_event()
 * below to build and send its own out-of-band message. */
static void on_notable(const rc_event_t *evt, void *ctx)
{
    (void)ctx;
    (void)sub_telemetry_publish_event(evt);
}

/* Builds and sends the JSON for one "noteworthy" event, on its own topic
 * per event type (see the individual `send("...", buf)` calls below).
 * This is the function on_notable() calls, and it's also exposed in the
 * header so other code can push an event out immediately without waiting
 * for it to also flow through the slow-lane event bus. A `switch` picks
 * the right JSON shape based on which kind of event this is (`evt->id`);
 * each case reaches into the matching member of the `u` union described
 * above. */
rc_result_t sub_telemetry_publish_event(const rc_event_t *evt)
{
    char buf[PAYLOAD_MAX];

    switch (evt->id) {
    case RC_EVT_BARCODE_DECODED:  /* Buddy 3 decoded a barcode character -> topic car/01/barcode */
        (void)rc_snprintf(buf, sizeof(buf),
            "{\"sym\":\"%c\",\"cmd\":%d,\"rev\":%d}",
            evt->u.barcode.symbol,
            (int)evt->u.barcode.command,
            evt->u.barcode.reversed ? 1 : 0);
        return send("barcode", buf);

    case RC_EVT_HUMP_END:  /* Buddy 4 finished tracking a speed hump -> topic car/01/hump */
        (void)rc_snprintf(buf, sizeof(buf),
            "{\"peak_mm\":%u,\"ms\":%lu,\"pitch\":%d}",
            (unsigned int)evt->u.hump.peak_mm,
            (unsigned long)evt->u.hump.duration_ms,
            (int)evt->u.hump.max_pitch_deg);
        return send("hump", buf);

    case RC_EVT_OBSTACLE_PROFILE:  /* Buddy 5 finished a scan and built an obstacle profile -> topic car/01/obstacle */
        (void)rc_snprintf(buf, sizeof(buf),
            "{\"n\":%u,\"near_mm\":%u,\"at_deg\":%d,\"w_mm\":%u,"
            "\"cl_l\":%u,\"cl_r\":%u}",
            (unsigned int)evt->u.profile.n_points,
            (unsigned int)evt->u.profile.closest_mm,
            (int)evt->u.profile.closest_angle_deg,
            (unsigned int)evt->u.profile.width_mm,
            (unsigned int)evt->u.profile.clearance_left_mm,
            (unsigned int)evt->u.profile.clearance_right_mm);
        return send("obstacle", buf);

    case RC_EVT_IMPACT:  /* Buddy 4's IMU detected a hard jolt -> topic car/01/impact; fixed payload, nothing to fill in */
        return send("impact", "{\"impact\":1}");

    default:
        /* Any event ID this function doesn't know how to serialise --
         * should not normally happen since sub_telemetry_init() only
         * subscribes on_notable() to the IDs handled above, but this is
         * a safe fallback rather than silently doing nothing. */
        return RC_ERR_PARAM;
    }
}

/* ------------------------------------------------------------------ *
 *  Task
 * ------------------------------------------------------------------ */

/* The background task body. In this RTOS, a "task" is like a thread --
 * an independent stream of execution the kernel schedules alongside all
 * the other tasks (motor control, line following, ...). This function's
 * shape (INT stacd, void *exinf) is fixed by the RTOS's task API -- it's
 * what tk_cre_tsk() below expects to be able to call. `stacd`/`exinf`
 * are start-up parameters this task doesn't use, hence the `(void)...;`
 * "ignore this" lines. The `for (;;) { ... }` is an intentional infinite
 * loop -- this task runs for the entire lifetime of the program, just
 * like every other subsystem's background task. */
static void telemetry_task(INT stacd, void *exinf)
{
    uint32_t ticks = 0U;

    (void)stacd;
    (void)exinf;

    /* Open whatever sink is currently installed (the console sink,
     * unless sub_telemetry_set_sink() was already called before the
     * task started). */
    if (sink != NULL) {
        (void)sink->open();
    }

    for (;;) {
        /* tk_dly_tsk(): an RTOS kernel call that puts *this task* to
         * sleep for RC_PERIOD_TELEM_MS milliseconds, letting every other
         * task run in the meantime -- this is what makes the telemetry
         * loop periodic without a busy `while` loop burning CPU. See
         * TEAM_GUIDE.md §0.3 for why nothing in this codebase is allowed
         * to block/spin instead. RC_PERIOD_TELEM_MS lives in
         * rc_config.h, the shared tunables file every subsystem's
         * period/pin constants live in (see also RC_PERIOD_IMU_MS used
         * by Buddy 4, for the same pattern). */
        tk_dly_tsk(RC_PERIOD_TELEM_MS);

        /*
         *  TODO Buddy 1: connection recovery belongs here. Check
         *  sink->is_up(), and on a drop, close, wait with a backoff, and
         *  reopen. Do the backoff with tk_dly_tsk, never a retry loop.
         */

        publish_state();   /* send the per-tick status message every time */

        ticks++;
        if ((ticks % 8U) == 0U) {       /* every 2 s at the default rate */
            /* `%` is the modulo operator (remainder after division) --
             * `ticks % 8U == 0` is true exactly every 8th time through
             * the loop, so publish_heartbeat() runs 8x less often than
             * publish_state(). This is a cheap way to run something on a
             * slower sub-schedule without a second timer/task. */
            publish_heartbeat();
        }
    }
}

/* ------------------------------------------------------------------ */

/* Module entry point -- called once from the app's startup code
 * (usermain). Subscribes to every event this module reports on, then
 * creates and starts the background task that does the actual periodic
 * sending. */
rc_result_t sub_telemetry_init(void)
{
    /* T_CTSK: the RTOS's "create task" configuration struct -- you fill
     * in its fields to describe the task you want (which function to
     * run, at what priority, with how much stack memory, etc.), then
     * hand it to tk_cre_tsk() below to actually create it. This is the
     * same struct-of-settings pattern used all over embedded RTOS APIs. */
    T_CTSK ctsk;

    sink = &console_sink;   /* default transport until/unless sub_telemetry_set_sink() is called */

    /* rc_event_subscribe(event_id, lane, callback, ctx): registers a
     * callback against one event ID on one lane. Every subscription
     * here uses RC_LANE_SLOW because telemetry must never be allowed to
     * delay the fast/reactive lane used for steering and obstacle
     * response -- see TEAM_GUIDE.md §0.2. `(void)` on each call discards
     * the subscribe result (an rc_result_t) since these are expected to
     * always succeed at boot; NULL is passed as `ctx` because none of
     * these callbacks need any extra context beyond the event itself. */
    (void)rc_event_subscribe(RC_EVT_ODOMETRY, RC_LANE_SLOW, on_odometry, NULL);
    (void)rc_event_subscribe(RC_EVT_LINE_SAMPLE, RC_LANE_SLOW, on_line, NULL);
    (void)rc_event_subscribe(RC_EVT_BARCODE_DECODED, RC_LANE_SLOW,
                             on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_HUMP_END, RC_LANE_SLOW, on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_OBSTACLE_PROFILE, RC_LANE_SLOW,
                             on_notable, NULL);
    (void)rc_event_subscribe(RC_EVT_IMPACT, RC_LANE_SLOW, on_notable, NULL);

    /* Fill in the task-creation settings: exinf/no extra data, priority
     * and stack size from the shared config constants (so every
     * subsystem's task follows the same conventions), which function
     * the task should run (telemetry_task, defined above), and task
     * attribute flags (TA_HLNG = written in high-level language i.e. C
     * rather than assembly; TA_RNG3 = runs in the least-privileged CPU
     * protection ring) required by this RTOS. */
    ctsk.exinf   = NULL;
    ctsk.itskpri = RC_PRI_TELEMETRY;
    ctsk.stksz   = RC_STACK_SZ;
    ctsk.task    = telemetry_task;
    ctsk.tskatr  = TA_HLNG | TA_RNG3;

    /* tk_cre_tsk(): the kernel call that actually creates the task from
     * the settings above and returns an ID (a small integer handle) for
     * it, or a negative error code on failure -- `telem_tskid <= E_OK`
     * checks for that failure case (E_OK is the kernel's "no error"
     * value; a valid task ID is always greater than it). */
    telem_tskid = tk_cre_tsk(&ctsk);
    if (telem_tskid <= E_OK) {
        return RC_ERR_HARDWARE;
    }
    /* tk_sta_tsk(): starts the task running (tk_cre_tsk only creates it
     * in a dormant state) -- the `0` is the task's start code, unused
     * here. From this point, telemetry_task() begins executing on its
     * own schedule alongside every other task in the system. */
    (void)tk_sta_tsk(telem_tskid, 0);

    return RC_OK;
}

/* Swaps the active transport. Closes whatever was open before (if
 * anything), then switches `sink` to point at the new one -- or back to
 * the safe console_sink if new_sink is NULL -- and opens it. Everything
 * else in this file (send(), etc.) reads through the `sink` pointer, so
 * after this call every future message goes out via the new transport
 * with no other code changes needed. */
rc_result_t sub_telemetry_set_sink(const sub_telemetry_sink_t *new_sink)
{
    if (sink != NULL) {
        (void)sink->close();
    }
    sink = (new_sink != NULL) ? new_sink : &console_sink;

    return sink->open();
}

/* Just stores the callback and its context pointer for later -- nothing
 * currently calls cmd_cb (see the TODO in sub_telemetry.h: no transport
 * yet actually receives commands from outside the robot). Once a
 * transport's receive path exists, it would call cmd_cb(cmd, arg,
 * cmd_ctx) whenever a command arrives. */
rc_result_t sub_telemetry_on_command(sub_telemetry_cmd_cb_t cb, void *ctx)
{
    cmd_cb  = cb;
    cmd_ctx = ctx;
    return RC_OK;
}
