/*
 *  rc_config.h
 *
 *  Single place for every pin number and tuning constant in the robot.
 *  Nothing else in the tree hard-codes a GPIO.
 *
 *  Target: Raspberry Pi Pico W on a Cytron Robo Pico carrier, running
 *  micro T-Kernel 3.0 (mtk3smp-rp2040 port).
 *
 *  Read docs/HARDWARE.md before changing anything here. Several of these
 *  pins are fixed by the Robo Pico board and cannot be moved.
 */

#ifndef RC_CONFIG_H
#define RC_CONFIG_H

/* ------------------------------------------------------------------ *
 *  Fixed by the Robo Pico carrier board. Do not change.
 * ------------------------------------------------------------------ */

#define RC_PIN_MOTOR_L_A        (8U)    /* M1A, PWM slice 4 chan A */
#define RC_PIN_MOTOR_L_B        (9U)    /* M1B, PWM slice 4 chan B */
#define RC_PIN_MOTOR_R_A        (10U)   /* M2A, PWM slice 5 chan A */
#define RC_PIN_MOTOR_R_B        (11U)   /* M2B, PWM slice 5 chan B */

#define RC_PIN_SERVO_SCAN       (12U)   /* Servo port 1, slice 6 chan A */
#define RC_PIN_SERVO_SPARE      (13U)   /* Servo port 2, slice 6 chan B */

#define RC_PIN_BUZZER           (22U)
#define RC_PIN_BTN_1            (20U)
#define RC_PIN_BTN_2            (21U)
#define RC_PIN_NEOPIXEL         (18U)

/* ------------------------------------------------------------------ *
 *  Chosen by us. Move these if your wiring differs.
 * ------------------------------------------------------------------ */

/* Wheel encoders. GP4/GP5 are Grove port 3, so both fit one cable. */
#define RC_PIN_ENC_L            (4U)
#define RC_PIN_ENC_R            (5U)

/* HC-SR04. GP16/GP17 are Grove port 4.
 * ECHO is 5 V on this module and MUST go through a divider. See HARDWARE.md. */
#define RC_PIN_ULTRA_TRIG       (16U)
#define RC_PIN_ULTRA_ECHO       (17U)

/* IR line sensors, digital comparator output (DOUT). */
#define RC_PIN_IR_LINE_L        (6U)
#define RC_PIN_IR_LINE_R        (7U)

/* IR barcode sensor. Analogue path is used for decoding, digital as backup. */
#define RC_PIN_IR_BARCODE_A     (26U)   /* ADC channel 0 */
#define RC_ADC_CH_IR_BARCODE    (0U)
#define RC_PIN_IR_BARCODE_D     (27U)

/* Liveness LED. The port defaults BOARD_LED_PIN to GP16, which we need for
 * the ultrasonic trigger, so override it in sysdef.h to this pin. */
#define RC_PIN_STATUS_LED       (19U)

/* IMU sits on I2C1. The stock driver maps I2C1 to GP6/GP7, which collides
 * with the line sensors above, so the driver pin table is patched to
 * GP2/GP3 (Robo Pico Grove 2 / Maker port). See HARDWARE.md. */
#define RC_I2C_UNIT_IMU         (1U)
#define RC_PIN_I2C1_SDA         (2U)
#define RC_PIN_I2C1_SCL         (3U)

/* LSM303DLHC has two separate I2C addresses, one per die. */
#define RC_I2C_ADDR_ACCEL       (0x19U)
#define RC_I2C_ADDR_MAG         (0x1EU)

/* ------------------------------------------------------------------ *
 *  Timing
 * ------------------------------------------------------------------ */

#define RC_TICK_PERIOD_MS       (1)     /* kernel tick, matches the port */

#define RC_PERIOD_MOTION_MS     (20)    /* PID update rate, 50 Hz */
#define RC_PERIOD_LINE_MS       (5)     /* line sampling, 200 Hz */
#define RC_PERIOD_IMU_MS        (10)    /* IMU sampling, 100 Hz */
#define RC_PERIOD_SCAN_MS       (60)    /* one ultrasonic ping per step */
#define RC_PERIOD_TELEM_MS      (250)   /* telemetry publish rate */

/* ------------------------------------------------------------------ *
 *  Task priorities. Lower number wins in micro T-Kernel.
 * ------------------------------------------------------------------ */

#define RC_PRI_DEFER            (4)   /* bottom halves, above all */
#define RC_PRI_DISPATCH_FAST    (5)
#define RC_PRI_MOTION           (6)
#define RC_PRI_SENSE            (7)
#define RC_PRI_DISPATCH_SLOW    (9)
#define RC_PRI_TELEMETRY        (10)
#define RC_PRI_NAV              (8)

#define RC_STACK_SZ             (4096)  /* SMP kernel paths are deep */

/* ------------------------------------------------------------------ *
 *  Mechanical constants. Measure these on your own car.
 * ------------------------------------------------------------------ */

#define RC_ENC_SLOTS_PER_REV    (20U)   /* slots on the encoder disc */
#define RC_WHEEL_DIAM_MM        (65U)
#define RC_WHEEL_BASE_MM        (110U)  /* centre to centre of the wheels */

/* Distance per encoder edge, in micrometres, to stay in integer maths.
 * There is no FPU on the RP2040, so everything downstream is fixed point. */
#define RC_ENC_UM_PER_TICK      ((31416UL * (unsigned long)RC_WHEEL_DIAM_MM) \
                                 / (10UL * (unsigned long)RC_ENC_SLOTS_PER_REV))

/* ------------------------------------------------------------------ *
 *  Ultrasonic
 * ------------------------------------------------------------------ */

#define RC_ULTRA_TIMEOUT_US     (30000UL)  /* ~5 m, give up after this */
#define RC_ULTRA_TRIG_US        (12UL)     /* datasheet asks for >= 10 us */
#define RC_ULTRA_MAX_MM         (4000U)
#define RC_ULTRA_MIN_MM         (20U)

/* ------------------------------------------------------------------ *
 *  Servo scan geometry
 * ------------------------------------------------------------------ */

#define RC_SCAN_COARSE_START    (30)
#define RC_SCAN_COARSE_END      (150)
#define RC_SCAN_COARSE_STEP     (30)
#define RC_SCAN_FINE_STEP       (6)
#define RC_SCAN_MAX_POINTS      (32U)

/* ------------------------------------------------------------------ *
 *  Event bus sizing
 * ------------------------------------------------------------------ */

#define RC_EVENT_RING_SZ        (64U)   /* must be a power of two */
#define RC_EVENT_MAX_SUBS       (32U)   /* total subscriptions per lane */

#endif /* RC_CONFIG_H */
