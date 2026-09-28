/* ==========================================================================
 *  aeb.c  —  AUTONOMOUS EMERGENCY BRAKING CONTROLLER  (single-file build)
 *  QNX 8.0 RTOS · Raspberry Pi 5 · Team PRIORITY ZERO
 *  QNX eHACK 2026 · Problem Statement #2 (Automotive)
 *
 *  ONE process · EIGHT SCHED_FIFO threads · THREE message channels · pulses
 *  for timers / heartbeats / emergency override.
 *
 *  Every component is present:
 *    - RD-03D 24 GHz mmWave radar   (UART0 @ 256000, multi-target parser)
 *    - 2× L298N motor drivers        (all 4 wheels, fwd/rev/dynamic-brake)
 *    - 4× gear motors                (software PWM on the EN pins @ 1 kHz)
 *    - DPDT master switch            (GPIO24 interlock sense)
 *    - 4×AA pack (motors) + USB-C (Pi), single common-ground star point
 *
 *  BUILD (on a QNX SDP host):
 *      source ~/qnx800/qnxsdp-env.sh
 *      qcc -Vgcc_ntoaarch64le -O2 -Wall -std=gnu11 -D_QNX_SOURCE aeb.c -o aeb -lm
 *      # copy ./aeb to the target and run it there
 *
 *  >>> HARDWARE SEAM (must verify on your board) <<<
 *  The Pi 5 drives GPIO through the RP1 chip, NOT the classic BCM memory map.
 *  The GPIO block below is written against the classic layout (correct on a
 *  Pi 4) with every Pi-5 change point marked  ***RP1***.  Bind it to your
 *  BSP's GPIO driver / RP1 register window. Everything else is portable.
 * ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/neutrino.h>     /* ChannelCreate, MsgSend, pulses, ThreadCtl   */
#include <sys/dispatch.h>
#include <hw/inout.h>
#include <hw/i2c.h>           /* I²C master devctl interface (IMU)           */

/* ##########################################################################
 * SECTION 1 — CONFIGURATION  (pins, priorities, timing, model constants)
 * ######################################################################## */

/* ---- 1.1 GPIO pin map (BCM GPIO numbers, verbatim from slide 5) -------- */
enum {
    /* L298N #1 — FRONT axle */
    M1_ENA = 12, M1_IN1 = 5,  M1_IN2 = 6,
    M1_ENB = 13, M1_IN3 = 16, M1_IN4 = 26,
    /* L298N #2 — REAR axle */
    M2_ENA = 18, M2_IN1 = 17, M2_IN2 = 27,
    M2_ENB = 19, M2_IN3 = 22, M2_IN4 = 23,
    /* DPDT master-switch interlock sense */
    INTERLOCK_SENSE = 24
};

#define PWM_HZ           1000            /* software PWM engine frequency     */
#define PWM_PERIOD_US    (1000000/PWM_HZ)
#define UART_RADAR_DEV   "/dev/ser1"     /* map to your BSP's UART0           */
#define UART_RADAR_BAUD  256000          /* RD-03D multi-target mode          */
#define I2C_DEV          "/dev/i2c1"     /* Pi I²C1 : SDA=GPIO2, SCL=GPIO3    */
#define MPU6050_ADDR     0x68            /* IMU 7-bit address (AD0 = low)     */

/* ---- 1.2 SCHED_FIFO priorities — tiers taken verbatim from the Excel sheet
 *        PS#2 "Tasks & Scheduling":
 *          Sensor Task (High) · Collision Prediction (Highest) ·
 *          Brake Controller (Highest) · Vehicle State (Medium) · Logger (Low)
 *        Collision Prediction and Brake Controller share the SAME "Highest"
 *        tier. The Safety Watchdog is placed strictly ABOVE them so it can
 *        always preempt the brake task it supervises (the sheet lists the
 *        watchdog under RTOS Concepts / Timers, not as a ranked task).      */
enum {
    PRIO_WATCHDOG = 62,   /* safety supervisor — above Highest              */
    PRIO_PREDICT  = 58,   /* HIGHEST                                        */
    PRIO_BRAKE    = 58,   /* HIGHEST  (EQUAL to prediction — per the sheet) */
    PRIO_PWM      = 57,   /* brake-actuation helper, just below Highest     */
    PRIO_RADAR    = 50,   /* HIGH  — sensor tier                            */
    PRIO_IMU      = 50,   /* HIGH  — sensor tier                            */
    PRIO_VEHICLE  = 30,   /* MEDIUM                                         */
    PRIO_LOGGER   = 10    /* LOW                                            */
};
#define PRIO_EMERGENCY_PULSE  63

/* ---- 1.3 CPU core affinity runmasks (slide 10) ------------------------- */
#define CORE0 (1u<<0)   /* brake + pwm  (safety actuation)                  */
#define CORE1 (1u<<1)   /* predict      (collision prediction)              */
#define CORE2 (1u<<2)   /* radar + imu + vehicle + logger (sensing/misc)    */
#define CORE3 (1u<<3)   /* watchdog     (supervision)                       */

/* ---- 1.4 Timing (slides 13-14, 17) ------------------------------------- */
#define SENSOR_PERIOD_MS     20      /* 50 Hz pipeline heartbeat             */
#define WATCHDOG_PERIOD_MS   10      /* 100 Hz liveness check                */
#define WATCHDOG_TIMEOUT_MS  120     /* silence beyond this => task dead     */
#define BRAKING_DEADLINE_US  50000   /* sense→actuation budget               */
#define T_DELAY_S            0.080   /* 80 ms total reaction budget          */

/* ---- 1.5 Braking model (slide 17)  D = v*t_delay + v^2/(2a) ------------- */
#define SAFETY_FACTOR   1.3
#define TTC_CRITICAL_S  0.6
#define DECEL_PARTIAL   2.0
#define DECEL_STRONG    4.5
#define DECEL_FULL      7.5
#define CRUISE_DUTY     60      /* forward EN duty when SAFE (car rolls)     */

/* ---- 1.5b IMU (MPU-6050) : deceleration feedback + safety monitoring --- */
#define IMU_PERIOD_MS      10      /* 100 Hz accelerometer sampling          */
#define IMPACT_G           3.0     /* |accel| spike beyond this = collision  */
#define BRAKE_FAULT_MS     200     /* window over which braking must bite    */
#define BRAKE_FAULT_FRAC   0.5     /* measured < 50% of commanded = fault    */
#define IMU_MOVING_MPS     1.0     /* only judge brake effectiveness moving  */

/* ---- 1.6 State machine (slide 15) -------------------------------------- */
typedef enum { ST_SAFE=0, ST_WARNING, ST_STRONG, ST_EMERGENCY } aeb_state_t;
typedef enum { LVL_NONE=0, LVL_PARTIAL, LVL_STRONG, LVL_FULL }  brake_level_t;

/* ##########################################################################
 * SECTION 2 — IPC PROTOCOL  (messages + pulses, slides 11-12)
 * ######################################################################## */
enum {
    PULSE_TICK_SENSOR = _PULSE_CODE_MINAVAIL + 1,
    PULSE_TICK_WDOG   = _PULSE_CODE_MINAVAIL + 2,
    PULSE_HEARTBEAT   = _PULSE_CODE_MINAVAIL + 3,
    PULSE_EMERGENCY   = _PULSE_CODE_MINAVAIL + 4
};
enum { MT_SENSOR = 1, MT_BRAKE = 2 };
typedef enum { HB_RADAR=0, HB_PREDICT, HB_BRAKE, HB_VEHICLE, HB_IMU, HB_COUNT } hb_id_t;

typedef struct {                 /* radar → predict */
    uint16_t type; double distance_m; double closing_mps;
    uint64_t t_sense_ns;
} sensor_msg_t;

typedef struct {                 /* predict → brake */
    uint16_t type; brake_level_t level; aeb_state_t state;
    double decel, ttc_s, margin_m, distance_m; uint64_t t_sense_ns; bool emergency;
} brake_cmd_t;

/* ##########################################################################
 * SECTION 3 — SHARED STATE + GLOBAL IPC HANDLES
 * ######################################################################## */
typedef struct {
    pthread_mutex_t lock;
    double speed_mps, obstacle_m, safety_factor, ttc_threshold;
    double decel_partial, decel_strong, decel_full;
    aeb_state_t state; brake_level_t level; double commanded_decel;
    /* IMU feedback + safety flags */
    double measured_decel;   /* longitudinal deceleration from the IMU, m/s^2 */
    double accel_mag_g;      /* total |acceleration| in g (impact detection)  */
    bool   imu_ok;           /* IMU present and responding                    */
    bool   brake_fault;      /* commanded braking not producing deceleration  */
    bool   impact;           /* collision-level deceleration spike seen       */
    bool interlock_ok; volatile bool running;
} controller_t;

static controller_t g_ctl;


static int g_chid_predict, g_chid_brake, g_chid_wdog;
static int g_coid_predict, g_coid_brake, g_coid_brake_em;
static int g_coid_wdog[HB_COUNT];

static void lock(void)   { pthread_mutex_lock(&g_ctl.lock);   }
static void unlock(void) { pthread_mutex_unlock(&g_ctl.lock); }

static uint64_t aeb_now_ns(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec*1000000000ull + (uint64_t)ts.tv_nsec;
}

/* ##########################################################################
 * SECTION 4 — HARDWARE ABSTRACTION LAYER  (GPIO / UART / I²C)
 * ######################################################################## */

/* ---- 4.1 GPIO (memory-mapped) ------------------------------------------ */
/* ***RP1*** Pi 5: replace base + offsets with the RP1 GPIO window from your
 * BSP. The classic layout below is correct on a Pi 4-class BSP.            */
#define BCM_GPIO_BASE  0xFE200000ul     /* Pi 4 peripheral base (BCM2711)    */
#define BCM_GPIO_LEN   0xB4
#define GPFSEL0  (0x00/4)
#define GPSET0   (0x1C/4)
#define GPCLR0   (0x28/4)
#define GPLEV0   (0x34/4)

static volatile uint32_t *g_gpio = NULL;

static int hal_gpio_init(void)
{
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1) { perror("ThreadCtl(_NTO_TCTL_IO)"); return -1; }
    g_gpio = (volatile uint32_t *)mmap_device_memory(
                 NULL, BCM_GPIO_LEN, PROT_READ|PROT_WRITE|PROT_NOCACHE,
                 0, BCM_GPIO_BASE);        /* ***RP1*** base for Pi 5         */
    if (g_gpio == MAP_FAILED) { perror("mmap_device_memory"); g_gpio=NULL; return -1; }
    return 0;
}
static void hal_gpio_fini(void)
{ if (g_gpio){ munmap_device_memory((void*)g_gpio, BCM_GPIO_LEN); g_gpio=NULL; } }

static void hal_gpio_mode(int gpio, int is_out)
{
    if (!g_gpio) return;
    int reg = GPFSEL0 + gpio/10, sh = (gpio%10)*3;
    uint32_t v = g_gpio[reg]; v &= ~(0x7u<<sh);
    if (is_out) v |= (0x1u<<sh);           /* 001=out, 000=in                */
    g_gpio[reg] = v;
}
static void hal_gpio_write(int gpio, int val)
{
    if (!g_gpio) return;
    if (val) g_gpio[GPSET0 + gpio/32] = (1u<<(gpio%32));
    else     g_gpio[GPCLR0 + gpio/32] = (1u<<(gpio%32));
}
static int hal_gpio_read(int gpio)
{
    if (!g_gpio) return -1;
    return (g_gpio[GPLEV0 + gpio/32] >> (gpio%32)) & 1u;
}

/* ---- 4.2 UART (radar) -------------------------------------------------- */
/* Device names vary by QNX image / BSP. Try the usual candidates in order
 * and report which one opened, instead of failing on a single hard-coded
 * name. Run  ls /dev/ser*  on the target to see what your image publishes. */
static const char *UART_CANDIDATES[] = {
    UART_RADAR_DEV, "/dev/ser1", "/dev/ser2", "/dev/ser3", "/dev/ser4", NULL
};

static int hal_uart_open(const char *dev, int baud)
{
    int fd = -1;
    const char *opened = NULL;
    for (int i = 0; UART_CANDIDATES[i]; ++i){
        if (i > 0 && dev && strcmp(dev, UART_CANDIDATES[i]) == 0) continue;
        fd = open(UART_CANDIDATES[i], O_RDWR | O_NOCTTY);
        if (fd >= 0){ opened = UART_CANDIDATES[i]; break; }
    }
    if (fd < 0){
        fprintf(stderr,
            "uart: no serial device found (tried /dev/ser1..4): %s\n"
            "      run  ls /dev/ser*  on the target; if empty the serial\n"
            "      driver is not running. Radar disabled; CLI bench mode\n"
            "      (o <m> / v <mps>) still exercises the full controller.\n",
            strerror(errno));
        return -1;
    }
    fprintf(stderr, "uart: opened %s\n", opened);
    struct termios tio;
    if (tcgetattr(fd,&tio)!=0){ perror("tcgetattr"); close(fd); return -1; }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL|CREAD); tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN]=1; tio.c_cc[VTIME]=1;
    if (cfsetispeed(&tio,baud)!=0 || cfsetospeed(&tio,baud)!=0)
        perror("cfsetspeed 256000 (verify BSP support)");
    if (tcsetattr(fd,TCSANOW,&tio)!=0){ perror("tcsetattr"); close(fd); return -1; }
    tcflush(fd,TCIOFLUSH);
    return fd;
}
static int  hal_uart_read(int fd, uint8_t *b, int n){ return (int)read(fd,b,(size_t)n); }
static void hal_uart_close(int fd){ if(fd>=0) close(fd); }

/* ---- 4.3 IMU (MPU-6050 over I²C) --------------------------------------- *
 * Uses the QNX i2c master devctl interface. Requires your BSP's I²C driver
 * running and publishing I2C_DEV (e.g. i2c-bcm2711 → /dev/i2c1). On the Pi 5
 * the controller is behind RP1 — confirm the device name your BSP exposes.
 *
 * MPU-6050 registers:  PWR_MGMT_1=0x6B (write 0 to wake), ACCEL_XOUT_H=0x3B
 * Accel is 6 bytes big-endian signed (X,Y,Z). Default range ±2g => 16384
 * LSB/g.  Mounting assumption: vehicle-forward axis = +X, gravity on Z.
 * ----------------------------------------------------------------------- */
#define MPU_PWR_MGMT_1  0x6B
#define MPU_ACCEL_XOUT  0x3B
#define MPU_LSB_PER_G   16384.0
#define G_MPS2          9.80665

static int hal_imu_open(const char *dev)
{
    int fd = open(dev, O_RDWR);
    if (fd < 0){ perror("i2c open (IMU optional; monitor disabled)"); return -1; }
    /* wake the MPU-6050: write 0x00 to PWR_MGMT_1 */
    struct { i2c_send_t hdr; uint8_t data[2]; } w;
    w.hdr.slave.addr = MPU6050_ADDR; w.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    w.hdr.len = 2; w.hdr.stop = 1;
    w.data[0] = MPU_PWR_MGMT_1; w.data[1] = 0x00;
    int rc = devctl(fd, DCMD_I2C_SEND, &w, sizeof w, NULL);
    if (rc != EOK){
        /* NOTE: QNX devctl() RETURNS the error code and does not set errno,
         * so strerror(rc) is correct here and perror() would be wrong.      */
        fprintf(stderr, "IMU wake (DCMD_I2C_SEND) failed: %s (%d)\n",
                strerror(rc), rc);
        close(fd);
        return -1;
    }
    return fd;
}

/* Read the three accel axes in m/s^2. Returns 0 on success, -1 on error. */
static int hal_imu_read_accel(int fd, double *ax, double *ay, double *az)
{
    if (fd < 0) return -1;
    struct { i2c_sendrecv_t hdr; uint8_t data[6]; } m;
    m.hdr.slave.addr = MPU6050_ADDR; m.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    m.hdr.send_len = 1; m.hdr.recv_len = 6; m.hdr.stop = 1;
    m.data[0] = MPU_ACCEL_XOUT;
    if (devctl(fd, DCMD_I2C_SENDRECV, &m, sizeof m, NULL) != EOK) return -1;
    int16_t rx = (int16_t)((m.data[0]<<8) | m.data[1]);
    int16_t ry = (int16_t)((m.data[2]<<8) | m.data[3]);
    int16_t rz = (int16_t)((m.data[4]<<8) | m.data[5]);
    *ax = (rx / MPU_LSB_PER_G) * G_MPS2;
    *ay = (ry / MPU_LSB_PER_G) * G_MPS2;
    *az = (rz / MPU_LSB_PER_G) * G_MPS2;
    return 0;
}
static void hal_imu_close(int fd){ if(fd>=0) close(fd); }

/* ##########################################################################
 * SECTION 5 — RD-03D RADAR FRAME PARSER
 *   Frame: AA FF 03 00 | T1(8) T2(8) T3(8) | 55 CC   (30 bytes, LE fields)
 *   Target block: [X mm][Y mm][speed cm/s][dist-res mm], each 16-bit.
 *   Sign: bit15=1 => positive (value = raw&0x7FFF); bit15=0 => negative.
 *   Range = sqrt(X^2+Y^2). Nearest target with speed>0 = the threat.
 * ######################################################################## */
typedef struct { bool valid; double distance_m; double closing_mps; } radar_target_t;
typedef struct { uint8_t buf[64]; int len; } radar_parser_t;

static void radar_parser_init(radar_parser_t *p){ p->len = 0; }

static int16_t rd03d_signed(uint8_t lo, uint8_t hi)
{
    uint16_t raw = (uint16_t)lo | ((uint16_t)hi<<8);
    int16_t mag = (int16_t)(raw & 0x7FFF);
    return (raw & 0x8000) ? mag : (int16_t)(-mag);
}
static bool parse_frame(const uint8_t *f, radar_target_t *out)
{
    if (!(f[0]==0xAA&&f[1]==0xFF&&f[2]==0x03&&f[3]==0x00)) return false;
    if (!(f[28]==0x55&&f[29]==0xCC)) return false;
    double best=1e9,spd=0; bool found=false;
    for (int t=0;t<3;t++){
        const uint8_t *b=f+4+t*8;
        int16_t x=rd03d_signed(b[0],b[1]), y=rd03d_signed(b[2],b[3]), v=rd03d_signed(b[4],b[5]);
        if (x==0&&y==0&&v==0) continue;
        double r=sqrt((double)x*x+(double)y*y)/1000.0;
        if (r<best){ best=r; spd=(double)v/100.0; found=true; }
    }
    if (!found){ out->valid=false; return true; }
    out->valid=true; out->distance_m=best; out->closing_mps=spd; return true;
}
static bool radar_feed(radar_parser_t *p, uint8_t byte, radar_target_t *out)
{
    if (p->len < (int)sizeof p->buf) p->buf[p->len++] = byte;
    if (p->len < 30) return false;
    for (int i=0; i+30 <= p->len; ++i){
        if (p->buf[i]==0xAA&&p->buf[i+1]==0xFF&&p->buf[i+2]==0x03&&p->buf[i+3]==0x00){
            if (parse_frame(&p->buf[i], out)){
                int consumed=i+30, rem=p->len-consumed;
                memmove(p->buf, p->buf+consumed, rem); p->len=rem;
                return true;
            }
        }
    }
    if (p->len > 30){ int keep=29; memmove(p->buf,p->buf+(p->len-keep),keep); p->len=keep; }
    return false;
}

/* ##########################################################################
 * SECTION 6 — MOTOR PRIMITIVES  (2× L298N, 4 wheels, software PWM)
 * ######################################################################## */
static const int EN_PINS[4]   = { M1_ENA, M1_ENB, M2_ENA, M2_ENB };
static const int IN_PINS[4][2]= { {M1_IN1,M1_IN2}, {M1_IN3,M1_IN4},
                                  {M2_IN1,M2_IN2}, {M2_IN3,M2_IN4} };
static volatile int g_duty[4] = { 0,0,0,0 };   /* per-EN duty 0..100         */

static void wheel_forward(int w){ hal_gpio_write(IN_PINS[w][0],1); hal_gpio_write(IN_PINS[w][1],0); }
__attribute__((unused)) static void wheel_reverse(int w){ hal_gpio_write(IN_PINS[w][0],0); hal_gpio_write(IN_PINS[w][1],1); }
static void wheel_brake  (int w){ hal_gpio_write(IN_PINS[w][0],0); hal_gpio_write(IN_PINS[w][1],0); }

static void motors_init_pins(void)
{
    for (int w=0;w<4;w++){ hal_gpio_mode(IN_PINS[w][0],1); hal_gpio_mode(IN_PINS[w][1],1); wheel_brake(w); }
    for (int i=0;i<4;i++){ hal_gpio_mode(EN_PINS[i],1); hal_gpio_write(EN_PINS[i],0); }
    hal_gpio_mode(INTERLOCK_SENSE, 0);
}
static void apply_level(brake_level_t lvl)
{
    /* NONE = drive forward at cruise duty; braking = short-brake (both IN low)
     * with EN duty scaled to the braking level (dynamic braking).            */
    int duty = (lvl==LVL_NONE)?CRUISE_DUTY : (lvl==LVL_PARTIAL)?40 :
               (lvl==LVL_STRONG)?70 : 100;
    for (int w=0;w<4;w++){
        if (lvl==LVL_NONE) wheel_forward(w); else wheel_brake(w);
        g_duty[w]=duty;
    }
}
/* NB: wheel_reverse() is ready for the judges' "reverse instead of brake"
 * modification round — swap the apply_level() brake call for wheel_reverse. */

/* ##########################################################################
 * SECTION 7 — LOGGING (ring buffer + latency history, slide 18)
 * ######################################################################## */
typedef struct {
    uint64_t t_ns; aeb_state_t state; brake_level_t level;
    double distance_m, ttc_s, margin_m; uint64_t latency_ns;
} log_evt_t;

#define LOG_RING 64
static log_evt_t s_ring[LOG_RING];
static volatile int s_head=0, s_tail=0;
static pthread_mutex_t s_ring_lock = PTHREAD_MUTEX_INITIALIZER;

static void log_push(const log_evt_t *e)
{
    pthread_mutex_lock(&s_ring_lock);
    int nxt=(s_head+1)%LOG_RING;
    if (nxt!=s_tail){ s_ring[s_head]=*e; s_head=nxt; }   /* full => drop      */
    pthread_mutex_unlock(&s_ring_lock);
}

/* Collision timeline: a separate ring the logger never drains, so the 't'
 * CLI command can always show the most recent events (slide 18).            */
#define TL_HIST 16
static log_evt_t s_tl[TL_HIST]; static volatile int s_tl_n=0, s_tl_i=0;
static pthread_mutex_t s_tl_lock = PTHREAD_MUTEX_INITIALIZER;

static void timeline_push(const log_evt_t *e)
{
    pthread_mutex_lock(&s_tl_lock);
    s_tl[s_tl_i]=*e; s_tl_i=(s_tl_i+1)%TL_HIST; if (s_tl_n<TL_HIST) s_tl_n++;
    pthread_mutex_unlock(&s_tl_lock);
}
static void timeline_print(void)
{
    const char *ST[]={"SAFE","WARNING","STRONG","EMERGENCY"};
    const char *LV[]={"NONE","PARTIAL","STRONG","FULL"};
    pthread_mutex_lock(&s_tl_lock);
    if (!s_tl_n){ puts("(no events yet)"); pthread_mutex_unlock(&s_tl_lock); return; }
    puts("  t(ms)     state     level   dist(m)  ttc(s)  margin(m)  lat(ms)");
    int start=(s_tl_i - s_tl_n + TL_HIST)%TL_HIST;
    for (int k=0;k<s_tl_n;k++){
        log_evt_t e=s_tl[(start+k)%TL_HIST];
        printf("  %-8.1f %-9s %-7s %7.2f %7.2f %9.2f %8.3f\n",
            e.t_ns/1e6, ST[e.state], LV[e.level], e.distance_m,
            e.ttc_s, e.margin_m, e.latency_ns/1e6);
    }
    pthread_mutex_unlock(&s_tl_lock);
}

#define LAT_HIST 128
static uint64_t s_lat[LAT_HIST]; static volatile int s_lat_n=0, s_lat_i=0;
static volatile uint32_t s_misses=0;
static pthread_mutex_t s_lat_lock = PTHREAD_MUTEX_INITIALIZER;

static void lat_push(uint64_t ns)
{
    pthread_mutex_lock(&s_lat_lock);
    s_lat[s_lat_i]=ns; s_lat_i=(s_lat_i+1)%LAT_HIST;
    if (s_lat_n<LAT_HIST) s_lat_n++;
    if (ns > (uint64_t)BRAKING_DEADLINE_US*1000ull) s_misses++;
    pthread_mutex_unlock(&s_lat_lock);
}
static void lat_stats(uint64_t *mn,uint64_t *mx,double *avg,uint32_t *miss,uint32_t *cnt)
{
    pthread_mutex_lock(&s_lat_lock);
    uint64_t lo=~0ull,hi=0,sum=0;
    for (int i=0;i<s_lat_n;i++){ uint64_t v=s_lat[i]; if(v<lo)lo=v; if(v>hi)hi=v; sum+=v; }
    *mn=s_lat_n?lo:0; *mx=hi; *avg=s_lat_n?(double)sum/s_lat_n:0; *miss=s_misses; *cnt=s_lat_n;
    pthread_mutex_unlock(&s_lat_lock);
}

/* ##########################################################################
 * SECTION 8 — BRAKING MODEL  (stopping distance, decision ladder)
 * ######################################################################## */
static double stopping_distance(double v, double a)
{ return (a<=0.0) ? 1e9 : v*T_DELAY_S + (v*v)/(2.0*a); }

/* Decision logic (slides 15 & 17), corrected so it is internally consistent:
 *
 *   d_p > d_s > d_f  are the SF-inflated stopping distances at partial/strong/
 *   full braking (gentler braking needs more room, so d_partial is largest).
 *
 *   Graduated trigger — brake harder as the gap closes:
 *       distance >  d_p          -> NONE      (even gentle braking has room;
 *                                              no need to act yet)
 *       d_s < distance <= d_p    -> PARTIAL
 *       d_f < distance <= d_s    -> STRONG
 *       distance <= d_f          -> FULL
 *
 *   margin = distance - d_f  is the TRUE safety margin: distance beyond the
 *   minimum needed to stop at maximum braking. If it goes negative we cannot
 *   stop even at full braking -> force FULL + EMERGENCY (this is the escalation
 *   the presentation describes, expressed without the band-boundary
 *   contradiction the old loop had).
 *
 *   TTC below threshold overrides the whole ladder -> FULL.
 */
static brake_level_t decide(double distance, double v,
                            double sf, double ttc_thresh,
                            double dp, double ds, double df,
                            double *ttc_out, double *margin_out,
                            aeb_state_t *state_out, bool *emergency_out)
{
    double ttc = (v > 0.01) ? distance / v : 1e9;
    *ttc_out = ttc;
    *emergency_out = false;

    double d_p = sf * stopping_distance(v, dp);
    double d_s = sf * stopping_distance(v, ds);
    double d_f = sf * stopping_distance(v, df);

    /* true safety margin: room beyond a full-braking stop */
    double margin = distance - d_f;
    *margin_out = margin;

    /* 1) critical time-to-collision overrides everything */
    if (ttc < ttc_thresh) {
        *state_out = ST_EMERGENCY; *emergency_out = true; return LVL_FULL;
    }
    /* 2) cannot stop even at full braking -> maximum effort, emergency */
    if (margin < 0.0) {
        *state_out = ST_EMERGENCY; *emergency_out = true; return LVL_FULL;
    }
    /* 3) graduated ladder */
    if (distance > d_p) { *state_out = ST_SAFE;    return LVL_NONE;    }
    if (distance > d_s) { *state_out = ST_WARNING; return LVL_PARTIAL; }
    if (distance > d_f) { *state_out = ST_STRONG;  return LVL_STRONG;  }
    *state_out = ST_EMERGENCY; *emergency_out = true; return LVL_FULL;
}

/* ##########################################################################
 * SECTION 9 — THREADS  (the eight tasks)
 * ######################################################################## */
static void pin_and_prioritise(unsigned runmask, int prio)
{
    struct sched_param sp; sp.sched_priority=prio;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    ThreadCtl(_NTO_TCTL_RUNMASK, (void*)(uintptr_t)runmask);
}

/* ---- 9.1 pwm_task  (prio 57=helper, core0) : 1 kHz software PWM on EN pins ----- */
static void *pwm_task(void *arg)
{
    (void)arg; pin_and_prioritise(CORE0, PRIO_PWM);
    while (g_ctl.running){
        int d[4]; for (int i=0;i<4;i++) d[i]=g_duty[i];
        for (int i=0;i<4;i++) if (d[i]>0) hal_gpio_write(EN_PINS[i],1);
        for (int step=0; step<100; ++step){
            struct timespec s={0,(PWM_PERIOD_US*1000)/100}; nanosleep(&s,NULL);
            for (int i=0;i<4;i++) if (step+1>=d[i]) hal_gpio_write(EN_PINS[i],0);
        }
    }
    for (int i=0;i<4;i++) hal_gpio_write(EN_PINS[i],0);
    return NULL;
}

/* ---- 9.2 radar_task (prio 50=High, core2) : RD-03D read @ 50 Hz → predict ---- */
static void *radar_task(void *arg)
{
    (void)arg; pin_and_prioritise(CORE2, PRIO_RADAR);
    int fd = hal_uart_open(UART_RADAR_DEV, UART_RADAR_BAUD);
    radar_parser_t parser; radar_parser_init(&parser);

    int chid = ChannelCreate(0);
    int coid = ConnectAttach(0,0,chid,_NTO_SIDE_CHANNEL,0);
    struct sigevent ev; SIGEV_PULSE_INIT(&ev,coid,PRIO_RADAR,PULSE_TICK_SENSOR,0);
    timer_t tid; timer_create(CLOCK_MONOTONIC,&ev,&tid);
    struct itimerspec its={ {0,SENSOR_PERIOD_MS*1000000},{0,SENSOR_PERIOD_MS*1000000} };
    timer_settime(tid,0,&its,NULL);

    radar_target_t tgt={ .valid=false, .distance_m=99.9, .closing_mps=0.0 };
    struct _pulse pulse;
    while (g_ctl.running){
        if (MsgReceivePulse(chid,&pulse,sizeof pulse,NULL)!=0) continue;
        if (pulse.code != PULSE_TICK_SENSOR) continue;

        uint8_t b[64]; int n = (fd>=0)? hal_uart_read(fd,b,sizeof b):0;
        for (int i=0;i<n;i++){ radar_target_t g; if (radar_feed(&parser,b[i],&g)&&g.valid) tgt=g; }

        double dist,closing;
        lock();
        if (fd<0 || !tgt.valid){ dist=g_ctl.obstacle_m; closing=g_ctl.speed_mps; }
        else                   { dist=tgt.distance_m;   closing=tgt.closing_mps; }
        unlock();

        sensor_msg_t msg={ .type=MT_SENSOR, .distance_m=dist, .closing_mps=closing,
                           .t_sense_ns=aeb_now_ns() };
        MsgSend(g_coid_predict,&msg,sizeof msg,NULL,0);           /* one in flight */
        MsgSendPulse(g_coid_wdog[HB_RADAR],PRIO_RADAR,PULSE_HEARTBEAT,HB_RADAR);
    }
    if (fd>=0) hal_uart_close(fd);
    return NULL;
}

/* ---- 9.3 imu_task (prio 50=High, core2) : deceleration feedback + safety ---- *
 * The IMU turns the braking system from open-loop into CLOSED-LOOP on the
 * actuator: it measures the deceleration the vehicle actually achieves and
 * cross-checks it against what the controller commanded. Two safety outputs:
 *   1. brake_fault — braking was commanded while the vehicle was moving, yet
 *      the measured deceleration stayed far below the commanded value for
 *      longer than BRAKE_FAULT_MS. That means the brakes are not biting.
 *   2. impact — a deceleration spike above IMPACT_G, i.e. a collision event.
 * The effectiveness check runs only with a real IMU and while actually moving,
 * so a bench IMU sitting on a desk never raises a false fault.              */
static void *imu_task(void *arg)
{
    (void)arg; pin_and_prioritise(CORE2, PRIO_IMU);
    int fd = hal_imu_open(I2C_DEV);
    lock(); g_ctl.imu_ok = (fd >= 0); unlock();

    struct timespec p = { 0, IMU_PERIOD_MS * 1000000 };
    uint64_t fault_since = 0;   /* when the commanded/measured mismatch began */

    while (g_ctl.running){
        double ax=0, ay=0, az=0, decel=0, mag_g=0;
        bool have = (fd >= 0) && (hal_imu_read_accel(fd, &ax, &ay, &az) == 0);

        if (have){
            /* longitudinal deceleration: forward = +X, braking pushes -X.
             * (Flip the sign here if your IMU is mounted the other way.)     */
            decel = -ax; if (decel < 0) decel = 0;
            mag_g = sqrt(ax*ax + ay*ay + az*az) / G_MPS2;
        }

        lock();
        double cmd   = g_ctl.commanded_decel;
        double speed = g_ctl.speed_mps;
        bool   imu_ok= g_ctl.imu_ok;
        if (have){ g_ctl.measured_decel = decel; g_ctl.accel_mag_g = mag_g; }
        unlock();

        /* impact detection */
        if (have && mag_g > IMPACT_G){
            lock(); g_ctl.impact = true; unlock();
            log_evt_t e={ aeb_now_ns(), ST_EMERGENCY, LVL_FULL, 0,0,0,0 };
            log_push(&e); timeline_push(&e);
        }

        /* brake-effectiveness monitor (only with a real IMU, and moving) */
        if (have && imu_ok && cmd > 0.1 && speed > IMU_MOVING_MPS){
            if (decel < BRAKE_FAULT_FRAC * cmd){
                uint64_t now = aeb_now_ns();
                if (fault_since == 0) fault_since = now;
                else if (now - fault_since > (uint64_t)BRAKE_FAULT_MS*1000000ull){
                    lock(); g_ctl.brake_fault = true; unlock();
                    /* brakes not biting → force maximum braking + emergency  */
                    MsgSendPulse(g_coid_brake_em, PRIO_EMERGENCY_PULSE, PULSE_EMERGENCY, 0);
                }
            } else {
                fault_since = 0;
                lock(); g_ctl.brake_fault = false; unlock();
            }
        } else {
            fault_since = 0;
        }

        MsgSendPulse(g_coid_wdog[HB_IMU], PRIO_IMU, PULSE_HEARTBEAT, HB_IMU);
        nanosleep(&p, NULL);
    }
    hal_imu_close(fd);
    return NULL;
}

/* ---- 9.4 predict_task (prio 58=Highest, core1) : model + decision ladder -------- */
static void *predict_task(void *arg)
{
    (void)arg; pin_and_prioritise(CORE1, PRIO_PREDICT);
    sensor_msg_t in;
    while (g_ctl.running){
        int rcvid = MsgReceive(g_chid_predict,&in,sizeof in,NULL);
        if (rcvid<=0) continue;
        if (in.type!=MT_SENSOR){ MsgReply(rcvid,0,NULL,0); continue; }

        lock();
        double sf=g_ctl.safety_factor, tt=g_ctl.ttc_threshold;
        double dp=g_ctl.decel_partial, ds=g_ctl.decel_strong, df=g_ctl.decel_full;
        unlock();

        double v=in.closing_mps, ttc,margin; aeb_state_t st; bool em;
        brake_level_t lvl = decide(in.distance_m,v,sf,tt,dp,ds,df,&ttc,&margin,&st,&em);

        MsgReply(rcvid,0,NULL,0);                 /* release radar first       */

        brake_cmd_t cmd={ .type=MT_BRAKE, .level=lvl, .state=st,
            .decel=(lvl==LVL_PARTIAL)?dp:(lvl==LVL_STRONG)?ds:(lvl==LVL_FULL)?df:0.0,
            .ttc_s=ttc, .margin_m=margin, .distance_m=in.distance_m,
            .t_sense_ns=in.t_sense_ns, .emergency=em };
        MsgSend(g_coid_brake,&cmd,sizeof cmd,NULL,0);
        MsgSendPulse(g_coid_wdog[HB_PREDICT],PRIO_PREDICT,PULSE_HEARTBEAT,HB_PREDICT);
    }
    return NULL;
}

/* ---- 9.5 brake_task (prio 58=Highest, core0) : drives both L298N + override ----- */
static void *brake_task(void *arg)
{
    (void)arg; pin_and_prioritise(CORE0, PRIO_BRAKE);
    union { struct _pulse pulse; brake_cmd_t cmd; } m;
    while (g_ctl.running){
        int rcvid = MsgReceive(g_chid_brake,&m,sizeof m,NULL);
        if (rcvid==0){                            /* a pulse                   */
            if (m.pulse.code==PULSE_EMERGENCY){
                apply_level(LVL_FULL);
                lock(); g_ctl.state=ST_EMERGENCY; g_ctl.level=LVL_FULL;
                        g_ctl.commanded_decel=g_ctl.decel_full; unlock();
                log_evt_t e={ aeb_now_ns(), ST_EMERGENCY, LVL_FULL, 0,0,0,0 };
                log_push(&e);
            }
            continue;
        }
        if (rcvid<0) continue;
        if (m.cmd.type!=MT_BRAKE){ MsgReply(rcvid,0,NULL,0); continue; }

        apply_level(m.cmd.level);
        uint64_t now=aeb_now_ns(), lat=now-m.cmd.t_sense_ns;
        lock(); g_ctl.state=m.cmd.state; g_ctl.level=m.cmd.level;
                g_ctl.commanded_decel=m.cmd.decel; unlock();
        MsgReply(rcvid,0,NULL,0);

        lat_push(lat);
        log_evt_t e={ .t_ns=now, .state=m.cmd.state, .level=m.cmd.level,
                      .distance_m=m.cmd.distance_m, .ttc_s=m.cmd.ttc_s,
                      .margin_m=m.cmd.margin_m, .latency_ns=lat };
        log_push(&e);
        timeline_push(&e);   /* non-consumed history for the 't' command      */
        MsgSendPulse(g_coid_wdog[HB_BRAKE],PRIO_BRAKE,PULSE_HEARTBEAT,HB_BRAKE);
    }
    apply_level(LVL_NONE);
    return NULL;
}

/* ---- 9.6 vehicle_task (prio 30=Medium, core2) : sim loop + interlock @ 100 Hz --- */
static void *vehicle_task(void *arg)
{
    (void)arg; pin_and_prioritise(CORE2, PRIO_VEHICLE);
    struct timespec p={0,10*1000000}; uint64_t last=aeb_now_ns();
    while (g_ctl.running){
        nanosleep(&p,NULL);
        uint64_t now=aeb_now_ns(); double dt=(now-last)/1e9; last=now;

        /* Master-switch interlock sense (GPIO24): reads whether the DPDT
         * switch is armed. Read outside the mutex, then publish under it.   */
        int armed = hal_gpio_read(INTERLOCK_SENSE);

        lock();
        g_ctl.interlock_ok = (armed != 0);
        double a=g_ctl.commanded_decel;
        g_ctl.speed_mps -= a*dt; if (g_ctl.speed_mps<0) g_ctl.speed_mps=0;
        g_ctl.obstacle_m -= g_ctl.speed_mps*dt; if (g_ctl.obstacle_m<0) g_ctl.obstacle_m=0;
        unlock();
        MsgSendPulse(g_coid_wdog[HB_VEHICLE],PRIO_VEHICLE,PULSE_HEARTBEAT,HB_VEHICLE);
    }
    return NULL;
}

/* ---- 9.7 watchdog_task (prio 62=supervisor, core3) : liveness → emergency pulse ---- */
static void *watchdog_task(void *arg)
{
    (void)arg; pin_and_prioritise(CORE3, PRIO_WATCHDOG);
    struct sigevent ev;
    int coid_self = ConnectAttach(0,0,g_chid_wdog,_NTO_SIDE_CHANNEL,0);
    SIGEV_PULSE_INIT(&ev,coid_self,PRIO_WATCHDOG,PULSE_TICK_WDOG,0);
    timer_t tid; timer_create(CLOCK_MONOTONIC,&ev,&tid);
    struct itimerspec its={ {0,WATCHDOG_PERIOD_MS*1000000},{0,WATCHDOG_PERIOD_MS*1000000} };
    timer_settime(tid,0,&its,NULL);

    uint64_t last_hb[HB_COUNT], now0=aeb_now_ns();
    for (int i=0;i<HB_COUNT;i++) last_hb[i]=now0;
    const uint64_t timeout=(uint64_t)WATCHDOG_TIMEOUT_MS*1000000ull;

    struct _pulse pulse;
    while (g_ctl.running){
        if (MsgReceivePulse(g_chid_wdog,&pulse,sizeof pulse,NULL)!=0) continue;
        if (pulse.code==PULSE_HEARTBEAT){
            int id=pulse.value.sival_int; if (id>=0&&id<HB_COUNT) last_hb[id]=aeb_now_ns();
        } else if (pulse.code==PULSE_TICK_WDOG){
            uint64_t now=aeb_now_ns();
            const int crit[]={HB_RADAR,HB_PREDICT,HB_BRAKE};
            for (unsigned k=0;k<sizeof crit/sizeof crit[0];k++){
                if (now-last_hb[crit[k]] > timeout){
                    MsgSendPulse(g_coid_brake_em,PRIO_EMERGENCY_PULSE,PULSE_EMERGENCY,0);
                    last_hb[crit[k]]=now;      /* debounce                    */
                }
            }
        }
    }
    return NULL;
}

/* ---- 9.8 logger_task (prio 10=Low, core2) : drain ring → stdout + CSV ------- */
static void *logger_task(void *arg)
{
    (void)arg; pin_and_prioritise(CORE2, PRIO_LOGGER);
    FILE *f=fopen("aeb_log.csv","w");
    if (f) fprintf(f,"t_ns,state,level,distance_m,ttc_s,margin_m,latency_ns\n");
    const char *ST[]={"SAFE","WARNING","STRONG","EMERGENCY"};
    const char *LV[]={"NONE","PARTIAL","STRONG","FULL"};
    struct timespec p={0,20*1000000};
    while (g_ctl.running){
        pthread_mutex_lock(&s_ring_lock);
        while (s_tail!=s_head){
            log_evt_t e=s_ring[s_tail]; s_tail=(s_tail+1)%LOG_RING;
            pthread_mutex_unlock(&s_ring_lock);
            if (f){ fprintf(f,"%llu,%s,%s,%.3f,%.3f,%.3f,%llu\n",
                    (unsigned long long)e.t_ns,ST[e.state],LV[e.level],
                    e.distance_m,e.ttc_s,e.margin_m,(unsigned long long)e.latency_ns);
                    fflush(f); }
            pthread_mutex_lock(&s_ring_lock);
        }
        pthread_mutex_unlock(&s_ring_lock);
        nanosleep(&p,NULL);
    }
    if (f) fclose(f);
    return NULL;
}

/* ##########################################################################
 * SECTION 10 — BRING-UP + CLI + main()
 * ######################################################################## */
static pthread_t th[8];

static void ctl_init(void)
{
    memset(&g_ctl,0,sizeof g_ctl);
    pthread_mutex_init(&g_ctl.lock,NULL);
    g_ctl.speed_mps=0.0; g_ctl.obstacle_m=10.0;
    g_ctl.safety_factor=SAFETY_FACTOR; g_ctl.ttc_threshold=TTC_CRITICAL_S;
    g_ctl.decel_partial=DECEL_PARTIAL; g_ctl.decel_strong=DECEL_STRONG; g_ctl.decel_full=DECEL_FULL;
    g_ctl.state=ST_SAFE; g_ctl.level=LVL_NONE; g_ctl.commanded_decel=0;
    g_ctl.interlock_ok=false; g_ctl.running=true;
}
static void channels_init(void)
{
    g_chid_predict=ChannelCreate(0); g_chid_brake=ChannelCreate(0); g_chid_wdog=ChannelCreate(0);
    g_coid_predict =ConnectAttach(0,0,g_chid_predict,_NTO_SIDE_CHANNEL,0);
    g_coid_brake   =ConnectAttach(0,0,g_chid_brake,  _NTO_SIDE_CHANNEL,0);
    g_coid_brake_em=ConnectAttach(0,0,g_chid_brake,  _NTO_SIDE_CHANNEL,0);
    for (int i=0;i<HB_COUNT;i++) g_coid_wdog[i]=ConnectAttach(0,0,g_chid_wdog,_NTO_SIDE_CHANNEL,0);
}
static pthread_t spawn(void*(*fn)(void*))
{
    pthread_t t; pthread_attr_t a; pthread_attr_init(&a);
    pthread_attr_setinheritsched(&a,PTHREAD_EXPLICIT_SCHED);
    pthread_create(&t,&a,fn,NULL); pthread_attr_destroy(&a); return t;
}
static void spawn_all(void)
{
    th[0]=spawn(watchdog_task); th[1]=spawn(brake_task); th[2]=spawn(pwm_task);
    th[3]=spawn(predict_task);  th[4]=spawn(radar_task); th[5]=spawn(imu_task);
    th[6]=spawn(vehicle_task);  th[7]=spawn(logger_task);
}
static void cli_help(void)
{
    puts("\nAEB CLI — Team PRIORITY ZERO\n"
         "  v <mps>    set vehicle speed\n"
         "  o <m>      set obstacle distance (bench)\n"
         "  sf <x>     set safety factor\n"
         "  ttc <s>    set critical TTC threshold\n"
         "  a <p s f>  set decel levels partial/strong/full (m/s^2)\n"
         "  e          run a worked braking example\n"
         "  s          status\n  t  timeline\n  l  latency stats\n  i  IMU reading\n"
         "  h          help\n  q  quit (motors safe)\n");
}
static void cli_status(void)
{
    const char *ST[]={"SAFE","WARNING","STRONG","EMERGENCY"};
    const char *LV[]={"NONE","PARTIAL","STRONG","FULL"};
    lock();
    printf("speed=%.2f m/s obstacle=%.2f m state=%s level=%s cmd_decel=%.1f\n"
           "SF=%.2f TTC*=%.2fs a[p/s/f]=%.1f/%.1f/%.1f interlock=%s\n"
           "IMU=%s measured_decel=%.2f m/s^2 brake_fault=%s impact=%s\n",
        g_ctl.speed_mps,g_ctl.obstacle_m,ST[g_ctl.state],LV[g_ctl.level],
        g_ctl.commanded_decel,g_ctl.safety_factor,g_ctl.ttc_threshold,
        g_ctl.decel_partial,g_ctl.decel_strong,g_ctl.decel_full,
        g_ctl.interlock_ok?"ARMED":"open",
        g_ctl.imu_ok?"ok":"absent", g_ctl.measured_decel,
        g_ctl.brake_fault?"YES":"no", g_ctl.impact?"YES":"no");
    unlock();
}
static void cli_imu(void)
{
    lock();
    printf("IMU=%s  measured decel=%.2f m/s^2  |accel|=%.2f g  "
           "commanded=%.2f m/s^2  brake_fault=%s  impact=%s\n",
        g_ctl.imu_ok?"present":"absent", g_ctl.measured_decel, g_ctl.accel_mag_g,
        g_ctl.commanded_decel, g_ctl.brake_fault?"YES":"no", g_ctl.impact?"YES":"no");
    unlock();
}
static void cli_latency(void)
{
    uint64_t mn,mx; double avg; uint32_t miss,cnt; lat_stats(&mn,&mx,&avg,&miss,&cnt);
    printf("latency n=%u min=%.3fms max=%.3fms avg=%.3fms misses=%u (deadline=%.1fms)\n",
        cnt,mn/1e6,mx/1e6,avg/1e6,miss,BRAKING_DEADLINE_US/1000.0);
}

int main(void)
{
    printf("== AEB Controller (QNX) — Team PRIORITY ZERO ==\n");
    ctl_init();
    if (hal_gpio_init()!=0)
        fprintf(stderr,"WARN: GPIO init failed — logic-only (no motor output)\n");
    else
        motors_init_pins();
    channels_init();
    spawn_all();
    cli_help();

    char line[128];
    while (g_ctl.running && fgets(line,sizeof line,stdin)){
        char cmd[16]={0}; double x=0,y=0,z=0;
        if (sscanf(line,"%15s %lf %lf %lf",cmd,&x,&y,&z)<1) continue;
        if      (!strcmp(cmd,"q")) g_ctl.running=false;
        else if (!strcmp(cmd,"h")) cli_help();
        else if (!strcmp(cmd,"s")) cli_status();
        else if (!strcmp(cmd,"l")) cli_latency();
        else if (!strcmp(cmd,"i")) cli_imu();
        else if (!strcmp(cmd,"t")) timeline_print();
        else if (!strcmp(cmd,"v")) { lock(); g_ctl.speed_mps=x;    unlock(); }
        else if (!strcmp(cmd,"o")) { lock(); g_ctl.obstacle_m=x;   unlock(); }
        else if (!strcmp(cmd,"sf")){ lock(); g_ctl.safety_factor=x;unlock(); }
        else if (!strcmp(cmd,"ttc")){lock();g_ctl.ttc_threshold=x; unlock(); }
        else if (!strcmp(cmd,"a")) { lock(); g_ctl.decel_partial=x; g_ctl.decel_strong=y; g_ctl.decel_full=z; unlock(); }
        else if (!strcmp(cmd,"e")) { lock(); g_ctl.speed_mps=10; g_ctl.obstacle_m=8; unlock();
                                     puts("example: v=10 m/s, obstacle=8 m — watch 's' and 'l'"); }
        else puts("? unknown (h for help)");
    }

    g_ctl.running=false;
    for (int i=0;i<8;i++) pthread_join(th[i],NULL);
    for (int i=0;i<4;i++) hal_gpio_write(EN_PINS[i],0);   /* motors safe       */
    hal_gpio_fini();
    puts("stopped. motors safe.");
    return 0;
}
