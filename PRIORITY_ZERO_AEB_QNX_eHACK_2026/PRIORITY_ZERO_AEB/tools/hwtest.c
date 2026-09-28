/* ==========================================================================
 *  hwtest.c  —  COMPONENT BRING-UP TESTER
 *  Team PRIORITY ZERO · AEB on Raspberry Pi 5 · QNX Neutrino RTOS
 *
 *  Tests every component ONE AT A TIME so you can find exactly which part
 *  of the hardware is wrong, instead of debugging the whole system at once.
 *
 *  BUILD
 *      source ~/qnx800/qnxsdp-env.sh
 *      qcc -Vgcc_ntoaarch64le -O2 -Wall -std=gnu11 -D_QNX_SOURCE \
 *          hwtest.c -o hwtest -lm
 *      # copy ./hwtest to the target, then:  ./hwtest
 *
 *  RUN THE TESTS IN THIS ORDER — each one assumes the previous passed:
 *      0  discover devices       (what does this image actually publish?)
 *      1  GPIO output toggle     (no motor power needed)
 *      2  interlock input        (GPIO24 pull-down)
 *      3  one motor at a time    (motor power ON)
 *      4  PWM duty sweep         (speed control)
 *      5  all four motors        (direction agreement)
 *      6  UART raw bytes         (is the radar talking at all?)
 *      7  radar frame decode     (is it talking correctly?)
 *      8  I2C bus scan           (is the IMU on the bus?)
 *      9  IMU live accelerometer (is it oriented correctly?)
 * ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <dirent.h>
#include <termios.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>
#include <hw/inout.h>
#include <hw/i2c.h>

/* ---------------- pin map : identical to aeb.c ---------------- */
enum {
    M1_ENA = 12, M1_IN1 = 5,  M1_IN2 = 6,
    M1_ENB = 13, M1_IN3 = 16, M1_IN4 = 26,
    M2_ENA = 18, M2_IN1 = 17, M2_IN2 = 27,
    M2_ENB = 19, M2_IN3 = 22, M2_IN4 = 23,
    INTERLOCK_SENSE = 24
};
static const int EN_PINS[4]    = { M1_ENA, M1_ENB, M2_ENA, M2_ENB };
static const int IN_PINS[4][2] = { {M1_IN1,M1_IN2}, {M1_IN3,M1_IN4},
                                   {M2_IN1,M2_IN2}, {M2_IN3,M2_IN4} };
static const char *WHEEL[4] = { "M1 front-left", "M2 front-right",
                                "M3 rear-left",  "M4 rear-right" };

#define MPU6050_ADDR   0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_WHO_AM_I   0x75
#define MPU_ACCEL_XOUT 0x3B
#define MPU_LSB_PER_G  16384.0
#define G_MPS2         9.80665

/* ***RP1*** Pi 5 routes GPIO through RP1, not the classic BCM window.
 * This base is correct on Pi 4-class BSPs. If test 1 shows no pin activity
 * on a Pi 5, this is the line to change to your BSP's GPIO window.        */
#define GPIO_BASE 0xFE200000ul
#define GPIO_LEN  0xB4
#define GPFSEL0 (0x00/4)
#define GPSET0  (0x1C/4)
#define GPCLR0  (0x28/4)
#define GPLEV0  (0x34/4)

static volatile uint32_t *g_gpio = NULL;

static int gpio_init(void)
{
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1){ perror("ThreadCtl(_NTO_TCTL_IO)"); return -1; }
    g_gpio = (volatile uint32_t *)mmap_device_memory(
                NULL, GPIO_LEN, PROT_READ|PROT_WRITE|PROT_NOCACHE, 0, GPIO_BASE);
    if (g_gpio == MAP_FAILED){ perror("mmap_device_memory"); g_gpio=NULL; return -1; }
    return 0;
}
static void gpio_mode(int g, int out)
{
    if (!g_gpio) return;
    int reg = GPFSEL0 + g/10, sh = (g%10)*3;
    uint32_t v = g_gpio[reg]; v &= ~(0x7u<<sh);
    if (out) v |= (0x1u<<sh);
    g_gpio[reg] = v;
}
static void gpio_write(int g, int v)
{
    if (!g_gpio) return;
    if (v) g_gpio[GPSET0 + g/32] = (1u<<(g%32));
    else   g_gpio[GPCLR0 + g/32] = (1u<<(g%32));
}
static int gpio_read(int g)
{
    if (!g_gpio) return -1;
    return (g_gpio[GPLEV0 + g/32] >> (g%32)) & 1u;
}

static void msleep(int ms){ struct timespec t={ms/1000,(ms%1000)*1000000L}; nanosleep(&t,NULL); }
static void pause_enter(void){ printf("\n   [press ENTER to continue] "); fflush(stdout);
                               int c; while((c=getchar())!='\n' && c!=EOF){} }

static void all_motors_off(void)
{
    for (int w=0; w<4; w++){ gpio_write(IN_PINS[w][0],0); gpio_write(IN_PINS[w][1],0); }
    for (int i=0; i<4; i++)  gpio_write(EN_PINS[i],0);
}

static void pins_init(void)
{
    for (int w=0; w<4; w++){ gpio_mode(IN_PINS[w][0],1); gpio_mode(IN_PINS[w][1],1); }
    for (int i=0; i<4; i++)  gpio_mode(EN_PINS[i],1);
    gpio_mode(INTERLOCK_SENSE, 0);
    all_motors_off();
}

/* Software PWM burst: hold `duty`% for `ms` milliseconds on one EN pin. */
static void pwm_burst(int en, int duty, int ms)
{
    int periods = ms;                       /* 1 kHz => 1 period per ms */
    for (int p=0; p<periods; p++){
        if (duty > 0)  gpio_write(en,1);
        msleep(0);                          /* yield */
        for (int step=0; step<100; step++){
            struct timespec t={0,10000};    /* 10 us slice */
            nanosleep(&t,NULL);
            if (step+1 >= duty) gpio_write(en,0);
        }
    }
    gpio_write(en,0);
}

/* ====================================================================== */
/* TEST 0 — what does this image actually publish?                        */
/* ====================================================================== */
static void t0_discover(void)
{
    puts("\n=== TEST 0 : DEVICE DISCOVERY ===");
    puts("Looking for serial and I2C devices this QNX image provides.\n");
    DIR *d = opendir("/dev");
    if (!d){ perror("opendir /dev"); return; }
    struct dirent *e;
    int nser=0, ni2c=0;
    puts("   serial devices:");
    while ((e = readdir(d))){
        if (!strncmp(e->d_name,"ser",3)){ printf("      /dev/%s\n", e->d_name); nser++; }
    }
    if (!nser) puts("      (none)  -> serial driver is NOT running");
    rewinddir(d);
    puts("   i2c devices:");
    while ((e = readdir(d))){
        if (!strncmp(e->d_name,"i2c",3)){ printf("      /dev/%s\n", e->d_name); ni2c++; }
    }
    if (!ni2c) puts("      (none)  -> I2C driver is NOT running");
    closedir(d);

    puts("\n   If a list is empty, start the driver on the target, e.g.");
    puts("      serial :  devc-serminiuart   (or devc-serpl011)");
    puts("      i2c    :  i2c-bcm2711        (name varies by BSP)");
    puts("   then re-run this test. Check running drivers with:  pidin ar");
}

/* ====================================================================== */
/* TEST 1 — GPIO output toggle (NO motor power needed)                    */
/* ====================================================================== */
static void t1_gpio(void)
{
    puts("\n=== TEST 1 : GPIO OUTPUT TOGGLE ===");
    puts("Motor power should be OFF. Probe each pin with a multimeter or an");
    puts("LED+330R to GND. Each pin is driven HIGH for 1 s, then LOW.\n");
    struct { int pin; const char *name; } pins[] = {
        {M1_ENA,"L298N#1 ENA (GPIO12, phys 32)"},
        {M1_IN1,"L298N#1 IN1 (GPIO5,  phys 29)"},
        {M1_IN2,"L298N#1 IN2 (GPIO6,  phys 31)"},
        {M1_ENB,"L298N#1 ENB (GPIO13, phys 33)"},
        {M1_IN3,"L298N#1 IN3 (GPIO16, phys 36)"},
        {M1_IN4,"L298N#1 IN4 (GPIO26, phys 37)"},
        {M2_ENA,"L298N#2 ENA (GPIO18, phys 12)"},
        {M2_IN1,"L298N#2 IN1 (GPIO17, phys 11)"},
        {M2_IN2,"L298N#2 IN2 (GPIO27, phys 13)"},
        {M2_ENB,"L298N#2 ENB (GPIO19, phys 35)"},
        {M2_IN3,"L298N#2 IN3 (GPIO22, phys 15)"},
        {M2_IN4,"L298N#2 IN4 (GPIO23, phys 16)"},
    };
    for (unsigned i=0;i<sizeof pins/sizeof pins[0];i++){
        printf("   %-34s HIGH ... ", pins[i].name); fflush(stdout);
        gpio_write(pins[i].pin,1); msleep(1000);
        gpio_write(pins[i].pin,0);
        printf("LOW\n");
    }
    puts("\n   PASS if every pin measured ~3.3 V during its HIGH second.");
    puts("   If ALL pins stayed at 0 V, the GPIO base address is wrong");
    puts("   for this board (see the ***RP1*** note at the top of this file).");
}

/* ====================================================================== */
/* TEST 2 — interlock input                                               */
/* ====================================================================== */
static void t2_interlock(void)
{
    puts("\n=== TEST 2 : INTERLOCK INPUT (GPIO24, phys 18) ===");
    puts("Reading for 10 s. With the 22k pull-down fitted and an SPST switch,");
    puts("this should read LOW the whole time (that is correct and expected).");
    puts("If you fitted a DPDT, flip it now and watch the value change.\n");
    for (int i=0;i<20;i++){
        int v = gpio_read(INTERLOCK_SENSE);
        printf("\r   GPIO24 = %d   (%s)      ", v, v ? "HIGH / armed" : "LOW / open");
        fflush(stdout);
        msleep(500);
    }
    puts("\n\n   PASS if the value is a steady 0 (not flickering).");
    puts("   Flickering means the pull-down resistor is missing.");
}

/* ====================================================================== */
/* TEST 3 — one motor at a time                                           */
/* ====================================================================== */
static void t3_one_motor(void)
{
    puts("\n=== TEST 3 : ONE MOTOR AT A TIME ===");
    puts("*** TURN MOTOR POWER ON (master switch to I) ***");
    puts("Wheels should be off the ground. Each wheel runs forward 2 s,");
    puts("reverse 2 s, then brakes.\n");
    pause_enter();
    for (int w=0; w<4; w++){
        printf("   %-16s FORWARD ... ", WHEEL[w]); fflush(stdout);
        gpio_write(IN_PINS[w][0],1); gpio_write(IN_PINS[w][1],0);
        gpio_write(EN_PINS[w],1); msleep(2000);
        printf("REVERSE ... "); fflush(stdout);
        gpio_write(IN_PINS[w][0],0); gpio_write(IN_PINS[w][1],1);
        msleep(2000);
        gpio_write(IN_PINS[w][0],0); gpio_write(IN_PINS[w][1],0);
        gpio_write(EN_PINS[w],0);
        printf("BRAKE\n");
        msleep(600);
    }
    all_motors_off();
    puts("\n   Note which wheels ran BACKWARDS during the FORWARD phase.");
    puts("   Fix: swap that motor's two OUT wires on its L298N. No code change.");
    puts("   (Left and right motors face opposite ways, so one side normally");
    puts("    needs swapping — that is expected, not a fault.)");
}

/* ====================================================================== */
/* TEST 4 — PWM duty sweep                                                */
/* ====================================================================== */
static void t4_pwm(void)
{
    puts("\n=== TEST 4 : PWM DUTY SWEEP ===");
    puts("Each wheel runs forward at 25%, 50%, 75%, 100% duty for 1.5 s each.");
    puts("You should HEAR and SEE the speed increase in steps.\n");
    puts("If speed does not change, the ENA/ENB jumper caps are still fitted.");
    pause_enter();
    int duties[] = {25,50,75,100};
    for (int w=0; w<4; w++){
        printf("   %s: ", WHEEL[w]); fflush(stdout);
        gpio_write(IN_PINS[w][0],1); gpio_write(IN_PINS[w][1],0);
        for (int d=0; d<4; d++){
            printf("%d%% ", duties[d]); fflush(stdout);
            pwm_burst(EN_PINS[w], duties[d], 1500);
        }
        gpio_write(IN_PINS[w][0],0); gpio_write(IN_PINS[w][1],0);
        printf("\n");
    }
    all_motors_off();
    puts("\n   PASS if each wheel visibly sped up across the four steps.");
}

/* ====================================================================== */
/* TEST 5 — all four motors together                                      */
/* ====================================================================== */
static void t5_all_motors(void)
{
    puts("\n=== TEST 5 : ALL FOUR MOTORS ===");
    puts("All wheels forward 3 s, then a short brake (both IN pins LOW).");
    puts("This is exactly what apply_level(LVL_NONE) then LVL_FULL does.\n");
    pause_enter();
    printf("   FORWARD (cruise, 60%% duty) ... "); fflush(stdout);
    for (int w=0; w<4; w++){ gpio_write(IN_PINS[w][0],1); gpio_write(IN_PINS[w][1],0); }
    for (int i=0;i<4;i++) gpio_write(EN_PINS[i],1);
    msleep(3000);
    printf("FULL BRAKE\n");
    for (int w=0; w<4; w++){ gpio_write(IN_PINS[w][0],0); gpio_write(IN_PINS[w][1],0); }
    msleep(1500);
    all_motors_off();
    puts("\n   PASS if all four wheels turned the SAME way, then stopped sharply.");
    puts("   If the chassis tried to rotate, one side is still wired backwards.");
}

/* ====================================================================== */
/* TEST 6 — UART raw bytes                                                */
/* ====================================================================== */
static int uart_open_any(char *found, size_t n)
{
    const char *cands[] = {"/dev/ser1","/dev/ser2","/dev/ser3","/dev/ser4",NULL};
    for (int i=0;cands[i];i++){
        int fd = open(cands[i], O_RDWR|O_NOCTTY);
        if (fd >= 0){ snprintf(found,n,"%s",cands[i]); return fd; }
    }
    return -1;
}

static void t6_uart_raw(void)
{
    puts("\n=== TEST 6 : UART RAW BYTES (radar) ===");
    char dev[32] = "";
    int fd = uart_open_any(dev, sizeof dev);
    if (fd < 0){
        printf("   FAIL: no serial device opened (%s)\n", strerror(errno));
        puts("   Run TEST 0. If no /dev/ser* exists the serial driver is not");
        puts("   running — start it, e.g. devc-serminiuart, then retry.");
        return;
    }
    printf("   opened %s at 256000 baud\n", dev);
    struct termios tio;
    if (tcgetattr(fd,&tio)==0){
        cfmakeraw(&tio);
        tio.c_cflag |= (CLOCAL|CREAD);
        tio.c_cc[VMIN]=0; tio.c_cc[VTIME]=5;     /* 0.5 s read timeout */
        if (cfsetispeed(&tio,256000)!=0 || cfsetospeed(&tio,256000)!=0)
            puts("   WARN: 256000 baud not accepted by this driver");
        tcsetattr(fd,TCSANOW,&tio);
        tcflush(fd,TCIOFLUSH);
    }
    puts("   Reading for 3 s — you should see a repeating pattern starting AA FF 03 00\n");
    uint8_t b[64];
    int total=0, aa=0;
    for (int i=0;i<6;i++){
        int n = read(fd,b,sizeof b);
        if (n > 0){
            total += n;
            printf("   ");
            for (int k=0;k<n && k<24;k++){ printf("%02X ", b[k]); if(b[k]==0xAA) aa++; }
            printf("%s\n", n>24 ? "..." : "");
        } else {
            puts("   (no bytes)");
        }
    }
    close(fd);
    printf("\n   %d bytes read, %d x 0xAA header bytes seen.\n", total, aa);
    if (total == 0){
        puts("   FAIL: nothing received. Check:");
        puts("     - radar VCC on Pi 5V (phys 2/4), GND to star ground");
        puts("     - radar TX -> Pi GPIO15 (phys 10), radar RX -> Pi GPIO14 (phys 8)");
        puts("     - TX/RX are CROSSED, not straight through");
    } else if (aa == 0){
        puts("   Bytes arriving but no 0xAA header -> baud rate mismatch.");
    } else {
        puts("   PASS: radar is transmitting frames. Run TEST 7 to decode them.");
    }
}

/* ====================================================================== */
/* TEST 7 — radar frame decode                                            */
/* ====================================================================== */
static int16_t rd03d_signed(uint8_t lo, uint8_t hi)
{
    uint16_t raw = (uint16_t)lo | ((uint16_t)hi<<8);
    int16_t mag = (int16_t)(raw & 0x7FFF);
    return (raw & 0x8000) ? mag : (int16_t)(-mag);
}

static void t7_radar_decode(void)
{
    puts("\n=== TEST 7 : RADAR FRAME DECODE ===");
    char dev[32]="";
    int fd = uart_open_any(dev,sizeof dev);
    if (fd < 0){ puts("   FAIL: no serial device (run TEST 6 first)"); return; }
    struct termios tio;
    if (tcgetattr(fd,&tio)==0){
        cfmakeraw(&tio); tio.c_cflag |= (CLOCAL|CREAD);
        tio.c_cc[VMIN]=0; tio.c_cc[VTIME]=5;
        cfsetispeed(&tio,256000); cfsetospeed(&tio,256000);
        tcsetattr(fd,TCSANOW,&tio); tcflush(fd,TCIOFLUSH);
    }
    puts("   Move your hand toward and away from the radar for 10 s.\n");
    uint8_t buf[128]; int len=0, frames=0;
    for (int iter=0; iter<20; iter++){
        uint8_t b[64];
        int n = read(fd,b,sizeof b);
        for (int i=0;i<n;i++){
            if (len < (int)sizeof buf) buf[len++] = b[i];
        }
        for (int i=0; i+30 <= len; i++){
            if (buf[i]==0xAA&&buf[i+1]==0xFF&&buf[i+2]==0x03&&buf[i+3]==0x00
                && buf[i+28]==0x55 && buf[i+29]==0xCC){
                const uint8_t *t = buf+i+4;
                int16_t x = rd03d_signed(t[0],t[1]);
                int16_t y = rd03d_signed(t[2],t[3]);
                int16_t v = rd03d_signed(t[4],t[5]);
                if (x||y||v){
                    double r = sqrt((double)x*x + (double)y*y)/1000.0;
                    printf("   target: range %6.2f m   closing %6.2f m/s\n",
                           r, v/100.0);
                    frames++;
                }
                int consumed = i+30, rem = len-consumed;
                memmove(buf, buf+consumed, rem); len = rem; i = -1;
            }
        }
        if (len > 96){ memmove(buf, buf+len-29, 29); len = 29; }
    }
    close(fd);
    printf("\n   %d target frames decoded.\n", frames);
    puts(frames ? "   PASS: radar range/closing speed are being decoded."
                : "   No targets decoded. If TEST 6 passed, the frame format\n"
                  "   differs on your firmware — adjust the parser in aeb.c.");
}

/* ====================================================================== */
/* TEST 8 — I2C bus scan                                                  */
/* ====================================================================== */
static int i2c_open_any(char *found, size_t n)
{
    const char *cands[] = {"/dev/i2c1","/dev/i2c0","/dev/i2c2","/dev/i2c-1",NULL};
    for (int i=0;cands[i];i++){
        int fd = open(cands[i], O_RDWR);
        if (fd >= 0){ snprintf(found,n,"%s",cands[i]); return fd; }
    }
    return -1;
}

static int i2c_read_reg(int fd, uint8_t addr, uint8_t reg, uint8_t *out, int n)
{
    struct { i2c_sendrecv_t hdr; uint8_t data[16]; } m;
    memset(&m,0,sizeof m);
    m.hdr.slave.addr = addr; m.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    m.hdr.send_len = 1; m.hdr.recv_len = n; m.hdr.stop = 1;
    m.data[0] = reg;
    int rc = devctl(fd, DCMD_I2C_SENDRECV, &m, sizeof m, NULL);
    if (rc != EOK) return rc;               /* devctl RETURNS the error code */
    memcpy(out, m.data, n);
    return EOK;
}

static void t8_i2c_scan(void)
{
    puts("\n=== TEST 8 : I2C BUS SCAN ===");
    char dev[32]="";
    int fd = i2c_open_any(dev,sizeof dev);
    if (fd < 0){
        printf("   FAIL: no I2C device opened (%s)\n", strerror(errno));
        puts("   Run TEST 0. If no /dev/i2c* exists, the I2C driver is not");
        puts("   running — start it (e.g. i2c-bcm2711) and retry.");
        return;
    }
    printf("   opened %s\n   scanning addresses 0x08..0x77 ...\n", dev);
    int found=0;
    for (uint8_t a=0x08; a<=0x77; a++){
        uint8_t v;
        if (i2c_read_reg(fd, a, 0x00, &v, 1) == EOK){
            printf("      device at 0x%02X%s\n", a,
                   a==MPU6050_ADDR ? "   <-- MPU-6050 IMU" : "");
            found++;
        }
    }
    if (!found){
        puts("   FAIL: no devices responded. Check:");
        puts("     - IMU VCC on 3.3V (phys 1), NOT 5V");
        puts("     - SDA -> GPIO2 (phys 3), SCL -> GPIO3 (phys 5)");
        puts("     - AD0 tied to GND, GND to star ground");
    } else {
        printf("   %d device(s) found.\n", found);
    }
    close(fd);
}

/* ====================================================================== */
/* TEST 9 — IMU live accelerometer                                        */
/* ====================================================================== */
static void t9_imu(void)
{
    puts("\n=== TEST 9 : IMU LIVE ACCELEROMETER ===");
    char dev[32]="";
    int fd = i2c_open_any(dev,sizeof dev);
    if (fd < 0){ puts("   FAIL: no I2C device (run TEST 8 first)"); return; }

    /* WHO_AM_I should read 0x68 */
    uint8_t who=0;
    int rc = i2c_read_reg(fd, MPU6050_ADDR, MPU_WHO_AM_I, &who, 1);
    if (rc != EOK){
        printf("   FAIL: WHO_AM_I read error: %s (%d)\n", strerror(rc), rc);
        close(fd); return;
    }
    printf("   WHO_AM_I = 0x%02X  (expect 0x68)%s\n", who,
           who==0x68 ? "  OK" : "  <-- unexpected");

    /* wake the device */
    struct { i2c_send_t hdr; uint8_t data[2]; } w;
    memset(&w,0,sizeof w);
    w.hdr.slave.addr = MPU6050_ADDR; w.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    w.hdr.len = 2; w.hdr.stop = 1;
    w.data[0] = MPU_PWR_MGMT_1; w.data[1] = 0x00;
    rc = devctl(fd, DCMD_I2C_SEND, &w, sizeof w, NULL);
    if (rc != EOK){ printf("   FAIL: wake error: %s (%d)\n", strerror(rc), rc);
                    close(fd); return; }
    puts("   IMU awake. Reading for 15 s.\n");
    puts("   Tilt the board NOSE-DOWN: aX should go clearly NEGATIVE.");
    puts("   Flat and still: aZ should read about +9.8, aX and aY near 0.\n");

    for (int i=0;i<60;i++){
        uint8_t d[6];
        if (i2c_read_reg(fd, MPU6050_ADDR, MPU_ACCEL_XOUT, d, 6) != EOK){
            puts("\n   read error"); break;
        }
        double ax = ((int16_t)((d[0]<<8)|d[1]) / MPU_LSB_PER_G) * G_MPS2;
        double ay = ((int16_t)((d[2]<<8)|d[3]) / MPU_LSB_PER_G) * G_MPS2;
        double az = ((int16_t)((d[4]<<8)|d[5]) / MPU_LSB_PER_G) * G_MPS2;
        double mag = sqrt(ax*ax+ay*ay+az*az)/G_MPS2;
        printf("\r   aX %7.2f   aY %7.2f   aZ %7.2f  m/s^2   |a| %5.2f g   "
               "decel %6.2f    ", ax, ay, az, mag, ax < 0 ? -ax : 0.0);
        fflush(stdout);
        msleep(250);
    }
    close(fd);
    puts("\n\n   PASS if |a| stayed near 1.00 g when still, and aX went");
    puts("   negative when you tilted the nose down.");
    puts("   If aX is POSITIVE under braking on the vehicle, flip the sign");
    puts("   on the 'decel = -ax;' line in imu_task() in aeb.c.");
}

/* ====================================================================== */
int main(void)
{
    puts("==========================================================");
    puts("  AEB COMPONENT BRING-UP TESTER  —  Team PRIORITY ZERO");
    puts("==========================================================");

    if (gpio_init() != 0){
        puts("\nWARNING: GPIO mapping failed. Tests 1-5 will do nothing.");
        puts("Tests 0, 6-9 (UART / I2C) still work.\n");
    } else {
        pins_init();
        puts("\nGPIO mapped. All motor pins set LOW (motors safe).");
    }

    for (;;){
        puts("\n----------------------------------------------------------");
        puts("  0  device discovery        5  all four motors");
        puts("  1  GPIO output toggle      6  UART raw bytes (radar)");
        puts("  2  interlock input         7  radar frame decode");
        puts("  3  one motor at a time     8  I2C bus scan");
        puts("  4  PWM duty sweep          9  IMU live accelerometer");
        puts("  q  quit (motors safe)");
        printf("\n  select > "); fflush(stdout);

        char line[16];
        if (!fgets(line,sizeof line,stdin)) break;
        switch (line[0]){
            case '0': t0_discover();    break;
            case '1': t1_gpio();        break;
            case '2': t2_interlock();   break;
            case '3': t3_one_motor();   break;
            case '4': t4_pwm();         break;
            case '5': t5_all_motors();  break;
            case '6': t6_uart_raw();    break;
            case '7': t7_radar_decode();break;
            case '8': t8_i2c_scan();    break;
            case '9': t9_imu();         break;
            case 'q': case 'Q':
                all_motors_off();
                if (g_gpio) munmap_device_memory((void*)g_gpio, GPIO_LEN);
                puts("\nmotors safe. bye.");
                return 0;
            default: puts("  ? unknown option");
        }
    }
    all_motors_off();
    if (g_gpio) munmap_device_memory((void*)g_gpio, GPIO_LEN);
    return 0;
}
