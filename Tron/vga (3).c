#include <stdint.h>
#include <stdio.h>

#include "address_map_niosv.h"

typedef uint16_t pixel_t;

volatile pixel_t * VGA = (pixel_t *)FPGA_PIXEL_BUF_BASE;
volatile uint32_t * HEX3_HEX0 = (uint32_t *)HEX3_HEX0_BASE;
volatile uint32_t * HEX5_HEX4 = (uint32_t *)HEX5_HEX4_BASE;
volatile uint32_t * SW = (uint32_t *)SW_BASE;
volatile uint32_t * KEY = (uint32_t *)KEY_BASE;
volatile uint32_t * LEDR = (uint32_t *)LEDR_BASE;

const pixel_t COLOR_BLACK = 0x0000;
const pixel_t COLOR_WHITE = 0xFFFF;
const pixel_t COLOR_P1 = 0x07E0; // green
const pixel_t COLOR_P2 = 0xF800; // red

typedef struct {
    int x;
    int y;
    int dx;
    int dy;
    pixel_t color;
    int alive;
} Player;


Player human;
Player robot;

volatile int round_over = 0; // 1 when a crash happens
volatile int round_winner = 0; // 0 = tie, 1 = human, 2 = robot

// Pending turn: -1 = left, +1 = right, 0 = none
volatile int pending_turn = 0;

// Previous KEY state for edge detection (polled in main)
uint32_t prev_keys = 0;

void drawPixel(int y, int x, pixel_t c) {
    if (x < 0 || x >= MAX_X || y < 0 || y >= MAX_Y) return;
    VGA[(y << YSHIFT) + x] = c;
}

pixel_t readPixel(int y, int x) {
    if (x < 0 || x >= MAX_X || y < 0 || y >= MAX_Y) {
        return COLOR_WHITE;
    }
    return VGA[(y << YSHIFT) + x];
}

void clear_screen(pixel_t c) {
    for (int y = 0; y < MAX_Y; y++) {
        for (int x = 0; x < MAX_X; x++) {
            drawPixel(y, x, c);
        }
    }
}

void draw_border(void) {
    for (int x = 0; x < MAX_X; x++) {
        drawPixel(0, x, COLOR_WHITE);
        drawPixel(MAX_Y - 2, x, COLOR_WHITE);
    }
    for (int y = 0; y < MAX_Y; y++) {
        drawPixel(y, 0, COLOR_WHITE);
        drawPixel(y, MAX_X - 1, COLOR_WHITE);
    }
}

void draw_obstacles(void) { // "I-bar" obstacle
    int mid_x = MAX_X / 2;
    for (int y = MAX_Y / 4; y < (3 * MAX_Y) / 4; y++) {
        drawPixel(y, mid_x, COLOR_WHITE);
    }
    for (int x = MAX_X / 4; x < (3 * MAX_X) / 4; x++) {
        drawPixel(MAX_Y / 4, x, COLOR_WHITE);
        drawPixel((3 * MAX_Y) / 4, x, COLOR_WHITE);
    }
}

unsigned char seven_seg_digits[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

void show_scores(int s1, int s2) {
    if (s1 < 0) s1 = 0;
    if (s1 > 9) s1 = 9;
    if (s2 < 0) s2 = 0;
    if (s2 > 9) s2 = 9;

    unsigned char d0 = seven_seg_digits[s1]; // HEX0
    unsigned char d1 = seven_seg_digits[s2]; // HEX1

    unsigned int value = 0;
    value  = (unsigned int)d0; // HEX0 bits 7 to 0
    value |= ((unsigned int)d1) << 8; // HEX1 bits 15 to 8

    *HEX3_HEX0 = value; // HEX2/HEX3 blank
    *HEX5_HEX4 = 0; // HEX4/HEX5 blank
}

void reset_playfield(void) {
    clear_screen(COLOR_BLACK);
    draw_border();
    draw_obstacles();
}

// 2-pixel look-ahead for robot AI
int path_blocked(int x, int y, int dx, int dy) {
    int x1 = x + dx;
    int y1 = y + dy;
    int x2 = x1 + dx;
    int y2 = y1 + dy;

    if (readPixel(y1, x1) != COLOR_BLACK) return 1;
    if (readPixel(y2, x2) != COLOR_BLACK) return 1;
    return 0;
}

// Robot AI: if blocked ahead, try LEFT, else RIGHT, else straight
void update_robot_direction(Player *p) {
    if (!path_blocked(p->x, p->y, p->dx, p->dy)) {
        return; // keep going straight
    }

    // try turning left: (dx,dy) -> (-dy, dx)
    int ldx = -p->dy;
    int ldy =  p->dx;

    if (!path_blocked(p->x, p->y, ldx, ldy)) {
        p->dx = ldx;
        p->dy = ldy;
        return;
    }

    // then try turning right: (dx,dy) = (dy, -dx)
    int rdx =  p->dy;
    int rdy = -p->dx;

    if (!path_blocked(p->x, p->y, rdx, rdy)) {
        p->dx = rdx;
        p->dy = rdy;
        return;
    }
}

// Apply a left or right turn to the human
// dir = -1 = LEFT, dir = 1 = RIGHT
static void apply_turn(Player *p, int dir) {
    int old_dx = p->dx;
    int old_dy = p->dy;

    if (dir == -1) { // LEFT
        p->dx = -old_dy;
        p->dy =  old_dx;
    } else if (dir == 1) { // RIGHT
        p->dx =  old_dy;
        p->dy = -old_dx;
    }
}

unsigned int get_tick_period(void) {
    uint32_t sw = *SW;
    uint32_t val = sw & 0x3FF; // SW[9:0]

    if (val == 0) {
        val = 1; // avoid 0 period
    }
    return val * 200000U; //adjust for speed
}

void update_leds(void) {
    uint32_t v = 0;

    // LEDR[1] = LEFT pending, LEDR[0] = RIGHT pending
    if (pending_turn == -1) {
        v |= (1u << 1); // left
    } else if (pending_turn == 1) {
        v |= (1u << 0); // right
    }

    *LEDR = v;
}

void init_round(void) {
    reset_playfield();

    human.x = MAX_X / 3;
    human.y = MAX_Y / 2;
    human.dx = 1;
    human.dy = 0;
    human.color = COLOR_P1;
    human.alive = 1;

    robot.x = (2 * MAX_X) / 3;
    robot.y = MAX_Y / 2;
    robot.dx = -1;
    robot.dy = 0;
    robot.color = COLOR_P2;
    robot.alive = 1;

    drawPixel(human.y, human.x, human.color);
    drawPixel(robot.y,  robot.x,  robot.color);

    round_over = 0;
    round_winner = 0;

    pending_turn = 0;
    prev_keys = *KEY;
    update_leds();
}

// One game "tick": called by the timer ISR
void step_round_tick(void) {
    if (round_over) {
        return;
    }

    if (!human.alive && !robot.alive) {
        round_over = 1;
        round_winner = 0;
        return;
    }

    if (pending_turn != 0) {
        apply_turn(&human, pending_turn);
        pending_turn = 0; // "reset"
        update_leds(); // LEDs off (no pending turn now)
    }

    update_robot_direction(&robot);

    int h_new_x = human.x + human.dx;
    int h_new_y = human.y + human.dy;
    int r_new_x = robot.x + robot.dx;
    int r_new_y = robot.y + robot.dy;

    int h_dead = 0;
    int r_dead = 0;

    // head-on collision
    if (h_new_x == r_new_x && h_new_y == r_new_y) {
        h_dead = 1;
        r_dead = 1;
    } else {
        if (readPixel(h_new_y, h_new_x) != COLOR_BLACK) {
            h_dead = 1;
        }
        if (readPixel(r_new_y, r_new_x) != COLOR_BLACK) {
            r_dead = 1;
        }
    }

    if (!h_dead) {
        human.x = h_new_x;
        human.y = h_new_y;
        drawPixel(human.y, human.x, human.color);
    }
    if (!r_dead) {
        robot.x = r_new_x;
        robot.y = r_new_y;
        drawPixel(robot.y, robot.x, robot.color);
    }

    human.alive = !h_dead;
    robot.alive = !r_dead;

    if (!human.alive || !robot.alive) {
        round_over = 1;
        if (human.alive && !robot.alive) {
            round_winner = 1;
        } else if (!human.alive && robot.alive) {
            round_winner = 2;
        } else {
            round_winner = 0;
        }
    }
}

// This runs in main loop at full CPU speed, so even a short press between ticks gets captured.
void poll_keys_for_pending_turn(void) {
    if (round_over) return;

    uint32_t keys = *KEY;
    uint32_t new_presses = keys & ~prev_keys; // 0 to 1 edge
    prev_keys = keys;

    // KEY[1] to LEFT (pending_turn = -1)
    if (new_presses & 0x2) {
        if (pending_turn == -1) {
            pending_turn = 0; // double-press cancels
        } else {
            pending_turn = -1; // queue LEFT
        }
    }

    // KEY[0] to RIGHT (pending_turn = +1)
    if (new_presses & 0x1) {
        if (pending_turn == 1) {
            pending_turn = 0;
        } else {
            pending_turn = 1;
        }
    }

    update_leds();
}

static void handler(void) __attribute__ ((interrupt("machine")));

static volatile uint32_t * const mtime_ptr = (volatile uint32_t *) MTIME_BASE;

static void set_mtimer(volatile uint32_t *time_ptr, uint64_t new_time64)
{
    time_ptr[0] = (uint32_t)0; // prevent hi increment
    time_ptr[1] = (uint32_t)(new_time64 >> 32); // set high 32 bits
    time_ptr[0] = (uint32_t)new_time64; // set low  32 bits
}

static uint64_t get_mtimer(volatile uint32_t *time_ptr)
{
    uint32_t mtime_h, mtime_l;

    do {
        mtime_h = time_ptr[1]; // read hi
        mtime_l = time_ptr[0]; // read lo
    } while (mtime_h != time_ptr[1]); // if hi changed, try again

    return ((uint64_t)mtime_h << 32) | mtime_l;
}

static void setup_mtimecmp(void)
{
    uint64_t now = get_mtimer(mtime_ptr); // current mtime
    uint64_t period = (uint64_t)get_tick_period(); // ticks per game step

    uint64_t next = now + period; // time of first tick

    set_mtimer(mtime_ptr + 2, next); // program mtimecmp
}

static void mtimer_ISR(void)
{
    // Read current mtime
    uint64_t now = get_mtimer(mtime_ptr); // current time
    uint64_t period = (uint64_t)get_tick_period(); // ticks between game steps

    uint64_t next = now + period;

    // Write next mtimecmp (starts at mtime_ptr+2)
    set_mtimer(mtime_ptr + 2, next);

    // Advance the game by one tick
    step_round_tick();
}

static void handler(void)
{
    int mcause_value;
    __asm__ volatile ("csrr %0, mcause" : "=r"(mcause_value));

    if (mcause_value == 0x80000007) { // machine timer interrupt
        mtimer_ISR();
    }
    // ignore all other traps
}

static void setup_cpu_irqs(uint32_t new_mie_value)
{
    uint32_t mstatus_value, mtvec_value, old_mie_value;

    mstatus_value = 0b1000; // mask for global MIE bit
    mtvec_value   = (uint32_t)&handler; // address of trap handler

    // Disable global interrupts while we configure things
    __asm__ volatile ("csrc mstatus, %0" :: "r"(mstatus_value));

    // Set trap vector (where to jump on an interrupt)
    __asm__ volatile ("csrw mtvec, %0" :: "r"(mtvec_value));

    // Read old mie, then clear all enabled interrupt sources
    __asm__ volatile ("csrr %0, mie" : "=r"(old_mie_value));
    __asm__ volatile ("csrc mie, %0" :: "r"(old_mie_value));

    // Enable only the sources specified by new_mie_value
    __asm__ volatile ("csrs mie, %0" :: "r"(new_mie_value));

    // Re-enable global interrupts
    __asm__ volatile ("csrs mstatus, %0" :: "r"(mstatus_value));
}

int main(void)
{
    int score1 = 0;
    int score2 = 0;

    show_scores(score1, score2);

    init_round(); // set up players and playfield
    setup_mtimecmp(); // program first mtimecmp interrupt
    setup_cpu_irqs(0x80); // enable machine timer IRQs (bit 7)

    while (1) {
        // Poll keys at full speed = queue turns & update LEDs
        poll_keys_for_pending_turn();

        if (round_over) {
            if (round_winner == 1) {
                score1++;
            } else if (round_winner == 2) {
                score2++;
            }
            show_scores(score1, score2);

            // First to 9 wins
            if (score1 >= 9 || score2 >= 9) {
                pixel_t winner_color =
                    (score1 > score2) ? COLOR_P1 : COLOR_P2;
                clear_screen(winner_color);
                while (1) {
                    // freeze
                }
            }

            // New round: just reset state; timer keeps running
            init_round();
        }

        // Useless work while ISR runs the game
        __asm__ volatile ("nop");
    }

    return 0;
}
