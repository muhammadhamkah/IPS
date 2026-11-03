/*
 * IDPS test marker (docs/MARKER.md): an IR LED that blinks a drone ID code, as each drone's
 * marker will. The engine reads the code from the brightness the cameras see (docs/PROTOCOL.md
 * section 6.2, Mode A): free-running, 30 chips a second, HIGH = 100 % and LOW = 40 % of the LED's
 * brightness (never off, so the cameras keep tracking it).
 *
 * The two levels are PWM duty cycles at 200 kHz, far faster than a camera exposure, so every
 * exposure sees a steady level. A timer steps through the code. A visible LED (the XIAO's user
 * LED) shows the same code, so it can be checked by eye.
 *
 * USB console (115200, any serial terminal, or `idf.py monitor`): id N, rate R, low P, bright P,
 * status. Settings are kept in flash.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "markercode.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "marker";

#define PWM_BITS LEDC_TIMER_7_BIT
#define PWM_MAX ((1u << 7) - 1u)

static struct {
    int id;
    int rate_mhz;        /* chips per 1000 s (30000 = 30 chips/s) */
    int low_pct;         /* LOW level, % of HIGH */
    int bright_pct;      /* HIGH level, % of full duty */
} cfg = {CONFIG_MARKER_DEFAULT_ID, 30000, 40, 100};

static uint8_t chips[MC_CHIPS];
static volatile int chip_i;
static esp_timer_handle_t timer;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

// ------------------------------------------------------------------ settings in flash
static void save(void) {
    nvs_handle_t h;
    if (nvs_open("marker", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "id", cfg.id);
    nvs_set_i32(h, "rate", cfg.rate_mhz);
    nvs_set_i32(h, "low", cfg.low_pct);
    nvs_set_i32(h, "bright", cfg.bright_pct);
    nvs_commit(h);
    nvs_close(h);
}

static void load(void) {
    nvs_handle_t h;
    if (nvs_open("marker", NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, "id", &v) == ESP_OK && v >= 0 && v <= MC_ID_MAX) cfg.id = v;
    if (nvs_get_i32(h, "rate", &v) == ESP_OK && v >= 1000 && v <= 120000) cfg.rate_mhz = v;
    if (nvs_get_i32(h, "low", &v) == ESP_OK && v >= 0 && v <= 100) cfg.low_pct = v;
    if (nvs_get_i32(h, "bright", &v) == ESP_OK && v >= 0 && v <= 100) cfg.bright_pct = v;
    nvs_close(h);
}

// ------------------------------------------------------------------ the LED
static uint32_t duty_for(int high) {
    const uint32_t hi = PWM_MAX * (uint32_t)cfg.bright_pct / 100u;
    return high ? hi : hi * (uint32_t)cfg.low_pct / 100u;
}

static void set_preview(int high) {
#if CONFIG_MARKER_PREVIEW_GPIO >= 0
#if CONFIG_MARKER_PREVIEW_ACTIVE_LOW
    gpio_set_level(CONFIG_MARKER_PREVIEW_GPIO, high ? 0 : 1);
#else
    gpio_set_level(CONFIG_MARKER_PREVIEW_GPIO, high ? 1 : 0);
#endif
#endif
}

static void on_chip(void *arg) {
    portENTER_CRITICAL(&lock);
    const int i = chip_i = (chip_i + 1) % MC_CHIPS;
    const int high = chips[i];
    portEXIT_CRITICAL(&lock);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_for(high));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    set_preview(high);
}

static void apply_code(void) {
    uint8_t c[MC_CHIPS];
    mc_chips(cfg.id, c);
    portENTER_CRITICAL(&lock);
    memcpy(chips, c, sizeof chips);
    chip_i = MC_CHIPS - 1;             /* the next chip starts a frame */
    portEXIT_CRITICAL(&lock);
}

static void apply_rate(void) {
    esp_timer_stop(timer);
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, (uint64_t)(1e9 / cfg.rate_mhz)));   /* us per chip */
}

// ------------------------------------------------------------------ console commands
static void print_status(void) {
    char code[MC_CHIPS + 1];
    for (int i = 0; i < MC_CHIPS; i++) code[i] = chips[i] ? 'H' : 'L';
    code[MC_CHIPS] = 0;
    printf("IDPS marker: ID %d (CRC %X), %.3f chips/s, LOW %d %%, brightness %d %%, IR LED GPIO%d\n"
           "  code %s (one frame = %.2f s)\n",
           cfg.id, mc_crc4((uint8_t)cfg.id), cfg.rate_mhz / 1000.0, cfg.low_pct, cfg.bright_pct,
           CONFIG_MARKER_IR_GPIO, code, MC_CHIPS * 1000.0 / cfg.rate_mhz);
}

static int int_arg(int argc, char **argv, int lo, int hi, int *out) {
    if (argc != 2) return -1;
    char *end;
    const long v = strtol(argv[1], &end, 10);
    if (*end || v < lo || v > hi) return -1;
    *out = (int)v;
    return 0;
}

static int cmd_id(int argc, char **argv) {
    if (int_arg(argc, argv, 0, MC_ID_MAX, &cfg.id)) { printf("usage: id 0..255 (the drone ID)\n"); return 1; }
    apply_code();
    save();
    print_status();
    return 0;
}

static int cmd_rate(int argc, char **argv) {
    double r = argc == 2 ? atof(argv[1]) : 0;
    if (r < 1 || r > 120) { printf("usage: rate 1..120 (chips/s; 30 is the system's; 1-3 to watch by eye)\n"); return 1; }
    cfg.rate_mhz = (int)(r * 1000 + 0.5);
    apply_rate();
    save();
    print_status();
    return 0;
}

static int cmd_low(int argc, char **argv) {
    if (int_arg(argc, argv, 0, 100, &cfg.low_pct)) { printf("usage: low 0..100 (LOW level, %% of HIGH; 40)\n"); return 1; }
    save();
    print_status();
    return 0;
}

static int cmd_bright(int argc, char **argv) {
    if (int_arg(argc, argv, 0, 100, &cfg.bright_pct)) { printf("usage: bright 0..100 (HIGH level, %%)\n"); return 1; }
    save();
    print_status();
    return 0;
}

static int cmd_status(int argc, char **argv) {
    print_status();
    return 0;
}

static void start_console(void) {
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt = "marker>";
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw, &rc, &repl));
#else
    esp_console_dev_uart_config_t hw = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw, &rc, &repl));
#endif
    const esp_console_cmd_t cmds[] = {
        {.command = "id", .help = "id N: set the marker ID (0-255)", .func = cmd_id},
        {.command = "rate", .help = "rate R: chips per second (30; 1-3 to watch the preview LED)", .func = cmd_rate},
        {.command = "low", .help = "low P: LOW level, % of HIGH (40)", .func = cmd_low},
        {.command = "bright", .help = "bright P: HIGH level, % (100)", .func = cmd_bright},
        {.command = "status", .help = "show the settings and the code", .func = cmd_status},
    };
    for (size_t i = 0; i < sizeof cmds / sizeof cmds[0]; i++) ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    load();

    const ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = CONFIG_MARKER_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    const ledc_channel_config_t c = {
        .gpio_num = CONFIG_MARKER_IR_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
#if CONFIG_MARKER_PREVIEW_GPIO >= 0
    gpio_reset_pin(CONFIG_MARKER_PREVIEW_GPIO);
    gpio_set_direction(CONFIG_MARKER_PREVIEW_GPIO, GPIO_MODE_OUTPUT);
#endif

    apply_code();
    const esp_timer_create_args_t ta = {.callback = on_chip, .name = "chip"};
    ESP_ERROR_CHECK(esp_timer_create(&ta, &timer));
    apply_rate();
    ESP_LOGI(TAG, "running");
    print_status();
    start_console();
}
