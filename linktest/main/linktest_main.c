/*
 * IDPS radio link test (docs/LINKTEST.md). Two ESP32-C5 boards measure the link that will carry
 * drone positions: the GROUND board (at the PC) sends a position-sized packet at the position rate
 * (120/s), the DRONE board (carried around) answers each one. The ground board prints, every
 * second, how many positions the drone got, the longest time the drone went without one, the round
 * trip time and the signal strength.
 *
 * Link: Wi-Fi, the ground board is the access point (5 GHz channel 36 by default; `channel 6` to
 * compare 2.4 GHz), the drone board joins it, UDP both ways. Unicast, so Wi-Fi retries lost frames,
 * as the real link will.
 *
 * Console (USB, `idf.py monitor`): role ground|drone, rate, size, channel, country, stats, reset,
 * watch on|off. Settings are kept in flash.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_console.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "linktest";

#define SSID "idps-linktest"
#define PASS "idps-linktest"
#define GROUND_IP "192.168.4.1"
#define PORT 14600
#define MAGIC 0x4B4C4449u            /* "IDLK" */
#define LED_GPIO 27                  /* XIAO ESP32-C5 user LED, lit when low */

enum { T_POS = 1, T_ECHO = 2, T_HELLO = 3 };
enum { ROLE_NONE = 0, ROLE_GROUND = 1, ROLE_DRONE = 2 };

/* Gap thresholds (ms): how long the drone went without a new position. 8.3 ms is normal at 120/s. */
#define NGAP 4
static const int GAP_MS[NGAP] = {25, 50, 100, 250};

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t type;
    uint8_t pad0;
    uint16_t size;
    uint32_t seq;
    uint32_t epoch;                  /* POS: the ground's counter epoch; the drone restarts its counters on a change */
    int64_t t_us;                    /* ground clock when sent; echoed back */
    /* ECHO only, the drone's counters since the epoch began: */
    uint32_t rx;                     /* positions received */
    uint32_t late;                   /* arrived after a newer one (out of order or duplicate) */
    uint32_t gap_max_window_us;      /* longest gap in the last second */
    uint32_t gap_max_us;             /* longest gap since the epoch began */
    uint32_t gaps[NGAP];             /* gaps longer than GAP_MS[i] */
    int8_t rssi;                     /* drone's received signal, dBm */
    uint8_t pad1[3];
} pkt_t;

static struct {
    int32_t role, rate, size, channel;
    char country[4];
    int32_t watch;
} cfg = {ROLE_NONE, 120, 64, 36, "01", 1};

static int sock = -1;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

// ------------------------------------------------------------------ settings
static void save(void) {
    nvs_handle_t h;
    if (nvs_open("linktest", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "role", cfg.role);
    nvs_set_i32(h, "rate", cfg.rate);
    nvs_set_i32(h, "size", cfg.size);
    nvs_set_i32(h, "chan", cfg.channel);
    nvs_set_i32(h, "watch", cfg.watch);
    nvs_set_str(h, "country", cfg.country);
    nvs_commit(h);
    nvs_close(h);
}

static void load(void) {
    nvs_handle_t h;
    if (nvs_open("linktest", NVS_READONLY, &h) != ESP_OK) return;
    int32_t v;
    if (nvs_get_i32(h, "role", &v) == ESP_OK && v >= 0 && v <= 2) cfg.role = v;
    if (nvs_get_i32(h, "rate", &v) == ESP_OK && v >= 1 && v <= 1000) cfg.rate = v;
    if (nvs_get_i32(h, "size", &v) == ESP_OK && v >= (int)sizeof(pkt_t) && v <= 1400) cfg.size = v;
    if (nvs_get_i32(h, "chan", &v) == ESP_OK && v >= 1 && v <= 177) cfg.channel = v;
    if (nvs_get_i32(h, "watch", &v) == ESP_OK) cfg.watch = v != 0;
    size_t n = sizeof cfg.country;
    if (nvs_get_str(h, "country", cfg.country, &n) != ESP_OK) strcpy(cfg.country, "01");
    nvs_close(h);
}

static void led(int on) { gpio_set_level(LED_GPIO, on ? 0 : 1); }

// ------------------------------------------------------------------ ground: statistics
#define RTT_BIN_US 250
#define RTT_BINS 2000                /* up to 500 ms; longer goes in the last bin */
#define WIN_MAX 1024

static struct {
    int64_t t_reset_us;
    uint32_t epoch;
    uint32_t sent, send_errors, echoes;
    uint32_t sent_prev, rx_prev;     /* at the last 1 s line */
    pkt_t last;                      /* the newest echo's drone counters */
    int have_echo;
    int64_t last_echo_us;
    uint32_t rtt_hist[RTT_BINS];
    uint32_t rtt_win[WIN_MAX];
    int nwin;
    uint32_t rtt_max_us;
    int8_t rssi_ground;
} st;

static struct sockaddr_in drone_addr;
static volatile int drone_known;

static void stats_reset(void) {
    portENTER_CRITICAL(&lock);
    const uint32_t epoch = st.epoch + 1;
    memset(&st, 0, sizeof st);
    st.epoch = epoch;
    st.t_reset_us = esp_timer_get_time();
    portEXIT_CRITICAL(&lock);
}

static int cmp_u32(const void *a, const void *b) {
    const uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return x < y ? -1 : x > y;
}

static double hist_pct(const uint32_t *h, uint32_t total, double pct) {
    if (!total) return 0;
    const uint32_t want = (uint32_t)(total * pct / 100.0 + 0.999);
    uint32_t acc = 0;
    for (int i = 0; i < RTT_BINS; i++) {
        acc += h[i];
        if (acc >= want) return (i + 1) * RTT_BIN_US / 1000.0;
    }
    return RTT_BINS * RTT_BIN_US / 1000.0;
}

static void print_totals(void) {
    static uint32_t hist[RTT_BINS];
    portENTER_CRITICAL(&lock);
    const pkt_t d = st.last;
    const uint32_t sent = st.sent, errs = st.send_errors, echoes = st.echoes, rmax = st.rtt_max_us;
    const int have = st.have_echo;
    const double secs = (esp_timer_get_time() - st.t_reset_us) / 1e6;
    memcpy(hist, st.rtt_hist, sizeof hist);
    portEXIT_CRITICAL(&lock);
    printf("\n=== since reset (%.0f s), %d positions/s of %d bytes, channel %d ===\n", secs, (int)cfg.rate,
           (int)cfg.size, (int)cfg.channel);
    if (!have) { printf("no reply from the drone yet\n\n"); return; }
    const int lost = (int)sent - (int)d.rx;
    printf("positions: sent %lu, drone got %lu, lost %d (%.3f %%)%s\n", (unsigned long)sent, (unsigned long)d.rx,
           lost < 0 ? 0 : lost, sent ? 100.0 * (lost < 0 ? 0 : lost) / sent : 0.0,
           "");
    if (errs) printf("  of which %lu could not be sent at all (radio busy)\n", (unsigned long)errs);
    printf("gaps at the drone (time without a new position): ");
    for (int i = 0; i < NGAP; i++) printf(">%d ms: %lu   ", GAP_MS[i], (unsigned long)d.gaps[i]);
    printf("longest %.1f ms\n", d.gap_max_us / 1000.0);
    printf("round trip (ground -> drone -> ground): median %.2f ms, 99%% %.2f ms, 99.9%% %.2f ms, max %.1f ms\n",
           hist_pct(hist, echoes, 50), hist_pct(hist, echoes, 99), hist_pct(hist, echoes, 99.9), rmax / 1000.0);
    printf("answers lost on the way back: %ld; arrived out of order: %lu\n\n", (long)d.rx - (long)echoes,
           (unsigned long)d.late);
}

// ------------------------------------------------------------------ ground: tasks
static TaskHandle_t sender_task;
static esp_timer_handle_t tick_timer;

static void on_tick(void *arg) { xTaskNotifyGive(sender_task); }

static void ground_sender(void *arg) {
    static uint8_t buf[1400];
    uint32_t seq = 0;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!drone_known) continue;
        pkt_t p = {.magic = MAGIC, .type = T_POS, .size = (uint16_t)cfg.size, .seq = ++seq};
        portENTER_CRITICAL(&lock);
        p.epoch = st.epoch;
        portEXIT_CRITICAL(&lock);
        p.t_us = esp_timer_get_time();
        memcpy(buf, &p, sizeof p);
        const int r = sendto(sock, buf, cfg.size, 0, (struct sockaddr *)&drone_addr, sizeof drone_addr);
        portENTER_CRITICAL(&lock);
        st.sent++;
        if (r < 0) st.send_errors++;
        portEXIT_CRITICAL(&lock);
    }
}

static void ground_receiver(void *arg) {
    static uint8_t buf[1500];
    for (;;) {
        struct sockaddr_in from;
        socklen_t fl = sizeof from;
        const int n = recvfrom(sock, buf, sizeof buf, 0, (struct sockaddr *)&from, &fl);
        if (n < (int)sizeof(pkt_t)) continue;
        pkt_t p;
        memcpy(&p, buf, sizeof p);
        if (p.magic != MAGIC) continue;
        const int64_t now = esp_timer_get_time();
        if (!drone_known || from.sin_addr.s_addr != drone_addr.sin_addr.s_addr) {
            drone_addr = from;
            drone_known = 1;
            printf("drone connected from %s\n", inet_ntoa(from.sin_addr));
            stats_reset();
        }
        if (p.type != T_ECHO) continue;
        portENTER_CRITICAL(&lock);
        if (p.epoch == st.epoch) {
            if (st.have_echo && p.rx < st.last.rx) {
                /* the drone restarted: its counters went back to zero */
                portEXIT_CRITICAL(&lock);
                printf("the drone restarted: counters reset\n");
                stats_reset();
                continue;
            }
            const uint32_t rtt = (uint32_t)(now - p.t_us);
            st.echoes++;
            st.rtt_hist[rtt / RTT_BIN_US < RTT_BINS ? rtt / RTT_BIN_US : RTT_BINS - 1]++;
            if (st.nwin < WIN_MAX) st.rtt_win[st.nwin++] = rtt;
            if (rtt > st.rtt_max_us) st.rtt_max_us = rtt;
            if (!st.have_echo || p.rx >= st.last.rx) st.last = p;
            st.have_echo = 1;
            st.last_echo_us = now;
        }
        portEXIT_CRITICAL(&lock);
    }
}

static void ground_reporter(void *arg) {
    static uint32_t win[WIN_MAX];
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
        wifi_sta_list_t sl;
        int8_t rssi = 0;
        if (esp_wifi_ap_get_sta_list(&sl) == ESP_OK && sl.num > 0) rssi = sl.sta[0].rssi;
        portENTER_CRITICAL(&lock);
        const pkt_t d = st.last;
        const int have = st.have_echo;
        const int64_t age = esp_timer_get_time() - st.last_echo_us;
        const uint32_t sent = st.sent - st.sent_prev, got = d.rx - st.rx_prev;
        st.sent_prev = st.sent;
        st.rx_prev = d.rx;
        const int n = st.nwin;
        memcpy(win, st.rtt_win, n * sizeof win[0]);
        st.nwin = 0;
        const double secs = (esp_timer_get_time() - st.t_reset_us) / 1e6;
        portEXIT_CRITICAL(&lock);
        led(have && age < 200000);
        if (!cfg.watch) continue;
        if (!drone_known) { printf("waiting for the drone board to connect...\n"); continue; }
        if (!have || age > 1000000) {
            printf("%5.0f s  NO ANSWER from the drone for %.1f s\n", secs, have ? age / 1e6 : secs);
            continue;
        }
        qsort(win, n, sizeof win[0], cmp_u32);
        const int lost = (int)sent - (int)got;
        printf("%5.0f s  sent %3lu  drone got %3lu  lost %2d | longest gap %5.1f ms | round trip %5.2f / %5.2f ms"
               " (median / max) | signal %d dBm here, %d dBm at the drone\n",
               secs, (unsigned long)sent, (unsigned long)got, lost < 0 ? 0 : lost, d.gap_max_window_us / 1000.0,
               n ? win[n / 2] / 1000.0 : 0.0, n ? win[n - 1] / 1000.0 : 0.0, rssi, d.rssi);
    }
}

// ------------------------------------------------------------------ drone
static struct {
    uint32_t epoch, rx, late, last_seq, gap_max_window, gap_max, gaps[NGAP];
    int64_t last_rx_us;
    int started;
} dr;
static volatile int drone_rssi;

static void drone_task(void *arg) {
    static uint8_t buf[1500];
    struct sockaddr_in ground = {.sin_family = AF_INET, .sin_port = htons(PORT)};
    inet_aton(GROUND_IP, &ground.sin_addr);
    struct timeval tv = {.tv_sec = 0, .tv_usec = 200000};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    for (;;) {
        const int n = recv(sock, buf, sizeof buf, 0);
        const int64_t now = esp_timer_get_time();
        if (n < (int)sizeof(pkt_t)) {
            /* nothing for 200 ms: say hello so the ground board knows where we are */
            if (now - dr.last_rx_us > 500000) {
                pkt_t h = {.magic = MAGIC, .type = T_HELLO, .size = sizeof(pkt_t)};
                sendto(sock, &h, sizeof h, 0, (struct sockaddr *)&ground, sizeof ground);
            }
            continue;
        }
        pkt_t p;
        memcpy(&p, buf, sizeof p);
        if (p.magic != MAGIC || p.type != T_POS) continue;
        portENTER_CRITICAL(&lock);
        if (!dr.started || p.epoch != dr.epoch) {
            memset(&dr, 0, sizeof dr);
            dr.started = 1;
            dr.epoch = p.epoch;
            dr.last_rx_us = now;
        }
        if (dr.rx && p.seq <= dr.last_seq) {
            dr.late++;                /* not a new position */
        } else {
            const uint32_t gap = (uint32_t)(now - dr.last_rx_us);
            if (dr.rx) {
                if (gap > dr.gap_max_window) dr.gap_max_window = gap;
                if (gap > dr.gap_max) dr.gap_max = gap;
                for (int i = 0; i < NGAP; i++)
                    if (gap > (uint32_t)GAP_MS[i] * 1000u) dr.gaps[i]++;
            }
            dr.last_rx_us = now;
            dr.last_seq = p.seq;
        }
        dr.rx++;
        pkt_t e = p;
        e.type = T_ECHO;
        e.size = sizeof(pkt_t);
        e.rx = dr.rx - dr.late;
        e.late = dr.late;
        e.gap_max_window_us = dr.gap_max_window;
        e.gap_max_us = dr.gap_max;
        memcpy(e.gaps, dr.gaps, sizeof e.gaps);
        e.rssi = (int8_t)drone_rssi;
        portEXIT_CRITICAL(&lock);
        sendto(sock, &e, sizeof e, 0, (struct sockaddr *)&ground, sizeof ground);
    }
}

static void drone_reporter(void *arg) {
    TickType_t wake = xTaskGetTickCount();
    uint32_t prev = 0;
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
        int rssi = 0;
        if (esp_wifi_sta_get_rssi(&rssi) == ESP_OK) drone_rssi = rssi;
        portENTER_CRITICAL(&lock);
        const uint32_t rx = dr.rx - dr.late, gmax = dr.gap_max_window;
        dr.gap_max_window = 0;
        portEXIT_CRITICAL(&lock);
        if (cfg.watch)
            printf("got %3lu positions in the last second, longest gap %5.1f ms, signal %d dBm\n",
                   (unsigned long)(rx - prev), gmax / 1000.0, rssi);
        prev = rx;
    }
}

static void drone_led(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20));
        portENTER_CRITICAL(&lock);
        const int ok = dr.rx && esp_timer_get_time() - dr.last_rx_us < 100000;
        portEXIT_CRITICAL(&lock);
        led(ok);                      /* on: positions arriving; off: more than 100 ms without one */
    }
}

// ------------------------------------------------------------------ Wi-Fi
static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        printf("lost the ground board, reconnecting...\n");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
            printf("connected to the ground board on channel %d, signal %d dBm\n", ap.primary, ap.rssi);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        printf("a board joined the link\n");
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        printf("the drone board left the link\n");
    }
}

static void wifi_start(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL));
    const int ground = cfg.role == ROLE_GROUND;
    if (ground) esp_netif_create_default_wifi_ap();
    else esp_netif_create_default_wifi_sta();
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    esp_err_t e = esp_wifi_set_country_code(cfg.country, false);
    if (e != ESP_OK) printf("country '%s' not accepted (%s); using the default\n", cfg.country, esp_err_to_name(e));
    wifi_config_t wc = {0};
    if (ground) {
        strcpy((char *)wc.ap.ssid, SSID);
        wc.ap.ssid_len = strlen(SSID);
        strcpy((char *)wc.ap.password, PASS);
        wc.ap.channel = cfg.channel;
        wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
        wc.ap.max_connection = 2;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    } else {
        strcpy((char *)wc.sta.ssid, SSID);
        strcpy((char *)wc.sta.password, PASS);
        wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        wc.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    e = esp_wifi_set_band_mode(ground ? (cfg.channel > 14 ? WIFI_BAND_MODE_5G_ONLY : WIFI_BAND_MODE_2G_ONLY)
                                      : WIFI_BAND_MODE_AUTO);
    if (e != ESP_OK) printf("band mode: %s\n", esp_err_to_name(e));
    if (!ground) ESP_ERROR_CHECK(esp_wifi_connect());
    if (ground) {
        /* the channel after the band, so a 5 GHz channel is accepted */
        e = esp_wifi_set_config(WIFI_IF_AP, &wc);
        if (e != ESP_OK) printf("could not start the link on channel %d: %s\n", (int)cfg.channel, esp_err_to_name(e));
        uint8_t prim;
        wifi_second_chan_t sec;
        if (esp_wifi_get_channel(&prim, &sec) == ESP_OK && prim != cfg.channel)
            printf("WARNING: asked for channel %d but the radio is on %d (country '%s' may not allow it)\n",
                   (int)cfg.channel, prim, cfg.country);
        else
            printf("link up on channel %d (%s GHz)\n", (int)cfg.channel, cfg.channel > 14 ? "5" : "2.4");
    }
    esp_wifi_set_ps(WIFI_PS_NONE);    /* no power saving: it delays packets */
}

static void open_socket(void) {
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(PORT), .sin_addr.s_addr = htonl(INADDR_ANY)};
    ESP_ERROR_CHECK(bind(sock, (struct sockaddr *)&a, sizeof a) == 0 ? ESP_OK : ESP_FAIL);
}

// ------------------------------------------------------------------ console
static const char *role_name(int r) { return r == ROLE_GROUND ? "ground" : r == ROLE_DRONE ? "drone" : "not set"; }

static void print_status(void) {
    printf("IDPS link test: role %s, channel %d (%s GHz), %d positions/s of %d bytes, country %s\n",
           role_name(cfg.role), (int)cfg.channel, cfg.channel > 14 ? "5" : "2.4", (int)cfg.rate, (int)cfg.size,
           cfg.country);
}

static int need_restart(const char *what) {
    save();
    printf("%s saved; restarting...\n", what);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return 0;
}

static int cmd_role(int argc, char **argv) {
    if (argc == 2 && !strcmp(argv[1], "ground")) cfg.role = ROLE_GROUND;
    else if (argc == 2 && !strcmp(argv[1], "drone")) cfg.role = ROLE_DRONE;
    else { printf("usage: role ground|drone\n"); return 1; }
    return need_restart("role");
}

static int cmd_rate(int argc, char **argv) {
    const int v = argc == 2 ? atoi(argv[1]) : 0;
    if (v < 1 || v > 1000) { printf("usage: rate 1..1000 (positions per second; 120)\n"); return 1; }
    cfg.rate = v;
    save();
    if (cfg.role == ROLE_GROUND) {
        esp_timer_stop(tick_timer);
        esp_timer_start_periodic(tick_timer, 1000000 / cfg.rate);
        stats_reset();
    }
    print_status();
    return 0;
}

static int cmd_size(int argc, char **argv) {
    const int v = argc == 2 ? atoi(argv[1]) : 0;
    if (v < (int)sizeof(pkt_t) || v > 1400) { printf("usage: size %d..1400 (bytes per packet; 64)\n", (int)sizeof(pkt_t)); return 1; }
    cfg.size = v;
    save();
    if (cfg.role == ROLE_GROUND) stats_reset();
    print_status();
    return 0;
}

static int cmd_channel(int argc, char **argv) {
    const int v = argc == 2 ? atoi(argv[1]) : 0;
    if (v < 1 || v > 177) { printf("usage: channel N (36, 40, 44, 48, 149... for 5 GHz; 1-13 for 2.4 GHz)\n"); return 1; }
    cfg.channel = v;
    if (cfg.role != ROLE_GROUND) { save(); printf("saved; the drone board finds the channel itself\n"); return 0; }
    return need_restart("channel");
}

static int cmd_country(int argc, char **argv) {
    if (argc != 2 || strlen(argv[1]) != 2) { printf("usage: country XX (e.g. US, GB, AU, JP; 01 = world safe)\n"); return 1; }
    strcpy(cfg.country, argv[1]);
    return need_restart("country");
}

static int cmd_stats(int argc, char **argv) {
    if (cfg.role != ROLE_GROUND) { printf("run 'stats' on the ground board\n"); return 1; }
    print_totals();
    return 0;
}

static int cmd_reset(int argc, char **argv) {
    if (cfg.role != ROLE_GROUND) { printf("run 'reset' on the ground board\n"); return 1; }
    stats_reset();
    printf("counters reset\n");
    return 0;
}

static int cmd_watch(int argc, char **argv) {
    if (argc != 2 || (strcmp(argv[1], "on") && strcmp(argv[1], "off"))) { printf("usage: watch on|off\n"); return 1; }
    cfg.watch = !strcmp(argv[1], "on");
    save();
    return 0;
}

static int cmd_status(int argc, char **argv) {
    print_status();
    return 0;
}

static void start_console(void) {
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt = cfg.role == ROLE_GROUND ? "ground>" : cfg.role == ROLE_DRONE ? "drone>" : "linktest>";
#if defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw, &rc, &repl));
#else
    esp_console_dev_uart_config_t hw = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw, &rc, &repl));
#endif
    const esp_console_cmd_t cmds[] = {
        {.command = "role", .help = "role ground|drone: what this board is (restarts)", .func = cmd_role},
        {.command = "stats", .help = "totals since the last reset (ground board)", .func = cmd_stats},
        {.command = "reset", .help = "start counting again (ground board)", .func = cmd_reset},
        {.command = "rate", .help = "rate N: positions per second (120)", .func = cmd_rate},
        {.command = "size", .help = "size N: bytes per packet (64)", .func = cmd_size},
        {.command = "channel", .help = "channel N: 36 (5 GHz) or 6 (2.4 GHz) etc. (ground board; restarts)", .func = cmd_channel},
        {.command = "country", .help = "country XX: radio rules for your country (restarts)", .func = cmd_country},
        {.command = "watch", .help = "watch on|off: the line every second", .func = cmd_watch},
        {.command = "status", .help = "show the settings", .func = cmd_status},
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
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    led(0);
    print_status();

    if (cfg.role == ROLE_NONE) {
        printf("\nSet this board's role first:\n"
               "  role ground   the board that stays at the PC\n"
               "  role drone    the board you carry around\n\n");
    } else {
        wifi_start();
        open_socket();
        if (cfg.role == ROLE_GROUND) {
            stats_reset();
            xTaskCreate(ground_sender, "send", 4096, NULL, 20, &sender_task);
            xTaskCreate(ground_receiver, "recv", 4096, NULL, 19, NULL);
            xTaskCreate(ground_reporter, "report", 4096, NULL, 5, NULL);
            const esp_timer_create_args_t ta = {.callback = on_tick, .name = "tick"};
            ESP_ERROR_CHECK(esp_timer_create(&ta, &tick_timer));
            ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 1000000 / cfg.rate));
            printf("ground board: waiting for the drone board. Type 'stats' for totals, 'reset' to start again.\n");
        } else {
            xTaskCreate(drone_task, "drone", 4096, NULL, 20, NULL);
            xTaskCreate(drone_reporter, "report", 4096, NULL, 5, NULL);
            xTaskCreate(drone_led, "led", 2048, NULL, 4, NULL);
            printf("drone board: looking for the ground board...\n");
        }
    }
    ESP_LOGI(TAG, "ready");
    start_console();
}
