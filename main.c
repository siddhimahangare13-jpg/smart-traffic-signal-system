/* main.c - Smart Traffic Congestion and Signal Management System (DSA engine)
 * Runs the SAME vehicle arrivals through two controllers:
 *   mode 0: fixed-time (30 s green per lane, round robin)
 *   mode 1: adaptive (min-heap picks next lane, green time follows queue length,
 *           aging prevents starvation, ambulances preempt the cycle)
 * Writes data.js for the browser to replay.
 * Build: gcc main.c -o traffic -lm      Run: ./traffic
 */
#include <stdio.h>
#include <math.h>
#include "queue.h"
#include "heap.h"

#define T        600     /* simulated seconds (1 tick = 1 s) */
#define NL       4       /* lanes: 0=N 1=S 2=E 3=W */
#define FIXED_G  30
#define MIN_G    8
#define MAX_G    40
#define CLEAR    2       /* all-red gap between phases */
#define HEADWAY  2       /* one vehicle leaves every 2 s on green */
#define FW       11      /* values per frame */

typedef struct { const char *key, *label; double rate[NL]; int rush, amb; } Scenario;
static const Scenario SC[] = {
    {"low",    "Low traffic",      {.05,.05,.05,.05}, 0, 0},
    {"medium", "Medium traffic",   {.09,.09,.08,.08}, 0, 0},
    {"rush",   "Rush hour",        {.12,.12,.05,.05}, 1, 0},
    {"amb",    "Ambulance drill",  {.09,.09,.08,.08}, 0, 1},
};
static const int AMB_T[4] = {90, 250, 420, 530}, AMB_L[4] = {2, 0, 3, 1};

typedef struct { double sum, max, ambSum, ambMax; int served, arrived, dropped, ambN; } Metrics;

static unsigned rng_state;
static double rnd(void) { rng_state = rng_state * 1103515245u + 12345u; return ((rng_state >> 16) & 0x7fff) / 32768.0; }

static int arr[T][NL];                 /* 0 none, 1 car, 2 ambulance */
static int frames[2][T][FW];

static void gen_arrivals(const Scenario *s, unsigned seed) {
    rng_state = seed;
    for (int t = 0; t < T; t++) {
        double f = s->rush ? 0.6 + 0.8 * sin(3.14159265 * t / T) : 1.0;
        for (int l = 0; l < NL; l++) arr[t][l] = rnd() < s->rate[l] * f;
        if (s->amb) for (int i = 0; i < 4; i++) if (t == AMB_T[i]) arr[t][AMB_L[i]] = 2;
    }
}

static int has_emergency(CQueue *q) { for (int i = 0; i < q->count; i++) if (cq_at(q, i)->emergency) return 1; return 0; }

/* Adaptive choice: heap of lanes keyed by -priority. */
static int pick_adaptive(CQueue q[], int t, int last, int *green) {
    MinHeap h; h_init(&h);
    for (int l = 0; l < NL; l++) {
        double head = q[l].count ? t - cq_at(&q[l], 0)->arrive : 0;
        double pri = q[l].count + 0.3 * head;            /* congestion + aging */
        if (has_emergency(&q[l])) pri += 1000;           /* emergency wins */
        h_push(&h, -pri + 0.001 * ((l - last + 3) % NL), l);  /* tiny tiebreak: rotate */
    }
    int lane = h_pop(&h).lane;
    int g = HEADWAY * q[lane].count + 2;                 /* green proportional to queue */
    *green = g < MIN_G ? MIN_G : g > MAX_G ? MAX_G : g;
    return lane;
}

static Metrics simulate(int mode) {
    CQueue q[NL]; Metrics m = {0}; int id = 0;
    for (int l = 0; l < NL; l++) cq_init(&q[l]);
    int cur = -1, last = NL - 1, green_left = 0, elapsed = 0, clear_left = 1;
    for (int t = 0; t < T; t++) {
        for (int l = 0; l < NL; l++) if (arr[t][l]) {            /* 1. arrivals */
            Vehicle v = {id++, t, arr[t][l] == 2};
            if (cq_enqueue(&q[l], v)) m.arrived++; else m.dropped++;
        }
        if (cur < 0 && --clear_left <= 0) {                       /* 2. choose next phase */
            if (mode == 0) { cur = (last + 1) % NL; green_left = FIXED_G; }
            else cur = pick_adaptive(q, t, last, &green_left);
            elapsed = 0;
        }
        if (cur >= 0) {
            if (mode == 1 && elapsed >= 3)                        /* emergency preemption */
                for (int l = 0; l < NL; l++) if (l != cur && has_emergency(&q[l])) green_left = 0;
            if (green_left <= 0) { last = cur; cur = -1; clear_left = CLEAR; }
        }
        int dep = 0;                                              /* 3. discharge */
        if (cur >= 0) {
            if (!cq_empty(&q[cur]) && elapsed % HEADWAY == 0) {
                Vehicle v = cq_dequeue(&q[cur]);
                double w = t - v.arrive;
                m.sum += w; if (w > m.max) m.max = w; m.served++;
                if (v.emergency) { m.ambSum += w; m.ambN++; if (w > m.ambMax) m.ambMax = w; dep = 2; } else dep = 1;
            }
            elapsed++; green_left--;
        }
        int *f = frames[mode][t];                                 /* 4. snapshot */
        f[0] = cur; f[1] = cur >= 0 && green_left > 0 ? green_left : 0;
        for (int l = 0; l < NL; l++) {
            f[2 + l] = q[l].count; f[6 + l] = 0;
            for (int i = 0; i < q[l].count; i++) if (cq_at(&q[l], i)->emergency) { f[6 + l] = i + 1; break; }
        }
        f[10] = dep;
    }
    for (int l = 0; l < NL; l++)                                  /* still waiting at the end */
        for (int i = 0; i < q[l].count; i++) {
            Vehicle *v = cq_at(&q[l], i); double w = T - v->arrive;
            m.sum += w; if (w > m.max) m.max = w; m.served += 0; m.arrived += 0;
            if (v->emergency) { m.ambSum += w; m.ambN++; if (w > m.ambMax) m.ambMax = w; }
        }
    return m;
}

static int total_counted(Metrics *m, int mode) {   /* vehicles included in the average */
    int n = m->served;
    for (int l = 0; l < NL; l++) { int c = frames[mode][T - 1][2 + l]; n += c; }
    return n;
}

static void write_mode(FILE *fp, int mode, Metrics *m) {
    int n = total_counted(m, mode);
    fprintf(fp, "{\"m\":{\"avg\":%.2f,\"max\":%.0f,\"served\":%d,\"arrived\":%d,\"dropped\":%d,\"ambAvg\":%.2f,\"ambMax\":%.0f,\"ambN\":%d},\"f\":[",
        n ? m->sum / n : 0, m->max, m->served, m->arrived, m->dropped,
        m->ambN ? m->ambSum / m->ambN : 0, m->ambMax, m->ambN);
    for (int t = 0; t < T; t++) {
        fprintf(fp, "%s[", t ? "," : "");
        for (int k = 0; k < FW; k++) fprintf(fp, "%s%d", k ? "," : "", frames[mode][t][k]);
        fputc(']', fp);
    }
    fputs("]}", fp);
}

int main(void) {
    FILE *fp = fopen("data.js", "w");
    if (!fp) { perror("data.js"); return 1; }
    fprintf(fp, "const TRAFFIC_DATA={ticks:%d,scenarios:[", T);
    printf("%-18s %-9s %9s %9s %8s %9s\n", "Scenario", "Mode", "AvgWait", "MaxWait", "Served", "AmbWait");
    for (size_t s = 0; s < sizeof SC / sizeof SC[0]; s++) {
        gen_arrivals(&SC[s], 1234u + (unsigned)s);
        Metrics a = simulate(0), b = simulate(1);
        const char *nm[2] = {"Fixed", "Adaptive"}; Metrics *ms[2] = {&a, &b};
        for (int k = 0; k < 2; k++)
            printf("%-18s %-9s %8.1fs %8.0fs %8d %8.1fs\n", k ? "" : SC[s].label, nm[k],
                   total_counted(ms[k], k) ? ms[k]->sum / total_counted(ms[k], k) : 0, ms[k]->max,
                   ms[k]->served, ms[k]->ambN ? ms[k]->ambSum / ms[k]->ambN : 0);
        fprintf(fp, "%s{\"key\":\"%s\",\"label\":\"%s\",\"fixed\":", s ? "," : "", SC[s].key, SC[s].label);
        write_mode(fp, 0, &a); fputs(",\"adaptive\":", fp); write_mode(fp, 1, &b); fputc('}', fp);
    }
    fputs("]};\n", fp); fclose(fp);
    puts("\nWrote data.js - now open index.html");
    return 0;
}
