/* queue.h - Circular queue (one per lane). Fixed capacity, O(1) enqueue/dequeue. */
#ifndef QUEUE_H
#define QUEUE_H
#define QCAP 60

typedef struct { int id, arrive, emergency; } Vehicle;
typedef struct { Vehicle buf[QCAP]; int front, rear, count; } CQueue;

static void cq_init(CQueue *q)  { q->front = q->rear = q->count = 0; }
static int  cq_empty(CQueue *q) { return q->count == 0; }

/* Vehicle arrives at the rear. Returns 0 if the lane is full (vehicle dropped). */
static int cq_enqueue(CQueue *q, Vehicle v) {
    if (q->count == QCAP) return 0;
    q->buf[q->rear] = v;
    q->rear = (q->rear + 1) % QCAP;   /* wrap around */
    q->count++;
    return 1;
}
/* Vehicle leaves from the front on green. */
static Vehicle cq_dequeue(CQueue *q) {
    Vehicle v = q->buf[q->front];
    q->front = (q->front + 1) % QCAP;
    q->count--;
    return v;
}
/* Peek at the i-th vehicle from the front (0 = first in line). */
static Vehicle *cq_at(CQueue *q, int i) { return &q->buf[(q->front + i) % QCAP]; }
#endif
