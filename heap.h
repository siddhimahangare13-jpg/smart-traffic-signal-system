/* heap.h - Min-heap priority queue. Smallest key pops first.
   The scheduler stores key = -priority, so the highest-priority lane wins. */
#ifndef HEAP_H
#define HEAP_H
typedef struct { double key; int lane; } HNode;
typedef struct { HNode a[8]; int n; } MinHeap;

static void h_init(MinHeap *h) { h->n = 0; }

static void h_push(MinHeap *h, double key, int lane) {
    int i = h->n++;
    h->a[i].key = key; h->a[i].lane = lane;
    while (i > 0) {                               /* sift up */
        int p = (i - 1) / 2;
        if (h->a[p].key <= h->a[i].key) break;
        HNode t = h->a[p]; h->a[p] = h->a[i]; h->a[i] = t;
        i = p;
    }
}
static HNode h_pop(MinHeap *h) {
    HNode top = h->a[0];
    h->a[0] = h->a[--h->n];
    int i = 0;
    for (;;) {                                    /* sift down */
        int l = 2 * i + 1, r = l + 1, m = i;
        if (l < h->n && h->a[l].key < h->a[m].key) m = l;
        if (r < h->n && h->a[r].key < h->a[m].key) m = r;
        if (m == i) break;
        HNode t = h->a[m]; h->a[m] = h->a[i]; h->a[i] = t;
        i = m;
    }
    return top;
}
#endif
