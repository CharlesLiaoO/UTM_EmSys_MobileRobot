#include <Arduino.h>
#include <cmath>

class VelocityPlan {
    float jm = 9999;
    float am = 9999;
    float vm = 9999;
    // float dm = 9999;

    float js = 9999;
    float as = 9999;
    float vs = 9999;
    // float ds = 9999;

    int state = 0;  // 0: stop; 1: a+(0->as, j=js); 2: a=as; 3: a-(as->0); 4: a=0,v=vs; 5: a-(0->ds, j=-js); 6: a=ds; 7: a+(ds->0)
    uint t = 0;  // time in ms
    uint tj = 0;  // const jerk time
    uint ta = 0;  // const acc time
    uint tv = 0;  // const vel time

    uint tk_sta[7];  // state tick

    float j = 9999;
    float a = 9999;
    float v = 9999;

    float ps = 0;
    float pf = 0;
    float p = 0;

    void MoveRel(float distance) {
        ps = p + distance;

        int tj = as / js;  // time of (constant) jerk
        int ta = vs / as;
        int tv = (distance / vs) - (ta + tj);

        if (tv < 0) {
            tv = 0;
            vs = std::sqrt(distance * js); // re-calculate vs
            ta = vs / as;
        }

        int tTotal = 2 * (tj + ta) + tv; // 总时间

        tk_sta[0] = millis();
        tk_sta[1] = tk_sta[0] + tj;
        tk_sta[2] = tk_sta[1] + ta;
        tk_sta[3] = tk_sta[2] + tj;
        tk_sta[4] = tk_sta[3] + tv;
        tk_sta[5] = tk_sta[4] + tj;
        tk_sta[6] = tk_sta[5] + ta;
        tk_sta[7] = tk_sta[6] + tj;
        state = 1;  // start moving
    }

    void Move() {
        if (state == 0) {
            return;
        }
        uint t = millis();
        int dt;
        if (dt = t - tk_sta[1]; dt < 0) {
            state = 1;
            dt = -dt;
            a = j * dt;
            v += a * dt;
        } else if (dt = t - tk_sta[2]; dt < 0) {
            state = 2;
            dt = -dt;
            a = as;
            v += a * dt;
        } else if (dt = t - tk_sta[3]; dt < 0) {
            state = 3;
            dt = -dt;
            a = as - j * dt;
            v += a * dt;
        } else if (dt = t - tk_sta[4]; dt < 0) {
            state = 4;
            dt = -dt;
            a = 0;
            v = vs;
        } else if (dt = t - tk_sta[5]; dt < 0) {
            state = 5;
            dt = -dt;
            a = -j * dt;
            v += a * dt;  // a<0
        } else if (dt = t - tk_sta[6]; dt < 0) {
            state = 6;
            dt = -dt;
            a = -as;
            v += a * dt;
        } else if (dt = t - tk_sta[7]; 0 < dt && dt < tj) {
            state = 7;
            dt = -dt;
            a = -as + j * dt;
            v += a * dt;
        } else {
            state = 0;
            a = 0;
            v = 0;
            // p = ps;
        }
        p += v * dt;
    }
};