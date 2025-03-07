#include <cfloat>

class PID
{
public:
    // 1. pars need to be set
    // 1.1 manually
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float output_min = FLT_MIN;
    float output_max = FLT_MAX;
    float integral_limit = FLT_MAX;
    // 1.2 automatically by program
    float setpoint = 0;
    float feedback = 0;

    // just for remember output
    float output = 0;

    // debug
    bool printVars = false;

    // inner vars
    float error = 0;
    float error_last = 0;
    float error_last_last = 0;
    float integral = 0;

    void setPID(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }
    void setLimit(float output_min=FLT_MIN, float output_max=FLT_MAX, float integral_limit=FLT_MAX) {
        this->output_min = output_min;
        this->output_max = output_max;
        this->integral_limit = integral_limit;
    }

    float CalOutput_Pos() {
        error = setpoint - feedback;
        if (setpoint == 0) {
            integral = 0;  // clear
            error_last = 0;
        }

        integral += error;
        if (integral > integral_limit) integral = integral_limit;
        else if (integral < -integral_limit) integral = -integral_limit;

        output = kp * error + ki * integral + kd * (error - error_last);
        if (printVars) {
            printVars = false;
            Serial.printf("kp=%.3f, error=%.3f, ki=%.3f, integral=%.3f, kd=%.3f, error_last=%.3f, output=%.3f\n", kp, error, ki, integral, kd, error_last, output);
        }
        if (output > output_max) output = output_max;
        else if (output < output_min) output = output_min;

        error_last = error;
        return output;
    }

    float CalOutput_Inc() {
        error = setpoint - feedback;

        float output_dt = kp * (error - error_last) + ki * error + kd * (error - 2*error_last + error_last_last);
        output += output_dt;
        // Serial.printf("E=%.3f, EL=%.3f, ELL=%.3f, OD=%.3f, O=%.3f\n", error, error_last, error_last_last, output_dt, output);
        if (output > output_max) output = output_max;
        else if (output < output_min) output = output_min;

        error_last_last = error_last;
        error_last = error;

        // if (setpoint == 0) {
        //     output = 0;  // clear
        // }
        return output;
    }

    // --- following for debug
    bool isNotSame_Assign_Main(const PID &other) {
        bool ret = !isSame_Main(other);
        if (ret) assign_Main(other);
        return ret;
    }
    bool isNotSame_Assign_IE(const PID &other) {
        bool ret = !isSame_IE(other);
        if (ret) assign_IE(other);
        return ret;
    }

    bool isSame_Main(const PID &other) {
        if (setpoint == other.setpoint && feedback == other.feedback && output == other.output)
            return true;
        else
            return false;
    }
    bool isSame_IE(const PID &other) {
        if (integral == other.integral && error_last == other.error_last && error_last_last == other.error_last_last)
            return true;
        else
            return false;
    }
    void assign_Main(const PID &other) {
        setpoint = other.setpoint;
        feedback = other.feedback;
        output = other.output;
    }
    void assign_IE(const PID &other) {
        integral = other.integral;
        error_last = other.error_last;
        error_last_last = other.error_last_last;
    }

    String getString(int prefix) {
        char tmp[64];
        itoa(prefix, tmp, 10);
        return getString(tmp);
    }
    String getString(const char *prefix) {
        String ret = String() + prefix + "_setpoint:" + String(setpoint, 3) + ", " +
            prefix + "_feedback:" + String(feedback, 3) + ", " +
            prefix + "_output:" + String(output, 3);
        // Serial.println(ret);
        return ret;
    }
    String getShortString(int prefix) {
        char tmp[64];
        itoa(prefix, tmp, 10);
        return getShortString(tmp);
    }
    String getShortString(const char *prefix) {
        char tmp[512];
        sprintf(tmp, "%.3f,%.3f,%.3f", setpoint, feedback, output);
        String ret(tmp);
        // Serial.println(ret);
        return ret;
    }
    String getJson(int prefix) {
        char tmp[64];
        itoa(prefix, tmp, 10);
        return getJson(tmp);
    }
    String getJson(const char *prefix) {
        char tmp[512];
        sprintf(tmp, R"({"P_setpoint":%.3f,"P_feedback":%.3f,"P_output":%.3f})", setpoint, feedback, output);
        String ret(tmp);
        ret.replace("P", prefix);
        // Serial.println(ret);
        return ret;
    }

    String getPlotString(int prefix) {
        char tmp[64];
        itoa(prefix, tmp, 10);
        return getPlotString(tmp);
    }
    String getPlotString(const char *prefix) {
        return ">" + getString(prefix);
    }

    String getDataString_IE(int prefix) {
        char tmp[64];
        itoa(prefix, tmp, 10);
        return getDataString_IE(tmp);
    }
    String getDataString_IE(const char *prefix) {
        char tmp[512];
        sprintf(tmp, "\1_I:%.3f, \1_EL:%.3f, \1_ELL:%.3f", integral, error_last, error_last_last);
        String ret(tmp);
        ret.replace("\1", prefix);
        return ret;
    }
};
