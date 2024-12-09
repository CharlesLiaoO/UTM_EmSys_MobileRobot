#include <cfloat>

class PID
{
public:
    // 1. pars need to be set
    // 1.1 manually
    float Kp, Ki, Kd;
    float output_max;
    float integral_limit;
    // 1.2 automatically by program
    float target;
    float actual;

    // just for remember output
    float output;

    // inner vars
    float error;
    float error_last;
    float integral;

    void setPID(float p, float i, float d, float output_max=FLT_MAX, float integral_limit=FLT_MAX) {
        Kp = p;
        Ki = i;
        Kd = d;
        this->output_max = output_max;
        this->integral_limit = integral_limit;
    }

    float CalOutput_Pos() {
        error = target - actual;

        integral += error;
        if (integral > integral_limit)
            integral = integral_limit;
        else if (integral < -integral_limit)
            integral = -integral_limit;

        output = Kp * error + Ki * integral + Kd * (error - error_last);
        if (output > output_max) output = output_max;

        error_last = error;
        return output;
    }

    // String getDataString() {
    //     char tmp[512];
    //     sprintf(tmp, "%f,%f,%f", target, actual, output);
    //     String ret(tmp);
    //     return ret;
    // }
    String getPlotString(const char *prefix) {
        char tmp[512];
        sprintf(tmp, ">\1-target:%f,\1-actual:%f,\1-output:%f", target, actual, output);  // Teleplot in VSCode
        // sprintf(tmp, "\1-target:%f,\1-actual:%f,\1-output:%f", target, actual, output);  // Arduino IDE
        String ret(tmp);
        ret.replace("\1", prefix);
        return ret;
    }
};