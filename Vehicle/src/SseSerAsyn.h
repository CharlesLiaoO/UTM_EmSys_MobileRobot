#include <Print.h>
#include <AsyncEventSource.h>

class SseSerAsyn: public Print
{
private:
    AsyncEventSource *sseClient;
public:
    SseSerAsyn(AsyncEventSource *sseClient) {
        this->sseClient = sseClient;
    }
    ~SseSerAsyn() {};
    size_t write(uint8_t) { return 0; }

    size_t write(const uint8_t *buffer, size_t size) {
        if (!sseClient) {
            Serial.write(buffer, size);
        } else {
            String message = String((const char *)buffer, size);
            sseClient->send(message.c_str());
            // Serial.write(buffer, size);
        }
        return 0;
    }

    void begin(int n) {};
};
