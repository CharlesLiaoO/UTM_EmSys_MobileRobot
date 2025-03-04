#include <Print.h>
#include <WiFiClient.h>

class SseSer: public Print
{
private:
    WiFiClient *sseClient;
public:
    SseSer(WiFiClient *sseClient);
    ~SseSer() {};
    size_t write(uint8_t) { return 0; }

    size_t write(const uint8_t *buffer, size_t size) {
        if (!sseClient)
            return 0;
        sseClient->print("data: ");  //DO NOT forget data: ...
        size_t n = sseClient->write(buffer, size);
        sseClient->println();  // end data
        return n;
    }

    void begin(int n) {};
};

inline SseSer::SseSer(WiFiClient *sseClient)
{
    this->sseClient = sseClient;
}
