#include <iostream>
#include <unistd.h>
#include "ADS1115.hpp"

int main() {

    ADS1115 ads(ADS1115_ADDRS);

    ads.init();

    std::cout << "ADS1115 Initialized in Continuous Mode\n";

    while (true) {

        int16_t adcValue = ads.readADC();

        double voltage = (adcValue * 4.096) / 32768.0;

        std::cout << "ADC Value: " << adcValue << " | Voltage: " << voltage << " V\n";

        usleep(500000);  // 500ms delay
    }

    return 0;
}
