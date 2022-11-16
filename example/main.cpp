#include "stdio.h"
#include "pico/stdlib.h"
#include "DPS310.hpp"

int main() {
    stdio_init_all();

    float press;
    float temp;

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    sleep_ms(1000);

    DPS310 dps310 = DPS310(i2c1,2,3,100*1000);
    while (true) {
        dps310.measurement();
        press = dps310.getPressure();
        temp = dps310.getTemperature();

        printf("Pressure:%f [hPa] Temperature:%f [°C]\n",press/100,temp);

        gpio_put(LED_PIN,!gpio_get(LED_PIN));

        sleep_ms(1000);
    }
}