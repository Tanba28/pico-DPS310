#include "stdio.h"
#include "pico/stdlib.h"
#include "DPS310.hpp"
#include "pico_i2c.hpp"
#include "task_base.hpp"

class DPS310Task : public TaskBase{
    public:
        DPS310Task():TaskBase("dps310_task",3,4096){
        }
        void task(){
            gpio_init(18);
            gpio_set_dir(18, GPIO_OUT);

            float press;
            float temp;
            PicoDmaI2C::createInstance(i2c1,2,3,100*1000);
            i2c = PicoDmaI2C::getInstance();
            DPS310 dps310 = DPS310(i2c);

            for(;;){
                dps310.measurement();
                press = dps310.getPressure();
                temp = dps310.getTemperature();

                printf("Pressure:%f [hPa] Temperature:%f [°C]\n",press/100,temp);

                gpio_put(18,!gpio_get(18));

                vTaskDelay(10);
            }
        }
    private:
        PicoDmaI2C *i2c;
};
int main() {
    sleep_ms(1000);
    stdio_init_all();

    // float press;
    // float temp;

    // const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    // gpio_init(LED_PIN);
    // gpio_set_dir(LED_PIN, GPIO_OUT);

    // sleep_ms(1000);
    // // PicoI2C *i2c = new PicoI2C(i2c1,2,3,100*1000);

    // DPS310 dps310 = DPS310(i2c1,2,3,100*1000);
    // DPS310 dps310 = DPS310(i2c);

    DPS310Task task = DPS310Task();
    task.createTask();

    vTaskStartScheduler();
    while (true) {
        // dps310.measurement();
        // press = dps310.getPressure();
        // temp = dps310.getTemperature();

        // printf("Pressure:%f [hPa] Temperature:%f [°C]\n",press/100,temp);

        // gpio_put(18,!gpio_get(18));

        // sleep_ms(1000);
    }
}