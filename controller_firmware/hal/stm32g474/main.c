#include "hal.h"
#include "app.h"

int main(void)
{
    hal_init();
    app_init();
    hal_start();

    uint32_t last = hal_millis();
    for (;;) {
        app_poll();
        uint32_t now = hal_millis();
        if (now != last) {
            last = now;
            app_tick();
        }
    }
}
