/* Standalone encoder readout over ST-LINK USB; no actuator or Pi-link code. */
#include <zephyr/kernel.h>
#include "encoder.h"

int main(void)
{
    int rc = encoders_init();
    printk("ENCODER ONLY: left PA0/PA1 (TIM2), right PB6/PB7 (TIM4).\n");
    printk("Encoder init=%d; CPR=%u, nominal value checked against ten hand-turned revolutions.\n",
           rc, COUNTS_PER_WHEEL_REV);
    printk("Use encoder watch / encoder stop for live output; encoder status / zero / sign remain available.\n");
    printk("No Pi needed; motor PWM, servo PWM and USART1 are disabled in this profile.\n");
    return 0;
}
