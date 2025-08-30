#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "app_system.h"
#include "app_uart.h"
#include "app_tasks.h"
#include "app_proto.h"

/**
 * @brief       Main application entry point
 * @param       None
 * @retval      Never returns (FreeRTOS scheduler takes control)
 */
int main(void) {
    System_Init();      // clock 72MHz, bus prescalers
    relocate_vtor();    // VTOR -> APP_BASE (extremely important)
    uart_init();        // USART1/3 + IRQ + ring buffer

    // (Optional) initialize shared resources for tasks
    app_tasks_preinit(); // create queue, shared variables if needed

    // Create tasks
    xTaskCreate(blink_task,    "blink",   128, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(logger_task,   "logger",  256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(uart_cmd_task, "uartcmd", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(producer_task, "prod",    128, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(consumer_task, "cons",    128, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    // If we reach here: insufficient heap or incorrect config
    for(;;) {}
}
