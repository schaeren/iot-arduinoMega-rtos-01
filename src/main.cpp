#define DEBUG

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal.h>
#include <semphr.h>
#include <queue.h>
#include <portable.h>

// Stack sizes for tasks (experimentally determined approximately, multiplied by 2)
const configSTACK_DEPTH_TYPE stackTaskBlinky = 70 * 2;
const configSTACK_DEPTH_TYPE stackTaskAnalogInput = 80 * 2;
const configSTACK_DEPTH_TYPE stackTaskDigitalInput = 80 * 2;
const configSTACK_DEPTH_TYPE stackTaskLogWriter = 110 * 2;
const configSTACK_DEPTH_TYPE stackTaskLcd = 100 * 2;

long __heapSize = 0;
void myTraceMalloc(size_t size, void* ptr)
{
#ifdef DEBUG
    __heapSize += size;
    Serial.print("pvPortMalloc() size=");
    Serial.print(size);
    Serial.print(", total allocated bytes=");
    Serial.print(__heapSize);
    Serial.print(", ptr=");
    Serial.println((long)ptr);
#endif
}
void myTraceFree(void* ptr)
{
#ifdef DEBUG
    Serial.print("vPortFree()    ptr=");
    Serial.println((long)ptr);
#endif
}
void vApplicationMallocFailedHook( void )
{
    Serial.println("*** MALLOC FAILED ***");
}
#undef configCHECK_FOR_STACK_OVERFLOW
#define configCHECK_FOR_STACK_OVERFLOW = 2
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
    // Seems not top work with Arduino (?)
    Serial.print("*** STACK OVERFLOW IN TASK ");
    Serial.print((char*)pcTaskName);
    Serial.println(" ***");
}

uint8_t redLedPin = 4;          // Output for red LED
uint8_t yellowLedPin = 3;       // Output for yellow LED
uint8_t greenLedPin = 2;        // Output for green LED
uint8_t redPotPin = A0;         // ADC input for potentiometer for red LED
uint8_t yellowPotPin = A1;      // ADC input for potentiometer for yellow LED
uint8_t voltageRefPin = A2;     // ADC input connected to 3.3V reference
uint8_t stopButtonPin = 10;     // Input for button 'stop stop' of green LED
uint8_t slowButtonPin = 11;     // Input for button 'slow blicking' of green LED
uint8_t fastButtonPin = 12;     // Input for button 'fast blinking' of green LED

// Configuration
const long delayBeforeAdcRead = 2; // Wait time before ADC is read in ticks (1 tick = 15ms)
                                   // Remark: A short recovery time between the reading of
                                   // different ADC channels leads to more stable results.
const long minDelay = portTICK_PERIOD_MS; // Min. delay = min. half period time for LED flashing in ms
const long maxDelay = 1000;     // Max. delay = max. half period time for LED flashing in ms
const long minDelayDiff = 5;    // Min. difference so that delay change is written logQueue in ms
const long slowDelay = 500;     // Delay for slow blinking of green LED (1 Hz)
const long fastDelay = 50;      // Delay for faslt blinking of green LED (10 Hz)

// Half period time for LEDs, access is protected by mutex
SemaphoreHandle_t delaysMutex;
int16_t delayRed = -1;          // ms
int16_t delayYellow = -1;       // ms
int16_t delayGreen = -1;        // ms

// Messag queue for log writer
QueueHandle_t logQueue;
UBaseType_t logQueueLen = 3;
typedef char LogQueueEntry_t[50];

// Messag queue used to send messages for interrupt handler to TaskDigitalInput
QueueHandle_t greenQueue;
UBaseType_t greenQueueLen = 2;
enum GreenQueueEntry_t { stop, slow, fast, undefined };

// struct used for task parameters
struct TaskParams_t {
    const char* color;
    uint8_t ledPin;
    int16_t* delayTime;
};

TaskParams_t taskRedBlinkyParams = {
    .color = "red",
    .ledPin = redLedPin,
    .delayTime = &delayRed
};
TaskParams_t taskYellowBlinkyParams = {
    .color = "yellow",
    .ledPin = yellowLedPin,
    .delayTime = &delayYellow
};
TaskParams_t taskGreenBlinkyParams = {
    .color = "green",
    .ledPin = greenLedPin,
    .delayTime = &delayGreen
};

void sendTextToLogQueue (const char* format, ...);

void TaskAnalogInput([[maybe_unused]] void *pvParameters )
{
    int16_t maxAdcValue = 0;    // ADC readout value for 3.3V
    int16_t _oldDelayRed = -1;
    int16_t _oldDelayYellow = -1;

    sendTextToLogQueue("Task AnalogIn started, input pins: %d, %d", redLedPin, yellowLedPin);
    pinMode(redPotPin, INPUT);
    pinMode(yellowPotPin, INPUT);
    maxAdcValue = analogRead(voltageRefPin);

    // Temporary local variables for delay times [ms], at the end of a loop
    // these values will be written to global variables, too.
    int16_t _delayRed, _delayYellow = 0;

    for (;;) {
        // Get delay for red LED
        vTaskDelay(delayBeforeAdcRead);
        int potRedValue = analogRead(redPotPin);
        _delayRed = map(potRedValue, 0, maxAdcValue, minDelay, maxDelay);

        // Get delay for yellow LED
        vTaskDelay(delayBeforeAdcRead);
        int potYellowValue = analogRead(yellowPotPin);
        _delayYellow = map(potYellowValue, 0, maxAdcValue, minDelay, maxDelay);

        // If at least one delay time [ms] has changed by more than minDelayDiff milliseconds,
        // write it to global variables, access is protected by mutex.
        if (abs(_delayRed - _oldDelayRed) > minDelayDiff || abs(_delayYellow - _oldDelayYellow) > minDelayDiff) {
            _oldDelayRed = _delayRed;
            _oldDelayYellow = _delayYellow;
            if (xSemaphoreTake(delaysMutex, portMAX_DELAY) == pdTRUE) {
                delayRed = _delayRed;
                delayYellow = _delayYellow;
                xSemaphoreGive(delaysMutex);
            }
            sendTextToLogQueue("red: %dms, yellow: %dms.", _delayRed, _delayYellow);
        }
    }
}

void TaskDigitalInput([[maybe_unused]] void *pvParameters )
{
    int16_t _oldDelayGreen = -1;

    sendTextToLogQueue("Task DigitalIn started, input pins: %d, %d, %d",
        stopButtonPin, slowButtonPin, fastButtonPin);
    
    for (;;) {
        GreenQueueEntry_t cmd;
        if (xQueueReceive(greenQueue, &cmd, portMAX_DELAY) != pdTRUE) {
            Serial.println("TaskDigitalInput");
        }
        int16_t _delayGreen = -1;
        switch (cmd) {
            case stop:
                _delayGreen = -1;
                break;
            case slow:
                _delayGreen = slowDelay;
                break;
            case fast:
                _delayGreen = fastDelay;
                break;
            default:
                break;
        }
        // If delay time [ms] has changed, write it to global variables,
        // access is protected by mutex.
        if (_delayGreen != _oldDelayGreen) {
            _oldDelayGreen = _delayGreen;
            if (xSemaphoreTake(delaysMutex, portMAX_DELAY) == pdTRUE) {
                delayGreen = _delayGreen;
                xSemaphoreGive(delaysMutex);
            }
            sendTextToLogQueue("green: %dms.", _delayGreen);
        }
        vTaskDelay(5);  // wait 75 ms
    }
}

void TaskBlinky([[maybe_unused]] void *pvParameters )
{
    TaskParams_t* param = (TaskParams_t *) pvParameters;
    const char* color = param->color;
    uint8_t ledPin = param->ledPin;
    sendTextToLogQueue("Task Blinky for %s started, output pin %d", color, ledPin);

    pinMode(ledPin, OUTPUT);

    for (;;) {
        // Get delay time [ms] from global varibale, access is protected by mutex.
        int16_t _delayTime;
        if (xSemaphoreTake(delaysMutex, portMAX_DELAY) == pdTRUE) {
            _delayTime = *(param->delayTime);
            xSemaphoreGive(delaysMutex);
        }

        if (_delayTime != -1) {
            digitalWrite(ledPin, !digitalRead(ledPin));
            long _delayTicks = round((0.0 + _delayTime) / portTICK_PERIOD_MS);
            vTaskDelay(_delayTicks);
        } else {
            digitalWrite(ledPin, LOW);
            vTaskDelay(1); // 16 ms
        }
    }
}

void TaskLogWriter([[maybe_unused]] void *pvParameters )
{
    LogQueueEntry_t buffer;

    for (;;) {
        if (xQueueReceive(logQueue, buffer, portMAX_DELAY) == pdTRUE) {
            Serial.println(buffer);
        } else {
            Serial.println("Task TaskLogWriter: Failed to receive from queue.");
        }
    }
}

// Mapping of LCD pins to Arduino Mega pins.
const int rs = 50, en = 48, d4 = 46, d5 = 44, d6 = 42, d7 = 40;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void TaskLcd([[maybe_unused]] void *pvParameters)
{
    int16_t _delayRed, _delayYellow,_delayGreen = 0;
    int16_t _lastDelayRed, _lastDelayYellow,_lastDelayGreen = 0;
    sendTextToLogQueue("Task TaskLcd started");

    // Set up the LCD's number of columns and rows.
    lcd.begin(16, 2);
    // Print a message to the LCD.
    lcd.print("Starting...");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 s

    for (;;) {
        if (xSemaphoreTake(delaysMutex, portMAX_DELAY) == pdTRUE) {
            _delayRed = delayRed;
            _delayYellow = delayYellow;
            _delayGreen = delayGreen;
            xSemaphoreGive(delaysMutex);
        }
        if (_delayRed != _lastDelayRed || _delayYellow != _lastDelayYellow || _delayGreen != _lastDelayGreen) {
            _lastDelayRed = _delayRed;
            _lastDelayYellow = _delayYellow;
            _lastDelayGreen = _delayGreen;
            char text[17];
            sprintf(text, "r=%d, y=%d", _delayRed, _delayYellow);
            lcd.clear();
            lcd.print(text);
            lcd.setCursor(0, 1);
            sprintf(text, "g=%d", _delayGreen);
            lcd.print(text);
        }
        vTaskDelay(5); // 80 ms
    }
}

// Interrupt handler for 'Port Change Interrupt' inputs 10..12
ISR(PCINT0_vect)
{
    static long lastUpdateTime = 0;
    const long debounceDuration = 20; // ms
    static long lastStopButtonState = HIGH;
    static long lastSlowButtonState = HIGH;
    static long lastFastButtonState = HIGH;

    long stopButtonState = digitalRead(stopButtonPin);
    long slowButtonState = digitalRead(slowButtonPin);
    long fastButtonState = digitalRead(fastButtonPin);

    long now = millis();
    if ((stopButtonState != lastStopButtonState ||
         slowButtonState != lastSlowButtonState ||
         fastButtonState != lastFastButtonState) && now - debounceDuration > lastUpdateTime) {
        lastStopButtonState = stopButtonState;
        lastSlowButtonState = slowButtonState;
        lastFastButtonState = fastButtonState;
        lastUpdateTime = now;

        GreenQueueEntry_t cmd = undefined;
        if (stopButtonState == LOW) {
            cmd = stop;
        } else if (slowButtonState == LOW) {
            cmd = slow;
        } else if (fastButtonState == LOW) {
            cmd = fast;
        }
        if (cmd != undefined) {
            BaseType_t higherPriorityTaskWoken;
            if (xQueueSendToFrontFromISR(greenQueue, &cmd, &higherPriorityTaskWoken) != pdTRUE) {
                Serial.println("ISR: Failed to send command to queue 'greenQueue'.");
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.print("configMAX_PRIORITIES=");
    Serial.println(configMAX_PRIORITIES);
    UBaseType_t taskPriority = configMAX_PRIORITIES - 1 ;

    // Create/initialize mutex (semaphore).
    delaysMutex = xSemaphoreCreateMutex();

    // Create queues
    logQueue = xQueueCreate(logQueueLen, sizeof(LogQueueEntry_t));
    if (logQueue == NULL) {
        Serial.println("Failed to create queue 'logQueueLen'.");
    }
    greenQueue = xQueueCreate(greenQueueLen, sizeof(GreenQueueEntry_t));
    if (greenQueue == NULL) {
        Serial.println("Failed to create queue 'greenQueue'.");
    }

    pinMode(stopButtonPin, INPUT_PULLUP);
    pinMode(slowButtonPin, INPUT_PULLUP);
    pinMode(fastButtonPin, INPUT_PULLUP);

    // Enable 'Pin Change Interrupts' for inputs 10..12
    noInterrupts();
    // PCICR – Pin Change Interrupt Control Register
    PCICR |= 1 << PCIE0; // Enable pin change irq from port B (i.e. Arduino i/o 10..13)
    // PCMSK0 - Pin Change Mask Register 0
    PCMSK0 |= 1 << PCINT4 | 1 << PCINT5 | 1 << PCINT6; // Enable PCINT6..6 (i.e. Arduino i/o 10..12)
    interrupts();

    // Create/start tasks.
    xTaskCreate(
        TaskLogWriter,          // task function
        "TaskLogWriter",        // task name
        stackTaskLogWriter,     // stack size
        NULL,                   // paramater for task
        taskPriority + 1,       // task priority (higher than other tasks!)
        NULL );                 // task handle

    xTaskCreate(
        TaskAnalogInput,        // task function
        "TaskAnalogInput",      // task name
        stackTaskAnalogInput,   // stack size
        NULL,                   // paramater for task
        taskPriority,           // task priority
        NULL );                 // task handle

    xTaskCreate(
        TaskDigitalInput,       // task function
        "TaskDigitalInput",     // task name
        stackTaskDigitalInput,  // stack size
        NULL,                   // paramater for task
        taskPriority,           // task priority
        NULL );                 // task handle

    xTaskCreate(
        TaskBlinky,             // task function
        "TaskBlinky-red",       // task name
        stackTaskBlinky,        // stack size
        &taskRedBlinkyParams,   // paramater for task
        taskPriority,           // task priority
        NULL );                 // task handle

    xTaskCreate(
        TaskBlinky,             // task function
        "TaskBlinky-yellow",    // task name
        stackTaskBlinky,        // stack size
        &taskYellowBlinkyParams,// paramater for task
        taskPriority,           // task priority
        NULL );                 // task handle

    xTaskCreate(
        TaskBlinky,             // task function
        "TaskBlinky-green",     // task name
        stackTaskBlinky,        // stack size
        &taskGreenBlinkyParams, // paramater for task
        taskPriority,           // task priority
        NULL );                 // task handle

    xTaskCreate(
        TaskLcd,                // task function
        "TaskLcd",              // task name
        stackTaskLcd,           // stack size
        NULL,                   // paramater for task
        taskPriority - 1,       // task priority
        NULL );                 // task handle

     vTaskStartScheduler();
     Serial.println("Failed to start RTOS scheduler!");
}

void loop() {
    // idle task
}

void sendTextToLogQueue (const char* format, ...) {
    va_list argptr;
    va_start(argptr, format);
    LogQueueEntry_t sendBuffer;
    vsprintf(sendBuffer, format, argptr);
    va_end(argptr);
    if (xQueueSend(logQueue, sendBuffer, portMAX_DELAY) != pdPASS) {
        pinMode(greenLedPin, OUTPUT);
        digitalWrite(greenLedPin, HIGH);
        Serial.println("sendTextToLogQueue() failed!");
    }
}