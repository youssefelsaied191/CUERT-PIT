#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <atomic>
#include <cstdlib>
#include <cstring>

// ==========================================
// HARDWARE & TIMING CONSTANTS (ESP32 DevKit)
// ==========================================
constexpr uint8_t LED_PIN = 2;          // Onboard LED on standard ESP32 DevKit v1
constexpr uint32_t PWM_FREQ = 5000;      // 5 kHz PWM
constexpr uint8_t PWM_RESOLUTION = 8;    // 8-bit resolution (0-255)

constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500;
constexpr uint32_t STATUS_INTERVAL_MS = 1000;
constexpr uint8_t  QUEUE_DEPTH = 16;
constexpr size_t   MAX_LINE_BUF = 64;

// ==========================================
// DATA STRUCTURES
// ==========================================
enum class CommandType : char {
    Throttle = 'T',
    Steer    = 'S',
    Brake    = 'B',
    Ping     = 'P',
    Unknown  = 'U'
};

struct Command_t {
    CommandType type{CommandType::Unknown};
    int16_t value{0};          // 0..100 (Throttle/Brake) or -100..100 (Steer)
    uint32_t timestamp_ms{0};   // System millis() when received
};

// ==========================================
// GLOBAL STATE & IPC
// ==========================================
static QueueHandle_t xCommandQueue = nullptr;
static SemaphoreHandle_t xStateMutex = nullptr;

// Atomic timestamp for zero-latency, thread-safe watchdog reading
static std::atomic<uint32_t> g_last_command_timestamp{0};

// Strict Brake Safety State
static std::atomic<int16_t> g_current_brake{0};

// System Telemetry State
static Command_t g_last_cmd;
static uint8_t   g_current_pwm_percent = 0;
static bool      g_failsafe_active = false;

// ==========================================
// HELPER FUNCTIONS
// ==========================================
void Output_SetPWM(uint8_t percent) {
    if (percent > 100) percent = 100;
    g_current_pwm_percent = percent;

    uint32_t duty = (percent * 255) / 100;
    ledcWrite(LED_PIN, duty); // ESP32 v3.x API uses LED_PIN directly
}

void ParseAndEnqueueLine(char* line) {
    // Strip trailing \r or \n
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) {
        line[--len] = '\0';
    }
    if (len == 0) return;

    Command_t cmd;
    cmd.timestamp_ms = millis();

    // Check PING
    if (strcmp(line, "PING") == 0) {
        cmd.type = CommandType::Ping;
        cmd.value = 0;
        Serial.println("[ACK] PONG");
    } else {
        // Tokenize command string
        char* token = strtok(line, " ");
        if (token == nullptr) return;

        char* val_str = strtok(nullptr, " ");

        if (strcmp(token, "THROTTLE") == 0) {
            cmd.type = CommandType::Throttle;
        } else if (strcmp(token, "STEER") == 0) {
            cmd.type = CommandType::Steer;
        } else if (strcmp(token, "BRAKE") == 0) {
            cmd.type = CommandType::Brake;
        } else {
            // Malformed / unknown string -> Ignore safely
            return;
        }

        // Validate integer value
        if (val_str == nullptr) return;
        char* endptr = nullptr;
        long parsed_val = strtol(val_str, &endptr, 10);
        if (endptr == val_str || *endptr != '\0') {
            // Non-numeric trailing characters -> Malformed input ignored
            return;
        }
        cmd.value = static_cast<int16_t>(parsed_val);
    }

    // Update link activity timestamp immediately
    g_last_command_timestamp.store(cmd.timestamp_ms);

    // Push to Queue with Queue Full Protection for BRAKE
    if (xQueueSend(xCommandQueue, &cmd, 0) != pdPASS) {
        if (cmd.type == CommandType::Brake) {
            // Queue is full! Overwrite or clear 1 space to guarantee BRAKE is NEVER dropped
            Command_t dummy;
            xQueueReceive(xCommandQueue, &dummy, 0);
            xQueueSend(xCommandQueue, &cmd, 0);
        }
    }
}

// ==========================================
// FREERTOS TASKS
// ==========================================

// Task 1: COMMAND_RX (Priority 4 - Highest)
void Task_CommandRX(void* pvParameters) {
    static char rx_buffer[MAX_LINE_BUF];
    static size_t rx_idx = 0;

    for (;;) {
        while (Serial.available() > 0) {
            char c = static_cast<char>(Serial.read());
            if (c == '\n') {
                rx_buffer[rx_idx] = '\0';
                ParseAndEnqueueLine(rx_buffer);
                rx_idx = 0;
            } else if (c != '\r') {
                if (rx_idx < MAX_LINE_BUF - 1) {
                    rx_buffer[rx_idx++] = c;
                } else {
                    // Buffer overflow protection: flush line
                    rx_idx = 0;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task 2: ACTUATE (Priority 3 - Medium High)
void Task_Actuate(void* pvParameters) {
    Command_t cmd;

    for (;;) {
        if (xQueueReceive(xCommandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
                g_last_cmd = cmd;
                g_failsafe_active = false; // Recover from failsafe on new command

                // Update active brake latch
                if (cmd.type == CommandType::Brake) {
                    g_current_brake.store(cmd.value);
                }

                // STRICT SAFETY OVERRIDE:
                // As long as brake > 0, force PWM output to 0% regardless of THROTTLE
                if (g_current_brake.load() > 0) {
                    Output_SetPWM(0);
                } else if (cmd.type == CommandType::Throttle) {
                    Output_SetPWM(static_cast<uint8_t>(cmd.value));
                }

                // Log execution
                Serial.print("[ACTUATE] Executed Cmd Type: ");
                Serial.print(static_cast<char>(cmd.type));
                Serial.print(" | Value: ");
                Serial.print(cmd.value);
                Serial.print(" | Active Brake: ");
                Serial.print(g_current_brake.load());
                Serial.print(" | PWM Output: ");
                Serial.print(g_current_pwm_percent);
                Serial.println("%");

                xSemaphoreGive(xStateMutex);
            }
        }
    }
}

// Task 3: WATCHDOG / FAIL-SAFE (Priority 2 - Low, Always Runs)
void Task_Watchdog(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool prev_failsafe_state = false;

    for (;;) {
        uint32_t now = millis();
        uint32_t last_ts = g_last_command_timestamp.load();

        if (last_ts > 0 && (now - last_ts > WATCHDOG_TIMEOUT_MS)) {
            if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
                g_failsafe_active = true;

                if (!prev_failsafe_state) {
                    Serial.println("LINK LOST, failing safe");
                    prev_failsafe_state = true;
                }

                // Safe Idle Blink Pattern (5 Hz -> Toggle every 100ms)
                static bool toggle = false;
                toggle = !toggle;
                Output_SetPWM(toggle ? 100 : 0);

                xSemaphoreGive(xStateMutex);
            }
        } else {
            prev_failsafe_state = false;
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

// Task 4: STATUS (Priority 1 - Lowest)
void Task_Status(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print("[STATUS] Uptime: ");
            Serial.print(millis());
            Serial.print(" ms | Last Cmd: ");
            Serial.print(static_cast<char>(g_last_cmd.type));
            Serial.print(":");
            Serial.print(g_last_cmd.value);
            Serial.print(" | Brake State: ");
            Serial.print(g_current_brake.load());
            Serial.print(" | PWM: ");
            Serial.print(g_current_pwm_percent);
            Serial.print("% | Link: ");
            Serial.println(g_failsafe_active ? "FAIL-SAFE (LINK LOST)" : "ACTIVE");

            xSemaphoreGive(xStateMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(STATUS_INTERVAL_MS));
    }
}

// ==========================================
// MAIN SETUP & LOOP
// ==========================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000); // Wait for serial console initialization

    // Attach PWM to onboard LED (Pin 2) using modern ESP32 v3.x API
    ledcAttach(LED_PIN, PWM_FREQ, PWM_RESOLUTION);

    // Initialize IPC objects
    xCommandQueue = xQueueCreate(QUEUE_DEPTH, sizeof(Command_t));
    xStateMutex   = xSemaphoreCreateMutex();

    if (xCommandQueue == nullptr || xStateMutex == nullptr) {
        Serial.println("[FATAL] FreeRTOS static allocation failed!");
        while (1);
    }

    // Create RTOS Tasks
    xTaskCreate(Task_CommandRX, "COMMAND_RX", 3072, nullptr, 4, nullptr);
    xTaskCreate(Task_Actuate,   "ACTUATE",    2048, nullptr, 3, nullptr);
    xTaskCreate(Task_Watchdog,  "WATCHDOG",   2048, nullptr, 2, nullptr);
    xTaskCreate(Task_Status,    "STATUS",     2048, nullptr, 1, nullptr);

    Serial.println("[SYSTEM] RTOS Sensor-to-CAN Bridge Online.");
}

void loop() {
    // Managed entirely by FreeRTOS scheduler
    vTaskDelete(NULL);
}