#include "M5StickCPlus.h"
#include <string.h>

const bool showCube = true;
static constexpr bool DEVICE_IS_SENSOR_NODE = false;
static constexpr bool DEVICE_HAS_RADIO_LINK = true;

struct CommsConfig
{
    uint32_t imu_link_baud;
    uint32_t imu_link_send_interval_ms;
    uint32_t imu_link_timeout_ms;
};

static constexpr CommsConfig comms_unthrottled_115200_air = {     // measured max 37 messages per second tx, limited by screen refresh.
                    .imu_link_baud=115200, 
                    .imu_link_send_interval_ms=0, 
                    .imu_link_timeout_ms=250};

static constexpr CommsConfig comms_unthrottled_9600_air = {     // measured max 37 messages per second tx, limited by screen refresh.
                    .imu_link_baud=9600, 
                    .imu_link_send_interval_ms=0, 
                    .imu_link_timeout_ms=250};

static constexpr CommsConfig comms_20_Hz_9600_air = {           // measure 19 messages per second tx
                    .imu_link_baud=9600, 
                    .imu_link_send_interval_ms=50,
                    .imu_link_timeout_ms=250};

static constexpr CommsConfig comms_unthrottled_4800_air = { // measured max 37 messages per second tx, limited by screen refresh.
                    .imu_link_baud=4800, 
                    .imu_link_send_interval_ms=0,
                    .imu_link_timeout_ms=250};

static constexpr CommsConfig comms_unthrottled_2400_air = { // measured max 30 messages per second tx
                    .imu_link_baud=2400, 
                    .imu_link_send_interval_ms=0,
                    .imu_link_timeout_ms=250};

static constexpr CommsConfig comms_20_Hz_2400_air = { // measured max 20 messages per second tx
                    .imu_link_baud=2400, 
                    .imu_link_send_interval_ms=50,
                    .imu_link_timeout_ms=250};

static constexpr CommsConfig comms_unthrottled_1200_water = { // max 15 messages per second tx
                    .imu_link_baud=1200, 
                    .imu_link_send_interval_ms=0,
                    .imu_link_timeout_ms=250};

static constexpr CommsConfig comms_12_Hz_1200_water = { // max 12 messages per second tx
                    .imu_link_baud=1200, 
                    .imu_link_send_interval_ms=83, 
                    .imu_link_timeout_ms=250};

static constexpr CommsConfig comms_10Hz_1200_water = { // max 10 messages per second tx
                    .imu_link_baud=1200, 
                    .imu_link_send_interval_ms=100, 
                    .imu_link_timeout_ms=250};


static constexpr CommsConfig active_comms_config = comms_12_Hz_1200_water;
typedef struct {
    double x;
    double y;
    double z;
} point_3d_t;

typedef struct {
    point_3d_t start_point;
    point_3d_t end_point;
} line_3d_t;

typedef struct {
    double x;
    double y;
} point_2d_t;

double r_rand = PI / 180;

double r_alpha = 19.47 * PI / 180;
double r_gamma = 20.7 * PI / 180;

double sin_alpha = sin(19.47 * PI / 180);
double cos_alpha = cos(19.47 * PI / 180);
double sin_gamma = sin(20.7 * PI / 180);
double cos_gamma = cos(20.7 * PI / 180);

TFT_eSprite Disbuff = TFT_eSprite(&M5.Lcd);

static constexpr int16_t DISPLAY_WIDTH = 135;
static constexpr int16_t DISPLAY_HEIGHT = 240;
static constexpr int16_t CUBE_CENTER_X = DISPLAY_WIDTH / 2;
static constexpr int16_t CUBE_CENTER_Y = 58;
static constexpr double CUBE_SCALE = 24.0;
static constexpr double CUBE_AXIS_LENGTH = CUBE_SCALE * 2.0;
static constexpr int16_t PLOT_CENTER_X = DISPLAY_WIDTH / 2;
static constexpr int16_t PLOT_CENTER_Y = 180;
static constexpr int16_t PLOT_RADIUS = 55;
static constexpr int16_t ELLIPSE_TOP_OFFSET = 10;
static constexpr int16_t ELLIPSE_PLOT_CENTER_Y =
    ELLIPSE_TOP_OFFSET + ((DISPLAY_HEIGHT - ELLIPSE_TOP_OFFSET) / 2);
static constexpr int16_t ELLIPSE_PLOT_RADIUS_X = DISPLAY_WIDTH / 2;
static constexpr int16_t ELLIPSE_PLOT_RADIUS_Y =
    (DISPLAY_HEIGHT - ELLIPSE_TOP_OFFSET) / 2;
static constexpr bool START_IN_ELLIPSE_MODE = true;
static constexpr int16_t SPOT_RADIUS = 10;
static constexpr int16_t LEVEL_SPOT_RADIUS = 20;
static constexpr int16_t PLOT_LINE_WIDTH = 3;
static constexpr int16_t PITCH_LABEL_X = 20;
static constexpr int16_t ROLL_LABEL_X = 115;
static constexpr int16_t ATTITUDE_LABEL_Y = 5;

static constexpr int16_t MESSAGE_RATE_X = 5;
static constexpr int16_t MESSAGE_RATE_Y = 210;
static constexpr uint16_t MESSAGE_RATE_COLOUR = TFT_WHITE;
static constexpr bool MESSAGE_RATE_AS_PERCENT = false;
static constexpr uint32_t MESSAGE_RATE_WINDOW_MS = 3000;
static constexpr uint32_t MESSAGE_RATE_DISPLAY_UPDATE_MS = 250;
static constexpr uint8_t MESSAGE_RATE_BUCKET_COUNT = 30;

static constexpr int16_t DEVICE_MODE_X = 125;
static constexpr int16_t DEVICE_MODE_Y = 210;
static constexpr int16_t DEVICE_MODE_COLOUR = TFT_WHITE;

static constexpr int16_t SENSOR_MSG_FREQ_Y = 105;
static constexpr int16_t SENSOR_MSG_FREQ_X = 5;
static constexpr int16_t SENSOR_MSG_FREQ_COLOUR = TFT_YELLOW;

static constexpr int16_t BAUD_Y = 105;
static constexpr int16_t BAUD_X = 129;
static constexpr int16_t BAUD_COLOUR = TFT_YELLOW;

static constexpr int16_t SPOT_LIMIT_LABEL_Y_OFFSET = 8;
static constexpr int16_t FORWARD_ARROW_LENGTH = 30;
static constexpr int16_t FORWARD_ARROW_HALF_WIDTH = 8;
static constexpr int16_t FORWARD_ARROW_FLUTE_DEPTH = 4;
static constexpr int16_t RED_SPOT_RETURN_ARROW_LENGTH = 28;
static constexpr int16_t RED_SPOT_RETURN_ARROW_HEAD_LENGTH = 10;
static constexpr int16_t RED_SPOT_RETURN_ARROW_HEAD_WIDTH = 14;
static constexpr int16_t RED_SPOT_RETURN_ARROW_SHAFT_WIDTH = 6;
static constexpr int16_t RED_SPOT_RETURN_ARROW_SPOT_OVERLAP = 6;
static constexpr uint16_t RED_SPOT_RETURN_ARROW_COLOR = TFT_MAGENTA;
static constexpr int16_t HOLLOW_SPOT_RING_WIDTH = 4;
static constexpr bool PLOT_BACKGROUND_BLACK = false;
static constexpr uint16_t DISPLAY_BACKGROUND_CONTROLLER_COLOR = TFT_NAVY;
static constexpr uint16_t DISPLAY_BACKGROUND_SENSOR_COLOR = TFT_DARKCYAN;
#define DISPLAY_BACKGROUND_COLOR (DEVICE_IS_SENSOR_NODE ? DISPLAY_BACKGROUND_SENSOR_COLOR : DISPLAY_BACKGROUND_CONTROLLER_COLOR)
static constexpr uint16_t TFT_DARK_GREEN_70 = 0x0260;
static constexpr uint16_t TFT_DARK_YELLOW_70 = 0x4A60;
static constexpr uint16_t TFT_DARK_RED_70 = 0x4800;
static constexpr uint16_t TFT_DARK_ORANGE_70 = 0x49C0;

static constexpr double DEGREES_PER_RADIAN = 57.29577951308232;
static constexpr uint16_t ALARM_BUZZER_FREQUENCY = 2000;
static constexpr uint32_t ALERT_TOGGLE_MS = 250;
static constexpr uint32_t LEVEL_LED_PULSE_INTERVAL_MS = 3000;
static constexpr uint32_t LEVEL_LED_PULSE_MS = 5;
static constexpr uint32_t ALARM_SPOT_FLASH_TICK_MS = 150;
static constexpr uint32_t REFERENCE_RESET_HOLD_MS = 100;
static constexpr uint16_t REFERENCE_RESET_BEEP_FREQUENCY = 500;
static constexpr uint32_t REFERENCE_RESET_BEEP_MS = 1000;
static constexpr uint32_t GEOMETRY_TOGGLE_HOLD_MS = 1000;
static constexpr uint32_t BUTTON_RELEASE_ARM_MS = 20;
static constexpr uint32_t LIMIT_CYCLE_HOLD_MS = 100;
static constexpr uint16_t LIMIT_CYCLE_BEEP_FREQUENCY = 3000;
static constexpr uint32_t LIMIT_CYCLE_BEEP_MS = 100;
static constexpr uint32_t SOUND_TOGGLE_HOLD_MS = 1000;
static constexpr uint32_t SOUND_TOGGLE_TONE_MS = 100;
static constexpr uint8_t SOUND_TOGGLE_TONE_COUNT = 3;
static constexpr uint16_t SOUND_TOGGLE_ON_TONES[SOUND_TOGGLE_TONE_COUNT] = {
    784, 1047, 1319};
static constexpr uint16_t SOUND_TOGGLE_OFF_TONES[SOUND_TOGGLE_TONE_COUNT] = {
    1319, 1047, 784};
static constexpr double INVERSION_DOT_THRESHOLD = 0.0;
static constexpr int8_t IMU_LINK_M5_YELLOW_LABEL_GROVE_GPIO = 32;
static constexpr int8_t IMU_LINK_M5_WHITE_LABEL_GROVE_GPIO = 33;
static constexpr int8_t HC12_SET_GPIO = 26;
static constexpr bool HC12_CONFIGURE_BAUD_ON_STARTUP = DEVICE_HAS_RADIO_LINK;
static constexpr bool HC12_AT_DIAGNOSTICS = true;
static constexpr bool HC12_FORCE_BAUD_SET_ON_STARTUP = false;
static constexpr uint32_t HC12_DIAGNOSTIC_SERIAL_WAIT_MS = 1500;
static constexpr uint32_t HC12_COMMAND_MODE_ENTER_MS = 50;
static constexpr uint32_t HC12_COMMAND_MODE_EXIT_MS = 90;
static constexpr uint32_t HC12_COMMAND_RESPONSE_TIMEOUT_MS = 200;
static constexpr uint32_t HC12_QUERY_RESPONSE_TIMEOUT_MS = 500;
static constexpr uint32_t HC12_SUPPORTED_BAUDS[] = {
    1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
// static constexpr int8_t IMU_LINK_RX_PIN = 33;
// TX Sensor Yellow 32 --> RXD on radio (with no heatshrink)
// RX Sensor White 33 --> TXD on radio (with no heatshrink)
static constexpr int8_t IMU_LINK_RX_PIN = (   // with radio 33 needed no matter what
            DEVICE_HAS_RADIO_LINK ? IMU_LINK_M5_WHITE_LABEL_GROVE_GPIO
                          : (DEVICE_IS_SENSOR_NODE ? IMU_LINK_M5_WHITE_LABEL_GROVE_GPIO : IMU_LINK_M5_YELLOW_LABEL_GROVE_GPIO));
// 
// RX Sensor Yellow 32 --> TXD on radio
//static constexpr int8_t IMU_LINK_TX_PIN = 32;
static constexpr int8_t IMU_LINK_TX_PIN = // with radio 32 needed no matter what;
(    DEVICE_HAS_RADIO_LINK ? IMU_LINK_M5_YELLOW_LABEL_GROVE_GPIO
                          : (DEVICE_IS_SENSOR_NODE ? IMU_LINK_M5_YELLOW_LABEL_GROVE_GPIO : IMU_LINK_M5_WHITE_LABEL_GROVE_GPIO) );

static constexpr uint8_t IMU_LINK_MAGIC_0 = 0xA5;
static constexpr uint8_t IMU_LINK_MAGIC_1 = 0x5A;
static constexpr uint8_t IMU_LINK_VERSION = 1;
static constexpr uint8_t IMU_LINK_FLAG_INVERTED = 0x01;
static constexpr uint8_t IMU_LINK_VERSION_SHIFT = 4;
static constexpr uint8_t IMU_LINK_FRAME_BYTES = 8;
static constexpr bool SHOW_IMU_LINK_DIAGNOSTICS = true;
static constexpr bool IMU_LINK_SERIAL_DIAGNOSTICS = true;
static constexpr uint32_t IMU_LINK_SERIAL_DIAGNOSTIC_MS = 1000;

struct HeadsUpLimitConfig {
    double level_limit_degrees;
    double warning_limit_degrees;
    double plot_limit_degrees;
};

HeadsUpLimitConfig HeadsUpLimitStates[] = {
    {.level_limit_degrees = 5.0,
     .warning_limit_degrees = 15.0,
     .plot_limit_degrees = 35.0},
    {.level_limit_degrees = 7.5,
     .warning_limit_degrees = 17.5,
     .plot_limit_degrees = 35.0},
    {.level_limit_degrees = 10.0,
     .warning_limit_degrees = 20.0,
     .plot_limit_degrees = 35.0},
    {.level_limit_degrees = 12.5,
     .warning_limit_degrees = 22.5,
     .plot_limit_degrees = 35.0},
};
uint8_t HeadsUpLimitStateIndex = 0;

enum class HeadsUpAlertState {
    Level,
    Warning,
    Alarm,
};

struct HeadsUpPlotGeometry {
    int16_t center_x;
    int16_t center_y;
    int16_t radius_x;
    int16_t radius_y;
    bool ellipse;
};

struct AccelVector {
    double x;
    double y;
    double z;
};

struct RemoteImuSample {
    double pitch_degrees;
    double roll_degrees;
    bool inverted;
    bool valid;
    uint32_t received_ms;
};

struct ImuLinkDiagnostics {
    uint32_t bytes_received;
    uint32_t frames_received;
    uint32_t checksum_errors;
    uint32_t version_errors;
};

struct MessageRateTracker {
    uint32_t bucket_started_ms[MESSAGE_RATE_BUCKET_COUNT];
    uint16_t bucket_count[MESSAGE_RATE_BUCKET_COUNT];
};

bool FeedbackBeepActive = false;
uint32_t FeedbackBeepStartedMs = 0;
uint32_t FeedbackBeepDurationMs = 0;
bool SoundEnabled = true;
bool SoundToggleTonesActive = false;
const uint16_t *SoundToggleTones = NULL;
uint8_t SoundToggleToneIndex = 0;
uint32_t SoundToggleTonesStartedMs = 0;

HardwareSerial ImuLinkSerial(2);
ImuLinkDiagnostics ImuLinkDiag = {0, 0, 0, 0};
MessageRateTracker ImuLinkRateTracker = {{0}, {0}};

void drawBaudRate(TFT_eSprite *display, uint32_t baud, uint32_t tx_send_interval_ms);

hw_timer_t *timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux       = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t TimerCount = 0;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    digitalWrite(10, TimerCount % 100);
    TimerCount++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void checkAXPPress() {
    if (M5.Axp.GetBtnPress()) {
        do {
            M5.update();
        } while (M5.Axp.GetBtnPress());
        M5.Beep.mute();
        ESP.restart();
    }
}

bool checkAXP192() {
    float VBat = M5.Axp.GetBatVoltage();

    while (VBat < 3.2) {
        VBat = M5.Axp.GetBatVoltage();
    }

    return true;
}

void Displaybuff() {
    Disbuff.pushSprite(0, 0);
}

void startupWaitMs(uint32_t wait_ms) {
    uint32_t started_ms = millis();
    while (millis() - started_ms < wait_ms) {
        M5.update();
        yield();
    }
}

void beginImuLinkSerial(uint32_t baud) {
    ImuLinkSerial.end();
    ImuLinkSerial.begin(baud, SERIAL_8N1, IMU_LINK_RX_PIN, IMU_LINK_TX_PIN);
    if (HC12_AT_DIAGNOSTICS) {
        Serial.printf("IMU UART: begin baud=%lu rx=%d tx=%d\r\n",
                      (unsigned long)baud, IMU_LINK_RX_PIN, IMU_LINK_TX_PIN);
    }
}

void releaseHc12SetPin() {
    pinMode(HC12_SET_GPIO, INPUT);
    if (HC12_AT_DIAGNOSTICS) {
        Serial.printf("HC12 SET GPIO%d released high/input\r\n",
                      HC12_SET_GPIO);
    }
}

void pullHc12SetPinLow() {
    digitalWrite(HC12_SET_GPIO, LOW);
    pinMode(HC12_SET_GPIO, OUTPUT);
    if (HC12_AT_DIAGNOSTICS) {
        Serial.printf("HC12 SET GPIO%d pulled low/output\r\n",
                      HC12_SET_GPIO);
    }
}

void enterHc12CommandMode() {
    if (HC12_AT_DIAGNOSTICS) {
        Serial.println("HC12: entering AT command mode");
    }
    pullHc12SetPinLow();
    startupWaitMs(HC12_COMMAND_MODE_ENTER_MS);
}

void exitHc12CommandMode() {
    if (HC12_AT_DIAGNOSTICS) {
        Serial.println("HC12: exiting AT command mode");
    }
    releaseHc12SetPin();
    startupWaitMs(HC12_COMMAND_MODE_EXIT_MS);
}

void clearImuLinkSerialInput() {
    while (ImuLinkSerial.available() > 0) {
        ImuLinkSerial.read();
    }
}

bool readHc12Response(char *response, size_t response_size,
                      const char *expected_response, uint32_t timeout_ms) {
    if (response_size == 0) {
        return false;
    }

    size_t response_length = 0;
    response[0] = '\0';
    uint32_t started_ms = millis();
    while (millis() - started_ms < timeout_ms) {
        while (ImuLinkSerial.available() > 0) {
            char value = (char)ImuLinkSerial.read();
            if (response_length + 1 < response_size) {
                response[response_length++] = value;
                response[response_length] = '\0';
            }
        }
        if (strstr(response, expected_response) != NULL) {
            if (HC12_AT_DIAGNOSTICS) {
                Serial.printf("HC12 RX: \"%s\"\r\n", response);
            }
            return true;
        }
        M5.update();
        yield();
    }

    if (HC12_AT_DIAGNOSTICS) {
        Serial.printf("HC12 RX timeout waiting for \"%s\", got \"%s\"\r\n",
                      expected_response, response);
    }
    return false;
}

bool sendHc12Command(const char *command, const char *expected_response) {
    char response[24];

    clearImuLinkSerialInput();
    if (HC12_AT_DIAGNOSTICS) {
        Serial.printf("HC12 TX: \"%s\", expecting \"%s\"\r\n", command,
                      expected_response);
    }
    ImuLinkSerial.print(command);
    ImuLinkSerial.flush();

    if (!readHc12Response(response, sizeof(response), expected_response,
                          HC12_COMMAND_RESPONSE_TIMEOUT_MS)) {
        return false;
    }
    return true;
}

bool queryHc12Settings(uint32_t expected_baud) {
    char response[96];
    char expected_response[16];

    snprintf(expected_response, sizeof(expected_response), "OK+B%lu",
             (unsigned long)expected_baud);

    clearImuLinkSerialInput();
    if (HC12_AT_DIAGNOSTICS) {
        Serial.printf("HC12 TX: \"AT+RX\", expecting \"%s\"\r\n",
                      expected_response);
    }
    ImuLinkSerial.print("AT+RX");
    ImuLinkSerial.flush();

    if (!readHc12Response(response, sizeof(response), expected_response,
                          HC12_QUERY_RESPONSE_TIMEOUT_MS)) {
        Serial.println("HC12: AT+RX query failed");
        return false;
    }

    Serial.printf("HC12: settings query returned \"%s\"\r\n", response);
    return true;
}

bool hc12CommandModeRespondsAt(uint32_t baud) {
    if (HC12_AT_DIAGNOSTICS) {
        Serial.printf("HC12: probing command baud %lu\r\n",
                      (unsigned long)baud);
    }
    beginImuLinkSerial(baud);
    startupWaitMs(5);
    return sendHc12Command("AT", "OK");
}

uint32_t findHc12CommandBaud(uint32_t preferred_baud) {
    if (hc12CommandModeRespondsAt(preferred_baud)) {
        return preferred_baud;
    }

    if (preferred_baud != 9600 && hc12CommandModeRespondsAt(9600)) {
        return 9600;
    }

    for (uint8_t index = 0;
         index < sizeof(HC12_SUPPORTED_BAUDS) / sizeof(HC12_SUPPORTED_BAUDS[0]);
         index++) {
        uint32_t baud = HC12_SUPPORTED_BAUDS[index];
        if (baud == preferred_baud || baud == 9600) {
            continue;
        }
        if (hc12CommandModeRespondsAt(baud)) {
            return baud;
        }
    }

    return 0;
}

bool setHc12BaudRate(uint32_t target_baud) {
    char command[16];
    char expected_response[16];

    if (HC12_AT_DIAGNOSTICS) {
        Serial.printf("HC12: configure target baud=%lu\r\n",
                      (unsigned long)target_baud);
    }
    enterHc12CommandMode();
    uint32_t command_baud = findHc12CommandBaud(target_baud);
    if (command_baud == 0) {
        exitHc12CommandMode();
        beginImuLinkSerial(target_baud);
        Serial.println("HC12: no AT response, using configured UART baud");
        return false;
    }

    Serial.printf("HC12: command mode at %lu baud\r\n",
                  (unsigned long)command_baud);

    if (command_baud != target_baud || HC12_FORCE_BAUD_SET_ON_STARTUP) {
        snprintf(command, sizeof(command), "AT+B%lu",
                 (unsigned long)target_baud);
        snprintf(expected_response, sizeof(expected_response), "OK+B%lu",
                 (unsigned long)target_baud);
        if (!sendHc12Command(command, expected_response)) {
            exitHc12CommandMode();
            beginImuLinkSerial(target_baud);
            Serial.printf("HC12: failed to set baud to %lu\r\n",
                          (unsigned long)target_baud);
            return false;
        }
        Serial.printf("HC12: baud set to %lu\r\n", (unsigned long)target_baud);
        if (command_baud != target_baud) {
            exitHc12CommandMode();
            beginImuLinkSerial(target_baud);
            enterHc12CommandMode();
        }
    } else if (HC12_AT_DIAGNOSTICS) {
        Serial.println("HC12: baud already matches target");
    }

    queryHc12Settings(target_baud);

    exitHc12CommandMode();
    beginImuLinkSerial(target_baud);
    if (HC12_AT_DIAGNOSTICS) {
        Serial.printf("HC12: normal data mode at %lu baud\r\n",
                      (unsigned long)target_baud);
    }
    return true;
}

void setupImuLinkSerial() {
    if (HC12_AT_DIAGNOSTICS) {
        Serial.printf("IMU link setup: radio=%s role=%s configured_baud=%lu\r\n",
                      DEVICE_HAS_RADIO_LINK ? "yes" : "no",
                      DEVICE_IS_SENSOR_NODE ? "sensor" : "controller",
                      (unsigned long)active_comms_config.imu_link_baud);
    }
    releaseHc12SetPin();
    if (HC12_CONFIGURE_BAUD_ON_STARTUP) {
        setHc12BaudRate(active_comms_config.imu_link_baud);
        return;
    }

    beginImuLinkSerial(active_comms_config.imu_link_baud);
}

bool point3Dto2D(point_3d_t *source, point_2d_t *point) {
    point->x = (source->x * cos_gamma) - (source->y * sin_gamma);
    point->y = -(source->x * sin_gamma * sin_alpha) -
               (source->y * cos_gamma * sin_alpha) + (source->z * cos_alpha);
    return true;
}

bool point2DToDisPoint(point_2d_t *point, uint8_t *x, uint8_t *y) {
    *x = point->x + 120;
    *y = 67 - point->y;
    return true;
}

bool printLine3D(TFT_eSprite *display, line_3d_t *line, uint32_t color) {
    uint8_t start_x, start_y, end_x, end_y;
    point_2d_t point;
    point3Dto2D(&line->start_point, &point);
    point2DToDisPoint(&point, &start_x, &start_y);
    point3Dto2D(&line->end_point, &point);
    point2DToDisPoint(&point, &end_x, &end_y);

    display->drawLine(start_x, start_y, end_x, end_y, color);

    return true;
}

bool point2DToDisPointAt(point_2d_t *point, int16_t *x, int16_t *y,
                         int16_t origin_x, int16_t origin_y) {
    *x = origin_x + point->x;
    *y = origin_y - point->y;
    return true;
}

bool printLine3DAt(TFT_eSprite *display, line_3d_t *line, uint32_t color,
                   int16_t origin_x, int16_t origin_y) {
    int16_t start_x, start_y, end_x, end_y;
    point_2d_t point;
    point3Dto2D(&line->start_point, &point);
    point2DToDisPointAt(&point, &start_x, &start_y, origin_x, origin_y);
    point3Dto2D(&line->end_point, &point);
    point2DToDisPointAt(&point, &end_x, &end_y, origin_x, origin_y);

    display->drawLine(start_x, start_y, end_x, end_y, color);

    return true;
}

void setRedLed(bool enabled) {
    digitalWrite(M5_LED, enabled ? LOW : HIGH);
}

uint8_t headsUpLimitStateCount() {
    return sizeof(HeadsUpLimitStates) / sizeof(HeadsUpLimitStates[0]);
}

HeadsUpLimitConfig *currentHeadsUpLimits() {
    return &HeadsUpLimitStates[HeadsUpLimitStateIndex];
}

void cycleHeadsUpLimits() {
    HeadsUpLimitStateIndex =
        (HeadsUpLimitStateIndex + 1) % headsUpLimitStateCount();
}

void startSoundToggleTones(bool sound_enabled) {
    FeedbackBeepActive = false;
    SoundToggleTonesActive = true;
    SoundToggleTones = sound_enabled ? SOUND_TOGGLE_ON_TONES
                                     : SOUND_TOGGLE_OFF_TONES;
    SoundToggleToneIndex = 0;
    SoundToggleTonesStartedMs = millis();
    M5.Beep.tone(SoundToggleTones[SoundToggleToneIndex]);
}

void toggleSoundEnabled() {
    SoundEnabled = !SoundEnabled;
    startSoundToggleTones(SoundEnabled);
}

bool updateSoundToggleTones(uint32_t now) {
    if (!SoundToggleTonesActive) {
        return false;
    }

    uint32_t elapsed_ms = now - SoundToggleTonesStartedMs;
    uint8_t tone_index = elapsed_ms / SOUND_TOGGLE_TONE_MS;
    if (tone_index >= SOUND_TOGGLE_TONE_COUNT) {
        SoundToggleTonesActive = false;
        M5.Beep.mute();
        return false;
    }

    if (tone_index != SoundToggleToneIndex) {
        SoundToggleToneIndex = tone_index;
        M5.Beep.tone(SoundToggleTones[SoundToggleToneIndex]);
    }
    return true;
}

void startFeedbackBeep(uint16_t frequency, uint32_t duration_ms) {
    if (!SoundEnabled || SoundToggleTonesActive) {
        return;
    }

    FeedbackBeepActive = true;
    FeedbackBeepStartedMs = millis();
    FeedbackBeepDurationMs = duration_ms;
    M5.Beep.tone(frequency);
}

bool updateFeedbackBeep(uint32_t now) {
    if (!FeedbackBeepActive) {
        return false;
    }

    if (!SoundEnabled) {
        FeedbackBeepActive = false;
        M5.Beep.mute();
        return false;
    }

    if (now - FeedbackBeepStartedMs < FeedbackBeepDurationMs) {
        return true;
    }

    FeedbackBeepActive = false;
    M5.Beep.mute();
    return false;
}

double clampDouble(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

uint32_t messageRateBucketMs() {
    return MESSAGE_RATE_WINDOW_MS / MESSAGE_RATE_BUCKET_COUNT;
}

void recordMessageRateEvent(uint32_t now) {
    uint32_t bucket_ms = messageRateBucketMs();
    uint32_t bucket_started_ms = now - (now % bucket_ms);
    uint8_t bucket_index =
        (bucket_started_ms / bucket_ms) % MESSAGE_RATE_BUCKET_COUNT;

    if (ImuLinkRateTracker.bucket_started_ms[bucket_index] !=
        bucket_started_ms) {
        ImuLinkRateTracker.bucket_started_ms[bucket_index] = bucket_started_ms;
        ImuLinkRateTracker.bucket_count[bucket_index] = 0;
    }
    ImuLinkRateTracker.bucket_count[bucket_index]++;
}

double averageMessageRatePerSecond() {
    uint32_t now = millis();
    uint32_t window_started_ms =
        now > MESSAGE_RATE_WINDOW_MS ? now - MESSAGE_RATE_WINDOW_MS : 0;
    uint32_t count = 0;
    for (uint8_t index = 0; index < MESSAGE_RATE_BUCKET_COUNT; index++) {
        uint32_t bucket_started_ms =
            ImuLinkRateTracker.bucket_started_ms[index];
        if (bucket_started_ms >= window_started_ms &&
            bucket_started_ms <= now) {
            count += ImuLinkRateTracker.bucket_count[index];
        }
    }

    uint32_t elapsed_ms = now - window_started_ms;
    if (elapsed_ms == 0) {
        return 0.0;
    }

    return (count * 1000.0) / elapsed_ms;
}

double expectedMessageRatePerSecond() {
    uint32_t interval_ms = active_comms_config.imu_link_send_interval_ms;
    if (interval_ms == 0) {
        return 0.0;
    }

    return 1000.0 / interval_ms;
}

void formatMessageRateText(char *rate_text, size_t rate_text_size) {
    double message_rate = averageMessageRatePerSecond();
    if (MESSAGE_RATE_AS_PERCENT) {
        double expected_rate = expectedMessageRatePerSecond();
        int percent =
            expected_rate > 0.0
                ? (int)((message_rate * 100.0 / expected_rate) + 0.5)
                : 100;
        if (percent < 0) {
            percent = 0;
        } else if (percent > 100) {
            percent = 100;
        }
        snprintf(rate_text, rate_text_size, "%d%%", percent);
    } else {
        snprintf(rate_text, rate_text_size, "%.0f/s", message_rate);
    }
}

int16_t degreesToCentidegrees(double degrees) {
    double centidegrees = degrees * 100.0;
    if (centidegrees <= -32768.0) {
        return -32768;
    }
    if (centidegrees >= 32767.0) {
        return 32767;
    }
    return (int16_t)(centidegrees >= 0 ? centidegrees + 0.5
                                       : centidegrees - 0.5);
}

void writeInt16LE(uint8_t *buffer, uint8_t offset, int16_t value) {
    buffer[offset] = value & 0xFF;
    buffer[offset + 1] = (value >> 8) & 0xFF;
}

int16_t readInt16LE(uint8_t *buffer, uint8_t offset) {
    return (int16_t)(buffer[offset] | (buffer[offset + 1] << 8));
}

uint8_t imuLinkChecksum(uint8_t *frame) {
    uint8_t checksum = 0;
    for (uint8_t index = 0; index < IMU_LINK_FRAME_BYTES - 1; index++) {
        checksum += frame[index];
    }
    return checksum;
}

void sendImuLinkSample(double pitch_degrees, double roll_degrees,
                       bool inverted) {
    static bool schedule_started = false;
    static uint32_t next_send_due_ms = 0;
    uint32_t now = millis();
    uint32_t interval_ms = active_comms_config.imu_link_send_interval_ms;

    if (interval_ms > 0) {
        if (!schedule_started) {
            schedule_started = true;
            next_send_due_ms = now;
        }
        if ((int32_t)(now - next_send_due_ms) < 0) {
            return;
        }
        do {
            next_send_due_ms += interval_ms;
        } while ((int32_t)(now - next_send_due_ms) >= 0);
    } else {
        schedule_started = false;
        next_send_due_ms = now;
    }

    uint8_t frame[IMU_LINK_FRAME_BYTES] = {0};

    frame[0] = IMU_LINK_MAGIC_0;
    frame[1] = IMU_LINK_MAGIC_1;
    frame[2] = (IMU_LINK_VERSION << IMU_LINK_VERSION_SHIFT) |
               (inverted ? IMU_LINK_FLAG_INVERTED : 0);
    writeInt16LE(frame, 3, degreesToCentidegrees(pitch_degrees));
    writeInt16LE(frame, 5, degreesToCentidegrees(roll_degrees));
    frame[7] = imuLinkChecksum(frame);

    if (ImuLinkSerial.availableForWrite() < IMU_LINK_FRAME_BYTES) {
        return;
    }

    ImuLinkSerial.write(frame, IMU_LINK_FRAME_BYTES);
    recordMessageRateEvent(now);
}

bool decodeImuLinkFrame(uint8_t *frame, RemoteImuSample *sample) {
    if (frame[0] != IMU_LINK_MAGIC_0 || frame[1] != IMU_LINK_MAGIC_1) {
        return false;
    }

    if ((frame[2] >> IMU_LINK_VERSION_SHIFT) != IMU_LINK_VERSION) {
        if (frame[0] == IMU_LINK_MAGIC_0 && frame[1] == IMU_LINK_MAGIC_1) {
            ImuLinkDiag.version_errors++;
        }
        return false;
    }

    if (frame[7] != imuLinkChecksum(frame)) {
        ImuLinkDiag.checksum_errors++;
        return false;
    }

    sample->pitch_degrees = readInt16LE(frame, 3) / 100.0;
    sample->roll_degrees = readInt16LE(frame, 5) / 100.0;
    sample->inverted = (frame[2] & IMU_LINK_FLAG_INVERTED) != 0;
    sample->received_ms = millis();
    sample->valid = true;
    ImuLinkDiag.frames_received++;
    recordMessageRateEvent(sample->received_ms);
    return true;
}

bool readRemoteImuSample(RemoteImuSample *sample) {
    static uint8_t frame[IMU_LINK_FRAME_BYTES] = {0};
    static uint8_t frame_index = 0;
    bool received_sample = false;

    while (ImuLinkSerial.available() > 0) {
        uint8_t value = ImuLinkSerial.read();
        ImuLinkDiag.bytes_received++;
        if (frame_index == 0 && value != IMU_LINK_MAGIC_0) {
            continue;
        }
        if (frame_index == 1 && value != IMU_LINK_MAGIC_1) {
            frame[0] = value;
            frame_index = value == IMU_LINK_MAGIC_0 ? 1 : 0;
            continue;
        }

        frame[frame_index++] = value;
        if (frame_index >= IMU_LINK_FRAME_BYTES) {
            if (decodeImuLinkFrame(frame, sample)) {
                received_sample = true;
            }
            frame_index = 0;
        }
    }

    return received_sample;
}

bool remoteImuSampleIsFresh(RemoteImuSample *sample) {
    return sample->valid &&
           millis() - sample->received_ms <=
               active_comms_config.imu_link_timeout_ms;
}

void printImuLinkRuntimeDiagnostics(RemoteImuSample *sample,
                                    bool have_attitude_sample) {
    static uint32_t last_printed_ms = 0;
    if (!IMU_LINK_SERIAL_DIAGNOSTICS || DEVICE_IS_SENSOR_NODE) {
        return;
    }

    uint32_t now = millis();
    if (last_printed_ms != 0 &&
        now - last_printed_ms < IMU_LINK_SERIAL_DIAGNOSTIC_MS) {
        return;
    }
    last_printed_ms = now;

    Serial.printf(
        "Controller RX: bytes=%lu frames=%lu rate=%.1f/s checksum=%lu version=%lu fresh=%s pitch=%.1f roll=%.1f inverted=%d\r\n",
        (unsigned long)ImuLinkDiag.bytes_received,
        (unsigned long)ImuLinkDiag.frames_received,
        averageMessageRatePerSecond(),
        (unsigned long)ImuLinkDiag.checksum_errors,
        (unsigned long)ImuLinkDiag.version_errors,
        have_attitude_sample ? "yes" : "no",
        sample->pitch_degrees,
        sample->roll_degrees,
        sample->inverted ? 1 : 0);
}

void readTiltSample(double *theta, double *phi, AccelVector *accel) {
    float accX = 0;
    float accY = 0;
    float accZ = 0;

    M5.Imu.getAccelData(&accX, &accY, &accZ);
    if (accel != NULL) {
        accel->x = accX;
        accel->y = accY;
        accel->z = accZ;
    }

    *theta = asin(clampDouble(-accX, -1.0, 1.0)) * DEGREES_PER_RADIAN;
    if (accZ != 0) {
        *phi = atan(accY / accZ) * DEGREES_PER_RADIAN;
    } else {
        *phi = 0;
    }
}

void readTiltDegrees(double *theta, double *phi) {
    readTiltSample(theta, phi, NULL);
}

double accelDotProduct(AccelVector *a, AccelVector *b) {
    double a_length = sqrt((a->x * a->x) + (a->y * a->y) + (a->z * a->z));
    double b_length = sqrt((b->x * b->x) + (b->y * b->y) + (b->z * b->z));
    if (a_length <= 0.0 || b_length <= 0.0) {
        return 1.0;
    }

    return ((a->x * b->x) + (a->y * b->y) + (a->z * b->z)) /
           (a_length * b_length);
}

bool isInvertedFromReference(AccelVector *current, AccelVector *reference) {
    return accelDotProduct(current, reference) < INVERSION_DOT_THRESHOLD;
}

HeadsUpAlertState getHeadsUpAlertState(double pitch_degrees,
                                       double roll_degrees,
                                       HeadsUpLimitConfig *limits,
                                       bool inverted) {
    if (inverted) {
        return HeadsUpAlertState::Alarm;
    }

    double pitch_deflection = fabs(pitch_degrees);
    double roll_deflection = fabs(roll_degrees);
    double axis_deflection =
        pitch_deflection > roll_deflection ? pitch_deflection : roll_deflection;

    if (axis_deflection <= limits->level_limit_degrees) {
        return HeadsUpAlertState::Level;
    }
    if (axis_deflection > limits->warning_limit_degrees) {
        return HeadsUpAlertState::Alarm;
    }
    return HeadsUpAlertState::Warning;
}

void updateHeadsUpAlerts(HeadsUpAlertState alert_state) {
    static uint32_t last_level_pulse_ms = 0;
    static uint32_t level_pulse_started_ms = 0;
    static bool level_pulse_active = false;
    uint32_t now = millis();
    bool alert_phase_on = ((now / ALERT_TOGGLE_MS) % 2) == 0;
    bool sound_toggle_tones_active = updateSoundToggleTones(now);
    bool feedback_beep_active =
        sound_toggle_tones_active ? false : updateFeedbackBeep(now);

    if (alert_state == HeadsUpAlertState::Alarm) {
        level_pulse_active = false;
        setRedLed(true);
        if (sound_toggle_tones_active || feedback_beep_active) {
            return;
        }
        if (!SoundEnabled) {
            M5.Beep.mute();
            return;
        }
        if (alert_phase_on) {
            M5.Beep.tone(ALARM_BUZZER_FREQUENCY);
        } else {
            M5.Beep.mute();
        }
    } else if (alert_state == HeadsUpAlertState::Warning) {
        level_pulse_active = false;
        setRedLed(alert_phase_on);
        if (!sound_toggle_tones_active && !feedback_beep_active) {
            M5.Beep.mute();
        }
    } else {
        if (!sound_toggle_tones_active && !feedback_beep_active) {
            M5.Beep.mute();
        }
        if (level_pulse_active &&
            now - level_pulse_started_ms >= LEVEL_LED_PULSE_MS) {
            level_pulse_active = false;
            last_level_pulse_ms = now;
        } else if (!level_pulse_active &&
                   now - last_level_pulse_ms >= LEVEL_LED_PULSE_INTERVAL_MS) {
            level_pulse_active = true;
            level_pulse_started_ms = now;
        }
        setRedLed(level_pulse_active);
    }
}

void drawThickCircle(TFT_eSprite *display, int16_t x, int16_t y,
                     int16_t radius, uint32_t color) {
    int16_t half_width = PLOT_LINE_WIDTH / 2;
    for (int16_t offset = -half_width; offset <= half_width; offset++) {
        display->drawCircle(x, y, radius + offset, color);
    }
}

void drawThickEllipse(TFT_eSprite *display, int16_t x, int16_t y,
                      int16_t radius_x, int16_t radius_y, uint32_t color) {
    int16_t half_width = PLOT_LINE_WIDTH / 2;
    for (int16_t offset = -half_width; offset <= half_width; offset++) {
        display->drawEllipse(x, y, radius_x + offset, radius_y + offset,
                             color);
    }
}

void drawThickHorizontalLine(TFT_eSprite *display, int16_t x0, int16_t x1,
                             int16_t y, uint32_t color) {
    int16_t half_width = PLOT_LINE_WIDTH / 2;
    for (int16_t offset = -half_width; offset <= half_width; offset++) {
        display->drawLine(x0, y + offset, x1, y + offset, color);
    }
}

void drawThickVerticalLine(TFT_eSprite *display, int16_t x, int16_t y0,
                           int16_t y1, uint32_t color) {
    int16_t half_width = PLOT_LINE_WIDTH / 2;
    for (int16_t offset = -half_width; offset <= half_width; offset++) {
        display->drawLine(x + offset, y0, x + offset, y1, color);
    }
}

HeadsUpPlotGeometry headsUpPlotGeometry(bool ellipse_mode) {
    if (ellipse_mode) {
        return {.center_x = PLOT_CENTER_X,
                .center_y = ELLIPSE_PLOT_CENTER_Y,
                .radius_x = ELLIPSE_PLOT_RADIUS_X,
                .radius_y = ELLIPSE_PLOT_RADIUS_Y,
                .ellipse = true};
    }

    return {.center_x = PLOT_CENTER_X,
            .center_y = PLOT_CENTER_Y,
            .radius_x = PLOT_RADIUS,
            .radius_y = PLOT_RADIUS,
            .ellipse = false};
}

void drawHeadsUpAttitudeLabel(TFT_eSprite *display, double pitch_degrees,
                              double roll_degrees) {
    char pitch_text[8];
    char roll_text[8];
    snprintf(pitch_text, sizeof(pitch_text), "%.0f", pitch_degrees);
    snprintf(roll_text, sizeof(roll_text), "%.0f", roll_degrees);
    display->setTextSize(2);
    display->setTextColor(TFT_GREENYELLOW);
    display->drawCentreString(pitch_text, PITCH_LABEL_X, ATTITUDE_LABEL_Y, 1);
    display->setTextColor(TFT_MAGENTA);
    display->drawCentreString(roll_text, ROLL_LABEL_X, ATTITUDE_LABEL_Y, 1);
    display->setTextSize(1);
}

void drawDeviceRoleLabel(TFT_eSprite *display, bool sensor_node_mode)
{
    display->setTextSize(2);
    display->setTextColor(DEVICE_MODE_COLOUR);
    const char *sensor_label = "S";
    const char *controller_label = "C";

    display->drawCentreString(
        sensor_node_mode ? sensor_label : controller_label, DEVICE_MODE_X,
        DEVICE_MODE_Y, 2);
}

void drawMessageRate(TFT_eSprite *display) {
    static char message_rate_text[10] = "0/s";
    static uint32_t last_updated_ms = 0;
    uint32_t now = millis();
    if (last_updated_ms == 0 ||
        now - last_updated_ms >= MESSAGE_RATE_DISPLAY_UPDATE_MS) {
        formatMessageRateText(message_rate_text, sizeof(message_rate_text));
        last_updated_ms = now;
    }

    display->setTextSize(2);
    display->setTextColor(MESSAGE_RATE_COLOUR);
    display->drawString(message_rate_text, MESSAGE_RATE_X, MESSAGE_RATE_Y, 2);
    display->setTextSize(1);
}

void drawImuLinkWaitingLabel(TFT_eSprite *display) {
    char diagnostic_text[40];

    uint16_t shift_uart_msg_upwards = 50;

    display->setTextSize(2);
    display->setTextColor(TFT_RED);
    display->drawCentreString("NO UART", PLOT_CENTER_X,
                              DISPLAY_HEIGHT / 2 - 8 - shift_uart_msg_upwards, 1);
    if (SHOW_IMU_LINK_DIAGNOSTICS) {
        snprintf(diagnostic_text, sizeof(diagnostic_text), "B%lu F%lu C%lu V%lu",
                 (unsigned long)ImuLinkDiag.bytes_received,
                 (unsigned long)ImuLinkDiag.frames_received,
                 (unsigned long)ImuLinkDiag.checksum_errors,
                 (unsigned long)ImuLinkDiag.version_errors);
        display->setTextSize(1);
        display->setTextColor(TFT_WHITE);
        display->drawCentreString(diagnostic_text, PLOT_CENTER_X,
                                  DISPLAY_HEIGHT / 2 + 14 - shift_uart_msg_upwards, 1);
    }

    drawBaudRate(display, active_comms_config.imu_link_baud, active_comms_config.imu_link_send_interval_ms);

    display->setTextSize(1);
}

void drawHeadsUpSpotLimitLabel(TFT_eSprite *display, int16_t spot_x,
                               int16_t spot_y, HeadsUpLimitConfig *limits) {
    char limit_text[8];
    snprintf(limit_text, sizeof(limit_text), "%d",
             (int)floor(limits->level_limit_degrees));
    display->setTextSize(2);
    display->setTextColor(TFT_BLUE);
    display->drawCentreString(limit_text, spot_x,
                              spot_y - SPOT_LIMIT_LABEL_Y_OFFSET, 1);
    display->setTextSize(1);
}

void drawHeadsUpWarningSpotLimitLabel(TFT_eSprite *display, int16_t spot_x,
                                      int16_t spot_y,
                                      HeadsUpLimitConfig *limits) {
    char limit_text[8];
    snprintf(limit_text, sizeof(limit_text), "%.0f",
             limits->warning_limit_degrees);
    display->setTextSize(2);
    display->setTextColor(TFT_WHITE);
    display->drawCentreString(limit_text, spot_x,
                              spot_y - SPOT_LIMIT_LABEL_Y_OFFSET, 1);
    display->setTextSize(1);
}

uint32_t plotBackgroundColorForSpot(uint32_t spot_color) {
    if (PLOT_BACKGROUND_BLACK) {
        return TFT_BLACK;
    }
    if (spot_color == TFT_GREEN) {
        return TFT_DARK_GREEN_70;
    }
    if (spot_color == TFT_YELLOW) {
        return TFT_DARK_YELLOW_70;
    }
    if (spot_color == TFT_RED) {
        return TFT_DARK_RED_70;
    }
    if (spot_color == TFT_ORANGE) {
        return TFT_DARK_ORANGE_70;
    }
    return DISPLAY_BACKGROUND_COLOR;
}

void drawHeadsUpPlotBackground(TFT_eSprite *display,
                               HeadsUpPlotGeometry *geometry,
                               uint32_t spot_color) {
    uint32_t background_color = plotBackgroundColorForSpot(spot_color);
    if (geometry->ellipse) {
        display->fillEllipse(geometry->center_x, geometry->center_y,
                             geometry->radius_x, geometry->radius_y,
                             background_color);
    } else {
        display->fillCircle(geometry->center_x, geometry->center_y,
                            geometry->radius_x, background_color);
    }
}

void drawHeadsUpPlotBoundary(TFT_eSprite *display,
                             HeadsUpPlotGeometry *geometry, uint32_t color) {
    if (geometry->ellipse) {
        drawThickEllipse(display, geometry->center_x, geometry->center_y,
                         geometry->radius_x, geometry->radius_y, color);
    } else {
        drawThickCircle(display, geometry->center_x, geometry->center_y,
                        geometry->radius_x, color);
    }
}

void drawHeadsUpForwardArrow(TFT_eSprite *display,
                             HeadsUpPlotGeometry *geometry,
                             uint32_t color, uint32_t background_color) {
    int16_t arrow_tip_y = geometry->center_y - geometry->radius_y;
    int16_t arrow_base_y = arrow_tip_y + FORWARD_ARROW_LENGTH;
    display->fillTriangle(geometry->center_x, arrow_tip_y,
                          geometry->center_x - FORWARD_ARROW_HALF_WIDTH,
                          arrow_base_y,
                          geometry->center_x + FORWARD_ARROW_HALF_WIDTH,
                          arrow_base_y, color);
    display->fillTriangle(geometry->center_x,
                          arrow_base_y - FORWARD_ARROW_FLUTE_DEPTH,
                          geometry->center_x - FORWARD_ARROW_HALF_WIDTH,
                          arrow_base_y,
                          geometry->center_x + FORWARD_ARROW_HALF_WIDTH,
                          arrow_base_y, background_color);
}

void drawBaudRate(TFT_eSprite *display, uint32_t baud, uint32_t tx_send_interval_ms)
{
    String tx_msg_freq;

    if (tx_send_interval_ms == 0)
        tx_msg_freq = "+Hz";
    else
    {
        uint32_t frequency = float(1)/float(tx_send_interval_ms) * 1000.0;
        tx_msg_freq = String(frequency)+"Hz";
    }
    
    display->setTextColor(SENSOR_MSG_FREQ_COLOUR);
    display->setTextSize(2);
    display->drawString(tx_msg_freq,SENSOR_MSG_FREQ_X,SENSOR_MSG_FREQ_Y,1);

    display->setTextColor(BAUD_COLOUR);
    display->setTextSize(2);
    display->drawRightString(String(baud),BAUD_X,BAUD_Y,1);
}

int16_t roundDoubleToInt16(double value) {
    return (int16_t)(value >= 0 ? value + 0.5 : value - 0.5);
}

void fillTriangleRounded(TFT_eSprite *display, double x0, double y0, double x1,
                         double y1, double x2, double y2, uint32_t color) {
    display->fillTriangle(roundDoubleToInt16(x0), roundDoubleToInt16(y0),
                          roundDoubleToInt16(x1), roundDoubleToInt16(y1),
                          roundDoubleToInt16(x2), roundDoubleToInt16(y2),
                          color);
}

void drawRedSpotReturnArrow(TFT_eSprite *display,
                            HeadsUpPlotGeometry *geometry, int16_t spot_x,
                            int16_t spot_y, int16_t spot_radius,
                            bool point_away_from_origin) {
    double direction_x = point_away_from_origin
                             ? spot_x - geometry->center_x
                             : geometry->center_x - spot_x;
    double direction_y = point_away_from_origin
                             ? spot_y - geometry->center_y
                             : geometry->center_y - spot_y;
    double direction_length =
        sqrt((direction_x * direction_x) + (direction_y * direction_y));
    if (direction_length <= 0.0 && point_away_from_origin) {
        direction_x = 0.0;
        direction_y = -1.0;
        direction_length = 1.0;
    } else if (direction_length <= 0.0) {
        return;
    }

    direction_x /= direction_length;
    direction_y /= direction_length;

    double arrow_length = RED_SPOT_RETURN_ARROW_LENGTH;
    if (!point_away_from_origin) {
        double max_arrow_length = direction_length - spot_radius;
        if (arrow_length > max_arrow_length) {
            arrow_length = max_arrow_length;
        }
    }
    if (arrow_length <= 0.0) {
        return;
    }

    double perpendicular_x = -direction_y;
    double perpendicular_y = direction_x;
    double tail_x =
        spot_x + direction_x * (spot_radius - RED_SPOT_RETURN_ARROW_SPOT_OVERLAP);
    double tail_y =
        spot_y + direction_y * (spot_radius - RED_SPOT_RETURN_ARROW_SPOT_OVERLAP);
    double tip_x = spot_x + direction_x * (spot_radius + arrow_length);
    double tip_y = spot_y + direction_y * (spot_radius + arrow_length);
    double head_length = RED_SPOT_RETURN_ARROW_HEAD_LENGTH;
    if (head_length > arrow_length) {
        head_length = arrow_length;
    }
    double head_base_x = tip_x - direction_x * head_length;
    double head_base_y = tip_y - direction_y * head_length;
    double shaft_half_width = RED_SPOT_RETURN_ARROW_SHAFT_WIDTH / 2.0;
    double head_half_width = RED_SPOT_RETURN_ARROW_HEAD_WIDTH / 2.0;

    double tail_left_x = tail_x + perpendicular_x * shaft_half_width;
    double tail_left_y = tail_y + perpendicular_y * shaft_half_width;
    double tail_right_x = tail_x - perpendicular_x * shaft_half_width;
    double tail_right_y = tail_y - perpendicular_y * shaft_half_width;
    double shaft_left_x = head_base_x + perpendicular_x * shaft_half_width;
    double shaft_left_y = head_base_y + perpendicular_y * shaft_half_width;
    double shaft_right_x = head_base_x - perpendicular_x * shaft_half_width;
    double shaft_right_y = head_base_y - perpendicular_y * shaft_half_width;
    double head_left_x = head_base_x + perpendicular_x * head_half_width;
    double head_left_y = head_base_y + perpendicular_y * head_half_width;
    double head_right_x = head_base_x - perpendicular_x * head_half_width;
    double head_right_y = head_base_y - perpendicular_y * head_half_width;

    fillTriangleRounded(display, tail_left_x, tail_left_y, shaft_left_x,
                        shaft_left_y, shaft_right_x, shaft_right_y,
                        RED_SPOT_RETURN_ARROW_COLOR);
    fillTriangleRounded(display, tail_left_x, tail_left_y, shaft_right_x,
                        shaft_right_y, tail_right_x, tail_right_y,
                        RED_SPOT_RETURN_ARROW_COLOR);
    fillTriangleRounded(display, tip_x, tip_y, head_left_x, head_left_y,
                        head_right_x, head_right_y,
                        RED_SPOT_RETURN_ARROW_COLOR);
}

HeadsUpAlertState drawHeadsUpPlot(TFT_eSprite *display, double pitch_degrees,
                                  double roll_degrees, bool ellipse_mode,
                                  bool inverted) {
    HeadsUpLimitConfig *limits = currentHeadsUpLimits();
    HeadsUpPlotGeometry geometry = headsUpPlotGeometry(ellipse_mode);

    double plot_x =
        clampDouble(roll_degrees / limits->plot_limit_degrees, -1.0, 1.0);
    double plot_y =
        clampDouble(pitch_degrees / limits->plot_limit_degrees, -1.0, 1.0);
    double plot_magnitude = sqrt((plot_x * plot_x) + (plot_y * plot_y));
    if (plot_magnitude > 1.0) {
        plot_x /= plot_magnitude;
        plot_y /= plot_magnitude;
    }

    int16_t spot_radius = SPOT_RADIUS;
    uint32_t spot_color = TFT_YELLOW;
    HeadsUpAlertState alert_state =
        getHeadsUpAlertState(pitch_degrees, roll_degrees, limits, inverted);
    if (alert_state == HeadsUpAlertState::Level) {
        spot_radius = LEVEL_SPOT_RADIUS;
        spot_color = TFT_GREEN;
    } else if (alert_state == HeadsUpAlertState::Alarm) {
        spot_radius = LEVEL_SPOT_RADIUS;
        spot_color = TFT_RED;
    }

    int16_t spot_x = geometry.center_x + (int16_t)(plot_x * geometry.radius_x);
    int16_t spot_y = geometry.center_y + (int16_t)(plot_y * geometry.radius_y);

    if (alert_state == HeadsUpAlertState::Alarm) {
        uint32_t flash_phase = (millis() / ALARM_SPOT_FLASH_TICK_MS) % 3;
        spot_color = flash_phase == 0 ? TFT_ORANGE : TFT_RED;
    }

    uint32_t plot_background_color = plotBackgroundColorForSpot(spot_color);
    drawHeadsUpPlotBackground(display, &geometry, spot_color);
    drawHeadsUpPlotBoundary(display, &geometry, spot_color);
    drawThickHorizontalLine(display, geometry.center_x - geometry.radius_x,
                            geometry.center_x + geometry.radius_x,
                            geometry.center_y, spot_color);
    drawThickVerticalLine(display, geometry.center_x,
                          geometry.center_y - geometry.radius_y +
                              FORWARD_ARROW_LENGTH,
                          geometry.center_y + geometry.radius_y, spot_color);
    drawHeadsUpForwardArrow(display, &geometry, spot_color,
                            plot_background_color);
    drawBaudRate(display, active_comms_config.imu_link_baud, active_comms_config.imu_link_send_interval_ms);

    if (alert_state == HeadsUpAlertState::Alarm && spot_color == TFT_RED) {
        drawRedSpotReturnArrow(display, &geometry, spot_x, spot_y,
                               spot_radius, inverted);
    }
    bool hollow_spot = inverted && alert_state == HeadsUpAlertState::Alarm;
    display->fillCircle(spot_x, spot_y, spot_radius, spot_color);
    if (hollow_spot) {
        display->fillCircle(spot_x, spot_y,
                            spot_radius - HOLLOW_SPOT_RING_WIDTH, TFT_BLACK);
    }
    if (alert_state == HeadsUpAlertState::Level) {
        drawHeadsUpSpotLimitLabel(display, spot_x, spot_y, limits);
    } else if (alert_state == HeadsUpAlertState::Alarm &&
               spot_color == TFT_RED && !hollow_spot) {
        drawHeadsUpWarningSpotLimitLabel(display, spot_x, spot_y, limits);
    }
    drawHeadsUpAttitudeLabel(display, pitch_degrees, roll_degrees);
    return alert_state;
}

void RotatePoint(point_3d_t *point, double x, double y, double z) {
    if (x != 0) {
        point->y = point->y * cos(x * r_rand) - point->z * sin(x * r_rand);
        point->z = point->y * sin(x * r_rand) + point->z * cos(x * r_rand);
    }

    if (y != 0) {
        point->x = point->z * sin(y * r_rand) + point->x * cos(y * r_rand);
        point->z = point->z * cos(y * r_rand) - point->x * sin(y * r_rand);
    }

    if (z != 0) {
        point->x = point->x * cos(z * r_rand) - point->y * sin(z * r_rand);
        point->y = point->x * sin(z * r_rand) + point->y * cos(z * r_rand);
    }
}

void RotatePoint(point_3d_t *point, point_3d_t *point_new, double x, double y,
                 double z) {
    if (x != 0) {
        point_new->y = point->y * cos(x * r_rand) - point->z * sin(x * r_rand);
        point_new->z = point->y * sin(x * r_rand) + point->z * cos(x * r_rand);
    }

    if (y != 0) {
        point_new->x = point->z * sin(y * r_rand) + point->x * cos(y * r_rand);
        point_new->z = point->z * cos(y * r_rand) - point->x * sin(y * r_rand);
    }

    if (z != 0) {
        point_new->x = point->x * cos(z * r_rand) - point->y * sin(z * r_rand);
        point_new->y = point->x * sin(z * r_rand) + point->y * cos(z * r_rand);
    }
}

line_3d_t rect[12] = {
    {.start_point = {-1, -1, 1}, .end_point = {1, -1, 1}},
    {.start_point = {1, -1, 1}, .end_point = {1, 1, 1}},
    {.start_point = {1, 1, 1}, .end_point = {-1, 1, 1}},
    {.start_point = {-1, 1, 1}, .end_point = {-1, -1, 1}},
    {
        .start_point = {-1, -1, 1},
        .end_point   = {-1, -1, -1},
    },
    {
        .start_point = {1, -1, 1},
        .end_point   = {1, -1, -1},
    },
    {
        .start_point = {1, 1, 1},
        .end_point   = {1, 1, -1},
    },
    {
        .start_point = {-1, 1, 1},
        .end_point   = {-1, 1, -1},
    },
    {.start_point = {-1, -1, -1}, .end_point = {1, -1, -1}},
    {.start_point = {1, -1, -1}, .end_point = {1, 1, -1}},
    {.start_point = {1, 1, -1}, .end_point = {-1, 1, -1}},
    {.start_point = {-1, 1, -1}, .end_point = {-1, -1, -1}},
};

void prepareHeadsUpCube(line_3d_t rect_source[12]) {
    for (int n = 0; n < 12; n++) {
        rect_source[n].start_point.x = rect[n].start_point.x * CUBE_SCALE;
        rect_source[n].start_point.y = rect[n].start_point.y * CUBE_SCALE;
        rect_source[n].start_point.z = rect[n].start_point.z * CUBE_SCALE;
        rect_source[n].end_point.x   = rect[n].end_point.x * CUBE_SCALE;
        rect_source[n].end_point.y   = rect[n].end_point.y * CUBE_SCALE;
        rect_source[n].end_point.z   = rect[n].end_point.z * CUBE_SCALE;
    }
}

void drawHeadsUpCube(TFT_eSprite *display, double theta, double phi,
                     line_3d_t rect_source[12]) {
    line_3d_t x = {.start_point = {0, 0, 0}, .end_point = {0, 0, 0}};
    line_3d_t y = {.start_point = {0, 0, 0}, .end_point = {0, 0, 0}};
    line_3d_t z = {.start_point = {0, 0, 0}, .end_point = {0, 0, 0}};
    line_3d_t rect_dis;

    z.end_point.z = CUBE_AXIS_LENGTH;
    RotatePoint(&z.end_point, theta, phi, 0);
    RotatePoint(&z.end_point, &x.end_point, -90, 0, 0);
    RotatePoint(&z.end_point, &y.end_point, 0, 90, 0);

    for (int n = 0; n < 12; n++) {
        RotatePoint(&rect_source[n].start_point, &rect_dis.start_point, theta,
                    phi, (double)0);
        RotatePoint(&rect_source[n].end_point, &rect_dis.end_point, theta, phi,
                    (double)0);
        printLine3DAt(display, &rect_dis, TFT_WHITE, CUBE_CENTER_X,
                      CUBE_CENTER_Y);
    }

    printLine3DAt(display, &x, TFT_RED, CUBE_CENTER_X, CUBE_CENTER_Y);
    printLine3DAt(display, &y, TFT_GREEN, CUBE_CENTER_X, CUBE_CENTER_Y);
    printLine3DAt(display, &z, TFT_BLUE, CUBE_CENTER_X, CUBE_CENTER_Y);
}

void MPU6886Test() {
    float accX = 0;
    float accY = 0;
    float accZ = 0;

    double theta = 0, last_theta = 0;
    double phi = 0, last_phi = 0;
    double alpha = 0.2;

    line_3d_t x = {.start_point = {0, 0, 0}, .end_point = {0, 0, 0}};
    line_3d_t y = {.start_point = {0, 0, 0}, .end_point = {0, 0, 0}};
    line_3d_t z = {.start_point = {0, 0, 0}, .end_point = {0, 0, 30}};

    line_3d_t rect_source[12];
    line_3d_t rect_dis;
    for (int n = 0; n < 12; n++) {
        rect_source[n].start_point.x = rect[n].start_point.x * 30;
        rect_source[n].start_point.y = rect[n].start_point.y * 30;
        rect_source[n].start_point.z = rect[n].start_point.z * 30;
        rect_source[n].end_point.x   = rect[n].end_point.x * 30;
        rect_source[n].end_point.y   = rect[n].end_point.y * 30;
        rect_source[n].end_point.z   = rect[n].end_point.z * 30;
    }

    while ((!M5.BtnA.isPressed()) && (!M5.BtnB.isPressed())) {
        M5.Imu.getAccelData(&accX, &accY, &accZ);
        // M5.MPU6886.getAccelData(&accX, &accY, &accZ);
        if ((accX < 1) && (accX > -1)) {
            theta = asin(-accX) * 57.295;
        }
        if (accZ != 0) {
            phi = atan(accY / accZ) * 57.295;
        }

        theta = alpha * theta + (1 - alpha) * last_theta;
        phi   = alpha * phi + (1 - alpha) * last_phi;

        Disbuff.fillRect(0, 0, 240, 135, TFT_BLACK);

        z.end_point.x = 0;
        z.end_point.y = 0;
        z.end_point.z = 60;
        RotatePoint(&z.end_point, theta, phi, 0);
        RotatePoint(&z.end_point, &x.end_point, -90, 0, 0);
        RotatePoint(&z.end_point, &y.end_point, 0, 90, 0);

        for (int n = 0; n < 12; n++) {
            RotatePoint(&rect_source[n].start_point, &rect_dis.start_point,
                        theta, phi, (double)0);
            RotatePoint(&rect_source[n].end_point, &rect_dis.end_point, theta,
                        phi, (double)0);
            printLine3D(&Disbuff, &rect_dis, TFT_WHITE);
        }
        // Disbuff.fillRect(0,0,160,80,BLACK);
        printLine3D(&Disbuff, &x, TFT_RED);
        printLine3D(&Disbuff, &y, TFT_GREEN);
        printLine3D(&Disbuff, &z, TFT_BLUE);
        /*
        Disbuff.setTextColor(TFT_WHITE);
        Disbuff.setTextSize(1);
        Disbuff.fillRect(0,0,52,18,Disbuff.color565(20,20,20));
        Disbuff.drawString("MPU6886",5,5,1);
        */
        Displaybuff();
        last_theta = theta;
        last_phi   = phi;

        M5.update();
        checkAXPPress();
    }
    while ((M5.BtnA.isPressed()) || (M5.BtnB.isPressed())) {
        M5.update();
        checkAXPPress();
        if (SoundEnabled) {
            M5.Beep.tone(4000);
        }
    }
    M5.Beep.mute();
    Disbuff.setTextColor(TFT_WHITE);
}

void MPU6886Test_heads_up(bool show_cube) {
    double theta_reference = 0;
    double phi_reference = 0;
    double theta = 0, last_theta = 0;
    double phi = 0, last_phi = 0;
    double alpha = 0.2;
    AccelVector reference_accel = {0, 0, 1};
    RemoteImuSample remote_sample = {0, 0, false, false, 0};

    if (DEVICE_IS_SENSOR_NODE) {
        readTiltSample(&theta_reference, &phi_reference, &reference_accel);
        theta = last_theta = theta_reference;
        phi = last_phi = phi_reference;
    }

    line_3d_t rect_source[12];
    bool cube_prepared = false;

    bool button_a_tracking = false;
    bool geometry_toggle_handled = false;
    uint32_t button_a_press_started_ms = 0;
    bool ellipse_mode = START_IN_ELLIPSE_MODE;
    bool button_b_tracking = false;
    bool sound_toggle_handled = false;
    uint32_t button_b_press_started_ms = 0;

    while (true) {
        double raw_theta = 0;
        double raw_phi = 0;
        AccelVector current_accel = {0, 0, 1};
        if (DEVICE_IS_SENSOR_NODE) {
            readTiltSample(&raw_theta, &raw_phi, &current_accel);

            theta = alpha * raw_theta + (1 - alpha) * last_theta;
            phi   = alpha * raw_phi + (1 - alpha) * last_phi;
        } else {
            readRemoteImuSample(&remote_sample);
        }

        if (M5.BtnA.wasPressed() ||
            (!button_a_tracking && M5.BtnA.isPressed())) {
            button_a_tracking = true;
            geometry_toggle_handled = false;
            button_a_press_started_ms = M5.BtnA.lastChange();
        }

        if (button_a_tracking && !geometry_toggle_handled &&
            M5.BtnA.pressedFor(GEOMETRY_TOGGLE_HOLD_MS)) {
            ellipse_mode = !ellipse_mode;
            geometry_toggle_handled = true;
        }

        if (button_a_tracking &&
            M5.BtnA.releasedFor(BUTTON_RELEASE_ARM_MS)) {
            uint32_t button_a_held_ms =
                M5.BtnA.lastChange() - button_a_press_started_ms;
            if (DEVICE_IS_SENSOR_NODE && !geometry_toggle_handled &&
                button_a_held_ms >= REFERENCE_RESET_HOLD_MS) {
                theta_reference = theta;
                phi_reference = phi;
                reference_accel = current_accel;
                startFeedbackBeep(REFERENCE_RESET_BEEP_FREQUENCY,
                                  REFERENCE_RESET_BEEP_MS);
            }
            button_a_tracking = false;
        }

        if (M5.BtnB.wasPressed() ||
            (!button_b_tracking && M5.BtnB.isPressed())) {
            button_b_tracking = true;
            sound_toggle_handled = false;
            button_b_press_started_ms = M5.BtnB.lastChange();
        }

        if (button_b_tracking && !sound_toggle_handled &&
            M5.BtnB.pressedFor(SOUND_TOGGLE_HOLD_MS)) {
            toggleSoundEnabled();
            sound_toggle_handled = true;
        }

        if (button_b_tracking &&
            M5.BtnB.releasedFor(BUTTON_RELEASE_ARM_MS)) {
            uint32_t button_b_held_ms =
                M5.BtnB.lastChange() - button_b_press_started_ms;
            if (!sound_toggle_handled &&
                button_b_held_ms >= LIMIT_CYCLE_HOLD_MS) {
                cycleHeadsUpLimits();
                startFeedbackBeep(LIMIT_CYCLE_BEEP_FREQUENCY,
                                  LIMIT_CYCLE_BEEP_MS);
            }
            button_b_tracking = false;
        }

        double roll_delta = 0;
        double pitch_delta = 0;
        bool inverted = false;
        bool have_attitude_sample = DEVICE_IS_SENSOR_NODE;
        if (DEVICE_IS_SENSOR_NODE) {
            roll_delta = theta - theta_reference;
            pitch_delta = phi - phi_reference;
            inverted = isInvertedFromReference(&current_accel, &reference_accel);
            sendImuLinkSample(pitch_delta, roll_delta, inverted);
        } else if (remoteImuSampleIsFresh(&remote_sample)) {
            roll_delta = remote_sample.roll_degrees;
            pitch_delta = remote_sample.pitch_degrees;
            inverted = remote_sample.inverted;
            have_attitude_sample = true;
        }
        printImuLinkRuntimeDiagnostics(&remote_sample, have_attitude_sample);

        Disbuff.fillRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                         DISPLAY_BACKGROUND_COLOR);
        drawDeviceRoleLabel(&Disbuff, DEVICE_IS_SENSOR_NODE);

        if (!have_attitude_sample) {
            drawImuLinkWaitingLabel(&Disbuff);
            // Suppress sounds
            // updateHeadsUpAlerts(HeadsUpAlertState::Alarm);
            Displaybuff();
            M5.update();
            checkAXPPress();
            continue;
        }

        bool draw_cube = show_cube && !ellipse_mode;
        if (draw_cube && !cube_prepared) {
            prepareHeadsUpCube(rect_source);
            cube_prepared = true;
        }
        if (draw_cube) {
            drawHeadsUpCube(&Disbuff,
                            DEVICE_IS_SENSOR_NODE ? theta : roll_delta,
                            DEVICE_IS_SENSOR_NODE ? phi : pitch_delta,
                            rect_source);
        }
        HeadsUpAlertState alert_state =
            drawHeadsUpPlot(&Disbuff, pitch_delta, roll_delta, ellipse_mode,
                            inverted);
        drawMessageRate(&Disbuff);
        updateHeadsUpAlerts(alert_state);

        Displaybuff();
        last_theta = theta;
        last_phi   = phi;

        M5.update();
        checkAXPPress();
    }
}

SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t start_dis  = NULL;
SemaphoreHandle_t start_fft  = NULL;
int8_t i2s_readraw_buff[2048];
uint8_t fft_dis_buff[241][128] = {0};
uint16_t posData               = 160;

uint8_t addrcheckbuff[3] = {
    0x34,  //
    0x51,  //
    0x68   //
};

int checkI2CAddr() {
    uint8_t faild_count = 0;

    do {
        faild_count = 0;
        for (int i = 0; i < sizeof(addrcheckbuff); i++) {
            Wire1.beginTransmission(addrcheckbuff[i]);
            if (Wire1.endTransmission() == ESP_OK) {
                Serial.printf("find %02X addr successful\r\n",
                              addrcheckbuff[i]);
            } else {
                Serial.printf("find %02X addr faild\r\n", addrcheckbuff[i]);
                char strbuff[128];
                sprintf(strbuff, "i2c %02X fail", addrcheckbuff[i]);

                faild_count++;
            }
        }
    } while (faild_count != 0);

    return 0;
}


void setup() 
{
    M5.begin();
    M5.Axp.ScreenBreath(100);    
    Serial.begin(115200);
    if (HC12_AT_DIAGNOSTICS) {
        startupWaitMs(HC12_DIAGNOSTIC_SERIAL_WAIT_MS);
        Serial.println();
        Serial.println("HUD boot: starting HC-12 diagnostics");
    }
    setupImuLinkSerial();

    pinMode(M5_LED, OUTPUT);
    setRedLed(false);

    M5.Lcd.setRotation(0);

    checkI2CAddr();

    checkAXP192();

    Disbuff.createSprite(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    Disbuff.fillRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                     DISPLAY_BACKGROUND_COLOR);
    Disbuff.pushSprite(0, 0);

    if (DEVICE_IS_SENSOR_NODE) {
        M5.Imu.Init();
    }
}

void loop() {
    M5.update();
    MPU6886Test_heads_up(showCube);
}
