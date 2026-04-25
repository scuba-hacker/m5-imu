#include "M5StickCPlus.h"

const bool showCube = false;

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

extern const unsigned char ImageData[768];
extern const unsigned char error_48[4608];
extern const unsigned char icon_ir[4608];
extern const unsigned char icon_ble[4608];
extern const unsigned char icon_wifi[4608];
extern const unsigned char icon_ble_disconnect[4608];

bool TestMode = false;

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
static constexpr int16_t SPOT_RADIUS = 10;
static constexpr int16_t LEVEL_SPOT_RADIUS = 20;
static constexpr int16_t PLOT_LINE_WIDTH = 3;
static constexpr int16_t ATTITUDE_LABEL_Y = PLOT_CENTER_Y - PLOT_RADIUS - 19;
static constexpr int16_t SPOT_LIMIT_LABEL_Y_OFFSET = 8;
static constexpr bool PLOT_BACKGROUND_BLACK = false;
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
static constexpr uint32_t LIMIT_CYCLE_HOLD_MS = 100;
static constexpr uint16_t LIMIT_CYCLE_BEEP_FREQUENCY = 3000;
static constexpr uint32_t LIMIT_CYCLE_BEEP_MS = 100;

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

bool FeedbackBeepActive = false;
uint32_t FeedbackBeepStartedMs = 0;
uint32_t FeedbackBeepDurationMs = 0;

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
    if (TestMode) {
        Disbuff.setTextSize(1);
        Disbuff.setTextColor(TFT_RED);
        Disbuff.drawString("Test Mode", 0, 0, 1);
        Disbuff.setTextColor(TFT_WHITE);
    }
    Disbuff.pushSprite(0, 0);
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

void startFeedbackBeep(uint16_t frequency, uint32_t duration_ms) {
    FeedbackBeepActive = true;
    FeedbackBeepStartedMs = millis();
    FeedbackBeepDurationMs = duration_ms;
    M5.Beep.tone(frequency);
}

bool updateFeedbackBeep(uint32_t now) {
    if (!FeedbackBeepActive) {
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

void readTiltDegrees(double *theta, double *phi) {
    float accX = 0;
    float accY = 0;
    float accZ = 0;

    M5.Imu.getAccelData(&accX, &accY, &accZ);

    *theta = asin(clampDouble(-accX, -1.0, 1.0)) * DEGREES_PER_RADIAN;
    if (accZ != 0) {
        *phi = atan(accY / accZ) * DEGREES_PER_RADIAN;
    } else {
        *phi = 0;
    }
}

HeadsUpAlertState getHeadsUpAlertState(double pitch_degrees,
                                       double roll_degrees,
                                       HeadsUpLimitConfig *limits) {
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
    bool feedback_beep_active = updateFeedbackBeep(now);

    if (alert_state == HeadsUpAlertState::Alarm) {
        level_pulse_active = false;
        setRedLed(true);
        if (feedback_beep_active) {
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
        if (!feedback_beep_active) {
            M5.Beep.mute();
        }
    } else {
        if (!feedback_beep_active) {
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

void drawHeadsUpAttitudeLabel(TFT_eSprite *display, double pitch_degrees,
                              double roll_degrees) {
    char attitude_text[16];
    snprintf(attitude_text, sizeof(attitude_text), "%.0f %.0f", pitch_degrees,
             roll_degrees);
    display->setTextSize(2);
    display->setTextColor(TFT_WHITE);
    display->drawCentreString(attitude_text, PLOT_CENTER_X, ATTITUDE_LABEL_Y,
                              1);
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
    return TFT_BLACK;
}

HeadsUpAlertState drawHeadsUpPlot(TFT_eSprite *display, double pitch_degrees,
                                  double roll_degrees) {
    HeadsUpLimitConfig *limits = currentHeadsUpLimits();

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
        getHeadsUpAlertState(pitch_degrees, roll_degrees, limits);
    if (alert_state == HeadsUpAlertState::Level) {
        spot_radius = LEVEL_SPOT_RADIUS;
        spot_color = TFT_GREEN;
    } else if (alert_state == HeadsUpAlertState::Alarm) {
        spot_radius = LEVEL_SPOT_RADIUS;
        spot_color = TFT_RED;
    }

    int16_t spot_limit = PLOT_RADIUS;
    int16_t spot_x = PLOT_CENTER_X + (int16_t)(plot_x * spot_limit);
    int16_t spot_y = PLOT_CENTER_Y + (int16_t)(plot_y * spot_limit);

    if (alert_state == HeadsUpAlertState::Alarm) {
        uint32_t flash_phase = (millis() / ALARM_SPOT_FLASH_TICK_MS) % 3;
        spot_color = flash_phase == 0 ? TFT_ORANGE : TFT_RED;
    }

    drawHeadsUpAttitudeLabel(display, pitch_degrees, roll_degrees);
    display->fillCircle(PLOT_CENTER_X, PLOT_CENTER_Y, PLOT_RADIUS,
                        plotBackgroundColorForSpot(spot_color));
    drawThickCircle(display, PLOT_CENTER_X, PLOT_CENTER_Y, PLOT_RADIUS,
                    TFT_WHITE);
    drawThickHorizontalLine(display, PLOT_CENTER_X - PLOT_RADIUS,
                            PLOT_CENTER_X + PLOT_RADIUS, PLOT_CENTER_Y,
                            spot_color);
    drawThickVerticalLine(display, PLOT_CENTER_X, PLOT_CENTER_Y - PLOT_RADIUS,
                          PLOT_CENTER_Y + PLOT_RADIUS, spot_color);

    display->fillCircle(spot_x, spot_y, spot_radius, spot_color);
    if (alert_state == HeadsUpAlertState::Level) {
        drawHeadsUpSpotLimitLabel(display, spot_x, spot_y, limits);
    } else if (alert_state == HeadsUpAlertState::Alarm &&
               spot_color == TFT_RED) {
        drawHeadsUpWarningSpotLimitLabel(display, spot_x, spot_y, limits);
    }
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
        M5.Beep.tone(4000);
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

    readTiltDegrees(&theta_reference, &phi_reference);
    theta = last_theta = theta_reference;
    phi = last_phi = phi_reference;

    line_3d_t rect_source[12];
    if (show_cube) {
        prepareHeadsUpCube(rect_source);
    }

    bool reference_reset_armed = true;
    bool limit_cycle_armed = true;

    while (true) {
        double raw_theta = 0;
        double raw_phi = 0;
        readTiltDegrees(&raw_theta, &raw_phi);

        theta = alpha * raw_theta + (1 - alpha) * last_theta;
        phi   = alpha * raw_phi + (1 - alpha) * last_phi;

        if (M5.BtnA.isReleased()) {
            reference_reset_armed = true;
        } else if (reference_reset_armed &&
                   M5.BtnA.pressedFor(REFERENCE_RESET_HOLD_MS)) {
            theta_reference = theta;
            phi_reference = phi;
            reference_reset_armed = false;
            startFeedbackBeep(REFERENCE_RESET_BEEP_FREQUENCY,
                              REFERENCE_RESET_BEEP_MS);
        }

        if (M5.BtnB.isReleased()) {
            limit_cycle_armed = true;
        } else if (limit_cycle_armed &&
                   M5.BtnB.pressedFor(LIMIT_CYCLE_HOLD_MS)) {
            cycleHeadsUpLimits();
            limit_cycle_armed = false;
            startFeedbackBeep(LIMIT_CYCLE_BEEP_FREQUENCY, LIMIT_CYCLE_BEEP_MS);
        }

        double roll_delta = theta - theta_reference;
        double pitch_delta = phi - phi_reference;

        Disbuff.fillRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, TFT_BLACK);

        if (show_cube) {
            drawHeadsUpCube(&Disbuff, theta, phi, rect_source);
        }
        HeadsUpAlertState alert_state =
            drawHeadsUpPlot(&Disbuff, pitch_delta, roll_delta);
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

    Serial.begin(115200);

    Wire.begin(32, 33);

    pinMode(M5_LED, OUTPUT);
    setRedLed(false);

    M5.Lcd.setRotation(0);

    checkI2CAddr();

    checkAXP192();

    Disbuff.createSprite(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    Disbuff.fillRect(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                     Disbuff.color565(10, 10, 10));
    Disbuff.pushSprite(0, 0);

    M5.Imu.Init();
}

void loop() {
    M5.update();
    MPU6886Test_heads_up(showCube);
}
