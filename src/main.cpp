#include <Arduino.h>

#define PIN_LIDAR_TX 5
#define PIN_LIDAR_RX 6
#define PIN_LED_R 3
#define PIN_LED_G 4
#define PIN_MOS 7
#define PIN_POT 0

#define CHAN_MOS 5     // LEDC channel to run mosfet on
#define BITS_MOS 10
#define FREQ_MOS 21000 // drive frequency, in hz

#define START_PWM 1023 * .8
#define START_TIMEOUT 1000

#define POT_MIN 100
#define POT_MAX 4095

// Note: Find max.
#define RPM_MIN 120
#define RPM_MAX 320

int target_rpm = 0;

void setup()
{
    Serial.begin(460800);
    //Serial.println("Starting up...");
    Serial1.begin(115200, SERIAL_8N1, PIN_LIDAR_TX, PIN_LIDAR_RX);

    pinMode(PIN_MOS, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_POT, INPUT);

    // setup mosfet LEDC channel
    ledcSetup(CHAN_MOS, FREQ_MOS, BITS_MOS);
    ledcAttachPin(PIN_MOS, 5);
    ledcWrite(CHAN_MOS, 0);

    // setup ADC for potentiometer input
}

struct lidar_data
{
    unsigned distance : 14;
    unsigned warn_strength : 1;
    unsigned invalid : 1;
    unsigned strength : 16;
} __attribute__((packed));

// https://github.com/ssloy/neato-xv11-lidar
union
{
    struct
    {
        uint8_t start;
        uint8_t index;
        uint16_t speed;
        lidar_data data[4];
        uint16_t checksum;
    } p;
    uint8_t buf[sizeof(p)];
} __attribute__((packed)) lidar_packet;

unsigned long last_rx = 0;

// http://forums.trossenrobotics.com/showthread.php?7181-Neato-XV-11-Lidar-reading-problems
bool packet_is_valid()
{
    uint32_t chk32 = 0;
    int offset = 0;

    for (int i = 0; i < 10; i++)
    {
        uint16_t word = (uint16_t)(lidar_packet.buf[offset++] + (lidar_packet.buf[offset++] << 8));
        chk32 = (chk32 << 1) + word;
    }

    uint16_t checksum = (uint16_t)((chk32 & 0x7FFF) + ((chk32 >> 15) & 0x7FFF));
    return checksum == lidar_packet.p.checksum;
}

bool serial_fsm()
{
    static enum {
        IDLING,
        PARSING,
    } state;

    static uint8_t idx;
    static uint32_t corrupt_bytes = 0;

    if (!Serial1.available())
        return false;

    last_rx = millis();
    byte b = lidar_packet.buf[idx++] = Serial1.read();

    switch (state)
    {
    case IDLING:
        if (lidar_packet.p.start == 0xFA)
            state = PARSING;
        else
        {
            // Serial.println("CORRUPTION ON LINE");
            corrupt_bytes++;
            idx = 0;
        }
        break;

    case PARSING:
        if (idx >= sizeof(lidar_packet))
        {
            state = IDLING;
            idx = 0;
            return packet_is_valid();
        }
        break;
    }

    return false;
}

void write_motor(int val)
{
    ledcWrite(CHAN_MOS, val);
}

class
{
public:
    float kp = 0.2, ki = .6, kd = 0;
    int act, act_p, act_i, act_d, err;
    int target;
    int act_max = 1 << BITS_MOS;

    unsigned long t_last = millis();

    int run(int in)
    {
        int err = target - in;
        int d_err = err - this->err;
        int dt = millis() - t_last;

        act_p = err * kp;
        if (act < act_max && act > 0 && dt < 100)
            act_i += err * ki * dt / 1000;
        act_d = 0;

        t_last = millis();
        this->err = err;

        return constrain(act = act_p + act_i + act_d, 0, act_max);
    }
} pid;

void loop()
{
    static unsigned long timer;
    static enum {
        IDLING,
        STARTING,
        START_FAILED,
        RUNNING,
    } state;

    int pot_val = analogRead(PIN_POT);

    target_rpm = constrain(map(pot_val, POT_MIN, POT_MAX, RPM_MIN, RPM_MAX), RPM_MIN, RPM_MAX);

    bool packet_received = serial_fsm();
    if(packet_received) {
        Serial.write(lidar_packet.buf, sizeof(lidar_packet));
    }

    switch (state)
    {
    case IDLING:
        write_motor(0);

        digitalWrite(PIN_LED_R, LOW);
        digitalWrite(PIN_LED_G, LOW);

        if (pot_val <= POT_MIN)
            timer = millis();

        if (millis() - timer > 200)
        {
            state = STARTING;
            timer = millis();
        }
        break;

    case STARTING:
        write_motor(START_PWM);
        digitalWrite(PIN_LED_R, HIGH);
        digitalWrite(PIN_LED_G, HIGH);
        if (millis() - timer > START_TIMEOUT)
        {
            // Serial.println("START FAILED! Is the lidar properly connected??");
            // Serial.println(millis() - last_rx);
            state = START_FAILED;
        }

        if (millis() - last_rx < START_TIMEOUT)
            // Some bytes RXed - Start controlling LIDAR speed.
            state = RUNNING;

        break;

    case START_FAILED:
        digitalWrite(PIN_LED_R, HIGH);
        digitalWrite(PIN_LED_G, LOW);
        write_motor(0);

        if (pot_val < POT_MIN / 4)
            state = IDLING;
        break;

    case RUNNING:
        if (pot_val < POT_MIN / 4)
            state = IDLING;

        if (packet_received)
        {
            pid.target = target_rpm * 64;
            int action = pid.run(lidar_packet.p.speed);
            //Serial.printf("speed: %d - target: %d; err: %d, action: %d, p: %d, i: %d\n", lidar_packet.p.speed, pid.target, pid.err, action, pid.act_p, pid.act_i);
            write_motor(action);
        }
    default:
        break;
    }
}
