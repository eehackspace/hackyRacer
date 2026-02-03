#include "Arduino.h"
#include "ODriveArduino.h"


//ODriveArduino::ODriveArduino(Stream &serial)
ODriveArduino::ODriveArduino(Stream &serial)
    : serial_(serial) {}

void ODriveArduino::WaitIdle(int motor_number, float timeout)
{
    unsigned long timeout_ms = (unsigned long)timeout * 1000;
    unsigned long start_ms = millis();
    int state0 = -1, state1 = -1;

    do {
        delay(100);
        if (motor_number == 0 || motor_number == -1)
            serial_ << "r axis0.current_state\n";
        if (motor_number == 1 || motor_number == -1)
            serial_ << "r axis1.current_state\n";

        if (motor_number == 0 || motor_number == -1)
            state0 = readInt();
        if (motor_number == 1 || motor_number == -1)
            state1 = readInt();

        if (motor_number == 0)
            if (state0 == AXIS_STATE_IDLE) break;
        if (motor_number == 1)
            if (state1 == AXIS_STATE_IDLE) break;
        if (motor_number == -1)
            if (state0 == AXIS_STATE_IDLE && state1 == AXIS_STATE_IDLE) break;

    } while (millis() - start_ms < timeout_ms);
}

void ODriveArduino::WriteProperty(int motor_number, const char *property, double value) {
    serial_ << "w axis" << motor_number << "." << property << " " << value << "\n";
}

void ODriveArduino::WriteProperty(int motor_number, const char *property, float value) {
    serial_ << "w axis" << motor_number << "." << property << " " << value << "\n";
}

void ODriveArduino::WriteProperty(int motor_number, const char *property, int value) {
    serial_ << "w axis" << motor_number << "." << property << " " << value << "\n";
}

void ODriveArduino::ReadProperty(int motor_number, const char *property, double *value) {
    serial_ << "r axis" << motor_number << "." << property << "\n";
    *value = readFloat();
}

void ODriveArduino::ReadProperty(int motor_number, const char *property, float *value) {
    serial_ << "r axis" << motor_number << "." << property << "\n";
    *value = readFloat();
}

void ODriveArduino::ReadProperty(int motor_number, const char *property, int *value) {
    serial_ << "r axis" << motor_number << "." << property << "\n";
    *value = readInt();
}

void ODriveArduino::SetPosition(int motor_number, float position)
{
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward)
{
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    serial_ << "p " << motor_number << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

void ODriveArduino::SetVelocity(int motor_number, float velocity)
{
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    serial_ << "v " << motor_number << " " << velocity << " " << current_feedforward << "\n";
}

void ODriveArduino::SetCurrent(int motor_number, float current)
{
    serial_ << "c " << motor_number << " " << current << "\n";
}

void ODriveArduino::TrapezoidalMove(int motor_number, float position)
{
    serial_ << "t " << motor_number << " " << position << "\n";
}

// -- Comandos Optimizados para Ambos Motores --

void ODriveArduino::SetVelocityBoth(float velocity_M0, float velocity_M1) {
    serial_ << "v 0 " << velocity_M0 << "\nv 1 " << velocity_M1 << "\n";
}

void ODriveArduino::SetVelocityBoth_GetFeedback(float set_VM0, float set_VM1, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1) {
    CleanSerial();
    serial_ << "f 0\nf 1\nv 0 " << set_VM0 << "\nv 1 " << set_VM1 << "\n";
    *position_M0 = readFloat();
    *velocity_M0 = readFloat();
    *position_M1 = readFloat();
    *velocity_M1 = readFloat();
}

void ODriveArduino::SetVelocityBoth_GetFeedback_Vbus(float set_VM0, float set_VM1, float *vbus, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1) {
    CleanSerial();
    serial_ << "f 0\nf 1\nr vbus_voltage\nv 0 " << set_VM0 << "\nv 1 " << set_VM1 << "\n";
    *position_M0 = readFloat();
    *velocity_M0 = readFloat();
    *position_M1 = readFloat();
    *velocity_M1 = readFloat();
    *vbus = readFloat();
}

void ODriveArduino::SetVelocityBoth_GetVbus(float set_VM0, float set_VM1, float *vbus) {
    CleanSerial();
    serial_ << "r vbus_voltage\nv 0 " << set_VM0 << "\nv 1 " << set_VM1 << "\n";
    *vbus = readFloat();
}

// -- Getters y Feedback --

float ODriveArduino::GetVelocity(int motor_number) {
    serial_ << "r axis" << motor_number << ".encoder.vel_estimate\n";
    return readFloat();
}

float ODriveArduino::GetPosition(int motor_number)
{
    serial_ << "r axis" << motor_number << ".encoder.pos_estimate\n";
    return readFloat();
}

void ODriveArduino::GetFeedback(int motor_number, float *velocity_M, float *position_M) {
    CleanSerial();
    serial_ << "f " << motor_number << "\n";
    *position_M = readFloat();
    *velocity_M = readFloat();
}

void ODriveArduino::GetFeedbackBoth(float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1) {
    CleanSerial();
    serial_ << "f 0\nf 1\n";
    *position_M0 = readFloat();
    *velocity_M0 = readFloat();
    *position_M1 = readFloat();
    *velocity_M1 = readFloat();
}

bool ODriveArduino::run_state(int axis, int requested_state, bool wait_for_idle, float timeout) {
    int timeout_ctr = (int)(timeout * 100.0f); // Aumentada resolución para delays cortos
    serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait_for_idle) {
        do {
            delay(10); // Delay más corto para respuestas rápidas
            serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }
    return timeout_ctr > 0;
}

float ODriveArduino::readFloat(unsigned long timeout) {
    return readString(timeout).toFloat();
}

int32_t ODriveArduino::readInt(unsigned long timeout) {
    return readString(timeout).toInt();
}

String ODriveArduino::readString(unsigned long timeout) {
    String str = "";
    unsigned long timeout_start = micros();
    while (true) {
        while (!serial_.available()) {
            if (micros() - timeout_start >= timeout) {
                return str; // Timeout
            }
        }
        char c = serial_.read();
        if (c == '\n' || c == ' ') {
            break;
        }
        str += c;
    }
    return str;
}

void ODriveArduino::CleanSerial() {
    while (serial_.available()) {
        serial_.read();
    }
}