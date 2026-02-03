#ifndef ODriveArduino_h
#define ODriveArduino_h

#include "Arduino.h"
#include "ODriveEnums.h"

// Print with stream operator
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
    obj.print(arg);
    return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
    // ODrive no necesita más de 4 decimales
    obj.print(arg, 4);
    return obj;
}

class ODriveArduino
{
public:
    ODriveArduino(Stream &serial);

    // -- Funciones de Configuración y Estado (Inline) --
    inline void SaveConfig() { serial_ << "ss\n"; }
    inline void EraseConfig() { serial_ << "se\n"; }
    inline void Reboot() { serial_ << "sr\n"; }
    inline void ClearErrors() { serial_ << "sc\n"; }
    inline void SendCommand(const char *command) { serial_ << command << "\n"; }
    
    // -- Funciones Genéricas --
    void WaitIdle(int motor_number = -1, float timeout = 10.0f);
    void WriteProperty(int motor_number, const char *property, double value);
    void WriteProperty(int motor_number, const char *property, float value);
    void WriteProperty(int motor_number, const char *property, int value);
    void ReadProperty(int motor_number, const char *property, double *value);
    void ReadProperty(int motor_number, const char *property, float *value);
    void ReadProperty(int motor_number, const char *property, int *value);

    // -- Comandos de Movimiento --
    void SetPosition(int motor_number, float position);
    void SetPosition(int motor_number, float position, float velocity_feedforward);
    void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
    void SetVelocity(int motor_number, float velocity);
    void SetVelocity(int motor_number, float velocity, float current_feedforward);
    void SetCurrent(int motor_number, float current);
    void TrapezoidalMove(int motor_number, float position);
    
    // -- Comandos Optimizados para Ambos Motores --
    void SetVelocityBoth(float velocity_M0, float velocity_M1);
    void SetVelocityBoth_GetFeedback(float set_VM0, float set_VM1, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);
    void SetVelocityBoth_GetFeedback_Vbus(float set_VM0, float set_VM1, float *vbus, float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);
    void SetVelocityBoth_GetVbus(float set_VM0, float set_VM1, float *vbus);
    
    // -- Getters y Feedback --
    float GetVelocity(int motor_number);
    float GetPosition(int motor_number);
    void GetFeedback(int motor_number, float *velocity_M, float *position_M);
    void GetFeedbackBoth(float *velocity_M0, float *position_M0, float *velocity_M1, float *position_M1);

    // -- Lectura de bajo nivel --
    float readFloat(unsigned long timeout = 100);
    int32_t readInt(unsigned long timeout = 100);

    // -- Control de Estado --
    bool run_state(int axis, int requested_state, bool wait_for_idle, float timeout = 10.0f);


private:
    String readString(unsigned long timeout = 500);
    void CleanSerial();

    Stream &serial_;
    //Stream &serial_;
};

#endif // ODriveArduino_h