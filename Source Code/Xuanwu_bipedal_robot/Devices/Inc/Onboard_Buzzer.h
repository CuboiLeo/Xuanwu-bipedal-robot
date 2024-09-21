#ifndef ONBOARD_BUZZER_H
#define ONBOARD_BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

class Buzzer {
    public:
    void Init();
    void Beep();
};

#ifdef __cplusplus
}
#endif

#endif // ONBOARD_BUZZER_H
