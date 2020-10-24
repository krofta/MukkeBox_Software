#ifndef tda7719_h
#define tda7719_h
#include "main.h"
extern int TDA7719_begin(I2C_HandleTypeDef dev);

extern int TDA7719_mainSource(char _source);
extern int TDA7719_write_register(char _register);
extern int TDA7719_volume(int8_t _volume);
extern int TDA7719_bass(int8_t _value);
extern int TDA7719_middle(int8_t _value);
extern int TDA7719_treble(int8_t _value);
/*
int TDA7719_inputGain(byte _value);
int TDA7719_diffinMode(byte _mode);

int TDA7719_loudnessAttenuator(int _value);
int TDA7719_loudnessCenterFreq(int _freq);
int TDA7719_loudnessShape(byte _shape);
int TDA7719_loudnessSoftStep(byte _state);


int TDA7719_volumeSoftStep(byte _state);

int TDA7719_trebleAttenuator(int _value);
int TDA7719_trebleCenterFreq(int _freq);

int TDA7719_middleAttenuator(int _value);
int TDA7719_middleCenterFreq(int _freq);
int TDA7719_middleQFactor(byte _factor);
int TDA7719_middleSoftStep(byte _state);

int TDA7719_bassAttenuator(int _value);
int TDA7719_bassCenterFreq(byte _freq);
int TDA7719_bassQFactor(byte _factor);
int TDA7719_bassSoftStep(byte _state);
int TDA7719_bassDCMode(byte _state);

int TDA7719_smoothingFilter(byte _state);

int TDA7719_softMute();
int TDA7719_softMute(byte _state);
int TDA7719_softMuteTime(byte _value);
int TDA7719_softStepTime(byte _value);
int TDA7719_autoZero(byte _state);

int TDA7719_testMode(byte _state);
int TDA7719_testMux(byte _value);
int TDA7719_schLock(byte _state);
int TDA7719_mutePinConfig(byte _value);

int TDA7719_attenuator(char _value);
int TDA7719_attenuator(byte _channel, char _value);

static TDA7719_byte _register_data[14];

void TDA7719_printreg(byte _reg);
*/
#endif
