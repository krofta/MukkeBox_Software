#include "TDA7719.h"

#include "defines.h"
#include "main.h"
#include "TDA7719.h"

#define DEBUG_MODE

uint8_t TDA7418_register_data[REG_SPK_ATT_SUBR + 1];
I2C_HandleTypeDef i2c;


int TDA7719_begin(I2C_HandleTypeDef dev) {
	i2c = dev;
	TDA7418_register_data[REG_SOURCE_SEL ] 	= 0b11100011;
	TDA7418_register_data[REG_2SOURCE_SEL] 	= 0b11111011;
	TDA7418_register_data[REG_MIX_SOURCE ] 	= 0b00111111;
	TDA7418_register_data[REG_LEVELMETER ] 	= 0b00111111;
	TDA7418_register_data[REG_SOFTMUTE   ] 	= 0b11010011;
	TDA7418_register_data[REG_SOFT_STEP1 ] 	= 0b11111111;
	TDA7418_register_data[REG_SOFT_STEP2 ] 	= 0b11001111;

	TDA7418_register_data[REG_LOUDNESS] 	= 0b11110000;
	TDA7418_register_data[REG_VOLUME] 		= 0b11011111;
	TDA7418_register_data[REG_TREBLE] 		= 0b11111111;
	TDA7418_register_data[REG_MIDDLE] 		= 0b11111111;
	TDA7418_register_data[REG_BASS] 		= 0b11110000;	// +15dB
	TDA7418_register_data[REG_MID_BAS_FC] 	= 0b11111111;
	TDA7418_register_data[REG_SPK_ATT_LF] 	= 0b10000000;	// 0dB
	TDA7418_register_data[REG_SPK_ATT_RF] 	= 0b10000000;	// 0dB
	TDA7418_register_data[REG_SPK_ATT_LR] 	= 0b11100000;	// Mute
	TDA7418_register_data[REG_SPK_ATT_RR] 	= 0b11100000;	// Mute
	TDA7418_register_data[REG_SPK_ATT_SUBL] = 0b10000000;	// 0dB
	TDA7418_register_data[REG_SPK_ATT_SUBR] = 0b10000000;	// 0dB

	  char data[2] = {0,0};
	  HAL_StatusTypeDef ret;
	  for(int i = 0; i<= REG_SPK_ATT_SUBR; i++){
		  data[0] = i;
		  data[1] = TDA7418_register_data[i];
		  ret = HAL_I2C_Master_Transmit(&i2c, TDA_ADDR, &data, 2, 1000);
		  if(ret != HAL_OK)
			  return -1;
	  }
    return ret;
}



int TDA7719_mainSource(char _source) {
	if (_source >= MAIN_MD) {
		return -1;
	}
	TDA7418_register_data[REG_SOURCE_SEL] &= ~MASK_INPUT;
	TDA7418_register_data[REG_SOURCE_SEL] |= _source & MASK_INPUT;
    return TDA7719_write_register(REG_SOURCE_SEL);
}

int TDA7719_write_register(char _register) {

	HAL_StatusTypeDef ret;
	char data[2] = {_register, TDA7418_register_data[_register]};
	ret = HAL_I2C_Master_Transmit(&i2c, TDA_ADDR, &data, 2, 1000);
    return ret;
}
/*
int TDA7719_volume(int8_t _volume) {
	HAL_StatusTypeDef ret;
	if((_volume > 15) || (_volume < -15))
		return -1;

	TDA7418_register_data[REG_VOLUME] = (1 << VOLUME_OUTPUT_GAIN) | (1 << VOLUME_SOFT_STEP);
	if(_volume > 0 )
		TDA7418_register_data[REG_VOLUME] |= (1<<4);	// Vorzeichen Bit

	TDA7418_register_data[REG_VOLUME] +=
	ret = TDA7719_write_register(REG_VOLUME);
	return ret;
}

int TDA7719_attenuator(char _value) {
    byte _set_att;

	if (_value < -80 || _value > 15) {
		return -1;
	}

    // bits 0x00 - 0x0F -> 0dBto +15dB
    if (_value < VOL_OFFSET && _value >= 0) {
        _set_att = _value;
    }
    else {
        _set_att = VOL_OFFSET + -_value;
    }

    // Batch write attenuators 7 - 11
    Wire.beginTransmission(TDA_ADDR);
    Wire.write(REG_SPK_ATT_FL + AUTOINCREMENT);

    // Write the 5 attenuators with the same value
    for (byte x = REG_SPK_ATT_FL; x <= REG_SPK_ATT_SUB; x++) {
        _register_data[x] = _set_att;
        Wire.write(_register_data[x]);
    }

    return Wire.endTransmission(); // End batch write attenuators 7 - 11
}
*/
/*
int TDA7719_inputGain(byte _value) {

	if (_value > 15) {
		return -1;
	}

    _register_data[REG_SOURCE_SEL] &= ~MASK_INPUTGAIN;
    _register_data[REG_SOURCE_SEL] |= _value << 3 & MASK_INPUTGAIN;

    return _write_register(REG_SOURCE_SEL);
}


int TDA7719_diffinMode(byte _mode) {

	if (_mode > 1) {
      return -1;
    }

    if (_mode) {
        _register_data[REG_SOURCE_SEL] |= (1 << 7);
    }
    else {
        _register_data[REG_SOURCE_SEL] &= ~(1 << 7);
    }

    return _write_register(REG_SOURCE_SEL);
}


int TDA7719_loudnessAttenuator(int _value) {
    
	
	if (_value < -15 || _value > 0) {
		return -1;
	}

    _value = abs(_value); // ditch sign bit

    _register_data[REG_LOUDNESS] &= ~MASK_LOUDATT;
    _register_data[REG_LOUDNESS] |= _value & MASK_LOUDATT;

	return _write_register(REG_LOUDNESS);
}


int TDA7719_loudnessCenterFreq(int _freq) {

    byte _value = 0;

    switch (_freq) {
        case FLAT:
            _value = 0;
            loudnessShape(LOW);
        break;

        case 400:
            _value = 1;
        break;

        case 800:
            _value = 2;
        break;

        case 2400:
            _value = 3;
        break;
		
		default:
			return -1;
		break;
    }
	
	_register_data[REG_LOUDNESS] &= ~MASK_LOUDFREQ;
    _register_data[REG_LOUDNESS] |= _value << 4 & MASK_LOUDFREQ;

	return _write_register(REG_LOUDNESS);
}


int TDA7719_loudnessShape(byte _shape) {

	if (_shape > 1) {
      return -1;
    }


    if (_shape) {
        _register_data[REG_LOUDNESS] |= (1 << 6);
    }
    else {
        _register_data[REG_LOUDNESS] &= ~(1 << 6);
    }

    return _write_register(REG_LOUDNESS);
}


int TDA7719_loudnessSoftStep(byte _state) {

	if (_state > 1) {
      return -1;
    }

    if (_state) {
        _register_data[REG_LOUDNESS] &= ~(1 << 7);
    }
    else {
        _register_data[REG_LOUDNESS] |= (1 << 7);
    }

    return _write_register(REG_LOUDNESS);
}


int TDA7719_volume(char _volume) {
	return attenuator(REG_VOLUME, _volume);
}


int TDA7719_volumeSoftStep(byte _state) {

	if (_state > 1) {
      return -1;
    }

    if (_state) {
        _register_data[REG_VOLUME] &= ~(1 << 7);
    }
    else {
        _register_data[REG_VOLUME] |= (1 << 7);
    }

    return _write_register(REG_VOLUME);
}


int TDA7719_trebleAttenuator(int _value) {
    int _result;

	if (_value < -15 || _value > 15) {
		return -1;
	}

    if (_value >= 0) {
        _result = MASK_ATTPOS - _value;
    }
    else {
        _result = _value + MASK_ATTNEG;
    }

    _register_data[REG_TREBLE] &= ~MASK_ATT;
    _register_data[REG_TREBLE] |= _result & MASK_ATT;

    return _write_register(REG_TREBLE);
}


int TDA7719_trebleCenterFreq(int _freq) {
    byte _value = 0;

    switch (_freq) {
        case 10000:
            _value = 0;
        break;

        case 12500:
            _value = 1;
        break;

        case 15000:
            _value = 2;
        break;

        case 17500:
            _value = 3;
        break;

        default:
            return -1;
        break;
    }

    _register_data[REG_TREBLE] &= ~MASK_TREBLEFREQ;
    _register_data[REG_TREBLE] |= _value << 5 & MASK_TREBLEFREQ;

    return _write_register(REG_TREBLE);
}


int TDA7719_middleAttenuator(int _value) {
    int _result;

	if (_value < -15 || _value > 15) {
		return -1;
	}

    if (_value >= 0) {
        _result = MASK_ATTPOS - _value;
    }
    else {
        _result = _value + MASK_ATTNEG;
    }

    _register_data[REG_MIDDLE] &= ~MASK_ATT;
    _register_data[REG_MIDDLE] |= _result & MASK_ATT;

    return _write_register(REG_MIDDLE);
}


int TDA7719_middleCenterFreq(int _freq) {
    byte _value = 0;

    switch (_freq) {
        case 500:
            _value = 0;
        break;

        case 1000:
            _value = 1;
        break;

        case 1500:
            _value = 2;
        break;

        case 2500:
            _value = 3;
        break;

        default:
            return -1;
        break;
    }

    _register_data[REG_MID_BAS_FC] &= ~MASK_MIDDLEFREQ;
    _register_data[REG_MID_BAS_FC] |= _value & MASK_MIDDLEFREQ;

    return _write_register(REG_MID_BAS_FC);
}


int TDA7719_middleQFactor(byte _factor) {

	if (_factor > MID_QF_125) {
		return -1;
	}

    _register_data[REG_MIDDLE] &= ~MASK_QF;
    _register_data[REG_MIDDLE] |= _factor << 5 & MASK_QF;

    return _write_register(REG_MIDDLE);
}


int TDA7719_middleSoftStep(byte _state) {

	if (_state > 1) {
      return -1;
    }

    if (_state) {
        _register_data[REG_MIDDLE] &= ~(1 << 7);
    }
    else {
        _register_data[REG_MIDDLE] |= (1 << 7);
    }

    return _write_register(REG_MIDDLE);
}


int TDA7719_bassAttenuator(int _value) {
    int _result;

	if (_value < -15 || _value > 15) {
		return -1;
	}

    if (_value >= 0) {
        _result = MASK_ATTPOS - _value;
    }
    else {
        _result = _value + MASK_ATTNEG;
    }

    _register_data[REG_BASS] &= ~MASK_ATT;
    _register_data[REG_BASS] |= _result & MASK_ATT;

    return _write_register(REG_BASS);
}


int TDA7719_bassCenterFreq(byte _freq) {
    byte _value = 0;

    switch (_freq) {
        case 60:
            _value = 0;
        break;

        case 80:
            _value = 1;
        break;

        case 100:
            _value = 2;
        break;

        case 200:
            _value = 3;
        break;

        default:
            return -1;
        break;
    }

    _register_data[REG_MID_BAS_FC] &= ~MASK_BASSFREQ;
    _register_data[REG_MID_BAS_FC] |= _value << 2 & MASK_BASSFREQ;

    return _write_register(REG_MID_BAS_FC);
}


int TDA7719_bassQFactor(byte _factor) {

	if (_factor > BASS_QF_200) {
		return -1;
	}

    _register_data[REG_BASS] &= ~MASK_QF;
    _register_data[REG_BASS] |= _factor << 5 & MASK_QF;

    return _write_register(REG_BASS);
}


int TDA7719_bassSoftStep(byte _state) {

	if (_state > 1) {
      return -1;
    }

    if (_state) {
        _register_data[REG_BASS] &= ~(1 << 7);
    }
    else {
        _register_data[REG_BASS] |= (1 << 7);
    }

    return _write_register(REG_BASS);
}


int TDA7719_bassDCMode(byte _state) {

	if (_state > 1) {
      return -1;
    }

    if (_state) {
        _register_data[REG_MID_BAS_FC] |= (1 << 4);        
    }
    else {
        _register_data[REG_MID_BAS_FC] &= ~(1 << 4);
    }

    return _write_register(REG_MID_BAS_FC);
}


int TDA7719_smoothingFilter(byte _state) {

	if (_state > 1) {
      return -1;
    }

    if (_state) {
        _register_data[REG_MID_BAS_FC] |= (1 << 5);        
    }
    else {
        _register_data[REG_MID_BAS_FC] &= ~(1 << 5);
    }

    return _write_register(REG_MID_BAS_FC);
}


int TDA7719_softMute() {
    byte sm_state;

    Wire.beginTransmission(TDA_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.requestFrom(TDA_ADDR, 1);


    if (Wire.available()) {
        sm_state = Wire.read();
    }

    return sm_state;
}


int TDA7719_softMute(byte _state) {

	if (_state > 1) {
      return -1;
    }

    if (_state) { // Softmute On request
        _register_data[REG_SOFTMUTE] &= ~(1 << 0);
    }
    else {
        _register_data[REG_SOFTMUTE] |= (1 << 0);
    }

    return _write_register(REG_SOFTMUTE);
}


int TDA7719_softMuteTime(byte _value) {

	if (_value > SMT_123) {
		return -1;
	}

    _register_data[REG_SOFTMUTE] &= ~MASK_SMT;
    _register_data[REG_SOFTMUTE] |= _value << 1 & MASK_SMT;

    return _write_register(REG_SOFTMUTE);
}


int TDA7719_softStepTime(byte _value) {

	if (_value > SST_2048) {
		return -1;
	}

    _register_data[REG_SOFTMUTE] &= ~MASK_SST;
    _register_data[REG_SOFTMUTE] |= _value << 3 & MASK_SST;

    return _write_register(REG_SOFTMUTE);
}


int TDA7719_autoZero(byte _state) {

	if (_state > 1) {
      return -1;
    }

    if (_state) {
        _register_data[REG_SOFTMUTE] |= (1 << 6);
    }
    else {
        _register_data[REG_SOFTMUTE] &= ~(1 << 6);
    }

    return _write_register(REG_SOFTMUTE);
}


int TDA7719_testMode(byte _state) {

	if (_state > 1) {
      return -1;
    }

    if (_state) {
        _register_data[REG_AUDIO_TEST] |= (1 << 0);
    }
    else {
        _register_data[REG_AUDIO_TEST] &= ~(1 << 0);
    }

    return _write_register(REG_AUDIO_TEST);
}


int TDA7719_testMux(byte _value) {

	if (_value > Clk200kHz) {
		return -1;
	}

    _register_data[REG_AUDIO_TEST] &= ~MASK_TESTMUX;
    _register_data[REG_AUDIO_TEST] |= _value  << 2 & MASK_TESTMUX;

    return _write_register(REG_AUDIO_TEST);
}


int TDA7719_schLock(byte _state) {

	if (_state > 1) {
      return -1;
    }

    if (_state) {
        _register_data[REG_AUDIO_TEST] |= (1 << 6);
    }
    else {
        _register_data[REG_AUDIO_TEST] &= ~(1 << 6);
    }

    return _write_register(REG_AUDIO_TEST);
}


int TDA7719_mutePinConfig(byte _value) {

	if (_value != 0 && _value != 0x80 && _value != 0x82) {
		return -1;
	}

    _register_data[REG_AUDIO_TEST] &= ~MASK_MUTEPIN;
    _register_data[REG_AUDIO_TEST] |= _value & MASK_MUTEPIN;

    return _write_register(REG_AUDIO_TEST);
}


// Set all atenuators in auto-increment mode (batch writes)
int TDA7719_attenuator(char _value) {
    byte _set_att;

	if (_value < -80 || _value > 15) {
		return -1;
	}

    // bits 0x00 - 0x0F -> 0dBto +15dB
    if (_value < VOL_OFFSET && _value >= 0) {
        _set_att = _value;
    }
    else {
        _set_att = VOL_OFFSET + -_value;
    }

    // Batch write attenuators 7 - 11
    Wire.beginTransmission(TDA_ADDR);
    Wire.write(REG_SPK_ATT_FL + AUTOINCREMENT);

    // Write the 5 attenuators with the same value
    for (byte x = REG_SPK_ATT_FL; x <= REG_SPK_ATT_SUB; x++) {
        _register_data[x] = _set_att;
        Wire.write(_register_data[x]);
    }

    return Wire.endTransmission(); // End batch write attenuators 7 - 11
}


// Set single attenuator
int TDA7719_attenuator(byte _channel, char _value) {
    byte _set_att;

	if (_value < -80 || _value > 15) {
		return -1;
	}

    // bits 0x00 - 0x0F -> 0dBto +15dB
    if (_value < VOL_OFFSET && _value >= 0) {
        _set_att = _value;
    }
    else {
        _set_att = VOL_OFFSET + -_value;
    }

    _register_data[_channel] = _set_att;

    return _write_register(_channel);
}



void TDA7719_printreg(byte _reg) {
	Serial.print("Reg ");
	Serial.print(_reg);
	Serial.print(" value: ");
	Serial.println(_register_data[_reg], HEX);
}
*/
