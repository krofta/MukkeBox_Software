#ifndef defines_h
#define defines_h


#define enable 1
#define disable 0

// I2C bus address
#define TDA_ADDR (0x44<<1)


// 0x00 Main Selector   Bitmasks
#define MAIN_IN0    0x00
#define MAIN_IN1    0x01
#define MAIN_IN2    0x02
#define MAIN_IN3    0x03
#define MAIN_IN4    0x04
#define MAIN_IN5    0x05
#define MAIN_IN6    0x06
#define MAIN_IN7    0x07


#define MAIN_MD    0x08

#define MAIN_3DB    0x10

#define MAIN_CFG0   0x00
#define MAIN_CFG1   0x20
#define MAIN_CFG2   0x40
#define MAIN_CFG3   0x60
#define MAIN_CFG4   0x80
#define MAIN_CFG5   0xA0
#define MAIN_CFG6   0xC0
#define MAIN_CFG7   0xE0

// 0x01 Second Source selector
#define SECOND_IN0    0x00
#define SECOND_IN1    0x01
#define SECOND_IN2    0x02
#define SECOND_IN3    0x03
#define SECOND_IN4    0x04
#define SECOND_IN5    0x05
#define SECOND_IN6    0x06
#define SECOND_IN7    0x07

#define SECOND_MD1    0x00
#define SECOND_MD2    0x80

#define SECOND_0DB    0x00
#define SECOND_3DB    0x10

#define SECOND_BYPASS_FON   0x00
#define SECOND_BYPASS_FOFF  0x20
#define SECOND_BYPASS_RON   0x00
#define SECOND_BYPASS_ROFF  0x40
#define SECOND_BYPASS_SUBON   0x00
#define SECOND_BYPASS_SUBOFF  0x80

// 0x02 Mixing Source Selector
#define MIX_IN0 0x00
#define MIX_IN1 0x01
#define MIX_IN2 0x02
#define MIX_IN3 0x03
#define MIX_IN4 0x04
#define MIX_IN5 0x05
#define MIX_IN6 0x06
#define MIX_IN7 0x07

// 0x03 Mix Control
#define MIX_CON_TOFL    0x01
#define MIX_CON_TOFR    0x02
#define MIX_CON_TORL    0x04
#define MIX_CON_TORR    0x08
#define MIX_RINCONF     0x10
#define MIX_CON_VREF    0x20
#define MIX_CON_LM_RES  0x04
#define MIX_CON_DCOFFS  0x08

// 0x04 Soft Mute
#define SOFTMUTE_ONOFF  0x01
#define SOFTMUTE_PIN    0x02
#define SOFTMUTE_TIME   0x0C
#define SOFTMUTE_SUBINCONF  0x01
#define SOFTMUTE_SUBEN  0x20
#define SOFTMUTE_FASTCHARGE 0x40
#define SOFTMUTE_ANTIALIAS  0x80

// 0x05 SOFT STEP
#define SOFTSTEP_LOUDNESS   0x01
#define SOFTSTEP_VOL        0x02
#define SOFTSTEP_TREB       0x04
#define SOFTSTEP_MID        0x08
#define SOFTSTEP_BASS       0x10
#define SOFTSTEP_LF         0x20
#define SOFTSTEP_RF         0x40
#define SOFTSTEP_LR         0x80

// 0x06 SOFTSTEP2
#define SOFTSTEP2_RR        0x01
#define SOFTSTEP2_SUBL      0x02
#define SOFTSTEP2_SUBR      0x04
#define SOFTSTEP_TIME       0x08
#define SOFTSTEP_WINSIZE    0x30
#define SOFTSTEP_REJECT     0xC0

// 0x07 LOUDNESS
#define LOUDNESS_ATTENUATION    0x0F
#define LOUDNESS_CENTER_FREQ    0x30
#define LOUDNESS_HIGH_BOOST     0x40
#define LOUDNESS_SOFTSTEP_ACT   0x80


// 0x08
#define VOLUME_OUTPUT_GAIN	6
#define VOLUME_SOFT_STEP 7

// 0x0B
#define BASS_Q_1	5
#define BASS_Q_2	6
#define BASS_SOFT_STEP 7

// 0x0A
#define MIDDLE_Q_1	5
#define MIDDLE_Q_2	6
#define MIDDLE_SOFT_STEP 7

// 0x09
#define TREBLE_Q_1	5
#define TREBLE_Q_2	6
#define TREBLE_SOFT_STEP 7





// Input channels definitions
#define INPUT_SE1 0x01
#define INPUT_SE2 0x02
#define INPUT_SE3 0x03
#define INPUT_SE4_PD 0x00
#define INPUT_MUTE 0x04
#define PD 1
#define SE 0

#define VOL_MUTE -80

// Loudness
// LOW is already defined as 0
#define LOWHIGH 1
#define FLAT 0

// Middle settings
#define MID_QF_050 0
#define MID_QF_075 1
#define MID_QF_100 2
#define MID_QF_125 3

// Bass settings
#define BASS_QF_100 0
#define BASS_QF_125 1
#define BASS_QF_150 2
#define BASS_QF_200 3

// Soft Step/Soft mute times
#define SMT_048 0
#define SMT_096 1
#define SMT_123 2
#define SST_016 0
#define SST_032 1
#define SST_064 2
#define SST_128 3
#define SST_256 4
#define SST_512 5
#define SST_1024 6
#define SST_2048 7

// Bitmasks & etc...
#define VOL_OFFSET 0x10
#define PWRON_DEFAULT 0xFF
#define MASK_INPUT 0x07
#define MASK_INPUTGAIN 0x78
#define MASK_LOUDATT	0x0F
#define MASK_LOUDFREQ	0x30
#define MASK_VOLUME		0x7F
#define MASK_SST		0x38
#define MASK_SMT		0x06
#define MASK_TESTMUX	0x1C
#define MASK_MUTEPIN	0x82
#define MASK_QF		0x60
#define MASK_BASSFREQ	0x0C
#define MASK_MIDDLEFREQ	0x03
#define MASK_TREBLEFREQ	0x60
#define MASK_ATT	0x1F
#define MASK_ATTPOS	0x1F
#define MASK_ATTNEG	0x0F

// Test register settings
#define InMuxOutR   0
#define LoudOutR    1
#define VolumeOutR  2
#define VGB126      3
#define REF5V5      4
#define SSCLK       5
#define SMCLK       6
#define Clk200kHz   7
#define NORMAL_MODE 0
#define FAST_MODE   1
#define MUX_OUT     0x00
#define EXT_CLK     0x80
#define MUTE_PIN    0x82

// Registers map
#define AUTOINCREMENT 0x20

#define REG_SOURCE_SEL  0x00
#define REG_2SOURCE_SEL 0x01
#define REG_MIX_SOURCE  0x02
#define REG_LEVELMETER  0x03
#define REG_SOFTMUTE    0x04
#define REG_SOFT_STEP1   0x05
#define REG_SOFT_STEP2   0x06

#define REG_LOUDNESS    0x07
#define REG_VOLUME      0x08
#define REG_TREBLE      0x09
#define REG_MIDDLE      0x0A
#define REG_BASS        0x0B
#define REG_MID_BAS_FC  0x0C
#define REG_SPK_ATT_LF  0x0D
#define REG_SPK_ATT_RF  0x0E
#define REG_SPK_ATT_LR  0x0F
#define REG_SPK_ATT_RR  0x10
#define REG_SPK_ATT_SUBL 0x11
#define REG_SPK_ATT_SUBR 0x12




#endif
