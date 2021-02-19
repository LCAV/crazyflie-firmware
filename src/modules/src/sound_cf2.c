/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * sound_cf2.c - Module used to play melodies and system sounds though a buzzer
 */

#include <stdbool.h>

/* FreeRtos includes */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "config.h"
#include "param.h"
#include "log.h"
#include "sound.h"
#include "buzzer.h"

/**
 * Credit to http://tny.cz/e525c1b2 for supplying the tones
 */
#define OFF 0
#define C0 16
#define Db0 17
#define D0  18
#define Eb0 19
#define E0  20
#define F0  21
#define Gb0 23
#define G0  24
#define Ab0 25
#define A0 27
#define Bb0 29
#define B0  30
#define C1  32
#define Db1 34
#define D1  36
#define Eb1 38
#define E1  41
#define F1  43
#define Gb1 46
#define G1  49
#define Ab1 51
#define A1 55
#define Bb1 58
#define B1  61
#define C2  65
#define Db2 69
#define D2  73
#define Eb2 77
#define E2  82
#define F2  87
#define Gb2 92
#define G2  98
#define Ab2 103
#define A2 110
#define Bb2 116
#define B2  123
#define C3  130
#define Db3 138
#define D3  146
#define Eb3 155
#define E3  164
#define F3  174
#define Gb3 185
#define G3  196
#define Ab3 207
#define A3 220
#define Bb3 233
#define B3  246
#define C4  261
#define Db4 277
#define D4  293
#define Eb4 311
#define E4  329
#define F4  349
#define Gb4 369
#define G4  392
#define Ab4 415
#define A4 440
#define Bb4 466
#define B4  493
#define C5  523
#define Db5 554
#define D5  587
#define Eb5 622
#define E5  659
#define F5  698
#define Gb5 739
#define G5  783
#define Ab5 830
#define A5 880
#define Bb5 932
#define B5  987
#define C6  1046
#define Db6 1108
#define D6  1174
#define Eb6 1244
#define E6  1318
#define F6  1396
#define Gb6 1479
#define G6  1567
#define Ab6 1661
#define A6 1760
#define Bb6 1864
#define B6  1975
#define C7  2093
#define Db7 2217
#define D7  2349
#define Eb7 2489
#define E7  2637
#define F7  2793
#define Gb7 2959
#define G7  3135
#define Ab7 3322
#define A7 3520
#define Bb7 3729
#define B7  3951
#define C8  4186
#define Db8 4434
#define D8  4698
#define Eb8 4978
/* Duration of notes */
#define W  1  // 1/1
#define H  2  // 1/2
#define Q  4  // 1/4
#define E  8  // 1/8
#define S  16 // 1/16
#define ES 6
/* End markers */
#define STOP {0xFE, 0}
#define REPEAT {0xFF, 0}

#define MAX_NOTE_LENGTH 1025

static bool isInit=false;

typedef const struct {
  uint16_t tone;
  uint16_t duration;
} Note;

typedef const struct {
  uint32_t bpm;
  uint32_t delay;
  Note notes[MAX_NOTE_LENGTH];
} Melody;

static uint32_t neffect = 0;
static uint32_t sys_effect = 0;
static uint32_t user_effect = 0;
static uint16_t static_freq = 0;


static Melody range_slow = {.bpm = 120, .delay = 1, .notes = {{C4, H}, {D4, H}, {E4, H}, {F4, H}, {G4, H}, {A4, H}, {B4, H}, REPEAT}};
static Melody range_fast = {.bpm = 120, .delay = 1, .notes = {{C4, S}, {D4, S}, {E4, S}, {F4, S}, {G4, S}, {A4, S}, {B4, S}, REPEAT}};
static Melody startup = {.bpm = 120, .delay = 1, .notes = {{C6, S}, {C6, S}, STOP}};
static Melody calibrated = {.bpm = 120, .delay = 1, .notes = {{C4, S}, {E4, S}, {G4, S}, {C5, E}, STOP}};
static Melody chg_done = {.bpm = 120, .delay = 1, .notes = {{D4, Q}, {A4, Q}, STOP}};
static Melody lowbatt = {.bpm = 120, .delay = 1, .notes = {{D4, E}, {A4, E}, {D4, E}, REPEAT}};
static Melody usb_disconnect = {.bpm = 120, .delay = 1, .notes = {{C4, E}, STOP}};
static Melody usb_connect = {.bpm = 120, .delay = 1, .notes = {{A4, E}, STOP}};
static Melody factory_test = {.bpm = 120, .delay = 1, .notes = {{A1, Q}, {OFF, S}, {A2, Q}, {OFF, S}, REPEAT}};
static Melody starwars = {.bpm = 120, .delay = 1, .notes = {{A3, Q}, {A3, Q}, {A3, Q}, {F3, ES}, {C4, S},
    {A3, Q}, {F3, ES}, {C4, S}, {A3, H},
    {E4, Q}, {E4, Q}, {E4, Q}, {F4, ES}, {C4, S},
    {Ab3, Q}, {F3, ES}, {C4, S}, {A3, H},
    {A4, Q}, {A3, ES}, {A3, S}, {A4, Q}, {Ab4, ES}, {G4, S},
    {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
    {C4, S}, {B3, S}, {C4, E}, {0, E}, {F3, E}, {Ab3, Q}, {F3, ES}, {A3, S},
    {C4, Q}, {A3, ES}, {C4, S}, {E4, H},
    {A4, Q}, {A3, ES}, {A3, S}, {A4, Q}, {Ab4, ES}, {G4, S},
    {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
    {Gb4, S}, {E4, S}, {F4, E}, {0, E}, {Bb3, E}, {Eb4, Q}, {D4, ES}, {Db4, S},
    {C4,S}, {B3, S}, {C4, E}, {0, E}, {F3, E}, {Ab3, Q}, {F3, ES}, {C4, S},
    {A3, Q}, {F3, ES}, {C4, S}, {A3, H}, {0, H},
    REPEAT}};
static Melody valkyries = {.bpm = 140, .delay = 1, .notes = {{Gb3, Q}, {B3, Q},
    {Gb3, S}, {B3, E},  {D4, Q}, {B3, Q}, {D4, Q}, {B3, S}, {D4, E}, {Gb4, Q},
    {D4, Q}, {Gb4, Q}, {D4, S}, {Gb4, E}, {A4, Q}, {A3, Q}, {D4, Q}, {A3, S},
    {D4, E}, {Gb4, H},
    REPEAT}};
static Melody sweep = {.bpm = 60, .delay = 1, .notes = {{1000, W}, {1125, Q}, {1250, Q}, {1375, Q}, {1500, Q}, {1625, Q}, {1750, Q}, {1875, Q}, {2000, Q}, {2125, Q}, {2250, Q}, {2375, Q}, {2500, Q}, {2625, Q}, {2750, Q}, {2875, Q}, {3000, Q}, {3125, Q}, {3250, Q}, {3375, Q}, {3500, Q}, {3625, Q}, {3750, Q}, {3875, Q}, {4000, Q}, {4125, Q}, {4250, Q}, {4375, Q}, {4500, Q}, {4625, Q}, {4750, Q}, {4875, Q}, STOP}};
static Melody sweep_high = {.bpm = 60, .delay = 1, .notes = {{2000, W}, {2125, Q}, {2250, Q}, {2375, Q}, {2500, Q}, {2625, Q}, {2750, Q}, {2875, Q}, {3000, Q}, {3125, Q}, {3250, Q}, {3375, Q}, {3500, Q}, {3625, Q}, {3750, Q}, {3875, Q}, {4000, Q}, {4125, Q}, {4250, Q}, {4375, Q}, {4500, Q}, {4625, Q}, {4750, Q}, {4875, Q}, {5000, Q}, {5125, Q}, {5250, Q}, {5375, Q}, {5500, Q}, {5625, Q}, {5750, Q}, {5875, Q}, STOP}};
static Melody sweep_all = {.bpm = 120, .delay = 1, .notes = {{16, W}, {31, Q}, {47, Q}, {62, Q}, {78, Q}, {94, Q}, {109, Q}, {125, Q}, {141, Q}, {156, Q}, {172, Q}, {188, Q}, {203, Q}, {219, Q}, {234, Q}, {250, Q}, {266, Q}, {281, Q}, {297, Q}, {312, Q}, {328, Q}, {344, Q}, {359, Q}, {375, Q}, {391, Q}, {406, Q}, {422, Q}, {438, Q}, {453, Q}, {469, Q}, {484, Q}, {500, Q}, {516, Q}, {531, Q}, {547, Q}, {562, Q}, {578, Q}, {594, Q}, {609, Q}, {625, Q}, {641, Q}, {656, Q}, {672, Q}, {688, Q}, {703, Q}, {719, Q}, {734, Q}, {750, Q}, {766, Q}, {781, Q}, {797, Q}, {812, Q}, {828, Q}, {844, Q}, {859, Q}, {875, Q}, {891, Q}, {906, Q}, {922, Q}, {938, Q}, {953, Q}, {969, Q}, {984, Q}, {1000, Q}, {1016, Q}, {1031, Q}, {1047, Q}, {1062, Q}, {1078, Q}, {1094, Q}, {1109, Q}, {1125, Q}, {1141, Q}, {1156, Q}, {1172, Q}, {1188, Q}, {1203, Q}, {1219, Q}, {1234, Q}, {1250, Q}, {1266, Q}, {1281, Q}, {1297, Q}, {1312, Q}, {1328, Q}, {1344, Q}, {1359, Q}, {1375, Q}, {1391, Q}, {1406, Q}, {1422, Q}, {1438, Q}, {1453, Q}, {1469, Q}, {1484, Q}, {1500, Q}, {1516, Q}, {1531, Q}, {1547, Q}, {1562, Q}, {1578, Q}, {1594, Q}, {1609, Q}, {1625, Q}, {1641, Q}, {1656, Q}, {1672, Q}, {1688, Q}, {1703, Q}, {1719, Q}, {1734, Q}, {1750, Q}, {1766, Q}, {1781, Q}, {1797, Q}, {1812, Q}, {1828, Q}, {1844, Q}, {1859, Q}, {1875, Q}, {1891, Q}, {1906, Q}, {1922, Q}, {1938, Q}, {1953, Q}, {1969, Q}, {1984, Q}, {2000, Q}, {2016, Q}, {2031, Q}, {2047, Q}, {2062, Q}, {2078, Q}, {2094, Q}, {2109, Q}, {2125, Q}, {2141, Q}, {2156, Q}, {2172, Q}, {2188, Q}, {2203, Q}, {2219, Q}, {2234, Q}, {2250, Q}, {2266, Q}, {2281, Q}, {2297, Q}, {2312, Q}, {2328, Q}, {2344, Q}, {2359, Q}, {2375, Q}, {2391, Q}, {2406, Q}, {2422, Q}, {2438, Q}, {2453, Q}, {2469, Q}, {2484, Q}, {2500, Q}, {2516, Q}, {2531, Q}, {2547, Q}, {2562, Q}, {2578, Q}, {2594, Q}, {2609, Q}, {2625, Q}, {2641, Q}, {2656, Q}, {2672, Q}, {2688, Q}, {2703, Q}, {2719, Q}, {2734, Q}, {2750, Q}, {2766, Q}, {2781, Q}, {2797, Q}, {2812, Q}, {2828, Q}, {2844, Q}, {2859, Q}, {2875, Q}, {2891, Q}, {2906, Q}, {2922, Q}, {2938, Q}, {2953, Q}, {2969, Q}, {2984, Q}, {3000, Q}, {3016, Q}, {3031, Q}, {3047, Q}, {3062, Q}, {3078, Q}, {3094, Q}, {3109, Q}, {3125, Q}, {3141, Q}, {3156, Q}, {3172, Q}, {3188, Q}, {3203, Q}, {3219, Q}, {3234, Q}, {3250, Q}, {3266, Q}, {3281, Q}, {3297, Q}, {3312, Q}, {3328, Q}, {3344, Q}, {3359, Q}, {3375, Q}, {3391, Q}, {3406, Q}, {3422, Q}, {3438, Q}, {3453, Q}, {3469, Q}, {3484, Q}, {3500, Q}, {3516, Q}, {3531, Q}, {3547, Q}, {3562, Q}, {3578, Q}, {3594, Q}, {3609, Q}, {3625, Q}, {3641, Q}, {3656, Q}, {3672, Q}, {3688, Q}, {3703, Q}, {3719, Q}, {3734, Q}, {3750, Q}, {3766, Q}, {3781, Q}, {3797, Q}, {3812, Q}, {3828, Q}, {3844, Q}, {3859, Q}, {3875, Q}, {3891, Q}, {3906, Q}, {3922, Q}, {3938, Q}, {3953, Q}, {3969, Q}, {3984, Q}, {4000, Q}, {4016, Q}, {4031, Q}, {4047, Q}, {4062, Q}, {4078, Q}, {4094, Q}, {4109, Q}, {4125, Q}, {4141, Q}, {4156, Q}, {4172, Q}, {4188, Q}, {4203, Q}, {4219, Q}, {4234, Q}, {4250, Q}, {4266, Q}, {4281, Q}, {4297, Q}, {4312, Q}, {4328, Q}, {4344, Q}, {4359, Q}, {4375, Q}, {4391, Q}, {4406, Q}, {4422, Q}, {4438, Q}, {4453, Q}, {4469, Q}, {4484, Q}, {4500, Q}, {4516, Q}, {4531, Q}, {4547, Q}, {4562, Q}, {4578, Q}, {4594, Q}, {4609, Q}, {4625, Q}, {4641, Q}, {4656, Q}, {4672, Q}, {4688, Q}, {4703, Q}, {4719, Q}, {4734, Q}, {4750, Q}, {4766, Q}, {4781, Q}, {4797, Q}, {4812, Q}, {4828, Q}, {4844, Q}, {4859, Q}, {4875, Q}, {4891, Q}, {4906, Q}, {4922, Q}, {4938, Q}, {4953, Q}, {4969, Q}, {4984, Q}, {5000, Q}, {5016, Q}, {5031, Q}, {5047, Q}, {5062, Q}, {5078, Q}, {5094, Q}, {5109, Q}, {5125, Q}, {5141, Q}, {5156, Q}, {5172, Q}, {5188, Q}, {5203, Q}, {5219, Q}, {5234, Q}, {5250, Q}, {5266, Q}, {5281, Q}, {5297, Q}, {5312, Q}, {5328, Q}, {5344, Q}, {5359, Q}, {5375, Q}, {5391, Q}, {5406, Q}, {5422, Q}, {5438, Q}, {5453, Q}, {5469, Q}, {5484, Q}, {5500, Q}, {5516, Q}, {5531, Q}, {5547, Q}, {5562, Q}, {5578, Q}, {5594, Q}, {5609, Q}, {5625, Q}, {5641, Q}, {5656, Q}, {5672, Q}, {5688, Q}, {5703, Q}, {5719, Q}, {5734, Q}, {5750, Q}, {5766, Q}, {5781, Q}, {5797, Q}, {5812, Q}, {5828, Q}, {5844, Q}, {5859, Q}, {5875, Q}, {5891, Q}, {5906, Q}, {5922, Q}, {5938, Q}, {5953, Q}, {5969, Q}, {5984, Q}, {6000, Q}, {6016, Q}, {6031, Q}, {6047, Q}, {6062, Q}, {6078, Q}, {6094, Q}, {6109, Q}, {6125, Q}, {6141, Q}, {6156, Q}, {6172, Q}, {6188, Q}, {6203, Q}, {6219, Q}, {6234, Q}, {6250, Q}, {6266, Q}, {6281, Q}, {6297, Q}, {6312, Q}, {6328, Q}, {6344, Q}, {6359, Q}, {6375, Q}, {6391, Q}, {6406, Q}, {6422, Q}, {6438, Q}, {6453, Q}, {6469, Q}, {6484, Q}, {6500, Q}, {6516, Q}, {6531, Q}, {6547, Q}, {6562, Q}, {6578, Q}, {6594, Q}, {6609, Q}, {6625, Q}, {6641, Q}, {6656, Q}, {6672, Q}, {6688, Q}, {6703, Q}, {6719, Q}, {6734, Q}, {6750, Q}, {6766, Q}, {6781, Q}, {6797, Q}, {6812, Q}, {6828, Q}, {6844, Q}, {6859, Q}, {6875, Q}, {6891, Q}, {6906, Q}, {6922, Q}, {6938, Q}, {6953, Q}, {6969, Q}, {6984, Q}, {7000, Q}, {7016, Q}, {7031, Q}, {7047, Q}, {7062, Q}, {7078, Q}, {7094, Q}, {7109, Q}, {7125, Q}, {7141, Q}, {7156, Q}, {7172, Q}, {7188, Q}, {7203, Q}, {7219, Q}, {7234, Q}, {7250, Q}, {7266, Q}, {7281, Q}, {7297, Q}, {7312, Q}, {7328, Q}, {7344, Q}, {7359, Q}, {7375, Q}, {7391, Q}, {7406, Q}, {7422, Q}, {7438, Q}, {7453, Q}, {7469, Q}, {7484, Q}, {7500, Q}, {7516, Q}, {7531, Q}, {7547, Q}, {7562, Q}, {7578, Q}, {7594, Q}, {7609, Q}, {7625, Q}, {7641, Q}, {7656, Q}, {7672, Q}, {7688, Q}, {7703, Q}, {7719, Q}, {7734, Q}, {7750, Q}, {7766, Q}, {7781, Q}, {7797, Q}, {7812, Q}, {7828, Q}, {7844, Q}, {7859, Q}, {7875, Q}, {7891, Q}, {7906, Q}, {7922, Q}, {7938, Q}, {7953, Q}, {7969, Q}, {7984, Q}, {8000, Q}, {8016, Q}, {8031, Q}, {8047, Q}, {8062, Q}, {8078, Q}, {8094, Q}, {8109, Q}, {8125, Q}, {8141, Q}, {8156, Q}, {8172, Q}, {8188, Q}, {8203, Q}, {8219, Q}, {8234, Q}, {8250, Q}, {8266, Q}, {8281, Q}, {8297, Q}, {8312, Q}, {8328, Q}, {8344, Q}, {8359, Q}, {8375, Q}, {8391, Q}, {8406, Q}, {8422, Q}, {8438, Q}, {8453, Q}, {8469, Q}, {8484, Q}, {8500, Q}, {8516, Q}, {8531, Q}, {8547, Q}, {8562, Q}, {8578, Q}, {8594, Q}, {8609, Q}, {8625, Q}, {8641, Q}, {8656, Q}, {8672, Q}, {8688, Q}, {8703, Q}, {8719, Q}, {8734, Q}, {8750, Q}, {8766, Q}, {8781, Q}, {8797, Q}, {8812, Q}, {8828, Q}, {8844, Q}, {8859, Q}, {8875, Q}, {8891, Q}, {8906, Q}, {8922, Q}, {8938, Q}, {8953, Q}, {8969, Q}, {8984, Q}, {9000, Q}, {9016, Q}, {9031, Q}, {9047, Q}, {9062, Q}, {9078, Q}, {9094, Q}, {9109, Q}, {9125, Q}, {9141, Q}, {9156, Q}, {9172, Q}, {9188, Q}, {9203, Q}, {9219, Q}, {9234, Q}, {9250, Q}, {9266, Q}, {9281, Q}, {9297, Q}, {9312, Q}, {9328, Q}, {9344, Q}, {9359, Q}, {9375, Q}, {9391, Q}, {9406, Q}, {9422, Q}, {9438, Q}, {9453, Q}, {9469, Q}, {9484, Q}, {9500, Q}, {9516, Q}, {9531, Q}, {9547, Q}, {9562, Q}, {9578, Q}, {9594, Q}, {9609, Q}, {9625, Q}, {9641, Q}, {9656, Q}, {9672, Q}, {9688, Q}, {9703, Q}, {9719, Q}, {9734, Q}, {9750, Q}, {9766, Q}, {9781, Q}, {9797, Q}, {9812, Q}, {9828, Q}, {9844, Q}, {9859, Q}, {9875, Q}, {9891, Q}, {9906, Q}, {9922, Q}, {9938, Q}, {9953, Q}, {9969, Q}, {9984, Q}, {10000, Q}, {10016, Q}, {10031, Q}, {10047, Q}, {10062, Q}, {10078, Q}, {10094, Q}, {10109, Q}, {10125, Q}, {10141, Q}, {10156, Q}, {10172, Q}, {10188, Q}, {10203, Q}, {10219, Q}, {10234, Q}, {10250, Q}, {10266, Q}, {10281, Q}, {10297, Q}, {10312, Q}, {10328, Q}, {10344, Q}, {10359, Q}, {10375, Q}, {10391, Q}, {10406, Q}, {10422, Q}, {10438, Q}, {10453, Q}, {10469, Q}, {10484, Q}, {10500, Q}, {10516, Q}, {10531, Q}, {10547, Q}, {10562, Q}, {10578, Q}, {10594, Q}, {10609, Q}, {10625, Q}, {10641, Q}, {10656, Q}, {10672, Q}, {10688, Q}, {10703, Q}, {10719, Q}, {10734, Q}, {10750, Q}, {10766, Q}, {10781, Q}, {10797, Q}, {10812, Q}, {10828, Q}, {10844, Q}, {10859, Q}, {10875, Q}, {10891, Q}, {10906, Q}, {10922, Q}, {10938, Q}, {10953, Q}, {10969, Q}, {10984, Q}, {11000, Q}, {11016, Q}, {11031, Q}, {11047, Q}, {11062, Q}, {11078, Q}, {11094, Q}, {11109, Q}, {11125, Q}, {11141, Q}, {11156, Q}, {11172, Q}, {11188, Q}, {11203, Q}, {11219, Q}, {11234, Q}, {11250, Q}, {11266, Q}, {11281, Q}, {11297, Q}, {11312, Q}, {11328, Q}, {11344, Q}, {11359, Q}, {11375, Q}, {11391, Q}, {11406, Q}, {11422, Q}, {11438, Q}, {11453, Q}, {11469, Q}, {11484, Q}, {11500, Q}, {11516, Q}, {11531, Q}, {11547, Q}, {11562, Q}, {11578, Q}, {11594, Q}, {11609, Q}, {11625, Q}, {11641, Q}, {11656, Q}, {11672, Q}, {11688, Q}, {11703, Q}, {11719, Q}, {11734, Q}, {11750, Q}, {11766, Q}, {11781, Q}, {11797, Q}, {11812, Q}, {11828, Q}, {11844, Q}, {11859, Q}, {11875, Q}, {11891, Q}, {11906, Q}, {11922, Q}, {11938, Q}, {11953, Q}, {11969, Q}, {11984, Q}, {12000, Q}, {12016, Q}, {12031, Q}, {12047, Q}, {12062, Q}, {12078, Q}, {12094, Q}, {12109, Q}, {12125, Q}, {12141, Q}, {12156, Q}, {12172, Q}, {12188, Q}, {12203, Q}, {12219, Q}, {12234, Q}, {12250, Q}, {12266, Q}, {12281, Q}, {12297, Q}, {12312, Q}, {12328, Q}, {12344, Q}, {12359, Q}, {12375, Q}, {12391, Q}, {12406, Q}, {12422, Q}, {12438, Q}, {12453, Q}, {12469, Q}, {12484, Q}, {12500, Q}, {12516, Q}, {12531, Q}, {12547, Q}, {12562, Q}, {12578, Q}, {12594, Q}, {12609, Q}, {12625, Q}, {12641, Q}, {12656, Q}, {12672, Q}, {12688, Q}, {12703, Q}, {12719, Q}, {12734, Q}, {12750, Q}, {12766, Q}, {12781, Q}, {12797, Q}, {12812, Q}, {12828, Q}, {12844, Q}, {12859, Q}, {12875, Q}, {12891, Q}, {12906, Q}, {12922, Q}, {12938, Q}, {12953, Q}, {12969, Q}, {12984, Q}, {13000, Q}, {13016, Q}, {13031, Q}, {13047, Q}, {13062, Q}, {13078, Q}, {13094, Q}, {13109, Q}, {13125, Q}, {13141, Q}, {13156, Q}, {13172, Q}, {13188, Q}, {13203, Q}, {13219, Q}, {13234, Q}, {13250, Q}, {13266, Q}, {13281, Q}, {13297, Q}, {13312, Q}, {13328, Q}, {13344, Q}, {13359, Q}, {13375, Q}, {13391, Q}, {13406, Q}, {13422, Q}, {13438, Q}, {13453, Q}, {13469, Q}, {13484, Q}, {13500, Q}, {13516, Q}, {13531, Q}, {13547, Q}, {13562, Q}, {13578, Q}, {13594, Q}, {13609, Q}, {13625, Q}, {13641, Q}, {13656, Q}, {13672, Q}, {13688, Q}, {13703, Q}, {13719, Q}, {13734, Q}, {13750, Q}, {13766, Q}, {13781, Q}, {13797, Q}, {13812, Q}, {13828, Q}, {13844, Q}, {13859, Q}, {13875, Q}, {13891, Q}, {13906, Q}, {13922, Q}, {13938, Q}, {13953, Q}, {13969, Q}, {13984, Q}, {14000, Q}, {14016, Q}, {14031, Q}, {14047, Q}, {14062, Q}, {14078, Q}, {14094, Q}, {14109, Q}, {14125, Q}, {14141, Q}, {14156, Q}, {14172, Q}, {14188, Q}, {14203, Q}, {14219, Q}, {14234, Q}, {14250, Q}, {14266, Q}, {14281, Q}, {14297, Q}, {14312, Q}, {14328, Q}, {14344, Q}, {14359, Q}, {14375, Q}, {14391, Q}, {14406, Q}, {14422, Q}, {14438, Q}, {14453, Q}, {14469, Q}, {14484, Q}, {14500, Q}, {14516, Q}, {14531, Q}, {14547, Q}, {14562, Q}, {14578, Q}, {14594, Q}, {14609, Q}, {14625, Q}, {14641, Q}, {14656, Q}, {14672, Q}, {14688, Q}, {14703, Q}, {14719, Q}, {14734, Q}, {14750, Q}, {14766, Q}, {14781, Q}, {14797, Q}, {14812, Q}, {14828, Q}, {14844, Q}, {14859, Q}, {14875, Q}, {14891, Q}, {14906, Q}, {14922, Q}, {14938, Q}, {14953, Q}, {14969, Q}, {14984, Q}, {15000, Q}, {15016, Q}, {15031, Q}, {15047, Q}, {15062, Q}, {15078, Q}, {15094, Q}, {15109, Q}, {15125, Q}, {15141, Q}, {15156, Q}, {15172, Q}, {15188, Q}, {15203, Q}, {15219, Q}, {15234, Q}, {15250, Q}, {15266, Q}, {15281, Q}, {15297, Q}, {15312, Q}, {15328, Q}, {15344, Q}, {15359, Q}, {15375, Q}, {15391, Q}, {15406, Q}, {15422, Q}, {15438, Q}, {15453, Q}, {15469, Q}, {15484, Q}, {15500, Q}, {15516, Q}, {15531, Q}, {15547, Q}, {15562, Q}, {15578, Q}, {15594, Q}, {15609, Q}, {15625, Q}, {15641, Q}, {15656, Q}, {15672, Q}, {15688, Q}, {15703, Q}, {15719, Q}, {15734, Q}, {15750, Q}, {15766, Q}, {15781, Q}, {15797, Q}, {15812, Q}, {15828, Q}, {15844, Q}, {15859, Q}, {15875, Q}, {15891, Q}, {15906, Q}, {15922, Q}, {15938, Q}, {15953, Q}, {15969, Q}, {15984, Q}, {16000, Q}, STOP}};
static Melody sweep_hard = {.bpm = 120, .delay = 1, .notes = {{1172, W}, {1234, Q}, {1391, Q}, {3016, Q}, {3125, Q}, {3531, Q}, {3688, Q}, {4156, Q}, {4266, Q}, {5125, Q}, {5219, Q}, {6438, Q}, {6688, Q}, {8000, Q}, {8422, Q}, {10938, Q}, {13141, Q}, {14922, Q}, STOP}};
static Melody sweep_buzzer = {.bpm = 60, .delay = 1, .notes = {{1000, W}, {1016, Q}, {1031, Q}, {1047, Q}, {1062, Q}, {1078, Q}, {1094, Q}, {1109, Q}, {1125, Q}, {1141, Q}, {1156, Q}, {1172, Q}, {1188, Q}, {1203, Q}, {1219, Q}, {1234, Q}, {1250, Q}, {1266, Q}, {1281, Q}, {1297, Q}, {1312, Q}, {1328, Q}, {1344, Q}, {1359, Q}, {1375, Q}, {1391, Q}, {1406, Q}, {1422, Q}, {1438, Q}, {1453, Q}, {1469, Q}, {1484, Q}, {1500, Q}, {1516, Q}, {1531, Q}, {1547, Q}, {1562, Q}, {1578, Q}, {1594, Q}, {1609, Q}, {1625, Q}, {1641, Q}, {1656, Q}, {1672, Q}, {1688, Q}, {1703, Q}, {1719, Q}, {1734, Q}, {1750, Q}, {1766, Q}, {1781, Q}, {1797, Q}, {1812, Q}, {1828, Q}, {1844, Q}, {1859, Q}, {1875, Q}, {1891, Q}, {1906, Q}, {1922, Q}, {1938, Q}, {1953, Q}, {1969, Q}, {1984, Q}, {2000, Q}, {2016, Q}, {2031, Q}, {2047, Q}, {2062, Q}, {2078, Q}, {2094, Q}, {2109, Q}, {2125, Q}, {2141, Q}, {2156, Q}, {2172, Q}, {2188, Q}, {2203, Q}, {2219, Q}, {2234, Q}, {2250, Q}, {2266, Q}, {2281, Q}, {2297, Q}, {2312, Q}, {2328, Q}, {2344, Q}, {2359, Q}, {2375, Q}, {2391, Q}, {2406, Q}, {2422, Q}, {2438, Q}, {2453, Q}, {2469, Q}, {2484, Q}, {2500, Q}, {2516, Q}, {2516, Q}, {2562, Q}, {2625, Q}, {2688, Q}, {2781, Q}, {2891, Q}, {3016, Q}, {3125, Q}, {3188, Q}, {3266, Q}, {3375, Q}, {3469, Q}, {3531, Q}, {3578, Q}, {3625, Q}, {3688, Q}, {3719, Q}, {3766, Q}, {3812, Q}, {3859, Q}, {3922, Q}, {3969, Q}, {4000, Q}, {4047, Q}, {4109, Q}, {4156, Q}, {4219, Q}, {4266, Q}, {4328, Q}, {4375, Q}, {4438, Q}, {4500, Q}, {4562, Q}, {4625, Q}, {4688, Q}, {4766, Q}, {4828, Q}, {4906, Q}, {4969, Q}, {5062, Q}, {5125, Q}, {5219, Q}, {5297, Q}, {5391, Q}, {5469, Q}, {5578, Q}, {5656, Q}, {5766, Q}, {5859, Q}, {5969, Q}, {6078, Q}, {6203, Q}, {6312, Q}, {6438, Q}, {6562, Q}, {6703, Q}, {6844, Q}, {7000, Q}, {7156, Q}, {7297, Q}, {7469, Q}, {7641, Q}, {7812, Q}, {8016, Q}, {8203, Q}, {8422, Q}, {8641, Q}, {8875, Q}, {9125, Q}, {9391, Q}, {9672, Q}, {9953, Q}, {10266, Q}, {10594, Q}, {10938, Q}, {11328, Q}, {11734, Q}, {12172, Q}, {12641, Q}, {13141, Q}, {13688, Q}, {14281, Q}, {14922, Q}, {15641, Q}, STOP}};
static Melody sweep_slow = {.bpm = 24, .delay = 1, .notes = {{1750, Q}, {2375, Q}, {3125, Q}, {3875, Q}, REPEAT}};
static Melody sweep_fast = {.bpm = 120, .delay = 1, .notes = {{1750, Q}, {2375, Q}, {3125, Q}, {3875, Q}, REPEAT}};


typedef void (*BuzzerEffect)(uint32_t timer, uint32_t * mi, Melody * melody);

static void off(uint32_t counter, uint32_t * mi, Melody * m) {
  buzzerOff();
}

static void turnCurrentEffectOff() {
  if (sys_effect != 0) {
    sys_effect = 0;
  } else {
    user_effect = 0;
  }
}

static uint32_t mcounter = 0;
static void melodyplayer(uint32_t counter, uint32_t * mi, Melody * m) {
  uint16_t tone = m->notes[(*mi)].tone;
  uint16_t duration = m->notes[(*mi)].duration;

  if (mcounter == 0) {
    if (tone == 0xFE) {
      // Turn off buzzer since we're at the end
      (*mi) = 0;
      turnCurrentEffectOff();
    } else if (tone == 0xFF) {
      // Loop the melody
      (*mi) = 0;
    } else {
      // Play current note
      static_freq = tone;
      buzzerOn(tone);
      mcounter = (100 * 4 * 60) / (m->bpm * duration) - 1;
      (*mi)++;
    }
  } else {
    if (mcounter == 1) {
        buzzerOff();
    }
    mcounter--;
  }
}

static uint8_t static_ratio = 0;
static void bypass(uint32_t counter, uint32_t * mi, Melody * melody)
{
  buzzerOn(static_freq);
}

static uint16_t siren_start = 2000;
static uint16_t siren_freq = 2000;
static uint16_t siren_stop = 4000;
static int16_t siren_step = 40;
static void siren(uint32_t counter, uint32_t * mi, Melody * melody)
{
  siren_freq += siren_step;
  if (siren_freq > siren_stop) {
    siren_step *= -1;
    siren_freq = siren_stop;
  }
  if (siren_freq < siren_start) {
    siren_step *= -1;
    siren_freq = siren_start;
  }
  buzzerOn(siren_freq);
}

static int pitchid;
static int rollid;
static int pitch;
static int roll;
static int tilt_freq;
static int tilt_ratio;
static void tilt(uint32_t counter, uint32_t * mi, Melody * melody)
{
  pitchid = logGetVarId("stabilizer", "pitch");
  rollid = logGetVarId("stabilizer", "roll");

  pitch = logGetInt(pitchid);
  roll = logGetInt(rollid);
  tilt_freq = 0;
  tilt_ratio = 127;

  if (abs(pitch) > 5) {
    tilt_freq = 3000 - 50 * pitch;
  }

  buzzerOn(tilt_freq);
}

typedef struct {
  BuzzerEffect call;
  uint32_t mi;
  Melody * melody;
} EffectCall;

static EffectCall effects[] = {
    [SND_OFF] = {.call = &off},
    [FACTORY_TEST] = {.call = &melodyplayer, .melody = &factory_test},
    [SND_USB_CONN] = {.call = &melodyplayer, .melody = &usb_connect},
    [SND_USB_DISC] = {.call = &melodyplayer, .melody = &usb_disconnect},
    [SND_BAT_FULL] = {.call = &melodyplayer, .melody = &chg_done},
    [SND_BAT_LOW] = {.call = &melodyplayer, .melody = &lowbatt},
    [SND_STARTUP] = {.call = &melodyplayer, .melody = &startup},
    [SND_CALIB] = {.call = &melodyplayer, .melody = &calibrated},
    {.call = &melodyplayer, .melody = &range_slow},
    {.call = &melodyplayer, .melody = &range_fast},
    {.call = &melodyplayer, .melody = &starwars},
    {.call = &melodyplayer, .melody = &valkyries},
    {.call = &bypass}, //12
    {.call = &siren}, //13
    {.call = &tilt}, //14
    [15] = {.call = &melodyplayer, .melody = &sweep},
    [16] = {.call = &melodyplayer, .melody = &sweep_high},
    [18] = {.call = &melodyplayer, .melody = &sweep_all},
	[19] = {.call = &melodyplayer, .melody = &sweep_hard},
	[20] = {.call = &melodyplayer, .melody = &sweep_buzzer},
	[21] = {.call = &melodyplayer, .melody = &sweep_slow},
	[22] = {.call = &melodyplayer, .melody = &sweep_fast}

};

static xTimerHandle timer;
static StaticTimer_t timerBuffer;
static uint32_t counter = 0;

static void soundTimer(xTimerHandle timer)
{
  int effect;
  counter++;

  if (sys_effect != 0) {
    effect = sys_effect;
  } else {
    effect = user_effect;
  }

  if (effects[effect].call != 0) {
    effects[effect].call(counter * 10, &effects[effect].mi, effects[effect].melody);
  }
}

void soundInit(void)
{
  if (isInit) {
    return;
  }

  neffect = sizeof(effects) / sizeof(effects[0]) - 1;

  timer = xTimerCreateStatic("SoundTimer", M2T(10), pdTRUE, NULL, soundTimer, &timerBuffer);
  xTimerStart(timer, 100);

  isInit = true;
}

bool soundTest(void)
{
  return isInit;
}

void soundSetEffect(uint32_t effect)
{
  sys_effect = effect;
}

uint16_t soundGetFreq(void) {
	return (uint16_t) static_freq;
}

PARAM_GROUP_START(sound)
PARAM_ADD(PARAM_UINT8, effect, &user_effect)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, neffect, &neffect)
PARAM_ADD(PARAM_UINT16, freq, &static_freq)
PARAM_ADD(PARAM_UINT8, ratio, &static_ratio)
PARAM_GROUP_STOP(sound)
