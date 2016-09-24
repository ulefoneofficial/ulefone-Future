/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H


/* ============================================================
// define
// ============================================================*/
#define BAT_NTC_10 1
#define BAT_NTC_47 0

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             24000
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900
#endif

#define RBAT_PULL_UP_VOLT          1800



/* ============================================================
// ENUM
// ============================================================*/

/* ============================================================
// structure
// ============================================================*/

/* ============================================================
// typedef
// ============================================================*/
typedef struct _BATTERY_PROFILE_STRUCT {
	signed int percentage;
	signed int voltage;
	} BATTERY_PROFILE_STRUCT, *BATTERY_PROFILE_STRUCT_P;

typedef struct _R_PROFILE_STRUCT {
	signed int resistance; /* Ohm*/
	signed int voltage;
} R_PROFILE_STRUCT, *R_PROFILE_STRUCT_P;

typedef enum {
	T1_0C,
	T2_25C,
	T3_50C
} PROFILE_TEMPERATURE;

/* ============================================================
// External Variables
// ============================================================*/

/* ============================================================
// External function
// ============================================================*/

/* ============================================================
// <DOD, Battery_Voltage> Table
// ============================================================*/
#if (BAT_NTC_10 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
	{-20, 68237},
	{-15, 53650},
	{-10, 42506},
	{ -5, 33892},
	{  0, 27219},
	{  5, 22021},
	{ 10, 17926},
	{ 15, 14674},
	{ 20, 12081},
	{ 25, 10000},
	{ 30, 8315},
	{ 35, 6948},
	{ 40, 5834},
	{ 45, 4917},
	{ 50, 4161},
	{ 55, 3535},
	{ 60, 3014}
};
#endif

#if (BAT_NTC_47 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
	{-20, 483954},
	{-15, 360850},
	{-10, 271697},
	{ -5, 206463},
	{  0, 158214},
	{  5, 122259},
	{ 10, 95227},
	{ 15, 74730},
	{ 20, 59065},
	{ 25, 47000},
	{ 30, 37643},
	{ 35, 30334},
	{ 40, 24591},
	{ 45, 20048},
	{ 50, 16433},
	{ 55, 13539},
	{ 60, 11210}
};
#endif

/* T0 -10C */
BATTERY_PROFILE_STRUCT battery_profile_t0[] = {
 	  {0   ,4346 },
  	{2   ,4305 },
  	{4   ,4275 },
  	{5   ,4249 },
  	{7   ,4226 },
  	{9   ,4204 },
  	{11  ,4184 },
  	{13  ,4163 },
  	{14  ,4143 },
  	{16  ,4124 },
  	{18  ,4107 },
  	{20  ,4091 },
  	{21  ,4074 },
  	{23  ,4051 },
  	{25  ,4021 },
  	{27  ,3993 },
  	{29  ,3971 },
  	{30  ,3953 },
  	{32  ,3940 },
  	{34  ,3928 },
  	{36  ,3917 },
  	{38  ,3906 },
  	{39  ,3894 },
  	{41  ,3883 },
  	{43  ,3872 },
  	{45  ,3861 },
  	{47  ,3851 },
  	{48  ,3841 },
  	{50  ,3832 },
  	{52  ,3824 },
  	{54  ,3816 },
  	{55  ,3808 },
  	{57  ,3802 },
  	{59  ,3796 },
  	{61  ,3791 },
  	{63  ,3786 },
  	{64  ,3782 },
  	{66  ,3777 },
  	{68  ,3773 },
  	{70  ,3769 },
  	{72  ,3765 },
  	{73  ,3760 },
  	{75  ,3756 },
  	{77  ,3751 },
  	{79  ,3746 },
  	{81  ,3740 },
  	{82  ,3735 },
  	{84  ,3729 },
  	{86  ,3722 },
  	{88  ,3715 },
  	{90  ,3707 },
  	{91  ,3697 },
  	{93  ,3685 },
  	{95  ,3671 },
  	{97  ,3651 },
  	{98  ,3643 },
  	{98  ,3637 },
  	{99  ,3630 },
    {100 ,3624 },
    {100 ,3400 },
	       
};

/* T1 0C */
BATTERY_PROFILE_STRUCT battery_profile_t1[] = {
 	  {0   ,4347 },
  	{2   ,4314 },
  	{3   ,4288 },
  	{5   ,4266 },
  	{7   ,4245 },
  	{9   ,4225 },
  	{10  ,4206 },
  	{12  ,4188 },
  	{14  ,4169 },
  	{15  ,4151 },
  	{17  ,4132 },
  	{19  ,4114 },
  	{21  ,4097 },
  	{22  ,4083 },
  	{24  ,4070 },
  	{26  ,4048 },
  	{27  ,4015 },
  	{29  ,3986 },
  	{31  ,3965 },
  	{33  ,3950 },
  	{34  ,3939 },
  	{36  ,3930 },
  	{38  ,3919 },
  	{39  ,3908 },
  	{41  ,3895 },
  	{43  ,3883 },
  	{45  ,3872 },
  	{46  ,3861 },
  	{48  ,3851 },
  	{50  ,3841 },
  	{51  ,3833 },
  	{53  ,3824 },
  	{55  ,3817 },
  	{57  ,3810 },
  	{58  ,3803 },
  	{60  ,3797 },
  	{62  ,3792 },
  	{63  ,3787 },
  	{65  ,3783 },
  	{67  ,3780 },
  	{69  ,3777 },
  	{70  ,3773 },
  	{72  ,3770 },
  	{74  ,3766 },
  	{75  ,3762 },
  	{77  ,3756 },
  	{79  ,3750 },
  	{80  ,3743 },
  	{82  ,3735 },
  	{84  ,3727 },
  	{86  ,3719 },
  	{87  ,3712 },
  	{89  ,3706 },
  	{91  ,3700 },
  	{92  ,3693 },
  	{94  ,3681 },
  	{96  ,3653 },
  	{98  ,3598 },
    {99  ,3501 },
    {100 ,3400 },

};

/* T2 25C*/
BATTERY_PROFILE_STRUCT battery_profile_t2[] = {
 	  {0   ,4344 }, 
  	{2   ,4315 }, 
  	{3   ,4291 }, 
	{3, 4270},
	{5, 4250},
  	{7   ,4230 }, 
  	{9   ,4210 }, 
  	{10  ,4191 }, 
  	{12  ,4173 }, 
  	{14  ,4154 }, 
  	{16  ,4136 }, 
  	{17  ,4118 }, 
  	{19  ,4099 }, 
  	{21  ,4083 }, 
  	{23  ,4072 }, 
  	{24  ,4060 }, 
  	{26  ,4032 }, 
  	{28  ,4004 }, 
  	{30  ,3985 }, 
  	{31  ,3976 }, 
  	{33  ,3969 }, 
  	{35  ,3958 }, 
  	{37  ,3945 }, 
  	{38  ,3932 }, 
  	{40  ,3918 }, 
  	{42  ,3902 }, 
  	{44  ,3886 }, 
  	{45  ,3871 }, 
  	{47  ,3857 }, 
  	{49  ,3846 }, 
  	{51  ,3836 }, 
  	{52  ,3828 }, 
  	{54  ,3820 }, 
  	{56  ,3813 }, 
  	{58  ,3806 }, 
  	{59  ,3800 }, 
  	{61  ,3795 }, 
  	{63  ,3790 }, 
  	{65  ,3786 }, 
  	{66  ,3781 }, 
  	{68  ,3777 }, 
  	{70  ,3774 }, 
  	{72  ,3772 }, 
  	{73  ,3769 }, 
  	{75  ,3765 }, 
  	{77  ,3760 }, 
  	{79  ,3753 }, 
  	{80  ,3746 }, 
  	{82  ,3739 }, 
  	{84  ,3729 }, 
  	{86  ,3717 }, 
  	{87  ,3703 }, 
  	{89  ,3691 }, 
  	{91  ,3688 }, 
  	{93  ,3686 }, 
  	{94  ,3683 }, 
  	{96  ,3670 }, 
  	{98  ,3605 }, 
    {100 ,3482 }, 
    {100 ,3400 }, 

};

/* T3 50C*/
BATTERY_PROFILE_STRUCT battery_profile_t3[] = {
 	  {0   ,4344 },
  	{2   ,4319 },
  	{4   ,4296 },
  	{5   ,4275 },
  	{7   ,4254 },
  	{9   ,4234 },
  	{11  ,4215 },
  	{13  ,4196 },
  	{14  ,4176 },
  	{16  ,4158 },
  	{18  ,4139 },
  	{20  ,4121 },
  	{21  ,4103 },
	{23, 4085},
  	{25  ,4068 },
  	{27  ,4052 },
  	{29  ,4036 },
  	{30  ,4019 },
  	{32  ,4004 },
  	{34  ,3990 },
  	{36  ,3976 },
  	{37  ,3963 },
  	{39  ,3950 },
  	{41  ,3937 },
  	{43  ,3925 },
  	{45  ,3913 },
  	{46  ,3899 },
  	{48  ,3882 },
  	{50  ,3864 },
  	{52  ,3850 },
  	{54  ,3839 },
  	{55  ,3830 },
  	{57  ,3822 },
  	{59  ,3814 },
  	{61  ,3808 },
  	{62  ,3801 },
  	{64  ,3796 },
  	{66  ,3790 },
  	{68  ,3786 },
  	{70  ,3781 },
  	{71  ,3777 },
  	{73  ,3773 },
  	{75  ,3768 },
  	{77  ,3760 },
  	{79  ,3751 },
  	{80  ,3745 },
  	{82  ,3739 },
  	{84  ,3732 },
  	{86  ,3725 },
  	{87  ,3718 },
  	{89  ,3707 },
  	{91  ,3695 },
  	{93  ,3681 },
  	{95  ,3676 },
  	{96  ,3675 },
  	{98  ,3673 },
  	{100 ,3665 },
  	{100 ,3615 },
    {100 ,3513 },
    {100 ,3400 },

};

/* battery profile for actual temperature. The size should be the same as T1, T2 and T3*/
BATTERY_PROFILE_STRUCT battery_profile_temperature[] = {
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},


};

/* ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================*/
/* T0 -10C*/
R_PROFILE_STRUCT r_profile_t0[] = {
	{755 ,4346 },
 	{755 ,4305 },
 	{741 ,4275 },
 	{724 ,4249 },
 	{709 ,4226 },
 	{693 ,4204 },
 	{681 ,4184 },
 	{669 ,4163 },
 	{659 ,4143 },
 	{649 ,4124 },
 	{642 ,4107 },
 	{638 ,4091 },
 	{635 ,4074 },
 	{626 ,4051 },
 	{612 ,4021 },
 	{601 ,3993 },
 	{594 ,3971 },
 	{591 ,3953 },
 	{591 ,3940 },
 	{591 ,3928 },
 	{590 ,3917 },
 	{589 ,3906 },
 	{587 ,3894 },
 	{584 ,3883 },
 	{583 ,3872 },
 	{580 ,3861 },
 	{579 ,3851 },
 	{579 ,3841 },
 	{577 ,3832 },
 	{578 ,3824 },
 	{578 ,3816 },
 	{578 ,3808 },
 	{581 ,3802 },
 	{583 ,3796 },
 	{586 ,3791 },
 	{590 ,3786 },
 	{594 ,3782 },
 	{597 ,3777 },
 	{604 ,3773 },
 	{611 ,3769 },
 	{619 ,3765 },
 	{628 ,3760 },
 	{639 ,3756 },
 	{650 ,3751 },
 	{664 ,3746 },
 	{678 ,3740 },
 	{695 ,3735 },
 	{713 ,3729 },
 	{733 ,3722 },
 	{755 ,3715 },
 	{776 ,3707 },
 	{803 ,3697 },
 	{832 ,3685 },
 	{861 ,3671 },
 	{893 ,3651 },
 	{893 ,3643 },
 	{887 ,3637 },
 	{881 ,3630 },
  {875 ,3624 },
  {653 ,3400 },

};

/* T1 0C*/
R_PROFILE_STRUCT r_profile_t1[] = {
  {225 ,4347 }, 
	{392 ,4314 }, 
	{390 ,4288 }, 
	{386 ,4266 }, 
	{387 ,4245 }, 
	{385 ,4225 }, 
	{381 ,4206 }, 
	{379 ,4188 }, 
	{376 ,4169 }, 
	{373 ,4151 }, 
	{371 ,4132 }, 
	{367 ,4114 }, 
	{366 ,4097 }, 
	{366 ,4083 }, 
	{368 ,4070 }, 
	{363 ,4048 }, 
	{351 ,4015 }, 
	{344 ,3986 }, 
	{343 ,3965 }, 
	{340 ,3950 }, 
	{341 ,3939 }, 
	{342 ,3930 }, 
	{340 ,3919 }, 
	{337 ,3908 }, 
	{335 ,3895 }, 
	{332 ,3883 }, 
	{330 ,3872 }, 
	{330 ,3861 }, 
	{330 ,3851 }, 
	{330 ,3841 }, 
	{331 ,3833 }, 
	{332 ,3824 }, 
	{332 ,3817 }, 
	{333 ,3810 }, 
	{335 ,3803 }, 
	{336 ,3797 }, 
	{338 ,3792 }, 
	{340 ,3787 }, 
	{342 ,3783 }, 
	{346 ,3780 }, 
	{349 ,3777 }, 
	{353 ,3773 }, 
	{357 ,3770 }, 
	{362 ,3766 }, 
	{368 ,3762 }, 
	{374 ,3756 }, 
	{381 ,3750 }, 
	{389 ,3743 }, 
	{398 ,3735 }, 
	{408 ,3727 }, 
	{420 ,3719 }, 
	{436 ,3712 }, 
	{457 ,3706 }, 
	{484 ,3700 }, 
	{521 ,3693 }, 
	{571 ,3681 }, 
	{637 ,3653 }, 
	{710 ,3598 }, 
  {790 ,3501 }, 
  {702 ,3400 }, 
};

/* T2 25C*/
R_PROFILE_STRUCT r_profile_t2[] = {
  {187 ,4344 },
	{187 ,4315 },
	{184 ,4291 },
	{182 ,4270 },
	{180 ,4250 },
	{180 ,4230 },
	{180 ,4210 },
	{170 ,4191 },
	{161 ,4173 },
	{157 ,4154 },
	{153 ,4136 },
	{162 ,4118 },
	{168 ,4099 },
	{170 ,4083 },
	{174 ,4072 },
	{170 ,4060 },
	{158 ,4032 },
	{154 ,4004 },
	{152 ,3985 },
	{153 ,3976 },
	{155 ,3969 },
	{152 ,3958 },
	{152 ,3945 },
	{151 ,3932 },
	{150 ,3918 },
	{153 ,3902 },
	{153 ,3886 },
	{152 ,3871 },
	{153 ,3857 },
	{153 ,3846 },
	{144 ,3836 },
	{139 ,3828 },
	{137 ,3820 },
	{135 ,3813 },
	{134 ,3806 },
	{133 ,3800 },
	{134 ,3795 },
	{134 ,3790 },
	{134 ,3786 },
	{142 ,3781 },
	{149 ,3777 },
	{154 ,3774 },
	{160 ,3772 },
	{165 ,3769 },
	{162 ,3765 },
	{151 ,3760 },
	{146 ,3753 },
	{144 ,3746 },
	{143 ,3739 },
	{144 ,3729 },
	{144 ,3717 },
	{144 ,3703 },
	{144 ,3691 },
	{146 ,3688 },
	{152 ,3686 },
	{160 ,3683 },
	{172 ,3670 },
	{171 ,3605 },
  {190 ,3482 },
  {325 ,3400 },

};

/* T3 50C*/
R_PROFILE_STRUCT r_profile_t3[] = {
  {119  ,4344 },
 	{119  ,4319 },
 	{118  ,4296 },
 	{118  ,4275 },
 	{117  ,4254 },
 	{117  ,4234 },
 	{117  ,4215 },
 	{117  ,4196 },
 	{117  ,4176 },
 	{118  ,4158 },
 	{118  ,4139 },
 	{118  ,4121 },
 	{119  ,4103 },
 	{119  ,4085 },
 	{119  ,4068 },
 	{120  ,4052 },
 	{120  ,4036 },
 	{121  ,4019 },
 	{122  ,4004 },
 	{122  ,3990 },
 	{123  ,3976 },
 	{124  ,3963 },
 	{125  ,3950 },
 	{127  ,3937 },
 	{128  ,3925 },
 	{129  ,3913 },
 	{128  ,3899 },
 	{124  ,3882 },
 	{119  ,3864 },
 	{117  ,3850 },
 	{116  ,3839 },
 	{116  ,3830 },
 	{116  ,3822 },
 	{116  ,3814 },
 	{116  ,3808 },
 	{116  ,3801 },
 	{117  ,3796 },
 	{118  ,3790 },
 	{119  ,3786 },
 	{119  ,3781 },
 	{120  ,3777 },
 	{121  ,3773 },
 	{121  ,3768 },
 	{118  ,3760 },
 	{116  ,3751 },
 	{117  ,3745 },
 	{117  ,3739 },
 	{117  ,3732 },
 	{118  ,3725 },
 	{119  ,3718 },
 	{119  ,3707 },
 	{119  ,3695 },
 	{117  ,3681 },
 	{118  ,3676 },
 	{122  ,3675 },
 	{128  ,3673 },
 	{137  ,3665 },
 	{128  ,3615 },
  {132  ,3513 },
  {150  ,3400 },

};

/* r-table profile for actual temperature. The size should be the same as T1, T2 and T3*/
R_PROFILE_STRUCT r_profile_temperature[] = {
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},

};

/* ============================================================
// function prototype
// ============================================================*/
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUCT_P fgauge_get_profile(unsigned int temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUCT_P fgauge_get_profile_r_table(unsigned int temperature);

#endif

