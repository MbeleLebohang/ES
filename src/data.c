/*
 * data.c
 *
 *  Created on: 6 Nov 2017
 *      Author: MBELE
 */
#include "stm32f4xx.h"

							/*LOWEST NOTE*/
uint16_t LOWER_NOTES_ARR[] = {6108, 5765, 5442, 5136, 4848, 4576, 4319,
							4076, 3847, 3631, 3428, 3235, 3054, 2882,
							2720, 2568, 2423, 2287, 2159, 2038, 1923,
							1815, 1713, 1617, 1526, 1441, 1360, 1283,
							1211, 1143, 1079, 1018, 961, 907, 856,
							808, 763, 720, 679, 641, 605, 571,
							539, 509, 480, 453, 428, 404, 381} ;

uint16_t UPPER_NOTES_ARR[] = {359, 339, 320, 302, 285, 269, 254,
							240, 226, 213, 201, 190, 179, 169,
							160, 151, 142, 134, 126, 119, 113,
							106, 100, 94, 89, 84, 79, 75,
							71, 66, 63, 59, 56, 53, 50,
							47, 44, 42, 39, 37, 35, 33,
							31, 29, 27, 26, 24, 23, 22};
												/* HIGHEST NOTE*/

uint16_t SINE_Wave[] = {2048, 2099, 2151, 2202, 2254, 2305, 2356, 2406, 2457, 2507, 2557, 2607, 2656, 2705, 2754,
						2802, 2850, 2897, 2943, 2989, 3035, 3079, 3124, 3167, 3210, 3252, 3293, 3334, 3373, 3412,
						3450, 3487, 3523, 3558, 3593, 3626, 3658, 3690, 3720, 3749, 3777, 3804, 3830, 3855, 3879,
						3901, 3922, 3943, 3962, 3979, 3996, 4011, 4025, 4038, 4049, 4060, 4069, 4076, 4083, 4088,
						4092, 4095, 4096, 4096, 4095, 4092, 4088, 4083, 4076, 4069, 4060, 4049, 4038, 4025, 4011,
						3996, 3979, 3962, 3943, 3922, 3901, 3879, 3855, 3830, 3804, 3777, 3749, 3720, 3690, 3658,
						3626, 3593, 3558, 3523, 3487, 3450, 3412, 3373, 3334, 3293, 3252, 3210, 3167, 3124, 3079,
						3035, 2989, 2943, 2897, 2850, 2802, 2754, 2705, 2656, 2607, 2557, 2507, 2457, 2406, 2356,
						2305, 2254, 2202, 2151, 2099, 2048, 1997, 1945, 1894, 1842, 1791, 1740, 1690, 1639, 1589,
						1539, 1489, 1440, 1391, 1342, 1294, 1246, 1199, 1153, 1107, 1061, 1017, 972, 929, 886,
						844, 803, 762, 723, 684, 646, 609, 573, 538, 503, 470, 438, 406, 376, 347, 319, 292,
						266, 241, 217, 195, 174, 153, 134, 117, 100, 85, 71, 58, 47, 36, 27, 20, 13, 8, 4,
						1, 0, 0, 1, 4, 8, 13, 20, 27, 36, 47, 58, 71, 85, 100, 117, 134, 153, 174, 195, 217,
						241, 266, 292, 319, 347, 376, 406, 438, 470, 503, 538, 573, 609, 646, 684, 723, 762, 803,
						844, 886, 929, 972, 1017, 1061, 1107, 1153, 1199, 1246, 1294, 1342, 1391, 1440, 1489, 1539,
						1589, 1639, 1690, 1740, 1791, 1842, 1894, 1945, 1997, 2048};


uint16_t SQUARE_Wave[]={4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096, 4096,
						4096, 4096, 4096, 4096, 4096, 4096, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
						0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint16_t SAWTOOTH_Wave[] = {0, 16, 33, 49, 66, 82, 98, 115, 131, 147, 164, 180, 197, 213, 229, 246, 262, 279, 295,
							311, 328, 344, 360, 377, 393, 410, 426, 442, 459, 475, 492, 508, 524, 541, 557, 573,
							590, 606, 623, 639, 655, 672, 688, 705, 721, 737, 754, 770, 786, 803, 819, 836, 852,
							868, 885, 901, 918, 934, 950, 967, 983, 999, 1016, 1032, 1049, 1065, 1081, 1098, 1114,
							1130, 1147, 1163, 1180, 1196, 1212, 1229, 1245, 1262, 1278, 1294, 1311, 1327, 1343,
							1360, 1376, 1393, 1409, 1425, 1442, 1458, 1475, 1491, 1507, 1524, 1540, 1556, 1573,
							1589, 1606, 1622, 1638, 1655, 1671, 1688, 1704, 1720, 1737, 1753, 1769, 1786, 1802,
							1819, 1835, 1851, 1868, 1884, 1901, 1917, 1933, 1950, 1966, 1982, 1999, 2015, 2032,
							2048, 2064, 2081, 2097, 2114, 2130, 2146, 2163, 2179, 2195, 2212, 2228, 2245, 2261,
							2277, 2294, 2310, 2327, 2343, 2359, 2376, 2392, 2408, 2425, 2441, 2458, 2474, 2490,
							2507, 2523, 2540, 2556, 2572, 2589, 2605, 2621, 2638, 2654, 2671, 2687, 2703, 2720,
							2736, 2753, 2769, 2785, 2802, 2818, 2834, 2851, 2867, 2884, 2900, 2916, 2933, 2949,
							2966, 2982, 2998, 3015, 3031, 3047, 3064, 3080, 3097, 3113, 3129, 3146, 3162, 3178,
							3195, 3211, 3228, 3244, 3260, 3277, 3293, 3310, 3326, 3342, 3359, 3375, 3391, 3408,
							3424, 3441, 3457, 3473, 3490, 3506, 3523, 3539, 3555, 3572, 3588, 3604, 3621, 3637,
							3654, 3670, 3686, 3703, 3719, 3736, 3752, 3768, 3785, 3801, 3817, 3834, 3850, 3867,
							3883, 3899, 3916, 3932, 3949, 3965, 3981, 3998, 4014, 4030, 4047, 4063, 4080, 4096};

uint16_t TRIANGLE_Wave[] = {0, 33, 66, 98, 131, 164, 197, 229, 262, 295, 328, 360, 393, 426, 459, 492, 524, 557,
							590, 623, 655, 688, 721, 754, 786, 819, 852, 885, 918, 950, 983, 1016, 1049, 1081,
							1114, 1147, 1180, 1212, 1245, 1278, 1311, 1343, 1376, 1409, 1442, 1475, 1507, 1540,
							1573, 1606, 1638, 1671, 1704, 1737, 1769, 1802, 1835, 1868, 1901, 1933, 1966, 1999,
							2032, 2064, 2097, 2130, 2163, 2195, 2228, 2261, 2294, 2327, 2359, 2392, 2425, 2458,
							2490, 2523, 2556, 2589, 2621, 2654, 2687, 2720, 2753, 2785, 2818, 2851, 2884, 2916,
							2949, 2982, 3015, 3047, 3080, 3113, 3146, 3178, 3211, 3244, 3277, 3310, 3342, 3375,
							3408, 3441, 3473, 3506, 3539, 3572, 3604, 3637, 3670, 3703, 3736, 3768, 3801, 3834,
							3867, 3899, 3932, 3965, 3998, 4030, 4063, 4096, 4063, 4030, 3998, 3965, 3932, 3899,
							3867, 3834, 3801, 3768, 3736, 3703, 3670, 3637, 3604, 3572, 3539, 3506, 3473, 3441,
							3408, 3375, 3342, 3310, 3277, 3244, 3211, 3178, 3146, 3113, 3080, 3047, 3015, 2982,
							2949, 2916, 2884, 2851, 2818, 2785, 2753, 2720, 2687, 2654, 2621, 2589, 2556, 2523,
							2490, 2458, 2425, 2392, 2359, 2327, 2294, 2261, 2228, 2195, 2163, 2130, 2097, 2064,
							2032, 1999, 1966, 1933, 1901, 1868, 1835, 1802, 1769, 1737, 1704, 1671, 1638, 1606,
							1573, 1540, 1507, 1475, 1442, 1409, 1376, 1343, 1311, 1278, 1245, 1212, 1180, 1147,
							1114, 1081, 1049, 1016, 983, 950, 918, 885, 852, 819, 786, 754, 721, 688, 655, 623,
							590, 557, 524, 492, 459, 426, 393, 360, 328, 295, 262, 229, 197, 164, 131, 98, 66,
							33, 0};
