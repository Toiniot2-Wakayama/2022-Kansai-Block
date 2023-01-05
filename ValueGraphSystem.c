#include "D_Main.h"
#include "D_I2C.h"
#include "D_SIO.h"
#include "D_EIO.h"

/*
メインCPU
CN1[0]  ボール前
CN2[1]  ボール左前
CN3[2]  ボール左
CN4[3]  ボール左後ろ
CN5[4]  ボール後ろ
CN6[5]  ボール右後ろ
CN7[6]  ボール右
CN8[7]  ボール右前
CN9[8]  N.C.
CN10[9] N.C.
I2C 6chモータードライバ

サブCPU
CN1[0]  ライン前
CN2[1]  ライン左
CN3[2]  ライン後ろ
CN4[3]  ライン右
CN5[4]  N.C.
CN6[5]  Bボタン（決定）
CN7[6]  Aボタン（選択）
CN8[7]  （超音波右）
CN9[8]  （超音波後ろ）
CN10[9] （超音波左）
I2C 地磁気センサ

6chモータードライバ
gPwm[0] 右前
gPwm[1] 左後ろ
gPwm[2] N.C.
gPwm[3] 左前
gPwm[4] N.C.
gPwm[5] 右後ろ
*/

void user_main(void) {
	clr_timer(0);

	while (TRUE) {
		gV[VAR_A] = gAD[0];
		gV[VAR_B] = gAD[1];
		gV[VAR_C] = gAD[2];
		gV[VAR_D] = gAD[3];
		gV[VAR_E] = gAD[4];
		gV[VAR_F] = gAD[5];
		gV[VAR_G] = gAD[6];
		gV[VAR_H] = gAD[7];
		if (gT[0] % 125 == 0) {
			printf("%ld,", gV[VAR_A]);
			printf("%ld,", gV[VAR_B]);
			printf("%ld,", gV[VAR_C]);
			printf("%ld,", gV[VAR_D]);
			printf("%ld,", gV[VAR_E]);
			printf("%ld,", gV[VAR_F]);
			printf("%ld,", gV[VAR_G]);
			printf("%ld\r\n", gV[VAR_H]);
		}
	}
}
