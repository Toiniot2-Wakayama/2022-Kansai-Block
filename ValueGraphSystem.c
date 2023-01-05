#include "D_Main.h"
#include "D_I2C.h"
#include "D_SIO.h"
#include "D_EIO.h"

/*
���C��CPU
CN1[0]  �{�[���O
CN2[1]  �{�[�����O
CN3[2]  �{�[����
CN4[3]  �{�[�������
CN5[4]  �{�[�����
CN6[5]  �{�[���E���
CN7[6]  �{�[���E
CN8[7]  �{�[���E�O
CN9[8]  N.C.
CN10[9] N.C.
I2C 6ch���[�^�[�h���C�o

�T�uCPU
CN1[0]  ���C���O
CN2[1]  ���C����
CN3[2]  ���C�����
CN4[3]  ���C���E
CN5[4]  N.C.
CN6[5]  B�{�^���i����j
CN7[6]  A�{�^���i�I���j
CN8[7]  �i�����g�E�j
CN9[8]  �i�����g���j
CN10[9] �i�����g���j
I2C �n���C�Z���T

6ch���[�^�[�h���C�o
gPwm[0] �E�O
gPwm[1] �����
gPwm[2] N.C.
gPwm[3] ���O
gPwm[4] N.C.
gPwm[5] �E���
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
