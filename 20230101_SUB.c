#include "D_Main.h"
#include "D_I2C.h"
#include "D_SIO.h"
#include "D_EIO.h"

/*
[0]CN1  : �O
[1]CN2  : ��
[2]CN3  : ���
[3]CN4  : �E
[4]CN5  : �Ȃ�
[5]CN6  : �Ȃ�
[6]CN7  : �Ȃ�
[7]CN8  : �Ȃ�
[8]CN9  : �Ȃ�
[9]CN10 : �Ȃ�
*/

//���[�v�p�ϐ�
int l;

//���C���Z���T�̔����������l
int LINE_LOW;

//���C���֐�
void user_main(void) {
    //�Z���T�l��r�Ɏg�p����ϐ��̐錾
    /*
    �Z���T�l���ł��傫���Z���T�̃C���f�b�N�X
    �Z���T�l���ł��傫���Z���T�̃Z���T�l
    �Z���T�l����Ԗڂɑ傫���Z���T�̃C���f�b�N�X
    �Z���T�l����Ԗڂɑ傫���Z���T�̃Z���T�l
    �������l���Ⴂ�Z���T�l���o�͂����Z���T�̌�
    */
	int highIndex;
	int highValue;
	int highIndex2;
	int highValue2;
	int lowQuantity;

    //��r���̃Z���T�l
	int comparingValue;
	
	//�T�uCPU�̏�ԁi�N����ԁj���O���[�o�������ꂽ�t���O�ϐ��ɋL�^
	gV[VAR_Z] = 1;
        
	//���������[�v
    while (TRUE) {
        //�������l���O���[�o���ϐ�����ǂݍ���
        LINE_LOW = gV[VAR_S];

        //------------------------------------------------------------
        //�ł��傫���Z���T�l���擾����

        //�������l���Ⴂ�Z���T�l���o�͂����Z���T�̌���������
        lowQuantity = 0;

        //�Z���T�l���ł��傫���Z���T�̃C���f�b�N�X��������
        highIndex = 0;
        //���������ꂽ�C���f�b�N�X�ɑΉ������Z���T�l�����ōł��傫���Z���T�l�Ƃ���
        highValue = gAD[highIndex];

        //�������Ŏ擾�����ł��傫���Z���T�l���������l���Ⴂ�Ȃ�
        if (highValue < LINE_LOW) {
            //�������l���Ⴂ�Z���T�l���o�͂����Z���T�̌��ɉ��Z
            lowQuantity += 1;
        }

        //CN2����CN4�܂ł�3��J��Ԃ�
        for (l = 0; l < 3; l++) {

            //��r����Z���T�l���擾
            comparingValue = gAD[l + 1];

            //������r���̃Z���T�l���������l���Ⴂ�Ȃ�
            if (comparingValue < LINE_LOW) {
                lowQuantity += 1;
            }

            //������r���̃Z���T�l���ł��傫���Z���T�l�����傫���Ȃ�
            if (comparingValue > highValue) {
                highIndex = l + 1;
                highValue = comparingValue;
            }
        }

        //���C��CPU����ϐ��̓ǂݎ�肪�ł���悤�ɃO���[�o���ϐ��ɂ��ꂼ��̕ϐ�����
        gV[VAR_A] = highIndex;
        gV[VAR_B] = highValue;
        gV[VAR_C] = lowQuantity;

        //------------------------------------------------------------

        //------------------------------------------------------------
        //2�Ԗڂɑ傫���Z���T�l���擾����

        //�Z���T�l��2�Ԗڂɑ傫���Z���T�̃C���f�b�N�X��������
        switch (highIndex) {
        case 0:
            highIndex2 = 1;
            break;

        default:
            highIndex2 = 0;
            break;
        }

        //���������ꂽ�C���f�b�N�X�ɑΉ������Z���T�l������2�Ԗڂɑ傫���Z���T�l�Ƃ���
        highValue2 = gAD[highIndex2];

        //CN1����CN4�܂ł�4��J��Ԃ�
        for (l = 0; l < 4; l++) {

            //������r����C���f�b�N�X���Z���T�l�����Y���傫���Z���T�̃C���f�b�N�X�Ɠ���Ȃ�
            if (l == highIndex) {
                //�X�L�b�v����
                continue;
            }

            //��r����Z���T�l���擾
            comparingValue = gAD[l];

            //������r���̃Z���T�l��2�Ԗڂɑ傫���Z���T�l�����傫���Ȃ�
            if (comparingValue > highValue2) {
                highIndex2 = l;
                highValue2 = comparingValue;
            }
        }

        //���C��CPU����ϐ��̓ǂݎ�肪�ł���悤�ɃO���[�o���ϐ��ɂ��ꂼ��̕ϐ�����
        gV[VAR_D] = highIndex2;
        gV[VAR_E] = highValue2;

        //------------------------------------------------------------
    }
}