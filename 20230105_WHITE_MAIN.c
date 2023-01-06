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
gPwm[1] ���O
gPwm[2] N.C.
gPwm[3] �E���
gPwm[4] N.C.
gPwm[5] �����
*/

//���[�^�[�̍ő�l�ƍŏ��l
#define MOTOR_MAX 85
#define MOTOR_MIN 25

//��ᐧ��̃��[�v�̉�
#define LINE 5
#define BALL 1

//����̃��C���Z���T�����������Ƃ��ɔ����������������
#define LINE_EXTEND 4

//�L�[�p�[�@�p
/*
��ނ�����ɑO�i����Ƃ��̑O�i��
�����I�Ɍ��ɉ�����Ƃ��̌�ޕb��
���̕b���ɂȂ�Ǝ����I�Ɍ��ɉ�����
���E���ꂼ��̃Z���T�l�̍��v�l�̍��͈͎̔w��
*/
#define KEEPER_FRONT 10
#define KEEPER_BACK_TIME 1000
#define KEEPER_BACK_TIMELIMIT 5000
#define DIF_LR 200

//�Z���T�̔����������l
/*
���C���Z���T�̔��������������l
�����̃��C���Z���T�����������Ƃ��̊Ԋu�������Ƃ��͈͎̔w��
�{�[���Z���T�̔��������������l
�{�[���Z���T�̃{�[���������莞�̍��v�l�͈͎̔Z�o�������l ������-������
�{�[���Z���T�̃{�[���������莞�̍��v�l�͈͎̔Z�o�������l ������-�ߋ���
�{�[���Z���T��2�Ԗڂɑ傫���Z���T�̒l�̌��o�\�̈�ݒ莞�Ɏg�p�����Z�l
�n���C�Z���T�̑O���͈͂������l
*/
#define LINE_LOW 700
#define LINE_TIME 500
#define BALL_LOW 400
#define BALL_FAR_MEDIUM 1000
#define BALL_MEDIUM_NEAR 1250
#define HIGH2 0.875
#define FRONT_RANGE 10

//�p�����䎞�̊e�Q�C���l
/*
CPU�̏�������
���萔
�����萔
*/
#define DELTA_T 0.01
#define KP 0.75
#define KD 0.075





//for���̃��[�v�p�ϐ�
/*
��ʎg�p�p�ϐ�1
��ʎg�p�p�ϐ�2
��ʎg�p�p�ϐ�3
�{�[���Z���T�l��r�p�ϐ�
���[�^�[����p�ϐ�
*/
int i;
int j;
int k;
int b;
int m;





//���[�^�[����p�ϐ�
/*
���[�^�[�p���[�ڕW�l [0]~[3]
���[�^�[�p���[���߉ߋ��l [0]~[3]
���[�^�[�p���[����l [0]~[3]
���{�b�g�̒��߉^�������̋L�^�z��
���߂̉^�������ɉ��������[�V���������̃t���O�ϐ�
�p�����䎞�Ɏ��ۂɏo�͂��郂�[�^�[�p���[
�����g�Z���T���g�p���ăR�[�g�����։�A���鎞�Ɏ��ۂɏo�͂��郂�[�^�[�p���[
�L�[�p�[�@���{�[�������m���Ȃ������Ƃ��Ɍ�ނ��邩�ǂ����̃t���O�ϐ�
*/
int m0;
int m1;
int m2;
int m3;
int lm0;
int lm1;
int lm2;
int lm3;
int inm0;
int inm1;
int inm2;
int inm3;
int lastMotion[6];
int motionLongFlag;
int difMotor;
int disMotor;
int backStop;

//�Z���T�l�̔�r�Ɏg�p����ϐ��̐錾
/*
�ł��Z���T�l���傫���Z���T�̃C���f�b�N�X
�ł��Z���T�l���傫���Z���T�̃Z���T�l
2�ԖڂɃZ���T�l���傫���Z���T�̃C���f�b�N�X
2�ԖڂɃZ���T�l���傫���Z���T�̃Z���T�l
2�ԖڂɃZ���T�l���傫���Z���T�̒l�����o�\�̈�ɂ��邩�ǂ����̃t���O�ϐ�
�ł��Z���T���傫���Z���T�l�̒l�Ƃ��̍��E�̃Z���T�̒l�̍��v
�����ɂ���S�Z���T�̃Z���T�l�̍��v�l
�E���ɂ���S�Z���T�̃Z���T�l�̍��v�l
�Z���T�l�̍��v�l�͈̔͂��L�^����ϐ�
�������l��菬�����l���o�͂����Z���T�̌�
��r�l���L�^����ϐ�
�����Z���T�֌W�̕ϐ�
*/
int highIndex;
int highValue;
int highIndex2;
int highValue2;
int high2Check;
int sumValue;
int sumLeft;
int sumRight;
int sumFlag;
int lowQuantity;
int comparingValue;

int disIndex;
int disValue;
int disIndex2;
int disValue2;

//�p������Ɏg�p����ϐ��̐錾
/*
�n���C�Z���T�̑O������
���݊p�x�l
���߉ߋ��p�x�l
�O�������ƌ��ݒl�̊p�x��
���݂̔����W��
���߉ߋ��̔����W��
*/
int front;
int now;
int previous;
int dif;
int nowD;
int previousD;


//�{�^������Ɏg�p����ϐ��̐錾
/*
�T�uCPU��CN6�̃{�^�� BTN_B
�T�uCPU��CN7�̃{�^�� BTN_A
�{�^����������Ă��郂�[�h���L�^����ϐ�
���{�b�g�̃��[�h��ݒ肷��ϐ�
A�{�^���������ꂽ�񐔂��L�^����ϐ�
*/
int btnB;
int btnA;
int btnMode;
int mode;
int btnTime;

//--------------------------------------------------
//@@@@@@@@@@
//switch�֐����甲���o���ϐ�
int switchEscape;
//--------------------------------------------------





//�Z���T���͒l�̎擾�֐�
//�֐��̈����Ɏ擾�Z���T�^�C�v����͂��邱��
int refreshSensor(int sensortype) {

    switch (sensortype) {
    case 0:
        //�{�[���Z���T�̒l���X�V����
        //�������l��菬�����l���o�͂����Z���T�̌���������
        lowQuantity = 0;

        //�Z���T�l���ł��傫���Z���T�̃C���f�b�N�X��������
        highIndex = 0;
        //�����������C���f�b�N�X�ɉ������Z���T�l�����ōł��傫���Z���T�l�Ƃ���
        highValue = gAD[highIndex];

        //�������Œu�����Z���T�l���������l�����������Ȃ�
        if (highValue < BALL_LOW) {
            //�������l��菬�����l���o�͂����Z���T�̌��ɉ��Z
            lowQuantity += 1;
        }

        //���E���ꂼ��̃Z���T�l�̍��v�l��������
        sumLeft = 0;
        sumRight = 0;

        //CN2����CN8�܂ł�7��J��Ԃ�
        for (b = 0; b < 7; b++) {

            //��r����l���擾
            comparingValue = gAD[b + 1];

            //������r���̒l���������l��菬�����Ȃ�
            if (comparingValue < BALL_LOW) {
                //�������l��菬�����l���o�͂����Z���T�̌��ɉ��Z
                lowQuantity += 1;
            }

            //������r���̒l���ł��傫���Z���T�l�����傫���Ȃ�
            if (comparingValue > highValue) {
                //�Z���T�l���ł��傫���Z���T�̃C���f�b�N�X�ƃZ���T�l���X�V
                highIndex = b + 1;
                highValue = comparingValue;
            }
        }

        //���E���ꂼ��̃Z���T�̍��v�l���Z�o����
        for (b = 0; b < 3; b++) {
            sumLeft += gAD[b + 1];
            sumRight += gAD[b + 5];
        }

        //2�ԖڂɃZ���T�l���傫���Z���T�����o����
        switch (highIndex) {
        case 0:
            highIndex2 = 1;
            break;

        default:
            highIndex2 = 0;
            break;
        }
        highValue2 = gAD[highIndex2];

        for (b = 0; b < 8; b++) {

            if (b == highIndex) {
                continue;
            }

            comparingValue = gAD[b];

            if (comparingValue > highValue2) {
                highIndex2 = b;
                highValue = comparingValue;
            }
        }

        //2�ԖڂɃZ���T�l���傫���Z���T�̒l�����o�\�̈�ɂ��邩����
        if (highValue2 >= highValue * HIGH2) {
            high2Check = 1;
        }
        else {
            high2Check = 0;
        }

        //�ł��Z���T�l���傫���Z���T�̍��E�̃Z���T�l�̍��v���Z�o����
        if (highIndex - 1 < 0) {
            sumValue = gAD[7];
        }
        else {
            sumValue = gAD[highIndex - 1];
        }

        if (highIndex + 1 > 7) {
            sumValue += gAD[0];
        }
        else {
            sumValue += gAD[highIndex + 1];
        }

        //�Z���T�l�̍��v�l����͈͂��Z�o�E�L�^����
        if (sumValue < BALL_FAR_MEDIUM) {
            //�{�[���܂ł������Ƃ�
            sumFlag = 0;
        }
        else if (BALL_FAR_MEDIUM <= sumValue && sumValue < BALL_MEDIUM_NEAR) {
            //�{�[���܂ł��������̂Ƃ�
            sumFlag = 1;
        }
        else if (BALL_MEDIUM_NEAR <= sumValue) {
            //�{�[���܂ł��߂��Ƃ�
            sumFlag = 2;
        }

        break;

    case 1:
        //���C���Z���T�̒l�擾
        //�T�uCPU�̕ϐ����擾���čX�V����
        highIndex = sub_io_get_gV(1, VAR_A);
        highValue = sub_io_get_gV(1, VAR_B);
        lowQuantity = sub_io_get_gV(1, VAR_C);
        highIndex2 = sub_io_get_gV(1, VAR_D);
        highValue2 = sub_io_get_gV(1, VAR_E);

        break;

    case 2:
        //�����Z���T�̒l�擾
        //�Z���T�l�ƃC���f�b�N�X���擾
        //�T�uCPU�̕ϐ����擾���čX�V����
        disIndex = sub_io_get_gV(1, VAR_F);
        disValue = sub_io_get_gV(1, VAR_G);
        disIndex2 = sub_io_get_gV(1, VAR_H);
        disValue2 = sub_io_get_gV(1, VAR_I);
    }

    //�֐����I������
    return(0);
}





//���[�^�[�̖ڕW�l�����[�^�[�̍ő�l�E�ŏ��l�ƏƂ炵���킹�ēK���l�ɒ����֐�
int motorCheck(int motor0, int motor1, int motor2, int motor3) {

    //�����̒l���O���[�o���ϐ��ɑ�����ċ^���O���[�o����
    m0 = motor0;
    m1 = motor1;
    m2 = motor2;
    m3 = motor3;

    //�����ڕW�l�̐�Βl���ő�l�����傫���Ȃ�ő�l�Ƀ��[�^�[�p���[�𒼂�
    if (m0 < MOTOR_MAX * -1) {
        m0 = MOTOR_MAX * -1;
    }
    else if (m0 > MOTOR_MAX) {
        m0 = MOTOR_MAX;
    }

    //�����ڕW�l�̐�Βl��0�łȂ��A�ŏ��l�����������Ȃ�ŏ��l�Ƀ��[�^�[�p���[�𒼂�
    if (0 < m0 && m0 < MOTOR_MIN) {
        m0 = MOTOR_MIN;
    }
    else if (MOTOR_MIN * -1 < m0 && m0 < 0) {
        m0 = MOTOR_MIN * -1;
    }

    if (m1 < MOTOR_MAX * -1) {
        m1 = MOTOR_MAX * -1;
    }
    else if (m1 > MOTOR_MAX) {
        m1 = MOTOR_MAX;
    }

    if (0 < m1 && m1 < MOTOR_MIN) {
        m1 = MOTOR_MIN;
    }
    else if (MOTOR_MIN * -1 < m1 && m1 < 0) {
        m1 = MOTOR_MIN * -1;
    }

    if (m2 < MOTOR_MAX * -1) {
        m2 = MOTOR_MAX * -1;
    }
    else if (m2 > MOTOR_MAX) {
        m2 = MOTOR_MAX;
    }

    if (0 < m2 && m2 < MOTOR_MIN) {
        m2 = MOTOR_MIN;
    }
    else if (MOTOR_MIN * -1 < m2 && m2 < 0) {
        m2 = MOTOR_MIN * -1;
    }

    if (m3 < MOTOR_MAX * -1) {
        m3 = MOTOR_MAX * -1;
    }
    else if (m3 > MOTOR_MAX) {
        m3 = MOTOR_MAX;
    }

    if (0 < m3 && m3 < MOTOR_MIN) {
        m3 = MOTOR_MIN;
    }
    else if (MOTOR_MIN * -1 < m3 && m3 < 0) {
        m3 = MOTOR_MIN * -1;
    }

    //�֐����I������
    return(0);
}





//��ᐧ�䂠��Ń��[�^�[�ɏo�͂���֐�
int move(int motor0, int motor1, int motor2, int motor3, int loopMotor) {

    //�����̖ڕW�l��K��������
    motorCheck(motor0, motor1, motor2, motor3);

    //�����̃��[�v�񐔂Ŕ�ᐧ��̃��[�v����
    for (m = 0; m < loopMotor; m++) {

        //���[�^�[�p���[�Ƃ��đ������l���Z�o����
        inm0 = ((m0 - lm0) / (loopMotor - m) + lm0);
        inm1 = ((m1 - lm1) / (loopMotor - m) + lm1);
        inm2 = ((m2 - lm2) / (loopMotor - m) + lm2);
        inm3 = ((m3 - lm3) / (loopMotor - m) + lm3);

        //���[�^�[�p���[��������
        gPwm[1] = inm0 < 0 ? inm0 * -1 : inm0 | 0x80;
        gPwm[5] = inm1 < 0 ? inm1 * -1 : inm1 | 0x80;
        gPwm[3] = inm2 < 0 ? inm2 * -1 : inm2 | 0x80;
        gPwm[0] = inm3 < 0 ? inm3 * -1 : inm3 | 0x80;

        //���[�^�[�𓮂���
        pwm_out();

        //���[�^�[�p���[�̒��߉ߋ��l���X�V����
        lm0 = inm0;
        lm1 = inm1;
        lm2 = inm2;
        lm3 = inm3;
    }

    //���[�^�[�p���[�̒��߉ߋ��l���X�V����
    lm0 = m0;
    lm1 = m1;
    lm2 = m2;
    lm3 = m3;

    //�֐����I������
    return(0);
}





//��ᐧ��Ȃ��Ń��[�^�[�ɏo�͂���֐�
int moveNatural(int motor0, int motor1, int motor2, int motor3) {

    //�����̃��[�^�[�p���[��K��������
    motorCheck(motor0, motor1, motor2, motor3);

    //���[�^�[�p���[��������
    gPwm[1] = m0 < 0 ? m0 * -1 : m0 | 0x80;
    gPwm[5] = m1 < 0 ? m1 * -1 : m1 | 0x80;
    gPwm[3] = m2 < 0 ? m2 * -1 : m2 | 0x80;
    gPwm[0] = m3 < 0 ? m3 * -1 : m3 | 0x80;

    //���[�^�[�𓮂���
    pwm_out();

    //���[�^�[�p���[�̒��߉ߋ��l���X�V����
    lm0 = m0;
    lm1 = m1;
    lm2 = m2;
    lm3 = m3;

    //�֐����I������
    return(0);
}





//��������֌J��Ԃ��^�����铮����}������֐�
int motionWrite(int motionDir) {

    if ((lastMotion[0] == motionDir) == FALSE) {
        lastMotion[0] = motionDir;
        set_Led(0, on);
        set_Led(1, off);
        return(0);
    }
    else {
        if ((lastMotion[1] == motionDir) == FALSE) {
            lastMotion[1] = motionDir;
            set_Led(0, off);
            set_Led(1, on);
            return(0);
        }
        else {
            if ((lastMotion[2] == motionDir) == FALSE) {
                lastMotion[2] = motionDir;
                set_Led(0, on);
                set_Led(1, on);
                return(0);
            }
            else {
                motionLongFlag = 1;
                lastMotion[0] = 0;
                lastMotion[1] = 0;
                lastMotion[2] = 0;
                set_Led(0, off);
                set_Led(1, off);
                return(0);
            }
        }
    }

}




//���C���֐�
void user_main(void) {

    //�T�uCPU�̋N����Ԃ��L�^����ϐ�
    int subFlag;

    //��ᐧ��p�̒��߉ߋ��l�̏�����
    lm0 = 0;
    lm1 = 0;
    lm2 = 0;
    lm3 = 0;

    //���߂̉^��������������
    for (i = 0; i < 3; i++) {
        lastMotion[i] = 0;
    }

    //�p������Ɏg�p����ϐ��̏������E�l�擾
    /*
    �O�������̊p�x��n���C�Z���T�Ŏ擾����
    ���Œ��߉ߋ��p�x�l�Ɍ��݊p�x�l����
    ���Œ��߉ߋ��̔����W����0����
    */
    gV[VAR_F] = get_bno(0);
    previous = get_bno(0);
    previousD = 0;

    //�T�uCPU�̕ϐ��Ƀ��C���Z���T�̂������l��������
    sub_io_set_gV(1, VAR_S, LINE_LOW);

    //���߂̉^�������ɉ��������[�V���������̃t���O�ϐ���������
    motionLongFlag = 0;

    //A�{�^���������ꂽ�񐔁E�{�^���������ꂽ���[�h��������
    btnTime = 0;
    btnMode = 0;

    //���[�h�ݒ�ɔ��
    goto modeSetting;





    //********************************************************************************
modeSetting:
    //���������[�h�ݒ蕔��������

    //A�{�^���������ꂽ�񐔂��L�^�AB�{�^���Ń��[�h����
    while (TRUE) {

        //���C���E�T�uCPU�ɕt�����Ă��邷�ׂĂ�LED��_��������
        set_Led(0, on);
        set_Led(1, on);
        sub_io_set_Led(1, 0, on);
        sub_io_set_Led(1, 1, on);

        btnB = sub_io_get_sensor(1, 5);
        if (btnB > 550) {
            goto main;
        }

        btnA = sub_io_get_sensor(1, 6);
        if (btnA > 550 && btnMode == 0) {
            btnTime += 1;
            btnMode = 1;
        }

        if (btnA <= 550) {
            btnMode = 0;
        }
    }

    //********************************************************************************

main:
    //���������C������������

    set_Led(0, off);
    set_Led(1, off);
    sub_io_set_Led(1, 0, off);
    sub_io_set_Led(1, 1, off);

    switch (btnTime) {
    case 0:
    case 1:
        //�U�����[�h
        //���������[�v
        while (TRUE) {

            //============================================================

            //�T�uCPU�̕ϐ���ǂݎ��i�ǂݎ��Ȃ��ꍇ�A���j
            subFlag = sub_io_get_gV(1, VAR_Z);

            //�����T�uCPU�̕ϐ���1�ȊO�i�ǂݎ��Ȃ��A���̒l�Ȃǁj�Ȃ�
            if (subFlag != 1) {

                //���ׂẴ��[�^�[���~������
                move(0, 0, 0, 0, LINE);

                //���������[�v�̍ŏ��ɖ߂葱����
                continue;
            }

            //============================================================

            //���ݒl���擾����
            now = get_bno(0);

            //�O�������ƌ��ݒl�Ƃ̊p�x�����Z�o����
            dif = gV[VAR_F] - now;

            //�p�x���̒l���r���₷���悤�ɒ�������
            if (dif <= -180) {
                dif += 360;
            }
            else if (dif >= 180) {
                dif -= 360;
            }

            //------------------------------------------------------------

            //�����p�x�����O�������͈̔͂������l�̒��ɂȂ��Ȃ�
            if ((-1 * FRONT_RANGE <= dif && dif <= FRONT_RANGE) == FALSE) {

                //�T�uCPU�̔��FLED��_��������
                sub_io_set_Led(1, 0, on);

                //�����W�����Z�o����
                nowD = (now - previous) / DELTA_T;

                //���[�^�[�p���[���Z�o����
                difMotor = KP * dif + KD * (nowD - previousD);

                //��ᐧ��̓����Ă��Ȃ��֐��Ń��[�^�[�ɏo�͂���
                moveNatural(difMotor, difMotor, difMotor, difMotor);

                //���߉ߋ��̊p�x���X�V����
                previous = now;
                //���߉ߋ��̔����W�����X�V����
                previousD = nowD;

                //���������[�v�̍ŏ��ɖ߂�
                continue;
            }

            //�T�uCPU�̔��FLED������������
            sub_io_set_Led(1, 0, off);

            //============================================================

            //���C���Z���T�̃Z���T�l���X�V����
            refreshSensor(1);

            //------------------------------------------------------------
            
            //�������C���Z���T�̂ǂꂩ���������Ă�����
            if (lowQuantity != 4) {

                //�T�uCPU�̐ԐFLED��_��������
                sub_io_set_Led(1, 1, on);

                switch (highIndex) {
                case 0:
                    //�O���C���Z���T������
                    for (i = 0; i < 3; i++) {
                        move(-80, -80, 80, 80, LINE);
                    }

                    motionWrite(10);

                    if (motionLongFlag == 1) {
                        move(-80, -80, 80, 80, LINE);
                        motionLongFlag = 0;
                    }

                    break;

                case 1:
                    //�����C���Z���T������
                    move(80, -80, -80, 80, LINE);

                    motionWrite(20);

                    if (motionLongFlag == 1) {
                        move(80, -80, -80, 80, LINE);
                        motionLongFlag = 0;
                    }

                    break;

                case 2:
                    //��냉�C���Z���T������
                    move(80, 80, -80, -80, LINE);

                    motionWrite(30);

                    if (motionLongFlag == 1) {
                        move(80, 80, -80, -80, LINE);
                        motionLongFlag = 0;
                    }

                    break;

                case 3:
                    //�E���C���Z���T������
                    move(-80, 80, 80, -80, LINE);

                    motionWrite(40);

                    if (motionLongFlag == 1) {
                        move(-80, 80, 80, -80, LINE);
                        motionLongFlag = 0;
                    }

                    break;
                }
                continue;

                //------------------------------------------------------------
                //���C���Z���T��1�������Ă���
                /*
                if (lowQuantity == 3) {

                    sub_io_set_Led(1, 0, off);

                    //�������Ă��郉�C���Z���T�ɉ����ē���
                    switch (highIndex) {
                    case 0:
                        //�O���C���Z���T������
                        for (i = 0; i < 3; i++) {
                            move(-80, -80, 80, 80, LINE);
                        }

                        motionWrite(10);

                        if (motionLongFlag == 1) {
                            move(-80, -80, 80, 80, LINE);
                            motionLongFlag = 0;
                        }

                        break;

                    case 1:
                        //�����C���Z���T������
                        move(80, -80, -80, 80, LINE);

                        motionWrite(20);

                        if (motionLongFlag == 1) {
                            move(80, -80, -80, 80, LINE);
                            motionLongFlag = 0;
                        }

                        break;

                    case 2:
                        //��냉�C���Z���T������
                        move(80, 80, -80, -80, LINE);

                        motionWrite(30);

                        if (motionLongFlag == 1) {
                            move(80, 80, -80, -80, LINE);
                            motionLongFlag = 0;
                        }

                        break;

                    case 3:
                        //�E���C���Z���T������
                        move(-80, 80, 80, -80, LINE);

                        motionWrite(40);

                        if (motionLongFlag == 1) {
                            move(-80, 80, 80, -80, LINE);
                            motionLongFlag = 0;
                        }

                        break;
                    }
                    continue;
                }

                //------------------------------------------------------------
                //���C���Z���T��2�������Ă���

                else if (lowQuantity == 2) {

                    sub_io_set_Led(1, 0, on);

                    switch (highIndex) {
                    case 0:
                        //�O���C���Z���T������
                        switch (highIndex2) {
                        case 1:
                            move(0, -80, 0, 80, LINE);
                            break;

                        case 3:
                            move(-80, 0, 80, 0, LINE);
                            break;

                        default:
                            move(-80, -80, 80, 80, LINE);
                            break;
                        }
                        break;

                    case 1:
                        //�����C���Z���T������
                        switch (highIndex2) {
                        case 0:
                            move(0, -80, 0, 80, LINE);
                            break;

                        case 2:
                            move(80, 0, -80, 0, LINE);
                            break;

                        default:
                            move(80, -80, -80, 80, LINE);
                            break;
                        }
                        break;

                    case 2:
                        //��냉�C���Z���T������
                        switch (highIndex2) {
                        case 1:
                            move(80, 0, -80, 0, LINE);
                            break;

                        case 3:
                            move(0, 80, 0, -80, LINE);
                            break;

                        default:
                            move(80, 80, -80, -80, LINE);
                            break;
                        }
                        break;

                    case 3:
                        //�E���C���Z���T������
                        switch (highIndex2) {
                        case 0:
                            move(-80, 0, 80, 0, LINE);
                            break;

                        case 2:
                            move(0, 80, 0, -80, LINE);
                            break;

                        default:
                            move(-80, 80, 80, -80, LINE);
                            break;
                        }
                        break;
                    }
                    continue;
                }
                */
            }

            //�T�uCPU�̐ԐFLED������������
            sub_io_set_Led(1, 1, off);
            
            //============================================================

            //�{�[���Z���T�̃Z���T�l���X�V����
            refreshSensor(0);

            //------------------------------------------------------------

            //�����{�[���Z���T�����ׂĔ������Ă��Ȃ��Ȃ�
            if (lowQuantity == 8) {
                //�u���[�L��������
                move(0, 0, 0, 0, 1);

                //���C��CPU�̐ԐFLED��_��������
                set_Led(1, on);

                //���������[�v�̍ŏ��ɖ߂�
                continue;
            }

            //���C��CPU�̐ԐFLED������������
            set_Led(1, off);

            //------------------------------------------------------------
            //��������Ȃ��ŁA�ł��傫���Z���T�̃C���f�b�N�X�݂̂ɉ������{�[���Ǐ]
            /*
            switch (highIndex) {
            case 0:
                //�O�F�O
                move(80, 80, -80, -80, BALL);
                break;

            case 1:
                //���O�F��
                move(-80, 80, 80, -80, BALL);
                break;

            case 2:
                //���F�����
                move(-80, 0, 80, 0, BALL);
                break;

            case 3:
            case 5:
                //�����/�E���F���
                move(-80, -80, 80, 80, BALL);
                break;

            case 4:
            case 7:
                //���/�E�O�F�E
                move(80, -80, -80, 80, BALL);
                break;

            case 6:
                //�E�F�E���
                move(0, -80, 0, 80, BALL);
                break;
            }
            //���������[�v�̍ŏ��ɖ߂�
            continue;
            */
            //------------------------------------------------------------
            //�������肠��ŁA�ł��傫���Z���T�̃C���f�b�N�X�ɉ������{�[���Ǐ]

            switch (sumFlag) {
            case 0:
                //������
                switch (highIndex) {
                case 0:
                    move(80, 80, -80, -80, BALL);
                    break;

                case 1:
                    move(0, 80, 0, -80, BALL);
                    break;

                case 2:
                    move(-80, 80, 80, -80, BALL);
                    break;

                case 3:
                case 4:
                    move(-80, 0, 80, 0, BALL);
                    break;

                case 5:
                    move(0, -80, 0, 80, BALL);
                    break;

                case 6:
                    move(80, -80, -80, 80, BALL);
                    break;

                case 7:
                    move(80, 0, -80, 0, BALL);
                    break;
                }
                break;

            case 1:
                switch (highIndex) {
                case 0:
                    move(80, 80, -80, -80, BALL);
                    break;

                case 1:
                    move(-80, 80, 80, -80, BALL);
                    break;

                case 2:
                    move(-80, 0, 80, 0, BALL);
                    break;

                case 3:
                case 5:
                    move(-80, -80, 80, 80, BALL);
                    break;

                case 4:
                case 7:
                    move(80, -80, -80, 80, BALL);
                    break;

                case 6:
                    move(0, -80, 0, 80, BALL);
                    break;
                }
                break;

            case 2:
                switch (highIndex) {
                case 0:
                    move(80, 80, -80, -80, BALL);
                    break;

                case 1:
                    move(-80, 80, 80, -80, BALL);
                    break;

                case 2:
                    move(-80, 0, 80, 0, BALL);
                    break;

                case 3:
                    move(0, -80, 0, 80, BALL);
                    break;

                case 4:
                case 7:
                    move(80, -80, -80, 80, BALL);
                    break;

                case 5:
                    move(-80, 0, 80, 0, BALL);
                    break;

                case 6:
                    move(0, -80, 0, 80, BALL);
                    break;
                }
                break;
            }
            continue;
        }
        break;

        //********************************************************************************

    case 2:
    case 3:
        //������[�h
        //���������[�v
        while (TRUE) {

            //============================================================

            //�T�u��CPU�̕ϐ���ǂݎ��i�ǂݎ��Ȃ��ꍇ�A���j
            subFlag = sub_io_get_gV(1, VAR_Z);

            //�����T�uCPU�̕ϐ���1�ȊO�i�ǂݎ��Ȃ��A���̒l�Ȃǁj�Ȃ�
            if (subFlag != 1) {

                //���[�^�[���~������
                move(0, 0, 0, 0, 1);

                //���������[�v�̍ŏ��ɖ߂葱����
                continue;
            }

            //============================================================
            
            //���ݒl���擾����
            now = get_bno(0);

            //�O�������ƌ��ݒl�Ƃ̊p�x�����Z�o����
            dif = gV[VAR_F] - now;

            //�p�x���̒l���r���₷���悤�ɒ�������
            if (dif <= -180) {
                dif += 360;
            }
            else if (dif >= 180) {
                dif -= 360;
            }

            //------------------------------------------------------------
            
            //�����p�x�����O�������͈̔͂������l�̒��ɂȂ��Ȃ�
            if ((-1 * FRONT_RANGE <= dif && dif <= FRONT_RANGE) == FALSE) {

                //�T�uCPU�̔��FLED��_��������
                sub_io_set_Led(1, 0, on);

                //�����W�����Z�o����
                nowD = (now - previous) / DELTA_T;

                //���[�^�[�p���[���Z�o����
                difMotor = KP * dif + KD * (nowD - previousD);

                //��ᐧ��̓����Ă��Ȃ��֐��Ń��[�^�[�ɏo�͂���
                moveNatural(difMotor, difMotor, difMotor, difMotor);

                //���߉ߋ��̊p�x���X�V����
                previous = now;
                //���߉ߋ��̔����W�����X�V����
                previousD = nowD;

                //���������[�v�̍ŏ��ɖ߂�
                continue;
            }

            //�T�uCPU�̔��FLED������������
            sub_io_set_Led(1, 0, off);
            
            //============================================================
            
            //���C���Z���T�̓��͒l���擾���ăO���[�o���ϐ������C���Z���T�p�ɍX�V����
            refreshSensor(1);

            //------------------------------------------------------------
            
            //�������C���Z���T�̂ǂꂩ���������Ă�����
            if (lowQuantity != 4) {

                //�T�uCPU�̐ԐFLED��_��������
                sub_io_set_Led(1, 1, on);

                //���߂��\�@�ł���C�����Ȃ�
                if (gT[1] <= LINE_TIME) {
                    for (i = 0; i < 3; i++) {
                        if (lastMotion[i] == 20 && lastMotion[i + 1] == 30) {
                            for (k = 0; k < 3; k++) {
                                move(80, 80, -80, -80, LINE);
                                if (lowQuantity != 4 && highIndex == 0) {
                                    continue;
                                }
                                continue;
                            }
                            continue;
                        }
                        else if (lastMotion[i + 1] == 20 && lastMotion[i] == 30) {
                            for (k = 0; k < 3; k++) {
                                move(80, 80, -80, -80, LINE);
                                if (lowQuantity != 4 && highIndex == 0) {
                                    continue;
                                }
                                continue;
                            }
                            continue;
                        }
                    }
                }

                //���C���Z���T��1�������Ă���

                if (lowQuantity == 3) {

                    //�������Ă��郉�C���Z���T�ɉ����ē���
                    switch (highIndex) {

                    case 0:
                        //�O���C���Z���T������
                        move(0, 0, 0, 0, BALL);
                        move(-80, -80, 80, 80, LINE);

                        motionWrite(10);

                        if (motionLongFlag == 1) {
                            for (j = 0; j < LINE_EXTEND; j++) {
                                refreshSensor(1);
                                if(lowQuantity != 4){
                                    break;
                                }
                                move(-80, -80, 80, 80, LINE);
                            }
                            motionLongFlag = 0;
                        }

                        break;

                    case 1:
                        //�����C���Z���T������
                        move(0, 0, 0, 0, BALL);
                        move(80, -80, -80, 80, LINE);

                        motionWrite(20);

                        if (motionLongFlag == 1) {
                            for (j = 0; j < LINE_EXTEND; j++) {
                                refreshSensor(1);
                                if(lowQuantity != 4){
                                    break;
                                }
                                move(80, -80, -80, 80, LINE);
                            }
                            motionLongFlag = 0;
                        }

                        clr_timer(1);

                        break;

                    case 2:
                        //��냉�C���Z���T������
                        move(0, 0, 0, 0, BALL);
                        move(80, 80, -80, -80, LINE);

                        motionWrite(30);

                        if (motionLongFlag == 1) {
                            for (j = 0; j < LINE_EXTEND; j++) {
                                refreshSensor(1);
                                if(lowQuantity != 4){
                                    break;
                                }
                                move(80, 80, -80, -80, LINE);
                            }
                            motionLongFlag = 0;
                        }

                        break;

                    case 3:
                        //�E���C���Z���T������
                        move(0, 0, 0, 0, BALL);
                        move(-80, 80, 80, -80, LINE);

                        motionWrite(40);

                        if (motionLongFlag == 1) {
                            for (j = 0; j < LINE_EXTEND; j++) {
                                refreshSensor(1);
                                if(lowQuantity != 4){
                                    break;
                                }
                                move(-80, 80, 80, -80, LINE);
                            }
                            motionLongFlag = 0;
                        }

                        clr_timer(1);

                        break;
                    }

                    continue;
                }

                //------------------------------------------------------------
                //���C���Z���T��2�������Ă���

                else if (lowQuantity == 2) {
                    switch (highIndex) {
                    case 0:
                        switch (highIndex2) {
                        case 1:
                            move(0, -80, 0, 80, LINE);
                            break;

                        case 3:
                            move(-80, 0, 80, 0, LINE);
                            break;
                        
                        default:
                            move(-80, -80, 80, 80, LINE);
                            break;
                        }
                        break;

                    case 1:
                        switch (highIndex2) {
                        case 0:
                            move(0, -80, 0, 80, LINE);
                            break;

                        case 2:
                            move(80, 0, -80, 0, LINE);
                            break;

                        default:
                            move(80, -80, -80, 80, LINE);
                            break;
                        }
                        break;

                    case 2:
                        switch (highIndex2) {
                        case 1:
                            move(80, 0, -80, 0, LINE);
                            break;

                        case 3:
                            move(0, 80, 0, -80, LINE);
                            break;

                        default:
                            move(80, 80, -80, -80, LINE);
                            break;
                        }
                        break;

                    case 3:
                        switch (highIndex2) {
                        case 0:
                            move(-80, 0, 80, 0, LINE);
                            break;

                        case 2:
                            move(0, 80, 0, -80, LINE);
                            break;

                        default:
                            move(-80, 80, 80, -80, LINE);
                            break;
                        }
                        break;
                    }
                }
                continue;
            }

            //�T�uCPU�̐ԐFLED������������
            sub_io_set_Led(1, 1, off);
            
            //============================================================
            
            //�{�[���Z���T�̓��͒l���擾���ăO���[�o���ϐ����{�[���Z���T�p�ɍX�V����
            refreshSensor(0);

            //------------------------------------------------------------

            //�����{�[���Z���T�̂ǂ�����������Ă��Ȃ�������
            if (lowQuantity == 8) {

                //------------------------------------------------------------
                //����ɉ����葱����Ƃ�
                if (backStop == 0) {

                    //���������[�v
                    while (TRUE) {
                        //�^�C�}�[1�����Z�b�g
                        clr_timer(0);

                        //���ɉ����葱����
                        move(-80, -80, 80, 80, BALL);

                        //���C���Z���T�̒l���X�V
                        refreshSensor(1);

                        //����1�̃��C���Z���T���������Ă�����
                        if (lowQuantity == 3) {
                            switch (highIndex) {
                                //���ȊO�̃��C���Z���T���������Ă����甒�������̕����ɓ���
                            case 0:
                                move(-80, -80, 80, 80, LINE);
                                break;

                            case 1:
                                move(80, -80, -80, 80, LINE);
                                break;

                            case 3:
                                move(-80, 80, 80, -80, LINE);
                                break;

                            case 2:
                                //��냉�C���Z���T���������Ă�����u���[�L�������Ă���
                                move(0, 0, 0, 0, LINE);

                                //���񐔑O�i����
                                for (k = 0; k < KEEPER_FRONT; k++) {
                                    move(80, 80, -80, -80, LINE);
                                }

                                //���̏�Œ�~���邩�ǂ����̃t���O�ϐ���ύX����
                                backStop = 1;

                                break;
                            }

                            //�t���O�ϐ����ύX����Ă�����
                            if (backStop == 1) {
                                //���������[�v���甲���o��
                                break;
                            }

                            //���������[�v�̍ŏ��ɖ߂�
                            continue;
                        }
                    }
                }

                //------------------------------------------------------------
                //���̏�Œ�~����Ƃ�
                else if (backStop == 1) {
                    //���̏�Œ�~����
                    move(0, 0, 0, 0, BALL);
                }

                //�^�C�}�[1�����Z�b�g
                clr_timer(0);

                //���������[�v�̍ŏ��ɖ߂�
                continue;
            }

            move(0, 0, 0, 0, LINE);
            continue;

            //------------------------------------------------------------
            /*
            //�{�[�������m������A�{�[�������m���Ȃ��Ȃ����Ƃ��Ɍ�ނ���悤�Ƀt���O�ϐ���������
            backStop = 0;

            //------------------------------------------------------------
            //�������肠��ŁA�ł��傫���Z���T�̃C���f�b�N�X�ɉ������{�[���Ǐ]

            //�^�C�}�[���g�p������莞�Ԍo�ߌ�Ɍ�ނ���v���O�����i�{�[���ʒu�͊����Ȃ��j
            if (gT[0] >= KEEPER_BACK_TIMELIMIT) {
                //���̃��C���Z���T����������܂Ō�ނ���
                while (TRUE) {
                    //���C���Z���T�̒l���X�V����
                    
                    refreshSensor(1);

                    if (lowQuantity == 3) {
                        switch (highIndex) {
                        case 0:
                            move(-80, -80, 80, 80, LINE);

                            //���������[�v�̍ŏ��ɖ߂�
                            continue;

                        case 1:
                            move(80, -80, -80, 80, LINE);

                            //���������[�v�̍ŏ��ɖ߂�
                            continue;

                        case 3:
                            move(-80, 80, 80, -80, LINE);

                            //���������[�v�̍ŏ��ɖ߂�
                            continue;

                        case 2:
                            for (k = 0; k < KEEPER_FRONT; k++) {
                                move(80, 80, -80, -80, LINE);
                            }

                            //switch���̐�ɐi��
                            break;
                        }

                        //���������[�v���甲���o��
                        break;
                    }
                    

                    if (gT[0] > KEEPER_BACK_TIMELIMIT + KEEPER_BACK_TIME) {
                        break;
                    }

                    move(-80, -80, 80, 80, BALL);

                    //���������[�v�̍ŏ��ɖ߂�
                    continue;
                }

                clr_timer(0);
            }

            //------------------------------------------------------------

            switch (sumFlag) {
            case 0:
                //������
            case 1:
                //������
                //���w�S�[���ւ̃V���[�g���u���b�N
                //�������E���ꂼ��̍��v�l�̍����}�C�i�X�Ȃ�
                if (sumLeft - sumRight < 0) {
                    //�������v�l�̍����C�ӂ͈̔͂ɓ����Ă��Ȃ��Ȃ�
                    if ((-1 * DIF_LR < sumLeft - sumRight) == FALSE) {
                        //�ł��Z���T�l���傫���Z���T�̃C���f�b�N�X�ɉ����ē�����ς���
                        switch (highIndex) {
                        case 3:
                        case 4:
                        case 5:
                            //�{�[�������{�b�g����ɂ���Ƃ��͒�~
                            move(0, 0, 0, 0, BALL);
                            break;

                        default:
                            //�����łȂ���΍��v�l�̑傫�������E�ɐi��
                            move(80, -80, -80, 80, BALL);
                            break;
                        }

                        //���������[�v�̍ŏ��ɖ߂�
                        continue;
                    }
                    //���v�l�̍����C�ӂ͈̔͂̒��Ȃ�
                    else {
                        //���{�b�g���~������
                        move(0, 0, 0, 0, BALL);

                        //���������[�v�̍ŏ��ɖ߂�
                        continue;
                    }
                }

                //���E���ꂼ��̍��v�l�̍����v���X�Ȃ�
                else if (sumLeft - sumRight > 0) {
                    //�������v�l�̍����C�ӂ͈̔͂ɓ����Ă��Ȃ��Ȃ�
                    if ((sumLeft - sumRight < DIF_LR) == FALSE) {
                        //�ł��Z���T�l���傫���Z���T�̃C���f�b�N�X�ɉ����ē�����ς���
                        switch (highIndex) {
                        case 3:
                        case 4:
                        case 5:
                            //�{�[�������{�b�g����ɂ���Ƃ��͒�~
                            move(0, 0, 0, 0, BALL);
                            break;

                        default:
                            //�����łȂ���΍��v�l�̑傫���������ɐi��
                            move(-80, 80, 80, -80, BALL);
                            break;
                        }

                        //���������[�v�̍ŏ��ɖ߂�
                        continue;
                    }
                    //���v�l�̍����C�ӂ͈̔͂̒��Ȃ�
                    else {
                        //���{�b�g���~������
                        move(0, 0, 0, 0, BALL);

                        //���������[�v�̍ŏ��ɖ߂�
                        continue;
                    }
                }
                break;
                //------------------------------------------------------------

            case 2:
                //�ߋ���
                //�{�[����ߑ��E�S�[����
                //�ł��Z���T�l���傫���Z���T�̃C���f�b�N�X�ɉ����ē�����ς���
                switch (highIndex) {
                case 0:
                    //�O�F�O
                    move(80, 80, -80, -80, BALL);
                    break;

                case 1:
                    //���O�F��
                    move(-80, 80, 80, -80, BALL);
                    break;

                case 2:
                    //���F�����
                    move(-80, 0, 80, 0, BALL);
                    break;

                case 3:
                case 5:
                    //�����/�E���F���
                    move(-80, -80, 80, 80, BALL);
                    break;

                case 4:
                case 7:
                    //���/�E�O�F�E
                    move(80, -80, -80, 80, BALL);
                    break;

                case 6:
                    //�E�F�E���
                    move(0, -80, 0, 80, BALL);
                    break;
                }
                break;
            }
            */
            //============================================================
            /*
            //�ȉ��̐����P�̂Ŏ��s���邱�Ƃ͂Ȃ�
            //�����Z���T�̓��͒l���擾���ăO���[�o���ϐ��������Z���T�p�ɍX�V����
            refreshSensor(2);

            if ((disValue > 1000 || disValue2 = 0) == FALSE) {
                disMotor = (disValue - disValue2) * 0.1;
                switch (disIndex){
                case 7:
                    move(disMotor, disMotor, disMotor, disMotor, BALL);
                    break;
                case 9:
                    move(disMotor, disMotor, disMotor, disMotor, BALL);
                    break;
                }
            }
            */
        }
        break;
        
        //********************************************************************************

    case 4:
        //�{�[���̉����`�F�b�J�[���[�h
        while (TRUE) {

            //============================================================

            //�T�uCPU�̕ϐ���ǂݎ��i�ǂݎ��Ȃ��ꍇ�A���j
            subFlag = sub_io_get_gV(1, VAR_Z);

            //�����T�uCPU�̕ϐ���1�ȊO�i�ǂݎ��Ȃ��A���̒l�Ȃǁj�Ȃ�
            if (subFlag != 1) {

                //���ׂẴ��[�^�[���~������
                move(0, 0, 0, 0, LINE);

                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                btnTime = 0;

                goto modeSetting;

                //���������[�v�̍ŏ��ɖ߂葱����
                continue;
            }

            //============================================================

            refreshSensor(0);

            if (lowQuantity == 8) {
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                continue;
            }

            set_Led(1, off);

            switch (sumFlag) {
            case 0:
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 1:
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                break;

            case 2:
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, on);
                break;
            }

            //============================================================

            continue;
        }
        break;

        //********************************************************************************

    case 5:
        //�{�[����8���ʊp�x�`�F�b�J�[���[�h
        while (TRUE) {

            //============================================================

            //�T�uCPU�̕ϐ���ǂݎ��i�ǂݎ��Ȃ��ꍇ�A���j
            subFlag = sub_io_get_gV(1, VAR_Z);

            //�����T�uCPU�̕ϐ���1�ȊO�i�ǂݎ��Ȃ��A���̒l�Ȃǁj�Ȃ�
            if (subFlag != 1) {

                //���ׂẴ��[�^�[���~������
                move(0, 0, 0, 0, LINE);

                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                btnTime = 0;

                goto modeSetting;

                //���������[�v�̍ŏ��ɖ߂葱����
                continue;
            }

            //============================================================

            refreshSensor(0);

            if (lowQuantity == 8) {
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
                continue;
            }

            switch (highIndex) {
            case 0:
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 1:
                set_Led(0, on);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 2:
                set_Led(0, off);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 3:
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
                break;

            case 4:
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 5:
                set_Led(0, on);
                set_Led(1, off);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 6:
                set_Led(0, off);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;

            case 7:
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                break;
            }

            //============================================================

            continue;
        }
        break;

        //********************************************************************************

    case 6:
        //�{�[����16���ʊp�x�`�F�b�J�[���[�h
        while (TRUE) {

            //============================================================

            //�T�uCPU�̕ϐ���ǂݎ��i�ǂݎ��Ȃ��ꍇ�A���j
            subFlag = sub_io_get_gV(1, VAR_Z);

            //�����T�uCPU�̕ϐ���1�ȊO�i�ǂݎ��Ȃ��A���̒l�Ȃǁj�Ȃ�
            if (subFlag != 1) {

                //���ׂẴ��[�^�[���~������
                move(0, 0, 0, 0, LINE);

                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                btnTime = 0;

                goto modeSetting;

                //���������[�v�̍ŏ��ɖ߂葱����
                continue;
            }

            //============================================================

            refreshSensor(0);

            //------------------------------------------------------------

            if (lowQuantity == 8) {
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, on);
                continue;
            }

            //------------------------------------------------------------
            //�ł��Z���T�l���傫���Z���T�̃C���f�b�N�X�݂̂Ō��m

            if (high2Check == 0) {
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, on);
            }

            //------------------------------------------------------------
            //�ł��Z���T�l���傫���Z���T��2�ԖڂɃZ���T�l���傫���Z���T�̃C���f�b�N�X�Ō��m

            else if (high2Check == 1) {

                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
            }
            //============================================================

            continue;
        }
        break;

        //********************************************************************************

    case 7:
        //�{�[���̍��E�e���v�l��r�`�F�b�J�[���[�h

        while (TRUE) {

            //============================================================

            //�T�uCPU�̕ϐ���ǂݎ��i�ǂݎ��Ȃ��ꍇ�A���j
            subFlag = sub_io_get_gV(1, VAR_Z);

            //�����T�uCPU�̕ϐ���1�ȊO�i�ǂݎ��Ȃ��A���̒l�Ȃǁj�Ȃ�
            if (subFlag != 1) {

                //���ׂẴ��[�^�[���~������
                move(0, 0, 0, 0, LINE);

                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                btnTime = 0;

                goto modeSetting;

                //���������[�v�̍ŏ��ɖ߂葱����
                continue;
            }

            //============================================================

            refreshSensor(0);

            //------------------------------------------------------------

            if (lowQuantity == 8) {
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, off);
                continue;
            }

            //------------------------------------------------------------

            /*
            ���E�̍��v�l���͈͓�/�O�@�@�@�@�@�@���C����
            ���E�̍��v�l�ō������傫��/�������@���C����
            �{�[�������m���Ă��Ȃ�/����@�@�@�@�T�u��
            �{�[���̓��{�b�g����ɂ���/�Ȃ��@�@�T�u��
            */

            if (sumLeft - sumRight < 0) {
                if ((-1 * DIF_LR < sumLeft - sumRight) == FALSE) {
                    switch (highIndex) {
                    case 3:
                    case 4:
                    case 5:
                        //�{�[���͉E�� & ���{�b�g���
                        set_Led(0, on);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, on);
                        continue;

                    default:
                        //�{�[���͉E��
                        set_Led(0, on);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, off);
                        continue;
                    }
                }
                else {
                    //���E�̍��v�l�̍����͈͓�
                    set_Led(0, off);
                    set_Led(1, off);
                    sub_io_set_Led(1, 0, off);
                    sub_io_set_Led(1, 1, off);
                    continue;
                }
            }
            else if (sumLeft - sumRight > 0) {
                if ((sumLeft - sumRight < DIF_LR) == FALSE) {
                    switch (highIndex) {
                    case 3:
                    case 4:
                    case 5:
                        //�{�[���͍��� & ���{�b�g���
                        set_Led(0, on);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, on);
                        continue;

                    default:
                        //�{�[���͍���
                        set_Led(0, on);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, off);
                        continue;
                    }
                }
                else {
                    //���E�̍��v�l�̍����͈͓�
                    set_Led(0, off);
                    set_Led(1, off);
                    sub_io_set_Led(1, 0, off);
                    sub_io_set_Led(1, 1, off);
                    continue;
                }
            }

            //============================================================

            continue;
        }
        break;

        //********************************************************************************

    case 8:
        //���C���Z���T�`�F�b�J�[���[�h
        while (TRUE) {

            //============================================================

            //�T�uCPU�̕ϐ���ǂݎ��i�ǂݎ��Ȃ��ꍇ�A���j
            subFlag = sub_io_get_gV(1, VAR_Z);

            //�����T�uCPU�̕ϐ���1�ȊO�i�ǂݎ��Ȃ��A���̒l�Ȃǁj�Ȃ�
            if (subFlag != 1) {

                //���ׂẴ��[�^�[���~������
                move(0, 0, 0, 0, LINE);

                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                btnTime = 0;

                goto modeSetting;

                //���������[�v�̍ŏ��ɖ߂葱����
                continue;
            }

            //============================================================

            refreshSensor(1);

            //------------------------------------------------------------

            if (lowQuantity == 4) {
                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);
            }

            //------------------------------------------------------------

            switch (lowQuantity) {
            case 3:
                //1�̃��C���Z���T������
                switch (highIndex) {
                case 0:
                    set_Led(0, off);
                    set_Led(1, off);
                    sub_io_set_Led(1, 0, on);
                    sub_io_set_Led(1, 1, off);
                    break;

                case 1:
                    set_Led(0, on);
                    set_Led(1, off);
                    sub_io_set_Led(1, 0, on);
                    sub_io_set_Led(1, 1, off);
                    break;

                case 2:
                    set_Led(0, off);
                    set_Led(1, on);
                    sub_io_set_Led(1, 0, on);
                    sub_io_set_Led(1, 1, off);
                    break;

                case 3:
                    set_Led(0, on);
                    set_Led(1, on);
                    sub_io_set_Led(1, 0, on);
                    sub_io_set_Led(1, 1, off);
                    break;
                }
                break;

            case 2:
                //2�̃��C���Z���T������
                switch (highIndex) {
                case 0:
                    switch (highIndex2) {
                    case 1:
                        set_Led(0, off);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 2:
                        set_Led(0, off);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, on);
                        break;

                    case 3:
                        set_Led(0, off);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, on);
                        break;
                    }
                    break;

                case 1:
                    switch (highIndex2) {
                    case 0:
                        set_Led(0, on);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 2:
                        set_Led(0, on);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, on);
                        break;

                    case 3:
                        set_Led(0, on);
                        set_Led(1, off);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, on);
                        break;
                    }
                    break;

                case 2:
                    switch (highIndex2) {
                    case 0:
                        set_Led(0, off);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 1:
                        set_Led(0, off);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 3:
                        set_Led(0, off);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, on);
                        break;
                    }
                    break;

                case 3:
                    switch (highIndex2) {
                    case 0:
                        set_Led(0, on);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 1:
                        set_Led(0, on);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, on);
                        sub_io_set_Led(1, 1, off);
                        break;

                    case 2:
                        set_Led(0, on);
                        set_Led(1, on);
                        sub_io_set_Led(1, 0, off);
                        sub_io_set_Led(1, 1, on);
                        break;
                    }
                    break;
                }
                break;

            case 1:
            case 0:
                //3�܂���4�̃��C���Z���T������
                set_Led(0, on);
                set_Led(1, on);
                sub_io_set_Led(1, 0, on);
                sub_io_set_Led(1, 1, on);
                break;
            }

            //============================================================

            continue;
        }
        break;

        //********************************************************************************

    case 9:
        //�n���C�Z���T�`�F�b�J�[���[�h
        while (TRUE) {

            //============================================================

            //�T�uCPU�̕ϐ���ǂݎ��i�ǂݎ��Ȃ��ꍇ�A���j
            subFlag = sub_io_get_gV(1, VAR_Z);

            //�����T�uCPU�̕ϐ���1�ȊO�i�ǂݎ��Ȃ��A���̒l�Ȃǁj�Ȃ�
            if (subFlag != 1) {

                //���ׂẴ��[�^�[���~������
                move(0, 0, 0, 0, LINE);

                set_Led(0, off);
                set_Led(1, off);
                sub_io_set_Led(1, 0, off);
                sub_io_set_Led(1, 1, off);

                btnTime = 0;

                goto modeSetting;

                //���������[�v�̍ŏ��ɖ߂葱����
                continue;
            }

            //============================================================

            //���ݒl���擾����
            now = get_bno(0);

            //�O�������ƌ��ݒl�Ƃ̊p�x�����Z�o����
            dif = gV[VAR_F] - now;

            //�p�x���̒l���r���₷���悤�ɒ�������
            if (dif <= -180) {
                dif += 360;
            }
            else if (dif >= 180) {
                dif -= 360;
            }

            //------------------------------------------------------------

            //�����p�x�����O�������͈̔͂������l�̒��ɂȂ��Ȃ�
            if ((-1 * FRONT_RANGE <= dif && dif <= FRONT_RANGE) == FALSE) {

                //�T�uCPU�̔��FLED��_��������
                sub_io_set_Led(1, 0, on);

                //�����p�x�����͈͂������l�����������Ȃ�
                if (dif < -1 * FRONT_RANGE) {
                    //�T�uCPU�̐ԐFLED��_��������
                    sub_io_set_Led(1, 1, on);
                }

                //�����p�x�����͈͂������l�����傫���Ȃ�
                else if (FRONT_RANGE < dif) {
                    //�T�uCPU�̐ԐFLED������������
                    sub_io_set_Led(1, 1, off);
                }

                //���������[�v�̍ŏ��ɖ߂�
                continue;
            }

            //�T�uCPU�̔��FLED�ƐԐFLED������������
            sub_io_set_Led(1, 0, off);
            sub_io_set_Led(1, 1, off);

            //============================================================
        }
        break;

        //********************************************************************************  

    default:
        //���[�^�[���~������
        while (TRUE) {
            move(0, 0, 0, 0, 1);
        }
        break;
    }
}