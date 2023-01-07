#include "D_Main.h"
#include "D_I2C.h"
#include "D_SIO.h"
#include "D_EIO.h"

/*
[0]CN1  : 前
[1]CN2  : 左
[2]CN3  : 後ろ
[3]CN4  : 右
[4]CN5  : なし
[5]CN6  : なし
[6]CN7  : なし
[7]CN8  : なし
[8]CN9  : なし
[9]CN10 : なし
*/

//ループ用変数
int l;

//ラインセンサの反応しきい値
int LINE_LOW;

//メイン関数
void user_main(void) {
    //センサ値比較に使用する変数の宣言
    /*
    センサ値が最も大きいセンサのインデックス
    センサ値が最も大きいセンサのセンサ値
    センサ値が二番目に大きいセンサのインデックス
    センサ値が二番目に大きいセンサのセンサ値
    しきい値より低いセンサ値を出力したセンサの個数
    */
	int highIndex;
	int highValue;
	int highIndex2;
	int highValue2;
	int lowQuantity;

    //比較中のセンサ値
	int comparingValue;
	
	//サブCPUの状態（起動状態）をグローバル化されたフラグ変数に記録
	gV[VAR_Z] = 1;
        
	//無条件ループ
    while (TRUE) {
        //しきい値をグローバル変数から読み込む
        LINE_LOW = gV[VAR_S];

        //------------------------------------------------------------
        //最も大きいセンサ値を取得する

        //しきい値より低いセンサ値を出力したセンサの個数を初期化
        lowQuantity = 0;

        //センサ値が最も大きいセンサのインデックスを初期化
        highIndex = 0;
        //初期化されたインデックスに対応したセンサ値を仮で最も大きいセンサ値とおく
        highValue = gAD[highIndex];

        //もし仮で取得した最も大きいセンサ値がしきい値より低いなら
        if (highValue < LINE_LOW) {
            //しきい値より低いセンサ値を出力したセンサの個数に加算
            lowQuantity += 1;
        }

        //CN2からCN4までの3回繰り返す
        for (l = 0; l < 3; l++) {

            //比較するセンサ値を取得
            comparingValue = gAD[l + 1];

            //もし比較中のセンサ値がしきい値より低いなら
            if (comparingValue < LINE_LOW) {
                lowQuantity += 1;
            }

            //もし比較中のセンサ値が最も大きいセンサ値よりも大きいなら
            if (comparingValue > highValue) {
                highIndex = l + 1;
                highValue = comparingValue;
            }
        }

        //メインCPUから変数の読み取りができるようにグローバル変数にそれぞれの変数を代入
        gV[VAR_A] = highIndex;
        gV[VAR_B] = highValue;
        gV[VAR_C] = lowQuantity;

        //------------------------------------------------------------

        //------------------------------------------------------------
        //2番目に大きいセンサ値を取得する

        //センサ値が2番目に大きいセンサのインデックスを初期化
        switch (highIndex) {
        case 0:
            highIndex2 = 1;
            break;

        default:
            highIndex2 = 0;
            break;
        }

        //初期化されたインデックスに対応したセンサ値を仮で2番目に大きいセンサ値とおく
        highValue2 = gAD[highIndex2];

        //CN1からCN4までの4回繰り返す
        for (l = 0; l < 4; l++) {

            //もし比較するインデックスがセンサ値が元雄も大きいセンサのインデックスと同一なら
            if (l == highIndex) {
                //スキップする
                continue;
            }

            //比較するセンサ値を取得
            comparingValue = gAD[l];

            //もし比較中のセンサ値が2番目に大きいセンサ値よりも大きいなら
            if (comparingValue > highValue2) {
                highIndex2 = l;
                highValue2 = comparingValue;
            }
        }

        //メインCPUから変数の読み取りができるようにグローバル変数にそれぞれの変数を代入
        gV[VAR_D] = highIndex2;
        gV[VAR_E] = highValue2;

        //------------------------------------------------------------
    }
}
