/*
 * ICM.c
 *
 *  Created on: 2025年10月30日
 *      Author: ji_rencai
 */
#include "ICM.h"


#define sampleFreq  4000.0f    //闁插洦鐗辨０鎴犲芳閿涘牐浣嗛崗鐧哥礆
#define twoKpDef    (2.0f * 0.5f)  //0.5  2.0
#define twoKiDef    (2.0f * 0.0f)  //0.02
#define DT 0.00025     // 闁插洦鐗遍崨銊︽埂閿涳拷4000Hz閿涘苯宓� 0.25 ms
#define G 9.81         // 闁插秴濮忛崝鐘伙拷鐔峰閿涳拷1 g = 9.81 m/s铏�

/*
 * twoKpDef,濮ｆ柧绶ユ晶鐐垫抄閿涘牆缍嬮崜宥勮礋1閿涘鐨熼弫瀵搁兇缂佺喎鎼锋惔鏃堬拷鐔峰
 * p鐡掑﹪鐝敍宀冾嚖瀹割喚娈戣箛顐︼拷鐔峰冀鎼存棁绉哄鐚寸礉娴ｅ棗顕遍懛鏉戝З閹焦锟窖嗗厴閸欐ê妯婇敍鍫ｇТ鐠嬪喛绱�
 *  twoKiDef閿涘瞼袧閸掑棗顤冮惄濠忕礄閻╊喖澧犳稉锟�0閿涘绱濈拫鍐╂殻闂€鎸庢埂閹存牗鍙冮柅鐔峰綁閸栨牞绉奸崝璺ㄦ畱缁剧姵顒滈懗钘夊
 */

volatile float invsampleFreq = 1.0f / sampleFreq;
volatile float twoKp = twoKpDef;
volatile float twoKi = twoKiDef;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                  // quaternion of sensor frame relative to auxiliary frame
volatile float q01 = 1.0f, q11 = 0.0f, q21 = 0.0f, q31 = 0.0f;
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki
// 缁犳纭堕崣鍌涙殶閿涙瓬eta 娑撻缚鐨熼懞鍌氬冀妫ｅ牊顒為梹璺ㄦ畱缁粯鏆熼敍灞炬殶閸婅壈绉烘径褍鎼锋惔鏃囩Ш韫囶偓绱濇担鍡楀讲閼宠棄娅旀竟鎷岀窛婢讹拷
volatile float beta = 0.11f;

/* 1. volatile float 缂傛牞鐦ч崳銊ょ瑝娴兼氨绱︾€涙ǹ顕氶崣姗€鍣洪崐纭风礉绾喕绻氬В蹇旑偧鐠佸潡妫堕柈鐣屾纯閹恒儰绮犻崘鍛摠娑擃叀顕伴崣锟�
 * invSampleFreq 閸楁洑缍呴弮鍫曟？闁插洦鐗遍弫浼村櫤
 * 2. twoKp 濮ｆ柧绶ユ晶鐐垫抄
 * 3. twoKi 缁夘垰鍨庢晶鐐垫抄
 * 4. q0, q1, q2, q3 閸ユ稑鍘撻弫鏉垮灥婵锟界》绱濋崚鍡楀焼娴狅綀銆冩稉锟芥稉顏冪炊閹扮喎娅掗惄绋款嚠娴滃氦绶熼崝鈺傤攱閺嬪墎娈戦弮瀣祮
 *    閻€劍娼甸幓蹇氬牚閻椻晙缍嬮崷銊ょ瑏缂佸鈹栭梻缈犺厬閻ㄥ嫬协閹礁鎷伴弮瀣祮
 * 5. integralFBx, integralFBy, integralFBz 缁夘垰鍨庨崣宥夘洯鐠囶垰妯婃い锟�
 */


/* 鐟欐帒瀹� */
float Roll_A0, Pitch_A0, Yaw_A0;//閸掓繂顫愭慨鎸庯拷浣筋潡閿涘濠婃俺娴嗙憴鎺炵礉Y娣囶垯璇濈憴鎺炵礉Z閸嬪繗鍩呯憴鎺炵礄瀵冨閿涳拷
float Yaw_Lock_a;               //闁夸礁鐣鹃悩鑸碉拷浣烘畱鐟欐帒瀹砯loat Pitch_a, Roll_a, Yaw_a;   //鐎圭偞妞傛慨鎸庯拷浣筋潡
float Pitch_a_Pi, Roll_a_Pi, Roll_error_Pi, Yaw_a_Pi, Yaw_a_Pi_genrl, Yaw_a_Pi_local;
                                //閸欐牗膩鐟欐帒瀹抽崐纭风礉绾喕绻氶崷銊﹀瘹鐎规俺瀵栭崶杈剧礄0閸掕穻锟介幋鏍у弿閸涖劍婀￠敍澶婂敶
float Pitch_a_last, Roll_a_last, Yaw_a_last;
                                //閸樺棗褰舵慨鎸庯拷浣筋潡娣団剝浼呴敍宀€鏁ゆ禍搴ｅЦ閹浇绐￠煪顏傦拷浣规姢濞夘澁绱濇０鍕ゴ濮濄儵顎�
float aim_roll_a,aim_pitch_a,aim_yaw_a,d_aim_yaw_a;
                                //閸嬪繗鍩呯憴鎺旀畱閹稿洣鎶ら惃鍕殶閺佹挳锟界喎瀹�
/* (鐟欙拷)闁喎瀹� */
float Pitch_g, Roll_g, Yaw_g;   //鐎圭偞妞傜憴鎺楋拷鐔峰
float Pitch_g_F, Roll_g_F, Yaw_g_F;
                                //濠娿倖灏濈憴鎺楋拷鐔峰
float Pitch_g_F_last, Roll_g_F_last;
                                // 娑撳﹣绔村▎锛勬畱濠娿倖灏濈憴鎺楋拷鐔峰

float Pitch_g_last, Roll_g_last, Yaw_g_last;
                                //娑撳﹣绔撮崨銊︽埂閻ㄥ嫯顫楅柅鐔峰閿涘瞼鏁ゆ担婊冨棘閼板啯鍨ㄩ崢鍡楀蕉閺佺増宓侀悙锟�
float aim_x_v,aim_y_v,aim_y_v_last,aim_yaw_g;
                                //閻╊喗鐖ｇ憴鎺楋拷鐔峰
/* (缁撅拷)闁喎瀹� */
float X_V,Y_V;                  //缁惧潡锟界喎瀹抽崚鍡涘櫤
float X_V_R_last,Y_V_R_last;    //閸撳秳绔撮崨銊︽埂闁喎瀹抽弫鐗堝祦閿涘牏娴夌€甸€涚艾閺屾劒閲滈崣鍌濓拷鍐仯閻ㄥ嫰锟界喎瀹抽敍锟�
float X_V_last,Y_V_last;        //娑撳﹣绔存稉顏呮闂傛挳妫块梾鏃傛畱缁惧潡锟界喎瀹抽崐锟�
float X_V_F,Y_V_F;              //濠娿倖灏濈痪鍧楋拷鐔峰鐠囩粯鏆�
/* 閸旂娀锟界喎瀹� */
float Pitch_acc, Roll_acc, Yaw_acc;
                                //鐟欐帒濮為柅鐔峰
float Pitch_acc_F, Roll_acc_F, Yaw_acc_F;
                                //鏉╁洦鎶ょ憴鎺戝闁喎瀹�
float Pitch_a, Roll_a, Yaw_a;
float Pitch_b, Roll_b, Yaw_b;
float Pitch_c, Roll_c, Yaw_c;
float Pitch_b_use, Roll_b_use, Yaw_b_use;
float Pitch, Roll, Yaw;
biquad_state X_V_biquad, Y_V_biquad,
             Pitch_g_biquad, Roll_g_biquad, Yaw_g_biquad,
             Pitch_acc_biquad, Roll_acc_biquad, Yaw_acc_biquad;
float Yaw_angular_acc,Pitch_angular_acc;
volatile PID_t PIDK_AA = {700,0.15,7},// X 楠炲磭些
               PIDK_AB = {1,0,0},
               PIDK_AC = {1,0,10},

               PIDK_BA = {700,0.15,7},// Y 楠炲磭些
               PIDK_BB = {0,0,0},
               PIDK_BC = {5,0,20},

               PIDK_YG = {10,0.005,1},// Yaw
               PIDK_YA = {10,0,0},
               PIDK_YV = {0,0,15},

               PIDK_CA = {0,0,0},
               PIDK_CB = {0,0,0},
               PIDK_CC = {0,0,50};

//闂勶拷閾昏桨鍗庢潏鍛И閸掋倖鏌�//  閹存垶濡哥拠璇诧拷鍏兼杹閸掔櫟et_ICM_data()闁插奔绨�
float Yaw_angle_counter = 0.0;
float Yaw_angle  = 0.0;//yaw鐟欐帒瀹�
float Yaw_gyro = 0.0;//yaw鐟欐帒濮為柅鐔峰

static double P[3][3] = {
    {1e-3, 0,    0},
    {0,    1e-3, 0},
    {0,    0,    1e-3}
};
// 鐏忓棜顫楁惔锕傛閸掕泛婀� (-锜�, 锜篯 閻ㄥ嫬鐨銉ュ徔閸戣姤鏆�
//static double wrapToPi(double angle)
//{
//    while (angle >  M_PI) angle -= 2.0 * M_PI;
//    while (angle <= -M_PI) angle += 2.0 * M_PI;
//    return angle;
//}

float invSqrt(float x);     //濮瑰倸锟芥帗鏆�

// 閸忋劌鐪� EKF 閻樿埖锟戒礁寮烽崚婵嗩潗閸栨牗鐖ｈ箛妤嬬礄閺€鎯ф躬娑擃厽鏌囩粙瀣碍娑擃叀鐨熼悽銊︽娣囨繃瀵旈崗銊ョ湰閻樿埖锟戒緤绱�
EKF_State ekf;
int ekf_initialized = 0;

// 鏉堝懎濮崙鑺ユ殶閿涙艾鎮滈柌蹇撶秺娑擄拷閸栵拷
void normalize_vector(float *v, int n) {
    float norm = 0.0f;
    for (int i = 0; i < n; i++) {
        norm += v[i]*v[i];
    }
    norm = sqrtf(norm);
    if (norm < 1e-6f) return;
    for (int i = 0; i < n; i++) {
        v[i] /= norm;
    }
}

// 鏉堝懎濮崙鑺ユ殶閿涳拷3x3 閻晠妯€濮瑰倿锟藉棴绱欓弰鎯х础濮瑰倽袙閿涳拷
int invert_3x3(const float A[3][3], float Ainv[3][3]) {
    float det = A[0][0]*(A[1][1]*A[2][2]-A[1][2]*A[2][1])
              - A[0][1]*(A[1][0]*A[2][2]-A[1][2]*A[2][0])
              + A[0][2]*(A[1][0]*A[2][1]-A[1][1]*A[2][0]);
    if (fabsf(det) < 1e-6f) return -1; // 婵傚洤绱撻惌鈺呮█
    float invdet = 1.0f/det;
    Ainv[0][0] =  (A[1][1]*A[2][2]-A[1][2]*A[2][1])*invdet;
    Ainv[0][1] = -(A[0][1]*A[2][2]-A[0][2]*A[2][1])*invdet;
    Ainv[0][2] =  (A[0][1]*A[1][2]-A[0][2]*A[1][1])*invdet;
    Ainv[1][0] = -(A[1][0]*A[2][2]-A[1][2]*A[2][0])*invdet;
    Ainv[1][1] =  (A[0][0]*A[2][2]-A[0][2]*A[2][0])*invdet;
    Ainv[1][2] = -(A[0][0]*A[1][2]-A[0][2]*A[1][0])*invdet;
    Ainv[2][0] =  (A[1][0]*A[2][1]-A[1][1]*A[2][0])*invdet;
    Ainv[2][1] = -(A[0][0]*A[2][1]-A[0][1]*A[2][0])*invdet;
    Ainv[2][2] =  (A[0][0]*A[1][1]-A[0][1]*A[1][0])*invdet;
    return 0;
}

void get_ICM_data(void)
{
//    鏉╂瑤琚辨稉顏呮Ц闁劙顥pi閻拷
    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();
    float gx = icm42688_gyro_x * Ang2Rad;
    float gy = icm42688_gyro_y * Ang2Rad;
    float gz = icm42688_gyro_z * Ang2Rad;

    alpha_gx = (gx - prev_gx) * 4000;   // rad/s虏
    alpha_gy = (gy - prev_gy) * 4000;
    alpha_gz = (gz - prev_gz) * 4000;

    prev_gx = gx;
    prev_gy = gy;
    prev_gz = gz;
//    //鏉╂瑤琚辨稉顏呮Ц閸涘棜鎮撴笟鐕烮C閻拷
//    Get_Acc_ICM42688_IIC();
//    Get_Gyro_ICM42688_IIC();
    //閸旂娀锟界喎瀹崇€涙ê甯慨瀣拷锟�
    Pitch_acc =  icm42688_acc_y;
    Roll_acc  =  icm42688_acc_x;
    Yaw_acc   =  icm42688_acc_z;
    icm42688_acc_x_use = 0.992*icm42688_acc_x_use + 0.008*icm42688_acc_x;
    icm42688_acc_y_use = 0.992*icm42688_acc_y_use + 0.008*icm42688_acc_y;
    icm42688_acc_z_use = 0.992*icm42688_acc_z_use + 0.008*icm42688_acc_z;
//    icm42688_acc_x_use = kalman_filter2(icm42688_acc_x);
//    icm42688_acc_y_use = kalman_filter5(icm42688_acc_y);
//    icm42688_acc_z_use = kalman_filter6(icm42688_acc_z);
    acc = sqrtf(icm42688_acc_x*icm42688_acc_x + icm42688_acc_y*icm42688_acc_y + icm42688_acc_z*icm42688_acc_z);
    acc_use = sqrtf(icm42688_acc_x_use*icm42688_acc_x_use + icm42688_acc_y_use*icm42688_acc_y_use + icm42688_acc_z_use*icm42688_acc_z_use);
    //娴滃矂妯佸⿰銈嗗皾鐎电懓濮為柅鐔峰鏉╂稖顢戞穱顕€銈�
    Pitch_acc_F = biquad(&Pitch_acc_biquad,Pitch_acc);
    Roll_acc_F  = biquad(&Roll_acc_biquad,Roll_acc);
    Yaw_acc_F   = biquad(&Yaw_acc_biquad,Yaw_acc);
    //鐟欐帡锟界喎瀹崇€涙ê甯慨瀣拷锟�
    Pitch_g =  icm42688_gyro_y;
    Roll_g  =  icm42688_gyro_x;
    Yaw_g   =  icm42688_gyro_z;
    //娴滃矂妯佸⿰銈嗗皾鐎电顫楅柅鐔峰鏉╂稖顢戞穱顕€銈�
    Pitch_g_F = biquad(&Pitch_g_biquad,Pitch_g);
    Roll_g_F  = biquad(&Roll_g_biquad,Roll_g);
    Yaw_g_F   = biquad(&Yaw_g_biquad,Yaw_g);

    MahonyAHRSupdateIMU(icm42688_gyro_x,icm42688_gyro_y,icm42688_gyro_z,icm42688_acc_x_use,icm42688_acc_y_use,icm42688_acc_z_use);
//    updateMadgwickEulerIMU(icm42688_gyro_x,icm42688_gyro_y,icm42688_gyro_z,icm42688_acc_x_use,icm42688_acc_y_use,icm42688_acc_z_use,0.00025f);
    Pitch_a  = Rad2Ang * Pitch_a_Pi;//鐎圭偞妞傛慨鎸庯拷浣筋潡
    Roll_a   = Rad2Ang * Roll_a_Pi;
    Yaw_a    = Rad2Ang * Yaw_a_Pi;

//    Pitch_b  = Rad2Ang * Roll_a_Pi;//鐎圭偞妞傛慨鎸庯拷浣筋潡
//    Roll_b   = Rad2Ang * Pitch_a_Pi;
//    Yaw_b    = Rad2Ang * Yaw_a_Pi;
//    Pitch = 0.5*Pitch_a+0.5*Pitch_b;
//    Roll = 0.5*Roll_a+0.5*Roll_b;
    Pitch = 0.5*Pitch_a-0.5*Pitch_b;
    Roll = 0.5*Roll_a+0.5*Roll_b;
    Yaw = 0.5*Yaw_a+0.5*Yaw_b;
//    Roll_a  = -Rad2Ang * Pitch_a_Pi;//鐎圭偞妞傛慨鎸庯拷浣筋潡
//    Pitch_a   = Rad2Ang * Roll_a_Pi;
//    Yaw_a    = Rad2Ang * Yaw_a_Pi;

    Yaw_a_Pi_genrl = Yaw_a_Pi-Yaw_A0;

    Yaw_angle = Yaw_a;//閹存垵濮為惃锟�

    Yaw_gyro = Yaw_g_F;//閹存垵濮為惃鍕剁礉閸︹暞ervo閺傚洣娆㈤柌宀€鏁�
}


//閸氬嫯鐭惧⿰銈嗗皾閸ｃ劎娈戦崚婵嗩潗閸栵拷
void Filter_Init(void)
{
    biquad_filter_init(&X_V_biquad, BIQUAD_LOWPASS, 1000, PIDK_AC.Kd, 0.7071);
    biquad_filter_init(&Y_V_biquad, BIQUAD_LOWPASS, 1000, PIDK_BC.Kd, 0.7071);
    biquad_filter_init(&Pitch_g_biquad, BIQUAD_LOWPASS, 4000, 30, 0.7071);
    biquad_filter_init(&Roll_g_biquad, BIQUAD_LOWPASS, 4000, 30, 0.7071);
    biquad_filter_init(&Yaw_g_biquad, BIQUAD_LOWPASS, 4000, 30, 0.7071);
    biquad_filter_init(&Pitch_acc_biquad, BIQUAD_LOWPASS, 4000, 10, 0.7071);
    biquad_filter_init(&Roll_acc_biquad, BIQUAD_LOWPASS, 4000, 10, 0.7071);
    biquad_filter_init(&Yaw_acc_biquad, BIQUAD_LOWPASS, 4000, 10, 0.7071);
}

//娴滃矂妯侀敍鍦攊quad閿涘鎶ゅ▔銏犳珤闁板秶鐤嗙猾璇茬€烽柅澶嬪

// 2nd-order Butterworth: q_value=0.7071閿涘牊娓堕崸鍥у瘧閻ㄥ嫰锟藉瀚ㄩ幀褝绱濋幇蹇撴嚄閻拷閻╅晲缍呮径杈╂埂閺堬拷鐏忓骏绱�
// 2nd-order Chebyshev (ripple 1 dB): q_value=0.9565閿涘牆鍘戠拋绋挎躬閻楃懓鐣炬０鎴犲芳閼煎啫娲块崘鍛箒娑擄拷鐎规氨娈戝▔銏犲З閿涘奔绲炬０鎴濈敨婢舵牜娈戠悰鏉垮櫤闂堢偛鐖惰箛顐狅拷鍌滄暏閸︺劌顕梼璇茬敨鐞涙澘鍣洪張澶夊紬閺嶈壈顩﹀Ч鍌︾礆
// 2nd-order Thomson-Bessel: q_value=0.5773閿涘牐澹囨總鐣屾畱閺冨爼妫块崺鐔锋惙鎼存柨鎷伴惄闀愮秴缁炬寧锟窖冨閿涘奔缍嗗鎯扮箿閸滃矁澹囨總鐣屾祲娴ｅ秴鎼锋惔鏃傛畱缁崵绮烘稉顓濆▏閻㈩煉绱�
// 鐢箓锟芥碍鎶ゅ▔銏犳珤閻ㄥ墑_value = sqrt(f1*f2)/(f2-f1)
//閸氬簼绗佹稉顏勫棘閺佺増妲搁柌鍥ㄧ壉閻滃浄绱欑挧顐㈠悅閿涘锟戒礁甯慨瀣焻濮濄垽顣堕悳鍥风礄鐠ь偄鍚傞敍澶堬拷浣界殶閺佸瓨鎶ゅ▔銏犳珤閸濅浇宸濋崶鐘虫殶閻ㄥ嫬锟斤拷
void biquad_filter_init(biquad_state *state, biquad_type type, int fs, float fc, float q_value)
{
    float w0, sin_w0, cos_w0, alpha;
    float b0, b1, b2, a0, a1, a2;

    w0 = 2 * PI * fc / fs;//妫版垹宸兼潪顒佸床
    sin_w0 = sinf(w0);
    cos_w0 = cosf(w0);
    alpha = sin_w0 / (2.0 * q_value);//鐞涙澘鍣虹化缁樻殶

    switch(type)
    {
    case BIQUAD_LOWPASS:
        b0 = (1.0 - cos_w0) / 2.0;
        b1 = b0 * 2;
        b2 = b0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_HIGHPASS:
        b0 = (1.0 + cos_w0) / 2.0;
        b1 = -b0 * 2;
        b2 = b0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_BANDPASS_PEAK:
        b0 = alpha;
        b1 = 0.0;
        b2 = -alpha;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    case BIQUAD_BANDSTOP_NOTCH:
        b0 = 1.0;
        b1 = -2.0 * cos_w0;
        b2 = 1.0;
        a0 = 1.0 + alpha;
        a1 = -2.0 * cos_w0;
        a2 = 1.0 - alpha;
        break;
    }
    state->a0 = b0 / a0;
    state->a1 = b1 / a0;
    state->a2 = b2 / a0;
    state->a3 = a1 / a0;
    state->a4 = a2 / a0;
    state->x1 = state->x2 = 0.0;
    state->y1 = state->y2 = 0.0;
}

//娴滃矂妯佸⿰銈嗗皾
float biquad(biquad_state *state, float data)
{
    float result = 0;
    result = state->a0 * data + state->a1 * state->x1 + state->a2 * state->x2 -  state->a3 * state->y1 - state->a4 * state->y2;
    state->x2 = state->x1;
    state->x1 = data;
    state->y2 = state->y1;
    state->y1 = result;
    return result;
}

//閺佺増宓侀惃鍕吀缁狅拷
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    // 鐎规矮绠熼棃娆愶拷浣稿綁闁插骏绱濋悽銊ょ艾鐎涙ê鍋嶇拋锛勭暬缂佹挻鐏夐崪宀€濮搁幀浣碉拷鍌炴饯閹礁褰夐柌蹇撴躬閸戣姤鏆熷В蹇旑偧鐠嬪啰鏁ら弮鏈电窗娣囨繃瀵旈崗璺猴拷锟�
    static float recipNorm;
    static float halfvx, halfvy, halfvz;
    static float halfex, halfey, halfez;//閼割亜鎮滅憴鎺楋拷鐔峰
    static float qa, qb, qc;
    //閸掋倖鏌囬惃鍕暏闁棑绱濊ぐ鎾圭箥閸斻劌褰傞悽鐔稿鏉╂稖顢戠拋锛勭暬
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        //鐟欐帡锟界喎瀹虫潪顒佸床閹存劕濮惔锔肩礉閸楁洑缍呮稉绨塧d/s
        gx*=0.0174532925f;
        gy*=0.0174532925f;
        gz*=0.0174532925f;

        // 瑜版帊绔撮崠鏍у闁喎瀹崇拋鈩冪ゴ闁插繐锟斤拷
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 娴兼媽顓搁柌宥呭閺傜懓鎮滈崪灞界€惄缈犵艾绾句線锟芥岸鍣洪惃鍕厺闁诧拷
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 鐠囶垰妯婇弰顖欏強鐠侊繝鍣搁崝娑欐煙閸氭垳绗屽ù瀣櫤闁插秴濮忛弬鐟版倻閻ㄥ嫪绠荤粔顖欑閸滐拷
        // 鐠侊紕鐣诲▎褎濯虹憴鎺嶅強鐠佲€茬瑢鐎圭偤妾ù瀣櫤閻ㄥ嫪姘﹂崣澶屝濋敍灞肩稊娑撻缚顕ゅ顕€銆�
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 婵″倹鐏夐崥顖滄暏閿涘矁顓哥粻妤€鑻熸惔鏃傛暏缁夘垰鍨庨崣宥夘洯
        if(twoKi > 0.0f)
        {
            integralFBx += twoKi * halfex * invsampleFreq;  // 缁夘垰鍨庣拠顖氭▕閹稿i缂傗晜鏂�
            integralFBy += twoKi * halfey * invsampleFreq;
            integralFBz += twoKi * halfez * invsampleFreq;
            gx += integralFBx;  // 鎼存梻鏁ょ粔顖氬瀻閸欏秹顩�
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // 闂冨弶顒涚粔顖氬瀻妤楀崬鎷�
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }
//        if(fabs(acc_use-1009.0f) < 15) twoKp = 1.6f;
//        else if(fabs(acc_use-1009.0f) >= 15 && fabs(acc_use-1009.0f) < 40) twoKp = 0.2f;
//        else twoKp = 0.0f;
        if(fabs(acc-1008.0f) < 15) twoKp = 1.6f;
        else if(fabs(acc-1008.0f) >= 15 && fabs(acc-1008.0f) < 40) twoKp = 0.2f;
        else twoKp = 0.0f;
        // 鎼存梻鏁閸欏秹顩�
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;

    }
    // 鐠侊紕鐣婚崶娑樺帗閺佹壆娈戦弮鍫曟？缁夘垰鍨庨敍灞炬闂傛潙顤冮柌蹇旀Ц闁俺绻冩稊妯逛簰闁插洦鐗辨０鎴犲芳閻ㄥ嫪绔撮崡濠冩降鐠侊紕鐣婚惃锟�
    gx *= (0.5f * invsampleFreq);       // 妫板嫪绠荤敮鍝ユ暏閸ョ姴鐡�
    gy *= (0.5f * invsampleFreq);
    gz *= (0.5f * invsampleFreq);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 鐠侊紕鐣婚崶娑樺帗閺佹壆娈戝Ο锟犳毐閺夈儱缍婃稉锟介崠鏍ф磽閸忓啯鏆熼敍鍫⑩€樻穱婵嗗従濠娐ゅ喕閸楁洑缍呴崶娑樺帗閺佺増娼禒璁圭礆
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q_imu.w = q0;
    q_imu.x = q1;
    q_imu.y = q2;
    q_imu.z = q3;

    static float r11,r12,r21,r31,r32;
    // 鐠侊紕鐣婚弮瀣祮閻晠妯€閻ㄥ嫬鍘撶槐锟�
    r11 = 2.0f * (q0*q1 + q2*q3);
    r12 = 1.0f - 2.0f * (q1*q1 + q2*q2);
    r21 = 2.0f * (q0*q2 - q3*q1);
    r31 = 2.0f * (q0*q3 + q1*q2);
    r32 = 1.0f - 2.0f * (q2*q2 + q3*q3);

    // 鐠侊紕鐣诲▎褎濯虹憴锟�(瀵冨)
    Yaw_a_Pi = atan2f(r31, r32);
    Pitch_a_Pi = -asinf(r21);
    Roll_a_Pi = atan2f(r11, r12);
}

/*
 * updateMadgwickEulerIMU:
 *  娴ｈ法鏁ら梽锟介摶杞板崕閸滃苯濮為柅鐔峰鐠佲剝鏆熼幑顔芥纯閺傛澘娲撻崗鍐╂殶閿涘苯鍟€鐠侊紕鐣诲▎褎濯虹憴锟�
 *
 * 閸欏倹鏆熼敍锟�
 *   gx, gy, gz - 闂勶拷閾昏桨鍗庨弫鐗堝祦閿涘牆宕熸担宥忕窗rad/s閿涳拷
 *   ax, ay, az - 閸旂娀锟界喎瀹崇拋鈩冩殶閹诡噯绱欏楦款唴閸忓牆缍婃稉锟介崠鏍ㄥ灗娣囨繆鐦夐崷銊╁櫢閸旀稑濮為柅鐔峰闂勫嫯绻庨敍锟�
 *   dt         - 闁插洦鐗遍崨銊︽埂閿涘牏顫楅敍锟�
 */
void updateMadgwickEulerIMU(float gx, float gy, float gz,
                            float ax, float ay, float az, float dt) {
    // 鐏忓棝妾ч摶杞板崕閺佺増宓佹禒搴″/缁夋帟娴嗛幑顫礋瀵冨/缁夛拷
    gx *= Ang2Rad;
    gy *= Ang2Rad;
    gz *= Ang2Rad;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;


    // 1. 閺嶈宓侀梽锟介摶杞板崕閺佺増宓佺拋锛勭暬閸ユ稑鍘撻弫鎵畱鐎靛吋鏆熼敍鍫濆礋娴ｅ稄绱皉ad/s閿涳拷
    qDot0 = 0.5f * (-q11 * gx - q21 * gy - q31 * gz);
    qDot1 = 0.5f * ( q01 * gx + q21 * gz - q31 * gy);
    qDot2 = 0.5f * ( q01 * gy - q11 * gz + q31 * gx);
    qDot3 = 0.5f * ( q01 * gz + q11 * gy - q21 * gx);

    // 2. 婵″倹鐏夐崝鐘伙拷鐔峰鐠佲剝鏆熼幑顔芥箒閺佸牞绱濋崚娆忓焺閻€劍顫惔锔跨瑓闂勫秵纭堕崑姘冀妫ｅ牅鎱ㄥ锟�
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 瑜版帊绔撮崠鏍у闁喎瀹崇拋鈩冩殶閹癸拷
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 妫板嫯顓哥粻妞捐厬闂傛潙褰夐柌蹇ョ礉閸戝繐鐨柌宥咁槻鐠侊紕鐣�
        _2q0 = 2.0f * q01;
        _2q1 = 2.0f * q11;
        _2q2 = 2.0f * q21;
        _2q3 = 2.0f * q31;
        _4q0 = 4.0f * q01;
        _4q1 = 4.0f * q11;
        _4q2 = 4.0f * q21;
        _8q1 = 8.0f * q11;
        _8q2 = 8.0f * q21;
        q0q0 = q01 * q01;
        q1q1 = q11 * q11;
        q2q2 = q21 * q21;
        q3q3 = q31 * q31;

        // 鐠侊紕鐣诲顖氬娑撳妾锋穱顔筋劀妞わ拷
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q11 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q21 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q31 - _2q1 * ax + 4.0f * q2q2 * q31 - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 鐏忓棗寮芥＃鍫滄叏濮濓絽褰旈崝鐘插煂閸ユ稑鍘撻弫鏉款嚤閺侀鑵�
        qDot0 -= beta * s0;
        qDot1 -= beta * s1;
        qDot2 -= beta * s2;
        qDot3 -= beta * s3;
    }

    // 3. 缁夘垰鍨庡妤€鍩岄弬鎵畱閸ユ稑鍘撻弫锟�
    q01 += qDot0 * dt;
    q11 += qDot1 * dt;
    q21 += qDot2 * dt;
    q31 += qDot3 * dt;

    // 4. 瑜版帊绔撮崠鏍ф磽閸忓啯鏆熼敍宀€鈥樻穱婵嚹佹稉锟�1
    recipNorm = invSqrt(q01 * q01 + q11 * q11 + q21 * q21 + q31 * q31);
    q01 *= recipNorm;
    q11 *= recipNorm;
    q21 *= recipNorm;
    q31 *= recipNorm;

    // 5. 鐏忓棗娲撻崗鍐╂殶鏉烆剚宕叉稉鐑橆儌閹峰顫楅敍鍧畂ll閵嗕垢itch閵嗕簜aw閿涘绱濋崡鏇氱秴閿涙艾濮惔锟�
    Roll_b  = atan2f(2.0f * (q01 * q11 + q21 * q31),
                    1.0f - 2.0f * (q11 * q11 + q21 * q21));
    Pitch_b = asinf(2.0f * (q01 * q21 - q31 * q11));
    Yaw_b   = atan2f(2.0f * (q01 * q31 + q11 * q21),
                    1.0f - 2.0f * (q21 * q21 + q31 * q31));
    // 5. 鐏忓棗娲撻崗鍐╂殶鏉烆剚宕叉稉鐑橆儌閹峰顫楅敍鍧畂ll閵嗕垢itch閵嗕簜aw閿涘绱濋崡鏇氱秴閿涙艾濮惔锟�
//    Pitch_b  = atan2f(2.0f * (q01 * q11 + q21 * q31),
//                   1.0f - 2.0f * (q11 * q11 + q21 * q21));
//    Roll_b = asinf(2.0f * (q01 * q21 - q31 * q11));
//   Yaw_b   = atan2f(2.0f * (q01 * q31 + q11 * q21),
//                   1.0f - 2.0f * (q21 * q21 + q31 * q31));
    // 鏉烆剚宕叉稉楦款潡鎼达箑鍩�
    Roll_b  *= Rad2Ang;
    Pitch_b *= Rad2Ang;
    Yaw_b   *= Rad2Ang;

//    Roll_b_use = 0.01*Roll_b + 0.99*Roll_b_use;
//    Pitch_b_use = 0.01*Pitch_b + 0.99*Pitch_b_use;
//    Yaw_b_use = 0.01*Yaw_b + 0.99*Yaw_b_use;
}

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

///**
// * @brief IMU閺嶁€冲櫙閸戣姤鏆�
// *
// * 鐠囥儱鍤遍弫鎵暏娴滃骸顕甀MU閿涘牊鍎婚幀褎绁撮柌蹇撳礋閸忓喛绱氭潻娑滎攽閺嶁€冲櫙閵嗗倹鐗庨崙鍡氱箖缁嬪鑵戦敍宀€閮寸紒鐔剁窗閹绘劗銇氶悽銊﹀煕娣囨繃瀵旂拋鎯ь槵闂堟瑦顒涢敍锟�
// * 楠炶泛婀幐鍥х暰閺冨爼妫块崘鍛村櫚闂嗗棝妾ч摶杞板崕閺佺増宓侀妴鍌炲櫚闂嗗棗鍩岄惃鍕殶閹诡喚鏁ゆ禍搴ゎ吀缁犳妾ч摶杞板崕閻ㄥ嫬浜哥粔缁樼墡濮濓絽锟界锟斤拷
// *
// * 閺嶁€冲櫙濮濄儵顎冮敍锟�
// * 1. 閺勫墽銇氶弽鈥冲櫙閹绘劗銇氭穱鈩冧紖閿涘本褰佺粈铏规暏閹磋渹绻氶幐浣筋啎婢跺洭娼ゅ鈧拷锟�
// * 2. 閸︺劍瀵氱€规碍妞傞梻鏉戝敶閿涘湜MU_calibration_seconds閿涘绱濇禒锟�4KHz閻ㄥ嫰鍣伴弽椋庡芳闁插洭娉﹂梽锟介摶杞板崕閺佺増宓侀妴锟�
// * 3. 鐠侊紕鐣诲В蹇曨潡闁界喓娈戦梽锟介摶杞板崕閺佺増宓侀獮鍐叉綆閸婄》绱濋獮璺虹摠閸屻劌婀猤yro_history閺佹壆绮嶆稉顓滐拷锟�
// * 4. 鐠侊紕鐣婚弫缈犻嚋閺嶁€冲櫙閺冨爼妫块崘鍛畱闂勶拷閾昏桨鍗庨弫鐗堝祦楠炲啿娼庨崐纭风礉楠炶埖鐗撮幑顔款嚉楠炲啿娼庨崐鑹邦吀缁犳妾ч摶杞板崕閻ㄥ嫬浜哥粔缁樼墡濮濓絽锟界锟斤拷
// * 5. 閺勫墽銇氶弽鈥冲櫙鐎瑰本鍨氶幓鎰仛娣団剝浼呴妴锟�
// */
#define IMU_calibration_seconds 3
void IMU_calibration(void){

    uint32 s_cnt=0,g_cnt=0;

    float gyro_history[3][IMU_calibration_seconds];
    sint32 gyro_history_sum[3]={0,0,0};
    //閺嶁€冲櫙閹绘劗銇�
//    ips200_show_string(0,16*0,"IMU calibration");
//    ips200_show_string(0,16*2,"Don't move me!(2)");
    system_delay_ms(1000);
//    ips200_show_string(0,16*2,"Don't move me!(1)");
    system_delay_ms(1000);
    do{
        system_delay_us(250);
        Get_RAW_Gyro_ICM42688();
        if(g_cnt<4000){
            gyro_history_sum[0] += icm42688_gyro_x_r;
            gyro_history_sum[1] += icm42688_gyro_y_r;
            gyro_history_sum[2] += icm42688_gyro_z_r;
            g_cnt++;
        }
        else{
            gyro_history[0][s_cnt]=(float)gyro_history_sum[0]/4000.0f;
            gyro_history[1][s_cnt]=(float)gyro_history_sum[1]/4000.0f;
            gyro_history[2][s_cnt]=(float)gyro_history_sum[2]/4000.0f;
            gyro_history_sum[0]=0;
            gyro_history_sum[1]=0;
            gyro_history_sum[2]=0;
            g_cnt=0;
            s_cnt++;
        }
    }while(s_cnt<IMU_calibration_seconds);


    float sum[3]={0,0,0},k=icm42688_gyro_inv/IMU_calibration_seconds;
    for(uint8 i=0;i<IMU_calibration_seconds;i++){
        sum[0] += gyro_history[0][i];
        sum[1] += gyro_history[1][i];
        sum[2] += gyro_history[2][i];
    }
    gyro_x_correction = k*sum[0];
    gyro_y_correction = k*sum[1];
    gyro_z_correction = k*sum[2];
//    ips200_show_string(0,16*2,"IMU Calibration OK      ");

}




