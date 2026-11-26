// fuzzy_pid.h  -  Fuzzy gain scheduling cho Kp, Ki, Kd (robot tu can bang)
//
// Dau vao  : e  = sai so goc (do)          -> chuan hoa theo FZ_E_MAX
//            de = toc do nghieng (do/giay) -> chuan hoa theo FZ_DE_MAX
// Dau ra   : dKp, dKi, dKd trong khoang -1..+1 (nhan voi "span" de ra do lech that)
//
// 5 tap mo cho moi dau vao: NB (am lon), NS (am nho), ZO (khong), PS (duong nho), PB (duong lon)
// 25 luat cho moi gain, khu mo bang trung binh co trong so (Sugeno).
//
// Y tuong chinh:
//  - Kp: tang khi sai so lon va dang lech xa them, giam nhe khi dang quay ve, GIU NGUYEN khi dung gan can bang
//        (motor co vung chet o PWM thap, giam Kp o giua se lam robot phan ung cham hon)
//  - Kd: tang khi robot dang dao dong nhanh gan can bang (can nhieu giam chan), giam khi dang nghieng rat xa
//  - Ki: chi tang khi sai so nho va robot gan nhu dung yen (loai bo lech tinh), giam khi sai so lon (chong windup)

#pragma once

const float FZ_E_MAX  = 10.0;    // do: |error| >= gia tri nay -> coi la "rat lon"
const float FZ_DE_MAX = 150.0;   // do/giay: |gyro| >= gia tri nay -> coi la "rat nhanh"

// Hang = sai so e (NB, NS, ZO, PS, PB); cot = toc do de (NB, NS, ZO, PS, PB). Gia tri -2..2.
static const int8_t FZ_RULE_KP[5][5] = {
  {  2,  2,  2,  1,  1 },
  {  1,  1,  0, -1, -1 },
  {  0,  0,  0,  0,  0 },
  { -1, -1,  0,  1,  1 },
  {  1,  1,  2,  2,  2 }
};
static const int8_t FZ_RULE_KI[5][5] = {
  { -2, -2, -2, -2, -2 },
  { -1,  0,  1,  0, -1 },
  {  0,  1,  2,  1,  0 },
  { -1,  0,  1,  0, -1 },
  { -2, -2, -2, -2, -2 }
};
static const int8_t FZ_RULE_KD[5][5] = {
  {  0, -1, -1, -1,  0 },
  {  2,  1,  0,  1,  2 },
  {  2,  1, -1,  1,  2 },
  {  2,  1,  0,  1,  2 },
  {  0, -1, -1, -1,  0 }
};

static inline float fzClamp1(float x) {
  return x < -1.0f ? -1.0f : (x > 1.0f ? 1.0f : x);
}

// Tap mo tam giac, tam tai -1, -0.5, 0, 0.5, 1 (tong cac do thuoc luon = 1)
static inline void fzFuzzify(float x, float mu[5]) {
  for (int i = 0; i < 5; i++) {
    float c = -1.0f + 0.5f * (float)i;
    float m = 1.0f - fabsf(x - c) / 0.5f;
    mu[i] = m > 0.0f ? m : 0.0f;
  }
}

static inline void fuzzyCompute(float e, float de, float &dKp, float &dKi, float &dKd) {
  float mE[5], mD[5];
  fzFuzzify(fzClamp1(e  / FZ_E_MAX),  mE);
  fzFuzzify(fzClamp1(de / FZ_DE_MAX), mD);

  float sw = 0, sKp = 0, sKi = 0, sKd = 0;
  for (int i = 0; i < 5; i++) {
    if (mE[i] <= 0.0f) continue;
    for (int j = 0; j < 5; j++) {
      float w = mE[i] * mD[j];            // luat AND = tich
      if (w <= 0.0f) continue;
      sw  += w;
      sKp += w * FZ_RULE_KP[i][j];
      sKi += w * FZ_RULE_KI[i][j];
      sKd += w * FZ_RULE_KD[i][j];
    }
  }
  if (sw < 1e-6f) { dKp = dKi = dKd = 0; return; }
  dKp = sKp / sw / 2.0f;                  // -2..2 -> -1..1
  dKi = sKi / sw / 2.0f;
  dKd = sKd / sw / 2.0f;
}