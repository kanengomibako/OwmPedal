#include "main.h"

extern uint16_t pot[];
extern uint8_t sw[];

/* �G�t�F�N�g�I�� ���������邱�Ƃ��ł��� ------------------------------------------------*/
static float fx(float x)
{
  x = overdrive(x);
  return x;
}

/* �o�C�p�X�@�|�b�v�m�C�Y�΍�̂��߁A0.01�����ʑ��삵�G�t�F�N�g�؂�ւ�����--------------------*/
float bypass(float x)
{
  static uint16_t count = 0;
  if ( sw[3] != 0 ) // �G�t�F�N�gON
  {
    if (count < 100) // �o�C�p�X���ʃ_�E��
    {
      x = (1.0f - (float) count * 0.01f) * x;
      count++;
    }
    else if (count < 200) // �G�t�F�N�g���ʃA�b�v
    {
      x = ((float) count * 0.01f - 1.0f) * fx(x);
      count++;
    }
    else // count�I�� (count: 200)
    {
      x = fx(x);
    }
  }
  else // �G�t�F�N�gOFF
  {
    if (count > 100) // �G�t�F�N�g���ʃ_�E��
    {
      x = ((float) count * 0.01f - 1.0f) * fx(x);
      count--;
    }
    else if ( count > 0) // �o�C�p�X���ʃA�b�v
    {
      x = (1.0f - (float) count * 0.01f) * x;
      count--;
    }
  }
  return x;
}

/* 1��LPF�AHPF�p�̌W���v�Z fs=48000�A50�`10kHz�ł̋ߎ��Ȑ� ----------------------------*/
float lpfCoef(float fc)
{
  return 0.000000005146361f * fc * fc - 0.0001185165f * fc + 0.9943665f;
}
