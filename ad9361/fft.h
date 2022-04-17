#ifndef __FFT_H__
#define __FFT_H__
 
typedef struct complex //婢跺秵鏆熺猾璇茬��
{
  float real;  //鐎圭偤鍎�
  float imag;  //閾忔岸鍎�
}complex;

typedef struct{       //瀹氫箟涓�涓粨鏋勪綋琛ㄧず澶嶆暟鐨勭被鍨�
	double amplitude;
	double phase;
}fft_result;

#define SIZE_OF_FFT 16384
#define PI 3.1415926535897932384626433832795028841971
 
 
///////////////////////////////////////////
void conjugate_complex(int n,complex in[],complex out[]);
void c_plus(complex a,complex b,complex *c);//婢跺秵鏆熼崝锟�
void c_mul(complex a,complex b,complex *c) ;//婢跺秵鏆熸稊锟�
void c_sub(complex a,complex b,complex *c); //婢跺秵鏆熼崙蹇旂《
void c_div(complex a,complex b,complex *c); //婢跺秵鏆熼梽銈嗙《
void fft(int N,complex f[]);//閸屽懐鐝涢崣璺哄綁閹癸拷 鏉堟挸鍤稊鐔风摠閸︺劍鏆熺紒鍒︽稉锟�
void ifft(int N,complex f[]); // 閸屽懘鍣烽崣鍫曪拷鍡楀綁閹癸拷
void c_abs(complex f[],float out[],int n);//婢跺秵鏆熼弫鎵矋閸欐牗膩
fft_result fft_calculate(unsigned int length,unsigned int *bufi,unsigned int *bufq);
////////////////////////////////////////////
#endif

