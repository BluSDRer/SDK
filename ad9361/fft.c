#include "math.h"
#include "fft.h"
#include "xil_io.h"
#include "uart.h"
#include "command.h"

complex XX[SIZE_OF_FFT]={0};
void conjugate_complex(int n,complex in[],complex out[])
{
  int i = 0;
  for(i=0;i<n;i++)
  {
    out[i].imag = -in[i].imag;
    out[i].real = in[i].real;
  } 
}
 
void c_abs(complex f[],float out[],int n)
{
  int i = 0;
  float t;
  for(i=0;i<n;i++)
  {
    t = f[i].real * f[i].real + f[i].imag * f[i].imag;
    out[i] = sqrt(t);
  } 
}
 
 
void c_plus(complex a,complex b,complex *c)
{
  c->real = a.real + b.real;
  c->imag = a.imag + b.imag;
}
 
void c_sub(complex a,complex b,complex *c)
{
  c->real = a.real - b.real;
  c->imag = a.imag - b.imag; 
}
 
void c_mul(complex a,complex b,complex *c)
{
  c->real = a.real * b.real - a.imag * b.imag;
  c->imag = a.real * b.imag + a.imag * b.real; 
}
 
void c_div(complex a,complex b,complex *c)
{
  c->real = (a.real * b.real + a.imag * b.imag)/(b.real * b.real +b.imag * b.imag);
  c->imag = (a.imag * b.real - a.real * b.imag)/(b.real * b.real +b.imag * b.imag);
}
 
#define SWAP(a,b)  tempr=(a);(a)=(b);(b)=tempr
 
void Wn_i(int n,int i,complex *Wn,char flag)
{
  Wn->real = cos(2*PI*i/n);
  if(flag == 1)
  Wn->imag = -sin(2*PI*i/n);
  else if(flag == 0)
  Wn->imag = -sin(2*PI*i/n);
}
 
//閸屽懘鍣烽崣璺哄綁閸栵拷
void fft(int N,complex f[])
{
  complex t,wn;//娑擃參妫块崣姗�鍣�
  int i,j,k,m,n,l,r,M;
  int la,lb,lc;
  /*----鐠侊紕鐣婚崚鍡毿掗惃鍕獓閺佺檿=log2(N)----*/
  for(i=N,M=1;(i=i/2)!=1;M++); 
  /*----閹稿鍙庨崐鎺嶇秴鎼村繘鍣搁弬鐗堝笓閸掓甯穱鈥冲娇----*/
  for(i=1,j=N/2;i<=N-2;i++)
  {
    if(i<j)
    {
      t=f[j];
      f[j]=f[i];
      f[i]=t;
    }
    k=N/2;
    while(k<=j)
    {
      j=j-k;
      k=k/2;
    }
    j=j+k;
  }
 
  /*----FFT缁犳纭�----*/
  for(m=1;m<=M;m++)
  {
    la=pow(2,m); //la=2^m娴狅綀銆冪粭鐞虹痪褎鐦℃稉顏勫瀻缂佸嫭澧嶉崥顐ュΝ閻愯鏆�
    lb=la/2;    //lb娴狅綀銆冪粭鐞虹痪褎鐦℃稉顏勫瀻缂佸嫭澧嶉崥顐ゎ杽瑜般垹宕熼崗鍐╂殶
                 //閸氬本妞傜�瑰啩绡冪悰銊с仛濮ｅ繋閲滅喊鐔疯埌閸楁洖鍘撴稉濠佺瑓閼哄倻鍋ｆ稊瀣？閻ㄥ嫯绐涚粋锟�
    /*----绾扮喎鑸版潻鎰暬----*/
    for(l=1;l<=lb;l++)
    {
      r=(l-1)*pow(2,M-m); 
      for(n=l-1;n<N-1;n=n+la) //闁秴宸诲В蹇庨嚋閸掑棛绮嶉敍灞藉瀻缂佸嫭锟界粯鏆熸稉绡�/la
      {
        lc=n+lb;  //n,lc閸掑棗鍩嗘禒锝堛�冩稉锟芥稉顏嗩杽瑜般垹宕熼崗鍐畱娑撳锟戒椒绗呴懞鍌滃仯缂傛牕褰�
        Wn_i(N,r,&wn,1);//wn=Wnr
        c_mul(f[lc],wn,&t);//t = f[lc] * wn婢跺秵鏆熸潻鎰暬
        c_sub(f[n],t,&(f[lc]));//f[lc] = f[n] - f[lc] * Wnr
        c_plus(f[n],t,&(f[n]));//f[n] = f[n] + f[lc] * Wnr
      }
    }
  }
}
 
//閸屽懘鍣烽崣鍫曪拷鍡楀綁閹癸拷
void ifft(int N,complex f[])
{
  int i=0;
  conjugate_complex(N,f,f);
  fft(N,f);
  conjugate_complex(N,f,f);
  for(i=0;i<N;i++)
  {
    f[i].imag = (f[i].imag)/N;
    f[i].real = (f[i].real)/N;
  }
}

fft_result fft_calculate(unsigned int length,unsigned int *bufi,unsigned int *bufq)
{
    double max=0;
    unsigned int index=0;
	fft_result amp_ph;
	double xx_c=0;
	double am;
	double pm;
	unsigned int  i=0;
	for(i=0;i<SIZE_OF_FFT;i++)
	{
		if(bufi[i] >= 2048)
		{
			bufi[i] = bufi[i] - 4096;
		}
		if(bufq[i] >= 2048)
		{
			bufq[i] = bufq[i] - 4096;
		}
		XX[i].real = bufi[i];
		XX[i].imag = bufq[i];
	}
   fft(length,XX);
   for(i=1;i<SIZE_OF_FFT;i++)  //璁＄畻鍙樻崲缁撴灉鐨勬ā闀�
   {
	   am =sqrt(XX[i].imag*XX[i].imag+XX[i].real*XX[i].real)/16384/16384/50;
	   if(am > max)
	   {
		   max = am;
		   index = i;
		  // printf("i= %d max = %f \n",i,am);
	   }
	}
    i=index;
	am =sqrt(XX[i].imag*XX[i].imag+XX[i].real*XX[i].real)/16384/16384/50;
	//xx_c = sqrt(XX[i].imag*XX[i].imag+XX[i].real*XX[i].real);
	//pm = acos(XX[i].real/xx_c)*180/3.1415926;
	//if(XX[i].real<0)
	//{
	//	pm = pm +180;
	//}
	pm = atan2(XX[i].imag,XX[i].real)*180/PI;
	printf("i = %d  ",i);
	printf("am = %lf  ",am);
	printf("pm = %lf \n",pm);
	amp_ph.amplitude = am;
	amp_ph.phase = pm;

   return amp_ph;
}
