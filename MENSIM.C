
#define int8u      unsigned char
#define int16u     unsigned int
#define int32u	   unsigned long
#define int8s      signed   char
#define int16s	   signed   int
#define int32s     signed   long

#include <stdio.h>
#include <conio.h>
#include "process.h"
#include "dos.h"
#include <stdlib.h>
#include <string.h>

union REGS inregs , outregs ;
int16u x7seg , y7seg ;
int8u  oldmode ;
int8u  adcidx , adcgen,month ;

#define CHKSUM  0xAA

#define ENT		1
#define ESC		2
#define INC		3
#define DEC		4
#define SHIFT  	5
#define QUIT	6
#define NONE    10

#define NOEVENT			0
#define TCBREAK			1
#define TCSHORT			2
#define TCDIPPED		3
#define TCDETECTED		4
#define CYCLECOMPLETE	5
#define LATCH			6
#define NOLATCH			7
#define TCNOTDETECTED	8

#define staaddr 8100
#define noaddr  8102

#define OPN 	0
#define SHORT 	1
#define MSR 	2
#define SPN     3

#define TMR	0
#define TMP	1
#define TC 	2
#define HTN	3
#define RTC	4
#define PRN	5
#define CAL	6
#define MEM	7
#define BAT	8
#define EXT	9

#define R	0
#define S	1
#define B	2

#define N	0
#define Y	1

#define SPAN	0
#define ZERO	1

#define OPEN		0
#define READY		1
#define MEASURE		2
#define COMPLETE	3
#define ERROR		4

#define TC_LIMIT   	43274
#define	UP_LIM		53869/*A1768*/
#define LOW_LIM		43274/*A1000   */
#define TIMERVAL	15/*0*/

#define date_low	1
#define date_up     31
#define year_low	14
#define year_up		99
#define min_up		60
#define min_low		0
#define hour_low	0
#define hour_up		24
#define month_low	1
#define month_up	12

#define latch_low	5
#define latch_up	50
#define hold_low	3
#define hold_up		20
#define lad_up      99
#define lad_low     0
#define htn_up      1999
#define htn_low     0
#define SEC8		80

#define A1768       53869
#define Z0000       32768

#define ANM           0
#define AMR           1
#define AMW           2
#define MASTER_ACK    3
#define MASTER_NACK   4
#define START         5
#define STOP          6
#define RESTART	      7
#define ADDRH	      8
#define ADDRL	      9
#define mem_DATA      10
#define WORD_ADDR     11
#define rtc_DATA      12
#define tmp_DATA      13
#define COMMAND	      14
#define adc_DATA      15

/*      Slave addr      */
#define rtc_write_addr 	0xa2
#define rtc_read_addr 	0xa3
#define mem_write_addr 	0xa0
#define mem_read_addr 	0xa1
#define adc_write_addr 	0x20
#define adc_read_addr 	0x21
#define tmp_write_addr 	0x90
#define tmp_read_addr 	0x91

#define rtcaddrmax 	9
#define rtcaddrmin 	2

#define yr_addr 	8
#define month_addr 	7
#define day_addr 	5
#define hour_addr 	4
#define min_addr 	3
#define sec_addr 	2

#define NACK 1
#define ACK  0

void dispnum( int16u num);
void dispstr( int8u *str );
void i2c_slave_simulate(void);

int8u bcdtobin(int8u bcdval);
int8u bintobcd(int8u intval);

int8u adc_cur_state, i2cbusy;
/*  memory simulation  */

int8s gudbad[4]= {0};
int8u chk;

typedef struct
{
	int16u	heatno;
	int8u 	ladelno;
	int8u	latchtime;
	int8u	holdtime;
	int8u	sttpm;
	int16u	zrcnt;
	int16u	spcnt;
	int16u	gap1;
	int16u  gap2;
	int8u	tctype;
	int8u	chksum;
}param;

param pp[4];
const param dpp = { 1 , 1 , 5 , 15 , 1 , 32768 , 51617 , 0 , 0 , 1 , 0x0} ;

int16u noread,staddr;


typedef struct
{	int16u htno ;
	int16u rdate ;
	int16u rtime ;
	int16u temp ;
	int8u  ladno;
}logdata;

logdata reading = { 1, 2 ,4, 4 ,4};

 void i2csendbyte( int8u byte ) ;
 void i2csendstart( void ) ;
 void i2csendstop( void ) ;
 int16u getadc( void );

/***  ***  ***  ***  ***  ***  ***  ***  ***   ***  ***  ***  ***  ***  ***/
int8u i2cdata,mem_ACK,rtc_ACK;
int8u *p;
int8u bcdhours,bcdmins,bcdsecs,bcdyear,bcdmonth,bcdday;
int8u dmin,dhr,dyr,dmon,dday;
int8u w_month_flag,w_yr_flag,w_day_flag;
int8u addrl,addrh,s,resp,rtc_data,mem_data;
int8u wordaddr,w_hr_flag;
int8u w_min_flag,w_sec_flag,data,temp_ACK,temp_addr,cmd_addr,res_command;
int8u adc_addr,adc_ACK,configaddr,mem_currentstate,mem_currentevent;
int8u var_start,var_stop,VAL_Master_ACK = 0,VAL_NACK,i2cresp;
int8u rtc_currentevent,adc_currentevent,tmp_currentevent;
int8u rtc_currentstate,adc_currentstate,tmp_currentstate;
int8u temp_data = 0x19,getadc_flag,i2cbyte;
int16u mem_addr,recsize, mem_i2cdata_wrd , mem_i2cdata_byte,i2cword ;
FILE *fp ;

int32u rdsum;
int8u idxadc_data,count_complete_flag,cmplt_flag,totaltime;
int16u latch_data[latch_up]={0};
int16u CONVCOM ;

int8u hrs,mins,secs,yr,day,mnth,initflag;
int8u idx,newread,fifo_ful,latchtime,delay_time;
int8u sens_currentstate,sens_currentevent,oldchoice,oldnum,min,menuflag;
int16u latchmax,latchmin,latch_diff;
int8u noisemargin,quitkey,send_data_flag,runflag;
int8u holdtime,select,start_temp,tc_type,date,hour,adc10pos;
int16u delaycnt,adc_data,adccnt,spncnt,zrocnt,adc10cnt,bat_state,calsample,caladcidx;
int8u year,raddrl,raddrh,mem_i2cdata;
int8u ii,hh,dd,mm,yy,bytecnt;
int16u adc_read_data, i2cdata16;

int8u printstr[80];
int lad_no,htn_no;
int8u dispbuf[100] = {0};

int16u	   adcar[] =
{
	 32768 ,32768 , 32768 ,32768 , 32768 ,43123 ,44234 , 44345 ,
	 44335 ,44345 , 44345 ,44349 , 44350 ,44350 ,44345 , 44345 ,
	 44345 ,44345 , 44345 ,44351 , 44345 ,
	 44355 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,44345 , 44445 ,44345 , 44445 ,44345 ,
	 44445 ,44345 , 44445 ,
	 0,44350 , 44350 , 44350 , 44350 , 0,32768 ,44350 ,44390,
	 44350 , 52990 , 52990 ,      0, 44350 ,0, 44350 , 44350 , 44350 ,
	 43123 , 44234 , 44345 , 44349 , 44350 , 0,
	 44350 , 44350 , 44350 , 44350 , 44350 , 0,
	 44350 , 44350 , 44350 , 44350 , 44350 ,
	 43123 , 44234 , 44345 , 44349 , 44350 ,
	 44350 , 44350 , 44350 , 44350 , 44350 ,
	 47397 , 47397 , 47397 , 47397 , 47397 ,47397 , 47397 , 47397 , 47397 , 47397 ,47397 , 47397 , 47397 , 47397 , 47397 ,
	 47397 , 47397 , 0,47397 , 47397 , 47397 ,47397 , 47397 , 47397 , 47397 , 47397 ,47397 , 47397 , 47397 , 47397 , 47397 ,
	 47397 , 47397 , 47397 , 47397 , 47397 ,47397 , 47397 , 47397 , 47397 , 47397 ,47397 , 47397 , 47397 , 47397 , 47397 ,
	 47397 , 47397 , 47397 , 47397 , 47397 ,47397 , 47397 , 47397 , 47397 , 47397 ,47397 , 47397 , 47397 , 47397 , 47397 ,
	 0
} ;
int8u rtc_send_data[8];
int16u caladcar[]={ 0 , 10 , 100 , 1000 , 230 , 450 , 500 , 1000 , 1024 , 1025 , 1200 };
int adc10array[]= { 900 , 931 , 920 , 950 , 975 , 1000 , 1010 , 1024 , 1025 , 1008};

typedef struct
{
	int8u validevent;
	int8u nextstate;
	void (*action)(void);
} node;


int8u      keyflag , key ;
int16u     sample ,currval ;
int8u      curidx;

const int8u seg7tab[ 128 ] =
{
	 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  ,
	 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  ,
	 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  ,
	 0x3f , 0x6  , 0x5b , 0x4f , 0x66 , 0x6d , 0x7d , 0x07 , 0x7f , 0x6f , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  ,
	 0x0  , 0x77 , 0x7c , 0x39 , 0x5e , 0x79 , 0x71 , 0x6f , 0x74 , 0x30 , 0x0e , 0x70 , 0x38 , 0x55 , 0x54 , 0x5c ,
	 0x73 , 0x67 , 0x50 , 0x6d , 0x78 , 0x3e , 0x0  , 0x0  , 0x76 , 0x6e , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  ,
	 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  ,
	 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0  , 0x0
} ;

/*........................................................................................*/

int16u locnt, hicnt, lowtmp , hitmp ;
int16u idlspncnt, idlzrcnt,idlcnt,temperature;
int32u idladccnt,lin_val;
typedef struct
{
	int16u tmp ;
	int16u cnt ;
}tab;

tab Rtab[16] = {{1000 , 43274},{1050 , 43941},{1100 , 44618},{1150 , 45303},{1200 , 45996},{1250 , 46694},{1300 , 47397},
				{1350 , 48102},{1400 , 48808},{1450 , 49514},{1500 , 50219},{1550 , 50920},{1600 , 51617},{1650 , 52308},
				{1700 , 52990},{1768 , 53869}
			   };

void getrtc( void ) ;

int8u find_range( int16u cnt)
{  int8u i = 0;
	while(1)
	{
		if( cnt > Rtab[i].cnt && cnt < Rtab[i+1].cnt )
		{
			locnt  = Rtab[i].cnt;
			hicnt  = Rtab[i+1].cnt;
			lowtmp = Rtab[i].tmp;
			hitmp  = Rtab[i+1].tmp;
			return 0;
		}
		i++;
	}

}

void calidlcnt(void)
{

	idladccnt = latchmax - zrocnt ;
	idladccnt *= 10000 ;
	idladccnt = ( idladccnt + ( ( spncnt - zrocnt ) >> 1 ) ) / ( spncnt - zrocnt ) ;
	idladccnt = ( ( idladccnt * ( idlspncnt - idlzrcnt ) + 5000 ) / 10000 ) ;
	idlcnt    = idladccnt + idlzrcnt ;
}

void cal_linear_val(void)
{   calidlcnt();
	lin_val = idlcnt - locnt ;
	lin_val *= 10000 ;
	lin_val = lin_val / ( hicnt - locnt ) ;
	lin_val = ( lin_val * ( hitmp - lowtmp))/10000;
	lin_val += lowtmp ;
	temperature = lin_val ;
}

/*........................................................................................*/

#define PIT_FREQ 0x1234DDL
#define PIT_CTRL 0x43
#define PIT_CHN0 0x40

#ifdef __cplusplus
	#define __CPPARGS ...
#else
	#define __CPPARGS
#endif

unsigned long Timer_ticks, Timer_counter ;

void interrupt( *systemInt8 )(__CPPARGS) ;

void TimerISR_BIOS_Hook(void){}
void TimerISR_App_Hook(void);

void TimerISR_Hook( void ) ;

void interrupt TimerISR( void )
{
	systemInt8(__CPPARGS);
	Timer_ticks += Timer_counter ;
	if( Timer_ticks >= 0x10000L )
	{
		systemInt8(__CPPARGS);
		Timer_ticks &= 0xFFFF;
	}
	if ( CONVCOM ) CONVCOM--;
	TimerISR_BIOS_Hook();
	TimerISR_App_Hook();

	outportb( 0x20 , 0x20 );
}

void Terminate_timer( void )
{
	outportb( PIT_CTRL , 0x34 ) ;
	outportb( PIT_CHN0 , 0 ) ;
	outportb( PIT_CHN0 , 0 ) ;
	setvect( 8 , systemInt8 ) ;
}

void TimerInit(void)
{
	Timer_ticks = 0;
	Timer_counter = PIT_FREQ/1000L;

	outportb( PIT_CTRL , 0x34 ) ;
	outportb( PIT_CHN0 , Timer_counter & 0xFF ) ;
	outportb( PIT_CHN0 ,Timer_counter >> 8 ) ;

	systemInt8 = getvect( 8 ) ;
	setvect( 8 , TimerISR ) ;

}

/*........................................................................................*/

void putpix( int16u x , int16u y , int8u c )
{

	 inregs.h.ah = 0x0c ;
	 inregs.h.al = c ;
	 inregs.h.bh = 0x00 ;
	 inregs.x.cx = x ;
	 inregs.x.dx = y ;
	 int86( 0x10 , &inregs , &outregs ) ;

}

void drawhor( int16u x1 , int16u x2 , int16u y , int8u c )
{
	 int16u i ;
	 for ( i = x1 ; i <= x2 ; i++ )
		 putpix( i , y , c ) ;

}

void drawver( int16u x , int16u y1 , int16u y2 , int8u c )
{
	 int16u i ;
	 for ( i = y1 ; i <= y2 ; i++ )
		 putpix( x , i , c ) ;
}

void endgr( void )
{
	 inregs.h.ah = 0 ;
	 inregs.h.al = oldmode ;
	 int86( 0x10 , &inregs , &outregs ) ;
}

void initgrmd( void )
{
	 inregs.h.ah = 0xf ;
	 int86( 0x10 , &inregs , &outregs ) ;
	 oldmode = outregs.h.al ;
	 inregs.h.ah = 0 ;
	 inregs.h.al = 0x12 ;
	 int86( 0x10 , &inregs , &outregs ) ;
	 x7seg = 100 ;
	 y7seg = 100 ;
	 drawver( x7seg -  10 , y7seg - 10  , y7seg + 48 , 0x0b ) ;
	 drawver( x7seg -  20 , y7seg - 20  , y7seg + 58 , 0x0b ) ;
	 drawver( x7seg + 170 , y7seg - 10  , y7seg + 48 , 0x0b ) ;
	 drawver( x7seg + 180 , y7seg - 20  , y7seg + 58 , 0x0b ) ;
	 drawhor( x7seg -  10 , x7seg + 170 , y7seg - 10 , 0x0b ) ;
	 drawhor( x7seg -  20 , x7seg + 180 , y7seg - 20 , 0x0b ) ;
	 drawhor( x7seg -  10 , x7seg + 170 , y7seg + 48 , 0x0b ) ;
	 drawhor( x7seg -  20 , x7seg + 180 , y7seg + 58 , 0x0b ) ;
	 drawver(190, 80,158,0x0b);

	 drawver(80 ,180,210,0x0f); drawhor(80,110,180,0x0f);  drawhor(80,110,210,0x0f);
	 drawver(110,180,210,0x0f); drawhor(120,150,180,0x0f); drawhor(120,150,210,0x0f);
	 drawver(120,180,210,0x0f); drawhor(160,190,180,0x0f); drawhor(160,190,210,0x0f);
	 drawver(150,180,210,0x0f); drawhor(200,230,180,0x0f); drawhor(200,230,210,0x0f);
	 drawver(160,180,210,0x0f); drawver(190,180,210,0x0f); drawver(200,180,210,0x0f);
	 drawver(230,180,210,0x0f);

	 drawver(20,300,340,0x0a); drawver(500,300,340,0x0a);
	 drawhor(20,500,300,0x0a); drawhor(20,500,340,0x0a);

}

void draw7seg( int8u dgtno , int8u digit )
{
	int8u segments , color ;

	segments = seg7tab[ digit ] ;
	if ( segments & 0x01 ) color = 0xf ; else color = 0x0 ;
	drawhor( x7seg + ( dgtno * 16 ) +  3 , x7seg + ( dgtno * 16 ) + 13 , y7seg + 3 , color ) ;
	if ( segments & 0x02 ) color = 0xf ; else color = 0x0 ;
	drawver( x7seg + ( dgtno * 16 ) + 13 , y7seg + 3 , y7seg + 18 , color ) ;
	if ( segments & 0x04 ) color = 0xf ; else color = 0x0 ;
	drawver( x7seg + ( dgtno * 16 ) + 13 , y7seg + 18, y7seg + 33 , color ) ;
	if ( segments & 0x08 ) color = 0xf ; else color = 0x0 ;
	drawhor( x7seg + ( dgtno * 16 ) + 3 , x7seg + ( dgtno * 16 ) + 13 , y7seg + 33 , color ) ;
	if ( segments & 0x10 ) color = 0xf ; else color = 0x0 ;
	drawver( x7seg + ( dgtno * 16 ) + 3 , y7seg + 18 , y7seg + 33 , color ) ;
	if ( segments & 0x20 ) color = 0xf ; else color = 0x0 ;
	drawver( x7seg + ( dgtno * 16 ) + 3 , y7seg +  3 , y7seg + 18 , color ) ;
	if ( segments & 0x40 ) color = 0xf ; else color = 0x0 ;
	drawhor( x7seg + ( dgtno * 16 ) + 3 , x7seg + ( dgtno * 16 ) + 13 , y7seg + 18 , color ) ;
}

void waitkey( void )
{

		  int8u keystat , kcode ;

		  inregs.h.ah = 0 ;
		  int86( 0x16 , &inregs , &outregs ) ;
		  kcode = outregs.h.al ;
		  if ( kcode == 'O' )
		  {
			 adc_cur_state = OPN;
		  }
		  if ( kcode == 'C' )
		  {
			 adc_cur_state = SHORT;
		  }
		  if ( kcode == 'M'|| kcode == 'm' )
		  {
			 adc_cur_state = MSR;
		  }
		  if ( ( kcode > '0' ) && ( kcode < '8' ) )
		  {
			  keyflag = 1 ;
			  key     = kcode - '0' ;
		  }
		  else if ( kcode == '9' )
		  {
			  endgr() ;
			  exit( 0 ) ;
		  }
		  else if ( kcode == '8' )
		  {
			  adcidx = 0 ;
			  adcgen = 1 ;
		  }

}

void getkey( void )
{
	int8u keystat , kcode ;

	keyflag = 0 ;
	inregs.h.ah = 0x01 ;
	int86( 0x16 , &inregs , &outregs ) ;
	keystat = outregs.x.flags ;
	if ( !( keystat & 0x40 ) )
	{
		  inregs.h.ah = 0 ;
		  int86( 0x16 , &inregs , &outregs ) ;
		  kcode = outregs.h.al ;
		  if ( kcode == 'O'|| kcode == 'o' )
		  {
			 adc_cur_state = OPN;
		  }
		  if ( kcode == 'C'|| kcode == 'c')
		  {
			 adc_cur_state = SHORT;
		  }
		  if ( kcode == 'M'|| kcode == 'm')
		  {
			 adc_cur_state = MSR;
			 curidx = 0;
		  }
		  if ( kcode == 'S' || kcode == 's' )
		  {
			 adc_cur_state = SPN ;
		  }
		  if ( ( kcode > '0' ) && ( kcode < '9' ) )
		  {
			  keyflag = 1 ;
			  key     = kcode - '0' ;
			  if(key == 7){ runflag = 0; menuflag = 1;}
		  }
		  else if ( kcode == '0' )
		  {
			  endgr() ;
			  exit( 0 ) ;
		  }
		  else if ( kcode == '8' )
		  {
			  adcidx = 0 ;
			  adcgen = 1 ;
		  }
	}
}


int16u simadc(void)
{
	  switch(adc_cur_state)
	  {
		case OPN: return 65535;

		case SHORT: return 32768;

		case SPN: return 51617;

		case MSR: if( adcar[curidx] == 0)
				  { adc_cur_state = OPN;
					curidx =0;
					break;
				  }
				  else{	return adcar[ curidx++]; break; }
	   }
}


/*void getcaladc( void )
{
	calsample = caladcar[caladcidx ] ;
	caladcidx++ ;
	if(caladcidx == 15) caladcidx = 0;
}*/

void wrbyte( int8u addr , int8u data )
{
	fp = fopen ( "ineprom", "rb+" ) ;
	fseek ( fp, addr, SEEK_SET ) ;
	/*recsize = sizeof ( data ) ; */
	fwrite( &data, 1 , 1, fp ) ;
	fclose ( fp ) ;
}

int8u rdbyte( int8u addr )
{
	fp = fopen ( "ineprom", "rb" ) ;
	fseek ( fp, addr, SEEK_SET ) ;
	fread( &mem_data, 1, 1, fp );
	i2cdata = mem_data ;
	fclose ( fp ) ;
	return mem_data;
}

void writecopy( int8u copyno )
{
	int8u i;
	int8u addr;
	/*void *q;*/
	/*int8u *p ;*/
	chk = 0 ;

	p = (int8u *)(&pp[copyno]);
	/*p = &(int8u *)q ;*/
	for( addr = copyno * 16, i = 0 ; i != 15 ; i++, addr++ )
	{
		wrbyte( addr , *p );
		chk += *p;
		p++ ;
	}

	chk = CHKSUM - chk ;
	wrbyte( addr , chk );
}

int8u readcopy( int8u copyno )
{
	/*int8u *p;    */
	int8u i,addr;
	p = (int8u *)(&pp[copyno]);
	chk = 0;

	for( addr = copyno*16 , i = 0 ; i != 16 ; i++ , addr++ )
	{
		*p = rdbyte(addr);
		chk += *p ;
		p++;
	}

	if( chk == CHKSUM )
	return 1;
	else return -1;

}

void setpar(void)
{
	int8u i;
	for( i = 0 ; i != 4 ; i++ )
	{
		if ( gudbad[i] == -1 ) writecopy(i);
	}
}

void getpar(void)
{
	int8u i , j ;
	int8s sum = 0 ;
	for( i = 0 ; i != 4 ; i++)
	{
		gudbad[i] = readcopy(i);
		sum += gudbad[i];
	}

	if( sum == 4 ) return;

	else if( sum == -4 )
	{
		for( i = 0 ; i != 4 ; i++ )
		{
			pp[i] = dpp;
		}
	}
	else
	{
		for( i = 0 ; i != 4 ; i++ )
		{
			if( gudbad[i] == 1 )
			{
				j = i;
				break;
			}
		}
		for( i = 0 ; i != 4 ; i++ )
		{
			if( gudbad[i] == -1 )  pp[i] = pp[j];
		}
	}
	setpar();
}

/*  while storing input form user  */

void storeip(void)
{
	int i;

	pp[0].heatno = htn_no ;
	pp[0].ladelno = lad_no ;
	pp[0].latchtime = latchtime ;
	pp[0].holdtime = holdtime ;
	pp[0].sttpm = start_temp ;
	pp[0].zrcnt = zrocnt ;
	pp[0].spcnt = spncnt ;
	pp[0].tctype = tc_type ;
	pp[0].gap1 = 0 ; /*gap ;*/
	pp[0].gap2 = 0 ;
	pp[0].chksum = 0 ;

	pp[1] = pp[0] ;
	pp[2] = pp[0] ;
	pp[3] = pp[0] ;

	for( i = 0 ; i != 4 ; i++ )
	{
		gudbad[i] = -1;
	}
	setpar();
}

/*void timer_adc_hook( void ) /* Timer Every 10 mS */
void TimerISR_App_Hook( void )
{
	if( CONVCOM == 0 )
	{
		if(!i2cbusy)
		{
			adc_data = getadc() ;
			newread = 1 ;
		}
		CONVCOM = 1000 ;

	}  /* Using i2c*/
}


void display(int8u *str)
{
	draw7seg( 0 , str[0] ) ;
	draw7seg( 1 , str[1] ) ;
	draw7seg( 2 , str[2] ) ;
	draw7seg( 3 , str[3] ) ;
}

void dispstr( int8u *str )
{
	int i ;

	for ( i = 0 ; i < 4 ; i++ )
	{
		dispbuf[i] = *( str + i ) ;
	}
	display(dispbuf);
}

void dispnum( int16u num)
{   int i;
	for ( i = 3 ; i >= 0 ; i-- )
	{
		dispbuf[i] = ( num % 10 ) + 0x30 ;
		num = num / 10 ;
	}
	display(dispbuf);
}

int getchoice(int choice,int8u nochoice,int8u *str,int8u *msg)
{
	dispstr(msg);
	do { waitkey(); }  while (( key != ENT ));
	oldchoice = choice ;
	if ( key == ENT )
	{
		while(1)
		{
			dispstr(str+(choice*4));
			waitkey();
			switch( key )
			{
				case INC: choice++;
						  if(choice == nochoice) choice = 0;
						  break;

				case DEC: choice--;
						  if(choice < 0) choice = nochoice - 1;
						  break;

				case ESC: choice = oldchoice;
						  break;

				case ENT: return choice;

				case QUIT:choice = oldchoice;quitkey = 1;
						  return choice;

			}

		}

	}

}

int getnum( int num ,int lownum ,int  upnum ,int8u *msg )
{
	if( num < lownum) num = lownum;
	if( num > upnum ) num = upnum ;
	dispstr( msg );
	do{ waitkey(); } while (( key != ENT )) ;
	oldnum = num ;
	if ( key == ENT )
	{
		while(1)
		{
			dispnum( num );
			waitkey();
			switch( key )
			{
				case INC: num++;
						  if(num > upnum) num = lownum;
						  break;

				case DEC: num--;
						  if(num < lownum) num = upnum;
						  break;

				case ESC: num = oldnum;
						  break;

				case ENT: return num;

				case QUIT:num = oldnum; quitkey = 1;
						  return num;
			}

		}

	}

}

int getnum2( int num , int lownum , int upnum ,int8u  *msg )
{
	int incr,maxdig;
	if( num < lownum) num = lownum;
	if( num > upnum ) num = upnum ;
	dispstr( msg ); incr = 1; maxdig = 9;
	do{ waitkey(); } while (( key != ENT ));
	oldnum = num ;
	if ( key == ENT )
	{
		while(1)
		{
			dispnum( num );
			waitkey();
			switch( key )
			{
				case SHIFT:if(incr < 1000) incr*=10;
						   else incr = 1;
						   if(incr == 1000) maxdig = 1;
						   else maxdig = 9;
						   break;

				case INC: if(num < upnum)
						  {
							if((num%(10*incr)/incr)!= maxdig)
							num += incr;
						  }
						  break;

				case DEC: if(num > lownum)
						  {
							if((num/incr)%10)
							num -= incr;
						  }
						  break;

				case ESC: num = oldnum;
						  break;

				case ENT: return num;

				case QUIT:num = oldnum; quitkey = 1;
						  return num;

			}

		}

	}

}

void getlatch(void)
{
	latchtime = getnum( latchtime , latch_low , latch_up , " EDT");
}

void gethold(void)
{
	holdtime = getnum( holdtime , hold_low , hold_up , " EDT");
}

void tmrset(void)
{
	select = getchoice( 0 , 2 , " LAT HLD", " SEL");
	switch(select)
	{
		case 0 : getlatch(); break;
		case 1 : gethold();  break;
	}
}

void settemp(void)
{
	start_temp = getchoice( start_temp , 3 , " 60010001400" , " EDT");
	switch(start_temp)
	{
		case 0 : start_temp = 0;/*600 */ break;
		case 1 : start_temp = 1;/*1000*/ break;
		case 2 : start_temp = 2;/*1400*/ break;
	}
}

void tcset(void)
{
	tc_type = getchoice( tc_type , 3 , " RTP STP BTP", " EDT");
	switch(start_temp)
	{
		case R : tc_type = 0 ; break;
		case S : tc_type = 1 ; break;
		case B : tc_type = 2 ; break;
	}
}

void set_htnno(void)
{
	htn_no = getnum2( htn_no , htn_low , htn_up , " EDT" );
}

void set_ladno(void)
{
	lad_no = getnum2( lad_no , lad_low , lad_up , " EDT" );
}

void htnset(void)
{
	select = getchoice( 0 , 2 , " HTN LAD", " SEL" );
	switch(select)
	{
		case 0: set_htnno();break;
		case 1: set_ladno();break;
	}
}

void rtcset(void)
{   int8u rtc_w_addr,rtc_w_data;
	getrtc() ;
	select = getchoice( 0 , 5 , " DAT MON  YR  HR MIN", " SEL" );
	switch(select)
	{
		case 0: date = getnum( dd  , date_low  , date_up  , " EDT"); rtc_w_data =	bintobcd(date); rtc_w_addr = day_addr ; break;
		case 1: month= getnum( mm , month_low , month_up , " EDT"); rtc_w_data =	bintobcd(month); rtc_w_addr = month_addr; 	break;
		case 2: year = getnum( yy  , year_low  , year_up  , " EDT"); rtc_w_data =	bintobcd(year); rtc_w_addr = yr_addr; 	break;
		case 3: hour = getnum( hh  , hour_low  , hour_up  , " EDT"); rtc_w_data =	bintobcd(hour); rtc_w_addr = hour_addr; 	break;
		case 4: min  = getnum( ii   , min_low   , min_up   , " EDT"); rtc_w_data =	bintobcd(min); rtc_w_addr = min_addr; 	break;
	}

	i2csendstart();
	i2csendbyte( 0xa2 );
	i2csendbyte( rtc_w_addr);
	i2csendbyte( rtc_w_data);
	i2csendstop();

}
int8u i2cgetbyte( void )
{

	i2c_slave_simulate();
	i2cbyte = i2cdata;
	return i2cbyte;

}

int16u i2cgetword( void )
{
	i2c_slave_simulate();
	i2cword = i2cdata;
	i2cword = i2cword << 8;
	i2c_slave_simulate();
	i2cword += i2cdata;

	return i2cword;
}

int16u rdwordext(int16u addr)
{
	i2csendstart();
	i2csendbyte( mem_write_addr );
	raddrh = ( addr & 0xff00 ) >> 8;
	raddrl = ( addr & 0xff );
	i2csendbyte( raddrh );
	i2csendbyte( raddrl );
	i2csendstart();
	i2csendbyte( mem_read_addr );
	mem_i2cdata_wrd = i2cgetword();

	i2csendstop();
	return mem_i2cdata_wrd;
}

int8u rdbyteext(int16u addr)
{
	i2csendstart();
	i2csendbyte(mem_write_addr );
	raddrh = ( addr & 0xfff00 ) >> 8;
	raddrl = ( addr & 0xff );
	i2csendbyte( raddrh );
	i2csendbyte( raddrl );
	i2csendstart();
	i2csendbyte( mem_read_addr );

	mem_i2cdata = i2cgetbyte();
	i2csendstop();
	return mem_i2cdata;
}

void wrwordext(int16u worddata , int16u addr)
{
	i2csendstart();
	i2csendbyte(mem_write_addr );
	raddrh = ( addr & 0xff00 ) >> 8;
	raddrl = ( addr & 0xff );
	i2csendbyte( raddrh );
	i2csendbyte( raddrl );   /*chk ack*/
	i2cbyte = ( worddata & 0xff00)>> 8 ;
	i2csendbyte( i2cbyte );
	i2cbyte = (worddata & 0xff );
	i2csendbyte( i2cbyte );
	i2csendstop();
}

void wrbyteext( int8u bytdata , int16u addr)
{
	i2csendstart();
	i2csendbyte(mem_write_addr );
	raddrh = ( addr & 0xff00 ) >> 8;
	raddrl = ( addr & 0xff );
	i2csendbyte( raddrh );
	i2csendbyte( raddrl );   /*chk ack*/
	i2cbyte =  bytdata;
	i2csendbyte( i2cbyte );
	i2csendstop();
}

void writeread(void)
{
   /*	int8u *p;*/
	int16u addr;
	int8u i;
	staddr = rdwordext(staaddr);
	noread = rdwordext(noaddr);
	p = (int8u*)&reading;

	for( addr = staddr , i = 0 ; i != 9 ; i++ , addr++ )
	{
		wrbyteext( *p , addr );
		p++;
	}

	if( noread != 900 ) noread++;

	staddr += 9;
	if( staddr == 8100 ) staddr = 0;

	wrwordext( staddr , staaddr );
	wrwordext( noread , noaddr  );
	pp[0].gap1 = pp[1].gap1 = pp[2].gap1 = pp[3].gap1 = staddr;
	pp[0].gap2 = pp[1].gap2 = pp[2].gap2 = pp[3].gap2 = noread;
	gudbad[0] = gudbad[1] = gudbad[2] = gudbad[3] = -1;
	setpar();
}

int8s readread( int16u index )
{
	int16u addr;
	int8u i;
	staddr = rdwordext(staaddr);
	noread = rdwordext(noaddr);
	if( index >= noread ) return -1;
	p = (int8u*)&reading;

	if ( noread == 900 )
		addr = staddr + ( index * 9 ) ;
	else
		addr = index * 9 ;
	if ( addr > 8100 )
	{
		addr = addr - 8100 ;
	}

	for( i = 0 ; i != 9 ; i++ )
	{
		*p = rdbyteext( addr);
		addr++; p++;
	}

	return 1;

}

void print(int8u *str )
{  int8u i;
	int x = 0;
	if(initflag == 1)	x = 19;
	else x = 22;

	  for(i = 0; *str != 0 ; i++)
	  {	inregs.h.ah = 0x02 ;
		inregs.h.bh = 0;
		inregs.h.dh = x;
		inregs.h.dl = 4+i;
		int86( 0x10 , &inregs , &outregs ) ;
		inregs.h.ah = 0x0A ;
		inregs.h.al = *str++;
		inregs.h.bh = 00;
		inregs.h.bl = 0x0f;
		inregs.x.cx = 1;
		int86( 0x10 , &inregs , &outregs ) ;
	  }
}

void printread(void)
{
	int8u i;
	int16u j;

	noread = rdwordext(noaddr);

	for( i = 0 ; i < noread ; i++ )
	{
		if( readread(i) == 1)
		{
			itoa( reading.htno, printstr,10);
			for(j = strlen(printstr); j<8; j++) strcat(printstr," ");

			itoa( reading.ladno, printstr+8,10);
			for(j = strlen(printstr); j<16; j++) strcat(printstr," ");

			dday = (( reading.rdate & 0x1f)) ;
			dmon = (((reading.rdate >> 5) & 0x0f)+1) ;
			dyr  = ((reading.rdate >> 9) & 0xff);
			itoa((int16u)dday,printstr+16,10);
			strcat(printstr,"/");
			itoa((int16u)dmon,printstr+strlen(printstr),10);
			strcat(printstr,"/");
			itoa((int16u)dyr,printstr+strlen(printstr),10);
			/*itoa( reading.rdate, printstr+16,10);*/
			for(j = strlen(printstr); j<24; j++) strcat(printstr," ");

			dhr  = reading.rtime & 0xff;
			dmin = ((reading.rtime & 0xff00)>>8);
			itoa(dmin,printstr+24,10);
			/*itoa( drtime, printstr+24,10);*/
			strcat(printstr,":");
			itoa(dhr ,printstr+27,10);
			for(j = strlen(printstr); j<33; j++) strcat(printstr," ");

			itoa( reading.temp, printstr+33,10);
			/*strcat( printstr, "    " );*/

			strcat( printstr, "\r\n" );
		}

		print( printstr );
		for(j = 0 ; j < 1000 ; j++){}
	}

}
int16u getadc( void )
{
	i2csendstart();
	i2csendbyte(adc_read_addr);

	adc_read_data = i2cgetbyte();
	adc_read_data = adc_read_data << 8 ;
	adc_read_data +=i2cgetbyte();
	i2csendstop();
	return adc_read_data;
}

void initadc(void)
{
	i2csendstart();
	i2csendbyte(adc_write_addr);

	i2csendbyte(0x01);
	i2csendbyte(0x01);
	i2csendbyte(0x05);
	i2csendbyte(0x00);
	i2csendstart();

	i2csendbyte(adc_write_addr);

	i2csendbyte(0x02);
	i2csendbyte(0x01);
	i2csendstop();
}


void dispmem(void)
{
	int i;
	noread = rdwordext(noaddr);

	for( i = noread-1 ; i >= 0 ; i-- )
	{
		if( readread(i) == 1 )
		{
			dispstr(" HTN");
			waitkey();
			dispnum( reading.htno );

			waitkey();if( key == ESC || key == QUIT ) return;
			dispstr(" LAD"); waitkey();
			dispnum( reading.ladno);
			waitkey();if( key == ESC || key == QUIT ) return;

			dday = (( reading.rdate & 0x1f)) ;
			dmon = (((reading.rdate >> 5) & 0x0f)+1) ;
			dyr  = ((reading.rdate >> 9) & 0xff);

			dhr  = reading.rtime & 0xff;
			dmin = ((reading.rtime & 0xff00)>>8);

			dispstr(" DAT");
			dispnum( dday);
			waitkey();if( key == ESC || key == QUIT ) return;
			dispnum(dmon); waitkey(); dispnum(dyr);waitkey();
			dispstr(" TIM");waitkey();
			dispnum(dhr);waitkey(); dispnum(dmin);waitkey();
			dispstr(" TMP");
			waitkey();if( key == ESC || key == QUIT ) return;
			dispnum( reading.temp);
			waitkey();if( key == ESC || key == QUIT ) return;
			dispstr( "    ");
			waitkey();if( key == ESC || key == QUIT ) return;
		}
	}
}


int16u getcalval( int8u *msg , int16u lowcnt , int16u hicnt , int16u oldcal)
{
	int i;
	dispstr( msg );
	delaycnt = 500 ;
	for(i = 0 ; i < 1000 ; i++)   /* instead of timer */
	{
		for(i = 0 ; i < 1000 ; i++)
		{
			if(delaycnt > 0) delaycnt--;
		}
	}
	/*while(delaycnt){}*/
	while(1)
	{
		/*getcaladc();
		adccnt = calsample; */
		key = NONE ;
		getkey();
		if(key == ESC)  return oldcal ;
		if ( newread ) { dispnum( adc_data >> 4 ) ; newread = 0 ; }
		if(key == ENT)
		{
			if( ( adc_data > lowcnt ) && ( adc_data < hicnt) )
			{
				return adc_data;
			}
			else dispstr( " ERR");
			delaycnt = 500 ;
		}
	}
}

void caliberate(void)
{
	select = getchoice( 0 , 2 , " SPN ZRO", " SEL" );
	switch(select)
	{
		case SPAN: spncnt = getcalval( " SPN" , idlspncnt - 100 , idlspncnt + 100 , spncnt );
				   break;

		case ZERO: zrocnt = getcalval( " ZRO" , idlzrcnt - 100 , idlzrcnt + 100 , zrocnt );
				   break;
	}

}

void printlog(void)
{
	select = getchoice( 0 , 2 , " N   Y  ", " SEL" );
	switch(select)
	{
		case N: send_data_flag = 0; break;

		case Y: printread(); send_data_flag = 1; break;
	}

}

int16u getadc10cnt(void)
{
	adc10pos++;
	if(adc10pos == 10) adc10pos = 0;
	return adc10array[adc10pos];
}

void chkbat(void)
{
	adc10cnt = getadc10cnt();
	if( adc10cnt > 1000 ) adc10cnt = 1000;
	if( adc10cnt < 920  ) adc10cnt = 920;
	bat_state = (((adc10cnt - 920)*5)/4);/*///(((adccnt - 920)/80)*100)*/
	dispnum( bat_state);
	waitkey();
}

int menu(void)
{
  while(1)
  {
	select = getchoice( select , 10 , " TMR TMP  TC HTN RTC PRN CAL MEM BAT EXT"," MNU" );

		switch(select)
		{
			case TMR: tmrset();	break;

			case TMP: settemp();break;

			case TC : tcset(); 	break;

			case HTN: htnset();	break;

			case RTC: rtcset();	break;

			case PRN: printlog();	break;

			case CAL: caliberate(); break;

			case MEM: dispmem();	break;

			case BAT: chkbat();	break;

			case EXT: storeip(); runflag = 0 ;menuflag = 0 ; select = 0 ; return 0;

		}

		if (quitkey == 1) {  quitkey=0; return 0;  }
		/*select = getchoice( select , 9 , " TMR TMP  TC HTN RTC PRN CAL MEM BAT"," MEN" );*/
   }

}

/*getdate()
{
	if(month ==2 )
	{
		if(year%4) date_up = 29;
		else date_up =28;
	}
	else if(month == 1 ||month == 3||month ==5 ||month == 7|| month ==8||month ==10 ||month==12)
	{
		date_up = 31 ;
	}
	else date_up = 30 ;

}*/

/*/////////////////////////////////     SLAVE SIMULATE      /////////////////////////////////////////////*/

			  /*//////////////////////   RTC   ///////////////////////////*/
						/* RTC ACTIONS  */
void rtc_action00(){}
void rtc_action02(){  rtc_ACK=0;  }
void rtc_action01()
{   int i;
	 /*/read from PC rtc*/
	inregs.h.ah = 2 ;
	int86( 0x1a , &inregs , &outregs ) ;
	bcdhours  = outregs.h.ch ;

	rtc_send_data[ hour_addr ]= bcdhours;
	bcdmins  = outregs.h.cl;

	rtc_send_data[ min_addr ] = bcdmins;
	bcdsecs = outregs.h.dh ;

	rtc_send_data[ sec_addr ] = bcdsecs;

	inregs.h.ah = 4;
	int86( 0x1a , &inregs , &outregs ) ;
	bcdyear  = outregs.h.cl ;   rtc_send_data[yr_addr] = bcdyear;
	bcdmonth  = outregs.h.dh;   rtc_send_data[month_addr] = bcdmonth;
	bcdday = outregs.h.dl ;     rtc_send_data[day_addr] = bcdday;
	/*i2cdata = rtc_data  ;*/
	rtc_ACK=0;    wordaddr = min_addr;
	/*i2cdata = rtc_send_data[wordaddr];
	wordaddr++;
	if( wordaddr == rtcaddrmax ) {wordaddr = rtcaddrmin;}*/ /*VAL_Master_ACK=1;}/*only for test*/
}

void rtc_action05(){  rtc_ACK=1;  }

void rtc_action11()
{
	/*/read from PC rtc*/
	/*rtc_data = rtc_send_data[wordaddr];
	/*conv bcd*/
	i2cdata = rtc_send_data[ wordaddr ];
	wordaddr++;
	if( wordaddr == rtcaddrmax ) {wordaddr = rtcaddrmin;}
	/*i2cdata = rtc_data  ; */

}

void rtc_action15(){rtc_ACK=1;  }
void rtc_action10(){}

void rtc_action23()
{
	wordaddr  =  i2cdata;
	if(wordaddr == hour_addr) 	w_hr_flag  = 1; 	else  w_hr_flag =0;
	if(wordaddr == min_addr) 	w_min_flag = 1; 	else  w_min_flag =0;
	if(wordaddr == sec_addr) 	w_sec_flag = 1; 	else  w_sec_flag =0;
	if(wordaddr == yr_addr) 	w_yr_flag  = 1; 	else  w_yr_flag =0;
	if(wordaddr == month_addr)	w_month_flag = 1;	else  w_month_flag =0;
	if(wordaddr == day_addr) 	w_day_flag =1; 		else  w_day_flag =0;

	rtc_ACK=0;
}

void rtc_action20(){}

void rtc_action34()
{
	data = i2cdata;
	/*/write to PC RTC */
	inregs.h.ah = 3 ;
	if(w_hr_flag ==  1) inregs.h.ch = data; else inregs.h.ch = bintobcd( hh ) ;
	if(w_min_flag == 1)	inregs.h.cl = data; else inregs.h.cl = bintobcd( ii ) ;
	if(w_sec_flag == 1)	inregs.h.dh = data; else inregs.h.dh = 0 ;
	inregs.h.dl = 0;
	int86( 0x1a , &inregs , &outregs ) ;

	rtc_ACK=0;
	inregs.h.ah = 5 ;
	if(w_yr_flag ==1) inregs.h.cl = data; else inregs.h.cl = bintobcd( yy ) ;
	if(w_month_flag ==1) inregs.h.dh = data; else inregs.h.dh = bintobcd( mm ) ;
	if(w_day_flag == 1 ) inregs.h.dl = data; else inregs.h.dl = bintobcd( dd ) ;

	int86( 0x1a , &inregs , &outregs ) ;
	wordaddr++;
}

void rtc_action30(){}

void rtc_action44()
{
	data = i2cdata;

  /*/write to PC RTC   */
	inregs.h.ah = 3 ;
	if(w_hr_flag ==1)  	inregs.h.ch = data;
	if(w_min_flag ==1)	inregs.h.cl = data;
	if(w_sec_flag == 1 )inregs.h.dh = data;
	inregs.h.dl = 0;
	int86( 0x1a , &inregs , &outregs ) ;
	rtc_ACK=0;
	inregs.h.ah = 5 ;
	if(w_yr_flag ==1) 	inregs.h.cl = data;
	if(w_month_flag ==1)inregs.h.dh = data;
	if(w_day_flag == 1 )inregs.h.dl = data;

	int86( 0x1a , &inregs , &outregs ) ;
	rtc_ACK=0;
}

void rtc_action40(){}

void rtc_action50(){}

					/*RTC  DEFS*/

node rtc_state0[5]=
{ { AMW, 2, rtc_action02},{ AMR, 1 ,rtc_action01},{ ANM, 5 , rtc_action05},{RESTART,0, rtc_action00},{STOP,0,rtc_action00}};

node rtc_state1[4]=
{ {MASTER_ACK,1,rtc_action11} ,{MASTER_NACK, 5  , rtc_action15},{STOP ,0 , rtc_action10},{RESTART, 0, rtc_action10 } };

node rtc_state2[3]=
{ {WORD_ADDR, 3, rtc_action23}, {RESTART, 0, rtc_action20}, {STOP, 0, rtc_action20} };

node rtc_state3[3]=
{ {rtc_DATA ,4, rtc_action34}, {RESTART, 0, rtc_action30 }, {STOP, 0,  rtc_action30} };

node rtc_state4[3]=
{ {rtc_DATA, 4, rtc_action44}, {RESTART, 0, rtc_action40}, {STOP, 0, rtc_action40} };

node rtc_state5 [1]=
{ {STOP, 0, rtc_action50}};

node* rtc_states[7]={ rtc_state0, rtc_state1, rtc_state2 , rtc_state3 , rtc_state4 , rtc_state5 };
int8u rtc_validevents[7] = {5,4,3,3,3,1};

	/*/////////////////////   rtc  events   ///////////////////*/

int8u i2cslavertc( int8u i2cdata )
{
	 int i;

	if(rtc_currentstate == 0)
	{
	  if(var_start) 	rtc_currentevent= RESTART;
	  else if(var_stop) rtc_currentevent= STOP;
	  else if(i2cdata == rtc_write_addr) rtc_currentevent = AMW;
	  else if(i2cdata == rtc_read_addr ) rtc_currentevent = AMR;
	  else rtc_currentevent = ANM;
	}

	if(rtc_currentstate == 1)
	{
	  if( VAL_Master_ACK == 0) 	rtc_currentevent = MASTER_ACK;
	  else if( VAL_Master_ACK ==1)  rtc_currentevent = MASTER_NACK;
	  if (var_start) rtc_currentevent = RESTART;
	  else if(var_stop)   rtc_currentevent = STOP;
	}

	if(rtc_currentstate == 2)
	{
	  if (var_start) rtc_currentevent = RESTART;
	  else if(var_stop)   rtc_currentevent = STOP;
	  else rtc_currentevent = WORD_ADDR;
	}

	if(rtc_currentstate == 3)
	{
	  if(var_start) rtc_currentevent = RESTART;
	  else if(var_stop)  rtc_currentevent = STOP;
	  else rtc_currentevent = rtc_DATA;
	}

	if(rtc_currentstate == 4)
	{
	  if(var_start) rtc_currentevent = RESTART;
	  else if(var_stop)  rtc_currentevent = STOP;
	  else rtc_currentevent = rtc_DATA;
	}

	if(rtc_currentstate == 5)
	{
		if(var_stop) rtc_currentevent = STOP;
	}

	 /*          standard code for state change         */

	for(i = 0;i < rtc_validevents[rtc_currentstate];i++)
	if((rtc_states[rtc_currentstate]+i)->validevent == rtc_currentevent)
	{
		(rtc_states[rtc_currentstate]+i) -> action();
		rtc_currentstate = (rtc_states[rtc_currentstate]+i) -> nextstate;
	}

	  return 1;

}
			/*//////////////////////    MEM    /////////////////////////*/

					  /*  MEM ACTIONS   */

void mem_action00(){}
void mem_action01()
{
   /*	fp = fopen( "myeeprom", "rb" ) ;
	fseek ( fp,mem_addr, SEEK_SET ) ;
	fread ( &mem_data, sizeof (mem_data), 1, fp ) ;
	i2cdata = mem_data ;
	fclose ( fp ) ;  */
  /*/i2cdata = mem_data ;*/
	mem_ACK=0;
}

void mem_action02(){mem_ACK=0; }
void mem_action06(){mem_ACK=1;}

void mem_action11()
{
	fp = fopen ( "myeeprom", "rb" ) ;
	fseek ( fp,mem_addr, SEEK_SET ) ;
	fread ( &mem_data, sizeof(mem_data), 1, fp );
	i2cdata = mem_data ;
	fclose ( fp ) ;
  /*/i2cdata = mem_data ;*/
	mem_addr++;

 }

void mem_action16(){  mem_ACK=1; }
void mem_action10(){ }

void mem_action23()
{
	addrh = i2cdata ;
	mem_addr = addrh << 8  | addrl ;
	mem_ACK=0;
}
void mem_action20(){}

void mem_action34()
{
	addrl = i2cdata ;
	mem_addr = addrh << 8  | addrl ;
	mem_ACK=0;
}
void mem_action30(){}

void mem_action45()
{
	s = i2cdata ;
	fp = fopen ( "myeeprom", "rb+" ) ;
	fseek ( fp, mem_addr, SEEK_SET ) ;
	recsize = sizeof ( s ) ;
	fwrite ( &s, recsize, 1, fp ) ;
	fclose ( fp ) ;
	mem_ACK=0;
}
void mem_action40(){}

void mem_action55()
{
	s = i2cdata ;
	fp = fopen ( "myeeprom", "rb+" ) ;
	fseek ( fp, mem_addr, SEEK_SET ) ;
	recsize = sizeof ( s ) ;
	fwrite ( &s, recsize, 1, fp ) ;
	fclose ( fp ) ;
	mem_ACK=0;
}
void mem_action50(){}

void mem_action60(){}


						 /*MEM  DEFS*/

node mem_state0[5]=
{{AMW,2,mem_action02},{AMR,1,mem_action01},{ANM,6,mem_action06},{RESTART,0, mem_action00},{STOP,0,mem_action00}} ;

node mem_state1[4]=
{{MASTER_ACK,1,mem_action11},{MASTER_NACK,6,mem_action16},{STOP,0,mem_action10},{RESTART,0,mem_action10}};

node mem_state2[3]=
{ {ADDRH, 3, mem_action23}, {RESTART, 0, mem_action20}, {STOP, 0, mem_action20} };

node mem_state3 [3]=
{ {ADDRL,4, mem_action34}, {RESTART, 0, mem_action30}, {STOP, 0, mem_action30} };

node mem_state4 [3]=
{ {mem_DATA, 5, mem_action45},{RESTART, 0, mem_action40 },{STOP, 0, mem_action40}};

node mem_state5 [3]=
{ {mem_DATA, 5, mem_action55}, {STOP, 0, mem_action50}, {RESTART, 0, mem_action50}};

node mem_state6 [1]=
{ { STOP, 0 , mem_action60} };

node* mem_states[7]={ mem_state0, mem_state1, mem_state2 , mem_state3 , mem_state4 , mem_state5 , mem_state6};
int8u mem_validevents[7] = {5,4,3,3,3,3,1};
int8u adc_config_data[9] = {0};

		/*///////////////// mem events//////////////////*/

int8u i2cslavemem( int8u i2cdata )
{   int i;
	if(mem_currentstate == 0)
	{
	  if(var_start)     mem_currentevent= RESTART;
	  else if(var_stop) mem_currentevent= STOP;
	  else if(i2cdata == mem_write_addr) mem_currentevent = AMW;
	  else if(i2cdata == mem_read_addr)  mem_currentevent = AMR;
	  else mem_currentevent = ANM;
	}

	if(mem_currentstate == 1)
	{
	  if( VAL_Master_ACK == 0) mem_currentevent = MASTER_ACK;
	  else if( VAL_Master_ACK ==1)   mem_currentevent = MASTER_NACK;
	  if (var_start) mem_currentevent = RESTART;
	  else if(var_stop)   mem_currentevent = STOP;
	}

	if(mem_currentstate == 2)
	{
	  if(var_start) mem_currentevent = RESTART;
	  else if(var_stop) mem_currentevent = STOP;
	  else mem_currentevent = ADDRH;
	}

	if(mem_currentstate == 3)
	{
	  if(var_start) mem_currentevent = RESTART;
	  else if(var_stop) mem_currentevent = STOP;
	  else mem_currentevent = ADDRL;
	}

	if(mem_currentstate == 4)
	{
	  if(var_start) mem_currentevent = RESTART;
	  else if(var_stop) mem_currentevent = STOP;
	  else mem_currentevent = mem_DATA;
	}

	if(mem_currentstate == 5)
	{
	  if (var_start) mem_currentevent = RESTART;
	  else if(var_stop) mem_currentevent = STOP;
	  else {mem_currentevent = mem_DATA;mem_addr+=1;}
	}

	if(mem_currentstate == 6)
	{
		if(var_stop) mem_currentevent = STOP;
	}

	/*      standard state change */


	for(i = 0;i < mem_validevents[mem_currentstate];i++)
	if((mem_states[mem_currentstate]+i)->validevent == mem_currentevent)
	{
		(mem_states[mem_currentstate]+i) -> action();
		mem_currentstate = (mem_states[mem_currentstate]+i) -> nextstate;
	}

	  return 0;

}

			/*//////////////////////////////////   ADC   ////////////////////////////////////*/

							 /*  ADC ACTIONS   */
void adc_action00(){}
void adc_action01()
{
	i2cdata16 = simadc();
	bytecnt = 0 ;/*i2cdata = ( 0xff00 & i2cdata16 ) >> 8;*/
	adc_ACK=1;
}
void adc_action02(){adc_ACK=0;}
void adc_action03(){adc_ACK=1;}

void adc_action11()
{

	if(bytecnt == 0)	{ i2cdata = ( 0xff00 & i2cdata16 ) >> 8; bytecnt++;}

	else if(bytecnt == 1)	{i2cdata = (i2cdata16 & 0xff);	bytecnt = 0;}

	adc_ACK=1;
}

void adc_action13(){ }
void adc_action10(){ }

void adc_action22()
{
	configaddr = 0;
   adc_config_data[configaddr] = i2cdata ;
   adc_ACK=0;
   configaddr++;

}
void adc_action20(){}

void adc_action30(){}

						/*   ADC  STATES  */
node adc_state0 [5]=
{{ AMW, 2, adc_action02},{ AMR, 1 ,adc_action01},{ ANM, 3 , adc_action03},{RESTART,0, adc_action00},{STOP,0,adc_action00}};

node adc_state1 [4]=
{{MASTER_ACK,1,adc_action11},{MASTER_NACK,3,adc_action13},{STOP,0 ,adc_action10},{RESTART, 0, adc_action10} };

node adc_state2 [3]=
{ { adc_DATA , 2, adc_action22}, {RESTART, 0, adc_action20}, {STOP, 0, adc_action20} };

node adc_state3 [1]=
{ {STOP, 0, adc_action30} };

node* adc_states[7]={ adc_state0, adc_state1, adc_state2,adc_state3 };
int8u adc_validevents[4] = {5,4,3,1};

	/*////////////////		adc events		//////////////////*/

int8u i2cslaveadc( int8u i2cdata )
{
	 int i;
	 if(adc_currentstate == 0)
		 {
	  if(var_start) adc_currentevent= RESTART;
	  else if(var_stop) adc_currentevent= STOP;
	  else if(i2cdata == adc_write_addr) adc_currentevent = AMW;
	  else if(i2cdata == adc_read_addr) adc_currentevent = AMR;
	  else adc_currentevent = ANM;
	}

	if(adc_currentstate == 1)
	{
	  if (var_start) adc_currentevent = RESTART;
	  else if(var_stop) adc_currentevent = STOP;
	  else if( VAL_Master_ACK == 0) adc_currentevent = MASTER_ACK;
	  else if( VAL_Master_ACK ==1)   adc_currentevent = MASTER_NACK;
	}

	if(adc_currentstate == 2)
	{
	  if (var_start) adc_currentevent = RESTART;
	  else if(var_stop) adc_currentevent = STOP;
	  else adc_currentevent = adc_DATA;
	}

	if(adc_currentstate == 3)
	{
		if(var_stop) adc_currentevent = STOP;
	}

	/* standard state dig code*/

	for(i = 0;i < adc_validevents[adc_currentstate];i++)
	if((adc_states[adc_currentstate]+i)->validevent == adc_currentevent)
	{
		(adc_states[adc_currentstate]+i) -> action();
		adc_currentstate = (adc_states[adc_currentstate]+i) -> nextstate;
	}
	  return 1;
}

		   /*/////////////////////////////////    TEMPERATURE SENSOR    //////////////////////////////////////*/

						/*  TEMP ACTIONS  */

void temp_action00(){}
void temp_action02(){temp_ACK=0;}
void temp_action01()
{
  char temp_data ;
  i2cdata = temp_data ;

  /*/i2cdata = temp_data ;*/
  temp_ACK=1;
 }
void temp_action05(){}

void temp_action11()
{
	i2cdata = temp_data ;
  /*/i2cdata = temp_data ;  */
	temp_ACK=1;
 }
void temp_action15(){}
void temp_action10(){}

void temp_action23()
{
	res_command = i2cdata ;
	temp_ACK=0;
}
void temp_action20(){  }

void temp_action34()
{
	s = i2cdata ;
	temp_ACK=0;
}
void temp_action30(){}

void temp_action44()
{
	s = i2cdata ;
	temp_ACK=0;
}
void temp_action40(){}
void temp_action50(){}

						/*  TEMP DEFS*/

node temp_state0 [5]=
{{AMW, 2, temp_action02},{ AMR, 1 ,temp_action01},{ ANM, 5 , temp_action05},{RESTART,0, temp_action00},{STOP,0,temp_action00}};

node temp_state1 [4]=
{{MASTER_ACK,1, temp_action11} ,{MASTER_NACK, 5 , temp_action15},{STOP ,0 ,temp_action10},{RESTART, 0,  temp_action10 } };

node temp_state2 [3]=
{{ COMMAND , 3, temp_action23}, {RESTART, 0, temp_action20}, {STOP, 0, temp_action20} };

node temp_state3 [3]=
{{tmp_DATA ,4, temp_action34}, {RESTART, 0, temp_action30 }, {STOP, 0, temp_action30} };

node temp_state4 [3]=
{{tmp_DATA, 4, temp_action44}, {RESTART, 0, temp_action40 }, {STOP, 0, temp_action40} };

node temp_state5 [1]=
{ { STOP , 0 , temp_action50 } };

node* temp_states[7]={ temp_state0, temp_state1, temp_state2 , temp_state3 , temp_state4 , temp_state5 };
int8u temp_validevents[7] = {5,4,3,3,3,1};

		/*	temp  events	*/

int8u i2cslavetmp( int8u i2cdata )
{
	int i;
	if(tmp_currentstate == 0)
	{
	  if(var_start) tmp_currentevent= RESTART;
	  else if(var_stop) tmp_currentevent= STOP;
	  else if(i2cdata == tmp_write_addr) tmp_currentevent = AMW;
	  else if(i2cdata == tmp_read_addr) tmp_currentevent = AMR;
	  else tmp_currentevent = ANM;
	}

	if(tmp_currentstate == 1)
	{
	  if( VAL_Master_ACK == 0) tmp_currentevent = MASTER_ACK;
	  else if( VAL_Master_ACK ==1)   tmp_currentevent = MASTER_NACK;
	  if (var_start) tmp_currentevent = RESTART;
	  if(var_stop) tmp_currentevent = STOP;
	}

	if(tmp_currentstate == 2)
	{
	  if (var_start) tmp_currentevent = RESTART;
	  if(var_stop) tmp_currentevent = STOP;
	  else tmp_currentevent = COMMAND;
	}

	if(tmp_currentstate == 3)
	{
	  if (var_start) tmp_currentevent = RESTART;
	  if(var_stop) tmp_currentevent = STOP;
	  else tmp_currentevent = tmp_DATA;
	}

	if(tmp_currentstate == 4)
	{
	  if(var_start) tmp_currentevent = RESTART;
	  if(var_stop) tmp_currentevent = STOP;
	  else tmp_currentevent = tmp_DATA;
	}

	if(tmp_currentstate == 5)
	{
		if(var_stop) tmp_currentevent = STOP;
	}

	/* standard state dig code*/

	for(i = 0;i < temp_validevents[tmp_currentstate];i++)
	if((temp_states[tmp_currentstate]+i)->validevent == tmp_currentevent)
	{
		(temp_states[tmp_currentstate]+i) -> action();
		tmp_currentstate = (temp_states[tmp_currentstate]+i) -> nextstate;
	}

	  return 0;

}
	/* ///////////////////////////////*/

void i2c_slave_simulate(void)
{
		resp = NACK ; /* 1 */
		/* disable interrupts */
		resp &= i2cslavertc( i2cdata ) ;
		resp &= i2cslavemem( i2cdata ) ;
		resp &= i2cslaveadc( i2cdata ) ;
		resp &= i2cslavetmp( i2cdata ) ;
		/* enable interrupts */
		i2cresp = resp ;
}

/* System BIOS layer - simulator */

void i2csendbyte( int8u byte )
{
	i2cdata = byte ;
	i2c_slave_simulate() ;
}

void i2csendstart(void)
{
	var_start=1;  i2cbusy = 1;
	i2c_slave_simulate();
	var_start=0;
}

void i2csendstop(void)
{
	var_stop=1;
	i2c_slave_simulate();
	var_stop=0;  i2cbusy = 0;
}

int8u bcdtobin(int8u bcdval)
{
	int8u intval = 0;
	intval = (  (bcdval & 0x0f) + (((bcdval & 0xf0 )>>4)*10)  );
	return intval;
}

int8u bintobcd( int8u intval )
{
	int8u bcdval = 0;
	bcdval = (  ((intval / 10) << 4 ) | ( intval % 10) );
	return bcdval;
}

void getrtc(void)
{
	i2csendstart();
	i2csendbyte( rtc_write_addr);
	i2csendbyte( sec_addr);
	i2csendstart();
	i2csendbyte(rtc_read_addr);
	ii = i2cgetbyte();
	hh = i2cgetbyte();
	dd = i2cgetbyte();
	mm = i2cgetbyte();
	mm = i2cgetbyte();
	yy = i2cgetbyte();
	i2csendstop();
	ii = bcdtobin(ii);
	hh = bcdtobin(hh);
	dd = bcdtobin(dd);
	mm = bcdtobin(mm);
	yy = bcdtobin(yy);
}

void logreading(void)
{
	getrtc();
	reading.htno = htn_no;
	reading.ladno= lad_no;
	reading.rtime = (((int16u)hh) << 8) + ((int16u)ii) ;
	reading.rdate = (((int16u)yy)<<9) + (((int16u)(mm-1))<<5) + ((int16u) dd);
	reading.temp = temperature;
}

/*//////////////////////    sensor run mode state dig implementation    /////////////////////////*/
				  /*  sens ACTIONS   */

void sens_action00(){ dispstr(" OPN");}
void sens_action01(){ dispstr("TMSR");}
void sens_action12()
{
	idx = 0;
	fifo_ful = 0;
	totaltime = 0;
	latch_data[idx] = adc_data;
	idx++;
	dispstr(" MSR");
}

void sens_action11(){ dispstr(" CON");}
void sens_action10(){}
void sens_action22()
{
	int i;
	latch_data[idx] = adc_data;
	idx++;
	dispstr(" MSR");
	if ( idx == latchtime )
	{
		 fifo_ful=1;
		 idx=0;
	}
	if(fifo_ful)
	   {

			rdsum = latchmax = latchmin = latch_data[0] ;
			/* find highest and lowest reading and check for noise margin*/
			for(i = 1 ; i < latchtime ; i++)
			{
				latchmax = max(latch_data[i],latchmax);
				latchmin = min(latch_data[i],latchmin);
				rdsum += latch_data[i];
			}

				latch_diff = latchmax - latchmin;
				noisemargin = 70;
				if(latch_diff < noisemargin)
				{
					cmplt_flag=1;
					sens_currentevent =  LATCH;
					dispstr(" LAT");
					latchmax = rdsum/latchtime;
					find_range(latchmax);
					cal_linear_val();
					dispnum(temperature);
					logreading();
					writeread();
				}
		  }
}

void sens_action23()
{
	delay_time = 0;
}
void sens_action24(){dispstr("1ERR");}
void sens_action20(){}
void sens_action33(){ delay_time++;dispstr(" WT ");}
void sens_action30(){ int i; dispstr(" CMP"); for( i = 0; i < 100 ; i++);}
void sens_action40(){ dispstr(" OPN");}

						 /*sens  DEFS*/

node sens_state0[2] = {{TCDETECTED,1,sens_action01},{TCNOTDETECTED,0,sens_action00}};

node sens_state1[3] = {{TCDIPPED,2,sens_action12},{TCDETECTED,1,sens_action11},{TCNOTDETECTED,0,sens_action10}};

node sens_state2[5]=
{{TCDIPPED,2,sens_action22},{TCBREAK,4,sens_action24},{TCSHORT,4,sens_action24},{NOLATCH,4,sens_action24},{LATCH,3,sens_action23}};

node sens_state3[1] = {{CYCLECOMPLETE,0,sens_action30}};

node sens_state4[3] = {{TCBREAK,0,sens_action40},{TCSHORT,0,sens_action40},{TCNOTDETECTED,0,sens_action40}};

node* sens_states[5]= { sens_state0, sens_state1, sens_state2 , sens_state3 , sens_state4 };

int8u sens_validevents[5] = { 2 , 3 , 5 , 1 , 3 };

		/*///////////////// sens events//////////////////*/

void sens_eventgenerator(void)
{   int i,flo;
	if(sens_currentstate == OPEN)
	{
	  if(adc_data < TC_LIMIT) sens_currentevent = TCDETECTED;
	  if(adc_data > TC_LIMIT) sens_currentevent= TCNOTDETECTED;
	}

	if(sens_currentstate == READY)
	{
	  if((adc_data > LOW_LIM)&&(adc_data < UP_LIM)) sens_currentevent= TCDIPPED;
	  else if(adc_data < TC_LIMIT) sens_currentevent = TCDETECTED;
	  else if(adc_data > TC_LIMIT) sens_currentevent= TCNOTDETECTED;
	}

	if(sens_currentstate == MEASURE)
	{
	   totaltime++ ;

	   if(adc_data > UP_LIM) 	sens_currentevent = TCBREAK;
	   else if(adc_data < LOW_LIM)	sens_currentevent = TCSHORT;
	   else if ( totaltime > SEC8 ) sens_currentevent = NOLATCH ;
	   else sens_currentevent = TCDIPPED ;

	}

	if(sens_currentstate == COMPLETE)
	{
		delay_time++ ;
		if(delay_time == TIMERVAL)	sens_currentevent = CYCLECOMPLETE;
		else if(adc_data > TC_LIMIT) 	sens_currentevent= TCNOTDETECTED;
	}

	if(sens_currentstate == ERROR)
	{
		if(adc_data > UP_LIM) 		sens_currentevent = TCBREAK;
		else if(adc_data < LOW_LIM)	sens_currentevent = TCSHORT;
		else if(adc_data > TC_LIMIT) 	sens_currentevent= TCNOTDETECTED;
	}

}

int8u runmode( void)
{
	int i;
	if ( !newread ) return ;
	newread = 0 ;
	getkey();
	sens_eventgenerator();

	/*      standard state change */
	for(i = 0;i < sens_validevents[sens_currentstate];i++)
	if((sens_states[sens_currentstate]+i)->validevent == sens_currentevent)
	{
		(sens_states[sens_currentstate]+i) -> action();
		sens_currentstate = (sens_states[sens_currentstate]+i) -> nextstate;
	}
	/*/return 0;*/
}

void loadset(void)
{
	htn_no = pp[0].heatno ;
	lad_no = pp[0].ladelno ;
	latchtime = pp[0].latchtime ;
	holdtime  = pp[0].holdtime ;
	start_temp = pp[0].sttpm ;
	zrocnt = pp[0].zrcnt ;
	spncnt = pp[0].spcnt ;
	tc_type = pp[0].tctype ;
	/*gap = 0 ; gap ;*/
	chk = pp[0].chksum ;

}

int main(void)
{
	CONVCOM = 1000;
	initflag = 1;
	idlzrcnt = 32768 ;
	idlspncnt= 51617 ;

	initgrmd();

	initadc();

	print("HEAT    LADEL    DATE    TIME    READING ");
	/*waitkey();*/
	getpar();

	staddr = rdwordext(staaddr);
	noread = rdwordext(staaddr+2);

	if( pp[0].gap1 != staddr || pp[0].gap2 != noread)
	{
		 wrwordext( pp[0].gap1, staaddr);
		 wrwordext( pp[0].gap2, staaddr+2);
	}
	adc_cur_state = 0;
	loadset();
	TimerInit();
    initflag= 0;

	dispstr( " S S" ) ;
	/* waitkey() ;  */
	dispstr( " BTL" ) ;
	/*waitkey() ; */
	runflag = 1;
	while(1)
	{
		if( menuflag == 0 && runflag == 0) { runflag = 1; dispstr( " RUN" ) ; }
		if( runflag )
		{
			/*if( CONVCOM == 0 )
			{
				if(!i2cbusy)
				{
					adc_data = getadc() ;
					newread = 1 ;
				}
				CONVCOM = 100 ;

			} */ /* Using i2c*/

			runmode();
			/*adc_data -= 32768;
			adc_data /= 8;
			dispnum(adc_data);/*dispstr( " RUN" ) ;*/
		}
		getkey();
		if( menuflag == 1) { menu();}
	}

}