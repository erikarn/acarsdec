/*
 *  Copyright (c) 2014 Thierry Leconte (f4dwv)
 *
 *   
 *   This code is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU Library General Public License version 2
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Library General Public License for more details.
 *
 *   You should have received a copy of the GNU Library General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#define _GNU_SOURCE
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <rtl-sdr.h>
#include "acarsdec.h"

#define RTLRATE 24000
#define RTLMULT 50
#define RTLINRATE (RTLRATE*RTLMULT)


static	rtlsdr_dev_t *dev=NULL;

#define RTLOUTBUFSZ 128
#define RTLINBUFSZ (RTLOUTBUFSZ*RTLMULT*2)

static float dsbuff[RTLINBUFSZ/2];

#define MDF (RTLINRATE/2-2*RTLRATE)
static unsigned int chooseFc(unsigned int *Fd,unsigned int nbch)
{
int n;
int ne;

	do {
		ne=0;
		for(n=0;n<nbch-1;n++) {
		  if(Fd[n]>Fd[n+1]) {
		  	unsigned int t;
			t=Fd[n+1]; Fd[n+1]=Fd[n];Fd[n]=t;
			ne=1;
		  }
		}
	} while(ne);
	
	if((Fd[nbch-1]-Fd[0])>MDF) {
		fprintf(stderr,"Frequencies too far apart (max : %dKhz)\n",MDF/1000);
		return 0;
	}

	switch(nbch) {
	case 0: return 0;
	case 1 : return Fd[0]-2*RTLRATE;
	default : if((Fd[nbch/2-1]+RTLRATE)<=(Fd[nbch/2]-RTLRATE)) return (Fd[nbch/2-1]+Fd[nbch/2])/2;
	}
	for(n=1;n<nbch;n++)
		if((Fd[n-1]+RTLRATE)<=(Fd[n]-RTLRATE)) return (Fd[n-1]+Fd[n])/2;
	return Fd[0]-RTLRATE;
}

int initRtl(char **argv,int optind)
{
int r,n;
int dev_index;
char *argF;
unsigned int Fc;
unsigned int Fd[4];

	n = rtlsdr_get_device_count();
	if (!n) {
		fprintf(stderr, "No supported devices found.\n");
		exit(1);
	}

	if(argv[optind]==NULL) {
		fprintf(stderr, "Need device index (ex: 0) after -r\n");
		exit(1);
	}
	dev_index=atoi(argv[optind++]);

	fprintf(stderr, "Using device %d: %s\n",
		dev_index, rtlsdr_get_device_name(dev_index));

	r = rtlsdr_open(&dev, dev_index);
	if (r < 0) {
		fprintf(stderr, "Failed to open rtlsdr device\n");
		return r;
	}
	rtlsdr_set_tuner_gain_mode(dev, 0);
//	r=rtlsdr_set_tuner_gain(dev,-30);
//	if (r < 0) {
//		fprintf(stderr, "WARNING: Failed to set center freq.\n");}


	nbch=0;
	while((argF=argv[optind++])) {
		Fd[nbch++]=1000000*atof(argF);
	};
	Fc=chooseFc(Fd,nbch);
	if(Fc==0) return 1;

	for(n=0;n<nbch;n++) {
		channel_t *ch=&(channel[n]);

		ch->Infs=RTLRATE;
		ch->InBuff=malloc(RTLOUTBUFSZ*sizeof(sample_t));
		ch->AMFreq=((float)Fd[n]-(float)Fc)/(float)(RTLINRATE/2)*2.0*M_PI;

		ch->AMind=0;
		ch->AMDownI=ch->AMDownQ=0;
	}
	for(;n<MAXNBCHANNELS;n++) channel[n].Infs=0;

	fprintf(stderr, "Set center freq. to %dHz\n",(int)Fc);

	r = rtlsdr_set_center_freq(dev,Fc);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set center freq.\n");}

	r = rtlsdr_set_sample_rate(dev, RTLINRATE);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to set sample rate.\n");}

	r = rtlsdr_reset_buffer(dev);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to reset buffers.\n");}


	return 0;
}

int getRtlSample(void)
{
	int r,nread;
	int i,k;
	unsigned char rtlinbuff[RTLINBUFSZ];

	r=rtlsdr_read_sync(dev, &rtlinbuff, RTLINBUFSZ, &nread);
	if (r < 0) {
		fprintf(stderr, "WARNING: Failed to read.\n");
	}
	if (nread == 0) {
		fprintf(stderr, "Error: rtl read bad return.\n");
		return -1;
	}

	pthread_mutex_lock(&datamtx);
	while(wrkmask) pthread_cond_wait(&datawcd, &datamtx);
	pthread_mutex_unlock(&datamtx);

	/* rought /2 downsamplig */
	nread=RTLINBUFSZ/2;
	for(i=k=0;i<nread;i+=2,k+=4) {
		dsbuff[i]=(float)(rtlinbuff[k]+rtlinbuff[k+2]-256);
		dsbuff[i+1]=(float)(rtlinbuff[k+1]+rtlinbuff[k+3]-256);
	}
	return 0;
}


void demodAM(channel_t *ch)
{
	float I,Q,Ir,Qr,cp,sp,p;
	int i;
 
	ch->lenIn=0;
	for(i=0;i<RTLINBUFSZ/2;i+=2) {

		I=dsbuff[i];
		Q=dsbuff[i+1];

		
		p=ch->AMPhi+ch->AMFreq;
		if(p>M_PI) p-=2.0*M_PI;
		if(p<-M_PI) p+=2.0*M_PI;
		ch->AMPhi=p;

		sincosf(p,&sp,&cp);
		Ir=cp*I+sp*Q;Qr=-sp*I+cp*Q;
		ch->AMDownI+=Ir;
		ch->AMDownQ+=Qr;
		ch->AMind++;

		if(ch->AMind>=RTLMULT/2) {
			float val;

			val=hypot(ch->AMDownI,ch->AMDownQ)/(RTLMULT/2)/256.0;
			ch->InBuff[ch->lenIn]=val;

			ch->AMind=0;
			ch->AMDownI=ch->AMDownQ=0;
			ch->lenIn++;
		}
	}
		
}
