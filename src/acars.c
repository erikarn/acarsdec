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
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include "acarsdec.h"

#define SYN 0x16
#define SOH 0x01
#define STX 0x02
#define ETX 0x83
#define ETB 0x97
#define DLE 0x7f

static const unsigned char  numbits[256]={
0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,4,5,5,6,5,6,6,7,5,6,6,7,6,7,7,8
};
#include "syndrom.h"

typedef struct {
	unsigned char mode;
	unsigned char addr[8];
	unsigned char ack;
	unsigned char label[3];
	unsigned char bid;
	unsigned char no[5];
	unsigned char fid[7];
	unsigned char bs,be;
	unsigned char txt[220];
	unsigned char err,lvl;
} acarsmsg_t;

static pthread_mutex_t blkmtx=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t blkwcd= PTHREAD_COND_INITIALIZER;
static msgblk_t *blkq_s=NULL;
static msgblk_t *blkq_e=NULL;

/* CCITT 16 CRC */
#define POLY 0x1021
static void update_crc(unsigned short *crc, unsigned char c)
{
	unsigned char v;
	unsigned int i;
	unsigned short flag,C;

	v = 1;C=*crc;
	for (i = 0; i < 8; i++) {
		flag = (C & 0x8000);
		C = C << 1;
		if (c & v) C = C + 1;
		if (flag != 0) C = C ^ POLY;
		v = v << 1;
	}
	*crc=C;
}

static void fixerr(msgblk_t *blk,int sft,unsigned char msk)
{
 if(sft<=2) {
	if(verbose) fprintf(stderr,"#%d error in CRC\n",blk->chn+1);
	return;
 }
 blk->txt[blk->len-1-(sft-2)]^=msk;
}


void printtime(time_t t)
{
	struct tm *tmp;

	if(t==0) return;

	tmp = gmtime(&t);

	printf ("%02d/%02d/%04d %02d:%02d:%02d",
	     tmp->tm_mday, tmp->tm_mon + 1, tmp->tm_year + 1900,
	     tmp->tm_hour, tmp->tm_min, tmp->tm_sec);
}

void printmsg(acarsmsg_t *msg,int chn,time_t t)
{
	printf("\n[#%1d lvl:%3d err:%1d ",chn+1,msg->lvl,msg->err);
	printtime(t);
	printf(" ----------------------------------------------------------\n");
	printf("Aircraft reg: %s ", msg->addr);
	if(msg->bs!=0x03) printf("Flight id: %s", msg->fid);
	printf("\n");
	printf("Mode: %c ", msg->mode);
	printf("Msg. label: %s\n", msg->label);
	printf("Block id: %x ", (int) msg->bid);
	printf("Ack: %x\n", msg->ack);
	printf("Msg. no: %s\n", msg->no);
	if(msg->bs==0x03)
		printf("No Text\n");
	else
		printf("Message :\n%s\n", msg->txt);
	if(msg->be==0x17) printf("Block End\n");
}

void printoneline(acarsmsg_t *msg,int chn,time_t t)
{
	printf("#%1d %7s %6s %2s %4s ", chn+1,msg->addr,msg->fid,msg->label,msg->no);
	printtime(t);
	printf("\n");
}


static void * blk_thread(void *arg)
{
 do {
	msgblk_t *blk;
	acarsmsg_t  msg;
	int i,k,pn;
	unsigned short crc;

	pthread_mutex_lock(&blkmtx);
	while(blkq_e==NULL) pthread_cond_wait(&blkwcd,&blkmtx);

	blk=blkq_e;
	blkq_e=blk->prev;
	if(blkq_e==NULL) blkq_s=NULL;
	pthread_mutex_unlock(&blkmtx);

	if(blk==NULL) return;

	if(blk->len<13) {
		if(verbose) fprintf(stderr,"#%d too short\n",blk->chn+1);
		free(blk);
		continue;
	}

	/* force STX/ETX */
	blk->txt[12]&=(ETX|STX);
	blk->txt[12]|=(ETX&STX);

	msg.err=0;
	/* parity check */
	pn=0;
	for(i=0;i<blk->len;i++) {
		if((numbits[blk->txt[i]]&1)==0) {
		pn++;
		}
	}
	if(pn>1) {
		if(verbose) fprintf(stderr,"#%d too much parity error : %d\n",blk->chn+1,pn);
		free(blk);
		continue;
	}

	/* crc check */
	crc=0;
	for(i=0;i<blk->len;i++) {
		update_crc(&crc,blk->txt[i]);

	}
	update_crc(&crc,blk->crc[0]);
	update_crc(&crc,blk->crc[1]);

	/* try to fix errors */
	if(crc!=0) {
		int i,k,fx;
		if(verbose) fprintf(stderr,"#%d CRC error, try to recover 1 error\n",blk->chn+1);
		fx=0;
		for(i=0;i<242*8;i++) 
 			if(oneerr[i]==crc) {
				fixerr(blk,i/8,1<<(i%8));
				fx=1;
				msg.err=1;
				break;
			}

		if(fx==0 && pn==0 && blk->len<142) {
			int i,k,l;
			unsigned char  u,v;
			unsigned short n=0;
			if(verbose) fprintf(stderr,"#%d CRC error, try to recover 2 close errors\n",blk->chn+1);
			for(k=1,v=2;k<8;k++,v<<=1) {
		 	  for(l=0,u=1;l<k;l++,u<<=1) {
				if(twoerr[n]==crc) {
					fixerr(blk,0,u|v);
					fx=1;
					msg.err=2;
					break;
				}
				n++;
				for(i=1;i<142;i++) {
					if(twoerr[n]==crc) {
						fixerr(blk,i,u|v);
						fx=1;
						msg.err=2;
						break;
					}
					n++;
				}
				if(i<142) break;
			  }
			  if(l<k) break;
			}
		}

		if(fx==0) {
			if(verbose) fprintf(stderr,"#%d not able to fix it\n",blk->chn+1);
			free(blk);
			continue;
		} else {
			if(verbose) fprintf(stderr,"#%d fix it\n",blk->chn+1);
		}
	}

	/* redo parity checking and remove parity */
	pn=0;
	for(i=0;i<blk->len;i++) {
		if((numbits[blk->txt[i]]&1)==0) {
		pn++;
		}

		blk->txt[i]&=0x7f;
	}
	if(pn) {
		if(verbose) fprintf(stderr,"#%d parity error %d\n",blk->chn+1,pn);
		free(blk);
		continue;
	}

	/* fill msg struct */
	msg.txt[0]= '\0';
	msg.fid[0] = '\0';

	msg.lvl = blk->lvl;

	k = 0;
	msg.mode = blk->txt[k];
	k++;

	for (i = 0; i < 7; i++, k++) {
		msg.addr[i] = blk->txt[k];
	}
	msg.addr[7] = '\0';

	/* ACK/NAK */
	msg.ack = blk->txt[k];
	k++;

	msg.label[0] = blk->txt[k];
	k++;
	msg.label[1] = blk->txt[k];
	if(msg.label[1]==0x7f) msg.label[1]='d';
	k++;
	msg.label[2] = '\0';

	msg.bid = blk->txt[k];
	k++;
	/* STX/ETX */
	msg.bs=blk->txt[k];
	k++;

	if(k<blk->len) {

		for (i = 0; i < 4 && k<blk->len-1; i++, k++) {
			msg.no[i] = blk->txt[k];
		}
		msg.no[i] = '\0';

		msg.be=blk->txt[blk->len-1];
		for (i = 0; i < 6 && k<blk->len-1; i++, k++) {
			msg.fid[i] = blk->txt[k];
		}
		msg.fid[i] = '\0';

		blk->txt[blk->len-1]='\0';
		strcpy(msg.txt, &(blk->txt[k]));
	}

	if(outtype==0)
		printoneline(&msg,blk->chn,blk->t);
	else
		printmsg(&msg,blk->chn,blk->t);

	free(blk);

 } while(1);
}

int initAcars(channel_t *ch)
{

 pthread_t th;

 ch->outbits=0;
 ch->nbits=8;
 ch->Acarsstate=WSYN;

 ch->blk=malloc(sizeof(msgblk_t));
 ch->blk->chn=ch->chn;

 pthread_create(&th,NULL,blk_thread,NULL);
return 0;
}


void putbit(float v,channel_t *ch)
{
	ch->outbits>>=1;
	if(v>0) {
		ch->outbits|=0x80;
	} 
	ch->nbits--;
	if(ch->nbits<=0)decodeAcars(ch);
}

void decodeAcars(channel_t *ch)
{
	unsigned char r=ch->outbits;

	switch (ch->Acarsstate) {
		
		case WSYN:
			if (r==SYN) {
				ch->Acarsstate = SYN2;
				ch->nbits=8;
				return;
			}
			if (r==(unsigned char)~SYN) {
				ch->MskS^=2;
				ch->Acarsstate = SYN2;
				ch->nbits=8;
				return;
			}
			ch->nbits=1;
			return;

		case SYN2:
			if (r==SYN) {
				ch->Acarsstate = SOH1;
				ch->nbits=8;
				return ;
			}
			if (r==(unsigned char)~SYN) {
				ch->MskS^=2;
				ch->nbits=8;
				return;
			}
			ch->Acarsstate = WSYN;
			ch->nbits=1;
			return;

		case SOH1:
			if (r==SOH) {
				if(inpmode!=2) ch->blk->t = time(NULL);
				else ch->blk->t=0;
				ch->Acarsstate = TXT;
				ch->blk->len=0;
				ch->nbits=8;
				return ;
			}
			ch->Acarsstate = WSYN;
			ch->nbits=1;
			return;

		case TXT:
			ch->blk->txt[ch->blk->len] = r;
			ch->blk->len++;
			if (r == ETX || r == ETB) {
				ch->Acarsstate = CRC1;
				ch->nbits=8;
				return ;
			} 
			if (ch->blk->len>20 && r==DLE) {
				if(verbose) fprintf(stderr,"#%d miss txt end\n",ch->chn+1);
				ch->blk->len-=3;
				ch->blk->crc[0]=ch->blk->txt[ch->blk->len];
				ch->blk->crc[1]=ch->blk->txt[ch->blk->len+1];
				ch->Acarsstate = CRC2;
				goto putmsg_lbl;
			}
			if (ch->blk->len > 240) {
				if(verbose) fprintf(stderr,"#%d too long\n",ch->chn+1);
				ch->Acarsstate = WSYN;
				ch->nbits=1;
				return;
			} 
			ch->nbits=8;
			return ;

		case CRC1:
			ch->blk->crc[0]=r;
			ch->Acarsstate = CRC2;
			ch->nbits=8;
			return ;
		case CRC2:
			ch->blk->crc[1]=r;
			ch->blk->lvl=20*log10(ch->Mskdc)+106;
putmsg_lbl:
			pthread_mutex_lock(&blkmtx);
			ch->blk->prev=NULL;
			if(blkq_s) blkq_s->prev=ch->blk;
			blkq_s=ch->blk;
			if(blkq_e==NULL) blkq_e=blkq_s;
			pthread_cond_signal(&blkwcd);
			pthread_mutex_unlock(&blkmtx);

			ch->blk=malloc(sizeof(msgblk_t));
			ch->blk->chn=ch->chn;

			ch->Acarsstate = END;
			ch->nbits=8;
			return ;
		case END:
			ch->Acarsstate = WSYN;
			ch->nbits=8;
			return ;
	}
}
