#include <stdio.h>
#include <string.h>
#include "kalmanFilterDef.h"
#define LINE_SZ	256


int main(int argc,  char **argv) 

{

  kalmanFilter kf;
  float initPose[3];
  float vel[3] = { 350.0/54.0, 0, 0 };
  int   deltat =1;

  FILE *NS, *WE, *TR;
  char fname[256];

  float track[9];
  float NSdata[3];
  float WEdata[3];

  int sampleCount;

  if(argc < 2) { 
    printf("usage %s <filename-prefix>\n",argv[0]);
    return 0;
  }

    
  /* Open the data files */
  sprintf(fname,"%s-NS.csv",argv[1]);
  printf("%s\n",fname);
  if( (NS = fopen(fname, "r")) == NULL) return -1;
  
  sprintf(fname,"%s-WE.csv",argv[1]);
  if( (WE = fopen(fname, "r")) == NULL) return -1;
  
  sprintf(fname,"%s-TR.csv",argv[1]);
  if( (TR = fopen(fname, "w")) == NULL) return -1;
  
  printf("files opened\n");

  /* loop over the data file, run filter, dump track  */
  for(sampleCount=0;  ; sampleCount++) {
    if(fscanf(NS, "%f,%f,%f", &NSdata[0], &NSdata[1], &NSdata[2]) < 3) break;
    if(fscanf(WE, "%f,%f,%f", &WEdata[0], &WEdata[1], &WEdata[2]) < 3) break;;
    
    if(sampleCount==0) {
      initPose[0] = (NSdata[0] + WEdata[0])/2;
      initPose[1] = (NSdata[1] + WEdata[1])/2;
      initPose[2] = (NSdata[2] + WEdata[2])/2;
      /* Initialize the Kalman Filter */
      initKalmanFilter(&kf,initPose,vel,deltat);
    }
    else {
      rovioKalmanFilter(&kf,  NSdata, WEdata, track);	    
      fprintf(TR, "%f,%f,%f\n", track[0],track[1],track[2]);
    }
  }
  
  fclose(NS);
  fclose(WE);
  fclose(TR);
  return 0;
}
