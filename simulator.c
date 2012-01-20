/*
 * Used to simulate data sets with different FIR filter taps and coef
 * simulator <data_file> <coef_file>
 */

#include <robot_if.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>


int TAPS = 0 // how many filter taps

typedef struct 
{
  float coefficients[TAPS];
  unsigned  next_sample;
  float samples[TAPS];
} filter;

/* firFilterCreate()
 * creates, allocates,  and iniitializes a new firFilter
 */
filter *firFilterCreate(char *coef_file)
{
  int i;
  filter *f = malloc(sizeof(filter));
  FILE *fp = fopen(coef_file,"r+");
  if(fp==NULL){
    printf("Coefficients could not be loaded from %s\n", coef_file);
    exit(-1);
  }
  
  //Read in coef & count, for TAPS
  while(1){
    f->samples[i] = 0;
    if(1!=fscanf(fp,"%e ", &f->coefficients[i])){
      break;
    }
    TAPS+=1;
  }
   
  f->next_sample = 0;
}

/* firFilter 
 * inputs take a filter (f) and the next sample (val)
 * returns the next filtered sample
 * incorporates new sample into filter data array
 */

float firFilter(filter *f, float val)
{
  float sum =0;
  int i,j;

  /* assign  new value to "next" slot */
  f->samples[f->next_sample] = val;

  /* calculate a  weighted sum
     i tracks the next coeficeint
     j tracks the samples w/wrap-around */
  for( i=0,j=f->next_sample; i<TAPS; i++) {
    sum += f->coefficients[i]*f->samples[j++];
    if(j==TAPS)  j=0;
  }
  if(++(f->next_sample) == TAPS) f->next_sample = 0;
  return(sum);
}


int main(int argc, char **argv) {
        robot_if_t ri;

        // Make sure we have a valid command line argument
        if(argc != 3) {
                printf("Usage: simulator <data_file> <coef_file>\n");
                exit(-1);
        }

        
        char coef[256];
        strcpy(coef,argv[2]);
        FILE *f = fopen(arg[3],"r");
        if(NULL==f){
          printf("Could not open the data set\n");
        }
        
        filter *fir_x = firFilterCreate(coef);
        filter *fir_y = firFilterCreate(coef);
        filter *fir_theta = firFilterCreate(coef);
        filter *fir_left = firFilterCreate(coef);
        filter *fir_right = firFilterCreate(coef);
        filter *fir_rear = firFilterCreate(coef);
        
        //Variables from the data set
        int d_left=0, d_right=0, d_rear=0.0;
        int t_left=0, t_right=0, t_rear=0.0;
        
        int x=0, y=0;
        float theta = 0.0;
        
         //Variables created by FIR filter
        int fir_d_left=0, fir_d_right=0, fir_d_rear=0.0;
        int fir_t_left=0, fir_t_right=0, fir_t_rear=0.0;
        
        int fir_x=0, fir_y=0;
        float fir_theta = 0.0;
        
        // Action loop
        fscanf(f,"%d %d %f %d %d %d %d %d %d\n", x, y, theta, d_left, d_right, d_rear, t_left, t_right, t_rear" {
               
               // printf("N %ld.%ld %d %d %f\n",now.tv_sec, now.tv_usec, ri_getX(&ri), ri_getY(&ri), ri_getTheta(&ri));
                d_left = firFilter(&fir_left, ri_getWheelEncoder(&ri,RI_WHEEL_LEFT));
                d_right = firFilter(&fir_right, ri_getWheelEncoder(&ri,RI_WHEEL_RIGHT);
                d_rear = firFilter(&fir_rear, ri_getWheelEncoder(&ri,RI_WHEEL_REAR);
                t_left +=d_left;
                t_right +=d_right;
                t_rear +=d_rear;
               
               x = firFilter(&fir_x, ri_getX(&ri));
               y = firFilter(&fir_y, ri_getY(&ri);
               theta = firFilter(&fir_theta, ri_getTheta(&ri));
               
               printf("%d %d %f %d %d %d %d\n", x, y, theta, d_left, d_right, d_rear, t_left, t_right, t_rear);
           
        } while(1);

        return 0;
}
