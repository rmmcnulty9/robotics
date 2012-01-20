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


int TAPS = 0; // how many filter taps

typedef struct 
{
  float coefficients[20]; //WARNING SHOULD NOT HAVE MORE THAN 20 TAPS/COEF!!!
  unsigned  next_sample;
  float samples[20];
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
        FILE *f = fopen(argv[3],"r");
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
        int f_d_left=0, f_d_right=0, f_d_rear=0.0;
        int f_t_left=0, f_t_right=0, f_t_rear=0.0;
        int f_x=0, f_y=0;
        float f_theta = 0.0;
        
        // Action loop
        while(9==fscanf(f,"%d %d %f %d %d %d %d %d %d\n", 
          &x, &y, &theta,
          &d_left, &d_right, &d_rear, 
          &t_left, &t_right, &t_rear)) {
               
               // printf("N %ld.%ld %d %d %f\n",now.tv_sec, now.tv_usec, ri_getX(&ri), ri_getY(&ri), ri_getTheta(&ri));
                f_d_left = firFilter(fir_left, d_left);
                f_d_right = firFilter(fir_right, d_right);
                f_d_rear = firFilter(fir_rear, d_rear);
                f_t_left +=f_d_left;
                f_t_right +=f_d_right;
                f_t_rear +=f_d_rear;
               
               f_x = firFilter(fir_x, x);
               f_y = firFilter(fir_y, y);
               f_theta = firFilter(fir_theta,theta);
               
               printf("%d %d %f %d %d %d %d\n", f_x, f_y, f_theta, f_d_left, f_d_right, f_d_rear, f_t_left, f_t_right, f_t_rear);
           
        } while(1);

        return 0;
}
