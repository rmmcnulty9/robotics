/*
 * Used to simulate data sets with different FIR filter taps and coef
 * simulator <data_file> <coef_file>
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include "shared_constants.h"
#include <math.h>
 

typedef struct {
  float coefficients[30]; //WARNING SHOULD NOT HAVE MORE THAN 30 TAPS/COEF!!!
  int next_sample;
  float samples[30];
  int TAPS;
} filter;

 
filter *firFilterCreate(char *coef_file, float init_val)
{
  int i;
  filter *f = (filter *)malloc(sizeof(filter));
  //printf("%d\n", sizeof(filter));
  f->TAPS = 0;
  f->next_sample = 0;
  FILE *fp = fopen(coef_file,"r+");
  if(fp==NULL){
    printf("Coefficients could not be loaded from %s\n", coef_file);
    exit(-1);
  }
  
  //Read in coef & count, for TAPS
  for (i = 0; i < 30; i++){
    f->samples[i] = 0;
    if(1!=fscanf(fp,"%e ", &f->coefficients[i])){
      fclose(fp);
      break;
    }
    f->TAPS++;
  }
  
  return f;
}

// firFilter 
//inputs take a filter (f) and the next sample (val)
//returns the next filtered sample
//incorporates new sample into filter data array
 

float firFilter(filter *f, float val)
{
  float sum =0;
  int i,j;

  // assign  new value to "next" slot 
  f->samples[f->next_sample] = val;

  // calculate a  weighted sum
  //   i tracks the next coeficeint
  //   j tracks the samples w/wrap-around 
  for( i=0,j=f->next_sample; i<f->TAPS; i++) {
    sum += f->coefficients[i]*f->samples[j++];
    if(j == f->TAPS)  j=0;
  }
  if(++(f->next_sample) == f->TAPS) f->next_sample = 0;
  return(sum);
}

int main(int argc, char **argv) {

        // Make sure we have a valid command line argument
        if(argc != 3) {
              printf("Usage: simulator <data_file> <coef_file>\n");
              exit(-1);
        }

        char coef[256];
        strcpy(coef,argv[1]);
        FILE *f = fopen(argv[2],"r");
        if(NULL==f){
          printf("Could not open the data set\n");
        }
        
        filter *fir_x = firFilterCreate(coef,0 );
	//printf("derp %d\n", fir_x->next_sample);
        filter *fir_y = firFilterCreate(coef,0);
        filter *fir_theta = firFilterCreate(coef,0);
        
        filter *fir_left = firFilterCreate(coef,0);
        filter *fir_right = firFilterCreate(coef,0);
        filter *fir_rear = firFilterCreate(coef,0);
        
        //Variables from the data set
        int d_left=0, d_right=0, d_rear=0.0;
        int t_left=0, t_right=0, t_rear=0.0;
        int x=0, y=0;
        float theta = 0.0;
        
         //Variables created by FIR filter
        int f_d_left=0, f_d_right=0, f_d_rear=0;
        int f_t_left=0, f_t_right=0, f_t_rear=0;
        int f_x=0, f_y=0;
        float f_theta = 0.0, prev_theta = 0.0, total_theta=0.0, delta_theta=0.0, norm_theta=0.0;
        int pi_ctr=0;
        // Action loop
        while(9==fscanf(f,"%d %d %f %d %d %d %d %d %d\n", 
          &x, &y, &theta,
          &d_left, &d_right, &d_rear, 
          &t_left, &t_right, &t_rear)) {
          
           
          delta_theta = theta-prev_theta;
          
          if(abs(delta_theta)>(3*M_PI_2) && prev_theta>0 && theta<0){
            delta_theta+=(2*M_PI);
            pi_ctr+=1;
          }
          else if(abs(delta_theta)>(3*M_PI_2) && theta>0 && prev_theta<0){
            delta_theta-=(2*M_PI);
            pi_ctr-=1;
          } 
          
            prev_theta=theta;
            
           // total_theta += delta_theta;
            
                f_d_left = (int)firFilter(fir_left, (float)d_left);
                f_d_right = (int)firFilter(fir_right, (float)d_right);
                f_d_rear = (int)firFilter(fir_rear, (float)d_rear);

                f_t_left +=f_d_left;
                f_t_right +=f_d_right;
                f_t_rear +=f_d_rear;

               f_x = (int)firFilter(fir_x, (float)x);
               f_y = (int)firFilter(fir_y, (float)y);
               f_theta = firFilter(fir_theta, theta+(pi_ctr*2*M_PI));
               
               //put into range of -pi to +pi - "normalize"
               
                 // norm_theta = f_theta/(2*M_PI);
                 //norm_theta = ((int)(f_theta*100)%(2*314))/100.0;
                 // if(norm_theta>M_PI){
                 //   norm_theta -=(2*M_PI);
                 // }
                norm_theta = (f_theta-(pi_ctr*2*M_PI));
               
               printf("%d %d %f %d %d %d %d %d %d\n", f_x, f_y, norm_theta, f_d_left, f_d_right, f_d_rear, f_t_left, f_t_right, f_t_rear);
        }
        fclose(f);

        return 0;
}
