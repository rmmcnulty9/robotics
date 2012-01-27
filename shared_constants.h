/*
 * Some shared Constants across this project
 * */
 
 
typedef struct{
    float x;
    float y;
    float theta;
}pose;


const float we_to_cm = 1/20.0;
const float ns_to_cm = 1/300.0;

/*
 * FIR Filter Stuff
 * */
 
 
typedef struct {
  float coefficients[30]; //WARNING SHOULD NOT HAVE MORE THAN 30 TAPS/COEF!!!
  int next_sample;
  float samples[30];
  int TAPS;
} filter;

// firFilterCreate()
// creates, allocates,  and iniitializes a new firFilter
 
 
filter *firFilterCreate(char *coef_file)
{
  int i;
  filter *f = malloc(sizeof(filter));
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
   // printf("%f\n", f->coefficients[i]);
    f->TAPS++;
  }
  
//  printf("Coefficients:\n");
//  for (i = 0; i < f->TAPS; i++) {
//    printf("%d: %f\n", i, f->coefficients[i]);
//  }
  
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
