#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>

#define SIZE   30


uint32_t table[SIZE] = {0};
uint32_t* ptr = table;
uint32_t new_th = 0;
uint32_t old_th = 0 ;
float mean_s = 0;
float mean_t = 0; 

void add_to_table(uint32_t val){
  //out_val = *ptr;
  //new_val = val;
  *ptr = val;
  ptr = ptr + 1;
  if (ptr > &table[SIZE-1]) ptr = table;
}

/*
uint16_t mean_spd(void){
  mean_s = mean_s + (((new_val&0x0000FFFF)-(out_val&0x0000FFFF))*0.03);
  printf(" s_mean %X \t", (int)mean_s);
  return mean_s;
}
*/

uint16_t mean_spd_simple(void){
  long int sum = 0;
  float m = 0;
  int i;
  for (i=0; i<SIZE; i++){
    sum += (table[i]&0x0000FFFF);
  }
  m = sum/30;
  //printf(" s_mean %X \t", (int)m);
  return m;
}

uint16_t mean_trq_simple(void){
  long int sum = 0;
  float m = 0;
  int i;
  for (i=0; i<SIZE; i++){
    sum += ((table[i]&0xFFFF0000)>>16);
  }
  m = sum/30;
  //printf(" t_mean %X ", (int)m);
  return m;
}
/*
uint16_t mean_trq(void){
  mean_t = mean_t + ((((new_val&0xFFFF0000)>>16)-((out_val&0xFFFF0000)>>16))*0.03);
  printf(" t_mean %X \n", (int)mean_t);
  return mean_t;
}
*/

void main(void){
  FILE *f;
  int i;
  uint32_t s = 0;
  uint32_t t = 0;

  int assistance = 4;
  float k = 0.25;
  int maxSpeed = 0x40;

  remove("data.txt");
  f = fopen("data.txt", "a");
  
  fprintf(f, "coef %lf Assis %d\n", k,assistance);
  
  for (i=0;i<200;i++){
    add_to_table(maxSpeed);
    s = mean_spd_simple();
    fprintf(f,"%d - ",s);
    new_th = s*assistance*k;
    fprintf(f,"%d ",old_th);
    if (new_th>old_th) {
      old_th = old_th + (new_th - old_th)/3;
    }
    else{
      old_th = (old_th - new_th)/2;
    }
    fprintf(f,"- %d - %d\n",new_th,old_th);
    
     //for(i=0;i<30;i++){
     //  printf("- %x \n",table[i]);
     //}
     //printf("trq ! %X spd %X \n", t, s);
  }
  fclose(f);
}
