#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>


#define SIZE   30


uint32_b table[SIZE] = {0};
uint32_b* ptr = table;
float mean_spd = 0;
float mean_trq = 0; 

void add_to_table(uint32_b val){
  *ptr = val;
  ptr = ptr + 1;
  if (ptr > &table[SIZE-1]) ptr = table;
}

uint16_b mean_spd(void){
  float wgt = 0.97;
  mean_spd = (mean_spd * wgt) + (*ptr&(0x0000FFFF))*(1-wgt);
  printf(" mean if %lf\n", mean_spd);
  return (int)mean_spd;
}

uint16_b mean_trq(void){
  float wgt = 0.97;
  mean_trq = (mean_trq * wgt) + (*ptr&(0xFFFF0000)>>16)*(1-wgt);
  printf(" mean if %lf\n", mean_trq);
  return (int)mean_trq;
}
}

void main(void){
  int i;
  uint16_b s = 0;
  uint16_b t = 0;
  for (i=0;i++;i<30){
    add_to_table(rand()%0xFFFFFFFF);
    s = mean_spd();
    t = mean_trq();
  }
  printf("trq ! %d spd %d \n", t, s) 
} 
