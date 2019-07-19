#ifndef ELECTROMAGNETISM_H_
#define ELECTROMAGNETISM_H_

#include "common.h"

#define LEN 10;

extern que elec_que1;
extern que elec_que2;
extern que elec_que3;
extern que elec_que4;

void elec_init(void);
void elec_renew(void);
float elec_deal(que *q);






#endif