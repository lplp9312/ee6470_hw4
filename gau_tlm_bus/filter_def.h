#ifndef FILTER_DEF_H_
#define FILTER_DEF_H_

// gau_blur mask parameters
const int MASK_X = 3;
const int MASK_Y = 3;

// gau_blur Filter inner transport addresses
// Used between blocking_transport() & do_filter()
const int GAU_FILTER_R_ADDR = 0x00000000;
const int GAU_FILTER_RESULT_ADDR = 0x00000004;

union word {
  int sint;
  unsigned int uint;
  unsigned char uc[4];
};

// gau_blur mask
const double mask[MASK_X][MASK_Y] = {0.077847, 0.123317, 0.077847, 0.123317, 0.195346, 0.123317, 0.077847, 0.123317, 0.077847};

#endif
