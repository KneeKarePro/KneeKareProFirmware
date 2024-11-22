#include "utils.h"

uint16_t convertToAngle(uint16_t rawValue) {
  return map(rawValue, 0, 4095, 0, 180);
}