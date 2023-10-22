#ifndef __LGE_NFC_H
#define __LGE_NFC_H

enum NfcChip {
  PN553 = 0,
  SN100X = 1
};

struct matching_t {
  enum NfcChip chip;
  int sku;
};
int isDriverAvailable(enum NfcChip target);
#endif /* __LGE_NFC_H */
