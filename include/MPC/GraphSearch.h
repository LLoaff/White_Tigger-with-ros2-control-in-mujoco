#ifndef GRAPHSEARCH_H
#define GRAPHSEARCH_H

#include <vector>


struct ContactState {
  union {
    bool contact[4];
    struct {
      bool fr, fl, rr, rl;
    };
  };

  ContactState(bool _fr, bool _fl, bool _rr, bool _rl) {
    fr = _fr;
    fl = _fl;
    rr = _rr;
    rl = _rl;
  }

  ContactState() { }
};

#endif //CHEETAH_SOFTWARE_GRAPHSEARCH_H
