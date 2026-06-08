#ifndef LOWPASSFILTER
#define LOWPASSFILTER
#include <math.h>

class LPFilter{
public:
    LPFilter(double samplePeriod, double cutFrequency);
    ~LPFilter();
    void addValue(double newValue);
    double getValue();
    void clear();
private:
    double _weight;
    double _pastValue;
    bool _start;
};

#endif  // LOWPASSFILTER