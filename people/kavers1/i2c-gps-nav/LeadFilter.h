#ifndef LeadFilter_h
#define LeadFilter_h

#include "LeadFilter.h"

class LeadFilter {
public:
    LeadFilter() :
        _last_velocity(0) {
    }

    // setup min and max radio values in CLI
    int32_t         get_position(int32_t pos, int16_t vel, float lag_in_seconds = 1.0);
    void            clear() { _last_velocity = 0; }

private:
    int16_t         _last_velocity;

};

#endif
