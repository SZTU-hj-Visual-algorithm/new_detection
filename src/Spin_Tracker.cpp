#include "Spin_Tracker.h"

SpinTracker::SpinTracker(const Armor& src, chrono_time src_timestamp)
{
    last_armor = src;
    last_timestamp = src_timestamp;
    is_initialized = false;
}


bool SpinTracker::update_tracker(const Armor& new_armor, chrono_time new_timestamp)
{
    is_initialized = true;
    last_armor = new_armor;
    last_timestamp = new_timestamp;
    return true;
}


