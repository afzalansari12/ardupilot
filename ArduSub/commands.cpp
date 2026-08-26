#include "Sub.h"

// checks if we should update ahrs/RTL home position from the EKF
void Sub::update_home_from_EKF()
{
    // exit immediately if home already set
    if (ahrs.home_is_set()) {
        return;
    }
    if (!set_home_to_current_location(false)) {
        // ignore this failure
    }
}

// set_home_to_current_location - set home to current GPS location
bool Sub::set_home_to_current_location(bool lock)
{
    // get current location from EKF
    Location temp_loc;
    if (ahrs.get_location(temp_loc)) {

        update_surface_alt();

        // Make home always at the water's surface.
        // This allows disarming and arming again at depth.
        // This also ensures that mission items with relative altitude frame, are always
        // relative to the water's surface, whether in a high elevation lake, or at sea level.
        temp_loc.offset_up_m(-barometer.get_altitude());
        return set_home(temp_loc, lock);
    }
    return false;
}

// set_home - sets ahrs home (used for RTL) to specified location
//  returns true if home location set successfully
bool Sub::set_home(const Location& loc, bool lock)
{
    // check if EKF origin has been set
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // set ahrs home (used for RTL)
    if (!ahrs.set_home(loc)) {
        return false;
    }

    // lock home position
    if (lock) {
        ahrs.lock_home();
    }

    // return success
    return true;
}

void Sub::update_surface_alt()
{
    Location cur_loc;
    if (ahrs.get_location(cur_loc)) {
        cur_loc.offset_up_m(-barometer.get_altitude());
        int32_t alt_cm;
        if (cur_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
            surface_alt_cm = alt_cm;
            surface_alt_set = true;
        }
    }
}

bool Sub::get_surface_alt_cm(int32_t &alt_cm) const
{
    if (surface_alt_set) {
        alt_cm = surface_alt_cm;
        return true;
    }
    if (ahrs.home_is_set()) {
        const Location &home_loc = ahrs.get_home();
        return home_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm);
    }
    return false;
}

void Sub::set_surface_alt_cm(int32_t alt_cm)
{
    surface_alt_cm = alt_cm;
    surface_alt_set = true;
}

bool Sub::get_surface_location(Location &loc) const
{
    if (ahrs.get_location(loc)) {
        int32_t surf_alt;
        if (get_surface_alt_cm(surf_alt)) {
            loc.set_alt_cm(surf_alt, Location::AltFrame::ABSOLUTE);
            return true;
        }
    }
    return false;
}
