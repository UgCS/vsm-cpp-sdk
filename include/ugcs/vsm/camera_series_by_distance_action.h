// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

/**
 * @file camera_control_action.h
 */
#ifndef _UGCS_VSM_CAMERA_SERIES_BY_DISTANCE_ACTION_H_
#define _UGCS_VSM_CAMERA_SERIES_BY_DISTANCE_ACTION_H_

#include <ugcs/vsm/action.h>
#include <ugcs/vsm/mavlink.h>

namespace ugcs {
namespace vsm {

/** Performs a series of consequent camera shots in a fixed time intervals. */
class Camera_series_by_distance_action: public Action {
    DEFINE_COMMON_CLASS(Camera_series_by_distance_action, Action)

public:
    Camera_series_by_distance_action(double interval,
                                     Optional<int> count,
                                     std::chrono::milliseconds initial_delay):
        Action(Type::CAMERA_SERIES_BY_DISTANCE),
        interval(interval),
        count(count),
        initial_delay(initial_delay)
    {}
    /**
     * Construct action from protobuf command.
     */
    Camera_series_by_distance_action(const Property_list& p) :
        Action(Type::CAMERA_SERIES_BY_DISTANCE)
    {
        int tmp;
        float time;
        if (p.at("count")->Get_value(tmp)) {
            count = tmp;
        }
        p.at("distance")->Get_value(interval);
        p.at("delay")->Get_value(time);
        tmp = time * 1000;
        initial_delay = std::chrono::milliseconds(tmp);
    }

    /** Distance interval between two consequent shots in meters. */
    double interval;
    /** Total number of shots to perform. */
    Optional<int> count;
    /** Initial delay. Time interval between the command has been accepted by
     * the system and shooting the first picture.
     */
    std::chrono::milliseconds initial_delay;
};

/** Type mapper for camera control action. */
template<>
struct Action::Mapper<Action::Type::CAMERA_SERIES_BY_DISTANCE> {
    /** Real type. */
    typedef Camera_series_by_distance_action type;
};

} /* namespace vsm */
} /* namespace ugcs */

#endif /* _UGCS_VSM_CAMERA_SERIES_BY_DISTANCE_ACTION_H_ */
