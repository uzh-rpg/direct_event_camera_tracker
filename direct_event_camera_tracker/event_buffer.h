#ifndef EVENT_BUFFER_H_INCLUDED
#define EVENT_BUFFER_H_INCLUDED

#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <opencv2/opencv.hpp>

#include "core/camera_intrinsics.h"

typedef std::vector<dvs_msgs::Event> EventVector;

////////////////////////////////////////////////////////////////////////////////

class Eventframe
{
public:
    EventVector::const_iterator from_event, to_event;
    cv::Mat img;
};

////////////////////////////////////////////////////////////////////////////////

/*!
 * This class pulls apart EventArrays and stores them in a single buffer.
 *
 */
class EventBuffer
{
public:
    EventBuffer(const CameraIntrinsics& camera) : camera(camera) {};
    EventBuffer() {};

    void set_camera(const CameraIntrinsics& camera) { this->camera = camera; }

    //! feed in events here
    void callback(const dvs_msgs::EventArray::ConstPtr& ev_array);

    //! find first event >= t
    EventVector::const_iterator find(ros::Time t) const;

    const EventVector& get() const { return data; }

    EventVector::const_iterator begin() const { return data.begin(); }
    EventVector::const_iterator end()   const { return data.end(); }
    size_t size() const { return data.size(); }

    //! use cv::normalize() if you want a unit-norm patch
    Eventframe integrate(EventVector::const_iterator from, EventVector::const_iterator to) const;

    //! integrates given a start time and an event count
    //! if from_center_t is true, then the 'from' time specifies the *middle* of the integration window
    Eventframe integrate(ros::Time from, size_t num, bool from_middle_t=false) const;

    //! plot events on top of image
    void plot(cv::Mat& dst, EventVector::const_iterator from, EventVector::const_iterator to) const;

private:
    EventVector data;
    CameraIntrinsics camera;
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: EVENT_BUFFER_H_INCLUDED */
