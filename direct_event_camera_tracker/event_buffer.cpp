#include "event_buffer.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

void EventBuffer::callback(const dvs_msgs::EventArray::ConstPtr& ev_array)
{
    // append data
    data.insert(data.end(), ev_array->events.begin(), ev_array->events.end());
}

////////////////////////////////////////////////////////////////////////////////

EventVector::const_iterator EventBuffer::find(ros::Time t) const
{
    return
    lower_bound(data.begin(), data.end(), t,
            [](const dvs_msgs::Event& event, const ros::Time& stamp){
                return event.ts < stamp;
            });
}

////////////////////////////////////////////////////////////////////////////////

    //! use cv::normalize() if you want a unit-norm patch
Eventframe EventBuffer::integrate(EventVector::const_iterator from, EventVector::const_iterator to) const
{
    Eventframe ef;
    ef.from_event = from;
    ef.to_event   = to;
    ef.img = cv::Mat(camera.getCameraHeight(), camera.getCameraWidth(), CV_64FC1, cv::Scalar(0));

    // integrate
    for (EventVector::const_iterator it = from; it != to; it++) {
        assert(it->x < ef.img.cols);
        assert(it->y < ef.img.rows);

        if (it->x >= ef.img.cols || it->y >= ef.img.rows) {
            cerr << "WARNING: Ignoring out of bounds event at " << it->x << ", " << it->y << endl;
        } else {
            ef.img.at<double>(cv::Point(it->x, it->y)) += (it->polarity?1:-1);
        }
    }

    return ef;
}

////////////////////////////////////////////////////////////////////////////////

Eventframe EventBuffer::integrate(ros::Time from, size_t num, bool from_middle_t) const
{
    if (data.size() == 0) {
        throw "ERROR: cannot integrate events: Buffer is empty!";
    }

    EventVector::const_iterator from_it = find(from);
    if (from_it == data.end()) {
        cerr << "ERROR: invalid integration window requested: start time is " << from << ", but our data goes from " << data.front().ts << " to " << data.back().ts << endl;
        throw "start time not yet available";
    }

    if (from_middle_t) {
        if (distance((EventVector::const_iterator) data.begin(), from_it) < num/2) {
            throw "start time not available";
        }

        from_it -= num/2; // retreat to first event
    }

    EventVector::const_iterator to_it = from_it;

    if (distance(to_it, (EventVector::const_iterator) data.end()) < num) {
        throw "end time not yet available";
    }

    // advance to last event
    to_it += num;

    return integrate(from_it, to_it);
}

////////////////////////////////////////////////////////////////////////////////

void EventBuffer::plot(
        cv::Mat& dst,
        EventVector::const_iterator from,
        EventVector::const_iterator to) const
{
    assert(!dst.empty());

    if (dst.channels() == 1) {
        //cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR); // this only works for uint8 and floats, but not for doubles -.-
        cv::Mat tmp[] = {dst, dst, dst};
        cv::merge(tmp, 3, dst);
    }


    double scale_x = dst.cols / double(camera.getCameraWidth());
    double scale_y = dst.rows / double(camera.getCameraHeight());

    if (dst.depth() == CV_8U) {
        if (scale_x > 1 and scale_y > 1) {
            for (EventVector::const_iterator it = from; it != to; it++) {
                int x0 = it->x*scale_x;
                int y0 = it->y*scale_y;
                for (int y = 0; y < int(scale_y); y++) {
                    for (int x = 0; x < int(scale_x); x++) {
                        dst.at<cv::Vec3b>(y0+y, x0+x)
                            = it->polarity ? cv::Vec3b(255,0,0) : cv::Vec3b(0,0,255);
                    }
                }
            }
        } else {
            for (EventVector::const_iterator it = from; it != to; it++) {
                dst.at<cv::Vec3b>(cv::Point(it->x*scale_x, it->y*scale_y))
                    = it->polarity ? cv::Vec3b(255,0,0) : cv::Vec3b(0,0,255);
            }
        }
    } else {
        for (EventVector::const_iterator it = from; it != to; it++) {
            /*
            cv::Vec3b& v = dst.at<cv::Vec3b>(cv::Point(it->x, it->y));
            v[it->polarity?0:2] = 255;
            v[1] = 0; // make it a bit more visible
            */

            dst.at<cv::Vec3d>(cv::Point(it->x*scale_x, it->y*scale_y))
                = it->polarity ? cv::Vec3d(1,0,0) : cv::Vec3d(0,0,1);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
