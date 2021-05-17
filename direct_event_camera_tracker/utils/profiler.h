#ifndef PROFILER_H_QGAEYWJW
#define PROFILER_H_QGAEYWJW

// based on ros_common/timer.h

#include <chrono>
#include <map>
#include <string>
#include <iostream>

#include "utils/timer.h"

////////////////////////////////////////////////////////////////////////////////

struct MethodInfo
{
    std::chrono::nanoseconds total_duration;
    unsigned int total_calls;
};

class Profiler
{
public:
    // this is a singleton
    static Profiler& getInstance()
    {
        static Profiler instance;
        return instance;
    }

    void addTime(const std::string& name, const std::chrono::nanoseconds& t, bool increment_count = true, int total_count = -1)
    {
        std::map<std::string, MethodInfo>::iterator it = db.find(name);
        if (it != db.end()) {
            it->second.total_duration += t;
            if (total_count <= 0) {
                if (increment_count) {
                    it->second.total_calls++;
                }
            } else {
                it->second.total_calls += total_count;
            }
        } else {
            // not yet in map
            MethodInfo m;
            m.total_duration = t;
            if (total_count <= 0) {
                m.total_calls = 1;
            } else {
                m.total_calls = total_count;
            }
            db[name] = m;

            if (!increment_count) {
                std::cerr << "ERROR: Profiler::addTime(increment_count=false) for unknown timer '" << name << "' called. This should not happen!" << std::endl;
            }
        }
    }

    ~Profiler() {
        std::cout << "Profiling Results:" << std::endl;
        for (std::map<std::string, MethodInfo>::iterator it = db.begin(); it != db.end(); it++) {
            double ms_total = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(it->second.total_duration).count());
            std::cout << it->first << ": \t\t " << ms_total/it->second.total_calls << " ms/call (" << it->second.total_calls << " calls)" << std::endl;
        }
    }

    // don't allow copies
    Profiler(const Profiler&) = delete;
    void operator=(const Profiler&) = delete;

private:
    Profiler() {};
    std::map<std::string, MethodInfo> db;
};

////////////////////////////////////////////////////////////////////////////////
// to profile a method instantiate a ProfilerTimer at the beginning of your block

class ProfilerTimer
{
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;
    typedef std::chrono::nanoseconds Nanoseconds;
public:

    ProfilerTimer(const std::string& name)
        : name(name), start_time(Clock::now()), running(true), first_measurement(true) {}

    ~ProfilerTimer() {
        stop();
    }

    void start()
    {
        if (running) {
            // do NOT register time
            //stop();
        }

        start_time = Clock::now();
        running = true;
    }

    std::chrono::nanoseconds stop(int total_count = -1)
    {

        if (running) {
            // the "magic" is in here ;)
            const TimePoint end_time(Clock::now());

            std::chrono::nanoseconds dt = std::chrono::duration_cast<Nanoseconds>(end_time - start_time);

            Profiler::getInstance().addTime(name, dt, first_measurement, total_count);
            running = false;
            first_measurement = false;

            return dt;
        }

        return std::chrono::nanoseconds(0);
    }

    // stop but don't register
    void cancel()
    {
        running = false;
    }

private:

    std::string name;
    TimePoint start_time;
    bool running;
    bool first_measurement; // allows multiple sections in single block that count as one
};

////////////////////////////////////////////////////////////////////////////////

#endif /* end of include guard: PROFILER_H_QGAEYWJW */
