#ifndef OPENRAVE_BOOST_COMPAT_H
#define OPENRAVE_BOOST_COMPAT_H

// This is a compatibility file for the transition of some headers and classes between the boost and std namespaces.
// Note for the future: When the time comes for us to fully embrace the std namespace,
// simply delete this header file and add #includes in the other files as necessary.

#if OPENRAVE_USE_STD

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <shared_mutex>
#include <thread>

namespace rstd {

namespace this_thread = ::std::this_thread;

using condition       = ::std::condition_variable;
using mutex           = ::std::mutex;
using once_flag       = ::std::once_flag;
using recursive_mutex = ::std::recursive_mutex;
using shared_mutex    = ::std::shared_mutex;
using thread          = ::std::thread;
using defer_lock_t    = ::std::defer_lock_t;
using try_to_lock_t   = ::std::try_to_lock_t;
using adopt_lock_t    = ::std::adopt_lock_t;

#define BOOST_ONCE_INIT ::std::once_flag()

// NOTE: The argument order between std::call_once and boost::call_once is reversed.
// So we have to follow the Boost convention here.
template <typename Callable>
void call_once(Callable&& f, std::once_flag& flag) {
    std::call_once(flag, f);
}

template <typename Mutex>
using lock_guard = ::std::lock_guard<Mutex>;

template <typename Mutex>
using unique_lock = ::std::unique_lock<Mutex>;

template <typename Mutex>
using shared_lock = ::std::shared_lock<Mutex>;

namespace posix_time {

using microseconds = ::std::chrono::microseconds;
using milliseconds = ::std::chrono::milliseconds;
using seconds      = ::std::chrono::seconds;

} // namespace posix_time

} // namespace boost

#else

#include <boost/thread/condition.hpp>
#include <boost/thread.hpp>

namespace rstd = ::boost;

#endif // OPENRAVE_USE_STD
 
#endif // OPENRAVE_BOOST_COMPAT_H