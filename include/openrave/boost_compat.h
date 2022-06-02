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

using namespace ::std;
namespace posix_time = ::std::chrono;

using condition = ::std::condition_variable;

// NOTE: The argument order between std::call_once and boost::call_once is reversed.
// So we have to follow the Boost convention here.
template <typename Callable>
void call_once(Callable&& f, std::once_flag& flag) {
    std::call_once(flag, f);
}

#define BOOST_ONCE_INIT ::std::once_flag()

} // namespace

#else

#include <boost/thread/condition.hpp>
#include <boost/thread.hpp>

namespace rstd = ::boost;

#endif // OPENRAVE_USE_STD
 
#endif // OPENRAVE_BOOST_COMPAT_H