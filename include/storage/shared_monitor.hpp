#ifndef SHARED_MONITOR_HPP
#define SHARED_MONITOR_HPP

#include "storage/shared_datatype.hpp"

#include <boost/format.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#if defined(__linux__)
#define USE_BOOST_INTERPROCESS_CONDITION 1
#endif

namespace osrm
{
namespace storage
{

namespace
{
namespace bi = boost::interprocess;

template <class Lock> class InvertedLock
{
    Lock &lock;

  public:
    InvertedLock(Lock &lock) : lock(lock) { lock.unlock(); }
    ~InvertedLock() { lock.lock(); }
};
}

// The shared monitor implementation based on a semaphore and mutex
template <typename Data> struct SharedMonitor
{
    using mutex_type = bi::interprocess_mutex;

    SharedMonitor(const Data &initial_data)
    {
        shmem = bi::shared_memory_object(bi::open_or_create, Data::name, bi::read_write);

        bi::offset_t size = 0;
        if (shmem.get_size(size) && size == 0)
        {
            shmem.truncate(internal_size + sizeof(Data));
            region = bi::mapped_region(shmem, bi::read_write);
            new (&internal()) InternalData;
            new (&data()) Data(initial_data);
        }
        else
        {
            region = bi::mapped_region(shmem, bi::read_write);
        }
    }

    SharedMonitor()
    {
        try
        {
            shmem = bi::shared_memory_object(bi::open_only, Data::name, bi::read_write);

            bi::offset_t size = 0;
            if (!shmem.get_size(size) || size != internal_size + sizeof(Data))
            {
                auto message =
                    boost::format("Wrong shared memory block '%1%' size %2%, expected %3% bytes") %
                    (const char *)Data::name % size % (internal_size + sizeof(Data));
                throw util::exception(message.str() + SOURCE_REF);
            }

            region = bi::mapped_region(shmem, bi::read_write);
        }
        catch (const bi::interprocess_exception &exception)
        {
            auto message = boost::format("No shared memory block '%1%' found, have you forgotten "
                                         "to run osrm-datastore?") %
                           (const char *)Data::name;
            throw util::exception(message.str() + SOURCE_REF);
        }
    }

    Data &data() const
    {
        auto region_pointer = reinterpret_cast<char *>(region.get_address());
        return *reinterpret_cast<Data *>(region_pointer + internal_size);
    }

    mutex_type &get_mutex() const { return internal().mutex; }

#if USE_BOOST_INTERPROCESS_CONDITION
    template <typename Lock> void wait(Lock &lock) { internal().condition.wait(lock); }

    void notify_all() { internal().condition.notify_all(); }
#else
    template <typename Lock> void wait(Lock &lock)
    {
        if (((internal().head + 1) & (buffer_size - 1)) == (internal().tail & (buffer_size - 1)))
            throw util::exception(std::string("ring buffer is too small") + SOURCE_REF);

        auto index = (internal().head++) & (buffer_size - 1);
        auto semaphore = new (internal().buffer + index * sizeof(bi::interprocess_semaphore))
            bi::interprocess_semaphore(0);
        {
            InvertedLock<Lock> inverted_lock(lock);
            semaphore->wait();
        }
        semaphore->~interprocess_semaphore();
    }

    void notify_all()
    {
        bi::scoped_lock<mutex_type> lock(internal().mutex);
        while (internal().tail != internal().head)
        {
            auto index = (internal().tail++) & (buffer_size - 1);
            auto semaphore = reinterpret_cast<bi::interprocess_semaphore *>(
                internal().buffer + index * sizeof(bi::interprocess_semaphore));
            semaphore->post();
        }
    }
#endif

    static void remove() { bi::shared_memory_object::remove(Data::name); }

  private:
#if USE_BOOST_INTERPROCESS_CONDITION

    static constexpr int internal_size = 128;

    struct InternalData
    {
        mutex_type mutex;
        bi::interprocess_condition condition;
    };

#else

    static constexpr int buffer_size = 256;
    static constexpr int internal_size = 4 * 4096;

    struct InternalData
    {
        InternalData() : head(0), tail(0){};

        std::size_t head, tail;
        mutex_type mutex;
        char buffer[buffer_size * sizeof(bi::interprocess_semaphore)];
    };

    static_assert(buffer_size >= 2, "buffer size is too small");
    static_assert((buffer_size & (buffer_size - 1)) == 0, "buffer size is not power of 2");

#endif

    static_assert(sizeof(InternalData) <= internal_size, "not enough space to place internal data");
    static_assert(alignof(Data) <= internal_size, "incorrect data alignment");

    InternalData &internal() const
    {
        return *reinterpret_cast<InternalData *>(reinterpret_cast<char *>(region.get_address()));
    }

    bi::shared_memory_object shmem;
    bi::mapped_region region;
};
}
}

#undef USE_BOOST_INTERPROCESS_CONDITION

#endif // SHARED_MONITOR_HPP
