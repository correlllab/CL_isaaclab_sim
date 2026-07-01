
//shoutout to christophe meneboeuf for this thread safe deque implementation
#include <deque>
#include <mutex>
#include <condition_variable>

template< typename T >
class TQueueConcurrent {

    using const_iterator = typename std::deque<T>::const_iterator;

public:

    template<typename... Args>
    void emplace_front( Args&&... args )
    {
        addData_protected( [&] {
            _collection.emplace_front(std::forward<Args>(args)...);
        } );
    }

    //template<typename... Args>
    //void emplace_back( Args&&... args )
    //{
    //    addData_protected( [&] {
    //        _collection.emplace_back(std::forward<Args>(args)...);
    //    } );
    //}

    T pop_front( void ) noexcept
    {
        std::unique_lock<std::mutex> lock{_mutex};
        while (_collection.empty()) {
            _condNewData.wait(lock);
        }
        auto elem = std::move(_collection.front());
        _collection.pop_front();
        return elem;
    }

    T pop_back( void ) noexcept
    {
        std::unique_lock<std::mutex> lock{_mutex};
        while (_collection.empty()) {
            _condNewData.wait(lock);
        }
        auto elem = std::move(_collection.back());
        _collection.pop_back();

        std::cout << "just popped: {";

        for (const auto& numb : _collection) {
          std::cout << numb << ", ";
        }
        std::cout << "}" << "\n";
        return elem;
    }



private:

    template<class F>
    void addData_protected(F&& fct)
    {
        std::unique_lock<std::mutex> lock{ _mutex };
        fct();
        lock.unlock();
        _condNewData.notify_one();
    }

    std::deque<T> _collection;                     

    std::mutex   _mutex;                    

    std::condition_variable _condNewData;

};
