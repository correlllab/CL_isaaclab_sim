
#include <condition_variable>
#include <functional>
#include <mutex>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {

  public:
    ThreadPool(std::size_t num_threads = std::thread::hardware_concurrency());
    ~ThreadPool();

    void enqueue(std::function<void()> task);

  private:
    std::vector<std::thread> m_threads;
    std::queue<std::function<void()>> m_tasks; 
    std::mutex m_queueMutex;
    std::condition_variable m_cv;
    bool m_stop = false;

};
