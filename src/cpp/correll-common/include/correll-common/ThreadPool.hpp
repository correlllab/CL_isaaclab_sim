#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

class ThreadPool {

  public:
    ThreadPool(std::size_t num_threads) {
    
       for (std::size_t i = 0; i < num_threads; ++i) {
        m_threads.emplace_back([this] {
          while (true) {
            std::function<void()> task;

            {
              std::unique_lock<std::mutex> lock(m_queueMutex);
              m_cv.wait(lock, [this] {return !m_tasks.empty() || m_stop;});
            
              if (m_stop && m_tasks.empty()) {
                return;
              
              }

              task = std::move(m_tasks.front());
              m_tasks.pop();
            }
            task();
          }
            
            
        });
      
      } 
    };

    ~ThreadPool() {
      {
        std::unique_lock<std::mutex> lock(m_queueMutex);
        m_stop = true;
      }
      m_cv.notify_all();
      for (auto& thread : m_threads) {
        thread.join();
      }
    };

    void enqueue(std::function<void()> task) {
      {
        std::unique_lock<std::mutex> lock(m_queueMutex);
        m_tasks.emplace(std::move(task));
      }
      m_cv.notify_one();
    }


  private:
    std::vector<std::thread> m_threads;
    std::queue<std::function<void()>> m_tasks; 
    std::mutex m_queueMutex;
    std::condition_variable m_cv;

};

