
#include <semaphore>
#include <correll-common/ThreadSafeDeque.hpp>

struct Image {
  long dataPtr;
  uint8_t width, height;

  Image(long dP, uint8_t w, uint8_t h) : dataPtr(dP), width(w), height(h) {};

}

class ImageDeque {
  TQueueConcurrent<Image> deque; 
  std::counting_semaphore<> semaphore{0};

  public:

    void PopBackDeque();
    void EmplaceFrontDeque();
    void AcquireCountingSemaphore();
    void ReleaseCountingSemaphore();
}


