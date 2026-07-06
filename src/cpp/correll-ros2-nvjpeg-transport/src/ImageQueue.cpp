#include <correll-ros2-nvjpeg-transport/ImageQueue.hpp>

void ImageDeque::PopBackDeque() {
  deque.pop_back();
}

void ImageDeque::EmplaceFrontDeque(Image img) {
  deque.emplace_front(img);
}

void ImageDeque::AcquireCountingSemaphore() {
  semaphore.acquire();
}

void ImageDeque::ReleaseCountingSemaphore() {
  semaphore.release()
}
