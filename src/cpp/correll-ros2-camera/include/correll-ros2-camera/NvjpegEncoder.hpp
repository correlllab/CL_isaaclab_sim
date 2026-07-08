
#pragma once
#include <vector>
#include <unordered_map>
#include <memory>

#include <nvjpeg.h>

struct ImageData {

  long dataPtr;
  int width, height;
};



enum class DataType {DEPTH, RGB};

class NvjpegEncoder {
  public:
    NvjpegEncoder(DataType dtype);
    ~NvjpegEncoder();

    uint8_t Encode(ImageData data);

  private:
    DataType mInternalDataType;
    nvjpegHandle_t mNvHandle;
    nvjpegEncoderParams_t mNvEncParams;
    nvjpegEncoderState_t mNvState;
    cudaStream_t mCuda = NULL;


};
