
#pragma once
#include <vector>
#include <unordered_map>
#include <memory>

#include <nvjpeg.h>

enum class Structure {INTERLEAVED, PLANAR};

enum class DataType {DEPTH, RGB};

struct ImageData {

  long dataPtr;
  int width, height;
  Structure structure;
};

class NvjpegEncoder {
  public:
    NvjpegEncoder(DataType dtype);
    ~NvjpegEncoder();

    std::vector<unsigned char> Encode(ImageData data);

  private:
    DataType mInternalDataType;
    nvjpegHandle_t mNvHandle;
    nvjpegEncoderParams_t mNvEncParams;
    nvjpegEncoderState_t mNvState;
    cudaStream_t mCuda = NULL;


};
