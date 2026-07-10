
#include <vector>
#include <unordered_map>
#include <memory>
#include <iostream>
#include <cstring>

#include <turbojpeg.h>

enum class Structure {INTERLEAVED, PLANAR};

enum class DataType {DEPTH, RGB};

struct ImageData {

  long dataPtr;
  int width, height;
  Structure structure;
};

class TurboEncoder {
  public:
    TurboEncoder(DataType dtype);
    ~TurboEncoder();

    std::vector<unsigned char> Encode(ImageData data);

  private:
    DataType mInternalDataType;
    tjhandle mTurboJpegHandle;
    //nvjpegHandle_t mNvHandle;
    //nvjpegEncoderParams_t mNvEncParams;
    //nvjpegEncoderState_t mNvState;
    //cudaStream_t mCuda = NULL;


};
