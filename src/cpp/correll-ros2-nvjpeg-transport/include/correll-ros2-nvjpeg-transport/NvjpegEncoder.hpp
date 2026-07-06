#pragma once
#include <nvjpeg.h>
#include <vector>
//#include <correll-common/ThreadPool.hpp>
#include <iostream>
#include <unordered_map>

#include <pybind11/stl.h>

enum class StreamType {DEPTH, RGB};

class NvjpegEncoder {

  public:
    nvjpegEncoder(StreamType type, std::vector<std::string> names);
    ~nvjpegEncoder();

    void encodeImageBuffer(Image img, std::string topic, std::vector<unsigned char>& jpegBuffer);

    std::unordered_map<std::string, nvjpegEncoderState_t> mStates;

    nvjpegHandle_t mNvHandle;
    nvjpegEncoderParams_t mNvEncParams;
    cudaStream_t mCuda = NULL;
    //cant be nullptr bc of aliasing
    


    //std::unordered_map<std::string, nvjpegEncoderState_t> m_stateMap;
    //nvjpegHandle_t m_nvHandle;
    //nvjpegEncoderState_t m_nvEncState;
    //nvjpegEncoderParams_t m_nvEncParams;
    //cudaStream_t m_cudaStream = NULL;
    //nvjpegImage_t m_nvImage;
    //nvjpegStatus_t m_latestCudaError;
    //std::vector<std::string> m_topicNames;

};
