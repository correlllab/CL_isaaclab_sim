#pragma once
#include <nvjpeg.h>
#include <vector>
//#include <correll-common/ThreadPool.hpp>
#include <iostream>
#include <unordered_map>

#include <pybind11/stl.h>

class nvjpegEncoder {

  public:
    nvjpegEncoder(std::vector<std::string> topicNames);
    ~nvjpegEncoder();

    void encodeImageBuffer(long dataPtr, int width, int height, std::string topicName, std::vector<unsigned char>& jpegBuffer);


    std::unordered_map<std::string, nvjpegEncoderState_t> m_stateMap;
    nvjpegHandle_t m_nvHandle;
    nvjpegEncoderState_t m_nvEncState;
    nvjpegEncoderParams_t m_nvEncParams;
    cudaStream_t m_cudaStream = NULL;
    nvjpegImage_t m_nvImage;
    nvjpegStatus_t m_latestCudaError;
    std::vector<std::string> m_topicNames;

};
