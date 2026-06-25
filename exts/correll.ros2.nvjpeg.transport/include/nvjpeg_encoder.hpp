#pragma once
#include <nvjpeg.h>
#include <vector>
#include <thread_pool.hpp>
#include <unordered_map>


class nvjpegEncoder {

  public:
    nvjpegEncoder();
    ~nvjpegEncoder();

    void encodeImageBuffer(long dataPtr, int width, int height, std::string topicName, std::vector<unsigned char>& jpegBuffer);

  
    //void testicles(long dataPtr, int width, int height);
    
    std::unordered_map<std::string, nvjpegEncoderState_t> m_stateMap;
    nvjpegHandle_t m_nvHandle;
    nvjpegEncoderState_t m_nvEncState;
    nvjpegEncoderParams_t m_nvEncParams;
    cudaStream_t m_cudaStream = NULL;
    nvjpegImage_t m_nvImage;
    nvjpegStatus_t m_latestCudaError;
    ThreadPool m_threadPool;

};
