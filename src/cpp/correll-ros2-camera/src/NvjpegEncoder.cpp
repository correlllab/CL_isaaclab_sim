

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <correll-ros2-camera/NvjpegEncoder.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>


NvjpegEncoder::NvjpegEncoder(DataType dtype) {
  nvjpegCreateSimple(&mNvHandle);
  nvjpegEncoderStateCreate(mNvHandle, &mNvState, mCuda);
  nvjpegEncoderParamsCreate(mNvHandle, &mNvEncParams, mCuda);
  if (dtype == DataType::DEPTH) {
    nvjpegEncoderParamsSetSamplingFactors(mNvEncParams, NVJPEG_CSS_GRAY, mCuda);
    //subsampling leads to slightly darker grayscale images, not sure really how to fix without significantly more effort
  } else if (dtype == DataType::RGB) {
  
    nvjpegEncoderParamsSetSamplingFactors(mNvEncParams, NVJPEG_CSS_444, mCuda);
    
  }

}



NvjpegEncoder::~NvjpegEncoder() {
  nvjpegEncoderParamsDestroy(mNvEncParams);
  nvjpegDestroy(mNvHandle);

}

std::vector<unsigned char> NvjpegEncoder::Encode(ImageData data) {
  nvjpegStatus_t err;
  unsigned char* dataPtr = reinterpret_cast<unsigned char*>(data.dataPtr);
  nvjpegImage_t nvImage;
  //std::vector<unsigned char> gBuffer(data.width);
  //std::vector<unsigned char> bBuffer(data.width);
  //void** devPtr0;
  //cudaMalloc(devPtr0, data.width);

  //void** devPtr1;
  //cudaMalloc(devPtr1, data.width);

  //for (int i = 0; i < data.width; i++) {
  //  gBuffer[i]=0;
  //  bBuffer[i]=0;
  //
  //}

  //cudaMemcpy(devPtr0, gBuffer.data(), data.width, cudaMemcpyHostToDevice);
  //cudaMemcpy(devPtr1, bBuffer.data(), data.width, cudaMemcpyHostToDevice);

  if (data.structure == Structure::INTERLEAVED) {
    std::cout << "INTERLEAVED" << "\n";
    //if interleaved
    nvImage.channel[0] = dataPtr;
    nvImage.pitch[0] = data.width * 3;
    nvImage.channel[1] = NULL;
    nvImage.channel[2] = NULL;
    nvImage.channel[3] = NULL;

    nvjpegStatus_t err = nvjpegEncodeImage(mNvHandle, mNvState, mNvEncParams, &nvImage, NVJPEG_INPUT_RGBI, data.width, data.height, mCuda);
  
  } else if (data.structure == Structure::PLANAR) {
    std::cout << "PLANAR" << "\n";
    nvImage.channel[0] = dataPtr;
    nvImage.channel[1] = dataPtr + data.width * data.height;
    nvImage.channel[2] = dataPtr + 2* data.width * data.height;
    nvImage.channel[3] = NULL;
    //nvImage.channel[1] = dataPtr;
    //nvImage.channel[2] = dataPtr;
    
    nvImage.pitch[0] = data.width; 
    nvImage.pitch[1] = data.width; 
    nvImage.pitch[2] = data.width;
    nvjpegStatus_t err = nvjpegEncodeImage(mNvHandle, mNvState, mNvEncParams, &nvImage, NVJPEG_INPUT_RGB, data.width, data.height, mCuda);
    //nvjpegStatus_t err = nvjpegEncodeYUV(mNvHandle, mNvState, mNvEncParams, &nvImage, NVJPEG_INPUT_GRAY, data.width, data.height, mCuda);
    //std::cout << "debug 8" << "\n";
  }

  size_t length = 0;

  std::cout << "err: " << err << "\n";
  nvjpegEncodeRetrieveBitstream(mNvHandle, mNvState, NULL, &length, NULL);
  //std::cout << "debug 9" << "\n";
  cudaStreamSynchronize(mCuda);
  //std::cout << "debug 10" << "\n";
  std::vector<unsigned char> buffer(length);
  nvjpegEncodeRetrieveBitstream(mNvHandle, mNvState, buffer.data(), &length, 0);
  //std::cout << "debug 11" << "\n";
  //std::cout << buffer.size() << "\n";
  cudaStreamSynchronize(mCuda);
  //std::cout << "debug 11" << "\n";
  
  return buffer;

}


//PYBIND11_MODULE(nvjpeg_encoder_py, m) {
//  pybind11::class_<NvjpegEncoder, std::shared_ptr<NvjpegEncoder>>(m, "nvjpeg_encoder").def(pybind11::init<>());
//
//};


//for grayscale works but is fully red
//nvImage.channel[0] = dataPtr;
//  //for rgb planar:
//  nvImage.channel[1] = dataPtr + data.width * data.height;
//  nvImage.channel[2] = dataPtr + 2* data.width * data.height;
//  nvImage.pitch[0] = data.width;
//  nvImage.pitch[1] = data.width;
//  nvImage.pitch[2] = data.width;
//  //for rgbi:
//  //nvImage.pitch[0] = data.height * 3;
//  //nvImage.channel[1] = NULL;
//  //nvImage.channel[2] = NULL;
//  //nvImage.channel[3] = NULL;
//
//  nvjpegStatus_t err = nvjpegEncodeImage(mNvHandle, mNvState, mNvEncParams, &nvImage, NVJPEG_INPUT_RGB, data.width, data.height, mCuda);
// 
//
//doesnt work at all for grayscale(just thisn othing else):
//
  //nvImage.channel[0] = dataPtr;
