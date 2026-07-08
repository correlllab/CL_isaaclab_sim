

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <correll-ros2-camera/NvjpegEncoder.hpp>
#include <iostream>
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

uint8_t NvjpegEncoder::Encode(ImageData data) {
  std::cout << "encoding!" << "\n";
  std::cout << data.dataPtr << "\n";
  std::cout << data.width << "\n";
  std::cout << data.height << "\n";
  unsigned char* dataPtr = reinterpret_cast<unsigned char*>(data.dataPtr);
  nvjpegImage_t nvImage;
  if (true) {
    //works for both grayscale 1 channel and rgb planar...why, idk, it shouldnt but it does and it dosnt really make sense hy
    //works ofr grayscale but loses brightness
    nvImage.channel[0] = dataPtr;
    //for rgb planar:
    nvImage.channel[1] = dataPtr + data.width * data.height;
    nvImage.channel[2] = dataPtr + 2* data.width * data.height;
    nvImage.pitch[0] = data.width;
    nvImage.pitch[1] = data.width;
    nvImage.pitch[2] = data.width;
  }

  //for rgbi:
  //nvImage.pitch[0] = data.height * 3;
  //nvImage.channel[1] = NULL;
  //nvImage.channel[2] = NULL;
  //nvImage.channel[3] = NULL;

  nvjpegStatus_t err = nvjpegEncodeImage(mNvHandle, mNvState, mNvEncParams, &nvImage, NVJPEG_INPUT_RGB, data.width, data.height, mCuda);
  std::cout << "err: " << (err) << "\n";
  size_t length = 0;
  nvjpegEncodeRetrieveBitstream(mNvHandle, mNvState, NULL, &length, NULL);
  cudaStreamSynchronize(mCuda);
  std::vector<unsigned char> buffer(length);
  std::cout << "length should be: " << length << "\n";
  nvjpegEncodeRetrieveBitstream(mNvHandle, mNvState, buffer.data(), &length, 0);
  cudaStreamSynchronize(mCuda);
  std::cout << "new jpeg size: " << buffer.size() << "\n";
  std::ofstream output_file("test.jpg", std::ios::out | std::ios::binary);
  output_file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
  output_file.close();
  
  return 0;

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
