
#include <pybind11/pybind11.h>
#include <nvjpeg_encoder.hpp>


nvjpegEncoder::nvjpegEncoder() {

  nvjpegCreateSimple(&m_nvHandle);
  std::vector<std::string> topicNames = {"/realsense/left_hand/color/image_raw/compressed", "/realsense/left_hand/aligned_depth_to_color/image_raw/compressed", "/realsense/right_hand/color/image_raw/compressed", "/realsense/right_hand/aligned_depth_to_color/image_raw/compressed"};

  for (auto& topic : topicNames) {
    nvjpegEncoderState_t state;
    nvjpegEncoderStateCreate(m_nvHandle, &state, m_cudaStream);
    m_stateMap[topic] = state;

  
  
  }
  nvjpegEncoderParamsCreate(m_nvHandle, &m_nvEncParams, m_cudaStream);
  
  nvjpegEncoderStateCreate(m_nvHandle, &m_nvEncState, m_cudaStream); 
  nvjpegEncoderParamsSetSamplingFactors(m_nvEncParams, NVJPEG_CSS_444, m_cudaStream);

}

nvjpegEncoder::~nvjpegEncoder() {

  nvjpegEncoderParamsDestroy(m_nvEncParams);
  nvjpegDestroy(m_nvHandle);

}


void nvjpegEncoder::encodeImageBuffer(long dataPtr, int width, int height, std::string topicName, std::vector<unsigned char>& jpegBuffer) {

  nvjpegEncoderState_t state = m_stateMap[topicName];

  if (dataPtr == 0) {
    return;
  }

  if (width == 0) {
    return;
  
  }

  if (height == 0) {
    return;
  
  }

  unsigned char* charDataPtr = reinterpret_cast<unsigned char*>(dataPtr);

  nvjpegImage_t nvImage;
  nvImage.channel[0] = charDataPtr;
  nvImage.pitch[0] = width * 3;
  nvImage.channel[1] = NULL;
  nvImage.channel[2] = NULL;
  nvImage.channel[3] = NULL;
  std::cout << "compression width: " << width << "\n";
  std::cout << "compression height: " << height << "\n";
  if (nvImage.channel[0] == nullptr) {
    std::cout << "its null" << "\n";
  }
  else {
    std::cout << "not null" << "\n";
  
  }

  std::cout << "Before nvjpegEncodeImage:" << std::endl;
  std::cout << "  width=" << width << std::endl;
  std::cout << "  height=" << height << std::endl;
  std::cout << "  m_nvHandle=" << (void*)m_nvHandle << std::endl;
  std::cout << "  nvEncState=" << (void*)state << std::endl;
  std::cout << "  m_nvEncParams=" << (void*)m_nvEncParams << std::endl;
  std::cout << "  m_cudaStream=" << (void*)m_cudaStream << std::endl;
  std::cout << "  nvImage.channel[0]=" << (void*)nvImage.channel[0] << std::endl;
  std::cout << "  nvImage.pitch[0]=" << nvImage.pitch[0] << std::endl;
  std::cout << "  nvImage.channel[1]=" << (void*)nvImage.channel[1] << std::endl;
  std::cout << "  nvImage.pitch[1]=" << nvImage.pitch[1] << std::endl;
  std::cout << "  nvImage.channel[2]=" << (void*)nvImage.channel[2] << std::endl;
  std::cout << "  nvImage.pitch[2]=" << nvImage.pitch[2] << std::endl;
  std::cout << "  nvImage.channel[3]=" << (void*)nvImage.channel[3] << std::endl;
  std::cout << "  nvImage.pitch[3]=" << nvImage.pitch[3] << std::endl;

  m_latestCudaError = nvjpegEncodeImage(m_nvHandle, state, m_nvEncParams, &nvImage, NVJPEG_INPUT_RGBI, width, height, m_cudaStream);
  std::cout << "m_latestCudaError: " << typeid(m_latestCudaError).name() << "\n";
  std::cout << "m_latestCudaError: " << (m_latestCudaError) << "\n";
  ////std::cout << "charDataPtr: " << charDataPtr << "\n";
  ////nvImage->channel[0] = charDataPtr;
  //nvImage->pitch[0] = width * 3;

  //nvImage.channel[1] = nullptr;
  //nvImage.channel[2] = nullptr;
  //nvImage.channel[3] = nullptr;

  //std::cout << "image initailized" << "\n";

  //try {

  //  m_latestCudaError = nvjpegEncodeImage(m_nvHandle, nvEncState, m_nvEncParams, &nvImage2, NVJPEG_INPUT_RGBI, width, height, m_cudaStream);

  //} catch (const char* msg) {
  //  std::cout << "exception: " << msg << "\n";
  //
  //}
  //std::cout << m_latestCudaError << "\n";
  //std::cout << "nvjpeg encoded!" << "\n";
  size_t length = 0;

  nvjpegEncodeRetrieveBitstream(m_nvHandle, state, nullptr, &length, nullptr);
  cudaStreamSynchronize(m_cudaStream);

  jpegBuffer.clear();
  jpegBuffer.resize(length);
  //std::vector<unsigned char> jpeg(length);
  ////jpegBuffer.resize(length);

  nvjpegEncodeRetrieveBitstream(m_nvHandle, state, jpegBuffer.data(), &length, 0);

  cudaStreamSynchronize(m_cudaStream);

  std::cout << "jpeg new size: " << jpegBuffer.size() << "\n";
  //std::cout << "jpeg vector native size : " << jpeg.size() << "\n";
  //std::cout << "length should be : " << length << "\n";

  //jpegBuffer = &jpeg;

}

PYBIND11_MODULE(nvjpeg_encoder_py, m) {


  pybind11::class_<nvjpegEncoder, std::shared_ptr<nvjpegEncoder>>(m, "nvjpeg_encoder").def(pybind11::init<>());


};


