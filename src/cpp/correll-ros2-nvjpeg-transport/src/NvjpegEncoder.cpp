#include <pybind11/pybind11.h>
#include <correll-ros2-nvjpeg-transport/NvjpegEncoder.hpp>

nvjpegEncoder::nvjpegEncoder(std::vector<std::string> topicNames) {

  std::cout << "starting nvjpegEncoder initialziation for: " << "\n";
  std::cout << "nvjpegCreateSimple for: " << "\n";
  nvjpegCreateSimple(&m_nvHandle);
  std::cout << "m_topicNames before: " << "\n";
  m_topicNames = topicNames;
  std::cout << "m_topicNames after: " << "\n";

  bool depth = false;
  for (auto& topic : m_topicNames) {
    nvjpegEncoderState_t state;
    nvjpegEncoderStateCreate(m_nvHandle, &state, m_cudaStream);
    m_stateMap[topic] = state;

    if (topic.find("aligned_depth") != std::string::npos) {
      depth = true;
      std::cout << "depth!!!!" << "\n";

    }

  //  nvjpegEncoderParamsSetSamplingFactors(m_nvEncParams, NVJPEG_CSS_444, m_cudaStream);
  }
  std::cout << "nvjpegEncoderParamsCreate for: " << "\n";
  nvjpegEncoderParamsCreate(m_nvHandle, &m_nvEncParams, m_cudaStream);

  std::cout << "nvjpegEncoderStateCreate for: " << "\n";
  nvjpegEncoderStateCreate(m_nvHandle, &m_nvEncState, m_cudaStream);
  if (depth) {
    std::cout << " depth determiend nvjpegEncoderParamsSetSamplingFactors: " << "\n";
    nvjpegEncoderParamsSetSamplingFactors(m_nvEncParams, NVJPEG_CSS_GRAY, m_cudaStream);
  } else {
    std::cout << " color determiend nvjpegEncoderParamsSetSamplingFactors: " << "\n";
    nvjpegEncoderParamsSetSamplingFactors(m_nvEncParams, NVJPEG_CSS_444, m_cudaStream);
  }
  //nvjpegEncoderParamsSetSamplingFactors(m_nvEncParams, NVJPEG_CSS_444, m_cudaStream);
  //if (depth) {

  //  nvjpegEncoderParamsSetSamplingFactors(m_nvEncParams, NVJPEG_CSS_GRAY, m_cudaStream);
  //} else {

  //  nvjpegEncoderParamsSetSamplingFactors(m_nvEncParams, NVJPEG_CSS_444, m_cudaStream);
  //}

}

nvjpegEncoder::~nvjpegEncoder() {

  std::cout << "nvjpegEncoder destructor called for some rason" << "\n";
  nvjpegEncoderParamsDestroy(m_nvEncParams);
  nvjpegDestroy(m_nvHandle);

}


void nvjpegEncoder::encodeImageBuffer(long dataPtr, int width, int height, std::string topicName, std::vector<unsigned char>& jpegBuffer) {

  std::cout << "acquirng state for topicname for: " << topicName << "\n";
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

  std::cout << "casting charDataPtr for: " << topicName << "\n";
  unsigned char* charDataPtr = reinterpret_cast<unsigned char*>(dataPtr);

  std::cout << "intialzing nvImage instance: " << topicName << "\n";
  nvjpegImage_t nvImage;
  std::cout << "setting channel 0 to charDataPtr: " << topicName << "\n";
  nvImage.channel[0] = charDataPtr;

  if (topicName.find("depth") != std::string::npos) {
    std::cout << "nvImage.pitch = 1 for: " << topicName << "\n";
    nvImage.pitch[0] = width * 1;
  } else {
    std::cout << "nvImage.pitch = 3 for: " << topicName << "\n";
    nvImage.pitch[0] = width * 3;
  }
  std::cout << "nvImage channel null intiailzation for: " << topicName << "\n";
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

  //std::cout << "Before nvjpegEncodeImage:" << std::endl;
  //std::cout << "  width=" << width << std::endl;
  //std::cout << "  height=" << height << std::endl;
  //std::cout << "  m_nvHandle=" << (void*)m_nvHandle << std::endl;
  //std::cout << "  nvEncState=" << (void*)state << std::endl;
  //std::cout << "  m_nvEncParams=" << (void*)m_nvEncParams << std::endl;
  //std::cout << "  m_cudaStream=" << (void*)m_cudaStream << std::endl;
  //std::cout << "  nvImage.channel[0]=" << (void*)nvImage.channel[0] << std::endl;
  //std::cout << "  nvImage.pitch[0]=" << nvImage.pitch[0] << std::endl;
  //std::cout << "  nvImage.channel[1]=" << (void*)nvImage.channel[1] << std::endl;
  //std::cout << "  nvImage.pitch[1]=" << nvImage.pitch[1] << std::endl;
  //std::cout << "  nvImage.channel[2]=" << (void*)nvImage.channel[2] << std::endl;
  //std::cout << "  nvImage.pitch[2]=" << nvImage.pitch[2] << std::endl;
  //std::cout << "  nvImage.channel[3]=" << (void*)nvImage.channel[3] << std::endl;
  //std::cout << "  nvImage.pitch[3]=" << nvImage.pitch[3] << std::endl;

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
  //std::ofstream output_file("test_cpp.jpg", std::ios::out | std::ios::binary);

}

PYBIND11_MODULE(nvjpeg_encoder_py, m) {


  pybind11::class_<nvjpegEncoder, std::shared_ptr<nvjpegEncoder>>(m, "nvjpeg_encoder").def(pybind11::init<std::vector<std::string>>(), pybind11::arg("topicNames")).def("encode_image_buffer", &nvjpegEncoder::encodeImageBuffer);


};
