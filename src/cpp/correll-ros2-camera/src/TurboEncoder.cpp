//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>
#include <correll-ros2-camera/TurboEncoder.hpp>


TurboEncoder::TurboEncoder(DataType dtype) {
  mTurboJpegHandle = tjInitCompress();
  if (dtype == DataType::DEPTH) {
  
    mInternalDataType = DataType::DEPTH;
  } else if (dtype == DataType::RGB) {
    mInternalDataType = DataType::RGB;
  
  }


}

TurboEncoder::~TurboEncoder() {
  tjDestroy(mTurboJpegHandle);

}

std::vector<unsigned char> TurboEncoder::Encode(ImageData data) {
  unsigned char* compressedImage = NULL;
  long unsigned int jpegSize = 0;
  if (mInternalDataType == DataType::DEPTH) {
    tjCompress2(mTurboJpegHandle, reinterpret_cast<const unsigned char*>(data.dataPtr), data.width, 0, data.height, TJPF_GRAY, &compressedImage, &jpegSize, TJSAMP_GRAY, 75, TJFLAG_FASTDCT );

  } else if (mInternalDataType == DataType::RGB) {
      tjCompress2(mTurboJpegHandle, reinterpret_cast<const unsigned char*>(data.dataPtr), data.width, 0, data.height, TJPF_RGB, &compressedImage, &jpegSize, TJSAMP_444, 75, TJFLAG_FASTDCT );
  
  }
  std::vector<unsigned char> buffer(jpegSize);
  std::cout << jpegSize << "\n";
  //for (int i = 0; i < jpegSize; i++) {
  //  std::cout << *(compressedImage + i) << "\n";
  //
  //}
  std::memcpy((void*)buffer.data(), (void*)compressedImage, (size_t)jpegSize);
  tjFree(compressedImage);
  //std::ofstream output_file("test.jpg", std::ios::out | std::ios::binary);
  //output_file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
  //output_file.close();
  return buffer;

}



//PYBIND11_MODULE(turbo_encoder_py, m) {
//  pybind11::class_<TurboEncoder>(m, "turbo_encoder").def(pybind11::init<std::string>()).def("encode", &TurboEncoder::Encode);
//
//};
