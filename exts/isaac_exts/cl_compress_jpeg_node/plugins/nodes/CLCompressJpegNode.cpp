
//#include <OgnExampleNodeDatabase.h>
//#include <OgnTutorialEmptyDatabase.h>

#include <CLCompressJpegNodeDatabase.h> 
#include <iostream>

using omni::graph::core::Type;
using omni::graph::core::BaseDataType;

//#define CLCompressJpegOmniNodeStatusOK ((uint8_t) 0u)
//#define CLCompressJpegOmniNodeStatusERROR ((uint8_t) 1u)

namespace cl_compress_jpeg_node {

  class CLCompressJpegNode {
    public:
      static bool compute(CLCompressJpegNodeDatabase& db) {
        std::string path = "/home/mateo/correll/kit-extension-template-cpp/PACKAGE-INFO.yaml";
        std::printf("running compute");
        readData(path);
        return true;
      }
      static bool readData(std::string filePath) {
      
        std::ifstream f(filePath);
        if (!f.is_open()) {
        
          std::printf("Error opening file!");
          return false;
        }
        std::string s;
        while (getline(f, s)) {
          std::cout << s;
        }
        f.close();
        return true;

      }
  };
REGISTER_OGN_NODE()
}

