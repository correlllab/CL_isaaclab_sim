#include <iostream>

//#include <jpeglib.h>
//#include <turbojpeg.h>

#include <CLCompressJpegNodeDatabase.h> 
#include <iostream>
//#include <jpeglib.h>

using omni::graph::core::Type;
using omni::graph::core::BaseDataType;

#define WIDTH 1058
#define HEIGHT 794

namespace cl_compress_jpeg_node {

  class CLCompressJpegNode {
    public:
      //const auto& m_rNodeInputBuffer;
      //const uint8_t* m_pData;
      //const unsigned char** m_pOutputJpegBuffer;

      static bool compute(CLCompressJpegNodeDatabase& db) {



        //m_rNodeInputBuffer = db.inputs.input_buffer();
        //m_pData = db.inputs.input_buffer().data();
        
        return true;
      }

 //     static int compressInputRGB8AsJpeg(const uint8_t* pData, const int imageHeight, const int imageWidth) {
 //     
 //       struct jpeg_error_mgr jerr;
 //       FILE * outfile;
 //       JSAMPROW row_pointer[1];  
 //       int row_stride;  
 //       cinfo.err = jpeg_std_error(&jerr);
 //       jpeg_create_compress(&cinfo);

 //       if ((outfile = fopen(strImageName, "wb")) == NULL) {
 //           fprintf(stderr, "can't open %s\n", strImageName);
 //           //exit(1);
 //           return -1;
 //       }

 //       jpeg_stdio_dest(&cinfo, outfile);
 //       cinfo.image_width = image_width;    
 //       cinfo.image_height = image_height;
 //       cinfo.input_components = 3;     
 //       cinfo.in_color_space = JCS_RGB;     
 //       jpeg_set_defaults(&cinfo);
 //       jpeg_set_quality(&cinfo, quality, TRUE);
 //       jpeg_start_compress(&cinfo, TRUE);
 //       row_stride = image_width * 3;  
 //       int line = 0;

 //       while (line < cinfo.image_height) {
 //           row_pointer[0] = &image_buffer[line * row_stride];
 //           jpeg_write_scanlines(&cinfo, row_pointer, 1);
 //           line++;
 //       }
 //       jpeg_finish_compress(&cinfo);
 //       fclose(outfile);
 //       jpeg_destroy_compress(&cinfo);

 //       return 0;
 //     }
  };
REGISTER_OGN_NODE()
}

