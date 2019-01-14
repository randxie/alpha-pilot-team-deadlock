#include <boost/python.hpp>

#include <memory>
#include <iostream>
#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 2
#include <opencv2/contrib/contrib.hpp>
#endif

#include <libsgm.h>
#include "conversion.h"

#define ASSERT_MSG(expr, msg)          \
    if (!(expr))                       \
    {                                  \
        std::cerr << msg << std::endl; \
        std::exit(EXIT_FAILURE);       \
    }

namespace libsgm_ext
{
    namespace py = boost::python;
    PyObject* DepthEstimate(PyObject *leftMat, PyObject *rightMat)
    {
        // convert numpy array to cv::Mat
        cv::Mat left, right;
        left = pbcvt::fromNDArrayToMat(leftMat);
        right = pbcvt::fromNDArrayToMat(rightMat);

        // check matrix properties
        int disp_size = 64;
        ASSERT_MSG(left.size() == right.size() && left.type() == right.type(), "input images must be same size and type.");
        ASSERT_MSG(left.type() == CV_8U || left.type() == CV_16U, "input image format must be CV_8U or CV_16U.");
        ASSERT_MSG(disp_size == 64 || disp_size == 128, "disparity size must be 64 or 128.");

        int bits = 0;

        switch (left.type())
        {
        case CV_8UC1:
            bits = 8;
            break;
        case CV_16UC1:
            bits = 16;
            break;
        default:
            std::cerr << "invalid input image color format" << left.type() << std::endl;
            std::exit(EXIT_FAILURE);
        }

        sgm::StereoSGM ssgm(left.cols, left.rows, disp_size, bits, 8, sgm::EXECUTE_INOUT_HOST2HOST);
        cv::Mat output(cv::Size(left.cols, left.rows), CV_8UC1);
        ssgm.execute(left.data, right.data, output.data);

        output = output * 256 / disp_size;

        PyObject* ret = pbcvt::fromMatToNDArray(output);

        return ret;
    }

    #if (PY_VERSION_HEX >= 0x03000000)
        static void *init_ar()
        {
    #else
        static void init_ar()
        {
    #endif
            Py_Initialize();

            import_array();
            return NUMPY_IMPORT_ARRAY_RETVAL;
        }

    BOOST_PYTHON_MODULE(libsgm_ext)
    {
        //using namespace XM;
        init_ar();

        //initialize converters
        py::to_python_converter<cv::Mat, pbcvt::matToNDArrayBoostConverter>();
        pbcvt::matFromNDArrayBoostConverter();

        py::def("DepthEstimate", DepthEstimate);
    }
} // namespace libsgm_ext