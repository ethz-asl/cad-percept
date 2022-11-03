#include "NvInfer.h"

#include <iostream>
#include<memory>
#include<fstream>
#include<vector>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include<sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
// Options for the network
struct Options {
  // Use 16 bit floating point type for inference
  bool FP16 = true;
  // Batch sizes to optimize for.
  std::vector<int32_t> optBatchSizes;
  // Maximum allowable batch size
  int32_t maxBatchSize = 16;
  // Max allowable GPU memory to be used for model conversion, in bytes.
  // Applications should allow the engine builder as much workspace as they can afford;
  // at runtime, the SDK allocates no more than this and typically less.
  size_t maxWorkspaceSize = 4000000000;
  // GPU device index
  int deviceIndex = 0;
};
const Options m_options;


class Logger : public nvinfer1::ILogger {
 public:
  void log(nvinfer1::ILogger::Severity severity, const char* msg) throw() override {
    // suppress info-level messages
    if (severity == Severity::kINFO) return;

    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        std::cerr << "INTERNAL_ERROR: ";
        break;
      case Severity::kERROR:
        std::cerr << "ERROR: ";
        break;
      case Severity::kWARNING:
        std::cerr << "WARNING: ";
        break;
      case Severity::kINFO:
        std::cerr << "INFO: ";
        break;
      default:
        std::cerr << "UNKNOWN: ";
        break;
    }
    std::cerr << msg << std::endl;
  }
};
Logger m_Logger;
std::unique_ptr<nvinfer1::ICudaEngine> m_engine;
nvinfer1::IExecutionContext* m_Context;
nvinfer1::IRuntime* runtime;
std::unique_ptr<nvinfer1::IExecutionContext> m_context;
cudaStream_t m_CudaStream;

bool setup(){
  std::string engine_name("/home/mpantic/Work/SwissRoboticsDay/tensorrt/cnndirpred.plan");
  std::ifstream file(engine_name, std::ios::binary | std::ios::ate);
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::vector<char> buffer(size);
  if (!file.read(buffer.data(), size)) {
    throw std::runtime_error("Unable to read engine file");
  }
  std::cout << "runtime a" << std::endl;

  std::unique_ptr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(m_Logger)};
  std::cout << "runtime " << std::endl;
  if (!runtime) {
    return false;
  }

  // Set the device index

  auto ret = cudaSetDevice(m_options.deviceIndex);
  if (ret != 0) {
    int numGPUs;
    cudaGetDeviceCount(&numGPUs);
    auto errMsg = "Unable to set GPU device index to: " + std::to_string(m_options.deviceIndex) +
                  ". Note, your device has " + std::to_string(numGPUs) + " CUDA-capable GPU(s).";
    throw std::runtime_error(errMsg);
  }

  m_engine = std::unique_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(buffer.data(), buffer.size()));
  std::cout << "loaded" << std::endl;
  if (!m_engine) {
    return false;
  }

  m_context = std::unique_ptr<nvinfer1::IExecutionContext>(m_engine->createExecutionContext());
  if (!m_context) {
    return false;
  }

  auto cudaRet = cudaStreamCreate(&m_CudaStream);
  if (cudaRet != 0) {
    throw std::runtime_error("Unable to create cuda stream");
  }


  return true;
}


bool run(void* inputdata){

  auto dims = m_engine->getBindingDimensions(0);
  std::cout << (int)m_engine->getBindingDataType(0) << std::endl;
  auto outputL = m_engine->getBindingDimensions(1).d[1];
  nvinfer1::Dims4 inputDims = {1, dims.d[1], dims.d[2], dims.d[3]};
  std::cout << dims.d[1]<< std::endl;
  std::cout << dims.d[2]<< std::endl;
  std::cout << dims.d[3]<< std::endl;
  m_context->setBindingDimensions(0, inputDims);


  void* outputdata;
  cudaMalloc(&outputdata, 1*3*sizeof(float ));

  std::vector<void*> predicitonBindings = {inputdata,outputdata};

  bool status = m_context->enqueueV2(predicitonBindings.data(), m_CudaStream, nullptr);
  if (!status) {
    return false;
  }else{
    std::cout << "it run!" << std::endl;
  }
  void* outputdata_cpu;
  outputdata_cpu = malloc(3*sizeof(float));

  cudaMemcpy(outputdata_cpu, outputdata, 3*sizeof(float), cudaMemcpyDeviceToHost);


  Eigen::Vector3d normal;
  normal.x() =((float*)outputdata_cpu)[0];
  normal.y() =((float*)outputdata_cpu)[1];
  normal.z() =((float*)outputdata_cpu)[2];

  std::cout << normal.norm() << std::endl;
  std::cout << normal.normalized().transpose() << std::endl;

  if (!m_context->allInputDimensionsSpecified()) {
    throw std::runtime_error("Error, not all input dimensions specified.");
  }
  return true;

  cudaFree(&outputdata);
  cudaFree(&inputdata);
}
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  ROS_INFO("inside callback");

  cv::Mat data(172, 224, CV_32FC3);
  //std::cout << cloud_msg->data.size() << std::endl;
  //std::cout << cloud_msg->point_step << std::endl;
//std::cout << cloud_msg->row_step << std::endl;

for (auto channel : cloud_msg->fields){

  std::cout << channel.name << std::endl;
}
  for(int x=0; x<172; x++){
    for(int y=0; y<224; y++){


      int ix_cloud = cloud_msg->row_step*x + y*cloud_msg->point_step;
      data.at<cv::Vec3f>(x,y)[0] = *(float*)(cloud_msg->data.data() + ix_cloud);
      data.at<cv::Vec3f>(x,y)[1] =  *(float*)(cloud_msg->data.data() + ix_cloud+4);
      data.at<cv::Vec3f>(x,y)[2] =  *(float*)(cloud_msg->data.data() + ix_cloud+8);

      if(isnan(data.at<cv::Vec3f>(x,y)[0]) ||
          isnan(data.at<cv::Vec3f>(x,y)[1]) ||
          isnan(data.at<cv::Vec3f>(x,y)[2])){
        data.at<cv::Vec3f>(x,y)[0] = 0.0;
        data.at<cv::Vec3f>(x,y)[1] = 0.0;
        data.at<cv::Vec3f>(x,y)[2] =  0.0;
      }

      if(x>90 && x < 95 && y > 100 && y < 105) {
        //std::cout << (float)data.at<cv::Vec3f>(x, y)[0] << " " << (float)data.at<cv::Vec3f>(x, y)[1]
        //          << " " << (float)data.at<cv::Vec3f>(x, y)[2] << std::endl;
      }
    }

  }

  void* inputdata;
  cudaMalloc(&inputdata, 1*3*172*224*sizeof(float ));
  cudaMemcpy(inputdata, data.data, 1*3*172*224*sizeof(float ), cudaMemcpyHostToDevice);
run(inputdata);


}

int main(int argc, char **argv){
  ros::init(argc, argv, "pc");
  ros::NodeHandle n;
 ros::Subscriber sub = n.subscribe("/pico_flexx/points", 1000, cloud_callback);
  setup();
 ros::spin();


 //run();
}