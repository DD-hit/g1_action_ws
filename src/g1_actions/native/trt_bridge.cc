#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
thread_local std::string last_error;
struct Logger : nvinfer1::ILogger {
  void log(Severity s, const char* m) noexcept override {
    if (s <= Severity::kWARNING) std::cerr << "[TensorRT] " << m << '\n';
  }
} logger;
void check(cudaError_t error) {
  if (error != cudaSuccess) throw std::runtime_error(cudaGetErrorString(error));
}
struct Engine {
  std::unique_ptr<nvinfer1::IRuntime> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine> engine;
  std::unique_ptr<nvinfer1::IExecutionContext> context;
  cudaStream_t stream = nullptr;
  std::vector<void*> buffers;
  int input = -1, output = -1;
  size_t input_count = 0, output_count = 0;
  ~Engine() {
    if (stream) cudaStreamSynchronize(stream);
    for (void* p : buffers) if (p) cudaFree(p);
    if (stream) cudaStreamDestroy(stream);
  }
};
size_t volume(nvinfer1::Dims dims) {
  size_t n = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) throw std::runtime_error("Static positive input/output dimensions required");
    n *= dims.d[i];
  }
  return n;
}
}
extern "C" {
const char* ele_trt_error() { return last_error.c_str(); }
void* ele_trt_create(const char* path) {
  try {
    auto p = std::make_unique<Engine>();
    std::ifstream file(path, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open TensorRT engine");
    std::vector<char> bytes((std::istreambuf_iterator<char>(file)), {});
    initLibNvInferPlugins(&logger, "");
    p->runtime.reset(nvinfer1::createInferRuntime(logger));
    if (!p->runtime) throw std::runtime_error("TensorRT runtime unavailable");
    p->engine.reset(p->runtime->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!p->engine) throw std::runtime_error("Cannot deserialize engine; rebuild for this Jetson/TensorRT version");
    p->context.reset(p->engine->createExecutionContext());
    if (!p->context) throw std::runtime_error("Cannot create execution context");
    if (p->engine->getNbBindings() != 2) throw std::runtime_error("One-input/one-output engine required");
    p->buffers.resize(2, nullptr);
    for (int i = 0; i < 2; ++i) {
      if (p->engine->getBindingDataType(i) != nvinfer1::DataType::kFLOAT)
        throw std::runtime_error("Float32 I/O bindings required");
      size_t count = volume(p->engine->getBindingDimensions(i));
      check(cudaMalloc(&p->buffers[i], count * sizeof(float)));
      if (p->engine->bindingIsInput(i)) { p->input=i; p->input_count=count; }
      else { p->output=i; p->output_count=count; }
    }
    if (p->input < 0 || p->output < 0) throw std::runtime_error("Invalid engine bindings");
    check(cudaStreamCreate(&p->stream));
    return p.release();
  } catch (const std::exception& e) { last_error=e.what(); return nullptr; }
}
int ele_trt_shape(void* handle, int output, int* dims, int capacity) {
  auto p=static_cast<Engine*>(handle);
  auto shape=p->engine->getBindingDimensions(output ? p->output : p->input);
  if (capacity < shape.nbDims) return -1;
  for (int i=0; i<shape.nbDims; ++i) dims[i]=shape.d[i];
  return shape.nbDims;
}
int ele_trt_infer(void* handle, const float* input, size_t in_count,
                  float* output, size_t out_count) {
  try {
    auto p=static_cast<Engine*>(handle);
    if (in_count!=p->input_count || out_count!=p->output_count)
      throw std::runtime_error("TensorRT I/O size mismatch");
    check(cudaMemcpyAsync(p->buffers[p->input],input,in_count*sizeof(float),cudaMemcpyHostToDevice,p->stream));
    if (!p->context->enqueueV2(p->buffers.data(),p->stream,nullptr))
      throw std::runtime_error("TensorRT enqueue failed");
    check(cudaMemcpyAsync(output,p->buffers[p->output],out_count*sizeof(float),cudaMemcpyDeviceToHost,p->stream));
    check(cudaStreamSynchronize(p->stream));
    return 0;
  } catch (const std::exception& e) { last_error=e.what(); return -1; }
}
void ele_trt_destroy(void* handle) { delete static_cast<Engine*>(handle); }
}
