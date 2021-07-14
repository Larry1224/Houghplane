#pragma once
#define CL_HPP_CL_1_2_DEFAULT_BUILD
#define CL_HPP_TARGET_OPENCL_VERSION 120
#define CL_HPP_MINIMUM_OPENCL_VERSION 120
#define CL_HPP_ENABLE_EXCEPTIONS
#include <CL/opencl.hpp>
#include <cstdint>
#include <vector>
using namespace std;

// you are free to store all your dirty little secrets in this struct ...XD
struct mydata
{
    float *pts;
    float *oldcen;
    float *arrtheta;
    float *arrphi;
    int *accumulator;
    int n;
    int ntheta;
    int nphi;
    int nrho;
    float rhomax;
    float dtheta;
    float dphi;
    float drho;
    vector<float> output;
    cl::Platform platform;
    cl::Device device;
    cl::Context ctx;
    cl::CommandQueue queue;
    cl::Program prg;
    cl::Buffer dev_pts, dev_accumlator;
};

// Task 2 - allocate memory & read point-cloud data
bool readPointCloud(const char *filename, mydata &data);

// Task 3 - Center the point-cloud data and get the maximum possible rho
float centerPointCloudToOrigin(mydata &data);

// Task 4 - prepare accumulator
void prepareAccumulator(mydata &data, const float rho_max, const size_t n_theta, const size_t n_phi, const size_t n_rho);

// Task 5 - do 3D standard Hough Transform
void houghTransform(mydata &data);

// Task 6 - Find resultant plane parameters
void identifyPlaneParameters(mydata &data, const float threshold);

// Task 7 - Output ptx file
bool outputPtxFile(const mydata &data, const char *outputCloudData);

// Task 8 - release all allocated memory
void release(mydata &data);
