#include <iostream>
#include <fstream>
#include <sstream>
// #include <fixmath.h>
#include <stdint.h>

#include <stdlib.h>
#include <cstdlib>

#include <signal.h>
// #include <boost/date_time/posix_time/posix_time.hpp>
#include <random>
#include <array>
// #include <aocl_utils.h>
#include <algorithm>

#ifdef __APPLE__
#include <OpenCL/opencl.h>
#else
#include <CL/cl.h>
#endif

#include <boost/program_options.hpp>
#include "constants.h"
#include <boost/date_time/posix_time/posix_time.hpp>
// #include <aocl_utils.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include "constants.h"    
#include <malloc.h>
#include <boost/date_time/posix_time/posix_time.hpp>
// #include <aocl_utils.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include "constants.h"    
#include <malloc.h>
#include <math.h>
#include "idm.h"
// #include "Kernel/fixmath.h"
#include "epic_visualizer.h"
#include <xcl2.hpp>

using namespace std;
namespace pt = boost::posix_time;


// typedef struct{
//     int id;
//     int front[2];
//     int rear[2];

//     int lane[2];
//     int velocity[2];
//     int position[2];
// }CLVehicle;

///////// PROBLEM VARIABLES ///////////

    int laneCount = 4;
    int agentsize = 10;
    CLVehicle * lanes;
    short laneAgentCount[LANE_MAX * 2];
    short removerIndex[LANE_MAX]{0};
    CLVehicleFloat * vehiclesFloat;
    // double dt = 250.0/1000.0;

/////////// TIME VARIABLES /////////

double total_execution = 0;
double mem_write_start, mem_write_end, mem_write, fpga_start_time, fpga_end_time, execution_time, mem_read_start, mem_read_end, mem_read, ex_kernel_start, ex_kernel_end, ex_kernel, iteration_start, iteration_end;
double cpu_start_time, cpu_end_time, cpu_total;
cl_ulong time_exec; /// measures kernel execution time through profiler

/////////// KERNEL VARIABLES //////////

    std::vector<cl::Memory> inoutBufVec;
    cl_int err = 0;
    int globalWorkSize = 1;
    int localWorkSize = 1;
    bool useCpu = false;
    std::string aocx;
    bool scan = false;
    int iteration = 10;       
    bool emulate = false;        // select Kernel if to be worked on computer
    bool singleWorkItem = true; // select Kernel(swi) and adapt iteration count  /// TRUE BY DEFAULT -- change if not-so-pure version is to be used -- 
    bool &swi = singleWorkItem;

    cl::Context m_context;
    cl::CommandQueue m_command_queue;
    cl::Buffer agents_buffer; // lanes;
    cl::Program m_program;
    cl::Kernel m_kernel;

//////////// OUTPUT STREAMS /////////

std::ofstream clock_cycles("Results/clock_cycles.txt");
std::ofstream cpu_time("Results/cpu_time.txt");
std::ofstream fpga_time("Results/fpga_time.txt");
// std::ofstream positions("Results/positions.txt");


/////////// HELPER FUNCTIONS ///////////

template <class T>
void printArray(T * & array, int size){
        for(int i = 0; i < size; i++)
    {
        std::cout << array[i] << " ";
    }
    std::cout << std::endl;
}

template <class U>
void writeArray(std::ofstream & out, U * array, int size){ 
    for(int i = 0; i < size; i++)
    {
        out << std::setprecision(4) << std::right << std::setw(10) << array[i];
    }  
    out << std::endl; 
}

float rand_float() {
  return float(rand()) / float(RAND_MAX) * 20.0f - 10.0f;
}
// void writeVehicles(std::ofstream & positions, std::ofstream & velocities, bool fixedpoint=false){
//     for(size_t i = 0; i < agentsize; i++)
//     {
//         if(useCpu != true){
//             positions << std::setprecision(8) << std::right << std::setw(15) << fixedpt_tofloat( vehicles[i].position[1] );
//             velocities << std::setprecision(8) << std::right << std::setw(15) << fixedpt_tofloat( vehicles[i].velocity[1] );
//         }else{
//             positions << std::setprecision(8) << std::right << std::setw(15) << vehiclesFloat[i].position[1] ;
//             velocities << std::setprecision(8) << std::right << std::setw(15) << vehiclesFloat[i].velocity[1] ;
//         }
//     }
//     positions << std::endl;
//     velocities << std::endl;
// }

// High-resolution timer.
double getCurrentTimestamp() {
#ifdef _WIN32 // Windows
  // Use the high-resolution performance counter.

  static LARGE_INTEGER ticks_per_second = {};
  if(ticks_per_second.QuadPart == 0) {
    // First call - get the frequency.
    QueryPerformanceFrequency(&ticks_per_second);
  }

  LARGE_INTEGER counter;
  QueryPerformanceCounter(&counter);

  double seconds = double(counter.QuadPart) / double(ticks_per_second.QuadPart);
  return seconds;
#else         // Linux
  timespec a;
  clock_gettime(CLOCK_MONOTONIC, &a);
  return (double(a.tv_nsec) * 1.0e-9) + double(a.tv_sec);
#endif
}

/////////// FUNCTION DECLARATIONS ///////////////

void initialiseProblem();
void initialiseKernel();
int executeKernel();
bool run_model();
void releaseKernel();
void startSim();

//////////////////////////////////////////

int main(int argc, char *argv[])
{
    // waitMilliseconds(5000);
    // std::getchar();
    printf("Simulation starts");

    for(short i=1; i < argc; i++){
        if( strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "-cpu") == 0 ){
            useCpu = true;
        }
        else if(strcmp(argv[i], "-w") == 0){
            localWorkSize = stoi(argv[++i]);
        }
        else if(strcmp(argv[i], "-l") == 0){
            laneCount = stoi(argv[++i]);
        }
        else if(strcmp(argv[i], "-scan") == 0){
            scan = true;
        }
        else if(strcmp(argv[i], "-it") == 0){
            iteration = stoi(argv[++i]);
        }
        else if(strcmp(argv[i], "-emulate") == 0){
            emulate=true;
        }
        else if(strcmp(argv[i], "-swi") == 0 ){
            singleWorkItem = true;
        }
        else if(strcmp(argv[i], "-aocx") == 0 ){
            aocx = std::string(argv[++i]);
        }
        else if(strcmp(argv[i], "-agents") == 0 ){
            agentsize = stoi(argv[++i]);
            globalWorkSize = agentsize;
            if(agentsize > MAX_AGENT_SIZE){std::cout << std::endl << "too many agents" << std::endl; return 0;}
        }
    }
    if(scan){
        for(agentsize = 5; agentsize <= 200; agentsize += 10){
            startSim();
        }
        return 0;
    }
    startSim();

return 0;
}

void startSim(){

    std::ofstream myfile(useCpu ? "Results/agent_positions_cpu.txt" : "Results/agent_positions_fpga.txt");
    std::ofstream space_front(useCpu ? "Results/space_front_cpu.txt" : "Results/space_front_fpga.txt");

    initialiseProblem();
    initialiseKernel();
    executeKernel();
    // writeVehicles(myfile, space_front, true);
    // epic_visualizer(vehicles, 128, agentsize, lanes);
    releaseKernel();

    // if(useCpu) cpu_time << numOfAgent << " " << (int)total_execution << std::endl;
    
    // if(!useCpu && !swi) {clock_cycles << numOfAgent << " " << (int)sum_kernel/(int)sim_count * 150 << std::endl;
    // fpga_time << numOfAgent << " " << (int)sum_kernel/(int)kernel_count << std::endl; 
    // }else if(!useCpu && swi){
    //     fpga_time << numOfAgent << " " << total_execution << std::endl; // total execution in microseconds.
    //     clock_cycles << numOfAgent << " " << (int)kernel_time * 150 / (iteration * agents) << std::endl;
    // }

    // std::cout<<"*************************************************************"<<std::endl;

    // std::cout << "Summary:  Sense: " <<  << "us Think:" <<  <<
    //           "us Act:" <<  <<"us"<<std::endl;
    // std::cout << "Percentage:  Sense: " <<  << " Think:" <<  <<
    //           " Act:" <<  <<""<<std::endl;
    // std::cout<<"*************************************************************"<<std::endl;

    return;
}

void initialiseProblem(){
    lanes = (CLVehicle *)calloc(LANE_MAX * MAX_VEHICLES_PER_LANE, sizeof(CLVehicle));
    vehiclesFloat = (CLVehicleFloat *)calloc(MAX_AGENT_SIZE, sizeof(CLVehicle));

    for(size_t i = 1; i <= agentsize / 2; i++){
        LANES(0, i-1).id = i;
        LANES(0, i-1).position[0] = LANES(0, i-1).position[1] = fixedpt_fromint ( 6 * (agentsize - i) );
    }
    for(size_t i = (agentsize / 2) + 1; i <= agentsize; i++){
        LANES(1, i - agentsize/2 - 1 ).id = i;
        LANES(1, i - agentsize/2 - 1 ).position[0] = LANES(1, i - agentsize/2 - 1).position[1] = fixedpt_fromint ( 6 * (agentsize - i) );
    }
}

void initialiseKernel(){
        //running on FPGA
    if(useCpu == false){
        std::vector<cl::Device> devices = xcl::get_xil_devices();
        
        cl::Device device = devices[0];
        std::cout << "Initialising Kernel on the FPGA" << std::endl;

        OCL_CHECK(err, m_context = cl::Context(device, NULL, NULL, NULL, &err));
        OCL_CHECK(err, m_command_queue = cl::CommandQueue(m_context, device, CL_QUEUE_PROFILING_ENABLE, &err));
        OCL_CHECK(err, std::string device_name = device.getInfo<CL_DEVICE_NAME>(&err));
        std::cout << "Found Device=" << device_name.c_str() << std::endl;
       



        // double create_start = getCurrentTimestamp();
        // double create_stop = getCurrentTimestamp();
        // double create_duration = create_stop - create_start;
        // total_execution += create_duration;
        // std::cout << " Create Duration : " << create_duration << std::endl; 

        /* Create Kernel Program from the binary */

        std::string binaryFile = xcl::find_binary_file(device_name, "sim");
        // m_program = createProgramFromBinary(m_context, binary_file.c_str(), &device_id, num_devices);
        cl::Program::Binaries bins = xcl::import_binary_file(binaryFile);
        devices.resize(1);
        OCL_CHECK(err, m_program = cl::Program(m_context, devices, bins, NULL, &err));
        OCL_CHECK(err, m_kernel = cl::Kernel(m_program, "sim", &err));

        /* Create Memory Buffer */
        OCL_CHECK(err, agents_buffer = cl::Buffer(m_context, CL_MEM_USE_HOST_PTR | CL_MEM_READ_WRITE,
                                                    sizeof(CLVehicle) * MAX_AGENT_SIZE, lanes, &err));
        inoutBufVec.push_back(agents_buffer);

        OCL_CHECK(err, err = m_command_queue.enqueueMigrateMemObjects(inoutBufVec, 0));

        int narg = 0;
        OCL_CHECK(err, err = m_kernel.setArg(narg++, agents_buffer));
        OCL_CHECK(err, err = m_kernel.setArg(narg++, agentsize));
        OCL_CHECK(err, err = m_kernel.setArg(narg++, laneCount));
        OCL_CHECK(err, err = m_kernel.setArg(narg++, iteration));

    }else if(useCpu == true){
    }
//    free(platforms);
//    free(devices);
}

int executeKernel(){
    ex_kernel_start = getCurrentTimestamp();

    if( useCpu == false ){

        if(singleWorkItem) { globalWorkSize = 1; localWorkSize = 1; }

        fpga_start_time = getCurrentTimestamp(); 
        for(int i = 0; i < iteration; i++){
            iteration_start = getCurrentTimestamp();
            OCL_CHECK(err, err = m_command_queue.enqueueTask(m_kernel));
            iteration_end = getCurrentTimestamp();
            if(singleWorkItem) break;
        }
        m_command_queue.finish();
        OCL_CHECK(err, err = m_command_queue.enqueueMigrateMemObjects(inoutBufVec, CL_MIGRATE_MEM_OBJECT_HOST));

        fpga_end_time = getCurrentTimestamp();
        execution_time = fpga_end_time - fpga_start_time;
        total_execution += execution_time;

        // time_exec = getStartEndTime(kernel_execution);

        std::cout << "Execution Time: " << execution_time*1e6 << " Memory Read Time: " << mem_read*1e6 << std::endl;

        ex_kernel_end = getCurrentTimestamp();
        ex_kernel = ex_kernel_end - ex_kernel_start;
        std::cout << ex_kernel*1e6 << std::endl;

        return (int)( execution_time*1e6 );

    }
    else if (useCpu == true)
    {

        cpu_start_time = getCurrentTimestamp();

        run_model();

        cpu_end_time = getCurrentTimestamp();
        cpu_total = cpu_end_time - cpu_start_time;
        std::cout << "CPU Execution Time: " << cpu_total*1e6 << std::endl;
        total_execution += cpu_total;
        return cpu_total*1e6;
    }
    return 1;
}

bool run_model(){       

    // find_laneAgentCount(laneAgentCount, lanes, laneCount);
    // CLVehicle veh;
   
    return true;
}

void releaseKernel(){
    // cl_int ret;
    // execution_time = 0;
    // total_execution = 0;
    // if(useCpu == false){
    //     ret =  clReleaseProgram(m_program);
    //     for (int i = 0; i < 3; i++)
    //     {
    //         clReleaseMemObject(m_memObjects[i]);
    //     }
    //     ret |=  clReleaseCommandQueue(m_command_queue);
    //     ret |=  clReleaseContext(m_context);
    //     ret |=  clReleaseKernel(m_kernel);
    //     if (ret != CL_SUCCESS) {
    //         std::cerr<<"release error"<<std::endl;
    //         return;
    //     }
    // }
    free(vehiclesFloat);
    return;
}
    // cl_platform_id platform;
    // platform = findPlatform("Intel(R) FPGA SDK for OpenCL(TM)");
    // if (platform == NULL)
    // {
    //     printf("ERROR: Unable to find Intel FPGA OpenCL platform.\n");
    //     return false;
    // }

    // #include <cstring>
    // using namespace std;
    // #include <iostream>
    // #include "stdio.h"
    // #include "limits.h"
    // int main(){
    //     std::string a("10");
    //     // char b[]="10";
    //     // std::cout << a == b;
    //     std::cout << stoi(a);
    // }


// // A C++ program to move all zeroes at the end of array 
// #include "stdio.h" 
// #include <iostream>
// using namespace std; 

// // Function which pushes all zeros to end of an array. 
// void pushZerosToEnd(int arr[], int n) 
// { 
// 	int count = 0; // Count of non-zero elements 

// 	// Traverse the array. If element encountered is non- 
// 	// zero, then replace the element at index 'count' 
// 	// with this element 
// 	for (int i = 0; i < n; i++) 
// 		if (arr[i] != 0) 
// 			arr[count++] = arr[i]; // here count is 
// 								// incremented 

// 	// Now all non-zero elements have been shifted to 
// 	// front and 'count' is set as index of first 0. 
// 	// Make all elements 0 from count to end. 
// 	// while (count < n) 
// 	// 	arr[count++] = 0; 
// } 

// // Driver program to test above function 
// int main() 
// { 
// 	int arr[] = {1, 9, 8, 4, 0, 0, 2, 7, 0, 6, 0, 9}; 
// 	int n = sizeof(arr) / sizeof(arr[0]); 
// 	pushZerosToEnd(arr, n); 
// 	cout << "Array after pushing all zeros to end of array :n"; 
// 	for (int i = 0; i < n; i++) 
// 		printf("%d ", arr[i]); 
// 	return 0; 
// } 
