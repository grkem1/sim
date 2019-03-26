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

using namespace aocl_utils;
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

    size_t globalWorkSize = 1;
    size_t localWorkSize = 1;
    bool useCpu = false;
    std::string aocx;
    bool scan = false;
    int iteration = 10;       
    bool emulate = false;        // select Kernel if to be worked on computer
    bool singleWorkItem = true; // select Kernel(swi) and adapt iteration count  /// TRUE BY DEFAULT -- change if not-so-pure version is to be used -- 
    bool &swi = singleWorkItem;

    cl_context m_context = NULL;
    cl_command_queue m_command_queue = NULL;
    cl_mem m_memObjects[1]; // lanes;
    cl_program m_program = NULL;
    cl_kernel m_kernel = NULL;

//////////// OUTPUT STREAMS /////////

std::ofstream clock_cycles("Results/clock_cycles.txt");
std::ofstream cpu_time("Results/cpu_time.txt");
std::ofstream fpga_time("Results/fpga_time.txt");
// std::ofstream positions("Results/positions.txt");


/////////// HELPER FUNCTIONS ///////////

template <class T>
void printArray(T * & array, int size){
        for(size_t i = 0; i < size; i++)
    {
        std::cout << array[i] << " ";
    }
    std::cout << std::endl;
}

template <class U>
void writeArray(std::ofstream & out, U * array, int size){ 
    for(size_t i = 0; i < size; i++)
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

// void epic_visualizer(const CLVehicle * vehs, int length, int vehcount, int laneCount = 3, bool fixedpoint = true){
//     bool state=true;
//     int proportion;
//     int size;
//     std::vector< std::vector<int> > road;
//     road.resize(laneCount);

//     for(proportion = 1;;proportion++){
//         if(length/proportion <= 128){
//             size = length / proportion;
//             break;
//         }
//     }

//     printf("\n");
//     for(int k=0;k<size;k++){
//         printf("=");
//     }
//     printf("\n");

//     for(size_t i = 0; i < vehcount; i++)
//     {
//         road[vehs[i].lane[state]].push_back(fixedpoint ? ( fixedpt_tofloat(vehs[i].position[state]) ) : vehs[i].position[state]);
//     }
//     for(size_t i = 0; i < laneCount; i++){
//         std::transform(road[i].begin(), road[i].end(), road[i].begin(), [proportion](int& c){return c / proportion;});
//         std::sort(road[i].begin(), road[i].end());
//     }

//     int j;
//     for(size_t k = 0; k < laneCount; k++){
//         j = 0;
//         for(size_t i = 0; i < size; i++)
//         {
//             if(road[k].size() == 0){
//                 for(int k=0;k<size;k++){
//                     printf("-");
//                 }
//                 printf("\n");
//                 break;
//             }
//             if(i == road[k][j]){
//                 printf("0");
//                 j++;
//                 if(j == road[k].size()){
//                     j--;
//                 }
//             }else{
//             printf("-");
//             }
//         }
//         printf("\n");
//     }

//     for(int k=0;k<size;k++){
//         printf("=");
//     }
//     printf("\n");
// }

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
    Options options(argc,argv);

    for(short i=1; i < argc; i++){
        std::string arg = argv[i];   
        if(arg == "c" || arg == "cpu"){
            useCpu = true;
        }
        else if(argv == "w")){
            localWorkSize = (int)argv;
        }
        else if(arg == "l"){
            laneCount = options.get<int>("l");
        }
        else if(arg == "scan"){
            scan = true;
        }
        else if(arg == "it"){
            iteration = options.get<int>("it");
        }
        else if(arg == "emulate"){
            emulate=true;
        }
        else if(arg == "swi"){
            singleWorkItem = true;
        }
        else if(arg == "aocx"){
            aocx = options.get<std::string>("aocx");
        }
        else if(arg == "agents"){
            agentsize = options.get<int>("agents");
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
        cl_device_id device_id = NULL;
        cl_int ret;
        std::cout << "Initialising Kernel on the FPGA" << std::endl;

        cl_uint num_of_platforms;
        cl_uint num_of_devices = 0;
        int platform_iid;

        if (clGetPlatformIDs(0, NULL, &num_of_platforms) != CL_SUCCESS)
        {
            std::cout << "Unable to get platform_id" << std::endl;
            exit(1);
        }
        cl_platform_id *platform_ids = new cl_platform_id[num_of_platforms];
        if (clGetPlatformIDs(num_of_platforms, platform_ids, NULL) != CL_SUCCESS)
        {
            std::cout << "Unable to get platform_id" << std::endl;
            exit(1);
        }

        printf("Platform: %s\n", getPlatformName(*platform_ids).c_str());

        bool found = false;
        for (int i = 0; i < num_of_platforms; i++)
            if (clGetDeviceIDs(platform_ids[i], CL_DEVICE_TYPE_ALL, 1, &device_id, &num_of_devices) == CL_SUCCESS)
            {
                printf("  %s\n", getDeviceName(device_id).c_str() );
                found = true;
                std::cout << "Found FPGA at platform " << i << " device_id:" << device_id << std::endl;
                platform_iid = i;
                break;
            }
        if (!found)
        {
            std::cout << "Can not find any OpenCL device" << std::endl;
            exit(1);
        }


        cl_context_properties contextProperties[] =
            {
                CL_CONTEXT_PLATFORM,
                (cl_context_properties)platform_ids[platform_iid],
                0};

        m_context = clCreateContextFromType(contextProperties, CL_DEVICE_TYPE_ALL,
                                                NULL, NULL, &ret);
        delete platform_ids;
        if(ret != CL_SUCCESS){
            std::cout << "error creating context" << std::endl;
        }

        const cl_queue_properties props[] = {
            CL_QUEUE_PROPERTIES, (const cl_queue_properties)(CL_QUEUE_PROFILING_ENABLE), 0
        };
        // m_command_queue = clCreateCommandQueueWithProperties(m_context, device_id, props, &ret);
        // m_command_queue = clCreateCommandQueue(m_context, device_id, CL_QUEUE_PROFILING_ENABLE /*0*/, &ret);
        // if(ret != CL_SUCCESS)printError(ret);

        OCL_CHECK(ret, cl::CommandQueue m_command_queue(m_context, device_id, NULL,NULL,NULL,&ret));

        /* Create Memory Buffer */

        double create_start = getCurrentTimestamp();
        m_memObjects[0] = clCreateBuffer(m_context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                            sizeof(CLVehicle) * MAX_AGENT_SIZE, lanes, &ret);
        double create_stop = getCurrentTimestamp();
        double create_duration = create_stop - create_start;
        total_execution += create_duration;
        std::cout << " Create Duration : " << create_duration << std::endl; 

        if (ret != CL_SUCCESS) {
            std::cerr<<"alloc buffer error" <<ret<< " " <<std::endl;
            printError(ret) ;
            std::cout << std::endl;
            return ;
        }
        /* Create Kernel Program from the binary */

        int num_devices = 1;

        std::string fileName("sim");
        if(emulate) fileName += "_emulator";
        if(aocx != "") fileName = aocx;
        std::string binary_file = getBoardBinaryFile(fileName.c_str(), device_id);
        printf("Using AOCX: %s\n", binary_file.c_str());
        m_program = createProgramFromBinary(m_context, binary_file.c_str(), &device_id, num_devices);
        ret = clBuildProgram(m_program, 1, &device_id, NULL, NULL, NULL);

        if (ret != CL_SUCCESS) {
            // Determine the reason for the error
            char buildLog[16384];
            clGetProgramBuildInfo(m_program, device_id, CL_PROGRAM_BUILD_LOG,
                                sizeof(buildLog), buildLog, NULL);
            std::cerr << "Error in kernel: " << std::endl;
            std::cerr << buildLog;
            clReleaseProgram(m_program);
            return ;
        }

        /* Create OpenCL Kernel */
        m_kernel = clCreateKernel(m_program, "sim", &ret);
        if (ret != CL_SUCCESS) {
            std::cerr<<"create kernel error"<<std::endl;
            printError(ret);
            return ;
        }
        /* Set OpenCL Kernel Parameters */
        ret = clSetKernelArg(m_kernel,  0, sizeof(cl_mem), &m_memObjects[0]);
        ret |= clSetKernelArg(m_kernel, 1, sizeof(short), &agentsize);
        ret |= clSetKernelArg(m_kernel, 2, sizeof(short), &laneCount);
        ret |= clSetKernelArg(m_kernel, 3, sizeof(short), &iteration);

        if (ret != CL_SUCCESS) {
            std::cerr<<"memory bind error"<<std::endl;
            return ;
        }

    }else if(useCpu == true){
    }
//    free(platforms);
//    free(devices);
}

int executeKernel(){
    ex_kernel_start = getCurrentTimestamp();

    if( useCpu == false ){

        cl_int ret = 0;

        if(singleWorkItem) { globalWorkSize = 1; localWorkSize = 1; }

        ret = 0;

        fpga_start_time = getCurrentTimestamp(); 
        for(int i = 0; i < iteration; i++){
            iteration_start = getCurrentTimestamp();
            ret |= clEnqueueNDRangeKernel(m_command_queue, m_kernel, 1, NULL,
                                        &globalWorkSize, &localWorkSize,
                                        0, NULL, /*&kernel_execution*/NULL);
            clFinish(m_command_queue);
            // clEnqueueReadBuffer(m_command_queue, m_memObjects[2], CL_TRUE, 0, MAX_AGENT_SIZE * sizeof(int), m_position, 0, NULL, NULL);
            iteration_end = getCurrentTimestamp();
            // std::cout << i << " " << (iteration_end - iteration_start) * 1e6 << std::endl;
            if(singleWorkItem) break;
        }
        clEnqueueReadBuffer(m_command_queue, m_memObjects[0], CL_TRUE, 0, MAX_AGENT_SIZE * sizeof(CLVehicle), lanes, 0, NULL, NULL);

        fpga_end_time = getCurrentTimestamp();
        execution_time = fpga_end_time - fpga_start_time;
        total_execution += execution_time;

        // time_exec = getStartEndTime(kernel_execution);

        if (ret != CL_SUCCESS) {
            std::cerr<<"kernel launch error "<<ret<<std::endl;
            printError(ret);
            return false;
        }
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

    find_laneAgentCount(laneAgentCount, lanes, laneCount);
    CLVehicle veh;
   
    return true;
}

void releaseKernel(){
    cl_int ret;
    execution_time = 0;
    total_execution = 0;
    if(useCpu == false){
        ret =  clReleaseProgram(m_program);
        for (int i = 0; i < 3; i++)
        {
            clReleaseMemObject(m_memObjects[i]);
        }
        ret |=  clReleaseCommandQueue(m_command_queue);
        ret |=  clReleaseContext(m_context);
        ret |=  clReleaseKernel(m_kernel);
        if (ret != CL_SUCCESS) {
            std::cerr<<"release error"<<std::endl;
            return;
        }
    }
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
    // #include "Kernel/fixedptc.h"
    // #include "stdio.h"
    // #include "limits.h"
    // #include "Kernel/idm.h"
    // int main(){
    //     int z[100];
    //     unsigned char a = 5;
    //     unsigned char b = 10;
    //     std::cout << z[a*b];
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
