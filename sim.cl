#include "idm.h"
#include "epic_visualizer.h"


__constant fixedpt dt = FIXEDPT_QUARTER;
__constant fixedpt vehicleSize = FIXEDPT_FIVE;

__attribute__((max_global_work_dim(0)))
__kernel void sim(
                        __global CLVehicle * restrict lanes_global,
                        short agentCount,
                        short laneCount,
                        short simulationDuration
                        )
{
    bool state = false;
    CLVehicle lanes[MAX_AGENT_SIZE];
    short laneAgentCount[LANE_MAX * 2];
    short remover[MAX_AGENT_SIZE];
    short removerIndex[LANE_MAX];

    #pragma unroll 1
    for(short i = 0; i < laneCount; i++){
        removerIndex[i] = 0;
        #pragma ivdep
        for(short j = 0; j < MAX_VEHICLES_PER_LANE-2; j++){
            LANES(i,j+1) = LANES_GLOBAL(i,j);
            REMOVER(i,j) = 0;
        }
    }
    find_laneAgentCount(laneAgentCount, lanes, laneCount);
    CLVehicle veh;
    #pragma loop coalesce 3
    for(int i = 0; i < simulationDuration; i++){
        printf("+++++++   ITERATION: %d   +++++++\n", i);
        epic_visualizer_C(lanes, 128, agentCount, laneCount, !state);

        for(short j = 0; j < laneCount; j++){
            for(short k = 1; k < LANE_AGENT_COUNT(j,!state) + 1; k++){
                veh = LANES(j,k);
                // if(veh.id == 0){
                //     break;
                // }
                // fixedpt a = idm(veh.velocity[!state], k != 0 ? LANES(j,k-1).velocity[!state] - veh.velocity[!state] : 0, 50, k != 0 ? LANES(j,k-1).position[!state] - veh.position[!state] - FIXEDPT_FIVE : FIXEDPT_1024 );
                // updateAgent(&veh, a, dt, k != 0 ? LANES(j,k-1).position[!state] - veh.position[!state] - FIXEDPT_FIVE : FIXEDPT_1024, state);
                // LANES(j,k) = veh;
                lc(&veh, j, k, lanes, laneCount, laneAgentCount, state, remover, removerIndex);
            }
        }
        remove(lanes, remover, removerIndex);
        orderLanes(lanes, laneAgentCount, state);
        set_laneAgentCount(laneAgentCount, removerIndex, state);
        state = !state;
    }

    #pragma unroll 1
    for(short i = 0; i < laneCount; i++){
        for(short j = 0; j < MAX_VEHICLES_PER_LANE; j++){
            LANES_GLOBAL(i,j) = LANES(i,j);
        }
    }
}