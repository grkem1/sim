#include "idm.h"
#include "epic_visualizer.h"


__constant fixedpt dt = FIXEDPT_QUARTER;
__constant fixedpt vehicleSize = FIXEDPT_FIVE;

//__attribute__((max_global_work_dim(0)))
__kernel void sim(
                        __global CLVehicle * lanes_global,
                        short agentCount,
                        short laneCount,
                        short simulationDuration
                        )
{
    bool state = false;
    CLVehicle lanes[MAX_AGENT_SIZE]; //__attribute__((xcl_array_partition(cyclic,2,1)));
//	#pragma HLS array_partition variable=lanes complete


    short laneAgentCount[LANE_MAX * 2];
    short remover[MAX_AGENT_SIZE];
    short removerIndex[LANE_MAX];

//    #pragma unroll 1
    for(short i = 0; i < laneCount; i++){
        removerIndex[i] = 0;
//        #pragma ivdep
        for(short j = 0; j < MAX_VEHICLES_PER_LANE-2; j++){
            LANES(i,j) = LANES_GLOBAL(i,j);
            REMOVER(i,j) = 0;
        }
    }

//    mem_fence(CLK_LOCAL_MEM_FENCE);
    barrier(CLK_LOCAL_MEM_FENCE);

    find_laneAgentCount(laneAgentCount, lanes, laneCount);
    CLVehicle veh;
//    #pragma loop coalesce 3
    for(int i = 0; i < simulationDuration; i++){
//        printf("+++++++   ITERATION: %d   +++++++\n", i);
//        epic_visualizer_C(lanes, 128, agentCount, laneCount, !state);

        for(short j = 0; j < laneCount; j++){
            for(short k = 1; k < LANE_AGENT_COUNT(j,!state) + 1; k++){
                veh = LANES(j,k);
                 if(veh.id == 0){
                     break;
                 }
//                 fixedpt a = idm(veh.velocity[!state], k != 0 ? LANES(j,k-1).velocity[!state] - veh.velocity[!state] : 0, 50, k != 0 ? LANES(j,k-1).position[!state] - veh.position[!state] - FIXEDPT_FIVE : FIXEDPT_1024 );
//                 updateAgent(veh, a, dt, k != 0 ? LANES(j,k-1).position[!state] - veh.position[!state] - FIXEDPT_FIVE : FIXEDPT_1024, state);
//                 LANES(j,k) = veh;
//                 __attribute__((inline))
//                lc(veh, j, k, lanes, laneCount, laneAgentCount, state/*, remover, removerIndex*/);
                 short lane = j;
                 short position = k;

                 fixedpt politeness = 1; //0.1
                 fixedpt safe_decel = -48;//fixedpt_rconst(-3.0);
                 fixedpt incThreshold = FIXEDPT_ONE;//fixedpt_rconst(1);
                 // CLVehicle lefts[2] = { {.id=0, .velocity[0]=0, .velocity[1]=0, .position[0]=0, .position[1]=0}, {.id=0, .velocity[0]=0, .velocity[1]=0, .position[0]=0, .position[1]=0} }; // commented out because compiler version does not recognize this type of initialization
                 // CLVehicle rights[2] = { {.id=0, .velocity[0]=0, .velocity[1]=0, .position[0]=0, .position[1]=0}, {.id=0, .velocity[0]=0, .velocity[1]=0, .position[0]=0, .position[1]=0} };
                 CLVehicle lefts[2];
                 CLVehicle rights[2];
                 lefts[1].id = lefts[1].velocity[0] = lefts[1].velocity[1] = lefts[1].position[0] = lefts[1].position[1] = 0;
                 lefts[0].id = lefts[0].velocity[0] = lefts[0].velocity[1] = lefts[0].position[0] = lefts[0].position[1] = 0;
                 rights[1].id = rights[1].velocity[0] = rights[1].velocity[1] = rights[1].position[0] = rights[1].position[1] = 0;
                 rights[0].id = rights[0].velocity[0] = rights[0].velocity[1] = rights[0].position[0] = rights[0].position[1] = 0;
                 // lefts[0] = lefts[1] = rights[0] = rights[1] = LANES(0,0); copy is expensive
                 CLVehicle *neighbors[2];
                 neighbors[0] = lefts;
                 neighbors[1] = rights;

                 // short incentiveLeft, incentiveRight;
                 fixedpt incentives[2];

                 bool want_left = true;
                 bool want_right = true;

                 fixedpt args[9][3];
                 fixedpt results[9]; //= {[0 ... 8] = 32};
                 fixedpt vels[7];
                 fixedpt pos[7];

                 // #pragma unroll
//                 for(unsigned char i = 0; i < 2; i++)
//                 {
//                     if( (i == 0 && lane == 0) || (i == 1 && lane == laneCount - 1 ))continue;
//                     find_neighbors(veh, lanes, state, neighbors[i], LANE_AGENT_COUNT(lane+(2*i-1), !state), lane+2*i-1 );
//                 }

                 if( lane > 0 )
                 find_neighbors(veh, lanes, state, lefts, LANE_AGENT_COUNT(lane-1, !state), lane-1 );
                 if( lane < laneCount-1 )
                 find_neighbors(veh, lanes, state, rights, LANE_AGENT_COUNT(lane+1, !state), lane+1 );

                 //     find_neighbors(veh,vehs[heads[veh->lane[!state] - 1]], vehs, state, lefts);
                 // find_neighbors(veh,vehs[heads[veh->lane[!state] + 1]], vehs, state, rights);

                 vels[0] = lefts[1].velocity[!state];
                 vels[1] = lefts[0].velocity[!state];
                 vels[2] = LANES(lane, position+1).velocity[!state];
                 vels[3] = LANES(lane, position-1).velocity[!state];
                 vels[4] = rights[1].velocity[!state];
                 vels[5] = rights[0].velocity[!state];
                 vels[6] = veh.velocity[!state];

                 pos[0] = lefts[1].position[!state];
                 pos[1] = lefts[0].position[!state];
                 pos[2] = LANES(lane,position+1).position[!state];
                 pos[3] = LANES(lane,position-1).position[!state];
                 pos[4] = rights[1].position[!state];
                 pos[5] = rights[0].position[!state];
                 pos[6] = veh.position[!state];

//                 #if(C==1)
                 const short arg_ind_r[9] = {0,0,2,2,4,4,6,6,6};
                 const short arg_ind_l[9] = {1,6,6,3,5,6,3,1,5};
//                 #else
//                 const short arg_ind_r[9] = {0,0,2,2,4,4,6,6,6};
//                 const short arg_ind_l[9] = {1,6,6,3,5,6,3,1,5};
//                 #endif

                 // #pragma unroll
                 for(short i = 0; i < 9; i++)
                 {
                     args[i][1] = vels[arg_ind_l[i]] - vels[arg_ind_r[i]];
                     args[i][2] = pos[arg_ind_l[i]] - pos[arg_ind_r[i]] - FIXEDPT_FIVE;
                     results[i] = 0;
                 }

                  #pragma unroll 1
                 for(short i = 0; i < 9; i++)
                 {
                     fixedpt ds = args[i][2];
                     if( (lane==0 && (i==0||i==1||i==7) ) ){
                         want_left = false;
                         continue;
                     }
                     if( (lane==laneCount-1 && (i==4||i==5||i==8) ) ){
                         want_right = false;
                         continue;
                     }
                     if( (lefts[1].id==0&&(i==0||i==1)) || (position==LANE_AGENT_COUNT(lane, !state)&&(i==2||i==3)) || (rights[1].id==0&&(i==4||i==5)) ){
                         continue;
                     }
                     if( (lefts[0].id==0&&(i==0||i==7)) || (position==1&&(i==3||i==6)) || ( rights[0].id==0&&(i==4||i==8) ) ){
                         ds = FIXEDPT_1024;
                     }
                     results[i] = idm(args[i][0], args[i][1], 50, ds);
                 }
                 want_left = want_left && (results[1] >= safe_decel);
                 want_right = want_right && (results[5] >= safe_decel);


                 // #pragma unroll
//                 for(short i = 0; i < 2; i++)
//                 {
//                     incentives[i] = calculateIncentive( results[7+i], results[6], politeness, results[1 + 4*i], results[4*i], results[3], results[2] );
//                 }
                 incentives[0] = calculateIncentive( results[7], results[6], politeness, results[1], results[0], results[3], results[2]);
                 incentives[1] = calculateIncentive( results[8], results[6], politeness, results[5], results[4], results[3], results[2]);

                 want_left = want_left && (incentives[0] > incThreshold);
                 want_right = want_right && (incentives[1] > incThreshold);


                 bool prefer_left = true;

                 ///////ACT////////

                 if(want_left && (!want_right || incentives[0] > incentives[1] || (incentives[0] == incentives[1] && prefer_left))) {
                     updateAgent(&veh, results[7], FIXEDPT_QUARTER, (lefts[0].id != 0) ? lefts[0].position[!state]-veh.position[!state]-FIXEDPT_FIVE : FIXEDPT_1024, state);
                     lane_change(veh, lanes, lane, position, -1, state, laneAgentCount, remover, removerIndex);
//                     return -1;
                     continue;
                 } else if(want_right) {
                     updateAgent(&veh, results[8], FIXEDPT_QUARTER, (rights[0].id != 0) ? rights[0].position[!state]-veh.position[!state]-FIXEDPT_FIVE : FIXEDPT_1024, state);
                     lane_change(veh, lanes, lane, position, 1, state, laneAgentCount, remover, removerIndex);
//                     return 1;
                     continue;
                 }
                 updateAgent(&veh, results[6], FIXEDPT_QUARTER, LANES(lane, position-1).id!=0 ? LANES(lane, position-1).position[!state]-veh.position[!state]-FIXEDPT_FIVE : FIXEDPT_1024, state);
                 LANES(lane, position) = veh;

            }
        }
        barrier(CLK_LOCAL_MEM_FENCE);
        remove(lanes, remover, removerIndex);
        barrier(CLK_LOCAL_MEM_FENCE);
        orderLanes(lanes, laneAgentCount, state);
        barrier(CLK_LOCAL_MEM_FENCE);
        set_laneAgentCount(laneAgentCount, removerIndex, state);
        state = !state;
    }

    barrier(CLK_LOCAL_MEM_FENCE);
//    #pragma unroll 1
    for(short i = 0; i < laneCount; i++){
        for(short j = 0; j < MAX_VEHICLES_PER_LANE; j++){
            LANES_GLOBAL(i,j) = LANES(i,j);
        }
    }
}
