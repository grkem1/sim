#pragma once

// #include "constants.h"
#include "fixedptc.h"
// https://sourceforge.net/projects/fixedptc/
#define MAX_VEHICLES_PER_LANE 128
#define LANE_MAX 4
#define LANES_(i) &lanes[(i) * MAX_VEHICLES_PER_LANE]
#define LANES(i, j) lanes[(i) * MAX_VEHICLES_PER_LANE + (j)]
#define LANES_GLOBAL(i, j) lanes_global[(i) * MAX_VEHICLES_PER_LANE + (j)]
#define MAX_AGENT_SIZE MAX_VEHICLES_PER_LANE * LANE_MAX
#define REMOVER(i, j) remover[(i) * MAX_VEHICLES_PER_LANE + (j)]
#define LANE_AGENT_COUNT(i, j) laneAgentCount[LANE_MAX * (j) + i] // i: lane, j: state

typedef struct{
    short id;
    fixedpt velocity[2];
    fixedpt position[2];
}CLVehicle;

#if(C!=1)
typedef struct{
    short id;
    float velocity[2];
    float position[2];
}CLVehicleFloat;
#endif

fixedpt idm(fixedpt v, fixedpt dVFront, int vdesired, fixedpt ds){
    fixedpt free_road_term, interaction_term, vRat, ss, temp;
    if(ds <= 0) return fixedpt_rconst(-5); // return something less than minimum allowed acceleration
    // if (v<vdesired){
        vRat = fixedpt_xmul( v, fixedpt_rconst(0.0625) );// /vdesired; // 0.02 originally
        free_road_term = FIXEDPT_1_8;
    // } else {
        // vRat = vdesired/v;
        // free_road_term = -2.0f;
    // }
    fixedpt vRat2 = fixedpt_mul( vRat, vRat );
    vRat = fixedpt_mul( vRat2 , vRat2 );
    free_road_term = fixedpt_xmul( free_road_term, FIXEDPT_ONE - vRat );

    ss = fixedpt_mul( v, FIXEDPT_1_5 ) + ( fixedpt_mul( v, dVFront ) >> 2 ) + FIXEDPT_TWO; // multiplication by 0.26 instead of division by 3.79 // UP: division by 4
    temp = fixedpt_div( ss , ds );
    fixedpt temp2 = fixedpt_mul(temp, temp);
    if(temp2 == FIXEDPT_MAX) return -5;
    interaction_term = -fixedpt_mul( FIXEDPT_1_8 , temp2 );
    return free_road_term + interaction_term;
}


bool updateAgent(CLVehicle *veh, fixedpt acceleration, fixedpt dt, fixedpt ds, bool state){
    acceleration = acceleration > FIXEDPT_FIVE  ? FIXEDPT_FIVE : acceleration;
    acceleration = acceleration < -FIXEDPT_FIVE ? -FIXEDPT_FIVE : acceleration;
    fixedpt dv = fixedpt_mul( acceleration, dt );
    fixedpt v = veh->velocity[!state] + dv;
    v = v < 0 ? 0 : v;
    fixedpt distance = fixedpt_xmul( v, dt );
   
    ds = ds < 0 ? 0 : ds;
    distance = distance > fixedpt_xmul( ds, fixedpt_rconst(0.9) ) ? fixedpt_xmul( ds, fixedpt_rconst(0.9) ) : distance;
    v = distance > fixedpt_xmul( ds, fixedpt_rconst(0.9) ) ? fixedpt_div( distance, dt ) : v;

    veh->position[state] = veh->position[!state] + distance;
    veh->velocity[state] = v;

    return true;
}


void swap(CLVehicle *lanes, short lane, short i1, short i2){  // cok ALUT harciyor
    CLVehicle temp = LANES(lane, i1);
    LANES(lane, i1) = LANES(lane, i2);
    LANES(lane, i2) = temp;
}

void moveZeros(CLVehicle * lanes, short lane, short * laneAgentCount, bool state){
    for(short i = LANE_AGENT_COUNT(lane, state)-1; i > 0; i--)
    {
        if(LANES(lane, i).id == 0){
            short j = i;
            while( LANES(lane, j+1).id != 0 ){
                swap(lanes, lane, j, j+1);
                j++;
            }
        }
    }
}

// https://www.geeksforgeeks.org/move-zeroes-end-array/
// void moveZeros(CLVehicle * lanes, short lane, short * laneAgentCount, bool state){
//     short count = 1;
//     for(short i = 1; i < LANE_AGENT_COUNT(lane, state); i++)
//     {
//         if(LANES(lane, i).id != 0){
//             LANES(lane, count++) = LANES(lane, i);
//         }
//     }
//     while(count < LANE_AGENT_COUNT(lane, state)){
//         memset( &LANES(lane, count), 0, sizeof(CLVehicle) );
//     }

// }

void sortLane(CLVehicle * lanes, short lane, short * laneAgentCount, bool state){ // sort first time. Then move zeros back. -for consistency of indices-  
    for(size_t i = LANE_AGENT_COUNT(lane, !state)+1; i <= LANE_AGENT_COUNT(lane, state); i++) // sondan baslayip basa gelemezsin. cunku sonuncu eleman icin array sortlanmamis
    {
        size_t j = i;
        while( j > 1 && ( LANES(lane, j).position[!state] > LANES(lane, j-1).position[!state] || LANES(lane, j-1).id==0 ) ){
            swap(lanes, lane, j-1, j);
            j--;
        }
    }
}

void orderLanes(CLVehicle *lanes, short * laneAgentCount, bool state){
    for(size_t i = 0; i < LANE_MAX; i++)
    {
        sortLane(lanes, i, laneAgentCount, state);
        moveZeros(lanes, i, laneAgentCount, state);
    }
}

void find_neighbors(CLVehicle * veh, CLVehicle * lanes, bool state, CLVehicle * neighbors, short agentCount, short lane){ // assuming sorted 
    for(short i = 1; i <= agentCount; i++)
    {
        if(LANES(lane, i).position[!state] > veh->position[!state]){
            neighbors[0] = LANES(lane, i);
        }
        if(LANES(lane, agentCount - i + 1).position[!state] < veh->position[!state]){
            neighbors[1] = LANES(lane, agentCount - i + 1);
        }
    }
    return;
}

// // updates vehicle states(pointers, lane) according to changed lane. Needs to have neighbors in the next lane

int lane_change(CLVehicle * veh, CLVehicle * lanes, short lane, short position, short direction, bool state, short * laneAgentCount, short* remover, short * removerIndex){
    LANE_AGENT_COUNT(lane+direction, state)++;
    LANES(lane+direction, laneAgentCount[lane+direction+state*LANE_MAX]) = *veh;
    REMOVER(lane, removerIndex[lane]) = position;  // schedule remove
    removerIndex[lane]++;  
    return 1;
}

int remove(CLVehicle * lanes, short * remover, short * removerIndex){
    #pragma unroll 1
    for(size_t lane = 0; lane < LANE_MAX; lane++)
    {
        short index = removerIndex[lane];
        for(; index > 0; index--)
        {
            #if(C==1)
            memset(&LANES( lane, REMOVER(lane, index-1)), 0, sizeof(CLVehicle) );
            #else
            // std::memset(&LANES( lane, REMOVER(lane, index-1)), 0, sizeof(CLVehicle) );
            LANES(lane, REMOVER(lane, index-1)) = (const CLVehicle){0};
            #endif
            REMOVER(lane, index-1) = 0;
            // memset(&REMOVER(lane, index-1),0,sizeof(fixedpt));
            // LANES( lane, REMOVER(lane, index-1) ) = LANES(0,0);
        }
    }
    // memset(remover, 0, sizeof(short) * MAX_AGENT_SIZE);

    return 1;
}

void find_laneAgentCount(short * laneAgentCount, CLVehicle * lanes, short laneCount){
    short i;
    for(unsigned char lane = 0; lane < laneCount; lane++)
    {
        for(i = 1; i < MAX_VEHICLES_PER_LANE && LANES(lane,i).id!=0; i++){}
        LANE_AGENT_COUNT(lane, 0) = LANE_AGENT_COUNT(lane, 1) = i-1;
    }
}


void set_laneAgentCount(short * laneAgentCount, short * removerIndex, bool state){
    #pragma unroll 1
    for(unsigned char i = 0; i < LANE_MAX; i++)
    {
        LANE_AGENT_COUNT(i, state) -= removerIndex[i];
        LANE_AGENT_COUNT(i, !state) = LANE_AGENT_COUNT(i, state);
        removerIndex[i] = 0;  
    }
}

fixedpt calculateIncentive(fixedpt accThisNew, fixedpt accThisOld, fixedpt politeness, fixedpt accObsNew, fixedpt accObsOld, fixedpt accFreeNew, fixedpt accFreeOld ){
    return ( accThisNew - accThisOld + fixedpt_xmul( politeness, ( accObsNew - accObsOld + accFreeNew - accFreeOld ) ) );
}

int lc(CLVehicle * veh, short lane, short position, CLVehicle * lanes, short laneCount, short * laneAgentCount, bool state, short * remover, short * removerIndex){
    fixedpt politeness = 1; //0.1
    fixedpt safe_decel = -48;//fixedpt_rconst(-3.0);
    fixedpt incThreshold = FIXEDPT_ONE;//fixedpt_rconst(1);
    CLVehicle lefts[2] = { {.id=0, .velocity[0]=0, .velocity[1]=0, .position[0]=0, .position[1]=0}, {.id=0, .velocity[0]=0, .velocity[1]=0, .position[0]=0, .position[1]=0} };
    CLVehicle rights[2] = { {.id=0, .velocity[0]=0, .velocity[1]=0, .position[0]=0, .position[1]=0}, {.id=0, .velocity[0]=0, .velocity[1]=0, .position[0]=0, .position[1]=0} };
    // lefts[0] = lefts[1] = rights[0] = rights[1] = LANES(0,0);
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

    #pragma unroll
    for(unsigned char i = 0; i < 2; i++)
    {
        if( (i == 0 && lane == 0) || (i == 1 && lane == laneCount - 1 ))continue;
        find_neighbors(veh, lanes, state, neighbors[i], LANE_AGENT_COUNT(lane+(2*i-1), !state), lane+2*i-1 );
    }
    
    //     find_neighbors(veh,vehs[heads[veh->lane[!state] - 1]], vehs, state, lefts);
    // find_neighbors(veh,vehs[heads[veh->lane[!state] + 1]], vehs, state, rights);

    vels[0] = lefts[1].velocity[!state];
    vels[1] = lefts[0].velocity[!state];
    vels[2] = LANES(lane, position+1).velocity[!state];
    vels[3] = LANES(lane, position-1).velocity[!state];
    vels[4] = rights[1].velocity[!state];
    vels[5] = rights[0].velocity[!state];
    vels[6] = veh->velocity[!state];

    pos[0] = lefts[1].position[!state];
    pos[1] = lefts[0].position[!state];
    pos[2] = LANES(lane,position+1).position[!state];
    pos[3] = LANES(lane,position-1).position[!state];
    pos[4] = rights[1].position[!state];
    pos[5] = rights[0].position[!state];
    pos[6] = veh->position[!state];

    #if(C==1)
    uchar arg_ind_r[9] = {0,0,2,2,4,4,6,6,6};
    uchar arg_ind_l[9] = {1,6,6,3,5,6,3,1,5};
    #else
    unsigned char arg_ind_r[9] = {0,0,2,2,4,4,6,6,6};
    unsigned char arg_ind_l[9] = {1,6,6,3,5,6,3,1,5};   
    #endif

    // args[0][0] = args[1][0] = vels[0];
    // args[2][0] = args[3][0] = vels[2];
    // args[4][0] = args[5][0] = vels[4];
    // args[6][0] = args[7][0] = args[8][0] = vels[6];
    // args[0][1] = vels[1] - vels[0];
    // args[1][1] = vels[6] - vels[0];
    // args[2][1] = vels[6] - vels[2];
    // args[3][1] = vels[3] - vels[2];
    // args[4][1] = vels[5] - vels[4];
    // args[5][1] = vels[6] - vels[4];
    // args[6][1] = vels[3] - vels[6];
    // args[7][1] = vels[1] - vels[6];
    // args[8][1] = vels[5] - vels[6];
    // args[0][2] = pos[1] - pos[0];
    // args[1][2] = pos[6] - pos[0];
    // args[2][2] = pos[6] - pos[2];
    // args[3][2] = pos[3] - pos[2];
    // args[4][2] = pos[5] - pos[4]; 
    // args[5][2] = pos[6] - pos[4];
    // args[6][2] = pos[3] - pos[6];
    // args[7][2] = pos[1] - pos[6];
    // args[8][2] = pos[5] - pos[6];

    #pragma unroll
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


    #pragma unroll
    for(size_t i = 0; i < 2; i++)
    {
        incentives[i] = calculateIncentive( results[7+i], results[6], politeness, results[1 + 4*i], results[4*i], results[3], results[2] );
    }
    want_left = want_left && (incentives[0] > incThreshold);
    want_right = want_right && (incentives[1] > incThreshold);
    

    bool prefer_left = true;

    ///////ACT////////

    if(want_left && (!want_right || incentives[0] > incentives[1] || (incentives[0] == incentives[1] && prefer_left))) {
        updateAgent(veh, results[7], FIXEDPT_QUARTER, (lefts[0].id != 0) ? lefts[0].position[!state]-veh->position[!state]-FIXEDPT_FIVE : FIXEDPT_1024, state);
        lane_change(veh, lanes, lane, position, -1, state, laneAgentCount, remover, removerIndex);
        return -1;
    } else if(want_right) {
        updateAgent(veh, results[8], FIXEDPT_QUARTER, (rights[0].id != 0) ? rights[0].position[!state]-veh->position[!state]-FIXEDPT_FIVE : FIXEDPT_1024, state);
        lane_change(veh, lanes, lane, position, 1, state, laneAgentCount, remover, removerIndex);
        return 1;
    }
    updateAgent(veh, results[6], FIXEDPT_QUARTER, LANES(lane, position-1).id!=0 ? LANES(lane, position-1).position[!state]-veh->position[!state]-FIXEDPT_FIVE : FIXEDPT_1024, state);
    LANES(lane, position) = *veh;
    return 0;
}



////////////////////////// idm_float.h /////////////////////////////

// #if(C!=1)

// float idm_float(float v, float dVFront, int vdesired, float ds){
//     float free_road_term, interaction_term, vRat, ss, temp;
//     if(ds <= 0) return -5;

//     // if (v<vdesired){
//         vRat =  v * 0.0625;// /vdesired;
//         free_road_term = 1.8;
//     // } else {
//         // vRat = vdesired/v;
//         // free_road_term = -2.0f;
//     // }
//     vRat = pow(vRat, 4);
//     free_road_term = free_road_term * (1 - vRat);

//     ss =  v * 1.5 + v * dVFront / 4 + 2; // multiplication by 0.26 instead of division by 3.79 // UP: division by 4
//     temp = ss / ds;
//     interaction_term = -1.8f * temp * temp;
//     return free_road_term + interaction_term;
// }

// bool updateAgent_float(CLVehicle *veh, float acceleration, float dt, float ds, bool state){
//     acceleration = acceleration > 5  ? 5 : acceleration;
//     acceleration = acceleration < -5 ? -5 : acceleration;
//     fixedpt dv = acceleration * dt;
//     fixedpt v = veh->velocity[!state] + dv;
//     v = v < 0 ? 0 : v;
//     float distance = v * dt;
   
//     ds = ds < 0 ? 0 : ds;
//     if(distance > ds * 0.9){
//         distance = ds * 0.9;
//         v = distance / dt;
//     }
//     veh->position[state] = veh->position[!state] + distance;
//     veh->velocity[state] = v;

//     return true;
// }

// float calculateIncentive_float(float accThisNew, float accThisOld, float politeness, float accObsNew, float accObsOld, float accFreeNew, float accFreeOld ){
//     return ( accThisNew - accThisOld + politeness * ( accObsNew - accObsOld + accFreeNew - accFreeOld ) ) ;
// }

// int lc_float(CLVehicle * veh, short lane, short position, CLVehicle * lanes, short laneCount, short * laneAgentCount, bool state, short * remover, short * removerIndex){ // position: position in the lane, nth car when sorted through positions
//     float politeness = 0.1;
//     float safe_decel = -3;//fixedpt_rconst(-3.0);
//     float incThreshold = 1;//fixedpt_rconst(1);
//     CLVehicle lefts[2], rights[2];
//     // lefts[0] = lefts[1] = rights[0] = rights[1] = LANES(0,0);
//     memset(lefts, 0, sizeof(CLVehicle)*2);
//     memset(rights, 0, sizeof(CLVehicle)*2);
//     CLVehicle *neighbors[2];
//     neighbors[0] = lefts;
//     neighbors[1] = rights;

//     // short incentiveLeft, incentiveRight;
//     float incentives[2];

//     bool want_left = true;
//     bool want_right = true;

//     float args[9][3];
//     float results[9]; //= {[0 ... 8] = 32};
//     float vels[7];
//     float pos[7];

//     #pragma unroll
//     for(unsigned char i = 0; i < 2; i++)
//     {
//         if( (i == 0 && lane == 0) || (i == 1 && lane == laneCount - 1 ))continue;
//         find_neighbors(veh, lanes, state, neighbors[i], LANE_AGENT_COUNT(lane+(2*i-1), !state), lane+2*i-1 );
//     }
    
//     vels[0] = lefts[1].velocity[!state];
//     vels[1] = lefts[0].velocity[!state];
//     vels[2] = LANES(lane, position+1).velocity[!state];
//     vels[3] = LANES(lane, position-1).velocity[!state];
//     vels[4] = rights[1].velocity[!state];
//     vels[5] = rights[0].velocity[!state];
//     vels[6] = veh->velocity[!state];

//     pos[0] = lefts[1].position[!state];
//     pos[1] = lefts[0].position[!state];
//     pos[2] = LANES(lane,position+1).position[!state];
//     pos[3] = LANES(lane,position-1).position[!state];
//     pos[4] = rights[1].position[!state];
//     pos[5] = rights[0].position[!state];
//     pos[6] = veh->position[!state];

//     #if(C==1)
//     uchar arg_ind_r[9] = {0,0,2,2,4,4,6,6,6};
//     uchar arg_ind_l[9] = {1,6,6,3,5,6,3,1,5};
//     #else
//     unsigned char arg_ind_r[9] = {0,0,2,2,4,4,6,6,6};
//     unsigned char arg_ind_l[9] = {1,6,6,3,5,6,3,1,5};   
//     #endif

//     #pragma unroll
//     for(short i = 0; i < 9; i++)
//     {
//         args[i][1] = vels[arg_ind_l[i]] - vels[arg_ind_r[i]];
//         args[i][2] = pos[arg_ind_l[i]] - pos[arg_ind_r[i]] - FIXEDPT_FIVE;
//         results[i] = 0;
//     }
    
//     #pragma unroll 1
//     for(short i = 0; i < 9; i++)
//     {
//         fixedpt ds = args[i][2];
//         if( (lane==0 && (i==0||i==1||i==7) ) ){
//             want_left = false;
//             continue;
//         }
//         if( (lane==laneCount-1 && (i==4||i==5||i==8) ) ){
//             want_right = false;
//             continue;
//         }
//         if( (lefts[1].id==0&&(i==0||i==1)) || (position==LANE_AGENT_COUNT(lane, !state)&&(i==2||i==3)) || (rights[1].id==0&&(i==4||i==5)) ){
//             continue;
//         }
//         if( (lefts[0].id==0&&(i==0||i==7)) || (position==1&&(i==3||i==6)) || ( rights[0].id==0&&(i==4||i==8) ) ){
//             ds = FIXEDPT_1024;
//         }
//         results[i] = idm_float(args[i][0], args[i][1], 50, ds);
//     }
//     want_left = want_left && (results[1] >= safe_decel);
//     want_right = want_right && (results[5] >= safe_decel);


//     #pragma unroll
//     for(size_t i = 0; i < 2; i++)
//     {
//         incentives[i] = calculateIncentive_float( results[7+i], results[6], politeness, results[1 + 4*i], results[4*i], results[3], results[2] );
//     }
//     want_left = want_left && (incentives[0] > incThreshold);
//     want_right = want_right && (incentives[1] > incThreshold);
    

//     bool prefer_left = true;

//     ///////ACT////////

//     if(want_left && (!want_right || incentives[0] > incentives[1] || (incentives[0] == incentives[1] && prefer_left))) {
//         updateAgent_float(veh, results[7], dt, (lefts[0].id != 0) ? lefts[0].position[!state]-veh->position[!state]-5 : 1024, state);
//         lane_change(veh, lanes, lane, position, -1, state, laneAgentCount, remover, removerIndex);
//         return -1;
//     } else if(want_right) {
//         updateAgent_float(veh, results[8], dt, (rights[0].id != 0) ? rights[0].position[!state]-veh->position[!state]-5 : 1024, state);
//         lane_change(veh, lanes, lane, position, 1, state, laneAgentCount, remover, removerIndex);
//         return 1;
//     }
//     updateAgent(veh, results[6], dt, LANES(lane, position-1).id!=0 ? LANES(lane, position-1).position[!state]-veh->position[!state]-5 : 1024, state);
//     LANES(lane, position) = *veh;
//     return 0;
// }

// #endif
