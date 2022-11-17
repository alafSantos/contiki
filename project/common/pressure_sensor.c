#define VARIANCE 10
#include "contiki.h"
#include "pressure_sensor.h"

int random_variance(){
    return rand() % VARIANCE - (VARIANCE / 2);
}
int sensor_get(linear_tank_t* tank){
    unsigned long time_spent = clock_seconds() - tank->last_read;
    int mult;
    switch(tank->state){
        case INC:   mult = 1;break;
        case DEC:   mult = -1;break;
        case CONST: mult = 0;break;
    }
    tank->value = tank->value + tank->slope * time_spent * mult + random_variance();
    if(tank->state == INC) 
        tank->value %= tank->max;
    if(tank->value < 0)
        tank->value = 0;
    
    return tank->value;
}

void tank_init(linear_tank_t* tank,int max, int value,slope_state state,int slope){
    tank->state = state;
    tank->value = value;
    tank->last_read = clock_seconds();
    tank->slope = slope;
    tank->max = max;
}

void change_state(linear_tank_t* tank,slope_state state){
    tank->state = state;
}
static const char* str_value[]= {"INC","DEC","CONST","BAD"};
const char* slope_state_as_str(slope_state s){
    switch (s)
    {
    case INC: return str_value[0];
    case DEC: return str_value[1];
    case CONST: return str_value[2];
    default: return str_value[3];
    }
}