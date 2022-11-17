typedef enum {
    INC,
    DEC,
    CONST
}slope_state;

typedef struct {
    slope_state state;
    int value,slope,max;
    unsigned long last_read;
}linear_tank_t;

int sensor_get(linear_tank_t* tank);
void tank_init(linear_tank_t* tank,int max, int value,slope_state state,int slope);
void change_state(linear_tank_t* tank,slope_state state);