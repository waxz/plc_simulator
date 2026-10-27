USING Siemens.Simatic.Communication.OUC;
USING Siemens.Simatic.Communication.OpenUser;
USING Siemens.Simatic.Communication.MQTT;
USING Siemens.Simatic.Communication.Util;
USING System.Strings;
USING System.Serialization;


/*
#### IO map

1. control box
button: start, stop,reset
led, auto,stop,warn
errormsg

2. belt
motor command: enable, speed
motor state: state, speed

3. sensor
IR
Banner

#### Controller Main Logic
1. running state transition
idle -> running -> suspend -> stop
running -> error -> reset -> running

2. belt layout configuration
graph
link table


2. sequence start



*/



TYPE

    Station_1_Input : STRUCT
        button_1_1_1 : BOOL;
        button_1_1_2 : BOOL;
        button_1_1_3 : BOOL;
        sensor_1_1_1 : BOOL;
        button_1_2_1 : BOOL;
        button_1_2_2 : BOOL;
        button_1_2_3 : BOOL;
        sensor_1_2_1 : DWORD;
        button_1_3_1 : BOOL;
        button_1_3_2 : BOOL;
        button_1_3_3 : BOOL;
        sensor_1_3_1 : DWORD;
        clock : LINT;

 
    END_STRUCT;
END_TYPE

TYPE
    Station_1_Output : STRUCT
        led_1_1_1 : BOOL;
        led_1_1_2 : BOOL;
        led_1_1_3 : BOOL;
        led_1_2_1 : BOOL;
        led_1_2_2 : BOOL;
        led_1_2_3 : BOOL;
        led_1_3_1 : BOOL;
        led_1_3_2 : BOOL;
        led_1_3_3 : BOOL;

        belt_1_1_1: ARRAY[0..1] of REAL;
        belt_1_1_2: ARRAY[0..1] of REAL;
        belt_1_1_3: ARRAY[0..1] of REAL;
        belt_1_1_4: ARRAY[0..1] of REAL;
        belt_1_1_5: ARRAY[0..1] of REAL;
        // belt_1_1_6: ARRAY[0..1] of REAL;

        belt_1_2_1: ARRAY[0..1] of REAL;
        belt_1_2_2: ARRAY[0..1] of REAL;
        belt_1_2_3: ARRAY[0..1] of REAL;
        belt_1_2_4: ARRAY[0..1] of REAL;
        belt_1_2_5: ARRAY[0..1] of REAL;

        belt_1_3_1: ARRAY[0..1] of REAL;
        belt_1_3_2: ARRAY[0..1] of REAL;
        belt_1_3_3: ARRAY[0..1] of REAL;
        belt_1_3_4: ARRAY[0..1] of REAL;
        belt_1_3_5: ARRAY[0..1] of REAL;

    END_STRUCT;
END_TYPE