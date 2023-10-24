// variables needed for pid control function

/*
const float p_gain = 1;
const float d_gain = 1;
const float i_gain = 1;
*/

const float p_gain = 0.5;
const float d_gain = 1;
const float i_gain = 1;

const float p_gain_curve = 1;

float integral;
float proportional;
float derivative;

float last_error;
