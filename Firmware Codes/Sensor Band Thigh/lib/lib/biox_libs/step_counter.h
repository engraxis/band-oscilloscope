
#include <Adafruit_Sensor.h>

#define STEP_SAMPLES 100

uint16_t sample_counter = 0;
uint16_t steps = 0;

bool flag = false;

// The average acceleration must be below this threshold before the next step can be counted
float still_standing_threshold = 1.9;
// When the avg accelleartion goes beyond this value, a step is counted (provided the value has recently been below the still_standing_threshold)
float movement_threshold = 5.0;

float acc_samples[3][STEP_SAMPLES] = {};
float total_vector[STEP_SAMPLES] = {};
float total_vector_avg[STEP_SAMPLES] = {};

long last_step_time_millis = 0L;
// The minimum time that must pass between steps
long min_step_interval_millis = 1000L;

// Will count two steps if the last step was taken within this interval 
// (because the sensor will not register movement of the other leg)
long double_step_max_interval = 5000L;

void calculate_steps(sensors_event_t* linAcc, uint16_t* num_steps)
{
    *num_steps = steps;
    if(sample_counter >= STEP_SAMPLES)
    {
        // Calculate step
        sample_counter = 0; 
    }
    
    // acc_samples[0][sample_counter] = accel_array[0];
    // acc_samples[1][sample_counter] = accel_array[1];
    // acc_samples[2][sample_counter] = accel_array[2];
    acc_samples[0][sample_counter] = linAcc->acceleration.x;
    acc_samples[1][sample_counter] = linAcc->acceleration.y;
    acc_samples[2][sample_counter] = linAcc->acceleration.z;

    //total_vector[sample_counter] += acc_samples[0][sample_counter] * acc_samples[0][sample_counter];
    total_vector[sample_counter] = acc_samples[0][sample_counter] * acc_samples[0][sample_counter];
    total_vector[sample_counter] += acc_samples[1][sample_counter] * acc_samples[1][sample_counter];
    total_vector[sample_counter] += acc_samples[2][sample_counter] * acc_samples[2][sample_counter];
    total_vector[sample_counter] = sqrt(total_vector[sample_counter]);

    if (sample_counter > 0) {
        // Running average favoring newer measurements
        total_vector_avg[sample_counter] = (total_vector[sample_counter] + total_vector[sample_counter - 1]) / 2 ;
    } else {
        total_vector_avg[sample_counter] = total_vector[sample_counter];
    }
    
    //Serial.print("average: ");
    //Serial.println(total_vector_avg[sample_counter]);
    float new_avg = total_vector_avg[sample_counter];
    sample_counter++;

    long time_since_last_step = millis() - last_step_time_millis;
    bool min_time_has_passed = time_since_last_step >= min_step_interval_millis;

    if (min_time_has_passed && new_avg > movement_threshold && !flag) {
        if(time_since_last_step < double_step_max_interval) {
            steps += 2;
        } else {
            steps += 1;
        }   
        flag = true;
        last_step_time_millis = millis();
        Serial.print("Counted step: ");
        Serial.println(steps);
    } else if (new_avg < still_standing_threshold && flag) {
        flag = false;
    }
    if (steps < 0) {
        steps = 0;
    }

    // Write steps to output parameter
    *num_steps = steps;
}