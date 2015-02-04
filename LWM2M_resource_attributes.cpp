/*
LWM2M notification attribute example
------------------------------------------------
Copyright (c) 2006-2015 ARM Limited
 
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
 
http://www.apache.org/licenses/LICENSE-2.0
 
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
------------------------------------------------

Supports a specific interpretation of the LWM2M Write Attributes
pmin, pmax, gt, lt, and step

Functional implementation and interpretation is described in the comments below
 
Library specific code for handling sensor I/O, CoAP resources, notifications 
and receiving Write Attributes is removed for clarity

*/


/*
 
LWM2M Observe attributes for float and int data, for string and bool only
implement pmin and pmax. 
 
This implements the LWM2M 1.0 Notification attributes based on the following 
interpretation:

1. lt and gt define three signal bands or states, for example low alarm, normal, 
and high alarm. It is desired to transmit a notification whenever the measured
variable is in a different band from the last reported band, that is from nornal 
to high alarm, and from high alarm to normal, and all other possible state
transitions, e.g. high to low, etc. subject to pmin (see below)

2. step defines a minimum change needed in the measured variable relative to 
the last reported value to trigger a new report of the value. For example, if 
the last reported value was 51.0, and the step variable was set at 1.0, the 
signal would need to be greater than or equal to 52.0, or less than or equal 
to 50.0 to trigger a new report (notification). Each time the measured variable 
is reported, the upper and lower values required to trigger a new report are 
updated relative to the reported value.

3. pmin defines the minumum desired reporting interval. pmin is assumed to 
define a mandatory quiet period between notifications. a notification here is 
assumed to contain at least a single value. Therefore if a combination of 
potentially reportable events are triggered by lt, gt, and step during the 
quiet period, the value at the end of the quiet period MUST be reported 
(sent in a notification). Additional values captured during the quiet period 
MAY be sent along with the current value in a notification object.

4. pmax defines the largest time interval allowed without a notification. Each 
time a notification is sent, a timer is reset, set to expire at pmax seconds 
after the notification was sent. If the pmax timer expires, a notification is sent.
 
Note: each time a notification is sent, pmin and pmax timers are are restarted.
 
Implementation Notes:

The algorithm for lt and gt is generalized to accept from one to n limit values, 
each defining a boundary between n+1 signal bands (states). A transition from any 
state to any other state will create a reportable event.

The simple implementation below does not capture information about state changes 
that occur during the quiet period, only that some state transition has occurred, 
with a reporting of the current value at the end of the quiet period. 
Implementations MAY capture state transitions which occur during the quiet period 
and report them at the end of the quiet period along with the current value in a 
notification object (senml+json or lwm2m format tlv)
 
I.e., implementations MAY queue reportable events to be scheduled as a sequence 
object in the next notification.
*/


// data type float - int could be used but just convert/cast
typedef float sample; 

//algorithm can accept any number of limit values and report when signal changes
//between limit bands
#define MAX_LIMITS 2
static int num_limits = 2;
static sample limits[MAX_LIMITS];
static int last_band;

// step limit values updated on reporting
static sample high_step, low_step;

// flag for enabling triggering of immediate notification on reportable event
bool pmin_exceeded = false;

// flag for scheduling reporting at the expiration of pmin quiet period
bool report_scheduled = false;

// hardware timers for pmin and pmax 
Ticker pmin_timer, pmax_timer; 

// notification attributes and default values from LWM2M_resource.h
static sample LWM2M_gt = D_GT;
static sample LWM2M_lt = D_LT;
static sample LWM2M_step = D_STEP;
static float LWM2M_pmax = D_PMAX;
static float LWM2M_pmin = D_PMIN;

void LWM2M_notification_init();
void on_update(sample);

// instrumentation about which condition triggered a notification
static bool pmax_exceeded = false;
static bool pmin_trigger = false;

/*
 Functions
 */

/*
 Turn on and off notification based on the latest attributes.
 Update attributes should stop and start to update values
 */
//notifications on
void LWM2M_start_notification()
{
    LWM2M_observing = true;
    LWM2M_notification_init();
}
//notifications off
void LWM2M_stop_notification()
{
    LWM2M_observing = false;
}

/*
 examine one query option to see if the tag matches one of the observe 
 attributes if so, set the corresponding attribute pmin, pmax, lt, gt, 
 step and flag pending update
 */
void set_notification_attribute(char* option)
{
    char* attribute = strtok(option, "="); // first token
    char* value = strtok(NULL, "="); // next token
    
    pc.printf("Setting: %s = %s\r\n", attribute, value);
    
    if (strcmp(attribute, "pmin") == 0){
        sscanf(value, "%f", &LWM2M_pmin);
        attribute_update = true;
        return;
    }
    else if(strcmp(attribute, "pmax") == 0){
        sscanf(value, "%f", &LWM2M_pmax);
        attribute_update = true;
        return;
    }
    else if(strcmp(attribute, "gt") == 0){
        sscanf(value, "%f", &LWM2M_gt);
        attribute_update = true;
        return;
    }
    else if(strcmp(attribute, "lt") == 0){
        sscanf(value, "%f", &LWM2M_lt);
        attribute_update = true;
        return;
    }
    else if(strcmp(attribute, "st") == 0){
        sscanf(value, "%f", &LWM2M_step);
        attribute_update = true;
        return;
    }
    else if(strcmp(attribute, "cancel") == 0){
        LWM2M_stop_notification();
        attribute_update = true;
        return;
    }
}

/* 
 Determine which band [0..num_limits] the provided sample is in.
 Works with any number of bands 2 to MAX_LIMITS+1 using an array 
 of limit settings
*/
int band(sample s)
{
    if (s > limits[num_limits-1]){
        return num_limits;
    }
    else{
        for ( int limit = 0; limit < num_limits; limit++ ){
            if (s <= limits[limit]){
                return limit;
            }
        }
    }
    return -1;
}

/*
 trigger the build and sending of coap observe response sends
 current value
*/
bool send_notification(sample s) 
{
    notify_sample = s; // mailbox
    notification_trigger = true;// trigger notification 
    return true; // async
}

time_t get_current_time()
{
    return time(NULL);
}

/*
demand sensor read or return last callback update
*/
sample get_sample()
{
    // current_sample is most recent ADC read result within 100 mSec
    return current_sample;
}

int report_sample(sample s);//prototype for forward reference

/*
handler for the pmin timer, called when pmin expires 
if no reportable events have occurred, set the pmin expired flag
to inform the report scheduler to report immediately
If a reportable event has occured, report a new sample
*/
void on_pmin()
{
    if (report_scheduled){
        report_scheduled = false;
        pmin_trigger = true; // diagnostic for state machine visibility
        report_sample(get_sample());
    }
    else{
        pmin_exceeded = true; // state machine
        pmin_timer.detach();
    }
    return;
}

/*
handler for pmax timer, report a new sample
*/
void on_pmax()
{
    report_sample(get_sample());
    pmax_exceeded = true; // diagnostic state machine, cleared at reporting
    return;
}

/*
 for reporting a sample that satisfies the reporting criteria and 
 resetting the state machine
*/
int report_sample(sample s)
{
    if(send_notification(s)){  // sends current_sample if observing is on
        last_band = band(s); // limits state machine
        high_step = s + LWM2M_step; // reset floating band upper limit defined by step
        low_step = s - LWM2M_step; // reset floating band lower limit defined by step
        pmin_timer.detach();
        pmin_exceeded = false; // state machine to inhibit reporting at intervals < pmin
        pmin_timer.attach(&on_pmin, LWM2M_pmin);
        pmax_timer.detach();
        pmax_timer.attach(&on_pmax, LWM2M_pmax);
        return 1;
    }
    else return 0;
}

/*
 schedule_report
 this will send the report immediately if pmin is exceeded or else schedule it for
 later, when the pmin timer expires.

 Note that if a reportable event occurs during the pmin "quiet period" and then
 the sample returns to a non-reportable state, the sample will still be reported. T
 his can be used to identify signal excursions within the quiet period that aren't 
 reported otherwise. Implementations MAY queue reportable events to be scheduled 
 as a bulk object notification
*/
void schedule_report(sample s)
{
    if (pmin_exceeded){ 
        // immediate report if pmin is already passed
        report_sample(s);
    }
    else{
        // otherwise, schedule a report for when pmin expires
        // sample and timestamp would be added to the queue here to be batch reported at pmin
        // also would need to reset band and floating limits
        report_scheduled = true;
    }
    return;
}

/*
callback for sensor driver to update the value, e.g. if the sampled value changes
can be called for every sample acquisition
this will evaluate the sample against the reporting criteria and schedule a report 
if a reportable event occurs
*/
void on_update(sample s)// callback from sensor driver, e.g. on changing value 
{
    if (band(s) != last_band || s >= high_step || s <= low_step){ // test limits
        schedule_report(s);
    }
    return;
}

/*
initialize the limits for LWM2M mode and set the state by reporting the first sample
set LWM2M_observing true or false depending on desired behavior
if LWM2M_observing is false, the sample won't be transmitted
*/
void LWM2M_notification_init()
{
    pc.printf("init\r\n");
    limits[0] = LWM2M_lt;
    limits[1] = LWM2M_gt;
    report_sample(get_sample());
    return;
}


