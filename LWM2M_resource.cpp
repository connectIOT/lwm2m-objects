/*
LWM2M resource template implementation for mbed client
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

Based on the resource template in the mbed demo projects
for nsdl-C and using OMA LWM2M

Supports a specific interpretation of the LWM2M Write Attributes
pmin, pmax, gt, lt, and step

supports setting max-age for cache control

Functional implementation and interpretation is described in the comments below

*/

#include "mbed.h"
#include "rtos.h"
#include "nsdl_support.h"
#include "LWM2M_resource.h"
#include "string.h"

#define LWM2M_RES_ID    "3202/0/5600"
#define LWM2M_RES_RT    "oma.lwm2m"
#define OBS_TRUE 1
#define OBS_FALSE 0

extern Serial pc; 

// settings variables to point to when building response packet
uint8_t LWM2M_max_age = 0; // cache age in seconds, 0=disable caching
uint8_t LWM2M_content_type = 0; // 0=text/plain content-format

// obs variables
static uint8_t LWM2M_obs_number = 0;
static uint8_t *LWM2M_obs_token_ptr = NULL;
static uint8_t LWM2M_obs_token_len = 0;
static uint8_t LWM2M_obs_option; 
// values per: draft-ietf-core-observe-16
// OMA LWM2M CR ref.
#define START_OBS 0
#define STOP_OBS 1

// must be a scalar, ( decimal or integer )
typedef float sample;

// notification attributes and default values from LWM2M_resource.h
static sample LWM2M_gt = D_GT;
static sample LWM2M_lt = D_LT;
static sample LWM2M_step = D_STEP;
static float LWM2M_pmax = D_PMAX;
static float LWM2M_pmin = D_PMIN;

// flag to indicate at least one new attribute is being updated
static bool attribute_update = false;

void LWM2M_notification_init();
void on_update(sample);

// flag set in ISR to enable notifications to be triggered in ISR 
// and sent at thread priority
static bool notification_trigger = false;

// currentValue variables updated upon callback from sensor driver
static sample current_sample = 0, last_sample = 0, notify_sample = 0;

// switch to turn notification on and off
static bool LWM2M_observing = false;

//example for potentiometer or analog sensor reading 0-100%
AnalogIn LWM2M_Sensor(A0); 
char LWM2M_value_string[5];
char LWM2M_update_string[5];

// query string for setting notification attributes (LWM2M write attributes interface)
static char *LWM2M_query_string_ptr = NULL;
char *query_option;
char query_options[5][20];//static for now 
uint8_t num_options = 0;

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
Thread to sample the input and use the update callback on_update when sensor values change.
on_update will run the limits test and set the notification event trigger accordingly.
Also checks for the event trigger and sends a notification packet in this thread.
This allows the notification trigger to be set in a hardware timer ISR without blocking
the ISR with a network operation.
*/
static void LWM2M_notification_thread(void const *args)
{
    while (true){
        wait(.1);
        current_sample = LWM2M_Sensor.read() * (float) 100;
        
        // if this csample is different from last sample, then call the resource on_update
        if((current_sample != last_sample) && LWM2M_observing){
            //pc.printf("LWM2M resource update: %3.1f => %3.1f\r\n", last_sample, current_sample);
            last_sample = current_sample;
            on_update(current_sample); // callback to process notification attributes
        }
        
        // if triggered and in observing mode, build and send the notification packet
        if (notification_trigger && LWM2M_observing){
            
            if (pmax_exceeded){ pc.printf("pmax exceeded\r\n"); pmax_exceeded = false; }
            if (pmin_trigger){pc.printf("pmin trigger\r\n"); pmin_trigger = false; }
            
            sprintf(LWM2M_value_string, "%3.1f", notify_sample); 
            pc.printf("Sending: %s\r\n", LWM2M_value_string);        
            LWM2M_obs_number++;
            if(sn_nsdl_send_observation_notification
                (LWM2M_obs_token_ptr, LWM2M_obs_token_len, 
                (uint8_t*)LWM2M_value_string, strlen(LWM2M_value_string), 
                &LWM2M_obs_number, sizeof(LWM2M_obs_number), 
                COAP_MSG_TYPE_NON_CONFIRMABLE, 0) == 0){
                    
                pc.printf("LWM2M notification failed\r\n");
            }
            else{
                pc.printf("LWM2M notification\r\n");
                notification_trigger = false;
            }
        }
    }
}

/*
examine one query option to see if the tag matches one of the observe attributes
if so, set the corresponding attribute pmin, pmax, lt, gt, step and flag pending update
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
Callback for LWM2M (CoAP REST) operations allowed on the resosurce
GET and PUT methods allowed 
*/
static uint8_t LWM2M_resource_cb(sn_coap_hdr_s *received_coap_ptr, sn_nsdl_addr_s *address, sn_proto_info_s * proto)
{
    sn_coap_hdr_s *coap_res_ptr = 0;

    if(COAP_MSG_CODE_REQUEST_GET == received_coap_ptr->msg_code){
        coap_res_ptr = sn_coap_build_response(received_coap_ptr, COAP_MSG_CODE_RESPONSE_CONTENT);
   
        current_sample = LWM2M_Sensor.read() * (float) 100;
        sprintf(LWM2M_value_string,"%3.1f", current_sample);
        pc.printf("LWM2M resource callback\r\n");
        pc.printf("LWM2M resource state %s\r\n", LWM2M_value_string);
   
        coap_res_ptr->payload_len = strlen(LWM2M_value_string);
        coap_res_ptr->payload_ptr = (uint8_t*)LWM2M_value_string;
        
        coap_res_ptr->content_type_ptr = &LWM2M_content_type;
        coap_res_ptr->content_type_len = sizeof(LWM2M_content_type);
        
        coap_res_ptr->options_list_ptr = (sn_coap_options_list_s*)nsdl_alloc(sizeof(sn_coap_options_list_s));
        if(!coap_res_ptr->options_list_ptr){
            pc.printf("cant alloc option list\r\n");
            coap_res_ptr->options_list_ptr = NULL; //FIXME report error and recover
        }
        else{
            memset(coap_res_ptr->options_list_ptr, 0, sizeof(sn_coap_options_list_s));
            coap_res_ptr->options_list_ptr->max_age_ptr = &LWM2M_max_age;
            coap_res_ptr->options_list_ptr->max_age_len = sizeof(LWM2M_max_age);
        }
        
        if(received_coap_ptr->token_ptr){
            pc.printf("Token included\r\n");
            // replace any existing token
            if(LWM2M_obs_token_ptr){   
                free(LWM2M_obs_token_ptr);
                LWM2M_obs_token_ptr = 0;
            }
            //with new token
            LWM2M_obs_token_ptr = (uint8_t*)malloc(received_coap_ptr->token_len);
            if(LWM2M_obs_token_ptr){
                memcpy(LWM2M_obs_token_ptr, received_coap_ptr->token_ptr, received_coap_ptr->token_len);
                LWM2M_obs_token_len = received_coap_ptr->token_len;
            }
        }

        if(received_coap_ptr->options_list_ptr->observe) {
            // get observe start/stop value from received GET
            LWM2M_obs_option = * received_coap_ptr->options_list_ptr->observe_ptr;   
            // start or stop based on option value 
            // ref. draft-ietf-core-observe-16          
            if (START_OBS == LWM2M_obs_option){
                coap_res_ptr->options_list_ptr->observe_ptr = &LWM2M_obs_number;
                coap_res_ptr->options_list_ptr->observe_len = sizeof(LWM2M_obs_number);
                LWM2M_start_notification(); 
            }
            else if (STOP_OBS == LWM2M_obs_option){
                LWM2M_stop_notification();
            }
        }
 
        sn_nsdl_send_coap_message(address, coap_res_ptr);
        nsdl_free(coap_res_ptr->options_list_ptr);
        coap_res_ptr->options_list_ptr = NULL;
        coap_res_ptr->content_type_ptr = NULL;// parser_release below tries to free this memory
    }
    
    /* 
    PUT needs to be enabled for the write attributes operation which is empty payload + query options
    */
    else if(COAP_MSG_CODE_REQUEST_PUT == received_coap_ptr->msg_code){
        //pc.printf("PUT: %d bytes\r\n", received_coap_ptr->payload_len);
        if((received_coap_ptr->payload_len > 0) && (received_coap_ptr->payload_len <= 5)){
            memcpy(LWM2M_update_string, (char *)received_coap_ptr->payload_ptr, received_coap_ptr->payload_len);
            LWM2M_update_string[received_coap_ptr->payload_len] = '\0';
            pc.printf("PUT: %s\r\n", LWM2M_update_string);

            sscanf( LWM2M_update_string, "%f3.1", &current_sample); //update for read-back test, observe will clobber

            coap_res_ptr = sn_coap_build_response(received_coap_ptr, COAP_MSG_CODE_RESPONSE_CHANGED);
            sn_nsdl_send_coap_message(address, coap_res_ptr);
        }
        // see if there are query options and scan for write attributes, allow payload and query options
        // PUT without query ffrom web client reads some query string, wireshark it...
        if(received_coap_ptr->options_list_ptr->uri_query_ptr != NULL){
            LWM2M_query_string_ptr = (char*)nsdl_alloc(received_coap_ptr->options_list_ptr->uri_query_len+1);
            if (LWM2M_query_string_ptr){
                memcpy(LWM2M_query_string_ptr, 
                    received_coap_ptr->options_list_ptr->uri_query_ptr, 
                    received_coap_ptr->options_list_ptr->uri_query_len);
                memset(LWM2M_query_string_ptr + received_coap_ptr->options_list_ptr->uri_query_len,'\0',1);//string terminator
                // diagnostic
                // pc.printf("query string received: %s\r\n", LWM2M_query_string_ptr);
                attribute_update = false;
                // extract query options from string
                query_option = strtok(LWM2M_query_string_ptr, "&");// split the string
                num_options = 0;
                while (query_option != NULL){
                    strcpy(query_options[num_options++], query_option);
                    // pc.printf("query part: %s\r\n", query_option);
                    query_option = strtok(NULL, "&");// next query option
                }
                // pc.printf("setting attributes\r\n");
                for (int option = 0; option < num_options; option++)
                    set_notification_attribute(query_options[option]);

                nsdl_free(LWM2M_query_string_ptr);
                // if anything was updated, re-initialize the stored notification attributes
                if (attribute_update){
                    // initializes and sends an update if observing is on, don't change observing state
                    // allows cancel to turn off observing and updte state without sending a notification
                    LWM2M_notification_init(); 
                    coap_res_ptr = sn_coap_build_response(received_coap_ptr, COAP_MSG_CODE_RESPONSE_CHANGED); // 2.04
                }
                else
                    // if query options are sent but no notification attribute names were found, it's an error
                    coap_res_ptr = sn_coap_build_response(received_coap_ptr, COAP_MSG_CODE_RESPONSE_BAD_REQUEST);// 4.00
                    
                sn_nsdl_send_coap_message(address, coap_res_ptr);

            }
            else
                pc.printf("cant alloc query string\r\n"); 
        }
    }

    sn_coap_parser_release_allocated_coap_msg_mem(coap_res_ptr);

    return 0;
}

int create_LWM2M_resource(sn_nsdl_resource_info_s *resource_ptr)
{
    static Thread exec_thread(LWM2M_notification_thread);

    nsdl_create_dynamic_resource(resource_ptr, 
        sizeof(LWM2M_RES_ID)-1, (uint8_t*)LWM2M_RES_ID, 
        sizeof(LWM2M_RES_RT)-1, (uint8_t*)LWM2M_RES_RT, 
        OBS_TRUE, &LWM2M_resource_cb, 
        (SN_GRS_GET_ALLOWED | SN_GRS_PUT_ALLOWED));
    return 0;
}


/* LWM2M Observe attributes for float and int data, for string and bool only implement pmin and pmax
This implements the LWM2M 1.0 Notification attributes based on the following interpretation:

1. lt and gt define three signal bands or states, for example low alarm, normal, and high alarm
it is desired to transmit a notification whenever the measured variable is in a different band from 
the last reported band, that is from nornal to high alarm, and from high alarm to normal, and all 
other possible state transitions, e.g. high to low, etc. subject to pmin (see below)

2. step defines a minimum change needed in the measured variable relative to the last reported value
to trigger a new report of the value. For example, if the last reported value was 51.0, and the step 
variable was set at 1.0, the signal would need to be greater than or equal to 52.0, or less than or 
equal to 50.0 to trigger a new report (notification). Each time the measured variable is reported, the
upper and lower values required to trigger a new report are updated relative to the reported value.

3. pmin defines the minumum desired reporting interval. pmin is assumed to define a mandatory
quiet period between notifications. a notification here is assumed to contain at least a single value. 
Therefore if a combination of potentially reportable events are triggered by lt, gt, and step during the 
quiet period, the value at the end of the quiet period MUST be reported (sent in a notification). 
Additional values captured during the quiet period MAY be sent along with the current value in a
notification object.

4. pmax defines the largest time interval allowed without a notification. Each time a notification is sent, 
a timer is reset, set to expire at pmax seconds after the notification was sent. If the pmax timer expires,
a notification is sent.

Implementation Notes: Each time a notification is sent, pmin and pmax timers are are restarted. 

The algorithm for lt and gt is generalized to accept from one to n limit values, each defining a boundary 
between n+1 signal bands (states). A transition from any state to any other state will create a reportable 
event.

The simple implementation below does not capture information about state changes that occur during the quiet
period, only that some state transition has occurred, with a reporting of the current value at the end of the
quiet period. Implementations MAY capture state transitions which occur during the quiet period and report them
at the end of the quiet period along with the current value in a notification object (senml+json or 
lwm2m format tlv) 
*/

// data type float - int could be used but just convert/cast
typedef float sample; 

//algorithm can accept any number of limit values and report when signal changes between limit bands
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

/* 
Determine which band [0..num_limits] the provided sample is in
Works with any number of bands 2 to MAX_LIMITS+1 using an array of limit settings
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
trigger the build and sending of coap observe response
sends current value
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
    // notification thread is reading sensor in a polling loop and calling on_update 
    // when the sample value changes 
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
for reporting a sample that satisfies the reporting criteria and resetting the state machine
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
this will send the report immediately if pmin is exceeded or else schedule it for later, 
when the pmin timer expires.
Note that if a reportable event occurs during the pmin "quiet period" and then the sample 
returns to a non-reportable state, the sample will still be reported. This can be used to 
identify signal excursions within the quiet period that aren't reported otherwise.
Implementations MAY queue reportable events to be scheduled as a bulk object notification
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


