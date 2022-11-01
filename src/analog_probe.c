
// Handling of an analog probe.
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "adccmds.c"
#include "board/gpio.h" // struct gpio
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "trsync.h" // trsync_do_trigger
#include <math.h>

#define ANALOG_PROBE_BUFFER_LENGTH 200

// struct analog_in {
//     struct timer timer;
//     uint32_t rest_time, sample_time, next_begin_time;
//     uint16_t value, min_value, max_value;
//     struct gpio_adc pin;
//     uint8_t invalid_count, range_check_count;
//     uint8_t state, sample_count;
// };
// static uint_fast8_t analog_in_event(struct timer *timer);

struct analog_probe {
    struct analog_in adc_sensor;

    uint8_t trigger_sup, trigger_inf;
    
    float threshold;
    uint8_t auto_threshold;
    float std_multiplier;

    uint8_t tare_buffer_length, current_buffer_length, buffer_length, buffer_index;
    uint16_t buffer[ANALOG_PROBE_BUFFER_LENGTH];

    float current_value;
    float tare;

    //struct timer time;
    struct trsync *ts;
    uint8_t target, trigger_count, trigger_reason;
};

void
update_buffer(struct analog_probe *e, const int16_t new_value) {
    if (e->buffer_index < (e->buffer_length-1)) {
        e->buffer[e->buffer_index] = new_value;
        e->buffer_index++;
    } else {
        for (int i = 0; i < e->buffer_index; i++) {
            e->buffer[i] = e->buffer[i+1];
        }
        e->buffer[e->buffer_index] = new_value;
    }
    e->current_value = 0.0;
    if (e->buffer_index >= (e->current_buffer_length-1)) {
        for (int i = e->buffer_index-e->current_buffer_length+1; i <= e->buffer_index; i++) {
            e->current_value += e->buffer[i];
        }
        e->current_value /= e->current_buffer_length;
    }
}

uint8_t 
is_triggered(struct analog_probe *e){
    uint8_t inf_trig, sup_trig;
    if (e->trigger_inf) {
        inf_trig = e->current_value < ((1-e->threshold)*e->tare);
    } else {
        inf_trig = 0;
    }
    if (e->trigger_sup) {
        sup_trig = e->current_value > ((1+e->threshold)*e->tare);
    } else {
        sup_trig = 0;
    }
    return inf_trig || sup_trig;
}

void 
update_adc_sensor(struct analog_in *a)
{
    uint32_t sample_delay = gpio_adc_sample(a->pin);
    if (sample_delay) {
        a->timer.waketime += sample_delay;
        return;
    }
    uint16_t value = gpio_adc_read(a->pin);
    uint8_t state = a->state;
    if (state >= a->sample_count) {
        state = 0;
    } else {
        value += a->value;
    }
    a->value = value;
    a->state = state+1;
    if (a->state < a->sample_count) {
        a->timer.waketime += a->sample_time;
        return;
    }
    if (likely(a->value >= a->min_value && a->value <= a->max_value)) {
        a->invalid_count = 0;
    } else {
        a->invalid_count++;
        if (a->invalid_count >= a->range_check_count) {
            try_shutdown("ADC out of range");
            a->invalid_count = 0;
        }
    }
    a->next_begin_time += a->rest_time;
    a->timer.waketime = a->next_begin_time;
    return;
}

// Timer callback for the analog probe
static uint_fast8_t
analog_probe_event(struct timer *t)
{
    //analog_in_event(t);
    struct analog_in *a = container_of(t, struct analog_in, timer);
    update_adc_sensor(a);

    struct analog_probe *e = container_of(a, struct analog_probe, adc_sensor);
    update_buffer(e, e->adc_sensor.value);
    if (is_triggered(e) && e->target) {
        trsync_do_trigger(e->ts, e->trigger_reason);
        return SF_DONE;
    }
    return SF_RESCHEDULE;
}

void
command_config_analog_probe(uint32_t *args)
{
    struct analog_probe *e = oid_alloc(args[0], command_config_analog_probe, sizeof(*e));
    
    e->adc_sensor.pin = gpio_adc_setup(args[1]);
    
    e->trigger_sup = args[3];
    e->trigger_inf = args[4];

    e->threshold = args[5];
    e->auto_threshold = args[6];
    e->std_multiplier = args[7];

    if (args[7] < sizeof(e->buffer)/sizeof(uint16_t)) {
        e->tare_buffer_length = args[8];
    } else {
        e->tare_buffer_length = sizeof(e->buffer)/sizeof(uint16_t);
    }
    if (args[8] < sizeof(e->buffer)/sizeof(uint16_t)) {
        e->current_buffer_length = args[9];
    } else {
        e->current_buffer_length = sizeof(e->buffer)/sizeof(uint16_t);
    }
    e->buffer_length = sizeof(e->buffer)/sizeof(uint16_t);
    e->buffer_index = 0;
    e->tare = 0.0;
    e->current_value = 0.0;
}
DECL_COMMAND(command_config_analog_probe, "config_analog_probe oid=%c pin=%c pull_up=%c" 
                                          " trig_sup=%c trig_inf=%c trig_th=%f"
                                          " auto_th=%c auto_std_mul=%f"
                                          " tare_buf_len=%u cur_buf_len=%u");

void
command_analog_probe_home(uint32_t *args)
{
    struct analog_probe *e = oid_lookup(args[0], command_config_analog_probe);
    sched_del_timer(&e->adc_sensor.timer);
    gpio_adc_cancel_sample(e->adc_sensor.pin);
    e->adc_sensor.next_begin_time = args[1];
    e->adc_sensor.timer.waketime = e->adc_sensor.next_begin_time;
    e->adc_sensor.sample_time = args[2];
    e->adc_sensor.sample_count = args[3];
    if (!e->adc_sensor.sample_count) {
        // Disable end stop checking
        e->ts = NULL;
        e->target = 0;
        return;
    }
    e->adc_sensor.state = e->adc_sensor.sample_count + 1;
    e->adc_sensor.rest_time = args[4];
    e->adc_sensor.min_value = 0;
    e->adc_sensor.max_value = 65535;
    e->adc_sensor.range_check_count = 200;
    e->adc_sensor.timer.func = analog_probe_event;
    //e->trigger_count = e->sample_count;
    e->target = args[5];
    e->ts = trsync_oid_lookup(args[6]);
    e->trigger_reason = args[7];
    sched_add_timer(&e->adc_sensor.timer);
}
DECL_COMMAND(command_analog_probe_home,
             "analog_probe_home oid=%c clock=%u sample_ticks=%u sample_count=%c"
             " rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c");

void
command_analog_probe_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct analog_probe *e = oid_lookup(oid, command_config_analog_probe);

    irq_disable();
    uint8_t targ = e->target;
    uint32_t nextwake = e->adc_sensor.next_begin_time;
    uint8_t trig = is_triggered(e);
    irq_enable();

    sendf("endstop_state oid=%c homing=%c next_clock=%u pin_value=%c"
          , oid, targ, nextwake, trig);
}
DECL_COMMAND(command_analog_probe_query_state, "analog_probe_query_state oid=%c");

void 
command_update_buffer(uint32_t *args) {
    struct analog_probe *e = oid_lookup(args[0], command_config_analog_probe);
    e->tare_buffer_length = args[1];
    e->current_buffer_length = args[2];
    e->buffer_index = 0;
}
DECL_COMMAND(command_update_buffer, "analog_probe_update_buffer oid=%c tare_buf_len=%u cur_buf_len=%u");

void 
command_do_tare(uint32_t *args) {
    struct analog_probe *e = oid_lookup(args[0], command_config_analog_probe);
    e->tare = 0.0;
    for (int i = e->buffer_index-e->tare_buffer_length+1; i <= e->buffer_index; i++) {
        e->tare += e->buffer[i];
    }
    e->tare /= e->tare_buffer_length;
    if (e->auto_threshold) {
        e->threshold = 0.0;
        for (int i = e->buffer_index-e->tare_buffer_length+1; i <= e->buffer_index; i++) {
            e->threshold += pow((e->buffer[i]-e->tare), 2);
        }
        e->threshold = (e->std_multiplier*sqrt(e->threshold/e->tare_buffer_length))/e->tare;
    }
}
DECL_COMMAND(command_do_tare, "analog_probe_do_tare oid=%c");

void 
command_set_threshold(uint32_t *args){
    struct analog_probe *e = oid_lookup(args[0], command_config_analog_probe);
    if (!args[2]) {
        e->threshold = args[1]/1000;
        e->auto_threshold = 0;
    } else {
        e->auto_threshold = 1;
        e->std_multiplier = args[3];
    }
}
DECL_COMMAND(command_set_threshold, "analog_probe_set_thresh oid=%c trig_th=%f auto_th=%c auto_std_mul=%f");
