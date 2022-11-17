
// Handling of an analog probe.
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // struct gpio
#include "board/irq.h" // irq_disable
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "trsync.h" // trsync_do_trigger
#include <math.h>

#define ANALOG_PROBE_BUFFER_LENGTH 200

#define TOL 0.000001

double sqroot(double square)
{
    double root=square/3, last, diff=1;
    if (square <= 0) return 0;
    do {
        last = root;
        root = (root + square / root) / 2;
        diff = root - last;
    } while (diff > TOL || diff < -TOL);
    return root;
}

struct analog_probe {
    struct gpio_adc pin;

    uint8_t trigger_sup, trigger_inf;
    
    double threshold;
    uint8_t auto_threshold;
    double std_multiplier;

    uint8_t tare_buffer_length, current_buffer_length, buffer_length, buffer_index;
    uint16_t buffer[ANALOG_PROBE_BUFFER_LENGTH];

    uint16_t raw_value;
    double current_value;
    double tare;

    struct timer time;
    uint32_t rest_time, sample_time, nextwake;
    struct trsync *ts;
    uint8_t target, state, sample_count, trigger_count, trigger_reason;
};

void
update_buffer(struct analog_probe *pr) {
    if (pr->buffer_index < (pr->buffer_length-1)) {
        pr->buffer[pr->buffer_index] = pr->raw_value;
        pr->buffer_index++;
    } else {
        for (int i = 0; i < pr->buffer_index; i++) {
            pr->buffer[i] = pr->buffer[i+1];
        }
        pr->buffer[pr->buffer_index] = pr->raw_value;
    }
    pr->current_value = 0.0;
    if (pr->buffer_index >= (pr->current_buffer_length-1)) {
        for (int i = pr->buffer_index-pr->current_buffer_length+1; i <= pr->buffer_index; i++) {
            pr->current_value += pr->buffer[i];
        }
        pr->current_value /= pr->current_buffer_length;
    }
}

uint8_t 
is_triggered(struct analog_probe *pr){
    uint8_t inf_trig, sup_trig;
    if (pr->trigger_inf) {
        inf_trig = pr->current_value < ((1-pr->threshold)*pr->tare);
    } else {
        inf_trig = 0;
    }
    if (pr->trigger_sup) {
        sup_trig = pr->current_value > ((1+pr->threshold)*pr->tare);
    } else {
        sup_trig = 0;
    }
    return inf_trig || sup_trig;
}

void 
update_adc_sensor(struct analog_probe *pr)
{
    uint32_t sample_delay = gpio_adc_sample(pr->pin);
    if (sample_delay) {
        pr->time.waketime += sample_delay;
        return;
    }
    uint16_t value = gpio_adc_read(pr->pin);
    uint8_t state = pr->state;
    if (state >= pr->sample_count) {
        state = 0;
    } else {
        value += pr->raw_value;
    }
    pr->raw_value = value;
    pr->state = state+1;
    if (pr->state < pr->sample_count) {
        pr->time.waketime += pr->sample_time;
        return;
    }
    pr->nextwake += pr->rest_time;
    pr->time.waketime = pr->nextwake;
    return;
}

// Timer callback for the analog probe
static uint_fast8_t
analog_probe_event(struct timer *t)
{
    struct analog_probe *probe = container_of(t, struct analog_probe, time);
    // update_adc_sensor(probe);
    update_buffer(probe);
    // if (is_triggered(probe) && probe->target) {
    //     trsync_do_trigger(probe->ts, probe->trigger_reason);
    //     return SF_DONE;
    // }
    return SF_RESCHEDULE;
}

void
command_config_analog_probe(uint32_t *args)
{
    struct analog_probe *probe = oid_alloc(args[0], command_config_analog_probe, sizeof(*probe));
    
    probe->pin = gpio_adc_setup(args[1]);
    
    probe->trigger_sup = args[2];
    probe->trigger_inf = args[3];

    probe->threshold = (float)args[4]/10;
    probe->auto_threshold = args[5];
    probe->std_multiplier = (float)args[6]/100;

    if (args[7] < sizeof(probe->buffer)/sizeof(uint16_t)) {
        probe->tare_buffer_length = args[7];
    } else {
        probe->tare_buffer_length = sizeof(probe->buffer)/sizeof(uint16_t);
    }
    if (args[8] < sizeof(probe->buffer)/sizeof(uint16_t)) {
        probe->current_buffer_length = args[8];
    } else {
        probe->current_buffer_length = sizeof(probe->buffer)/sizeof(uint16_t);
    }
    probe->buffer_length = sizeof(probe->buffer)/sizeof(uint16_t);
    probe->buffer_index = 0;
    probe->tare = 0.0;
    probe->current_value = 0.0;
    probe->raw_value = 0;
}
DECL_COMMAND(command_config_analog_probe, "config_analog_probe oid=%c pin=%c" 
                                          " trig_sup=%u trig_inf=%u trig_th=%u"
                                          " auto_th=%u auto_std_mul=%u"
                                          " tare_buf_len=%u cur_buf_len=%u");

void
command_analog_probe_home(uint32_t *args)
{
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    sched_del_timer(&probe->time);
    gpio_adc_cancel_sample(probe->pin);
    probe->nextwake = args[1];
    probe->time.waketime = probe->nextwake;
    probe->sample_time = args[2];
    probe->sample_count = args[3];
    if (!probe->sample_count) {
        // Disable end stop checking
        probe->ts = NULL;
        probe->target = 0;
        return;
    }
    probe->state = probe->sample_count + 1;
    probe->rest_time = args[4];
    probe->time.func = analog_probe_event;
    //e->trigger_count = e->sample_count;
    probe->target = args[5];
    probe->ts = trsync_oid_lookup(args[6]);
    probe->trigger_reason = args[7];
    sched_add_timer(&probe->time);
}
DECL_COMMAND(command_analog_probe_home,
             "analog_probe_home oid=%c clock=%u sample_ticks=%u sample_count=%c"
             " rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c");

void
command_analog_probe_query_state(uint32_t *args)
{
    uint8_t oid = args[0];
    struct analog_probe *probe = oid_lookup(oid, command_config_analog_probe);

    irq_disable();
    uint8_t targ = probe->target;
    uint32_t nextwake = probe->nextwake;
    uint8_t trig = is_triggered(probe);
    irq_enable();

    sendf("endstop_state oid=%c homing=%c next_clock=%u pin_value=%c"
          , oid, targ, nextwake, trig);
}
DECL_COMMAND(command_analog_probe_query_state, "analog_probe_query_state oid=%c");

void 
command_update_buffer(uint32_t *args) {
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    probe->tare_buffer_length = args[1];
    probe->current_buffer_length = args[2];
    probe->buffer_index = 0;
}
DECL_COMMAND(command_update_buffer, "analog_probe_update_buffer oid=%c tare_buf_len=%u cur_buf_len=%u");

void 
command_do_tare(uint32_t *args) {
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    probe->tare = 0.0;
    for (int i = probe->buffer_index-probe->tare_buffer_length+1; i <= probe->buffer_index; i++) {
        probe->tare += probe->buffer[i];
    }
    probe->tare /= probe->tare_buffer_length;
    if (probe->auto_threshold) {
        probe->threshold = 0.0;
        for (int i = probe->buffer_index-probe->tare_buffer_length+1; i <= probe->buffer_index; i++) {
            probe->threshold += (probe->buffer[i]-probe->tare)*(probe->buffer[i]-probe->tare);
        }
        probe->threshold = (probe->std_multiplier*sqroot(probe->threshold/probe->tare_buffer_length))/probe->tare;
    }
}
DECL_COMMAND(command_do_tare, "analog_probe_do_tare oid=%c");

void 
command_set_threshold(uint32_t *args){
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    if (!args[2]) {
        probe->threshold = (double)args[1]/10;
        probe->auto_threshold = 0;
    } else {
        probe->auto_threshold = 1;
        probe->std_multiplier = (double)args[3]/100;
    }
}
DECL_COMMAND(command_set_threshold, "analog_probe_set_thresh oid=%c trig_th=%u auto_th=%u auto_std_mul=%u");

void 
command_report(uint32_t *args) {
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    
    irq_disable();
    uint16_t raw = probe->raw_value;
    double cur = probe->current_value;
    double tar = probe->tare;
    double thresh = probe->threshold;
    uint8_t auto_thresh = probe->auto_threshold;
    double std_mul = probe->std_multiplier;
    uint8_t tare_buf = probe->tare_buffer_length;
    uint8_t cur_buf = probe->current_buffer_length;
    irq_enable();

    sendf("analog_probe_report oid=%c raw=%u cur=%u tare=%u thresh=%u auto_th=%u std_mul=%u tare_buf=%u cur_buf=%u"
          , args[0], raw, (int)cur*1000, (int)tar*1000, (int)thresh*1000, auto_thresh, (int)std_mul*100, tare_buf, cur_buf);
}
DECL_COMMAND(command_report, "analog_probe_query_report oid=%c");