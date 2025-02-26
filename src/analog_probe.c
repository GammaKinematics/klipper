
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


#define ANALOG_PROBE_BUFFER_MAX_LENGTH 200

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

//enum probe_flags{INACTIVE, PROBING, LOGGING}

struct analog_probe {
    uint8_t oid;
    struct gpio_adc pin;

    uint8_t trigger_sup, trigger_inf;
    
    double threshold;
    uint8_t auto_threshold;
    double std_multiplier;

    uint8_t tare_buffer_length, current_buffer_length, used_buffer_length, n_samples;
    uint16_t buffer[ANALOG_PROBE_BUFFER_MAX_LENGTH];

    uint16_t raw_value;
    double current_value;
    double tare;
     
    struct timer time;
    uint32_t rest_time, sample_time, nextwake;
    struct trsync *ts;
    uint8_t target, sample_count, trigger_count, trigger_reason;

    uint8_t logging;
    uint32_t log_time;
};


void
update_buffer(struct analog_probe *pr) {
    for (int i = (pr->n_samples-1); i > 0; i--) {
        pr->buffer[i] = pr->buffer[i-1];
    }
    pr->buffer[0] = pr->raw_value;
    if (pr->n_samples < pr->used_buffer_length) {
        pr->n_samples++;
    }
    pr->current_value = 0.0;
    if (pr->n_samples >= pr->current_buffer_length) {
        for (int i = 0; i < pr->current_buffer_length; i++) {
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
tare(struct analog_probe *pr) {
    if (pr->n_samples >= pr->tare_buffer_length) {
        pr->tare = 0.0;
        for (int i = 0; i < pr->tare_buffer_length; i++) {
            pr->tare += pr->buffer[i];
        }
        pr->tare /= pr->tare_buffer_length;
        if (pr->auto_threshold) {
            pr->threshold = 0.0;
            for (int i = 0; i < pr->tare_buffer_length; i++) {
                pr->threshold += (pr->buffer[i]-pr->tare)*(pr->buffer[i]-pr->tare);
            }
            pr->threshold = (pr->std_multiplier*sqroot(pr->threshold/pr->tare_buffer_length))/pr->tare;
        }
    }
}


// Timer callback for the analog probe
static uint_fast8_t
analog_probe_event(struct timer *t)
{
    struct analog_probe *probe = container_of(t, struct analog_probe, time);
    
    // Check if the ADC is ready to spit a new value
    uint32_t sample_delay = gpio_adc_sample(probe->pin);
    if (sample_delay) {
        if (sample_delay > probe->rest_time) {
            probe->time.waketime += sample_delay;
        } else {
            probe->time.waketime += probe->rest_time;
        }
        probe->nextwake = probe->time.waketime;
        return SF_RESCHEDULE;
    }

    // Get the new ADC value and update the buffer with it
    probe->raw_value = gpio_adc_read(probe->pin);
    update_buffer(probe);

    // Send logging of the current probe infos if asked
    if (probe->logging) {
        irq_disable();
        uint8_t oid = probe->oid;
        uint32_t timestamp = probe->time.waketime;
        uint16_t raw = probe->raw_value;
        double cur = probe->current_value;
        double tar = probe->tare;
        double thresh = probe->threshold;
        uint8_t auto_thresh = probe->auto_threshold;
        double std_mul = probe->std_multiplier;
        uint8_t tare_buf = probe->tare_buffer_length;
        uint8_t cur_buf = probe->current_buffer_length;
        uint8_t trig = is_triggered(probe);
        uint8_t end = 0;
        if (probe->log_time) {
            end = probe->time.waketime > probe->log_time;
        }
        irq_enable();
        sendf("analog_probe_logs oid=%c ts=%u raw=%u cur=%u tare=%u thresh=%u auto_th=%u std_mul=%u tare_buf=%u cur_buf=%u trig=%u finished=%u",
            oid, timestamp, raw, (int)(cur*1000), (int)(tar*1000), (int)(thresh*1000), auto_thresh, (int)(std_mul*100), tare_buf, cur_buf, trig, end);
        if (end) {
            probe->logging = 1;
            probe->log_time = 0;
            if (!probe->sample_count) {
                sched_del_timer(&probe->time);
                gpio_adc_cancel_sample(probe->pin);
                sendf("analog_probe_active oid=%c active=%u", probe->oid, 0);
                return SF_DONE;
            }
        }
    }
    
    // Check if the probe is triggered and stop the movement if so
    if (probe->sample_count && (probe->tare > 0) && (probe->n_samples >= probe->current_buffer_length)) {
        if (!(is_triggered(probe) && probe->target)) {
            // No match - reschedule for the next attempt
            if (probe->trigger_count < probe->sample_count) {
                probe->time.waketime = probe->nextwake;
            } else {
                probe->time.waketime += probe->rest_time;
                probe->nextwake = probe->time.waketime;
            }
            probe->trigger_count = probe->sample_count;
            return SF_RESCHEDULE;
        }

        if (probe->trigger_count == probe->sample_count) {
            probe->nextwake = probe->time.waketime + probe->rest_time;
        }

        if (!(probe->trigger_count - 1)) {
            probe->sample_count = 0;
            trsync_do_trigger(probe->ts, probe->trigger_reason);
            sendf("analog_probe_active oid=%c active=%u", probe->oid, 0);
            return SF_DONE;
        }
        probe->trigger_count--;
        probe->time.waketime += probe->sample_time;
        return SF_RESCHEDULE;
    }

    // Default rescheduling
    probe->time.waketime += probe->rest_time;
    return SF_RESCHEDULE;
}


void
command_config_analog_probe(uint32_t *args)
{
    struct analog_probe *probe = oid_alloc(args[0], command_config_analog_probe, sizeof(*probe));
    
    probe->oid = args[0];

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
    probe->used_buffer_length = (probe->tare_buffer_length > probe->current_buffer_length) ? probe->tare_buffer_length : probe->current_buffer_length;
    probe->n_samples = 0;
    probe->tare = 0.0;
    probe->current_value = 0.0;
    probe->raw_value = 0;

    probe->sample_count = 0;
    probe->log_time = 0;
}
DECL_COMMAND(command_config_analog_probe, "config_analog_probe oid=%c pin=%c" 
                                          " trig_sup=%u trig_inf=%u trig_th=%u"
                                          " auto_th=%u auto_std_mul=%u"
                                          " tare_buf_len=%u cur_buf_len=%u");

void
command_analog_probe_init(uint32_t *args)
{
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    sched_del_timer(&probe->time);
    gpio_adc_cancel_sample(probe->pin);
    probe->time.waketime = args[1];
    probe->rest_time = args[2];
    probe->time.func = analog_probe_event;
    probe->n_samples = 0;
    sched_add_timer(&probe->time);
    sendf("analog_probe_active oid=%c active=%u", probe->oid, 1);
}
DECL_COMMAND(command_analog_probe_init,
             "analog_probe_init oid=%c clock=%u rest_ticks=%u");

void
command_analog_probe_start_log(uint32_t *args)
{
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    probe->logging = 1;
    if (args[1]) {
        probe->log_time = probe->time.waketime + args[1];
    } else {
        probe->log_time = 0;
    }
}
DECL_COMMAND(command_analog_probe_start_log,
             "analog_probe_start_log oid=%c log_ticks=%u");

void
command_analog_probe_stop_log(uint32_t *args)
{
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    probe->logging = 0;
    probe->log_time = 0;
    if (!probe->sample_count) {
        sched_del_timer(&probe->time);
        gpio_adc_cancel_sample(probe->pin);
        sendf("analog_probe_active oid=%c active=%u", probe->oid, 0);
    }
}
DECL_COMMAND(command_analog_probe_stop_log,
             "analog_probe_stop_log oid=%c");

void
command_analog_probe_home(uint32_t *args)
{
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    sched_del_timer(&probe->time);
    gpio_adc_cancel_sample(probe->pin);
    probe->time.waketime = args[1];
    probe->sample_time = args[2];
    probe->sample_count = args[3];
    if (!probe->sample_count) {
        // Disable end stop checking
        probe->ts = NULL;
        probe->target = 0;
        return;
    }
    probe->rest_time = args[4];
    probe->time.func = analog_probe_event;
    probe->target = args[5];
    probe->ts = trsync_oid_lookup(args[6]);
    probe->trigger_reason = args[7];
    probe->n_samples = 0;
    sched_add_timer(&probe->time);
    sendf("analog_probe_active oid=%c active=%u", probe->oid, 1);
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

    sendf("endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
          oid, targ, nextwake, trig);
}
DECL_COMMAND(command_analog_probe_query_state, "analog_probe_query_state oid=%c");

// void 
// command_update_buffer(uint32_t *args) {
//     struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
//     probe->tare_buffer_length = args[1];
//     probe->current_buffer_length = args[2];
//     probe->used_buffer_length = (probe->tare_buffer_length > probe->current_buffer_length) ? probe->tare_buffer_length : probe->current_buffer_length;
//     probe->n_samples = 0;
// }
// DECL_COMMAND(command_update_buffer, "analog_probe_update_buffer oid=%c tare_buf_len=%u cur_buf_len=%u");

void 
command_do_tare(uint32_t *args) {
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    tare(probe);
    irq_disable();
    double tar = probe->tare;
    double thresh = probe->threshold;
    uint8_t auto_thresh = probe->auto_threshold;
    double std_mul = probe->std_multiplier;
    irq_enable();
    sendf("analog_probe_tare oid=%c tare=%u thresh=%u auto_th=%u std_mul=%u",
          args[0], (int)(tar*1000), (int)(thresh*1000), auto_thresh, (int)(std_mul*100));
}
DECL_COMMAND(command_do_tare, "analog_probe_do_tare oid=%c");

void 
command_set_threshold(uint32_t *args){
    struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    if (!args[2]) {
        probe->threshold = (double)args[1]/1000;
        probe->auto_threshold = 0;
    } else {
        probe->auto_threshold = 1;
        probe->std_multiplier = (double)args[3]/100;
    }
}
DECL_COMMAND(command_set_threshold, "analog_probe_set_thresh oid=%c trig_th=%u auto_th=%u auto_std_mul=%u");

// void 
// command_report(uint32_t *args) {
//     struct analog_probe *probe = oid_lookup(args[0], command_config_analog_probe);
    
//     irq_disable();
//     uint16_t raw = probe->raw_value;
//     double cur = probe->current_value;
//     double tar = probe->tare;
//     double thresh = probe->threshold;
//     uint8_t auto_thresh = probe->auto_threshold;
//     double std_mul = probe->std_multiplier;
//     uint8_t tare_buf = probe->tare_buffer_length;
//     uint8_t cur_buf = probe->current_buffer_length;
//     irq_enable();

//     sendf("analog_probe_report oid=%c raw=%u cur=%u tare=%u thresh=%u auto_th=%u std_mul=%u tare_buf=%u cur_buf=%u",
//           args[0], raw, (int)(cur*1000), (int)(tar*1000), (int)(thresh*1000), auto_thresh, (int)(std_mul*100), tare_buf, cur_buf);
// }
// DECL_COMMAND(command_report, "analog_probe_query_report oid=%c");