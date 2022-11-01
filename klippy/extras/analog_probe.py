# Analog probe support

from . import probe


class AnalogProbe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self.handle_mcu_identify)
        self.position_endstop = config.getfloat('z_offset', minval=0.)

        # Create an "endstop" object to handle the sensor pin
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        pin_params['is_adc'] = True
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)

        # Parameters
        self.trigger_sup = config.getboolean('trigger_sup', True)
        self.trigger_inf = config.getboolean('trigger_inf', True)
        self.threshold = config.getfloat('trigger_threshold', 0.0)
        
        self.auto_threshold = config.getboolean('auto_threshold', True)
        self.auto_std_multiplier = config.getfloat('auto_std_multiplier', 5.0)

        self.tare_buffer_len = config.getint('tare_buffer_len', 100)
        self.current_buffer_len = config.getint('current_buffer_len', 5)
        self.buffer_len = max(self.tare_buffer_len, self.current_buffer_len)
        self.buffer = []
        self.buffer_index = 0

        self.current_raw_value = 0.0
        self.current_value = 0.0
        self.tare = 0.0
        
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop

        # # Register BLTOUCH_DEBUG command
        # self.gcode = self.printer.lookup_object('gcode')
        # self.gcode.register_command("BLTOUCH_DEBUG", self.cmd_BLTOUCH_DEBUG,
        #                             desc=self.cmd_BLTOUCH_DEBUG_help)
        # self.gcode.register_command("BLTOUCH_STORE", self.cmd_BLTOUCH_STORE,
        #                             desc=self.cmd_BLTOUCH_STORE_help)
        # register gcode commands

        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('UPDATE_THRESHOLD', self.cmd_UPDATE_THRESHOLD,
                                    desc=self.cmd_UPDATE_THRESHOLD_help)

        self.gcode.register_command('UPDATE_BUFFER_LEN', self.cmd_UPDATE_BUFFER_LEN,
                                    desc=self.cmd_UPDATE_BUFFER_LEN_help)

        self.gcode.register_command('PRINT_CURRENT_VALUES',
                                    self.cmd_PRINT_CURRENT_VALUES,
                                    desc=self.cmd_PRINT_CURRENT_VALUES_help)

        self.gcode.register_command('MAKE_TARE',
                                    self.cmd_MAKE_TARE,
                                    desc=self.cmd_MAKE_TARE_help)

    cmd_UPDATE_BUFFER_LEN_help = "Update the lenght of the buffers."
    cmd_MAKE_TARE_help = "Tare the probe."
    cmd_UPDATE_THRESHOLD_help = "Update the threshold of the probe."
    cmd_PRINT_CURRENT_VALUES_help = "Print current probe values."

    def _build_config(self):
        # Setup config
        self.mcu_endstop._mcu.add_config_cmd("config_analog_probe oid=%d pin=%s pull_up=%d" 
                                             " trig_sup=%c trig_inf=%c trig_th=%u"
                                             " auto_th=%c auto_std_mul=%u"
                                             " tare_buf_len=%u cur_buf_len=%u"
                                             % (self.mcu_endstop._oid, self.mcu_endstop._pin, self.mcu_endstop._pullup,
                                                self.trigger_sup, self.trigger_inf, int(self.threshold*10),
                                                self.auto_threshold, int(self.auto_std_multiplier*100),
                                                self.tare_buffer_len, self.current_buffer_len))
        self.mcu_endstop._mcu.add_config_cmd(
            "analog_probe_home oid=%d clock=0 sample_ticks=0 sample_count=0"
            " rest_ticks=0 pin_value=0 trsync_oid=0 trigger_reason=0"
            % (self.mcu_endstop._oid), on_restart=True)
        # Lookup commands
        cmd_queue = self.mcu_endstop._trsyncs[0].get_command_queue()
        self.mcu_endstop._home_cmd = self.mcu_endstop._mcu.lookup_command(
            "analog_probe_home oid=%c clock=%u sample_ticks=%u sample_count=%c"
            " rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c",
            cq=cmd_queue)
        self.mcu_endstop._query_cmd = self.mcu_endstop._mcu.lookup_query_command(
            "analog_probe_query_state oid=%c",
            "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
            oid=self.mcu_endstop._oid, cq=cmd_queue)
        self.mcu_endstop._update_buffer_cmd = self.mcu_endstop._mcu.lookup_command("analog_probe_update_buffer oid=%c tare_buf_len=%u cur_buf_len=%u", cq=cmd_queue)
        self.mcu_endstop._do_tare_cmd = self.mcu_endstop._mcu.lookup_command("analog_probe_do_tare oid=%c", cq=cmd_queue)
        self.mcu_endstop._set_threshold_cmd = self.mcu_endstop._mcu.lookup_command("analog_probe_set_thresh oid=%c trig_th=%u auto_th=%c auto_std_mul=%u", cq=cmd_queue)

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
      self.mcu_endstop._do_tare_cmd.send([self.mcu_endstop._oid])
      return self.mcu_endstop.home_start(print_time, sample_time, sample_count, rest_time, triggered)

    def cmd_UPDATE_BUFFER_LEN(self, gcmd):
        self.tare_buffer_len = gcmd.get_int("TARE", 100)
        self.current_buffer_len = gcmd.get_int("CURRENT", 5)
        self.mcu_endstop._update_buffer_cmd.send([self.mcu_endstop._oid, self.tare_buffer_len, self.current_buffer_len])

    def cmd_MAKE_TARE(self, gcmd):
        self.mcu_endstop._do_tare_cmd.send([self.mcu_endstop._oid])

    def cmd_UPDATE_THRESHOLD(self, gcmd):
        self.auto_threshold = gcmd.get_boolean("AUTO", True)
        if self.auto_threshold:
            self.auto_std_multiplier = gcmd.get_float("STD_MULTIPLIER", 5.0)
        else:
            self.threshold = gcmd.get_float("THRESHOLD", 0.5)
        self.mcu_endstop._set_threshold_cmd.send([self.mcu_endstop._oid, int(self.threshold*10), self.auto_threshold, int(self.auto_std_multiplier*100)])

    def cmd_PRINT_CURRENT_VALUES(self, gcmd):
        gcmd.respond_info("Raw: %.1f, Current: %.1f, Tare: %.1f, Threshold: %.1f" %           \
                          (self.current_raw_value, self.current_value, self.tare, self.threshold))

def load_config(config):
    analog_probe = AnalogProbe(config)
    config.get_printer().add_object('probe', probe.PrinterProbe(config, analog_probe))
    return analog_probe
