# Analog probe support
import logging
from . import probe


class AnalogProbe:
    def __init__(self, config):
        logging.info("CPGK Constructor started")
        self.printer = config.get_printer()
        # self.printer.register_event_handler("klippy:connect",
        #                                     self.handle_connect)
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self.handle_mcu_identify)
        self.position_endstop = config.getfloat('z_offset')
        self.stow_on_each_sample = config.getboolean(
            'deactivate_on_each_sample', True)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')

        # Parameters
        self.trigger_sup = config.getboolean('trigger_sup', True)
        self.trigger_inf = config.getboolean('trigger_inf', True)
        self.threshold = config.getfloat('trigger_threshold', 0.0)
        
        self.auto_threshold = config.getboolean('auto_threshold', True)
        self.auto_std_multiplier = config.getfloat('auto_std_multiplier', 5.0)

        self.tare_buffer_len = config.getint('tare_buffer_len', 100)
        self.current_buffer_len = config.getint('current_buffer_len', 5)

        logging.info("CPGK trig sup: %s", str(int(self.trigger_sup)))
        logging.info("CPGK trig inf: %s", str(int(self.trigger_inf)))
        logging.info("CPGK thresh: %s", str(self.threshold))
        logging.info("CPGK auto thresh: %s", str(int(self.auto_threshold)))
        logging.info("CPGK std mul: %s", str(self.auto_std_multiplier))
        logging.info("CPGK tare buf: %s", str(self.tare_buffer_len))
        logging.info("CPGK cur buf: %s", str(self.current_buffer_len))

        # Create an "endstop" object to handle the sensor pin
        ppins = self.printer.lookup_object('pins')
        pin = config.get('pin')
        pin_params = ppins.lookup_pin(pin, can_invert=True, can_pullup=True)
        mcu = pin_params['chip']
        pin_params['is_adc'] = True
        self.mcu_endstop = mcu.setup_pin('endstop', pin_params)
        mcu.register_config_callback(self.config_callbacks) #self.mcu_endstop._

        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop

        # Register analog commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('UPDATE_THRESHOLD', self.cmd_UPDATE_THRESHOLD,
                                    desc=self.cmd_UPDATE_THRESHOLD_help)

        self.gcode.register_command('UPDATE_BUFFER_LEN', self.cmd_UPDATE_BUFFER_LEN,
                                    desc=self.cmd_UPDATE_BUFFER_LEN_help)

        self.gcode.register_command('MAKE_TARE',
                                    self.cmd_MAKE_TARE,
                                    desc=self.cmd_MAKE_TARE_help)
        
        self.gcode.register_command('QUERY_STATE',
                                    self.cmd_QUERY_STATE,
                                    desc=self.cmd_MAKE_TARE_help)

        self.gcode.register_command('LOGGING_PROBE',
                                    self.cmd_LOGGING_PROBE,
                                    desc=self.cmd_LOGGING_PROBE_help)

        # multi probes state
        self.multi = 'OFF'

        logging.info("CPGK Constructor done")

    cmd_UPDATE_BUFFER_LEN_help = "Update the lenght of the buffers."
    cmd_MAKE_TARE_help = "Tare the probe."
    cmd_UPDATE_THRESHOLD_help = "Update the threshold of the probe."
    cmd_LOGGING_PROBE_help = "Start logging the probe values."

    def handle_mcu_identify(self):
        logging.info("CPGK handle_mcu checkpoint")
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    def config_callbacks(self):
        # Setup config
        self.mcu_endstop._mcu.add_config_cmd("config_analog_probe oid=%d pin=%s" 
                                             " trig_sup=%u trig_inf=%u trig_th=%u"
                                             " auto_th=%u auto_std_mul=%u"
                                             " tare_buf_len=%u cur_buf_len=%u"
                                             % (self.mcu_endstop._oid, self.mcu_endstop._pin,
                                                self.trigger_sup*1, self.trigger_inf*1, int(self.threshold*10),
                                                self.auto_threshold*1, int(self.auto_std_multiplier*100),
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
        self.mcu_endstop._set_threshold_cmd = self.mcu_endstop._mcu.lookup_command("analog_probe_set_thresh oid=%c trig_th=%u auto_th=%u auto_std_mul=%u", cq=cmd_queue)
        self.mcu_endstop._report_cmd = self.mcu_endstop._mcu.lookup_query_command("analog_probe_query_report oid=%c", 
                                                                                  "analog_probe_report oid=%c raw=%u cur=%u tare=%u thresh=%u auto_th=%u std_mul=%u tare_buf=%u cur_buf=%u",
                                                                                  oid=self.mcu_endstop._oid, cq=cmd_queue)
        self.mcu_endstop._logging_cmd = self.mcu_endstop._mcu.lookup_command("analog_probe_init oid=%c clock=%u rest_ticks=%u pin_value=%c", cq=cmd_queue)
        self.mcu_endstop._mcu.register_response(self._handle_logging,
                                                "analog_probe_log", self.mcu_endstop._oid)
        logging.info("CPGK build_config checkpoint")

    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
      self.mcu_endstop._do_tare_cmd.send([self.mcu_endstop._oid])
      return self.mcu_endstop.home_start(print_time, sample_time, sample_count, rest_time, triggered)

    def raise_probe(self): #modifs
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script")

    def lower_probe(self): #modifs
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")

    def multi_probe_begin(self):
        if self.stow_on_each_sample:
            return
        self.multi = 'FIRST'

    def multi_probe_end(self):
        if self.stow_on_each_sample:
            return
        self.raise_probe()
        self.multi = 'OFF'

    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'

    def probe_finish(self, hmove):
        if self.multi == 'OFF':
            self.raise_probe()

    def get_position_endstop(self):
        return self.position_endstop

    def cmd_UPDATE_BUFFER_LEN(self, gcmd):
        self.tare_buffer_len = gcmd.get_int("TARE", 100)
        self.current_buffer_len = gcmd.get_int("CURRENT", 5)
        self.mcu_endstop._update_buffer_cmd.send([self.mcu_endstop._oid, self.tare_buffer_len, self.current_buffer_len])

    def cmd_MAKE_TARE(self, gcmd):
        self.mcu_endstop._do_tare_cmd.send([self.mcu_endstop._oid])

    def cmd_UPDATE_THRESHOLD(self, gcmd):
        self.auto_threshold = bool(gcmd.get_int("AUTO", True))
        if self.auto_threshold:
            self.auto_std_multiplier = gcmd.get_float("STD_MULTIPLIER", 5.0)
        else:
            self.threshold = gcmd.get_float("THRESHOLD", 0.5)
        self.mcu_endstop._set_threshold_cmd.send([self.mcu_endstop._oid, int(self.threshold*10), self.auto_threshold*1, int(self.auto_std_multiplier*100)])

    def cmd_QUERY_STATE(self, gcmd):
        params = self.mcu_endstop._report_cmd.send([self.mcu_endstop._oid])
        
        self.threshold = float(params['thresh'])/1000
        self.auto_threshold = bool(params['auto_th'])
        self.auto_std_multiplier = float(params['std_mul'])/100
        self.tare_buffer_len = params['tare_buf']
        self.current_buffer_len = params['cur_buf']
        
        gcmd.respond_info("Raw: %i, Cur: %f, Tare: %f, Thresh: %f" % (params['raw'], 
                                                                      float(params['cur'])/1000,
                                                                      float(params['tare'])/1000,
                                                                      self.threshold))
        gcmd.respond_info("Auto: %i, Std mul: %f, Tare buf: %i, Cur buf: %i" % (self.auto_threshold*1, 
                                                                                self.auto_std_multiplier,
                                                                                self.tare_buffer_len,
                                                                                self.current_buffer_len))

    def cmd_LOGGING_PROBE(self, gcmd):
        rest_time = gcmd.get_float("TS", 0.000015)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        clock = self.mcu_endstop._mcu.print_time_to_clock(print_time)
        rest_ticks = self.mcu_endstop._mcu.print_time_to_clock(print_time+rest_time) - clock

        self.mcu_endstop._logging_cmd.send([self.mcu_endstop._oid, clock, rest_ticks, 1])

    def _handle_logging(self, params):
        ts = float(params['ts'])
        raw = float(params['raw'])
        cur = float(params['cur'])/1000
        tare = float(params['tare'])/1000
        threshold = float(params['thresh'])/1000
        auto_threshold = bool(params['auto_th'])
        auto_std_multiplier = float(params['std_mul'])/100
        tare_buffer_len = params['tare_buf']
        current_buffer_len = params['cur_buf']
        trig = float(params['trig'])
        
        #logging.info("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f" % (ts, raw, cur, tare, threshold, auto_threshold, auto_std_multiplier, tare_buffer_len, current_buffer_len, trig))


def load_config(config):
    analog_probe = AnalogProbe(config)
    config.get_printer().add_object('probe', probe.PrinterProbe(config, analog_probe))
    return analog_probe
