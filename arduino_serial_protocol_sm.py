from xworkflows import Workflow, WorkflowEnabled
import xworkflows
import logging as log
import array


class _Side:

    secondary = False
    primary = True


class _ProtocolWorkflow(Workflow):

    states = (
        ('UNDEFINED',        'Undefined state, protocol handler encountered unrecoverable error'),
        ('WAITING_SYNC',     'Waiting for SYNC request packet, initial state for secondary'),
        ('SENDING_SYNC',     'Sending SYNC request packets, initial state for primary'),
        ('IDLE',             'Idle state, waiting to receive packet'),
        ('SCAN_STROBE',      'Scan strobe state, usually happen on error in header (CRC)'),
        ('WRITE_SYNC_REPLY', 'Waiting for SYNC response packet to be send'),
        ('READ_SYNC_REST',   'Waiting for rest of SYNC to arrive'),
        ('READ_SYNC_R_REST', 'Waiting for rest of SYNC reply to arrive'),
        ('READ_HEADER',      'Waiting for rest of header to arrive'),
        ('READ_PAYLOAD',     'Waiting for payload to arrive'),
    )

    transitions = (
        ('sync_strobe_read',    'WAITING_SYNC',     'READ_SYNC_REST'),
        ('sync_rest_read',      'READ_SYNC_REST',   'WRITE_SYNC_REPLY'),
        ('unexp_sync_rest_w',   'READ_SYNC_REST',   'WAITING_SYNC'),
        ('unexp_sync_rest_i',   'READ_SYNC_REST',   'IDLE'),
        ('sync_reply_sent',     'WRITE_SYNC_REPLY', 'IDLE'),

        ('sync_r_strobe_read',  'SENDING_SYNC',     'READ_SYNC_R_REST'),
        ('sync_r_rest_read',    'READ_SYNC_R_REST', 'IDLE'),
        ('unexp_sync_r_rest',   'READ_SYNC_R_REST', 'SENDING_SYNC'),

        ('strobe_read',         'IDLE',             'READ_HEADER'),
        ('strobe_read_scan',    'SCAN_STROBE',      'READ_HEADER'),
        ('header_read',         'READ_HEADER',      'READ_PAYLOAD'),
        ('header_error',        'READ_HEADER',      'SCAN_STROBE'),
        ('payload_read',        'READ_PAYLOAD',     'IDLE'),
        ('payload_error',       'READ_PAYLOAD',     'IDLE'),

        ('be_primary',          'UNDEFINED',        'SENDING_SYNC'),
        ('be_secondary',        'UNDEFINED',        'WAITING_SYNC'),
    )

    initial_state = 'UNDEFINED'

    def log_transition(_, transition, from_state, *args, **kwargs):
        log.debug('ArduinoSerialProtocolSM transition <%s> from <%s> to <%s>',
                  str(transition.name),
                  str(from_state),
                  str(transition.target))


class _ProtocolStateMPayloadState:

    def __init__(self):
        self.payload_len = 0
        self.packet_id = 0
        self.crc16 = 0
        self.crc16_header = None


class _ProtocolStateM(WorkflowEnabled):

    _state = _ProtocolWorkflow()

    def __init__(self, side):
        assert isinstance(side, bool), 'Argument "side" must be Side.secondary or Side.primary'
        self._is_primary = side
        self._was_synced = False
        self._seq_id = 0
        self._payload_state = _ProtocolStateMPayloadState()
        self._buffer = array.array('B', [])
        if self._is_primary:
            self._init_primary()
        else:
            self._init_secondary()

    def read_bytes(self, byte_array):
        self._buffer.extend(byte_array)
        return self._process_bytes()

    def timer_fired(self, timer_data):
        return ''

    def create_sync_packet(self):
        return ''

    def create_packet(self, payload):
        return ''

    def sync_reply_sent(self):
        return ''

    @xworkflows.transition('be_primary')
    def _init_primary(self):
        pass

    @xworkflows.transition('be_secondary')
    def _init_secondary(self):
        pass

    def _process_bytes(self):
        return ''
