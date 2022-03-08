##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2010-2016 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

# TODO: Look into arbitration, collision detection, clock synchronisation, etc.
# TODO: Implement support for inverting sdata/SCL levels (0->1 and 1->0).
# TODO: Implement support for detecting various bus errors.

import sigrokdecode as srd

'''
OUTPUT_PYTHON format:

Packet:
[<ptype>, <pdata>]

<ptype>:
 - 'START' (START condition)
 - 'START REPEAT' (Repeated START condition)
 - 'CMD FRAME' (Command frame)
 - 'DATA FRAME' (Data frame)
 - 'STOP' (STOP condition)
 - 'ACK' (ACK bit)
 - 'NACK' (NACK bit)
 - 'BITS' (<pdata>: list of data/address bits and their ss/es numbers)

<pdata> is the data or address byte associated with the 'ADDRESS*' and 'DATA*'
command. Slave addresses do not include bit 0 (the READ/WRITE indication bit).
For example, a slave address field could be 0x51 (instead of 0xa2).
For 'START', 'START REPEAT', 'STOP', 'ACK', and 'NACK' <pdata> is None.
'''

# CMD: [annotation-type-index, long annotation, short annotation]
proto = {
    'START':           [0, 'Start',         'S'],
    'START REPEAT':    [1, 'Start repeat',  'Sr'],
    'STOP':            [2, 'Stop',          'P'],
    'ACK':             [3, 'ACK',           'A'],
    'NACK':            [4, 'NACK',          'N'],
    'BIT':             [5, 'Bit',           'B'],
    'CMD FRAME':       [6, 'Command Frame', 'CF'],
    'DATA FRAME':      [7, 'Data Frame',    'DF'],
}

class Decoder(srd.Decoder):
    api_version = 3
    id = 'spmi'
    name = 'SPMI'
    longname = 'System Power Management Interface'
    desc = 'Two-wire, multi-host, serial bus.'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['spmi']
    tags = ['Embedded/industrial']
    channels = (
        {'id': 'SCLK', 'name': 'SCLK', 'desc': 'Serial clock line'},
        {'id': 'sdataTA', 'name': 'sdataTA', 'desc': 'Serial data line'},
    )
    options = (
        {'id': 'address_format', 'desc': 'Displayed client address format',
            'default': 'shifted', 'values': ('shifted', 'unshifted')},
    )
    annotations = (
        ('SSC', 'Start Sequence Condition'),
        ('repeat-start', 'Repeat start condition'),
        ('stop', 'Stop condition'),
        ('ack', 'ACK'),
        ('nack', 'NACK'),
        ('bit', 'Data/address bit'),
        ('command-frame', 'Command Frame'),
        ('data-frame', 'Data Frame'),
        ('warnings', 'Human-readable warnings'),
        ('frame-transmission', 'Frame transmission'),
    )
    annotation_rows = (
        ('bits', 'Bits', (0, 1, 5,)),
        ('addr-data', 'Address/Data', (2, 3, 4, 6, 7,)),
        ('warnings', 'Warnings', (8,)),
        ('frame-transmission', 'Frame transmission', (9,)),
    )
    binary = (
        ('command-frame', 'Command Frame'),
        ('data-frame', 'Data Frame'),
        ('data-read', 'Data read'),
        ('data-write', 'Data write'),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.ss = self.es = self.ss_byte = -1
        self.bitcount = 0
        self.databytes = 0
        self.is_repeat_start = 0
        self.state = 'FIND START'
        self.pdu_start = None
        self.pdu_bits = 0
        self.bits = []

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_python = self.register(srd.OUTPUT_PYTHON)
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)
        self.out_bitrate = self.register(srd.OUTPUT_META,
                meta=(int, 'Bitrate', 'Bitrate from Start bit to Stop bit'))

    def putx(self, data):
        self.put(self.ss, self.es, self.out_ann, data)

    def putp(self, data):
        self.put(self.ss, self.es, self.out_python, data)

    def putb(self, data):
        self.put(self.ss, self.es, self.out_binary, data)

    def handle_start(self, pins):
        self.ss, self.es = self.samplenum, self.samplenum
        self.pdu_start = self.samplenum
        self.pdu_bits = 0
        cmd = 'START REPEAT' if (self.is_repeat_start == 1) else 'START'
        self.putp([cmd, None])
        self.putx([proto[cmd][0], proto[cmd][1:]])
        self.state = 'FIND COMMAND'
        self.bitcount = self.databytes = 0
        self.is_repeat_start = 1
        self.bits = []

    # Parse 13 bit command frame
    def handle_command(self, pins):
        sclk, sdata = pins
        self.pdu_bits += 1

        # Address and data are transmitted MSB-first.
        self.databytes <<= 1
        self.databytes |= sdata

        # Remember the start of the first data/address bit.
        if self.bitcount == 0:
            self.ss_byte = self.samplenum

        # Store individual bits and their start/end samplenumbers.
        # In the list, index 0 represents the LSB (I²C transmits MSB-first).
        self.bits.insert(0, [sdata, self.samplenum, self.samplenum])
        if self.bitcount > 0:
            self.bits[1][2] = self.samplenum
        if self.bitcount == 12:
            self.bitwidth = self.bits[1][2] - self.bits[2][2]
            self.bits[0][2] += self.bitwidth

        # Return if we haven't collected all 12 + 1 bits, yet.
        if self.bitcount < 12:
            self.bitcount += 1
            return

        d = self.databytes

        cmd = 'CMD FRAME'

        self.ss, self.es = self.ss_byte, self.samplenum + self.bitwidth

        self.putp(['BITS', self.bits])
        self.putp([cmd, d])

        self.putb([0, bytes([d >> 8, d & 0xff])])

        for bit in self.bits:
            self.put(bit[1], bit[2], self.out_ann, [5, ['%d' % bit[0]]])

        self.putx([proto[cmd][0], ['%s: %02X' % (proto[cmd][1], d),
                   '%s: %02X' % (proto[cmd][2], d), '%02X' % d]])

        # Done with this packet.
        self.bitcount = self.databytes = 0
        self.bits = []
        self.state = 'FIND DATA'

    # Gather 9 bits of data including parity
    def handle_data_frame(self, pins):
        sclk, sdata = pins
        self.pdu_bits += 1

        # Address and data are transmitted MSB-first.
        if self.bitcount < 8:
            self.databytes <<= 1
            self.databytes |= sdata

        # Remember the start of the first data/address bit.
        if self.bitcount == 0:
            self.ss_byte = self.samplenum

        # Store individual bits and their start/end samplenumbers.
        # In the list, index 0 represents the LSB (I²C transmits MSB-first).
        self.bits.insert(0, [sdata, self.samplenum, self.samplenum])
        if self.bitcount > 0:
            self.bits[1][2] = self.samplenum
        if self.bitcount == 8:
            self.bitwidth = self.bits[1][2] - self.bits[2][2]
            self.bits[0][2] += self.bitwidth

        # Return if we haven't collected all 8 + 1 bits, yet.
        if self.bitcount < 8:
            self.bitcount += 1
            return

        d = self.databytes
        cmd = 'DATA FRAME'

        self.ss, self.es = self.ss_byte, self.samplenum + self.bitwidth

        self.putp(['BITS', self.bits])
        self.putp([cmd, d])

        # Second byte only contains the parity bit
        self.putb([1, bytes([d, sdata])])

        for bit in self.bits:
            self.put(bit[1], bit[2], self.out_ann, [5, ['%d' % bit[0]]])

        self.putx([proto[cmd][0], ['%s: %02X' % (proto[cmd][1], d),
                   '%s: %02X' % (proto[cmd][2], d), '%02X' % d]])

        # Done with this packet.
        self.bitcount = self.databytes = 0
        self.bits = []
        self.state = 'FIND ACK'

    def get_ack(self, pins):
        sclk, sdata = pins
        self.ss, self.es = self.samplenum, self.samplenum + self.bitwidth
        cmd = 'NACK' if (sdata == 1) else 'ACK'
        self.putp([cmd, None])
        self.putx([proto[cmd][0], proto[cmd][1:]])
        # There could be multiple data bytes in a row, so either find
        # another data byte or a STOP condition next.
        self.state = 'FIND DATA'

    def handle_bus_park(self, pins):
        # Meta bitrate
        if self.samplerate:
            elapsed = 1 / float(self.samplerate) * (self.samplenum - self.pdu_start + 1)
            bitrate = int(1 / elapsed * self.pdu_bits)
            self.put(self.ss_byte, self.samplenum, self.out_bitrate, bitrate)

        cmd = 'STOP'
        self.ss, self.es = self.samplenum, self.samplenum
        self.putp([cmd, None])
        self.putx([proto[cmd][0], proto[cmd][1:]])
        self.state = 'FIND START'
        self.is_repeat_start = 0
        self.bits = []

    def decode(self):
        while True:
            # State machine.
            if self.state == 'FIND START':
                # Wait for a START condition (S): SCLK = low, SDATA = high.
                self.handle_start(self.wait({0: 'l', 1: 'h'}))
            elif self.state == 'FIND COMMAND':
                # Wait for a data bit: SCLK = rising.
                self.handle_command(self.wait({0: 'r'}))
            elif self.state == 'FIND DATA':
                # Wait for any of the following conditions (or combinations):
                #  a) Data sampling of receiver: SCLK = rising, and/or
                #  b) START condition (S): SCLK = low, SDATA = high, and/or
                #  c) PARK condition (P): SCLK = high, SDATA = falling
                pins = self.wait([{0: 'r'}, {0: 'l', 1: 'h'}, {0: 'h', 1: 'f'}])

                # Check which of the condition(s) matched and handle them.
                if self.matched[0]:
                    self.handle_data_frame(pins)
                elif self.matched[1]:
                    self.handle_start(pins)
                elif self.matched[2]:
                    self.handle_bus_park(pins)
            elif self.state == 'FIND ACK':
                # Wait for a data/ack bit: SCLK = high.
                self.get_ack(self.wait({0: 'h'}))
