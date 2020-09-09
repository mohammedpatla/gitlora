import time
from enum import Enum
import math
from collections import namedtuple
from random import random

import RPi.GPIO as GPIO
import spidev

from .constants import *

import binascii
import struct

class LoRa(object):
    def __init__(self, bus, channel, interrupt, freq=915, tx_power=14,
                 sync_word=0xF3,
                 modem_config=ModemConfig.Bw125Cr45Sf128, receive_all=False,
                 acks=False, crypto=None):

        self._bus = bus
        self._channel = channel
        self._interrupt = interrupt

        self._mode = None
        self._cad = None
        self._freq = freq
        self._tx_power = tx_power
        self._sync_word = sync_word
        self._modem_config = modem_config
        self._receive_all = receive_all
        self._acks = acks

        self._last_header_id = 0

        self._last_payload = None
        self.crypto = crypto

        self.cad_timeout = 0
        self.send_retries = 2
        self.wait_packet_sent_timeout = 0.2
        self.retry_timeout = 0.2

        # Setup the module
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._interrupt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self._interrupt, GPIO.RISING, callback=self._handle_interrupt)

        self.spi = spidev.SpiDev()
        self.spi.open(self._bus, self._channel)
        self.spi.max_speed_hz = 5000000

        self._spi_write(REG_01_OP_MODE, MODE_SLEEP | LONG_RANGE_MODE)
        time.sleep(0.1)

        assert self._spi_read(REG_01_OP_MODE) == (MODE_SLEEP | LONG_RANGE_MODE), \
            "LoRa initialization failed"

        self._spi_write(REG_0E_FIFO_TX_BASE_ADDR, 0)
        self._spi_write(REG_0F_FIFO_RX_BASE_ADDR, 0)

        self.set_mode_idle()

        # set modem config (Bw125Cr45Sf128)
        self._spi_write(REG_1D_MODEM_CONFIG1, self._modem_config.value[0])
        self._spi_write(REG_1E_MODEM_CONFIG2, self._modem_config.value[1])
        self._spi_write(REG_26_MODEM_CONFIG3, self._modem_config.value[2])

        # set preamble length (8)
        self._spi_write(REG_20_PREAMBLE_MSB, 0)
        self._spi_write(REG_21_PREAMBLE_LSB, 8)

        # set frequency
        frf = int((self._freq * 1000000.0) / FSTEP)
        self._spi_write(REG_06_FRF_MSB, (frf >> 16) & 0xff)
        self._spi_write(REG_07_FRF_MID, (frf >> 8) & 0xff)
        self._spi_write(REG_08_FRF_LSB, frf & 0xff)

        # Set tx power
        if self._tx_power < 5:
            self._tx_power = 5
        if self._tx_power > 23:
            self._tx_power = 23

        if self._tx_power < 20:
            self._spi_write(REG_4D_PA_DAC, PA_DAC_ENABLE)
            self._tx_power -= 3
        else:
            self._spi_write(REG_4D_PA_DAC, PA_DAC_DISABLE)

        self._spi_write(REG_09_PA_CONFIG, PA_SELECT | (self._tx_power - 5))

        # set the sync word
        self._spi_write(REG_39_SYNC_WORD, sync_word)

        
    def on_recv(self, message):
        # This should be overridden by the user
        pass

    def sleep(self):
        if self._mode != MODE_SLEEP:
            self._spi_write(REG_01_OP_MODE, MODE_SLEEP)
            self._mode = MODE_SLEEP

    def set_mode_tx(self):
        if self._mode != MODE_TX:
            self._spi_write(REG_01_OP_MODE, MODE_TX)
            self._spi_write(REG_40_DIO_MAPPING1, 0x40)  # Interrupt on TxDone
            self._mode = MODE_TX

    def set_mode_rx(self):
        if self._mode != MODE_RXCONTINUOUS:
            self._spi_write(REG_01_OP_MODE, MODE_RXCONTINUOUS)
            self._spi_write(REG_40_DIO_MAPPING1, 0x00)  # Interrupt on RxDone
            self._mode = MODE_RXCONTINUOUS

    def set_mode_cad(self):
        if self._mode != MODE_CAD:
            self._spi_write(REG_01_OP_MODE, MODE_CAD)
            self._spi_write(REG_40_DIO_MAPPING1, 0x80)  # Interrupt on CadDone
            self._mode = MODE_CAD

    def _is_channel_active(self):
        self.set_mode_cad()

        while self._mode == MODE_CAD:
            yield

        return self._cad

    def wait_cad(self):
        if not self.cad_timeout:
            return True

        start = time.time()
        for status in self._is_channel_active():
            if time.time() - start < self.cad_timeout:
                return False

            if status is None:
                time.sleep(0.1)
                continue
            else:
                return status

    def wait_packet_sent(self):
        # wait for `_handle_interrupt` to switch the mode back
        start = time.time()
        while time.time() - start < self.wait_packet_sent_timeout:
            if self._mode != MODE_TX:
                return True

        return False

    def set_mode_idle(self):
        if self._mode != MODE_STDBY:
            self._spi_write(REG_01_OP_MODE, MODE_STDBY)
            self._mode = MODE_STDBY

    def send(self, data, sensor_id, packet_type, packet_num):
        self.wait_packet_sent()
        self.set_mode_idle()
        self.wait_cad()

        header = [sensor_id >> 24, sensor_id >> 16, sensor_id >> 8, sensor_id % 0x100, packet_type]
        if type(data) == int:
            data = [data]
        elif type(data) == bytes:
            data = [p for p in data]
        elif type(data) == str:
            data = [ord(s) for s in data]

        if self.crypto and data:
            data = [b for b in self._encrypt(bytes(data))]

        if (data == None):
            payload = header
        else:
            payload = header + data

        self._spi_write(REG_0D_FIFO_ADDR_PTR, 0)
        self._spi_write(REG_00_FIFO, payload)
        self._spi_write(REG_22_PAYLOAD_LENGTH, len(payload))

        self.set_mode_tx()
        return True

    def send_to_wait(self, data, sensor_id, packet_type, packet_num, retries=3):
        self._last_header_id += 1

        for _ in range(retries + 1):
            self.send(data, sensor_id, packet_type, packet_num)
            self.set_mode_rx()

            # if header_to == BROADCAST_ADDRESS:  # Don't wait for acks from a broadcast message
            #     return True

            start = time.time()
            while time.time() - start < self.retry_timeout + (self.retry_timeout * random()):
                if self._last_payload:
                    if self._last_payload.sensor_id == sensor_id and \
                            self._last_payload.packet_type == PacketType.SENSOR_ACK:
                        
                        # We got an ACK
                        return True
        return False

    def send_ack(self, sensor_id):
        self.send(None, sensor_id, PacketType.GATEWAY_ACK.value, 0)
        self.wait_packet_sent()
        time.sleep(0.001)
        self.set_mode_rx()

    def _spi_write(self, register, payload):
        if type(payload) == int:
            payload = [payload]
        elif type(payload) == bytes:
            payload = [p for p in payload]
        elif type(payload) == str:
            payload = [ord(s) for s in payload]

        self.spi.xfer([register | 0x80] + payload)

    def _spi_read(self, register, length=1):
        if length == 1:
            return self.spi.xfer([register] + [0] * length)[1]
        else:
            return self.spi.xfer([register] + [0] * length)[1:]

    def _decrypt(self, message):
        return self.crypto.decrypt(message)

    def _encrypt(self, message):
        return self.crypto.encrypt(message)

    def _handle_interrupt(self, channel):
        irq_flags = self._spi_read(REG_12_IRQ_FLAGS)


        # If supposed to be receiving a message...
        if self._mode == MODE_RXCONTINUOUS and (irq_flags & RX_DONE):

            # check if CRC is good, if not do nothing
            if (irq_flags & PAYLOAC_CRC_ERROR) == 0:

                packet_len = self._spi_read(REG_13_RX_NB_BYTES)
                self._spi_write(REG_0D_FIFO_ADDR_PTR, self._spi_read(REG_10_FIFO_RX_CURRENT_ADDR))

                packet = self._spi_read(REG_00_FIFO, packet_len)
                self._spi_write(REG_12_IRQ_FLAGS, 0xff)  # Clear all IRQ flags

                snr = self._spi_read(REG_19_PKT_SNR_VALUE) / 4
                rssi = self._spi_read(REG_1A_PKT_RSSI_VALUE)

                if snr < 0:
                    rssi = snr + rssi
                else:
                    rssi = rssi * 16 / 15

                if self._freq >= 779:
                    rssi = round(rssi - 157, 2)
                else:
                    rssi = round(rssi - 164, 2)

                if packet_len >= HEADER_LEN:

                    sensor_id = (packet[0] << 24) | (packet[1] << 16) | (packet[2] << 8) | packet[3]
                    packet_type = packet[4]
                    # parse message into an array of bytes
                    data = list()

                    message = bytes(packet[HEADER_LEN:]) if packet_len > HEADER_LEN else b''

                    if self.crypto and message:
                        message = self._decrypt(message)         
                    
                    for x in range(0, len(message) // 2):
                        lowbyte = message[x*2]
                        highbyte = message[x*2+1]
                        data.append(highbyte << 8 | lowbyte)
                    
                    self.set_mode_rx()

                    self._last_payload = namedtuple(
                        "Payload",
                        ['data', 'sensor_id', 'packet_type', 'rssi', 'snr']
                    )(data, sensor_id, packet_type, rssi, snr)

                    # if not header_flags & FLAGS_ACK:
                    self.on_recv(self._last_payload)

                else:
                    print("Short Packet Received:")
                    for x in packet:
                        print(x, end = '')
                    print("\nEnd of packet")

            

        elif self._mode == MODE_TX and (irq_flags & TX_DONE):
            self.set_mode_idle()

        elif self._mode == MODE_CAD and (irq_flags & CAD_DONE):
            self._cad = irq_flags & CAD_DETECTED
            self.set_mode_idle()

        # clear IRQs
        self._spi_write(REG_12_IRQ_FLAGS, 0xff)


    def close(self):
        GPIO.cleanup()
        self.spi.close()

