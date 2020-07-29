from math import ceil

from migen import *
from migen.genlib.fsm import *
from migen.genlib.misc import WaitTimer
from migen.genlib.cdc import PulseSynchronizer

from misoc.interconnect import stream
from misoc.cores import code_8b10b
from misoc.cores.liteeth_mini.common import *


__all__ = ["TransmitPath", "ReceivePath", "PCS"]


def K(x, y):
    return (y << 5) | x


def D(x, y):
    return (y << 5) | x


class TransmitPath(Module):
    def __init__(self, lsb_first=False):
        self.config_stb = Signal()
        self.config_reg = Signal(16)
        self.tx_stb = Signal()
        self.tx_ack = Signal()
        self.tx_data = Signal(8)

        self.submodules.encoder = code_8b10b.Encoder(lsb_first=lsb_first)

        # SGMII Speed Adaptation
        self.timer = Signal(max=1000)
        self.sgmii_speed = Signal(2)

        # # #

        parity = Signal()
        c_type = Signal()
        self.sync += parity.eq(~parity)

        config_reg_buffer = Signal(16)
        load_config_reg_buffer = Signal()
        self.sync += If(load_config_reg_buffer, config_reg_buffer.eq(self.config_reg))

        # Timer for SGMII data rates
        timer_en = Signal()
        self.sync += [
            If(~timer_en | (self.timer == 0),
                If(self.sgmii_speed == 0b00,
                    self.timer.eq(99)
                ).Elif(self.sgmii_speed == 0b01,
                    self.timer.eq(9)
                ).Elif(self.sgmii_speed == 0b10,
                    self.timer.eq(0)
                )
            ).Elif(timer_en,
                self.timer.eq(self.timer - 1)
            )
        ]

        fsm = FSM()
        self.submodules += fsm

        fsm.act("START",
            If(self.config_stb,
                self.tx_ack.eq(1),  # discard TX data if we are in config_reg phase
                load_config_reg_buffer.eq(1),
                self.encoder.k[0].eq(1),
                self.encoder.d[0].eq(K(28, 5)),
                NextState("CONFIG_D")
            ).Else(
                If(self.tx_stb,
                    # the first byte sent is replaced by /S/
                    self.tx_ack.eq((self.timer == 0)),
                    timer_en.eq(1),
                    self.encoder.k[0].eq(1),
                    self.encoder.d[0].eq(K(27, 7)),
                    NextState("DATA")
                ).Else(
                    self.tx_ack.eq(1),  # discard TX data
                    self.encoder.k[0].eq(1),
                    self.encoder.d[0].eq(K(28, 5)),
                    NextState("IDLE")
                )
            )
        )
        # /K28.5/ == /C/
        # It is the first code-group in an ordered_set that provides 
        # synchronisation of bits & code-groups as well as establishes 
        # alignment of ordered_sets.
        fsm.act("CONFIG_D",
            If(c_type,
                self.encoder.d[0].eq(D(2, 2))
            ).Else(
                self.encoder.d[0].eq(D(21, 5))
            ),
            NextValue(c_type, ~c_type),
            NextState("CONFIG_REG_LSB")
        ),
        fsm.act("CONFIG_REG_LSB",
            self.encoder.d[0].eq(config_reg_buffer[:8]),
            NextState("CONFIG_REG_MSB")
        )
        fsm.act("CONFIG_REG_MSB",
            self.encoder.d[0].eq(config_reg_buffer[8:]),
            NextState("START")
        )
        fsm.act("IDLE",
            # due to latency in the encoder, we read here the disparity
            # just before the K28.5 was sent. K28.5 flips the disparity.
            If(self.encoder.disparity[0],
                # correcting /I1/ (D5.6 preserves the disparity)
                self.encoder.d[0].eq(D(5, 6))
            ).Else(
                # preserving /I2/ (D16.2 flips the disparity)
                self.encoder.d[0].eq(D(16, 2))
            ),
            NextState("START")
        )
        fsm.act("DATA",
            If(self.tx_stb,
                self.tx_ack.eq((self.timer == 0)),
                timer_en.eq(1),
                self.encoder.d[0].eq(self.tx_data)
            ).Else(
                self.tx_ack.eq(1),
                # /T/
                self.encoder.k[0].eq(1),
                self.encoder.d[0].eq(K(29, 7)),
                NextState("CARRIER_EXTEND_1")
            )
        )
        fsm.act("CARRIER_EXTEND_1",
            # /R/
            self.encoder.k[0].eq(1),
            self.encoder.d[0].eq(K(23, 7)),
            If(parity,
                NextState("START")
            ).Else(
                NextState("CARRIER_EXTEND_2")
            )
        )
        fsm.act("CARRIER_EXTEND_2",
            # /R/
            self.encoder.k[0].eq(1),
            self.encoder.d[0].eq(K(23, 7)),
            NextState("START")
        )


class ReceivePath(Module):
    def __init__(self, lsb_first=False):
        self.rx_en = Signal()
        self.rx_data = Signal(8)

        self.seen_valid_ci = Signal()
        self.seen_config_reg = Signal()
        self.config_reg = Signal(16)

        self.submodules.decoder = code_8b10b.Decoder(lsb_first=lsb_first)

        # SGMII Speed Adaptation
        self.timer = Signal(max=1000)
        self.sgmii_speed = Signal(2)
        self.sample_en = Signal()

        # # #

        config_reg_lsb = Signal(8)
        load_config_reg_lsb = Signal()
        self.load_config_reg_msb = load_config_reg_msb = Signal()
        self.sync += [
            self.seen_config_reg.eq(0),
            If(load_config_reg_lsb, 
                config_reg_lsb.eq(self.decoder.d)
            ),
            If(load_config_reg_msb,
                self.config_reg.eq(Cat(config_reg_lsb, self.decoder.d)),
                self.seen_config_reg.eq(1)
            )
        ]

        first_preamble_byte = Signal()
        self.comb += self.rx_data.eq(Mux(first_preamble_byte, 0x55, self.decoder.d))

        # Timer for SGMII data rates
        timer_en = Signal()
        self.sync += [
            If(~timer_en | (self.timer == 0),
                If(self.sgmii_speed == 0b00,
                    self.timer.eq(99)
                ).Elif(self.sgmii_speed == 0b01,
                    self.timer.eq(9)
                ).Elif(self.sgmii_speed == 0b10,
                    self.timer.eq(0)
                )
            ).Elif(timer_en,
                self.timer.eq(self.timer - 1)
            )
        ]

        # Speed adaptation
        self.comb += self.sample_en.eq(self.rx_en & (self.timer == 0))

        # PCS receive state diagram, Figure 36-7a/b
        fsm = FSM()
        self.submodules += fsm

        fsm.act("START",
            If(self.decoder.k,
                If(self.decoder.d == K(28, 5),
                    NextState("K28_5")
                ),
                If(self.decoder.d == K(27, 7),
                    self.rx_en.eq(1),
                    timer_en.eq(1),
                    first_preamble_byte.eq(1),
                    NextState("DATA")
                )
            )
        )
        # /K28.5/ == /C/
        # It is the first code-group in an ordered_set that provides 
        # synchronisation of bits & code-groups as well as establishes 
        # alignment of ordered_sets.
        # RX_CB
        fsm.act("K28_5",
            NextState("START"),
            If(~self.decoder.k,
                If((self.decoder.d == D(21, 5)) | (self.decoder.d == D(2, 2)),
                    self.seen_valid_ci.eq(1),
                    NextState("CONFIG_REG_LSB")
                ),
                If((self.decoder.d == D(5, 6)) | (self.decoder.d == D(16, 2)),
                    # idle
                    self.seen_valid_ci.eq(1),
                    NextState("START")
                ),
            )
        )
        # RX_CC
        fsm.act("CONFIG_REG_LSB",
            If(self.decoder.k,
                If(self.decoder.d == K(27, 7),
                    self.rx_en.eq(1),
                    timer_en.eq(1),
                    first_preamble_byte.eq(1),
                    NextState("DATA")
                ).Else(
                    NextState("START")  # error
                )
            ).Else(
                load_config_reg_lsb.eq(1),
                NextState("CONFIG_REG_MSB")
            )
        )
        # RX_CD
        fsm.act("CONFIG_REG_MSB",
            If(~self.decoder.k,
                load_config_reg_msb.eq(1)
            ),
            NextState("START")
        )
        fsm.act("DATA",
            If(self.decoder.k,
                NextState("START")
            ).Else(
                self.rx_en.eq(1),
                timer_en.eq(1)
            )
        )


class PCS(Module):
    def __init__(self, lsb_first=False, check_period=6e-3, more_ack_time=10e-3):
        self.submodules.tx = ClockDomainsRenamer("eth_tx")(
            TransmitPath(lsb_first=lsb_first))
        self.submodules.rx = ClockDomainsRenamer("eth_rx")(
            ReceivePath(lsb_first=lsb_first))

        self.tbi_tx = self.tx.encoder.output[0]
        self.tbi_rx = self.rx.decoder.input
        self.sink = stream.Endpoint(eth_phy_layout(8))
        self.source = stream.Endpoint(eth_phy_layout(8))
        
        self.link_up = Signal()
        self.restart = Signal()

        # SGMII Speed Adaptation
        self.is_sgmii = Signal()
        self.link_partner_adv_ability = Signal(16)

        # # #
        
        # endpoint interface
        self.comb += [
            self.tx.tx_stb.eq(self.sink.stb),
            self.sink.ack.eq(self.tx.tx_ack),
            self.tx.tx_data.eq(self.sink.data),
        ]

        rx_en_d = Signal()
        self.sync.eth_rx += [
            rx_en_d.eq(self.rx.rx_en),
            self.source.stb.eq(self.rx.sample_en),
            self.source.data.eq(self.rx.rx_data)
        ]
        self.comb += self.source.eop.eq(~self.rx.rx_en & rx_en_d)

        # main module
        seen_valid_ci = PulseSynchronizer("eth_rx", "eth_tx")
        self.submodules += seen_valid_ci
        self.comb += seen_valid_ci.i.eq(self.rx.seen_valid_ci)

        checker_max_val = ceil(check_period*125e6)
        checker_counter = Signal(max=checker_max_val+1)
        checker_tick = Signal()
        checker_ok = Signal()
        self.sync.eth_tx += [
            checker_tick.eq(0),
            If(checker_counter == 0,
                checker_tick.eq(1),
                checker_counter.eq(checker_max_val)
            ).Else(
                checker_counter.eq(checker_counter-1)
            ),
            If(seen_valid_ci.o, checker_ok.eq(1)),
            If(checker_tick, checker_ok.eq(0))
        ]

        autoneg_ack = Signal()
        self.comb += self.tx.config_reg.eq(
            (self.is_sgmii) |       # SGMII-specific
            (~self.is_sgmii << 5) | # Full-duplex
            (autoneg_ack << 14)     # ACK
        )

        rx_config_reg_abi = PulseSynchronizer("eth_rx", "eth_tx")
        rx_config_reg_ack = PulseSynchronizer("eth_rx", "eth_tx")
        self.submodules += [
            rx_config_reg_abi, rx_config_reg_ack
        ]

        more_ack_timer = ClockDomainsRenamer("eth_tx")(
            WaitTimer(ceil(more_ack_time*125e6)))
        self.submodules += more_ack_timer

        fsm_inited = Signal()
        config_reg_empty = Signal()

        fsm = ClockDomainsRenamer("eth_tx")(FSM())
        self.submodules += fsm

        # Only use checker for ability advertisement in non-SGMII mode
        fsm.act("AUTONEG_WAIT_ABI",
            self.tx.config_stb.eq(fsm_inited),
            If(rx_config_reg_abi.o,
                NextValue(fsm_inited, 1),
                NextState("AUTONEG_WAIT_ACK")
            ),
            If(checker_tick & ~checker_ok,
                self.restart.eq(1),
                NextState("AUTONEG_WAIT_ABI")
            )
        )
        fsm.act("AUTONEG_WAIT_ACK",
            self.tx.config_stb.eq(1),
            autoneg_ack.eq(1),
            If(rx_config_reg_ack.o,
                NextState("AUTONEG_SEND_MORE_ACK")
            ),
            If(checker_tick & ~checker_ok,
                self.restart.eq(1),
                NextState("AUTONEG_WAIT_ABI")
            )
        )
        # COMPLETE_ACKNOWLEDGE
        fsm.act("AUTONEG_SEND_MORE_ACK",
            self.tx.config_stb.eq(1),
            autoneg_ack.eq(1),
            more_ack_timer.wait.eq(1),
            If(more_ack_timer.done,
                NextState("RUNNING")
            ),
            If(checker_tick & ~checker_ok,
                self.restart.eq(1),
                NextState("AUTONEG_WAIT_ABI")
            )
        )
        # LINK_OK
        fsm.act("RUNNING",
            self.link_up.eq(1),
            If((checker_tick & ~checker_ok) | config_reg_empty,
                self.restart.eq(1),
                NextState("AUTONEG_WAIT_ABI")
            )
        )

        c_counter = Signal(max=5)
        previous_config_reg = Signal(16)
        preack_config_reg = Signal(16)
        self.sync.eth_rx += [
            # Restart consistency counter
            If(self.rx.seen_config_reg,
                c_counter.eq(4)
            ).Elif(c_counter != 0,
                c_counter.eq(c_counter - 1)
            ),

            rx_config_reg_abi.i.eq(0),
            rx_config_reg_ack.i.eq(0),
            If(self.rx.seen_config_reg,
                previous_config_reg.eq(self.rx.config_reg),
                If((c_counter == 1) & 
                    ((previous_config_reg|0x4000) == (self.rx.config_reg|0x4000)) &
                    ((self.rx.config_reg | 1) != 1),
                    # Ability match
                    rx_config_reg_abi.i.eq(1),
                    preack_config_reg.eq(previous_config_reg),
                    # Acknowledgement/Consistency match
                    If((previous_config_reg[14] & self.rx.config_reg[14]) &
                        ((preack_config_reg|0x4000) == (self.rx.config_reg|0x4000)),
                        rx_config_reg_ack.i.eq(1)
                    )
                ),
                # Record advertised ability of link partner
                self.link_partner_adv_ability.eq(self.rx.config_reg)
            )
        ]

        # Speed detection via SGMII
        sgmii_speed = Mux(self.is_sgmii,
            self.link_partner_adv_ability[10:12], 0b10)
        self.comb += [
            self.is_sgmii.eq(self.link_partner_adv_ability[0]),
            self.tx.sgmii_speed.eq(sgmii_speed),
            self.rx.sgmii_speed.eq(sgmii_speed)
        ]

        # Detect that config_reg is empty
        self.comb += [
            config_reg_empty.eq((self.link_partner_adv_ability | 1) == 1)
        ]
