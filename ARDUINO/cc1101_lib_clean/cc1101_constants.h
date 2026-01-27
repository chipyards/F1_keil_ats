#define BASE_TUNING (434000)
// CC1101 fine tuning (compensation of manufacturing variation of ref. ocillator frequency)
// #define FINE_TUNING (36)  // shield 1 
#define FINE_TUNING (37)  // shield 2 
// #define FINE_TUNING (33)  // shield 3 
// #define FINE_TUNING (31)  // shield 4 
// #define FINE_TUNING (31)  // shield 5 
// #define FINE_TUNING (30)  // shield 6
///
/// CC1101 constants
///

// configuration registers
//			adr   dump doc	<-- dump = hard reset values, doc = CC1101.pdf defaults
#define CC1101_IOCFG2   0x00 // 29 29 GDO2 
#define CC1101_IOCFG1   0x01 // 2e 2E GDO1 - Hi Z
#define CC1101_IOCFG0   0x02 // 3f 3F GDO0 - default 3F=135.4kHz
#define CC1101_FIFOTHR  0x03 // 07 07 FIFO Threshold & RX attenuator 07 = half fifo, 0dB att
#define CC1101_SYNC1    0x04 // d3 D3 Sync Word, High Byte 11010011
#define CC1101_SYNC0    0x05 // 91 91 Sync Word, Low Byte  10010001
#define CC1101_PKTLEN   0x06 // ff FF Packet Length (ou max length si variable)
#define CC1101_PKTCTRL1 0x07 // 04 04 Packet Preamble Qual. Estim.=0(laxiste), CRC autoflush=0(laxiste), Append Status=1(RSSI,LQI),Address=0(off)
#define CC1101_PKTCTRL0 0x08 // 45 45 Packet Whiten=on, mode normal(packet use FIFOs), CRC=on, length=variable
#define CC1101_ADDR     0x09 // 00 00 Device Address
#define CC1101_CHANNR   0x0A // 00 00 Channel Number
#define CC1101_FSCTRL1  0x0B // 0f 0F IF frequ 0F=381kHz, resolution Fxtal/(1<<10)
#define CC1101_FSCTRL0  0x0C // 00 00 Permanent Frequency Offset compensation (inspiree par le reg. FREQEST, resolution Fxtal/(1<<14)
#define CC1101_FREQ2    0x0D // 1e 1E Frequ Synth big endian, resolution Fxtal/(1<<16)
#define CC1101_FREQ1    0x0E // c4 C4 Frequ Synth "
#define CC1101_FREQ0    0x0F // ec EC Frequ Synth "	--> 800 MHz
#define CC1101_MDMCFG4  0x10 // 8c 8C Chan. Bandwidth 80=203kHz, resolution Fxtal/(1<<10), Symbol Rate exponent 0C
#define CC1101_MDMCFG3  0x11 // 22 22 Symbol Rate Mantissa M=22 E=0C ci-dessus --> 115.051 kBaud, resolution Fxtal/(1<<28)
#define CC1101_MDMCFG2  0x12 // 02 02 DC Rx filter=on, modulation FSK, Manchester=off, sync detect=16/16
#define CC1101_MDMCFG1  0x13 // 22 22 FEC=off, preamble=4bytes, Channel Spacing Exponent=2
#define CC1101_MDMCFG0  0x14 // f8 F8 Channel Spacing Mantissa=F8 --> 199.951 kHz, resolution Fxtal/(1<<18)
#define CC1101_DEVIATN  0x15 // 47 47 FM Deviation (on each side of base frequ.) -> 47.607 kHz, resolution Fxtal/(1<<17)
#define CC1101_MCSM2    0x16 // 07 07 RX Timout=off (leave RX at end of packet) N.B. otherwise timer needs RC osc
#define CC1101_MCSM1    0x17 // 30 30 CCA="low RSSI andnot receiving", RXOFF->IDLE, TXOFF->IDLE
#define CC1101_MCSM0    0x18 // 04 04 FS_AUTOCAL=off, PowerOn timout=37us(recomm plus), 3-Pin Control(hack)=off, force XOSC=off
#define CC1101_FOCCFG   0x19 // 76!36 Frequ Offset Compens: freeze until CS high=on, gain before sync=3K, after=K/2, saturation=on,+-BW/4
#define CC1101_BSCFG    0x1A // 6c 6C Bit Synchronization Configuration: loop gain before sync i=2K, p=3k, after i=K/2, p=K, saturation=0(loop off)
#define CC1101_AGCCTRL2 0x1B // 03 03 AGC: DVGA gain=max, LNA gain=max, target amplitude (consigne)=33dB (medium)
#define CC1101_AGCCTRL1 0x1C // 40 40 AGC: LNA gain prority=1, CS relative change threshold=off, CS absolute threshold=target amplitude
#define CC1101_AGCCTRL0 0x1D // 91 91 AGC: hysteresis=medium, delay=medium, freeze=off, FIR filter=16samples, OOK/ASK boundary=8dB
#define CC1101_WOREVT1  0x1E // 87 87 WOR Event0 Timeout: big endian, sera affecte d'un exposant 5*WOR_RES
#define CC1101_WOREVT0  0x1F // 6b 6B WOR Event0 Timeout: "   -> 1s
#define CC1101_WORCTRL  0x20 // f8 F8 WOR Wake On Radio: RC osc powerdown=1, Event1 delay=1.333ms, RC osc calibration=on, WOR_RES resolution=28us
#define CC1101_FREND1   0x21 // 56 56 Front End RX Config - undocumented
#define CC1101_FREND0   0x22 // 10 10 Front End TX Config: PA buffer current (undocumented), PA_POWER=0 aka max index (inclusive) dans PATABLE
#define CC1101_FSCAL3   0x23 // a9 A9 Frequency Synthesizer Calibration: VCO charge pump, calibration=on
#define CC1101_FSCAL2   0x24 // 0a 0A Frequency Synthesizer Calibration: VCO current
#define CC1101_FSCAL1   0x25 // 20 20 Frequency Synthesizer Calibration: VCO capacitor array
#define CC1101_FSCAL0   0x26 // 0d 0D Frequency Synthesizer Calibration - undocumented
#define CC1101_RCCTRL1  0x27 // 41 41 RC Oscillator Config - undocumented
#define CC1101_RCCTRL0  0x28 // 00 00 RC Oscillator Config - undocumented
#define CC1101_FSTEST	0x29 // 59 59 - undocumented
//#define CC1101_PTEST	0x2A // 7f 7F temperature sensor - write BF to enable in IDLE state
//#define CC1101_AGCTEST0x2B // 3f 3F - undocumented
#define CC1101_TEST2	0x2C // 88 88 - undocumented
#define CC1101_TEST1	0x2D // 31 31 - undocumented
#define CC1101_TEST0	0x2E // 0b 0B - undocumented

// CC1101 Strobe commands
#define CC1101_SRES         0x30        // Reset chip.
#define CC1101_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is running (for quick RX / TX turnaround).
#define CC1101_SXOFF        0x32        // Turn off crystal oscillator.
#define CC1101_SCAL         0x33        // Calibrate frequency synthesizer and turn it off (enables quick start).
#define CC1101_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
#define CC1101_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if MCSM0.FS_AUTOCAL=1.
					// If in RX state and CCA is enabled: Only go to TX if channel is clear.
#define CC1101_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
#define CC1101_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer UNDOCUMENTED
#define CC1101_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio) if WORCTRL.RC_PD=0
#define CC1101_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CC1101_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST      0x3C        // Reset real time clock to Event1 value
#define CC1101_SNOP         0x3D        // No operation (but read status byte).

// CC1101 STATUS REGISTERS
#define CC1101_PARTNUM           0x30        // Chip ID
#define CC1101_VERSION           0x31        // Chip ID
#define CC1101_FREQEST           0x32        // Frequency Offset Estimate from Demodulator
#define CC1101_LQI               0x33        // Demodulator Estimate for Link Quality
#define CC1101_RSSI              0x34        // Received Signal Strength Indication
#define CC1101_MARCSTATE         0x35        // Main Radio Control State Machine State
#define CC1101_WORTIME1          0x36        // High Byte of WOR Time
#define CC1101_WORTIME0          0x37        // Low Byte of WOR Time
#define CC1101_PKTSTATUS         0x38        // Current GDOx Status and Packet Status
#define CC1101_VCO_VC_DAC        0x39        // Current Setting from PLL Calibration Module
#define CC1101_TXBYTES           0x3A        // Underflow and Number of Bytes
#define CC1101_RXBYTES           0x3B        // Overflow and Number of Bytes
#define CC1101_RCCTRL1_STATUS    0x3C        // Last RC Oscillator Calibration Result
#define CC1101_RCCTRL0_STATUS    0x3D        // Last RC Oscillator Calibration Result

// CC1101 modulation methods
#define CC1101_2FSK	0
#define CC1101_GFSK	1
#define CC1101_ASKOOK	3
#define CC1101_AM	3
#define CC1101_4FSK	4
#define CC1101_MSK	7

// CC1101 GDO settings p. 62 - add 0x40 to invert
#define CC1101_GDO_RXFIFO  0 // Rx fifo above threshold, cleared by reading the FIFO below threshold
#define CC1101_GDO_RXEND  1 // Rx fifo above threshold or end-of-packet, cleared by drainig the FIFO
#define CC1101_GDO_P_IN_PROC	6	// Tx or Rx in process, from sync to end
#define CC1101_GDO_CRC_OK	7	// Rx CRC Ok, cleared by reading the FIFO
#define CC1101_GDO_CCA		9	// CCA
#define CC1101_GDO_C_SENSE	14	// Carrier sense

#define CC1101_GDO_HIZ		46	// high Z
#define CC1101_GDO_LO		0x2F	// 0
#define CC1101_GDO_HI		0x6F	// 1
#define CC1101_GDO_13MHZ	50	// XOSC/2 pour frequencemetre
