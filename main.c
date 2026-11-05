/**
 * @file main.c
 * @author alexandreplotean
 * @date 2026-11-03
 * @brief Peripheral bring-up test: power up the Magnetometer (RM3100), IMU
 *        (ASM330LHHTR) and MRAM (MR25H40) rails, talk to each over its own
 *        SPI bus, and leave the pass/fail results in global variables for
 *        inspection with the debugger (Watch window / print).
 *
 *        Pin map below comes from tracing real wire connectivity in the
 *        Altium schematics (Complete circuit.SchDoc, Sensors.SchDoc,
 *        Memory.SchDoc) -- not guessed from the symbol pinout alone.
 *
 *        SPI1 -> Magnetometer (RM3100):  SCK1=RD1, SDI1=RD2, SDO1=RD3, CS=RD12 (GPIO)
 *        SPI3 -> IMU (ASM330LHHTR):      SCK3=RB14, SDI3=RD14, SDO3=RD15, CS=RB15 (GPIO)
 *        SPI2 -> MRAM (MR25H40):         SCK2=RG6, SDI2=RC4, SDO2=RC1, CS=RC2 (GPIO)
 *                                        HOLD=RC3 (active-low, drive high), WP=RE7 (active-low, drive high)
 *        Switched 3V3 enables: MGN-EN=RD4, IMU-EN=RF3, MRAM-EN=RE5
 */
#include <xc.h>

#pragma config POSCMOD = OFF       // primary osc off for this test; OSC1/OSC2 behave as plain GPIO/inputs
#pragma config FNOSC = FRCDIV      // internal fast RC, no external clock dependency
#pragma config OSCIOFNC = OFF
#pragma config FSOSCEN = OFF       // secondary osc pins (SOSCI/SOSCO) are repurposed as GPIO/T1CK here
#pragma config IESO = OFF
#pragma config FCKSM = CSECMD
#pragma config JTAGEN = ON         // preserve the 4-wire JTAG debug link (this board has no usable 2-wire ICSP)
#pragma config FWDTEN = OFF        // no watchdog while bringing the board up

/* ---- results, inspect these with the debugger after it halts at the end of test() ---- */
volatile uint8_t g_mgn_revid   = 0;    // RM3100 REVID register (0x36); expect 0x22
volatile uint8_t g_imu_whoami  = 0;    // ASM330LHHTR WHO_AM_I register (0x0F); expect 0x6B
volatile uint8_t g_mram_sr_set = 0xFF; // MRAM status reg right after WREN; expect WEL bit (bit1) = 1
volatile uint8_t g_mram_sr_clr = 0xFF; // MRAM status reg right after WRDI; expect WEL bit (bit1) = 0
volatile uint8_t g_mgn_ok  = 0;
volatile uint8_t g_imu_ok  = 0;
volatile uint8_t g_mram_ok = 0;
volatile uint32_t g_test_done = 0;     // set to 0xA5A5A5A5 once every test has run

static void delay_cycles(uint32_t n) {
    volatile uint32_t i;
    for (i = 0; i < n; i++) {
        ;
    }
}

/* Unlock sequence required before PPS (RPnR / SDIxR) registers can be written. */
static void pps_unlock(void) {
    SYSKEY = 0x00000000;
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;
    CFGCONbits.IOLOCK = 0;
}

static void pps_lock(void) {
    CFGCONbits.IOLOCK = 1;
    SYSKEY = 0x00000000;
}

static void spi_init_master_mode0(volatile uint32_t *con, volatile uint32_t *brg) {
    *con = 0;                 // module off while configuring
    *brg = 40;                // slow, conservative SPI clock regardless of exact PBCLK
    __SPI1CONbits_t *bits = (__SPI1CONbits_t *)con;
    bits->MSTEN = 1;          // master mode
    bits->CKP = 0;            // idle clock low
    bits->CKE = 1;            // {CKP=0, CKE=1} = SPI Mode 0 (CPOL=0, CPHA=0) in PIC32 convention
    bits->SMP = 0;
    bits->MODE16 = 0;
    bits->MODE32 = 0;         // 8-bit mode
    bits->ON = 1;
}

static uint8_t spi1_xfer(uint8_t out) {
    while (SPI1STATbits.SPITBF) { ; }
    SPI1BUF = out;
    while (!SPI1STATbits.SPIRBF) { ; }
    return (uint8_t)SPI1BUF;
}

static uint8_t spi2_xfer(uint8_t out) {
    while (SPI2STATbits.SPITBF) { ; }
    SPI2BUF = out;
    while (!SPI2STATbits.SPIRBF) { ; }
    return (uint8_t)SPI2BUF;
}

static uint8_t spi3_xfer(uint8_t out) {
    while (SPI3STATbits.SPITBF) { ; }
    SPI3BUF = out;
    while (!SPI3STATbits.SPIRBF) { ; }
    return (uint8_t)SPI3BUF;
}

int main(void) {
    /* ---- GPIO directions ---- */
    // Debug/status
    TRISEbits.TRISE9 = 0;   LATEbits.LATE9 = 0;     // DEBUG-LED net (RE9) -- not populated on this board, harmless to drive

    /* ANSELx resets to all-analog on every pin that has an analog function,
     * which disables that pin's digital input buffer. RB14/RB15/RC1/RC2/RC4/RG6/
     * RD14/RD15 (SPI2 + SPI3's SCK/CS/MISO/MOSI pins) all double as analog
     * channels (AN9/AN10/AN22/AN21/AN19/AN14/AN32/AN33) and were never switched
     * to digital here -- without this, SPI2/SPI3 can never read a valid level
     * back from either sensor no matter what the bus is doing electrically.
     * RD1/RD2/RD3/RD12 (SPI1, the magnetometer) have no analog function on
     * this device, so this fix does not touch them. */
    ANSELBbits.ANSB14 = 0;  // SCK3 (IMU)
    ANSELBbits.ANSB15 = 0;  // IMU-CS
    ANSELCbits.ANSC1  = 0;  // SDO2 (MRAM-MOSI)
    ANSELCbits.ANSC2  = 0;  // MRAM-CS
    ANSELCbits.ANSC4  = 0;  // SDI2 (MRAM-MISO)
    ANSELGbits.ANSG6  = 0;  // SCK2 (MRAM)
    ANSELDbits.ANSD14 = 0;  // SDI3 (IMU-MISO)
    ANSELDbits.ANSD15 = 0;  // SDO3 (IMU-MOSI)

    // Switched peripheral rail enables (through the shared load-switch/current-sense cell)
    TRISDbits.TRISD4 = 0;   LATDbits.LATD4 = 0;     // MGN-EN
    TRISFbits.TRISF3 = 0;   LATFbits.LATF3 = 0;     // IMU-EN
    TRISEbits.TRISE5 = 0;   LATEbits.LATE5 = 0;     // MRAM-EN

    // Chip selects, idle high (deselected)
    TRISDbits.TRISD12 = 0;  LATDbits.LATD12 = 1;    // MGN-CS
    TRISBbits.TRISB15 = 0;  LATBbits.LATB15 = 1;    // IMU-CS
    TRISCbits.TRISC2  = 0;  LATCbits.LATC2  = 1;    // MRAM-CS

    // MRAM housekeeping pins (active-low): hold them high so neither is asserted
    TRISCbits.TRISC3 = 0;   LATCbits.LATC3 = 1;     // MRAM-HOLD
    TRISEbits.TRISE7 = 0;   LATEbits.LATE7 = 1;     // MRAM-WP

    /* SPI clock/MOSI pins: remapping them via [x]R/RPnR only selects which
     * peripheral signal is routed to the pin -- it does not touch TRISx, so
     * without this they stay at their reset state (input) and the MCU never
     * actually drives SCK or MOSI out to any of the three sensors. */
    TRISDbits.TRISD1  = 0;   // SCK1 (MGN)
    TRISDbits.TRISD3  = 0;   // SDO1 (MGN-MOSI)
    TRISGbits.TRISG6  = 0;   // SCK2 (MRAM)
    TRISCbits.TRISC1  = 0;   // SDO2 (MRAM-MOSI)
    TRISBbits.TRISB14 = 0;   // SCK3 (IMU)
    TRISDbits.TRISD15 = 0;   // SDO3 (IMU-MOSI)

    // MISO pins stay inputs (already the reset default, set explicitly for clarity)
    TRISDbits.TRISD2  = 1;   // SDI1 (MGN-MISO)
    TRISCbits.TRISC4  = 1;   // SDI2 (MRAM-MISO)
    TRISDbits.TRISD14 = 1;   // SDI3 (IMU-MISO)

    /* ---- power up all three sensor rails and let them settle ---- */
    LATDbits.LATD4 = 1;     // MGN-EN
    LATFbits.LATF3 = 1;     // IMU-EN
    LATEbits.LATE5 = 1;     // MRAM-EN
    delay_cycles(200000);

    /* ---- PPS: route SPI1/2/3 data lines to the pins this board actually wires them to ---- */
    pps_unlock();
    SDI1Rbits.SDI1R = 0x0;      // SDI1 <- RPD2  (MGN-MISO)
    RPD3Rbits.RPD3R = 0x5;      // RPD3 -> SDO1  (MGN-MOSI)
    SDI2Rbits.SDI2R = 0xA;      // SDI2 <- RPC4  (MRAM-MISO)
    RPC1Rbits.RPC1R = 0x6;      // RPC1 -> SDO2  (MRAM-MOSI)
    SDI3Rbits.SDI3R = 0xB;      // SDI3 <- RPD14 (IMU-MISO)
    RPD15Rbits.RPD15R = 0x7;    // RPD15 -> SDO3 (IMU-MOSI)
    pps_lock();

    spi_init_master_mode0(&SPI1CON, &SPI1BRG);
    spi_init_master_mode0(&SPI2CON, &SPI2BRG);
    spi_init_master_mode0(&SPI3CON, &SPI3BRG);

    /* ---- Magnetometer: read REVID (0x36), expect 0x22 ---- */
    LATDbits.LATD12 = 0;
    (void)spi1_xfer(0x36 | 0x80);
    g_mgn_revid = spi1_xfer(0x00);
    LATDbits.LATD12 = 1;
    g_mgn_ok = (g_mgn_revid == 0x22) ? 1 : 0;

    /* ---- IMU: read WHO_AM_I (0x0F), expect 0x6B ---- */
    LATBbits.LATB15 = 0;
    (void)spi3_xfer(0x0F | 0x80);
    g_imu_whoami = spi3_xfer(0x00);
    LATBbits.LATB15 = 1;
    g_imu_ok = (g_imu_whoami == 0x6B) ? 1 : 0;

    /* ---- MRAM: WREN -> RDSR (expect WEL=1) -> WRDI -> RDSR (expect WEL=0) ----
     * Never touches the memory array itself, just the internal write-enable latch. */
    LATCbits.LATC2 = 0; (void)spi2_xfer(0x06); LATCbits.LATC2 = 1;          // WREN
    LATCbits.LATC2 = 0; (void)spi2_xfer(0x05); g_mram_sr_set = spi2_xfer(0x00); LATCbits.LATC2 = 1; // RDSR
    LATCbits.LATC2 = 0; (void)spi2_xfer(0x04); LATCbits.LATC2 = 1;          // WRDI
    LATCbits.LATC2 = 0; (void)spi2_xfer(0x05); g_mram_sr_clr = spi2_xfer(0x00); LATCbits.LATC2 = 1; // RDSR
    g_mram_ok = ((g_mram_sr_set & 0x02) && !(g_mram_sr_clr & 0x02)) ? 1 : 0;

    g_test_done = 0xA5A5A5A5;

    while (1) {
        delay_cycles(1200000);
        LATEbits.LATE9 ^= 1;
    }

    return 0;
}
