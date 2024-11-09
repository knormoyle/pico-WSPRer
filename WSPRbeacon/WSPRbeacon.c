/////////////////////////////////////////////////////////////////////////////
//  Majority of code forked from work by
//  Roman Piksaykin [piksaykin@gmail.com], R2BDY
//  https:// www.qrz.com/db/r2bdy
//  PROJECT PAGE
//  https:// github.com/RPiks/pico-WSPR-tx

#include "WSPRbeacon.h"
#include <WSPRutility.h>
#include <maidenhead.h>
#include <math.h>
#include "pico/sleep.h"
#include "hardware/rtc.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"

static char grid5;
static char grid6;
static float altitude_snapshot;
static int rf_pin;
static int U4B_second_packet_has_started = 0;
static int U4B_second_packet_has_started_at_minute;
static int itx_trigger = 0;
static int itx_trigger2 = 0;
static int forced_xmit_in_process = 0;
static int transmitter_status = 0;
static absolute_time_t start_time;
static absolute_time_t time_of_last_serial_packet;
static int current_minute;

// array index is minute, (odd minutes are unused)
// value is -1 for NONE or 1-4 for U4B 1st msg,U4B 2nd msg,
// Zachtek 1st, Zachtek 2nd, and #5 for extended TELEN
static int oneshots[10];
static int schedule[10];

static int at_least_one_slot_has_elapsed;
static int at_least_one_first_packet_sent;
static int at_least_one_GPS_fixed_has_been_obtained;
static uint8_t _callsign_for_TYPE1[12];
static uint8_t altitude_as_power_fine;
static uint32_t previous_msg_count;
static absolute_time_t GPS_aquisiion_time;
static uint32_t minute_OF_GPS_aquisition;
static int tikk;
static int tester;
static uint32_t OLD_GPS_active_status;
const int8_t valid_dbm[19] = {0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40, 43, 47, 50, 53, 57, 60};

static void sleep_callback(void) {
    printf("RTC woke us up\n");
}
/// @brief Initializes a new WSPR beacon context.
/// @param pcallsign HAM radio callsign, 12 chr max.
/// @param pgridsquare Maidenhead locator, 7 chr max.
/// @param txpow_dbm TX power, db`mW.
/// @param pdco Ptr to working DCO.
/// @param dial_freq_hz The begin of working WSPR passband.
/// @param shift_freq_hz The shift of tx freq. relative to dial_freq_hz.
/// @param gpio Pico's GPIO pin of RF output.
/// @return Ptr to the new context.
/// @bug The function sets schedule hardcoded to minutes 1, 2 and 3. It may lead to
/// @bug out of schedule transmits if the device is switched on around minute 0

WSPRbeaconContext *WSPRbeaconInit(const char *pcallsign, const char *pgridsquare, int txpow_dbm,
PioDco *pdco, uint32_t dial_freq_hz, uint32_t shift_freq_hz,
int gpio,  uint8_t start_minute, uint8_t id13, uint8_t suffix, const char *TELEN_config)
{
    assert_(pcallsign);
    assert_(pgridsquare);
    assert_(pdco);
    WSPRbeaconContext *p = calloc(1, sizeof(WSPRbeaconContext));
    assert_(p);

    rf_pin=gpio; // save the value of the (base) RF pin for enabling/disabling them later
    strncpy(p->_pu8_callsign, pcallsign, sizeof(p->_pu8_callsign));
    strncpy(p->_pu8_locator, pgridsquare, sizeof(p->_pu8_locator));

    p->_u8_txpower = txpow_dbm;
    p->_pTX = TxChannelInit(682667, 0, pdco);
    assert_(p->_pTX);
    p->_pTX->_u32_dialfreqhz = dial_freq_hz + shift_freq_hz;
    p->_pTX->_i_tx_gpio = gpio;

    srand(3333);
    at_least_one_slot_has_elapsed=0;OLD_GPS_active_status=0;
    for (int i=0;i < 10;i++) schedule[i]=-1;

    tester=0;
    p->_txSched.minutes_since_boot=0;
    p->_txSched.minutes_since_GPS_aquisition=99999; minute_OF_GPS_aquisition=0;
    /* Following code sets packet types for each timeslot.
    1:U4B 1st msg, 2: U4B 2nd msg, 3: WSPR1 or Zachtek 1st,
    4:Zachtek 2nd,5:extended TELEN #1 6:extended TELEN #2
    */

    // if U4B protocol disabled ('--' entered for Id13),  
    // we will ONLY do Type 1 [and Type3 (zachtek)] at the specified minute
    if (id13==253) {
        if (suffix != 253) {
            // we get here if suffix is not '-', meaning that Zachtek (wspr type 3) message is desired
            schedule[start_minute]=3;
            schedule[(start_minute+2)%10]=4;
        } else {
            // we get here only is both U4B and ZAchtek(suffix) are disabled. 
            // this is for standalone (WSPR Type-1) only beacon mode
            schedule[start_minute]=3;
        }
    }

    // if we get here, U4B is enabled
    else {
        // kevin 11_3_24
        // send on 4 u4b channels (every 4 minutes) so we get faster output for test
        // this can be rough since no time left to recover 3dfx if RF interference with WSPR
        // not sure pcallsign is always null terminated?
        // AD6Z sizeof will be 4 if not null terminated?
        // strlen will walk looking for null term. undefined if no null term?
        printf("pcallsign '%s' strlen %d\n", pcallsign, strlen(pcallsign));

        if (0==strcmp(pcallsign, "TE4MIN")) {
            // doesn't depend on null term
            printf("TE4MIN starting 2 minute spray across 5 u4b channels)\n");
            for (int i=0;i < 10;i+2) {
                schedule[i]=1;      // do 1st U4b packet
                schedule[(i+1)]=2;   // do second U4B packet 2 minutes later
            }
        }
        // not sure pcallsign is always null terminated? it should be.
        // FIX! what if it just starts with AD6Z?
        // else if (pcallsign == "AD6Z") // reserve two u4b like it's two balloons.
        else if ( 0!=(
            strcmp(pcallsign, "AD6Z") || strcmp(pcallsign, "R2MAD") || strcmp(pcallsign, "R4MAD") ||
            strcmp(pcallsign, "R8MAD") || strcmp(pcallsign, "R12MAD") )) {

            printf("%s starting minute spray double channel\n", pcallsign);
            schedule[start_minute]=1;          // do 1st U4b packet at selected minute
            schedule[(start_minute+2)%10]=2;   // do second U4B packet 2 minutes later
            // this will be like doing two telen, but with normal transmissions
            // will the gps stay locked?
            schedule[(start_minute+4)%10]=1;   // same but for (..+6)%10 starting minute channel
            schedule[(start_minute+6)%10]=2;   // the corresponding telemetry start
            // the 8 and 9 starting minutes will be empty for gps lock
            // we can compare the two channels on traquito web site, and compare
        } else {
            schedule[start_minute]=1;          // do 1st U4b packet at selected minute
            schedule[(start_minute+2)%10]=2;   // do second U4B packet 2 minutes later
        }

        if (TELEN_config[0]!='-') schedule[(start_minute+4)%10]=5; // enable TELEN #1
        // enable TELEN #2 
        // (if someone tried to run Zachtek and Both TELENs, 
        // start_minute+6)%10 will get overwritten below anyway)
        if (TELEN_config[2]!='-') schedule[(start_minute+6)%10]=6; 

        // if Suffix enabled,
        // Do zachtek messages 4 mins BEFORE (ie 6 minutes in future) of u4b 
        // (because minus (-) after char to decimal conversion is 253)
        if (suffix != 253) {
            // if we get here, both U4B and Zachtek (suffix) enabled. hopefully telen not also enabled!
            schedule[(start_minute+6)%10]=3;
            schedule[(start_minute+8)%10]=4;
        }
    }

    at_least_one_GPS_fixed_has_been_obtained=0;
    transmitter_status=0;
    return p;
}

//*******************************************************************************
//*******************************************************************************
/// @brief Arranges WSPR sending in accordance with pre-defined schedule.
/// @brief It works only if GPS receiver available (for now).
/// @param pctx Ptr to Context.
/// @return 0 if OK, -1 if NO GPS received available
// called every half second from Main.c
int WSPRbeaconTxScheduler(WSPRbeaconContext *pctx, int verbose, int GPS_PPS_PIN)   
{
    assert_(pctx);

    // on if there ever were any serial data received from a GPS unit
    uint32_t is_GPS_available = pctx->_pTX->_p_oscillator->_pGPStime->_time_data._u32_nmea_gprmc_count;
    // on if valid 3d fix
    const uint32_t is_GPS_active = pctx->_pTX->_p_oscillator->_pGPStime->_time_data._u8_is_solution_active;

    // for testing
    /*
    for(;;)
    {
        pctx->_txSched.TELEN1_val1=rand() % 630000;
        pctx->_txSched.TELEN1_val2=rand() % 153000;
        pctx->_txSched.TELEN2_val1=rand() % 630000;
        pctx->_txSched.TELEN2_val2=rand() % 153000;

        WSPRbeaconCreatePacket(pctx, 5);
        WSPRbeaconCreatePacket(pctx, 6);
        sleep_ms(2000);
    }
    */

    pctx->_txSched.minutes_since_boot=floor((to_ms_since_boot(get_absolute_time()) / (uint32_t) 60000) );

    // GPS status has changed
    if (OLD_GPS_active_status!=is_GPS_active) {
        OLD_GPS_active_status=is_GPS_active; // make it a oneshot
        // it changed, and is now ON so save time it last went on
        if (is_GPS_active) minute_OF_GPS_aquisition = pctx->_txSched.minutes_since_boot;
    }

    if (is_GPS_active)
        // current time minus time it last went on is MINUTES since on
        pctx->_txSched.minutes_since_GPS_aquisition = pctx->_txSched.minutes_since_boot-minute_OF_GPS_aquisition;
    else
        //9xxx will indicate no GPS, but the xxx will still show time since last aquisition
        pctx->_txSched.minutes_since_GPS_aquisition = (pctx->_txSched.minutes_since_boot-minute_OF_GPS_aquisition)+9000;

    if (is_GPS_active) at_least_one_GPS_fixed_has_been_obtained=1;

    if (pctx->_txSched.force_xmit_for_testing) {
        if (forced_xmit_in_process==0) {
            StampPrintf("> FORCING XMISSION! for debugging   <"); pctx->_txSched.led_mode = 4;
            PioDCOStart(pctx->_pTX->_p_oscillator);
            // If this is disabled, the packet is all zeroes, and it xmits an unmodulated steady frequency.
            // but if you didnt power cycle since enabling Force_xmition will still have data in buffer
            // WSPRbeaconCreatePacket(pctx,0);
            sleep_ms(100);
            WSPRbeaconSendPacket(pctx);
            start_time = get_absolute_time();
            forced_xmit_in_process=1;
        }
        else if (absolute_time_diff_us( start_time, get_absolute_time()) > 120000000ULL) {
            forced_xmit_in_process=0; // restart after 2 mins
            PioDCOStop(pctx->_pTX->_p_oscillator);
            printf("Pio *STOP*  called by end of forced xmit. small pause before restart\n");
            sleep_ms(2000);
        }
        return -1;
    }

    if (!is_GPS_available) {
        if (pctx->_txSched.verbosity>=1)
            StampPrintf(" Waiting for GPS receiver to start communicating, or, serial comms interrupted");
        pctx->_txSched.led_mode = 0;  // waiting for GPS
        return -1;
    }

    if (!is_GPS_active) {
        if (pctx->_txSched.verbosity>=1)
            StampPrintf("Gps was available, but no valid 3d Fix. ledmode %d XMIT status %d",
                pctx->_txSched.led_mode,pctx->_pTX->_p_oscillator->_is_enabled);
    }

    // convert from char to int
    current_minute = pctx->_pTX->_p_oscillator->_pGPStime->_time_data._u8_last_digit_minutes - '0';

    // if the current minute is an odd minute or a non-scheduled minute
    if (schedule[current_minute]==-1) {
        for (int i=0;i < 10;i++) oneshots[i]=0;
        at_least_one_slot_has_elapsed=1;
        // if we want to switch oscillator off and are in non sheduled interval
        if (pctx->_txSched.oscillatorOff && schedule[(current_minute+9)%10]==-1) {
            transmitter_status=0;
            PioDCOStop(pctx->_pTX->_p_oscillator); // Stop the oscilator
        }
    }
    // prevent transmission if a location has never been received
    else if (
        is_GPS_available && at_least_one_slot_has_elapsed
        && schedule[current_minute]>0
        && oneshots[current_minute]==0
        && (at_least_one_GPS_fixed_has_been_obtained!=0) ) {

        oneshots[current_minute]=1;
        if (pctx->_txSched.verbosity>=3)
            printf("\nStarting TX. current minute: %i Schedule Value (packet type): %i\n",
            current_minute,schedule[current_minute]);

        PioDCOStart(pctx->_pTX->_p_oscillator);
        transmitter_status=1;
        // the schedule determines packet type 
        // (1-4 for U4B 1st msg,U4B 2nd msg,Zachtek 1st, Zachtek 2nd)
        WSPRbeaconCreatePacket(pctx, schedule[current_minute] );
        sleep_ms(50);
        WSPRbeaconSendPacket(pctx);

        /*
        if (schedule[current_minute]==2) {
            U4B_second_packet_has_started=1;
            U4B_second_packet_has_started_at_minute=current_minute;
        }
        */

        // wuz, the plus 2 at end is to allow 1 TELEN in low power mode
        if (schedule[current_minute]==2) {
            U4B_second_packet_has_started=1;
            U4B_second_packet_has_started_at_minute=(current_minute+2)%10;
        }

        // 1 - No valid GPS, not transmitting
        // 2 - Valid GPS, waiting for time to transmitt
        // 3 - Valid GPS, transmitting
        // 4 - no valid GPS, but (still) transmitting anyway */
        if (!is_GPS_active && transmitter_status) pctx->_txSched.led_mode = 4;
        else pctx->_txSched.led_mode = 1 + is_GPS_active + transmitter_status;

        if (previous_msg_count!=is_GPS_available) {
            previous_msg_count=is_GPS_available;
            time_of_last_serial_packet= get_absolute_time();
        }

        // if more than one or two serial packets are missed something is wrong
        if (absolute_time_diff_us(time_of_last_serial_packet, get_absolute_time()) > 3000000ULL) {
            pctx->_txSched.led_mode = 0;  // no GPS serial Comms
        }
        // time to sleep to save battery power
        if ( pctx->_txSched.low_power_mode &&
            U4B_second_packet_has_started &&
            (current_minute==((U4B_second_packet_has_started_at_minute+2) % 10)) ) {

            datetime_t t = {.year  = 2020,.month = 01,.day= 01, .dotw= 1,.hour=1,.min= 1,.sec = 00};
            // Start the RTC
            rtc_init();
            rtc_set_datetime(&t);
            uart_default_tx_wait_blocking();
            datetime_t alarm_time = t;
            // sleep for 55 minutes. 46 ~= 55 mins X (115Mhz/133Mhz)  
            //wuz, the -3 is to allow 1 TELEN in low power mode
            alarm_time.min += (46-3);
            // this is needed to disable IRQ callback on PPS
            gpio_set_irq_enabled(GPS_PPS_PIN, GPIO_IRQ_EDGE_RISE, false); 
            multicore_reset_core1();  // this is needed, otherwise causes instant reboot
            // this reduces sleep draw to 2mA! (without this will still sleep, but only at 8mA)
            sleep_run_from_dormant_source(DORMANT_SOURCE_ROSC);
            // blocks here during sleep period
            sleep_goto_sleep_until(&alarm_time, &sleep_callback); 

            watchdog_enable(100, 1);
            for (;;) {}
            // recovering from sleep is messy, this makes it reboot to get a fresh start
        }
        return 0;
    }
}


//*******************************************************************
// 1-6.  1: U4B 1st msg,U4B 2: 2nd msg, 
// 3: Zachtek 1st, 4: Zachtek 2nd 5:U4B telen 1, 6:U4B telen 2
int WSPRbeaconCreatePacket(WSPRbeaconContext *pctx,int packet_type)
{
    /*
    // turns a fan on via GPIO 18 every other packet. 
    // this forces temperature swings for testing TCXO stability
    if(0 == ++tikk % 2)
        gpio_put(18, 1);
    if(0 == (tikk+1) % 2)
        gpio_put(18, 0);
    */

    assert_(pctx);
    //********************************
    if (packet_type==1) {
        // U4B first msg
        pctx->_u8_txpower =10; // hardcoded at 10dbM when doing u4b MSG 1
        if (pctx->_txSched.verbosity>=3) printf("creating U4B packet 1\n");
        char _4_char_version_of_locator[5];
        // only take first 4 chars of locator
        strncpy(_4_char_version_of_locator, pctx->_pu8_locator, 4);     
        _4_char_version_of_locator[4]=0;  // add null terminator

        // look in WSPRutility.c for wspr_encode
        wspr_encode(
            pctx->_pu8_callsign,    
            _4_char_version_of_locator, 
            pctx->_u8_txpower, 
            pctx->_pu8_outbuf, 
            pctx->_txSched.verbosity);

        // record the values of grid chars 5 and 6 now, 
        // but they won't be used until packet type 2 is created
        grid5 = pctx->_pu8_locator[4];
        grid6 = pctx->_pu8_locator[5];
         // save the value for later when used in 2nd packet
        altitude_snapshot=pctx->_pTX->_p_oscillator->_pGPStime->_altitude;
        at_least_one_first_packet_sent=1;
    }
    //********************************
    // special encoding for 2nd packet of U4B protocol
    if (packet_type==2) {
        if (pctx->_txSched.verbosity>=3) printf("creating U4B packet 2 \n");
        char CallsignU4B[7];
        char Grid_U4B[7];
        uint8_t  power_U4B;

        // if a first packet was never created, the snapshots are incorrect. 
        // so put something in there. (issue %46)
        if (at_least_one_first_packet_sent==0) {
            // record the values of grid chars 5 and 6 now, 
            // but they won't be used until packet type 2 is created
            grid5 = pctx->_pu8_locator[4];
            grid6 = pctx->_pu8_locator[5];
            altitude_snapshot=pctx->_pTX->_p_oscillator->_pGPStime->_altitude;
            // so it wont do this next time...
            at_least_one_first_packet_sent==1;
        }

        /* inputs:
        pctx->_pu8_locator (6 char grid)
        pctx->_txSched->temp_in_Celsius
        pctx->_txSched->id13
        pctx->_txSched->voltage
        */

        // pick apart inputs
        // char grid5 = pctx->_pu8_locator[4];  
        // values of grid 5 and 6 were already set previously when packet 1 was created
        // char grid6 = pctx->_pu8_locator[5];
        // convert inputs into components of a big number
        uint8_t grid5Val = grid5 - 'A';
        uint8_t grid6Val = grid6 - 'A';
        uint16_t altFracM =  round((double)altitude_snapshot/ 20);

        // convert inputs into a big number
        uint32_t val = 0;
        val *=   24; val += grid5Val;
        val *=   24; val += grid6Val;
        val *= 1068; val += altFracM;

        // extract into altered dynamic base
        uint8_t id6Val = val % 26; val = val / 26;
        uint8_t id5Val = val % 26; val = val / 26;
        uint8_t id4Val = val % 26; val = val / 26;
        uint8_t id2Val = val % 36; val = val / 36;

        // convert to encoded CallsignU4B
        char id2 = EncodeBase36(id2Val);
        char id4 = 'A' + id4Val;
        char id5 = 'A' + id5Val;
        char id6 = 'A' + id6Val;

        CallsignU4B[0] =  pctx->_txSched.id13[0];   // string{ id13[0], id2, id13[1], id4, id5, id6 };
        CallsignU4B[1] =  id2;
        CallsignU4B[2] =  pctx->_txSched.id13[1];
        CallsignU4B[3] =  id4;
        CallsignU4B[4] =  id5;
        CallsignU4B[5] =  id6;
        CallsignU4B[6] =  0;

        /* inputs:
        pctx->_pu8_locator (6 char grid)
        pctx->_txSched->temp_in_Celsius
        pctx->_txSched->id13
        pctx->_txSched->voltage
        */
        /* outputs :
        char CallsignU4B[6];
        char Grid_U4B[7];
        uint8_t  power_U4B;
        */

        // parse input presentations
        double tempC   = pctx->_txSched.temp_in_Celsius;
        double voltage = pctx->_txSched.voltage;
        // map input presentations onto input radix (numbers within their stated range of possibilities)

        //**************
        // kevin 10_30_24
        // handle possible illegal range (double wrap on tempC?). or should it clamp at the bounds?
        // uint8_t tempCNum      = tempC - -50;
        uint8_t tempCNum      = ((uint8_t) tempC - -50) % 90;
        uint8_t voltageNum    = ((uint8_t)round(((voltage * 100) - 300) / 5) + 20) % 40;
        // encoding # of satelites into knots
        uint8_t speedKnotsNum = pctx->_pTX->_p_oscillator->_pGPStime->_time_data.sat_count;   

        // handle possible illegal (0-41 legal range). clamp to max, not wrap. 
        // maybe from bad GNGGA field (wrong sat count?)
        if (speedKnotsNum > 41) speedKnotsNum = 41;
        //****************

        // kevin old code since this isn't 0 or 1 it should really check zero. 
        // don't want to say valid if dead reckoning fix? (6)
        uint8_t gpsValidNum   = pctx->_pTX->_p_oscillator->_pGPStime->_time_data._u8_is_solution_active;
        // changed sept 27 2024.
        // because the traquito site won't show the 6 char grid if this bit is even momentarily off.
        // Anyway, redundant cause sat count is sent as knots
        gpsValidNum=1;
        // shift inputs into a big number
        val = 0;
        val *= 90; val += tempCNum;
        val *= 40; val += voltageNum;
        val *= 42; val += speedKnotsNum;
        val *=  2; val += gpsValidNum;
        // standard telemetry (1 for the 2nd U4B packet, 0 for "Extended TELEN") - Thanks Kevin!
        val *=  2; val += 1;

        // unshift big number into output radix values
        uint8_t powerVal = val % 19; val = val / 19;
        uint8_t g4Val    = val % 10; val = val / 10;
        uint8_t g3Val    = val % 10; val = val / 10;
        uint8_t g2Val    = val % 18; val = val / 18;
        uint8_t g1Val    = val % 18; val = val / 18;

        // map output radix to presentation
        char g1 = 'A' + g1Val;
        char g2 = 'A' + g2Val;
        char g3 = '0' + g3Val;
        char g4 = '0' + g4Val;

        Grid_U4B[0] = g1; // = string{ g1, g2, g3, g4 };
        Grid_U4B[1] = g2;
        Grid_U4B[2] = g3;
        Grid_U4B[3] = g4;
        Grid_U4B[4] = 0;

        power_U4B=valid_dbm[powerVal];
        wspr_encode(CallsignU4B, Grid_U4B, power_U4B, pctx->_pu8_outbuf,pctx->_txSched.verbosity);
    }

    //********************************
    if (packet_type==3) {
        // WSPR type 1 message (for standalone beacon mode, or 1st part of Zachtek protocol)
        uint8_t suffix_as_string[2];
        // if just using standalone beacon,  power is reported as 10. If doing Zachtek, this gets 
        /// overwritten below with rough altitude value
        uint8_t  power_value=10;
        if (pctx->_txSched.verbosity>=3) printf("creating WSPR type 1 [Zachtek packet 1]\n");

        // if standalone beacon mode (suffix was enterred as '-' (253)
        if (pctx->_txSched.suffix==253) {
            strcpy(_callsign_for_TYPE1,pctx->_pu8_callsign);
            strcat(_callsign_for_TYPE1,0); // add null terminate
        }
        // if doing Zachtek (as opposed to just standalone beacon)
        // do all the stuff below to append suffix and encode power.
        // my software does not allow Zachtek without suffix for now.
        else {
            // unless overwritten below when using a suffix and zachtek, hardcodes power at 10 for Bruce
            power_value=10;
            // this gets called with or without suffix
            strcpy(_callsign_for_TYPE1,pctx->_pu8_callsign);
            // dont append suffix if its an 'X' (thats option to disable suffix). X=88. 88-48('0') = 40
            if (pctx->_txSched.suffix!=40) {
                strcat(_callsign_for_TYPE1,"/");
                suffix_as_string[0]=pctx->_txSched.suffix+48;
                suffix_as_string[1]=0;
                strcat(_callsign_for_TYPE1,suffix_as_string);
                power_value=0;

                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>900) power_value=3;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>2100) power_value=7;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>3000) power_value=10;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>3900) power_value=13;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>5100) power_value=17;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>6000) power_value=20;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>6900) power_value=23;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>8100) power_value=27;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>9000) power_value=30;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>9900) power_value=33;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>11100) power_value=37;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>12000) power_value=40;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>12900) power_value=43;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>14100) power_value=47;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>15000) power_value=50;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>15900) power_value=53;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>17100) power_value=57;
                if (pctx->_pTX->_p_oscillator->_pGPStime->_altitude>18000) power_value=60;

                float fine_altitude = pctx->_pTX->_p_oscillator->_pGPStime->_altitude - (power_value*300.0f);
                altitude_as_power_fine=0;
                if (fine_altitude>60) altitude_as_power_fine=3;
                if (fine_altitude>140) altitude_as_power_fine=7;
                if (fine_altitude>200) altitude_as_power_fine=10;
                if (fine_altitude>260) altitude_as_power_fine=13;
                if (fine_altitude>340) altitude_as_power_fine=17;
                if (fine_altitude>400) altitude_as_power_fine=20;
                if (fine_altitude>460) altitude_as_power_fine=23;
                if (fine_altitude>540) altitude_as_power_fine=27;
                if (fine_altitude>600) altitude_as_power_fine=30;
                if (fine_altitude>660) altitude_as_power_fine=33;
                if (fine_altitude>740) altitude_as_power_fine=37;
                if (fine_altitude>800) altitude_as_power_fine=40;
                if (fine_altitude>860) altitude_as_power_fine=43;
                if (fine_altitude>940) altitude_as_power_fine=47;
                if (fine_altitude>1000) altitude_as_power_fine=50;
                if (fine_altitude>1060) altitude_as_power_fine=53;
                if (fine_altitude>1140) altitude_as_power_fine=57;
                if (fine_altitude>1200) altitude_as_power_fine=60;

                if (pctx->_txSched.verbosity>=3)
                    printf("Raw altitude: %0.3f rough: %d fine: %d\n",
                        pctx->_pTX->_p_oscillator->_pGPStime->_altitude,power_value,altitude_as_power_fine);
            }
        }
        wspr_encode(
            _callsign_for_TYPE1, 
            pctx->_pu8_locator, 
            power_value, 
            pctx->_pu8_outbuf,
            pctx->_txSched.verbosity);
    }

    //2nd Zachtek (WSPR type 3 message)
    if (packet_type==4) {
        if (pctx->_txSched.verbosity>=3) printf("creating Zachtek packet 2 (WSPR type 3)\n");
        wspr_encode(
            add_brackets(_callsign_for_TYPE1), 
            pctx->_pu8_locator, 
            altitude_as_power_fine, 
            pctx->_pu8_outbuf,
            pctx->_txSched.verbosity);
    }

    // TELEN #1 or #2 extended telemetry, gets sent right after the two U4B packets
    if ((packet_type==5) || (packet_type==6)) {
        if (pctx->_txSched.verbosity>=3) printf("creating TELEN packet 1\n");
        char _4_char_version_of_locator[5];
        char _callsign[7];
        uint32_t telen_val1;
        uint32_t telen_val2;

        if (packet_type==5) { // TELEN #1
            // gets values for TELEN from global vars
            telen_val1=pctx->_txSched.TELEN1_val1;
            telen_val2=pctx->_txSched.TELEN1_val2;
            printf("encoding TELEN #1\n");
        } else { // TELEN #2
            // gets values for TELEN from global vars
            telen_val1=pctx->_txSched.TELEN2_val1;
            telen_val2=pctx->_txSched.TELEN2_val2;
            printf("encoding TELEN #2\n");
        }

        char telen_chars[8];
        uint8_t telen_power;

        /*
        i made two ways of encoding telen (encode_telen and encode_telen2).
        they both produce identical results.
        the first one I painfully made by reverse engineering results.
        the 2nd way was after I realized that most of the logic for packet type 2 can be re-used.
        tbh I don't really understand either way...
        */

        // converts two 32bit ints into 8 characters and one byte to be transmitted
        // encode_telen(telen_val1,telen_val2,telen_chars, &telen_power,packet_type);

        // converts two 32bit ints into 8 characters and one byte to be transmitted
        encode_telen2(telen_val1,telen_val2,telen_chars, &telen_power,packet_type);

        // callsign: id13[0], telen char0, id13[1], telen char1, telen char2, telen char3
        _callsign[0] =  pctx->_txSched.id13[0];   
        _callsign[1] =  telen_chars[0];
        _callsign[2] =  pctx->_txSched.id13[1];
        _callsign[3] =  telen_chars[1];
        _callsign[4] =  telen_chars[2];
        _callsign[5] =  telen_chars[3];
        _4_char_version_of_locator[0]=telen_chars[4];
        _4_char_version_of_locator[1]=telen_chars[5];
        _4_char_version_of_locator[2]=telen_chars[6];
        _4_char_version_of_locator[3]=telen_chars[7];
        _4_char_version_of_locator[4]=0;  // add null terminator

        // look in WSPRutility.c for wspr_encode
        wspr_encode(
            _callsign, 
            _4_char_version_of_locator, 
            telen_power, 
            pctx->_pu8_outbuf, 
            pctx->_txSched.verbosity);
    }
    return 0;
}



//*********************************************************************
/// @brief Sends a prepared WSPR packet using TxChannel.
/// @param pctx Context.
/// @return 0, if OK.
//*********************************************************************
int WSPRbeaconSendPacket(const WSPRbeaconContext *pctx)
{
    assert_(pctx);
    assert_(pctx->_pTX);
    assert_(pctx->_pTX->_u32_dialfreqhz > 500 * kHz);
    TxChannelClear(pctx->_pTX);
    memcpy(pctx->_pTX->_pbyte_buffer, pctx->_pu8_outbuf, WSPR_SYMBOL_COUNT);  //162
    pctx->_pTX->_ix_input = WSPR_SYMBOL_COUNT;  // set count of bytes to send
    return 0;
}

//*********************************************************************
/// @brief encodes data for extended telemetry (TELEN)
/// @param telen_val1,telen_val2: the values to encode, telen_chars: 
// the output characters, telen_power: output power (in dbm)
//**********************************************************************
void encode_telen(uint32_t telen_val1, uint32_t telen_val2, char * telen_chars,
        uint8_t * telen_power, uint8_t packet_type) {
    /* i made two ways of encoding telen (encode_telen and encode_telen2).
    they both produce identical results.
    the first one I painfully made by reverse engineering results.
    the 2nd way was after I realized that most of the logic for packet type 2 can be re-used.
    tbh I don't really understand either way...
    */

    // TELEN packet  which has value 1 and value 2 (this same routine used for both telen#1 and telen#2).
    // first value  gets encoded into the callsign (1st char is alphannumeric, and last three chars are alpha).
    // Full callsign will be ID1, telen_char[0], ID3, telen_CHar[1],  telen_CHar[2], telen_CHar[3].
    // 2nd value gets encoded into GRID and power. grid = telen_CHar[4,5,6,7]. power = telen_power

    // max val of 1st one ~= 632k (per dave) [19 bits]
    // i had originally thought 651,013 (if first char Z, which ~=35, times 17565(26^3).
    // base 26 used, not base 36, because other chars must be only alpha, not alphanumeric
    // because of Ham callsign conventions)
    // max val of 2nd one ~= 153k  (per dave) [17 bits] i had thought over 200k....

    uint32_t tem=telen_val1;
    double temf=telen_val2;
    telen_chars[0]= '0'+floor(tem / 17576); tem-=(telen_chars[0]-'0')*17576;

    // shift up from numeric to alpha (there are 7 ascii codes between 9 and A)
    if (telen_chars[0] > '9') telen_chars[0]+=7;
    telen_chars[1]= 'A'+floor(tem / 676); tem-=(telen_chars[1]-'A')*676;
    telen_chars[2]= 'A'+floor(tem / 26); tem-=(telen_chars[2]-'A')*26;
    telen_chars[3]= 'A'+tem;
    telen_chars[4]= 'A'+floor(temf / 8550); temf-=(telen_chars[4]-'A')*8550;
    telen_chars[5]= 'A'+floor(temf / 475); temf-=(telen_chars[5]-'A')*475;
    telen_chars[6]= '0'+floor(temf / 47.5); temf-=(telen_chars[6]-'0')*47.5;
    telen_chars[7]= '0'+floor(temf / 4.75); temf-=((telen_chars[7]-'0')*4.75);
    int i=round(temf/0.25);  // there are 19 possible dbm values. And 4.75/19=0.25

    // The original (AND BROKEN) way.  
    // you kept thinking of certain bits being resevered for certain functions, 
    // but on the decode end they are expecting you to add, not set, so you must play along...
    /*
        i &= ~(1<<0); // clear lowest bit aka gps-sat (always for TELEN #1 and #2)
        if (packet_type==6)
            i |= (1<<1); // set 2nd bit (GPSValid)     for TELEN #2
        else
            i &= ~(1<<1); // clear 2nd bit (GPSValid)  for TELEN #1
    */

    // add 2 (aka the gps-sat bit) for telen #2 only (the new, correct, way...)
    if (packet_type==6) i=i+2;
    *telen_power = valid_dbm[i];
    printf("(Orig) val1: %d val2: %d the chars: %s the power:(as dBm: %d)\n",
        telen_val1,telen_val2,telen_chars,*telen_power);
    telen_chars[8]=0; // null terminate
}

//********************************************************************
void encode_telen2(uint32_t telen_val1,uint32_t telen_val2,char * telen_chars,uint8_t * telen_power, uint8_t packet_type) {
    uint32_t val = telen_val1;
    // extract into altered dynamic base
    uint8_t id6Val = val % 26; val = val / 26;
    uint8_t id5Val = val % 26; val = val / 26;
    uint8_t id4Val = val % 26; val = val / 26;
    uint8_t id2Val = val % 36; val = val / 36;
    // convert to encoded CallsignU4B
    telen_chars[0] = EncodeBase36(id2Val);
    telen_chars[1] = 'A' + id4Val;
    telen_chars[2] = 'A' + id5Val;
    telen_chars[3] = 'A' + id6Val;

    val = telen_val2*4;  //(bitshift to the left twice to make room for gps bits at end)
    // unshift big number into output radix values
    uint8_t powerVal = val % 19; val = val / 19;
    uint8_t g4Val    = val % 10; val = val / 10;
    uint8_t g3Val    = val % 10; val = val / 10;
    uint8_t g2Val    = val % 18; val = val / 18;
    uint8_t g1Val    = val % 18; val = val / 18;

    // map output radix to presentation
    telen_chars[4] = 'A' + g1Val;
    telen_chars[5] = 'A' + g2Val;
    telen_chars[6] = '0' + g3Val;
    telen_chars[7] = '0' + g4Val;

    // identifies it as the 2nd extended TELEN packet.
    // (this is the GPS-valid bit. note for extended TELEN we did NOT set the gps-sat bit)
    if (packet_type==6) powerVal=powerVal+2;
    *telen_power=valid_dbm[powerVal];

    printf("(New)val1: %d val2: %d the chars: %s the power:(as dBm: %d)\n",
        telen_val1,telen_val2,telen_chars,*telen_power);
    telen_chars[8]=0; // null terminate
}

/// @brief Dumps the beacon context to stdio.
/// @param pctx Ptr to Context.
// called ~ every 20 secs from main.c 
void WSPRbeaconDumpContext(const WSPRbeaconContext *pctx) {
    assert_(pctx);
    assert_(pctx->_pTX);

    const uint64_t u64tmnow = GetUptime64();

    StampPrintf("__________________");
    /* StampPrintf("=TxChannelContext=");
    StampPrintf("ftc:%llu", pctx->_pTX->_tm_future_call);
    StampPrintf("ixi:%u", pctx->_pTX->_ix_input);
    StampPrintf("dfq:%lu", pctx->_pTX->_u32_dialfreqhz);
    StampPrintf("gpo:%u", pctx->_pTX->_i_tx_gpio);   */

    GPStimeContext *pGPS = pctx->_pTX->_p_oscillator->_pGPStime;

    /*const uint32_t u32_unixtime_now = pctx->_pTX->_p_oscillator->_pGPStime->_time_data._u32_utime_nmea_last + u64_GPS_last_age_sec;
    assert_(pGPS);
    StampPrintf("=GPStimeContext=");
    StampPrintf("err:%ld", pGPS->_i32_error_count);
    StampPrintf("ixw:%lu", pGPS->_u8_ixw);
    StampPrintf("sol:%u", pGPS->_time_data._u8_is_solution_active);
    StampPrintf("unl:%lu", pGPS->_time_data._u32_utime_nmea_last);
    StampPrintf("snl:%llu", pGPS->_time_data._u64_sysclk_nmea_last);
    StampPrintf("age:%llu", u64_GPS_last_age_sec);
    StampPrintf("utm:%lu", u32_unixtime_now);
    StampPrintf("rmc:%lu", pGPS->_time_data._u32_nmea_gprmc_count); */
    StampPrintf("ppb:%lld", pGPS->_time_data._i32_freq_shift_ppb);
    StampPrintf("LED Mode: %d",pctx->_txSched.led_mode);
    StampPrintf("Grid: %s", (char *)WSPRbeaconGetLastQTHLocator(pctx));
    StampPrintf("lat: %lli", pctx->_pTX->_p_oscillator->_pGPStime->_time_data._i64_lat_100k);
    StampPrintf("lon: %lli", pctx->_pTX->_p_oscillator->_pGPStime->_time_data._i64_lon_100k);
    StampPrintf("altitude: %f", pctx->_pTX->_p_oscillator->_pGPStime->_altitude);
    StampPrintf("current minute: %i", current_minute);
}

/// @brief Extracts maidenhead type QTH locator (such as KO85) using GPS coords.
/// @param pctx Ptr to WSPR beacon context.
/// @return ptr to string of QTH locator (static duration object inside get_mh).
/// @remark It uses third-party project https:// github.com/sp6q/maidenhead .
char *WSPRbeaconGetLastQTHLocator(const WSPRbeaconContext *pctx)
{
    assert_(pctx);
    assert_(pctx->_pTX);
    assert_(pctx->_pTX->_p_oscillator);
    assert_(pctx->_pTX->_p_oscillator->_pGPStime);

    // Roman's original code used 1e-5 instead (bug)
    double lat = 1e-7 * (double)pctx->_pTX->_p_oscillator->_pGPStime->_time_data._i64_lat_100k;
    // Roman's original code used 1e-5 instead (bug)
    double lon = 1e-7 * (double)pctx->_pTX->_p_oscillator->_pGPStime->_time_data._i64_lon_100k;

    // DEBUGGING to simulate motion
    /*
    lon+=(double)0.3 + (0.03*(double)pctx->_txSched.minutes_since_boot);
    lon+=(double)1.2;
    lon+=(double)0.01*(double)pctx->_txSched.minutes_since_boot;
    */
    return get_mh(lat, lon, 6);
}

uint8_t WSPRbeaconIsGPSsolutionActive(const WSPRbeaconContext *pctx) {
    assert_(pctx);
    assert_(pctx->_pTX);
    assert_(pctx->_pTX->_p_oscillator);
    assert_(pctx->_pTX->_p_oscillator->_pGPStime);

    return YES == pctx->_pTX->_p_oscillator->_pGPStime->_time_data._u8_is_solution_active;
}

// used for Zachtek style
// adds <> around the callsign. this is what triggers a type 3 message.
char* add_brackets(const char * call) {
    static char temp_holder[20];
    temp_holder[0]=0;
    char first_brack[2]= "<";
    char second_brack[2]= ">";
    strcat(temp_holder, first_brack);
    strcat(temp_holder, call);
    strcat(temp_holder, second_brack);
    return temp_holder;
}

char EncodeBase36(uint8_t val) {
    char retVal;
    if (val < 10) retVal = '0' + val;
    else retVal = 'A' + (val - 10);
    return retVal;
}

/// @brief Sets dial (baseband minima) freq.
/// @param pctx Context.
/// @param freq_hz the freq., Hz.
void WSPRbeaconSetDialFreq(WSPRbeaconContext *pctx, uint32_t freq_hz) {
    assert_(pctx);
    pctx->_pTX->_u32_dialfreqhz = freq_hz;
}

/// @brief Constructs a new WSPR packet using the data available.
/// @param pctx Context
/// @return 0 if OK.
