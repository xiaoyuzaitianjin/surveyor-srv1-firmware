/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  main.c - main control loop for SRV-1 robot
 *    Copyright (C) 2005-2009  Surveyor Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details (www.gnu.org/licenses)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "srv.h"
#include "print.h"
#include "spi.h"
#include "string.h"
#include "myfunc.h"
extern int picoc(char *);

int main() {
    unsigned char ch;
    int ix;

    init_heap();
    init_io(); // Initialise LED, GPIO, serial flow & lasers.
    initRTC();
    initTMR4();
    init_uart0();
    //init_uart1();  // use UART1 with GPS until GPIO soft-UART is ready
    init_colors();
    init_sonar();
    disable_failsafe();
    serial_out_version();
    clear_sdram(); // Clears from 0x00100000 to 0x02000000
    camera_setup(); // Sets up the camera to 320x256
    led1_on();

    while (1) {
        if (getchar(&ch)) {
            switch (ch) {
                case 'I':
                    grab_frame();
                    send_frame();
                    break;
                case 'y':
                    invert_video();
                    break;
                case 'Y':
                    restore_video();
                    break;
                case 'o':   // turn on overlay_flag
                    overlay_on();
                    break;
                case 'O':   // turn off overlay_flag
                    overlay_off();
                    break;
                case 'a':   // 160 x 120
                    camera_reset(160);
                    break;
                case 'b':   // 320 x 240
                    camera_reset(320);
                    break;
                case 'c':   // 640 x 480
                    camera_reset(640);
                    break;
                case 'd':   // 1280 x 1024
                case 'A':   // 1280 x 1024
                    camera_reset(1280);
                    break;
                case 'V':   // send version string
                    serial_out_version();
                    break;
                case 'D':   // check battery
                    check_battery();
                    break;
                case 'R':   // show laser range
                    show_laser_range(0);
                    break;
                case 'r':   // show laser range with debug
                    show_laser_range(1);
                    break;
                case 'q':  // change image quality
                    change_image_quality();
                    break;
                case 'X':  // xmodem receive
                    xmodem_receive();
                    break;
                case 'Q': // execute C program from flash buffer
                    picoc((char *)FLASH_BUFFER);
                    break;
                case '$': // prototype zone
                    switch (getch()) {
                        case '!':  // reset processor
                            #ifdef SRV1X
                            SRV1X_reboot();
                            #endif
                            reset_cpu();
                        case 'X':
                            svs_master((unsigned short *)FLASH_BUFFER, 131072);
                            break;
                        case 'R':
                            svs_slave((unsigned short *)FLASH_BUFFER, 131072);
                            break;
                        case 'g':  // gps test
                            printf("gps test\n\r");
                            while (!getchar(&ch)) {
                                if (uart1GetChar(&ch))
                                    putchar(ch);
                            }
                            break;
                    }
                    break;
                case '%': // jump to user defined functions in myfunc.c
                    myfunc();
                    break;
                case 'E': // launch flash buffer editor
                    launch_editor();
                    break;
                case 't':
                    serial_out_time();
                    break;
                case 'T':
                    set_edge_thresh();
                    break;
                case 'z':  // read, write or dump a flash sector
                    switch (getch()) {
                        case 'r':
                              // read user flash sector into flash buffer
                            read_user_flash();
                            break;
                        case 'R':
                              // read user sector 00 - 63  to flash buffer
                            ix = ((getch() & 0x0F) * 10) + (getch() & 0x0F);
                            read_user_sector(ix);
                            break;
                        case 'w':
                              // write user flash sector from flash buffer
                            write_user_flash();
                            break;
                        case 'W':
                              // write user sector 00 - 63  from flash buffer
                            ix = ((getch() & 0x0F) * 10) + (getch() & 0x0F);
                            write_user_sector(ix);
                            break;
                        case 'Z':
                              // write boot flash sectors (1-2) from flash buffer
                            write_boot_flash();
                            break;
                        case 'd':
                              // dump flash buffer contents to console
                            serial_out_flashbuffer();
                            break;
                        case 'c':
                              // clear flash buffer
                            clear_flash_buffer();
                            break;
                        case 'C':
                              // compute flash buffer checksum
                            crc_flash_buffer();
                            break;
                    }
                    break;
                case 'p':   // ping sonar - supports up to 4 transducers
                    ping_sonar();
                    break;
                case 'S':   // grab 2 character servo 2/3 command string (left, right)
                    ppm1_command();
                    break;
                case 's':   // grab 2 character servo 6/7 command string (left, right)
                    ppm2_command();
                    break;
                case 'M':   // grab 3 character motor command string (left, right, delay)
                    motor_command();
                    break;
                case '+':   // increase base motor speed
                    motor_increase_base_speed();
                    break;
                case '-':   // decrease base motor speed
                    motor_decrease_base_speed();
                    break;
                case '<':   // bias motor drive to left
                    motor_trim_left();
                    break;
                case '>':   // bias motor drive to right
                    motor_trim_right();
                    break;
                case '7':   // drift left
                case '8':   // forward
                case '9':   // drift right
                case '4':   // turn left
                case '5':   // stop
                case '6':   // turn right
                case '1':   // back left
                case '2':   // back
                case '3':   // back right
                case '0':   // counter clockwise turn
                case '.':   // clockwise turn
                    motor_action(ch);
                    break;
                case 'l':   // lasers on
                    lasers_on();
                    break; 
                case 'L':   // lasers off
                    lasers_off();
                    break; 
                case 'v':   // vision commands
                    process_colors();
                    break;
                case 'n':   // neural net commands
                    process_neuralnet();
                    break;
                case 'F':   // set failsafe mode
                    enable_failsafe();
                    break;
                case 'f':   // clear failsafe mode
                    disable_failsafe();
                    break;
                case 'g':   // g0 = grab a reference frame and enable differencing
                            // g1 = enable color segmentation
                    ch = getch();
                    if (ch == '0')
                        enable_frame_diff();
                    if (ch == '1')
                        enable_segmentation();
                    if (ch == '2')
                        enable_edge_detect();
                    break;
                case 'G':   // disable frame differencing and color segmentation
                    disable_frame_diff();
                    break;
                case 'i':   // i2c read / write
                    process_i2c();
                    break;
                case 'h':   // set serial port to high speed
                    init_fast_uart0();
                    break;
            }
            reset_failsafe_clock();
            while (getchar(&ch)) // flush recv buffer
                continue;
        }
        check_failsafe();
        //check_trim();
        led0_on();
    }
}


