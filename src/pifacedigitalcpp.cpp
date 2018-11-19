/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PiFaceWrapper.cpp
 * Author: julian
 * 
 * Created on 9. November 2018, 14:25
 */
#include "pifacedigitalcpp.h"
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <mcp23s17.h>

int PiFaceDigital::mcp23s17_fd; 
int PiFaceDigital::pfd_count;
     
// Standard constructor, using which interrupts are disabled, hw adress 0 will be used.
PiFaceDigital::PiFaceDigital() {
    PiFaceDigital(0, 0);
}

PiFaceDigital::PiFaceDigital(int _hw_addr, int _interrupts_enabled) {
    hw_addr     =   _hw_addr;
    file_handle =   open();
    
    // Opening connection
    open();
    interrupts_enabled = enable_interrupts();
    
}

PiFaceDigital::PiFaceDigital(const PiFaceDigital& orig) {
}

PiFaceDigital::~PiFaceDigital() {
    close(hw_addr);
}

int PiFaceDigital::open_noinit(uint8_t hw_addr)
{
    // if initialising the first PiFace Digital, open the fd
    if (pfd_count <= 0) {
        if ((mcp23s17_fd = mcp23s17_open(bus, chip_select)) < 0) {
            fprintf(stderr,
                    "pifacedigital_open_noinit: ERROR Could not open MCP23S17 "
                    "device.");
            return -1;
        }
    }
    pfd_count++;
    return  mcp23s17_fd; // returns the fd, for advanced users.
}

int PiFaceDigital::open(){
      if ((mcp23s17_fd = open_noinit(hw_addr)) < 0) {
        fprintf(stderr,
                "pifacedigital_open: ERROR Could not open MCP23S17 device.");
        return -1;
      }
      
       // set up config register
    const uint8_t ioconfig = BANK_OFF | \
                             INT_MIRROR_OFF | \
                             SEQOP_OFF | \
                             DISSLW_OFF | \
                             HAEN_ON | \
                             ODR_OFF | \
                             INTPOL_LOW;
    
    mcp23s17_write_reg(ioconfig, IOCON, hw_addr, mcp23s17_fd);

    // I/O direction
    mcp23s17_write_reg(0x00, IODIRA, hw_addr, mcp23s17_fd);
    mcp23s17_write_reg(0xff, IODIRB, hw_addr, mcp23s17_fd);

    // GPIOB pull ups
    mcp23s17_write_reg(0xff, GPPUB, hw_addr, mcp23s17_fd);

    // enable interrupts
    mcp23s17_write_reg(0xff, GPINTENB, hw_addr, mcp23s17_fd);

    return mcp23s17_fd; // returns the fd, for advanced users.
}

void PiFaceDigital::close(uint8_t hw_addr)
{
    fprintf(stderr, "Close called");
    if (pfd_count <= 0)
        return;

    pfd_count--;

    // disable interrupts if enabled
    const uint8_t intenb = mcp23s17_read_reg(GPINTENB, hw_addr, mcp23s17_fd);
    if (intenb) {
        mcp23s17_write_reg(0, GPINTENB, hw_addr, mcp23s17_fd);
        // now do some other interrupt stuff...
        // TODO
    }

    // if no more PiFace Digital's, close the fd
    if (pfd_count <= 0) {
        pfd_count = 0;
        close(mcp23s17_fd);
    }
}


uint8_t PiFaceDigital::read_reg(uint8_t reg, uint8_t hw_addr)
{
    return mcp23s17_read_reg(reg, hw_addr, mcp23s17_fd);
}

void PiFaceDigital::write_reg(uint8_t data, uint8_t reg, uint8_t hw_addr)
{
    mcp23s17_write_reg(data, reg, hw_addr, mcp23s17_fd);
}

uint8_t PiFaceDigital::read_bit(uint8_t bit_num, uint8_t reg, uint8_t hw_addr)
{
    return mcp23s17_read_bit(bit_num, reg, hw_addr, mcp23s17_fd);
}

void PiFaceDigital::write_bit(uint8_t data,
                             uint8_t bit_num,
                             uint8_t reg,
                             uint8_t hw_addr)
{
    mcp23s17_write_bit(data, bit_num, reg, hw_addr, mcp23s17_fd);
}


uint8_t PiFaceDigital::digital_read(uint8_t pin_num)
{
    return read_bit(pin_num, PiFaceDigital::IN, hw_addr);
}

uint8_t PiFaceDigital::digital_read_inv(uint8_t pin_num)
{
    return ~read_bit(pin_num, PiFaceDigital::IN, hw_addr)&1;
}




void PiFaceDigital::digital_write(uint8_t pin_num, uint8_t value)
{
    write_bit(value, pin_num, PiFaceDigital::OUT, hw_addr);
}

int PiFaceDigital::enable_interrupts()
{
  // returns 
   interrupts_enabled =  !mcp23s17_enable_interrupts();
   return interrupts_enabled;
}

int PiFaceDigital::disable_interrupts()
{
   return mcp23s17_disable_interrupts();
}

int PiFaceDigital::wait_for_input(uint8_t *data,
                                 int timeout,
                                 uint8_t hw_addr)
{
    // Flush any pending interrupts prior to wait
    read_reg(0x11, hw_addr);

    // Wait for input state change
    int ret = mcp23s17_wait_for_interrupt(timeout);

    // Read & return input register, thus clearing interrupt
    *data = read_reg(0x11, hw_addr);
    return ret;
}

int PiFaceDigital::interrupts_enabledi(){
    return interrupts_enabled;
}