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

int PiFaceDigital::_mcp23s17_fd; 
int PiFaceDigital::pfd_count;
bool PiFaceDigital::EXITVAL;   

// Standard constructor, using which interrupts are disabled, hw adress 0 will be used.
//PiFaceDigital::PiFaceDigital() {
//    PiFaceDigital(0, 0);
//}

PiFaceDigital::PiFaceDigital() {
    PiFaceDigital(0,0, EXITVAL_ZERO);
}

PiFaceDigital::PiFaceDigital(int hw_addr = 0, int _interrupts_enabled = 1, bool _EXITVAL = EXITVAL_ZERO ) {
    _hw_addr     =   hw_addr;
    file_handle =   open();
    PiFaceDigital::EXITVAL = _EXITVAL; 
    // Opening connection
    _init_success = !(file_handle < 0);
    _interrupts_enabled = enable_interrupts();
}

PiFaceDigital::PiFaceDigital(const PiFaceDigital& orig) {
    _hw_addr = orig._hw_addr;
    file_handle = orig.file_handle;
    _init_success = orig._init_success;
    _interrupts_enabled = orig._interrupts_enabled;
}

PiFaceDigital::~PiFaceDigital() {
    if(PiFaceDigital::EXITVAL == PiFaceDigital::EXITVAL_PERSIST){
        if(caching_status()) caching_disable();  // Flushes the cache, leaving the device with the last values.
    }
    else if(PiFaceDigital::EXITVAL == PiFaceDigital::EXITVAL_ZERO){
        write_through(0);    // Or set outputs to zero on quit.
    }
    close();
}


// Could be static member.. 
int PiFaceDigital::open_noinit(uint8_t hw_addr)
{
    // if initialising the first PiFace Digital, open the fd
    if (pfd_count <= 0) {
        if ((_mcp23s17_fd = mcp23s17_open(bus, chip_select)) < 0) {
            fprintf(stderr,
                    "pifacedigital_open_noinit: ERROR Could not open MCP23S17 "
                    "device.");
            return -1;
        }
    }
    pfd_count++;
    return  _mcp23s17_fd; // returns the fd, for advanced users.
}

int PiFaceDigital::open(){
      if ((_mcp23s17_fd = open_noinit(_hw_addr)) < 0) {
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
    
    mcp23s17_write_reg(ioconfig, IOCON, _hw_addr, _mcp23s17_fd);

    // I/O direction
    mcp23s17_write_reg(0x00, IODIRA, _hw_addr, _mcp23s17_fd);
    mcp23s17_write_reg(0xff, IODIRB, _hw_addr, _mcp23s17_fd);

    // GPIOB pull ups
    mcp23s17_write_reg(0xff, GPPUB, _hw_addr, _mcp23s17_fd);

    // enable interrupts
    mcp23s17_write_reg(0xff, GPINTENB, _hw_addr, _mcp23s17_fd);

    return _mcp23s17_fd; // returns the fd, for advanced users.
}

void PiFaceDigital::close()
{
    fprintf(stderr, "Close called");
    if (pfd_count <= 0)
        return;

    pfd_count--;

    // disable interrupts if enabled
    const uint8_t intenb = mcp23s17_read_reg(GPINTENB, _hw_addr, _mcp23s17_fd);
    if (intenb) {
        mcp23s17_write_reg(0, GPINTENB, _hw_addr, _mcp23s17_fd);
        // now do some other interrupt stuff...
        // TODO
    }

    // if no more PiFace Digital's, close the fd
    if (pfd_count <= 0) {
        pfd_count = 0;
        close();
    }
}

uint8_t PiFaceDigital::read_byte(uint8_t reg)
{
    
                        // access on interrupt register may not be cached!
    if(caching_status()  && reg != INTERRUPT){ 
        if(reg == IN){
            return _cached_in;
        }
        
        else if(reg == OUT){
            return _cached_out;
        }
       
        else{
            printf("Error: invalid value for register: %d", (int) reg);
            return -1;
        }
    }
    else{
        return read_through(reg);
    }   
}

uint8_t PiFaceDigital::read_byte(){
    return read_byte(IN);
}

void PiFaceDigital::write_byte(uint8_t data, uint8_t reg)
{
    if(caching_status()){
        _cached_out = data;
    }
    else{
        write_through(data, reg);
    }
}

void PiFaceDigital::write_byte(uint8_t data){
    return write_byte(data, PiFaceDigital::OUT);
}

//void    write_byte(unsigned char data){
//    return write_byte(data, PiFaceDigital::OUT);
//}




void PiFaceDigital::flush() {
    write_through(_cached_out);
    _cached_in = read_through();
}


uint8_t PiFaceDigital::read_through(uint8_t reg)
{
    // If it is an input just reverse all the bits... 
    // TODO check if that does what it should. 
    uint8_t data = mcp23s17_read_reg(reg, _hw_addr, _mcp23s17_fd);
        if(reg == IN || reg == INTERRUPT){
            uint8_t datainv = ~data;
            return datainv;
        }
        else if(reg == OUT){
            return data;
        }
    
        else{
            printf("Warning: unknown value for reg: %d", (int) reg);
            return -1;
        }
 
}

uint8_t PiFaceDigital::read_through()
{
    return read_through(IN);
}

void PiFaceDigital::write_through(uint8_t data, uint8_t reg)
{
    mcp23s17_write_reg(data, reg, _hw_addr, _mcp23s17_fd);
}

void PiFaceDigital::write_through(uint8_t data)
{
    write_through(data, OUT);
}



uint8_t PiFaceDigital::read_bit(uint8_t bit_num, uint8_t reg )
{
    
        return (read_byte(reg) >> bit_num) & 1;
    
    
}

void PiFaceDigital::write_bit(bool data,
                             uint8_t bit_num
                             )
{
write_bit(data, bit_num, OUT);
}


void PiFaceDigital::write_bit(bool data,
                             uint8_t bit_num,
                             int direction)
{
    uint8_t reg_data = read_byte(direction);
      if (data) {
        reg_data |= 1 << bit_num; // set
    } else {
        reg_data &= 0xff ^ (1 << bit_num); // clear
    }
    write_byte(reg_data, direction);
}




void PiFaceDigital::write_pin(bool data, uint8_t bit_num, int direction){
    return this->write_bit(data, bit_num, direction);
}


void PiFaceDigital::write_pin(bool data, uint8_t bit_num){
    return this->write_pin(data, bit_num, OUT);
}




uint8_t PiFaceDigital::read_pin(uint8_t pin_num,  int direction)
{
    if(direction == IN){
        return this->read_bit(pin_num, direction) & 1;
    }
    else if(direction == OUT){        
         return this->read_bit(pin_num, direction);
    }
    else{
        printf("Error: invalid value for direction.");
        return -1;
    }
}


uint8_t PiFaceDigital::read_pin(uint8_t pin_num)
{
    return read_pin(pin_num, IN);
}

int PiFaceDigital::enable_interrupts()
{
  // returns 
   _interrupts_enabled =  !mcp23s17_enable_interrupts();
   return _interrupts_enabled;
}

int PiFaceDigital::disable_interrupts()
{
   return mcp23s17_disable_interrupts();
}

int PiFaceDigital::wait_for_input(uint8_t *data,
                                 int timeout)
{
    // Flush any pending interrupts prior to wait
    read_byte(INTERRUPT);

    // Wait for input state change
    int ret = mcp23s17_wait_for_interrupt(timeout);

    // Read & return input register, thus clearing interrupt
    *data = read_byte(INTERRUPT);
    return ret;
}

int PiFaceDigital::interrupts_enabled(){
    return _interrupts_enabled;
}

bool PiFaceDigital::init_success(){
    return _init_success;
}

int PiFaceDigital::caching_enable(){
    this->_cached_in  = read_through(IN);
    this->_cached_out = read_through(OUT);
    _caching_enabled= true;
    return 0;
}
  
int PiFaceDigital::caching_disable(){
    flush();
      _caching_enabled=false;
      return 0;
  }

bool PiFaceDigital::caching_status(){
      return _caching_enabled;
}



