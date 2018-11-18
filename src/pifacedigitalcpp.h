/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   PiFaceWrapper.h
 * Author: julian
 *
 * Created on 9. November 2018, 14:25
 * 
 *
 * First motivation was for saving the values that are put out.
 * 
 * 
 */


#ifndef PIFACEWRAPPER_H
#define PIFACEWRAPPER_H


#include <stdint.h>

#define OUTPUT 0x12 // GPIOA
#define INPUT  0x13 // GPIOB


class PiFaceDigital {
public:
    static const int OUT = 0x12;
    static const int IN  = 0x13;
    
    PiFaceDigital();
    PiFaceDigital(int _hw_addr, int _interrupts_enabled);
    PiFaceDigital(const PiFaceDigital& orig);
    virtual ~PiFaceDigital();
    
    uint8_t read_reg(uint8_t reg, uint8_t hw_addr);
    void    write_reg(uint8_t data, uint8_t reg, uint8_t hw_addr);

    uint8_t read_bit(uint8_t bit_num, uint8_t reg, uint8_t hw_addr);
    void    write_bit(uint8_t data,uint8_t bit_num, uint8_t reg, uint8_t hw_addr);
    
    uint8_t digital_read(uint8_t pin_num);
    void    digital_write(uint8_t pin_num, uint8_t value);
    
    int     wait_for_input(uint8_t *data, int timeout, uint8_t hw_addr);
    // implicitly 0 initiallised. static = same on all instances.
   
    
    static int mcp23s17_fd; 
    static int pfd_count;
     
    int interrupts_enabledi();
    
private:
   
    static const int bus = 0, chip_select = 0;
    int open();
    int open_noinit(uint8_t hw_addr);
    void close(uint8_t hw_addr);
    int enable_interrupts();
    int disable_interrupts();
    
    int file_handle;
    uint8_t inputs;         /**< Input bits (pins 0-7) */
    int hw_addr = 0;        /**< PiFaceDigital hardware address  */
    bool interrupts_enabled; /**< Whether or not interrupts are enabled  */
    
   

};

#endif /* PIFACEWRAPPER_H */

