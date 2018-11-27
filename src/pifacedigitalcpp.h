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
    static int _mcp23s17_fd; 
    static int pfd_count;
     
    PiFaceDigital();
    PiFaceDigital(int _hw_addr, int _interrupts_enabled);
    PiFaceDigital(const PiFaceDigital& orig);
    virtual ~PiFaceDigital();
    

    uint8_t read_pin(uint8_t pin_num, int direction);
    uint8_t read_pin(uint8_t pin_num);
    void write_pin(uint8_t data, uint8_t bit_num);
   
    int     wait_for_input(uint8_t *data, int timeout);
    // implicitly 0 initiallised. static = same on all instances.
   
    bool init_success();
    
    int caching_enable();
    int caching_disable();
    int interrupts_enabled();
    
    void flush();
    bool caching_status();


   uint8_t read_byte(uint8_t reg);
   uint8_t read_byte();
   void    write_byte(uint8_t data, uint8_t reg);
   void    write_byte(uint8_t data);
// void    write_byte(unsigned char data);
private:
    uint8_t _cached_out;
    uint8_t _cached_in; 
    
    bool _caching_enabled = false;
    bool _init_success = false;
    static const int bus = 0, chip_select = 0;
    int open();
    int open_noinit(uint8_t hw_addr);
    void close();
    int enable_interrupts();
    int disable_interrupts();

    
    
uint8_t read_through(uint8_t reg);
uint8_t read_through();
void    write_through(uint8_t data);
void    write_through(uint8_t data, uint8_t reg);

    
    

    uint8_t read_bit(uint8_t bit_num, uint8_t reg);
    void    write_bit(uint8_t data,uint8_t bit_num);
        
    int file_handle;
    uint8_t inputs;         /**< Input bits (pins 0-7) */
    int _hw_addr = 0;        /**< PiFaceDigital hardware address  */
    bool _interrupts_enabled; /**< Whether or not interrupts are enabled  */
    
   

};

#endif /* PIFACEWRAPPER_H */

