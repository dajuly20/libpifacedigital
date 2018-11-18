#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "src/pifacedigitalcpp.h"


int main( int argc, char *argv[] )
{
    uint8_t i = 0;          /**< Loop iterator */
    uint8_t inputs;         /**< Input bits (pins 0-7) */
    int hw_addr = 0;        /**< PiFaceDigital hardware address  */
    int enable_interrupts = 1; /**< Whether or not interrupts should be enabled  */
    
    

    /**
     * Read command line value for which PiFace to connect to
     */
    if (argc > 1) {
        hw_addr = atoi(argv[1]);
    }
    
    // Create Instance of pfd
    PiFaceDigital pfd(hw_addr, enable_interrupts);
    


    /**
     * Enable interrupt processing (only required for all blocking/interrupt methods).
     * Reverse the return value of pifacedigital_enable_interrupts() to be consistent
     * with the variable name "interrupts_enabled". (the function returns 0 on success)
     */
    if (!pfd.interrupts_enabledi())
        printf("Interrupts enabled.\n");
    else
        printf("Could not enable interrupts. Try running using sudo to enable PiFaceDigital interrupts.\n");


    /**
     * Bulk set all 8 outputs at once using a hexidecimal
     * representation of the inputs as an 8-bit binary
     * number, where each bit represents an output from
     * 0-7
     */
    /* Set all outputs off (00000000) */
    printf("Setting all outputs off\n");
    pfd.write_reg(0x00, OUTPUT, hw_addr);
    sleep(1);

    /* Set output states to alternating on/off (10101010) */
    printf("Setting outputs to 10101010\n");
    pfd.write_reg(0xaa, OUTPUT, hw_addr);
    sleep(1);

    /* Set output states to alternating off/on (01010101) */
    printf("Setting outputs to 01010101\n");
    pfd.write_reg(0x55, OUTPUT, hw_addr);
    sleep(1);

    /* Set all outputs off (000000) */
    printf("Setting all outputs off\n");
    pfd.write_reg(0x00, OUTPUT, hw_addr);


    /**
     * Read/write single input bits
     */
    uint8_t bit = pfd.read_bit(0, PiFaceDigital::OUT, hw_addr);
    printf("Reading bit 0: %d\n", bit);
    sleep(1);
    printf("Writing bit 0 to %d\n", bit);
    pfd.write_bit(0, 0, PiFaceDigital::OUT, hw_addr);


    /**
     * Set input pullups (must #include "mcp23s17.h")
     */
    /* pifacedigital_write_reg(0xff, GPPUB, hw_addr); */


    /**
     * Bulk read all inputs at once
     */
    inputs = pfd.read_reg(INPUT, hw_addr);
    printf("Inputs: 0x%x\n", inputs);


    /**
     * Write each output pin individually
     */
    for (i = 0; i < 8; i++) {
        const char *desc;
        if (i <= 1) desc = "pin with attached relay";
        else desc = "pin";

        /* Turn output pin i high */
        printf("Setting output %s %d HIGH\n", desc, (int)i);
        pfd.digital_write(i, 1);
        sleep(1);

        /* Turn output pin i low */
        printf("Setting output %s %d LOW\n", desc, (int)i);
        pfd.digital_write(i, 0);
        sleep(1);
    }


    /**
     * Read each input pin individually
     * A return value of 0 is pressed.
     */
    for (i = 0; i < 8; i++) {
        uint8_t pinState = pfd.digital_read(i);
        printf("Input %d value: %d\n", (int)i, (int)pinState);
    }


    /**
     * Wait for input change interrupt.
     * pifacedigital_wait_for_input returns a value <= 0 on failure.
     */
    if (pfd.interrupts_enabledi()) {
        printf("Waiting for input (press any button on the PiFaceDigital)\n");
        if (pfd.wait_for_input(&inputs, -1, hw_addr) > 0)
            printf("Inputs: 0x%x\n", inputs);
        else
            printf("Can't wait for input. Something went wrong!\n");
    }
    else
        printf("Interrupts disabled, skipping interrupt tests.\n");

    /**
     * Close the connection to the PiFace Digital
     * (pfd goes out of scope, pfd is detroyed, destructor closes piface)
     */
    
}
