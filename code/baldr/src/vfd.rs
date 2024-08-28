// This module contains code relating to transmission of data to the vfd display
// AC signal is generated across filament pins in main

use crate::hal::{
    gpio::{Pin, Output, PushPull, OpenDrain},
    spi::*,
    timer::SysDelay,
    prelude::*
};

use panic_rtt_core::{self, rtt_init_print, rprintln};


// Struct contains pins and spi bus which are used to interface with the VFD display
pub struct Vfd<S: stm32f4xx_hal::spi::Instance, const P: char, const L: u8, const O: u8> {
    pub spi_bus: Spi<S>, // Spi bus which the shift registers are attached to

    // Latch and output enable pin are on the same pin bank
    pub latch_pin: Pin<P, L, Output<PushPull>>,
    pub oe_pin: Pin<P, O, Output<OpenDrain>>, // OE pin should use an open drain because it has an external pullup resistor, OE is active low
}

// These arrays represent which bit needs to be on (in the shift registers) to switch the relevant grid / segment
const GRID_BITS: [u8; 11] = [6, 4, 2, 0, 13, 11, 9, 22, 20, 19, 17];

 //                         A, B, C, D, E, F, G DP, Comma, Hyphen
const SEG_BITS: [u8; 10] = [18, 21, 8, 14, 1, 3, 5, 10, 12, 16];

impl<S: stm32f4xx_hal::spi::Instance, const P: char, const L: u8, const O: u8> Vfd<S, P, L, O> {

    // Initialises vfd so that all segments are off
    pub fn init(&mut self, delay: &mut SysDelay) {
        // Latch and output enable pins are already initialised to a low state when they are constructed
        // So the shift registers are outputting by default

        // Write all 1's to the shift registers, because the wires connected to the VFD are pulled up to 12v.
        // Writing a 1 activates a transistor, thus sourcing current, thus turning off the relevant pin on the VFD.
        self.spi_write(&[u8::MAX; 3], delay)
    }

    // Turn off the vfd
    pub fn turn_off(&mut self, delay: &mut SysDelay) {
        for i in 0..GRID_BITS.len() {
            self.spi_write(&[u8::MAX; 3], delay);
        }
    
    }

    // Writes bytes over the spi interface to the VFD shift registers
    pub fn spi_write(&mut self, words: &[u8], delay: &mut SysDelay) {
        let _ = self.spi_bus.write(words);

        // Latch data
        delay.delay_us(1u32);
        self.latch_pin.set_high();
        delay.delay_us(1u32);
        self.latch_pin.set_low();
    }

    pub fn write_char(&mut self, character: char, grid: usize, delay: &mut SysDelay) {

        // Gets which segments (indices to the SEG_BITS array) need to be on to show a character
        let active_segments: &[usize] = match character {
            '0' => &[0, 1, 2, 3, 4, 5],
            '1' => &[1, 2],
            '2' => &[0, 1, 3, 4, 6],
            '3' => &[0, 1, 2, 3, 6],
            '4' => &[1, 2, 5, 6],
            '5' => &[0, 2, 3, 5, 6],
            '6' => &[0, 2, 3, 4, 5, 6],
            '7' => &[0, 1, 2],
            '8' => &[0, 1, 2, 3, 4, 5, 6],
            '9' => &[0, 1, 2, 5, 6],
            '\'' => &[9],
            '.' => &[7],
            ',' => &[8],
            ';' => &[7, 8],
            _ => &[],
        };

        // Initialise write_num with all 1's except for the grid we want to turn on
        let mut write_num = u32::MAX ^ 1 << GRID_BITS[grid];

        // Turn 1's to 0's where we want to turn on a segment
        for segment in active_segments.iter() {
            write_num ^= 1 << SEG_BITS[*segment];
        }

        // Turn write num into 3 bytes to send over the spi interface
        // Send most significant bits first
        let write_words: &[u8] = &[
            (write_num >> 16) as u8,
            (write_num >> 8) as u8,
            write_num as u8
        ];

        self.spi_write(write_words, delay);
    }

    // Display a string slice on the VFD
    pub fn display_str(&mut self, string_slice: &str, delay: &mut SysDelay) {
        for (i, character) in string_slice.chars().enumerate() {
            if i > GRID_BITS.len() - 1 {
                break;
            }
            self.write_char(character, i, delay);
            delay.delay_us(100u32);
        }
    }
}