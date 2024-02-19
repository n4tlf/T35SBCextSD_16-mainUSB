//`timescale 1ns / 1ps
// Documented Verilog UART
// Copyright (C) 2010 Timothy Goddard (tim@goddard.net.nz)
// Distributed under the MIT licence.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 

//NOTE:  This has been modified by TFOX to comment out one of the tx drivers
//          at lines 90 and 94, which caused errors
//       Modified by Jeff Wilson 2/14/24 with the help of S100Computers 
//       members to include a rcvd_byte register, an overflow flag,
//       and fixing various data_ready flags behavior. 

module uart(
    input  clk, 			    // The master clock for this module
    input  rst, 			    // Synchronous reset.
    input  rx, 				    // Incoming serial line
    output reg txd_out, 		// Outgoing serial line
    input  transmit, 			// Strobe to transmit
    input  [7:0] tx_byte, 		// Byte to transmit
    output received, 			// Copy of Data Ready
    output reg [7:0] rcvd_byte, // Byte received
//    output is_receiving, 		// Low when receive line is idle.
    output reg is_transmitting, // Low when transmit line is idle.
    output reg recv_error, 		// Indicates error in receiving packet.
    output reg data_ready,      // Indicates when the rcvd_byte[7:0] is ready
    input  data_read,           // Clears the data_ready flag
    output reg rcvr_overrun     // Indicates when a Rx Data Overun condition occurs (cleared by data_read)
	);

//parameter CLOCK_DIVIDE = 1302; // clock rate (50Mhz) / (baud rate (9600) * 4)
  parameter CLOCK_DIVIDE = 325;  // clock rate (50Mhz) / (baud rate (38400) * 4)

// States for the receiving state machine.
// These are just constants, not parameters to override.
parameter RX_IDLE           = 0;
parameter RX_CHECK_START    = 1;
parameter RX_READ_BITS      = 2;
parameter RX_CHECK_STOP     = 3;
parameter RX_DELAY_RESTART  = 4;
parameter RX_ERROR          = 5;
parameter RX_RECEIVED       = 6;

// States for the transmitting state machine.
// Constants - do not override.
parameter TX_IDLE           = 0;
parameter TX_SENDING        = 1;
parameter TX_DELAY_RESTART  = 2;

reg [10:0] rx_clk_divider = CLOCK_DIVIDE;
reg [10:0] tx_clk_divider = CLOCK_DIVIDE;

reg [2:0] recv_state = RX_IDLE;
reg [5:0] rx_countdown;
reg [3:0] rx_bits_remaining;
reg [7:0] rx_data;
reg rx_in;

reg  data_read_reg = 0;

reg [1:0] tx_state = TX_IDLE;
reg [5:0] tx_countdown;
reg [3:0] tx_bits_remaining;
reg [7:0] tx_data;

assign received = data_ready;

always @(posedge clk) 
begin
	if (rst) 
	begin
		recv_state      = RX_IDLE;
		tx_state        = TX_IDLE;
		data_ready      = 0;
		recv_error      = 0;
		rcvr_overrun    = 0;
		is_transmitting = 0;
	end
	
	// The clk_divider counter counts down from
	// the CLOCK_DIVIDE constant. Whenever it
	// reaches 0, 1/16 of the bit period has elapsed.
   	// Countdown timers for the receiving and transmitting
	// state machines are decremented.
	rx_clk_divider = rx_clk_divider - 1;
	if (!rx_clk_divider) 
	begin
		rx_clk_divider  = CLOCK_DIVIDE;
		rx_countdown    = rx_countdown - 1;
	end

	tx_clk_divider = tx_clk_divider - 1;

	if (!tx_clk_divider) 
	begin
		tx_clk_divider  = CLOCK_DIVIDE;
		tx_countdown    = tx_countdown - 1;
	end
	
	data_read_reg = data_read;
	rx_in = rx;

	if (data_read_reg)
		data_ready = 0;
	
	// Receive state machine
	case (recv_state)
		RX_IDLE: 
			begin
			// A low pulse on the receive line indicates the
			// start of data.
			if (!rx_in)
				begin
					// Wait half the period - should resume in the
					// middle of this first pulse.
					rx_clk_divider = CLOCK_DIVIDE;
					rx_countdown = 2;
					recv_state = RX_CHECK_START;
				end
			end
		RX_CHECK_START: 
            begin
                if (!rx_countdown) 
                begin
                    // Check the pulse is still there
                    if (!rx_in) 
                    begin
                        // Pulse still there - good
                        // Wait the bit period to resume half-way
                        // through the first bit.
                        rx_countdown = 4;
                        rx_bits_remaining = 8;
                        recv_state = RX_READ_BITS;
                    end else 
                    begin
                        // Pulse lasted less than half the period -
                        // not a valid transmission.
                        recv_state = RX_ERROR;
                    end
                end
            end
		RX_READ_BITS: 
            begin
                if (!rx_countdown) 
                begin
                    // Should be half-way through a bit pulse here.
                    // Read this bit in, wait for the next if we
                    // have more to get.
                    rx_data = {rx_in, rx_data[7:1]};
                    rx_countdown = 4;
                    rx_bits_remaining = rx_bits_remaining - 1;
                    recv_state = rx_bits_remaining ? RX_READ_BITS : RX_CHECK_STOP;
                end
            end
		RX_CHECK_STOP: 
            begin
                if (!rx_countdown) 
                begin
                    // Should resume half-way through the stop bit
                    // This should be high - if not, reject the
                    // transmission and signal an error.
                    recv_state = rx_in ? RX_RECEIVED : RX_ERROR;
                end
            end
		RX_DELAY_RESTART: 
            begin
                // Waits a set number of cycles before accepting
                // another transmission.
                recv_state = rx_countdown ? RX_DELAY_RESTART : RX_IDLE;
            end
		RX_ERROR: 
            begin
                // There was an error receiving.
                // Raises the recv_error flag for one clock
                // cycle while in this state and then waits
                // 2 bit periods before accepting another
                // transmission.
                rx_countdown = 8;
                recv_state = RX_DELAY_RESTART;
            end
		RX_RECEIVED: 
            begin
                // Successfully received a byte.
                // Raises the received flag for one clock
                // cycle while in this state.
                if (!data_ready)
                begin
                    // if no overun, copy the rx byte to rcvd_byte
                    rcvd_byte = rx_data;
                    recv_state = RX_IDLE;
                    data_ready = 1;
                end
                else
                begin
                    // else ignore
                    recv_state = RX_IDLE;
                    rcvr_overrun = 1;		
                end
            end
	endcase

	// Transmit state machine
	case (tx_state)
		TX_IDLE: 
            begin
                if (transmit) 
                begin
                    // If the transmit flag is raised in the idle
                    // state, start transmitting the current content
                    // of the tx_byte input.
                    tx_data = tx_byte;
                    // Send the initial, low pulse of 1 bit period
                    // to signal the start, followed by the data
                    tx_clk_divider = CLOCK_DIVIDE;
                    tx_countdown = 4;
                    txd_out = 0;
                    tx_bits_remaining = 8;
                    tx_state = TX_SENDING;
                    is_transmitting = 1;
                end
            end
		TX_SENDING: 
            begin
                if (!tx_countdown) 
                begin
                    if (tx_bits_remaining) 
                    begin
                        tx_bits_remaining = tx_bits_remaining - 1;
                        txd_out = tx_data[0];
                        tx_data = {1'b0, tx_data[7:1]};
                        tx_countdown = 4;
                        tx_state = TX_SENDING;
                    end else 
                    begin
                        // Set delay to send out 2 stop bits.
                        txd_out = 1;
                        tx_countdown = 8;
                        tx_state = TX_DELAY_RESTART;
                    end
                end
            end
		TX_DELAY_RESTART: 
            begin
                // Wait until tx_countdown reaches the end before
                // we send another transmission. This covers the
                // "stop bit" delay.
                if (tx_countdown)
                    tx_state = TX_DELAY_RESTART;
                else
                begin
                    tx_state = TX_IDLE;
                    is_transmitting = 0;
                end
            end
	endcase
end

endmodule
