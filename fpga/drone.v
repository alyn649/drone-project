module drone(dataIn, out, clock, load, done);
	input [10:0]dataIn;
	input clock, load;
	output out, done;

	wire [15:0] packet;
	
	dshotpacket A1(dataIn, packet);
	
	bitbang16v2 U1(packet, out, clock, load, done);
	
endmodule

module flipflop (D, Clock, Q);
	input D, Clock;
	output reg Q;
	always @(posedge Clock)
		Q = D;
endmodule

module D_latch (D, Clk, Q);
	input D, Clk;
	output reg Q;
	always @(D, Clk)
		if (Clk)
			Q = D;
endmodule

module tflipflop(T, clk, Q);
	input T, clk;
	output reg Q;
	always @()
	
endmodule

// 
module bitbang16v2(dataIn, out, clock, load, done);
	input [15:0]dataIn;
	input clock, load;
	output out, done;
	
	// Pulse dimentions
	parameter high = 2;
	parameter low = 1;
	parameter pulse = 3;
	
	
	
	
endmodule

// Creates 16 bit packet from 11 bit throttle data, creating 4 bit check sum
module dshotpacket(dataIn, dataOut);
	input [10:0] dataIn;
	output [15:0] dataOut;
	
	wire [11:0] data;
	
	assign data = {dataIn[10:0], dataIn >= 48};
	
	assign dataOut[15:4] = data;
	assign dataOut[3] = data[3] ^ data[7] ^ data[11];
	assign dataOut[2] = data[2] ^ data[6] ^ data[10];
	assign dataOut[1] = data[1] ^ data[5] ^ data[9];
	assign dataOut[0] = data[0] ^ data[4] ^ data[8];
endmodule


module bitbang16(dataIn, out, clock, load, done);
	input [15:0]dataIn;
	input clock, load;
	output out, done;
	
	wire loadNext;
	assign loadNext = (load | doneBit);
	
	reg val;
	
	wire doneBit;
	
	// Enables module at right time
	reg enable;
	always @(posedge clock)
		enable = load | (!done);
	
	// Done latch
	D_latch Done_L(!(pos < 16), loadNext, done);
	
	// latch for input data
	reg [15:0]data;
	always @(posedge load)
		data = dataIn;
	
	// Increment position every complete output
	wire [7:0] pos;
	upcount U_PosCounter(load, !out, enable, pos);
	
	// Output bits 
	bitbang U_BitBang(val, clock, enable, loadNext, out, doneBit);
		
	// Mux 16 to 1
	always @(pos, data)
		case(pos)
		0: val = data[15];
		1: val = data[14];
		2: val = data[13];
		3: val = data[12];
		4: val = data[11];
		5: val = data[10];
		6: val = data[9];
		7: val = data[8];
		8: val = data[7];
		9: val = data[6];
		10: val = data[5];
		11: val = data[4];
		12: val = data[3];
		13: val = data[2];
		14: val = data[1];
		15: val = data[0];
		default: val = 0;
		endcase
endmodule

module bitbang(valIn, clk, enable, ld, out, done);
	/* // For real dshot ratio usage, with 1/24 multiple period
	parameter high = 6;
	parameter low = 3;
	parameter pulse = 8;
	*/
	
	// Test Sizes (smaller)
	parameter high = 2;
	parameter low = 1;
	parameter pulse = 3;
	
	input valIn, clk, ld, enable;
	output out, done;
	
	wire val;
	
	reg countReset;
	wire [7:0] count;
	
	// counter for clock cycles and register for 0 or 1
	upcount C1(countReset, clk, 1, count);
	reg1bit Reg_val(valIn, ld, val);
	
	// determine the output of bit bang and done feedback based on parameters and time
	assign out = ((val & (count < high)) | (!val & (count < low))) & enable;
	assign done = (count >= (pulse)) & enable;
	
	// when clock and load are high reset, the clk falling edge will allow clock to run again
	always @(clk, ld, enable)
		if(ld & clk)
			countReset = 1;
		else
			countReset = !enable;
			
endmodule

//8 bit up counter
module upcount (Reset, Clock, E, Q);
	input Reset, Clock, E;
	output reg [7:0] Q;
	always @(posedge Reset, posedge Clock)
		if (Reset)
			Q <= 0;
		else if (E)
			Q <= Q + 8'b00000001;
endmodule

// 16 bit asynchronous write  register 
module reg16bit(data, write, val);
	input [15:0] data;
	input write;
	output reg [15:0] val;
	
	// On rising clock edges, update reg value if write bit high
	always @(posedge write)
			val <= data;
			
endmodule

// 4 bit asynchronous write register 
module reg4bit(data, write, val);
	input [3:0] data;
	input write;
	output reg [3:0] val;
	
	// On rising clock edges, update reg value if write bit high
	always @(posedge write)
		val <= data;
			
endmodule

// 8 bit asynchronous write register 
module reg8bit(data, write, val);
	input [7:0] data;
	input write;
	output reg [7:0] val;
	
	// On rising clock edges, update reg value if write bit high
	always @(posedge write)
		val <= data;
			
endmodule

// 1 bit asynchronous write register 
module reg1bit(data, write, val);
	input data;
	input write;
	output reg val;
	
	// On rising clock edges, update reg value if write bit high
	always @(negedge write)
			val <= data;
			
endmodule

