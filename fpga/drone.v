module drone(dataIn, out, clock, load, debug);
	input [10:0]dataIn;
	input clock, load;
	output out, debug;
	
	wire [15:0] packet;
	
	dshotpacket A1(dataIn, packet);
	
	bitbang16 U1(packet, out, clock, load, debug);
	
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


module bitbang16(dataIn, out, clock, load, debug);
	input [15:0]dataIn;
	input clock, load;
	output out, debug;
	
	wire [15:0]data;
	wire [3:0]pos;
	
	reg [3:0]nextPos;
	reg currBit;
	
	wire run;
	
	wire loadNext;
	wire done;
	wire bitbangOut;
	wire less15;
	wire loadRun;
	
	assign debug = less15;
	
	// On clock rise, either reset on load or increment pos
	always @(posedge clock)
		begin
		if(load)
			nextPos = 0;
		else if (run)
			nextPos = pos + 4'b0001;
		end
	
	// Registers for output packet, pos through serial and if the system should run
	reg16bit Reg_data(dataIn, load, data);
	reg4bit Reg_pos(nextPos, loadNext, pos);
	reg1bit Reg_run(less15, loadRun, run);
	
	// main bit bang module
	bitbang U1(currBit, clock, run, loadNext, bitbangOut, done);
	
	// various connections
	assign less15 = (pos < 15);
	assign loadRun = done | (load & clock);
	assign loadNext = (done & less15 & run) | (load & clock);
	assign out = bitbangOut;
		
	// mux 16 to 1
	always @(nextPos, data)
		case(nextPos)
		0: currBit = data[15];
		1: currBit = data[14];
		2: currBit = data[13];
		3: currBit = data[12];
		4: currBit = data[11];
		5: currBit = data[10];
		6: currBit = data[9];
		7: currBit = data[8];
		8: currBit = data[7];
		9: currBit = data[6];
		10: currBit = data[5];
		11: currBit = data[4];
		12: currBit = data[3];
		13: currBit = data[2];
		14: currBit = data[1];
		15: currBit = data[0];
		default: currBit = 0;
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
	always @(posedge write)
		val <= data;
			
endmodule

module flipflop (D, Clock, Q);
	input D, Clock;
	output reg Q;
	always @(posedge Clock)
	Q = D;
endmodule
