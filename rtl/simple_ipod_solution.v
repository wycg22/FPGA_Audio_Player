`default_nettype none
module simple_ipod_solution(

    //////////// CLOCK //////////
    CLOCK_50,
    TD_CLK27,

    //////////// LED //////////
    LEDR,

    //////////// KEY //////////
    KEY,

    //////////// SW //////////
    SW,

    //////////// SEG7 //////////
    HEX0,
    HEX1,
    HEX2,
    HEX3,
    HEX4,
    HEX5,

    //////////// Audio //////////
    AUD_ADCDAT,
    AUD_ADCLRCK,
    AUD_BCLK,
    AUD_DACDAT,
    AUD_DACLRCK,
    AUD_XCK,

    //////////// I2C for Audio  //////////
    FPGA_I2C_SCLK,
    FPGA_I2C_SDAT,
    
    
    //////// PS2 //////////
    PS2_CLK,
    PS2_DAT,
    
    //////// SDRAM //////////
    DRAM_ADDR,
    DRAM_BA,
    DRAM_CAS_N,
    DRAM_CKE,
    DRAM_CLK,
    DRAM_CS_N,
    DRAM_DQ,
    DRAM_LDQM,
    DRAM_UDQM,
    DRAM_RAS_N,
    DRAM_WE_N,
    
    //////// GPIO //////////
    GPIO_0,
    GPIO_1
    
);
`define zero_pad(width,signal)  {{((width)-$size(signal)){1'b0}},(signal)}
//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK //////////
input                       CLOCK_50;
input                       TD_CLK27;

//////////// LED //////////
output           [9:0]      LEDR;

//////////// KEY //////////
input            [3:0]      KEY;

//////////// SW //////////
input            [9:0]      SW;

//////////// SEG7 //////////
output           [6:0]      HEX0;
output           [6:0]      HEX1;
output           [6:0]      HEX2;
output           [6:0]      HEX3;
output           [6:0]      HEX4;
output           [6:0]      HEX5;



//////////// Audio //////////
input                       AUD_ADCDAT;
inout                       AUD_ADCLRCK;
inout                       AUD_BCLK;
output                      AUD_DACDAT;
inout                       AUD_DACLRCK;
output                      AUD_XCK;

//////////// I2C for Audio  //////////
output                      FPGA_I2C_SCLK;
inout                       FPGA_I2C_SDAT;

//////////// PS2 //////////
inout                       PS2_CLK;
inout                       PS2_DAT;

//////////// GPIO //////////
inout           [35:0]      GPIO_0;
inout           [35:0]      GPIO_1;
                
                
//////////// SDRAM //////////
output          [12:0]      DRAM_ADDR;
output        [1:0]         DRAM_BA;
output                      DRAM_CAS_N;
output                      DRAM_CKE;
output                      DRAM_CLK;
output                      DRAM_CS_N;
inout           [15:0]      DRAM_DQ;
output                      DRAM_LDQM;
output                      DRAM_UDQM;
output                      DRAM_RAS_N;
output                      DRAM_WE_N;


//=======================================================
//  REG/WIRE declarations
//=======================================================
// Input and output declarations
logic CLK_50M;
logic  [9:0] LED;
assign CLK_50M =  CLOCK_50;
assign LEDR[9:0] = LED[9:0];

//Character definitions

//numbers
parameter character_0 =8'h30;
parameter character_1 =8'h31;
parameter character_2 =8'h32;
parameter character_3 =8'h33;
parameter character_4 =8'h34;
parameter character_5 =8'h35;
parameter character_6 =8'h36;
parameter character_7 =8'h37;
parameter character_8 =8'h38;
parameter character_9 =8'h39;


//Uppercase Letters
parameter character_A =8'h41;
parameter character_B =8'h42;
parameter character_C =8'h43;
parameter character_D =8'h44;
parameter character_E =8'h45;
parameter character_F =8'h46;
parameter character_G =8'h47;
parameter character_H =8'h48;
parameter character_I =8'h49;
parameter character_J =8'h4A;
parameter character_K =8'h4B;
parameter character_L =8'h4C;
parameter character_M =8'h4D;
parameter character_N =8'h4E;
parameter character_O =8'h4F;
parameter character_P =8'h50;
parameter character_Q =8'h51;
parameter character_R =8'h52;
parameter character_S =8'h53;
parameter character_T =8'h54;
parameter character_U =8'h55;
parameter character_V =8'h56;
parameter character_W =8'h57;
parameter character_X =8'h58;
parameter character_Y =8'h59;
parameter character_Z =8'h5A;

//Lowercase Letters
parameter character_lowercase_a= 8'h61;
parameter character_lowercase_b= 8'h62;
parameter character_lowercase_c= 8'h63;
parameter character_lowercase_d= 8'h64;
parameter character_lowercase_e= 8'h65;
parameter character_lowercase_f= 8'h66;
parameter character_lowercase_g= 8'h67;
parameter character_lowercase_h= 8'h68;
parameter character_lowercase_i= 8'h69;
parameter character_lowercase_j= 8'h6A;
parameter character_lowercase_k= 8'h6B;
parameter character_lowercase_l= 8'h6C;
parameter character_lowercase_m= 8'h6D;
parameter character_lowercase_n= 8'h6E;
parameter character_lowercase_o= 8'h6F;
parameter character_lowercase_p= 8'h70;
parameter character_lowercase_q= 8'h71;
parameter character_lowercase_r= 8'h72;
parameter character_lowercase_s= 8'h73;
parameter character_lowercase_t= 8'h74;
parameter character_lowercase_u= 8'h75;
parameter character_lowercase_v= 8'h76;
parameter character_lowercase_w= 8'h77;
parameter character_lowercase_x= 8'h78;
parameter character_lowercase_y= 8'h79;
parameter character_lowercase_z= 8'h7A;

//Other Characters
parameter character_colon = 8'h3A;          //':'
parameter character_stop = 8'h2E;           //'.'
parameter character_semi_colon = 8'h3B;   //';'
parameter character_minus = 8'h2D;         //'-'
parameter character_divide = 8'h2F;         //'/'
parameter character_plus = 8'h2B;          //'+'
parameter character_comma = 8'h2C;          // ','
parameter character_less_than = 8'h3C;    //'<'
parameter character_greater_than = 8'h3E; //'>'
parameter character_equals = 8'h3D;         //'='
parameter character_question = 8'h3F;      //'?'
parameter character_dollar = 8'h24;         //'$'
parameter character_space=8'h20;           //' '     
parameter character_exclaim=8'h21;          //'!'


wire Clock_1KHz, Clock_1Hz;
wire Sample_Clk_Signal;

//=======================================================================================================================
//
// Insert your code for Lab2 here!
//
//




wire            flash_mem_read;
wire            flash_mem_waitrequest;
wire    [22:0]  flash_mem_address;
wire    [31:0]  flash_mem_readdata;
wire            flash_mem_readdatavalid;
wire    [3:0]   flash_mem_byteenable;


flash flash_inst (
    .clk_clk                 (CLK_50M),
    .reset_reset_n           (1'b1),
    .flash_mem_write         (1'b0),
    .flash_mem_burstcount    (1'b1),
    .flash_mem_waitrequest   (flash_mem_waitrequest),
    .flash_mem_read          (flash_mem_read),
    .flash_mem_address       (flash_mem_address),
    .flash_mem_writedata     (),
    .flash_mem_readdata      (flash_mem_readdata),
    .flash_mem_readdatavalid (flash_mem_readdatavalid),
    .flash_mem_byteenable    (flash_mem_byteenable)
);
            
wire clk_44K;
wire [31:0] count;
Play_Speed Controller (.clk(CLOCK_50), .faster(speed_up_event), .slower(speed_down_event), .reset(speed_reset_event), .speed(count)); //Speed controller

Clock_Divider Clk_Gen44K(.clk_in(CLOCK_50), .clk_out(clk_44K), .freq(count), .reset(1'b0)); //Generate the 44kHz clk

wire synced;
Synchronizer Sync_44KHz (.out_sync_sig(synced), .outclk(CLOCK_50), .async_sig(clk_44K)); //Synchronize 44KHz clk with 50MHz clk

wire flash_start;
wire finish;          
Flash_FSM Flash_Reader(.clk(CLOCK_50), .wait_req(flash_mem_waitrequest), .valid(flash_mem_readdatavalid), .read(flash_mem_read), .start(flash_start), .fin(finish));
assign Sample_Clk_Signal = Clock_1KHz;



wire play;
wire direction;
wire restart;
wire done;
//Audio Generation Signal
//Note that the audio needs signed data - so convert 1 bit to 8 bits signed
wire[7:0] audio;
Keyboard_Input keyboard(.clk(CLOCK_50), .flash_read(done), .direction(direction), .restart(restart), .kbd_data_ready(kbd_data_ready), .key_input(kbd_received_ascii_code), .play(play));
wire interrupt; //interrupt signal lab 3
Memory_Address_Controller thing(.start(play), .clk(CLOCK_50), .synced_clk(synced), .direction(direction), .restart(restart), .data(flash_mem_readdata), 
.flash_end(finish), .flash_start(flash_start), .done(done), .read(flash_mem_read), .address(flash_mem_address), .audio(audio), .byteenable(flash_mem_byteenable), .interrupt(interrupt)); 

wire [7:0] audio_data = audio;// = {~Sample_Clk_Signal,{7{Sample_Clk_Signal}}} //generate signed sample audio signal
// reg [31:0] onehz_freq = 32'd25000000; //50MHz/(2*1Hz) = 25M
// wire onehz_clk;
// Clock_Divider div2(.clk_in(CLOCK_50), .reset(1'b0), .freq(onehz_freq), .clk_out(onehz_clk)); //divider for LEDs
// LED_Shifter LEDshift(.clk_in(onehz_clk), .LEDD(LED));

picoblaze_template #(.clk_freq_in_hz(25000000)) pico (.clk(CLK_50M), .input_data(audio_data), .interrupt(interrupt), .LED0(LED[0]), .led(LED[9:2])); //picoblaze inst lab 3

//======================================================================================
// 
// Keyboard Interface
//
//

wire ps2c, ps2d; //filtered kbd wires
wire kbd_data_ready, Kbd_to_LCD_finish;

doublesync ps2c_doublsync
(.indata(PS2_CLK),
.outdata(ps2c),
.clk(CLK_50M),
.reset(1'b1));

doublesync ps2d_doublsync
(.indata(PS2_DAT),
.outdata(ps2d),
.clk(CLK_50M),
.reset(1'b1));

wire reset_kbd_data;
(* KEEP = "TRUE" *) wire conv_now_ignore_timing;
 
wire [7:0] kbd_received_ascii_code, kbd_scan_code;
    
Kbd_ctrl Kbd_Controller(
.kbd_clk(ps2c), 
.kbd_data(ps2d),
 .clk(CLK_50M), 
.scan_code(kbd_scan_code), 
.reset_kbd_reg(~reset_kbd_data), 
.data_ready(kbd_data_ready)
);
                
key2ascii kbd2ascii(
.key_code(kbd_scan_code),
.ascii_code(kbd_received_ascii_code),
.clk(conv_now_ignore_timing)
); 
            
parameter scope_info_bytes = 16;
parameter scope_info_bits_per_byte = 8;

wire [15:0] write_kbd_debug;

wire  [scope_info_bits_per_byte-1:0] scope_info0, scope_info1, scope_info2,
     scope_info3, scope_info4, scope_info5, scope_info6, scope_info7, scope_info8, 
     scope_info9, scope_info10, scope_info11, scope_info12, scope_info13, 
     scope_info14, scope_info15;
                
Write_Kbd_To_Scope_LCD Write_Kbd_To_LCD1
(.kbd_ascii_data(kbd_received_ascii_code), 
              .kbd_ready(kbd_data_ready), .reset_kbd_data(reset_kbd_data), 
                     .sm_clk(CLK_50M), .reset(1'b1), 
                     .finish(Kbd_to_LCD_finish), 
                     .scope_info0(scope_info0), 
                     .scope_info1(scope_info1),
                     .scope_info2(scope_info2),
                     .scope_info3(scope_info3),
                     .scope_info4(scope_info4),
                     .scope_info5(scope_info5),
                     .scope_info6(scope_info6),
                     .scope_info7(scope_info7),
                     .scope_info8(scope_info8),
                     .scope_info9(scope_info9),
                     .scope_info10(scope_info10),
                     .scope_info11(scope_info11),
                     .scope_info12(scope_info12),
                     .scope_info13(scope_info13),
                     .scope_info14(scope_info14),
                     .scope_info15(scope_info15),
                     .debug(write_kbd_debug),
                     .convert_now(conv_now_ignore_timing)
    );
                
//=====================================================================================
//
// LCD Scope Acquisition Circuitry Wire Definitions                 
//
//=====================================================================================

wire allow_run_LCD_scope;
wire [15:0] scope_channelA, scope_channelB;
(* keep = 1, preserve = 1 *)wire scope_clk;
reg user_scope_enable_trigger;
wire user_scope_enable;
wire user_scope_enable_trigger_path0, user_scope_enable_trigger_path1;
wire scope_enable_source = SW[8];
wire choose_LCD_or_SCOPE = SW[9];


doublesync user_scope_enable_sync1(.indata(scope_enable_source),
                  .outdata(user_scope_enable),
                  .clk(CLK_50M),
                  .reset(1'b1)); 

//Generate the oscilloscope clock
Generate_Arbitrary_Divided_Clk32 
Generate_LCD_scope_Clk(
.inclk(CLK_50M),
.outclk(scope_clk),
.outclk_Not(),
.div_clk_count(scope_sampling_clock_count),
.Reset(1'h1));

//Scope capture channels
//Scope capture channels

(* keep = 1, preserve = 1 *) logic ScopeChannelASignal;
(* keep = 1, preserve = 1 *) logic ScopeChannelBSignal;

assign ScopeChannelASignal = Sample_Clk_Signal;
assign ScopeChannelBSignal = SW[1];
//Scope capture channels

scope_capture LCD_scope_channelA(
.clk(scope_clk),
.the_signal(ScopeChannelASignal),
.capture_enable(allow_run_LCD_scope & user_scope_enable), 
.captured_data(scope_channelA), //Insert your channel B signal here
.reset(1'b1));

scope_capture LCD_scope_channelB
(
.clk(scope_clk),
.the_signal(ScopeChannelBSignal),
.capture_enable(allow_run_LCD_scope & user_scope_enable), 
.captured_data(scope_channelB), //Insert your channel A signal here
.reset(1'b1));

//The LCD scope and display
LCD_Scope_Encapsulated_pacoblaze_wrapper LCD_LED_scope(
					    //LCD control signals
					    .lcd_d(GPIO_0[7:0]),
					    .lcd_rs(GPIO_0[8]),
					    .lcd_rw(GPIO_0[9]),
					    .lcd_e(GPIO_0[10]),
					    .clk(CLK_50M),
                
                        //LCD Display values
                      .InH(8'hAA),
                      .InG(8'hBB),
                      .InF(8'h01),
                       .InE(8'h23),
                      .InD(8'h45),
                      .InC(8'h67),
                      .InB(8'h89),
                     .InA(8'h00),
                          
                     //LCD display information signals
                         .InfoH({scope_info15,scope_info14}),
                          .InfoG({scope_info13,scope_info12}),
                          .InfoF({scope_info11,scope_info10}),
                          .InfoE({scope_info9,scope_info8}),
                          .InfoD({scope_info7,scope_info6}),
                          .InfoC({scope_info5,scope_info4}),
                          .InfoB({scope_info3,scope_info2}),
                          .InfoA({scope_info1,scope_info0}),
                          
                  //choose to display the values or the oscilloscope
                          .choose_scope_or_LCD(choose_LCD_or_SCOPE),
                          
                  //scope channel declarations
                          .scope_channelA(scope_channelA), //don't touch
                          .scope_channelB(scope_channelB), //don't touch
                          
                  //scope information generation
                          .ScopeInfoA({character_1,character_K,character_H,character_lowercase_z}),
                          .ScopeInfoB({character_S,character_W,character_1,character_space}),
                          
                 //enable_scope is used to freeze the scope just before capturing 
                 //the waveform for display (otherwise the sampling would be unreliable)
                          .enable_scope(allow_run_LCD_scope) //don't touch
                          
    );  
    

//=====================================================================================
//
//  Seven-Segment and speed control
//
//=====================================================================================

wire speed_up_event, speed_down_event;

//Generate 1 KHz Clock
Generate_Arbitrary_Divided_Clk32 
Gen_1KHz_clk
(
.inclk(CLK_50M),
.outclk(Clock_1KHz),
.outclk_Not(),
.div_clk_count(32'h61A6), //change this if necessary to suit your module
.Reset(1'h1)); 

wire speed_up_raw;
wire speed_down_raw;

doublesync 
key0_doublsync
(.indata(!KEY[0]),
.outdata(speed_up_raw),
.clk(Clock_1KHz),
.reset(1'b1));


doublesync 
key1_doublsync
(.indata(!KEY[1]),
.outdata(speed_down_raw),
.clk(Clock_1KHz),
.reset(1'b1));


parameter num_updown_events_per_sec = 10;
parameter num_1KHZ_clocks_between_updown_events = 1000/num_updown_events_per_sec;

reg [15:0] updown_counter = 0;
always @(posedge Clock_1KHz)
begin
      if (updown_counter >= num_1KHZ_clocks_between_updown_events)
      begin
            if (speed_up_raw)
            begin
                  speed_up_event_trigger <= 1;          
            end 
            
            if (speed_down_raw)
            begin
                  speed_down_event_trigger <= 1;            
            end 
            updown_counter <= 0;
      end
      else 
      begin
           updown_counter <= updown_counter + 1;
           speed_up_event_trigger <=0;
           speed_down_event_trigger <= 0;
      end     
end

wire speed_up_event_trigger;
wire speed_down_event_trigger;

async_trap_and_reset_gen_1_pulse 
make_speedup_pulse
(
 .async_sig(speed_up_event_trigger), 
 .outclk(CLK_50M), 
 .out_sync_sig(speed_up_event), 
 .auto_reset(1'b1), 
 .reset(1'b1)
 );
 
async_trap_and_reset_gen_1_pulse 
make_speedown_pulse
(
 .async_sig(speed_down_event_trigger), 
 .outclk(CLK_50M), 
 .out_sync_sig(speed_down_event), 
 .auto_reset(1'b1), 
 .reset(1'b1)
 );


wire speed_reset_event; 

doublesync 
key2_doublsync
(.indata(!KEY[2]),
.outdata(speed_reset_event),
.clk(CLK_50M),
.reset(1'b1));

parameter oscilloscope_speed_step = 100;

wire [15:0] speed_control_val;                      
speed_reg_control 
speed_reg_control_inst
(
.clk(CLK_50M),
.up_event(speed_up_event),
.down_event(speed_down_event),
.reset_event(speed_reset_event),
.speed_control_val(speed_control_val)
);

logic [15:0] scope_sampling_clock_count;
parameter [15:0] default_scope_sampling_clock_count = 12499; //2KHz


always @ (posedge CLK_50M) 
begin
    scope_sampling_clock_count <= default_scope_sampling_clock_count+{{16{speed_control_val[15]}},speed_control_val};
end 

        
        
logic [7:0] Seven_Seg_Val[5:0];
logic [3:0] Seven_Seg_Data[5:0];
    
SevenSegmentDisplayDecoder SevenSegmentDisplayDecoder_inst0(.ssOut(Seven_Seg_Val[0]), .nIn(Seven_Seg_Data[0]));
SevenSegmentDisplayDecoder SevenSegmentDisplayDecoder_inst1(.ssOut(Seven_Seg_Val[1]), .nIn(Seven_Seg_Data[1]));
SevenSegmentDisplayDecoder SevenSegmentDisplayDecoder_inst2(.ssOut(Seven_Seg_Val[2]), .nIn(Seven_Seg_Data[2]));
SevenSegmentDisplayDecoder SevenSegmentDisplayDecoder_inst3(.ssOut(Seven_Seg_Val[3]), .nIn(Seven_Seg_Data[3]));
SevenSegmentDisplayDecoder SevenSegmentDisplayDecoder_inst4(.ssOut(Seven_Seg_Val[4]), .nIn(Seven_Seg_Data[4]));
SevenSegmentDisplayDecoder SevenSegmentDisplayDecoder_inst5(.ssOut(Seven_Seg_Val[5]), .nIn(Seven_Seg_Data[5]));

assign HEX0 = Seven_Seg_Val[0];
assign HEX1 = Seven_Seg_Val[1];
assign HEX2 = Seven_Seg_Val[2];
assign HEX3 = Seven_Seg_Val[3];
assign HEX4 = Seven_Seg_Val[4];
assign HEX5 = Seven_Seg_Val[5];
            
wire Clock_2Hz;
            
Generate_Arbitrary_Divided_Clk32 
Gen_2Hz_clk
(.inclk(CLK_50M),
.outclk(Clock_2Hz),
.outclk_Not(),
.div_clk_count(32'h17D7840 >> 1),
.Reset(1'h1)
); 
        
logic [23:0] actual_7seg_output;
reg [23:0] regd_actual_7seg_output;

always @(posedge Clock_2Hz)
begin
    regd_actual_7seg_output <= actual_7seg_output;
    Clock_1Hz <= ~Clock_1Hz;
end


assign Seven_Seg_Data[0] = regd_actual_7seg_output[3:0];
assign Seven_Seg_Data[1] = regd_actual_7seg_output[7:4];
assign Seven_Seg_Data[2] = regd_actual_7seg_output[11:8];
assign Seven_Seg_Data[3] = regd_actual_7seg_output[15:12];
assign Seven_Seg_Data[4] = regd_actual_7seg_output[19:16];
assign Seven_Seg_Data[5] = regd_actual_7seg_output[23:20];
    
assign actual_7seg_output =  scope_sampling_clock_count;




//=======================================================================================================================
//
//   Audio controller code - do not touch
//
//========================================================================================================================
wire [$size(audio_data)-1:0] actual_audio_data_left, actual_audio_data_right;
wire audio_left_clock, audio_right_clock;

to_slow_clk_interface 
interface_actual_audio_data_right
 (.indata(audio_data),
  .outdata(actual_audio_data_right),
  .inclk(CLK_50M),
  .outclk(audio_right_clock));
   
   
to_slow_clk_interface 
interface_actual_audio_data_left
 (.indata(audio_data),
  .outdata(actual_audio_data_left),
  .inclk(CLK_50M),
  .outclk(audio_left_clock));
   

audio_controller 
audio_control(
  // Clock Input (50 MHz)
  .iCLK_50(CLK_50M), // 50 MHz
  .iCLK_28(), // 27 MHz
  //  7-SEG Displays
  // I2C
  .I2C_SDAT(FPGA_I2C_SDAT), // I2C Data
  .oI2C_SCLK(FPGA_I2C_SCLK), // I2C Clock
  // Audio CODEC
  .AUD_ADCLRCK(AUD_ADCLRCK),                    //  Audio CODEC ADC LR Clock
  .iAUD_ADCDAT(AUD_ADCDAT),                 //  Audio CODEC ADC Data
  .AUD_DACLRCK(AUD_DACLRCK),                    //  Audio CODEC DAC LR Clock
  .oAUD_DACDAT(AUD_DACDAT),                 //  Audio CODEC DAC Data
  .AUD_BCLK(AUD_BCLK),                      //  Audio CODEC Bit-Stream Clock
  .oAUD_XCK(AUD_XCK),                       //  Audio CODEC Chip Clock
  .audio_outL({actual_audio_data_left,8'b1}), 
  .audio_outR({actual_audio_data_right,8'b1}),
  .audio_right_clock(audio_right_clock), 
  .audio_left_clock(audio_left_clock)
);


//=======================================================================================================================
//
//   End Audio controller code
//
//========================================================================================================================
                    
            
endmodule

module Flash_FSM (clk, start, wait_req, valid, read, fin); 
    input clk, start, wait_req, valid, read;
    output reg fin;
    reg [2:0] state;
    initial begin
        state = 3'b000;
    end

    always@(posedge clk) begin
        case(state)
            3'b000: state <= (start)? 3'b001 : 3'b000; // idle
            3'b001: state <= (read)? 3'b010 : 3'b001; // checking for read
            3'b010: state <= (wait_req)? 3'b010 : 3'b011; // checking for wait request
            3'b011: state <= (valid)? 3'b011 : 3'b100; // check if data valid
            3'b100: state <= 3'b000; //finished
        default: state <= 3'b000;
        endcase
    end
    always@(*) begin
        fin = state[2];
    end

endmodule

module Clock_Divider (input clk_in, input [31:0] freq, input reset, output reg clk_out);
reg [31:0] count;
initial count = 32'b0;
always@(posedge clk_in, posedge reset) begin
  if(reset) begin // not used
    clk_out <= 0;
    count <= 32'b0;
  end
  else if (count >= freq - 1) begin // inverts clkout and resets count
    count <= 0;
    clk_out <= ~clk_out; 
  end
  else count <= count + 1; // increments count
end
endmodule

module Synchronizer (async_sig, outclk, out_sync_sig);
    input async_sig, outclk;
    output reg out_sync_sig;

    wire FDC_1_OUT, FDC_2_OUT, FDC_3_OUT;

    FDC FDC_1(.D(out_sync_sig), .Q(FDC_1_OUT), .C(!outclk), .CLR(1'b0));
    FDC FDC_2(.D(1'b1), .Q(FDC_2_OUT), .C(async_sig), .CLR(FDC_1_OUT));
    FDC FDC_3(.D(FDC_2_OUT), .Q(FDC_3_OUT), .C(outclk), .CLR(1'b0));
    FDC FDC_4(.D(FDC_3_OUT), .Q(out_sync_sig), .C(outclk), .CLR(1'b0)); 

endmodule

module FDC (C, CLR, D, Q);
    input C, CLR, D;
    output reg Q;
    always @(posedge C, posedge CLR) begin
        if(CLR) Q <= 1'b0;
        else Q <= D;
    end
endmodule

module Play_Speed (clk, reset, faster, slower, speed);
    input clk, reset, faster, slower;
    output reg [31:0] speed;
    initial speed = 32'h238;
    always@(posedge clk, posedge reset) begin
        if(reset) speed = 32'h238;
        else begin
        case({faster, slower})
            2'b01: speed <= speed - 32'h8;
            2'b10: speed <= speed + 32'h8;
            default: speed <= speed; // nothing happens
        endcase
        end
    end

endmodule

module Memory_Address_Controller (start, restart, flash_end, direction, clk, synced_clk, data, flash_start, done, read, address, audio, byteenable, state, interrupt);
    input clk, synced_clk, start, direction, restart, flash_end;
    input [31:0] data;
    output reg flash_start, done,read, interrupt;
    output reg [22:0] address;
    output reg [7:0] audio;
    output [3:0] byteenable;

    output reg [3:0] state;
    initial state = 4'b0000;
    parameter idle = 4'b0000; 
    parameter read_flash = 4'b0001;
    parameter wait_data_1 = 4'b0010;
    parameter read_data_1 = 4'b0011;
    parameter wait_data_2 = 4'b0100;
    parameter read_data_2 = 4'b0101; 
    parameter check_dir = 4'b0110;
    parameter forward = 4'b0111;
    parameter backward = 4'b1000; 
    parameter finished = 4'b1001;
    assign byteenable = 4'b1111;
    parameter wait_data_3 = 4'b1010;
    parameter read_data_3 = 4'b1011;
    parameter wait_data_4 = 4'b1100;
    parameter read_data_4 = 4'b1110;
    

    always@(*) begin
        flash_start = (state == read_flash);
        read = (state == read_flash);
        done = (state == finished);
        interrupt = (state == read_data_1 || state == read_data_2 || state == read_data_3|| state == read_data_4);
    end
    always@(posedge clk) begin
        case(state)
            idle: state <= (start)? read_flash : idle;

            read_flash: state <= (flash_end)? wait_data_1: read_flash;

            wait_data_1: state <= (synced_clk)? read_data_1: wait_data_1;
  
            read_data_1: state <= wait_data_2;
           
            wait_data_2: state <= (synced_clk)? read_data_2: wait_data_2;

            read_data_2: state <= wait_data_3;

            wait_data_3: state <= (synced_clk)? read_data_3: wait_data_3;
  
            read_data_3: state <= wait_data_4;
           
            wait_data_4: state <= (synced_clk)? read_data_4: wait_data_4;

            read_data_4: state <= check_dir;

            check_dir: state <= (direction)? backward: forward;

            forward: state <= finished;

            backward: state <= finished;

            finished: state <= idle;
        
            default: state <= idle;
        endcase
    end

    always @(posedge clk) begin
        case(state)
            read_data_1: audio <= (direction)? data[31:24]: data[7:0];
            read_data_2: audio <= (direction)? data[23:16]: data[15:8];
            read_data_3: audio <= (direction)? data[15:8]: data[23:16];
            read_data_4: audio <= (direction)? data[7:0]: data[31:24];
            backward: begin
                if (restart) address <= 23'h7FFFF;
                else begin
                    if (address == 23'b0) address <= 23'h7FFFF; 
                    else address <= address - 23'd1;
                    end
            end
            forward: begin
                if (restart) address <= 23'b0;
                else begin
                    if (address == 23'h7FFFF) address <= 0;
                    else address <= address + 23'd1;
                end 
            end
            default: begin
                address <= address;
                audio <= audio;
            end
        endcase
    end
endmodule

module Keyboard_Input(kbd_data_ready, clk, flash_read, play, key_input, direction, restart);
    input kbd_data_ready, clk, flash_read;
    input [7:0] key_input;
    output reg play, direction, restart;

    parameter character_E = 8'h45;
    parameter character_lowercase_e = 8'h65;
    parameter character_D = 8'h44;
    parameter character_lowercase_d = 8'h64;
    parameter character_B = 8'h42;
    parameter character_lowercase_b = 8'h62;
    parameter character_F = 8'h46;
    parameter character_lowercase_f = 8'h66;
    parameter character_R = 8'h52;
    parameter character_lowercase_r = 8'h72;

    reg [2:0] state;
    reg paused;
    initial state = 3'b000;
    parameter wait_key = 3'b000; 
    parameter pause_F = 3'b001;
    parameter pause_B = 3'b010;
    parameter play_F = 3'b011;
    parameter play_B = 3'b100;
    parameter restart_F = 3'b101;
    parameter restart_B = 3'b110; 

    always@(posedge clk) begin
        case(state)
        wait_key: begin
            paused <= 1'b1;
            if (key_input == character_E || key_input == character_lowercase_e) state <= play_F;
            else if (key_input == character_B || key_input == character_lowercase_b) state <= pause_B;
            else state <= wait_key;
        end
        pause_F: begin
            paused <= 1'b1;
            if (key_input == character_E || key_input == character_lowercase_e) state <= play_F;
            else if (key_input == character_B || key_input == character_lowercase_b) state <= pause_B;
            else if ((key_input == character_R || key_input == character_lowercase_r) && kbd_data_ready) state <= restart_F;
            else state <= pause_F;
        end
        pause_B: begin
            paused <= 1'b1;
            if (key_input == character_E || key_input == character_lowercase_e) state <= play_B;
            else if (key_input == character_F || key_input == character_lowercase_f) state <= pause_F;
            else if ((key_input == character_R || key_input == character_lowercase_r) && kbd_data_ready) state <= restart_B;
            else state <= pause_B;
        end
        play_F: begin
            paused <= 1'b0;
            if (key_input == character_D || key_input == character_lowercase_d) state <= pause_F;
            else if (key_input == character_B || key_input == character_lowercase_b) state <= play_B;
            else if ((key_input == character_R || key_input == character_lowercase_r) && kbd_data_ready) state <= restart_F;
            else state <= play_F;
        end
        play_B: begin
            paused <= 1'b0;
            if (key_input == character_D || key_input == character_lowercase_d) state <= pause_B;
            else if (key_input == character_F || key_input == character_lowercase_f) state <= play_F;
            else if ((key_input == character_R || key_input == character_lowercase_r) && kbd_data_ready) state <= restart_B;
            else state <= play_B;
        end
        restart_F: begin
            if(flash_read) state <= (paused)? pause_F: play_F;
            else state <= restart_F;
        end
        restart_B: begin
            if(flash_read) state <= (paused)? pause_B: play_B;
            else state <= restart_B;
        end
        default: state <= wait_key;
        endcase
    end

    always@(*) begin
        restart = (state == restart_B || state == restart_F);
        play = (state == play_F || state == play_B || state == restart_B || state == restart_F);
        direction = (state == pause_B || state == play_B || state == restart_B);
    end

endmodule

// module LED_Shifter (input clk_in, output reg [7:0] LEDD);
//     initial LEDD = 8'b00000100;
//     reg shift = 1'b1; //direction to shift
//     always@(posedge clk_in) begin
//         if(shift && LEDD == 8'b00000001) begin //right edge
//             shift = 1'b0;
//             LEDD = 8'b00000010;
//     end
//     else if(LEDD == 8'b10000000 && !shift) begin //left edge
//         shift = 1'b1;
//         LEDD = 8'b01000000;
//     end
//     else if(shift) LEDD = LEDD >> 1; //shift right
//     else LEDD = LEDD << 1; //shift left
//     end
// endmodule