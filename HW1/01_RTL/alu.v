module alu #(
    parameter INST_W = 4,
    parameter INT_W  = 6,
    parameter FRAC_W = 10,
    parameter DATA_W = INT_W + FRAC_W
)(
    input                      i_clk,
    input                      i_rst_n,

    input                      i_in_valid,
    output                     o_busy,
    input         [INST_W-1:0] i_inst,
    input  signed [DATA_W-1:0] i_data_a,
    input  signed [DATA_W-1:0] i_data_b,

    output                     o_out_valid,
    output        [DATA_W-1:0] o_data
);

// Finite State Machine
localparam S_IDLE = 2'b00;
localparam S_BUSY = 2'b01;
localparam S_OUTPUT = 2'b10;

reg [1:0] state, state_nxt;

// Register & Wire Declaration
reg [INST_W-1:0]    inst, inst_nxt;
reg [DATA_W-1:0]    data_a, data_a_nxt;
reg [DATA_W-1:0]    data_b, data_b_nxt;

reg                 o_busy_reg, o_busy_reg_nxt;
reg                 o_out_valid_reg, o_out_valid_reg_nxt;
reg [DATA_W-1:0]    o_data_reg, o_data_reg_nxt;

reg [$clog2(DATA_W)-1:0] counter, counter_nxt;
reg [35:0] accumulation, accumulation_nxt;
reg accumulation_done, accumulation_done_nxt;
reg [DATA_W-1:0] matrix[0:7];
reg [DATA_W-1:0] matrix_nxt[0:7];
integer i;
genvar gv;
genvar gv1, gv2, gv3;

assign o_busy = o_busy_reg;
assign o_out_valid = o_out_valid_reg;
assign o_data = o_data_reg;

wire [DATA_W-1:0] addition, subtraction;
wire [2*DATA_W-1:0] mul[0:15];
wire [2*DATA_W-1:0] multiplication;
wire [35:0] mac;
wire [11:0] sin_data; // 2 + 10
wire [23:0] pwr2[0:11], pwr2_add; // 12 * 2
wire [21:0] sin_data_pwr2; // 2 + 20
wire [43:0] pwr3[0:21], pwr3_add; // 22 * 2
wire [31:0] sin_data_pwr3; // 2 + 30
wire [63:0] pwr5[0:31], pwr5_add; // 32 * 2
wire [51:0] sin_data_pwr5; // 2 + 50
wire [51:0] first, third, fifth;
wire [103:0] first_mul; // 52 * 2
wire [103:0] third_mul[0:4]; // 52 * 2
wire [103:0] fifth_mul[0:1]; // 52 * 2
wire [103:0] taylor_add;
wire [61:0] taylor; // 2 + 60

assign addition = data_a + data_b;
assign subtraction = data_a - data_b;

generate
    for (gv=0; gv<16; gv=gv+1) begin : mul_assignment
        if (gv == 0) assign mul[gv] = (data_b[gv]) ? {{DATA_W{data_a[15]}}, data_a} : {(2*DATA_W){1'b0}};
        else assign mul[gv] = (data_b[gv]) ? {{(DATA_W-gv){data_a[15]}}, data_a, {gv{1'b0}}} : {(2*DATA_W){1'b0}};
    end
endgenerate
assign multiplication = mul[0] + mul[1] + mul[2] + mul[3] + mul[4] + mul[5] + mul[6] + mul[7]
                      + mul[8] + mul[9] + mul[10] + mul[11] + mul[12] + mul[13] + mul[14] - mul[15];
assign mac = {{4{multiplication[31]}}, multiplication} + accumulation;

assign sin_data = data_a[11:0];
generate
    for (gv1=0; gv1<12; gv1=gv1+1) begin : pwr2_assignment
        if (gv1 == 0) assign pwr2[gv1] = (sin_data[gv1]) ? {{12{sin_data[11]}}, sin_data} : {24{1'b0}};
        else assign pwr2[gv1] = (sin_data[gv1]) ? {{(12-gv1){sin_data[11]}}, sin_data, {gv1{1'b0}}} : {24{1'b0}};
    end
endgenerate
assign pwr2_add = pwr2[0] + pwr2[1] + pwr2[2] + pwr2[3] + pwr2[4] + pwr2[5] 
                + pwr2[6] + pwr2[7] + pwr2[8] + pwr2[9] + pwr2[10] - pwr2[11];
assign sin_data_pwr2 = pwr2_add[21:0];
generate
    for (gv2=0; gv2<22; gv2=gv2+1) begin : pwr3_assignment
        if (gv2 == 0) assign pwr3[gv2] = (sin_data[gv2]) ? {{22{sin_data_pwr2[21]}}, sin_data_pwr2} : {44{1'b0}};
        else if (gv2 >= 12) assign pwr3[gv2] = (sin_data[11]) ? {{(22-gv2){sin_data_pwr2[21]}}, sin_data_pwr2, {gv2{1'b0}}} : {44{1'b0}};
        else assign pwr3[gv2] = (sin_data[gv2]) ? {{(22-gv2){sin_data_pwr2[21]}}, sin_data_pwr2, {gv2{1'b0}}} : {44{1'b0}};
    end
endgenerate
assign pwr3_add = pwr3[0] + pwr3[1] + pwr3[2] + pwr3[3] + pwr3[4] + pwr3[5] 
                + pwr3[6] + pwr3[7] + pwr3[8] + pwr3[9] + pwr3[10] + pwr3[11]
                + pwr3[12] + pwr3[13] + pwr3[14] + pwr3[15] + pwr3[16] + pwr3[17]
                + pwr3[18] + pwr3[19] + pwr3[20] - pwr3[21];
assign sin_data_pwr3 = pwr3_add[31:0];
generate
    for (gv3=0; gv3<32; gv3=gv3+1) begin : pwr5_assignment
        if (gv3 == 0) assign pwr5[gv3] = (sin_data_pwr2[gv3]) ? {{32{sin_data_pwr3[31]}}, sin_data_pwr3} : {66{1'b0}};
        else if (gv3 >= 22) assign pwr5[gv3] = (sin_data_pwr2[21]) ? {{(32-gv3){sin_data_pwr3[31]}}, sin_data_pwr3, {gv3{1'b0}}} : {66{1'b0}};
        else assign pwr5[gv3] = (sin_data_pwr2[gv3]) ? {{(32-gv3){sin_data_pwr3[31]}}, sin_data_pwr3, {gv3{1'b0}}} : {66{1'b0}};
    end
endgenerate
assign pwr5_add = pwr5[0] + pwr5[1] + pwr5[2] + pwr5[3] + pwr5[4] + pwr5[5] 
                + pwr5[6] + pwr5[7] + pwr5[8] + pwr5[9] + pwr5[10] + pwr5[11]
                + pwr5[12] + pwr5[13] + pwr5[14] + pwr5[15] + pwr5[16] + pwr5[17]
                + pwr5[18] + pwr5[19] + pwr5[20] + pwr5[21] + pwr5[22] + pwr5[23]
                + pwr5[24] + pwr5[25] + pwr5[26] + pwr5[27] + pwr5[28] + pwr5[29]
                + pwr5[30] - pwr5[31];
assign sin_data_pwr5 = pwr5_add[51:0];
assign first = {sin_data, {40{1'b0}}};
assign third = {sin_data_pwr3, {20{1'b0}}};
assign fifth = sin_data_pwr5;
assign first_mul = {{42{first[51]}}, first, {10{1'b0}}};
assign third_mul[0] = {{52{third[51]}}, third};
assign third_mul[1] = {{51{third[51]}}, third, 1'b0};
assign third_mul[2] = {{49{third[51]}}, third, {3{1'b0}}};
assign third_mul[3] = {{47{third[51]}}, third, {5{1'b0}}};
assign third_mul[4] = {{45{third[51]}}, third, {7{1'b0}}};
assign fifth_mul[0] = {{52{fifth[51]}}, fifth};
assign fifth_mul[1] = {{49{fifth[51]}}, fifth, {3{1'b0}}};
assign taylor_add = first_mul - third_mul[0] - third_mul[1] - third_mul[2] - third_mul[3] - third_mul[4] + fifth_mul[0] + fifth_mul[1];
assign taylor = taylor_add[61:0];
// Combinational Circuit
always @(*) begin
    state_nxt = state;
    inst_nxt = inst;
    data_a_nxt = data_a;
    data_b_nxt = data_b;
    o_busy_reg_nxt = o_busy_reg;
    o_out_valid_reg_nxt = o_out_valid_reg;
    o_data_reg_nxt = o_data_reg;
    counter_nxt = counter;
    accumulation_nxt = accumulation;
    accumulation_done_nxt = accumulation_done;
    for (i=0; i<8; i=i+1) begin
        matrix_nxt[i] = matrix[i];
    end

    case (state)
    S_IDLE: begin
        if (i_in_valid) begin
            inst_nxt = i_inst;
            if (i_inst == 4'b1001) begin
                if (counter == 4'b0111) begin
                    for(i=0; i<8; i=i+1) begin
                        matrix_nxt[i][1:0] = i_data_a[(14-i*2)+:2];
                    end
                    state_nxt = S_BUSY;
                    counter_nxt = 4'b0000;
                    o_busy_reg_nxt = 1'b1;
                end
                else begin
                    for(i=0; i<8; i=i+1) begin
                        matrix_nxt[i][(14-counter*2)+:2] = i_data_a[(14-i*2)+:2];
                    end
                    counter_nxt = counter + 1;
                end
            end
            else begin
                state_nxt = S_BUSY;
                data_a_nxt = i_data_a;
                data_b_nxt = i_data_b;
                o_busy_reg_nxt = 1'b1;
            end
        end
    end
    S_BUSY: begin
        case (inst)
        4'b0000: begin // Signed Addition
            if (!data_a[15] && !data_b[15]) begin
                if (addition[15]) o_data_reg_nxt = {1'b0, {(DATA_W-1){1'b1}}};
                else o_data_reg_nxt = addition;
            end
            else if (data_a[15] && data_b[15]) begin
                if (!addition[15]) o_data_reg_nxt = {1'b1, {(DATA_W-1){1'b0}}};
                else o_data_reg_nxt = addition;
            end
            else begin
                o_data_reg_nxt = addition;
            end
            state_nxt = S_OUTPUT;
            o_out_valid_reg_nxt = 1'b1;
        end
        4'b0001: begin // Signed Subtraction
            if (!data_a[15] && data_b[15]) begin
                if (subtraction[15]) o_data_reg_nxt = {1'b0, {(DATA_W-1){1'b1}}};
                else o_data_reg_nxt = subtraction;
            end
            else if (data_a[15] && !data_b[15]) begin
                if (!subtraction[15]) o_data_reg_nxt = {1'b1, {(DATA_W-1){1'b0}}};
                else o_data_reg_nxt = subtraction;
            end
            else begin
                o_data_reg_nxt = subtraction;
            end
            state_nxt = S_OUTPUT;
            o_out_valid_reg_nxt = 1'b1;
        end
        4'b0010: begin // Signed MAC
            if (!accumulation_done) begin
                accumulation_done_nxt = 1'b1;
                if (!multiplication[31] && !accumulation[35]) begin
                    if (mac[35]) accumulation_nxt = {1'b0, {35{1'b1}}};
                    else accumulation_nxt = mac;
                end
                else if (multiplication[31] && accumulation[35]) begin
                    if (!mac[35]) accumulation_nxt = {1'b1, {35{1'b0}}};
                    else accumulation_nxt = mac;
                end
                else begin
                    accumulation_nxt = mac;
                end
            end
            else begin
                accumulation_done_nxt = 1'b0;
                if (accumulation[35] == 1'b0) begin
                    if (accumulation[34:0] >= {{10{1'b0}}, {16{1'b1}}, {9{1'b0}}}) o_data_reg_nxt = {1'b0, {(DATA_W-1){1'b1}}};
                    else if (accumulation[9:0] >= {1'b1, {9{1'b0}}}) o_data_reg_nxt = accumulation[25:10] + 16'b1;
                    else o_data_reg_nxt = accumulation[25:10];
                end
                else begin
                    if (accumulation[34:0] <= {{10{1'b1}}, {16{1'b0}}, {9{1'b1}}}) o_data_reg_nxt = {1'b1, {(DATA_W-1){1'b0}}};
                    else if (accumulation[9:0] >= {1'b1, {9{1'b0}}}) o_data_reg_nxt = accumulation[25:10] + 16'b1;
                    else o_data_reg_nxt = accumulation[25:10];
                end
                state_nxt = S_OUTPUT;
                o_out_valid_reg_nxt = 1'b1;
            end
        end
        4'b0011: begin // Taylor Expansion of Sin Function
            if (taylor[49:0] >= {1'b1, {49{1'b0}}}) o_data_reg_nxt = {{4{taylor[61]}}, taylor[61:50]} + 12'b1;
            else o_data_reg_nxt = {{4{taylor[61]}}, taylor[61:50]};
            state_nxt = S_OUTPUT;
            o_out_valid_reg_nxt = 1'b1;
        end
        4'b0100: begin // Binary to Gray Code
            for (i=0; i<15; i=i+1) begin
                o_data_reg_nxt[i] = data_a[i] ^ data_a[i+1];
            end
            o_data_reg_nxt[15] = data_a[15];
            state_nxt = S_OUTPUT;
            o_out_valid_reg_nxt = 1'b1;
        end
        4'b0101: begin // LRCW
            if (counter == 4'b1111) begin
                if (data_a[0]) begin
                    for (i=0; i<15; i=i+1) begin
                        o_data_reg_nxt[i+1] = data_b[i];
                    end
                    o_data_reg_nxt[0] = ~data_b[15];
                end
                else begin
                    o_data_reg_nxt = data_b;
                end
                state_nxt = S_OUTPUT;
                o_out_valid_reg_nxt = 1'b1;
                counter_nxt = 4'b0000;
            end
            else begin
                if (data_a[0]) begin
                    for (i=0; i<15; i=i+1) begin
                        data_b_nxt[i+1] = data_b[i];
                    end
                    data_b_nxt[0] = ~data_b[15];
                end
                else begin
                    data_b_nxt = data_b;
                end
                data_a_nxt = data_a >> 1;
                counter_nxt = counter + 1;
            end
        end
        4'b0110: begin // Right Rotation
            if (data_b == 4'b0000) begin
                o_data_reg_nxt = data_a;
                state_nxt = S_OUTPUT;
                o_out_valid_reg_nxt = 1'b1;
            end
            else begin
                data_b_nxt = data_b - 1;
                for (i=0; i<15; i=i+1) begin
                    data_a_nxt[i] = data_a[i+1];
                end
                data_a_nxt[15] = data_a[0];
            end
        end
        4'b0111: begin // Count Leading Zeros
            if (data_a[15]) begin
                o_data_reg_nxt = {{12{1'b0}}, counter};
                state_nxt = S_OUTPUT;
                o_out_valid_reg_nxt = 1'b1;
                counter_nxt = 4'b0000;
            end
            else begin
                if (counter == 4'b1111) begin
                    o_data_reg_nxt = {{11{1'b0}}, 5'b10000};
                    state_nxt = S_OUTPUT;
                    o_out_valid_reg_nxt = 1'b1;
                    counter_nxt = 4'b0000;
                end
                else begin
                    data_a_nxt = data_a << 1;
                    counter_nxt = counter + 1;
                end
            end
        end
        4'b1000: begin // Reverse Match4
            for (i=0; i<13; i=i+1) begin
                o_data_reg_nxt[i] = (data_a[i+:4] == data_b[(15-i)-:4]);
            end
            o_data_reg_nxt[15:13] = 3'b000;
            state_nxt = S_OUTPUT;
            o_out_valid_reg_nxt = 1'b1;
        end
        4'b1001: begin // Matrix Transpose
            o_data_reg_nxt = matrix[0];
            counter_nxt = counter + 1;
            state_nxt = S_OUTPUT;
            o_out_valid_reg_nxt = 1'b1;
        end
        default: begin

        end
        endcase
    end
    S_OUTPUT: begin
        if (inst == 4'b1001) begin
            if (counter == 4'b1000) begin
                counter_nxt = 4'b0000;
                state_nxt = S_IDLE;
                o_busy_reg_nxt = 1'b0;
                o_out_valid_reg_nxt = 1'b0;
            end
            else begin
                o_data_reg_nxt = matrix[counter[2:0]];
                counter_nxt = counter + 1;
                o_out_valid_reg_nxt = 1'b1;
            end
        end
        else begin
            state_nxt = S_IDLE;
            o_busy_reg_nxt = 1'b0;
            o_out_valid_reg_nxt = 1'b0;
        end
    end
    default: begin

    end
    endcase
end

// Sequential Circuit
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin // Active low
        state <= S_IDLE;
        inst <= {INST_W{1'b0}};
        data_a <= {DATA_W{1'b0}};
        data_b <= {DATA_W{1'b0}};
        o_busy_reg <= 1'b0;
        o_out_valid_reg <= 1'b0;
        o_data_reg <= {DATA_W{1'b0}};
        counter <= {($clog2(DATA_W)){1'b0}};
        accumulation <= {36{1'b0}};
        accumulation_done <= 1'b0;
        for (i=0; i<8; i=i+1) begin
            matrix[i] <= {DATA_W{1'b0}};
        end
    end
    else begin
        state <= state_nxt;
        inst <= inst_nxt;
        data_a <= data_a_nxt;
        data_b <= data_b_nxt;
        o_busy_reg <= o_busy_reg_nxt;
        o_out_valid_reg <= o_out_valid_reg_nxt;
        o_data_reg <= o_data_reg_nxt;
        counter <= counter_nxt;
        accumulation <= accumulation_nxt;
        accumulation_done <= accumulation_done_nxt;
        for (i=0; i<8; i=i+1) begin
            matrix[i] <= matrix_nxt[i];
        end
    end
end

endmodule
