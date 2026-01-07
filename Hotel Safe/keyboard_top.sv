module keyboard_top (
  input  logic        MAX10_CLK1_50,
  input  logic [1:0]  KEY,
  inout  wire  [15:0] ARDUINO_IO,
  output logic [9:0]  LEDR,
  output logic [7:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5
);
  logic clk; assign clk = MAX10_CLK1_50;

  logic rst_n;  assign rst_n = KEY[1];
  logic rst_ff1, rst_ff2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rst_ff1 <= 1'b1;
      rst_ff2 <= 1'b1;
    end else begin
      rst_ff1 <= 1'b0;
      rst_ff2 <= rst_ff1;
    end
  end
  wire rst = rst_ff2;

  wire [3:0] col_pins = ARDUINO_IO[3:0];
  wire [3:0] row_pins;
  assign ARDUINO_IO[7:4] = row_pins;

  logic [3:0] out_code;
  logic       out_validn;
  assign ARDUINO_IO[11:8] = out_code;
  assign ARDUINO_IO[12]   = out_validn;

  assign ARDUINO_IO[15:13] = 3'bzzz;

  typedef enum logic [1:0] {R0, R1, R2, R3} scan_state_e;
  scan_state_e state, state_n;

  localparam int unsigned DEBOUNCE_CYCLES = 50_000;
  localparam int unsigned HOLD_MULT       = 8;
  localparam int unsigned HOLD_CYCLES     = DEBOUNCE_CYCLES * HOLD_MULT;

  logic [31:0] hold_cnt, hold_cnt_n;

  function automatic logic [3:0] row_onehot_lo(input scan_state_e s);
    case (s)
      R0: row_onehot_lo = 4'b1110;
      R1: row_onehot_lo = 4'b1101;
      R2: row_onehot_lo = 4'b1011;
      default: row_onehot_lo = 4'b0111;
    endcase
  endfunction

  logic [3:0] row_scan;  assign row_scan = row_onehot_lo(state);

  logic key_held;

  always_comb begin
    state_n    = state;
    hold_cnt_n = hold_cnt;

    if (key_held) begin
      state_n    = state;
      hold_cnt_n = hold_cnt;
    end else begin
      if (hold_cnt == 0) begin
        unique case (state)
          R0: state_n = R1;
          R1: state_n = R2;
          R2: state_n = R3;
          default: state_n = R0;
        endcase
        hold_cnt_n = HOLD_CYCLES;
      end else begin
        hold_cnt_n = hold_cnt - 1;
      end
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      state    <= R0;
      hold_cnt <= HOLD_CYCLES;
    end else begin
      state    <= state_n;
      hold_cnt <= hold_cnt_n;
    end
  end

  logic [3:0] row_deb_lo, col_deb_lo;
  logic       valid_level;
  logic       debounceOK;

  kb_db #(.DELAY(16)) u_db (
    .clk        (clk),
    .rst        (rst),
    .row_wires  (row_pins),
    .col_wires  (col_pins),
    .row_scan   (row_scan),
    .row        (row_deb_lo),
    .col        (col_deb_lo),
    .valid      (valid_level),
    .debounceOK (debounceOK)
  );

  localparam bit ROW_REVERSE = 1'b1;
  localparam bit COL_REVERSE = 1'b1;

  function automatic int idx_of_low(input logic [3:0] v);
    if      (v[0] == 1'b0) return 0;
    else if (v[1] == 1'b0) return 1;
    else if (v[2] == 1'b0) return 2;
    else if (v[3] == 1'b0) return 3;
    else return -1;
  endfunction

  function automatic int map_row_idx(input int r);
    if (r < 0) return -1;
    return ROW_REVERSE ? (3 - r) : r;
  endfunction

  function automatic int map_col_idx(input int c);
    if (c < 0) return -1;
    return COL_REVERSE ? (3 - c) : c;
  endfunction

  function automatic logic [3:0] map_code_idx(input int ri, input int ci);
    case (ri)
      0: case (ci)
           0: map_code_idx = 4'h1;
           1: map_code_idx = 4'h2;
           2: map_code_idx = 4'h3;
           3: map_code_idx = 4'hA;
         endcase
      1: case (ci)
           0: map_code_idx = 4'h4;
           1: map_code_idx = 4'h5;
           2: map_code_idx = 4'h6;
           3: map_code_idx = 4'hB;
         endcase
      2: case (ci)
           0: map_code_idx = 4'h7;
           1: map_code_idx = 4'h8;
           2: map_code_idx = 4'h9;
           3: map_code_idx = 4'hC;
         endcase
      default: case (ci)
           0: map_code_idx = 4'hF;
           1: map_code_idx = 4'h0;
           2: map_code_idx = 4'hE;
           3: map_code_idx = 4'hD;
         endcase
    endcase
  endfunction

  logic        armed;
  logic [3:0]  held_row_lo, held_col_lo;

  assign key_held = valid_level;

  wire key_capture = key_held & armed;

  logic commit_pulse;

  logic [3:0] code_reg;

  function automatic logic [3:0] code_from_lo(input logic[3:0] r_lo, input logic[3:0] c_lo);
    int rr, cc;
    begin
      rr = map_row_idx(idx_of_low(r_lo));
      cc = map_col_idx(idx_of_low(c_lo));
      code_from_lo = (rr>=0 && cc>=0) ? map_code_idx(rr, cc) : 4'h0;
    end
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      armed        <= 1'b1;
      held_row_lo  <= 4'hF;
      held_col_lo  <= 4'hF;
      code_reg     <= 4'h0;
      commit_pulse <= 1'b0;
    end else begin
      commit_pulse <= 1'b0;

      if (!key_held) begin
        armed       <= 1'b1;
        held_row_lo <= 4'hF;
        held_col_lo <= 4'hF;
      end else if (key_capture) begin
        held_row_lo  <= row_deb_lo;
        held_col_lo  <= col_deb_lo;
        code_reg     <= code_from_lo(row_deb_lo, col_deb_lo);
        armed        <= 1'b0;
        commit_pulse <= 1'b1;
      end
    end
  end

  localparam int unsigned PULSE_CYCLES = 50_000;
  logic [31:0] pulse_cnt;
  logic        pulse_active;

  always_ff @(posedge clk) begin
    if (rst) begin
      pulse_cnt    <= 0;
      pulse_active <= 1'b0;
      out_code     <= 4'h0;
    end else begin
      if (commit_pulse) begin
        pulse_cnt    <= PULSE_CYCLES - 1;
        pulse_active <= 1'b1;
        out_code     <= code_reg;
      end else if (pulse_active) begin
        if (pulse_cnt == 0) begin
          pulse_active <= 1'b0;
          out_code     <= 4'h0;
        end else begin
          pulse_cnt <= pulse_cnt - 1;
        end
      end
    end
  end
  assign out_validn = ~pulse_active;

  logic [3:0] digits [5:0];
  logic [5:0] digit_vld;

  integer i;
  always_ff @(posedge clk) begin
    if (rst) begin
      for (i=0; i<6; i++) digits[i] <= 4'h0;
      digit_vld <= 6'b0;
    end else if (commit_pulse) begin
      for (i=5; i>0; i--) begin
        digits[i]    <= digits[i-1];
        digit_vld[i] <= digit_vld[i-1];
      end
      digits[0]    <= code_reg;
      digit_vld[0] <= 1'b1;
    end
  end

  function automatic logic [7:0] sevenseg_hex(input logic[3:0] x);
    case (x)
      4'h0: sevenseg_hex = 8'b11000000; // 0
      4'h1: sevenseg_hex = 8'b11111001; // 1
      4'h2: sevenseg_hex = 8'b10100100; // 2
      4'h3: sevenseg_hex = 8'b10110000; // 3
      4'h4: sevenseg_hex = 8'b10011001; // 4
      4'h5: sevenseg_hex = 8'b10010010; // 5
      4'h6: sevenseg_hex = 8'b10000010; // 6
      4'h7: sevenseg_hex = 8'b11111000; // 7
      4'h8: sevenseg_hex = 8'b10000000; // 8
      4'h9: sevenseg_hex = 8'b10010000; // 9
      4'hA: sevenseg_hex = 8'b10001000; // A
      4'hB: sevenseg_hex = 8'b10000011; // b
      4'hC: sevenseg_hex = 8'b11000110; // c
      4'hD: sevenseg_hex = 8'b10100001; // d
      4'hE: sevenseg_hex = 8'b10000110; // E
      4'hF: sevenseg_hex = 8'b10001110; // F
      default: sevenseg_hex = 8'b11111111; // blank
    endcase
  endfunction

  function automatic logic [7:0] seg_or_blank(input logic vld, input logic[3:0] d);
    seg_or_blank = vld ? sevenseg_hex(d) : 8'b11111111;
  endfunction

  assign HEX0 = seg_or_blank(digit_vld[0], digits[0]);
  assign HEX1 = seg_or_blank(digit_vld[1], digits[1]);
  assign HEX2 = seg_or_blank(digit_vld[2], digits[2]);
  assign HEX3 = seg_or_blank(digit_vld[3], digits[3]);
  assign HEX4 = seg_or_blank(digit_vld[4], digits[4]);
  assign HEX5 = seg_or_blank(digit_vld[5], digits[5]);

  assign LEDR[9]   = key_held;
  assign LEDR[8]   = rst_n;
  assign LEDR[7:4] = key_held ? ~held_row_lo : 4'b0000;
  assign LEDR[3:0] = key_held ? ~held_col_lo : 4'b0000;

endmodule