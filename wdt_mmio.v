`ifndef WDT_MMIO_V
`define WDT_MMIO_V

`timescale 1ns/1ps

`ifndef CLK_FREQ_HZ
`define CLK_FREQ_HZ 32'd100_000_000                           // 100MHz
`endif

// Default timeout: 2ms = 2,000,000 ns
`ifndef WDT_DEFAULT_TIMEOUT
`define WDT_DEFAULT_TIMEOUT (`CLK_FREQ_HZ / 32'd1000 * 32'd2)
`endif

`ifndef WDT_MAX_TIMEOUT
`define WDT_MAX_TIMEOUT 32'hFFFF_FFFF  // ~0 at 100MHz ≈ 43s
`endif

module wdt_mmio #(
    parameter [31:0] BASE_ADDR       = 32'h8100_6000,
    parameter [31:0] CLK_FREQ        = `CLK_FREQ_HZ,            // Default 100MHz
    parameter [31:0] DEFAULT_TIMEOUT = `WDT_DEFAULT_TIMEOUT,    // N cycle
    parameter [31:0] MAX_TIMEOUT     = `WDT_MAX_TIMEOUT
)(
    input  wire                     clk,
    input  wire                     resetn,

    input  wire                     mem_valid,
    input  wire                     mem_instr,
    output reg                      mem_ready,
    input  wire [31:0]              mem_addr,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0]              mem_wdata,
    /* verilator lint_on  UNUSEDSIGNAL */
    input  wire [3:0]               mem_wstrb,
    output reg  [31:0]              mem_rdata,

    output reg                      wdt_reset,                  // Watchdog reset output
    output reg                      wdt_irq,
    input  wire                     eoi
);

    localparam [31:0] FEED_MAGIC = 32'hF00D_CAFE;

    reg wdt_irq_next;

    reg [31:0] ctrl_reg;
    reg [31:0] timeout_reg;     // Timeout cycle
    reg [31:0] feed_reg;        // Write 0xF00D_CAFE to feed dog
    reg [31:0] counter;

    reg        wdt_enabled;
    reg        wdt_armed;       // Armed dog is always enabled and timeout reg will wo
    reg        wdt_expired;

    localparam [31:0]
        RW_WDT_CTRL        = BASE_ADDR + 32'h00,
        RW_WDT_TIMEOUT     = BASE_ADDR + 32'h04,
        RO_WDT_CURRENT     = BASE_ADDR + 32'h08,
        WO_WDT_FEED        = BASE_ADDR + 32'h0C,    // Read always 0
        RO_WDT_STATUS      = BASE_ADDR + 32'h10;

    wire ctrl_wdt_en       = ctrl_reg[0];        // WDT enable
    wire ctrl_irq_en       = ctrl_reg[1];        // IRQ enable
    wire ctrl_reset_en     = ctrl_reg[2];        // Reset enable
    wire ctrl_wdt_arm      = ctrl_reg[3];        // WDT arm (write 1 to arm)
    wire ctrl_window_en    = ctrl_reg[4];        // Window mode enable

    wire status_wdt_expired = wdt_expired;
    wire status_wdt_armed   = wdt_armed;
    wire status_wdt_enabled = wdt_enabled;

    wire [31:0] status_wire = {
        29'b0,
        status_wdt_enabled,
        status_wdt_armed,
        status_wdt_expired
    };

    wire [31:0] wmask = { {8{mem_wstrb[3]}}, {8{mem_wstrb[2]}}, {8{mem_wstrb[1]}}, {8{mem_wstrb[0]}} };
    wire [31:0] wdata = mem_wdata & wmask;

    always @(posedge clk) begin: WDT_COUNTER
        if (!resetn) begin
            counter     <= 0;
            wdt_expired <= 0;
            wdt_armed   <= 0;
            wdt_enabled <= 0;
        end else begin
            wdt_enabled <= ctrl_wdt_en;

            if (ctrl_wdt_en && ctrl_wdt_arm && !wdt_expired) begin
                wdt_armed <= 1'b1;
            end

            if (wdt_armed && wdt_enabled) begin
                if (counter >= timeout_reg) begin
                    wdt_expired <= 1'b1;
                    counter <= counter;
                end else begin
                    counter <= counter + 1;
                end
            end else begin
                counter <= 0;
                wdt_expired <= 0;
            end

            if (feed_reg == FEED_MAGIC) begin
                counter <= 0;
                wdt_expired <= 0;
            end
        end
    end

    always @(posedge clk) begin: RESET_GEN
        if (!resetn) begin
            wdt_reset <= 0;
        end else begin
            if (wdt_expired && ctrl_reset_en) begin
                wdt_reset <= 1'b1;
            end else begin
                wdt_reset <= 0;
            end
        end
    end

    always @(*) begin: IRQ_GEN
        wdt_irq_next = 0;
        if (wdt_enabled && ctrl_irq_en) begin
            if (wdt_expired) begin: EXPIRED_AND_EN_IRQ
                wdt_irq_next = 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (!resetn)
            wdt_irq <= 0;
        else
            wdt_irq <= eoi ? 0 : wdt_irq_next;
    end

    always @(posedge clk) begin
        if (!resetn) begin
            mem_ready <= 0;
        end
        mem_ready <= mem_valid && !mem_instr;
    end

    always @(posedge clk) begin: MMIO_READ
        if (!resetn) begin
            mem_rdata <= 0;
        end else if (mem_valid && (!mem_instr) && mem_wstrb == 0) begin
            case (mem_addr)
                RW_WDT_CTRL:      mem_rdata <= ctrl_reg;
                RW_WDT_TIMEOUT:   mem_rdata <= timeout_reg;
                RO_WDT_CURRENT:   mem_rdata <= counter;
                WO_WDT_FEED:      mem_rdata <= 0;
                RO_WDT_STATUS:    mem_rdata <= status_wire;
                default:       mem_rdata <= 0;
            endcase
        end else begin
            mem_rdata <= 0;
        end
    end

    always @(posedge clk) begin: MMIO_WRITE
        if (!resetn) begin
            ctrl_reg    <= 0;
            timeout_reg <= DEFAULT_TIMEOUT;
            feed_reg    <= 0;
        end else begin
            if (mem_valid && (!mem_instr) && mem_wstrb != 0) begin
                case(mem_addr)
                    RW_WDT_CTRL: begin
                        // Only allow writes to enable/disable when not armed
                        if (!wdt_armed || wdt_expired) begin
                            ctrl_reg <= wdata;
                        end else begin
                            // Can only modify IRQ and reset enable bits when armed
                            ctrl_reg[2:1] <= wdata[2:1];
                        end
                    end
                    RW_WDT_TIMEOUT: begin
                        // Can only change timeout when not armed
                        if (!wdt_armed) begin
                            /* verilator lint_off CMPCONST */
                            timeout_reg <= (wdata > MAX_TIMEOUT) ? MAX_TIMEOUT : wdata;
                            /* verilator lint_on  CMPCONST */
                        end
                    end
                    WO_WDT_FEED: begin: FEED_DOG_IF_IS_FEED_MAGIC
                        feed_reg <= wdata;
                    end
                    default: ;
                endcase
            end else begin
                if (feed_reg == FEED_MAGIC) begin
                    feed_reg <= 0;
                end
            end
        end
    end

endmodule

`endif
