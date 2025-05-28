//-----------------------------------------------------------------------------
// UVM Includes and Package Import
//-----------------------------------------------------------------------------
`include "uvm_macros.svh"
import uvm_pkg::*;

//-----------------------------------------------------------------------------
// File: axi_if.sv (AXI4-Lite Interface Definition)
//-----------------------------------------------------------------------------
interface axi_if(input logic ACLK, input logic ARESETn);
  // Write Address Channel
  logic [31:0] AWADDR;
  logic        AWVALID;
  logic        AWREADY;
  // Write Data Channel
  logic [31:0] WDATA;
  logic [3:0]  WSTRB;
  logic        WVALID;
  logic        WREADY;
  // Write Response Channel
  logic [1:0]  BRESP;
  logic        BVALID;
  logic        BREADY;
  // Read Address Channel
  logic [31:0] ARADDR;
  logic        ARVALID;
  logic        ARREADY;
  // Read Data Channel
  logic [31:0] RDATA;
  logic [1:0]  RRESP;
  logic        RVALID;
  logic        RREADY;

  // Modports (Optional but good practice)
  modport master (
    output AWADDR, AWVALID, WDATA, WSTRB, WVALID, BREADY, ARADDR, ARVALID, RREADY,
    input  AWREADY, WREADY, BRESP, BVALID, ARREADY, RDATA, RRESP, RVALID,
    input  ACLK, ARESETn
  );

  modport slave (
    input  AWADDR, AWVALID, WDATA, WSTRB, WVALID, BREADY, ARADDR, ARVALID, RREADY,
    output AWREADY, WREADY, BRESP, BVALID, ARREADY, RDATA, RRESP, RVALID,
    input  ACLK, ARESETn
  );

endinterface

//-----------------------------------------------------------------------------
// File: axi_transaction.sv (Sequence Item)
//-----------------------------------------------------------------------------
class axi_transaction extends uvm_sequence_item;
  rand bit [31:0] addr;
  rand bit [31:0] data; // Data to write
  rand bit        write; // 1 = write, 0 = read

  // Add fields to store observed results if needed
  bit [31:0] rdata_observed;
  bit [1:0]  resp_observed;

  `uvm_object_utils_begin(axi_transaction)
    `uvm_field_int(addr, UVM_ALL_ON | UVM_HEX)
    `uvm_field_int(data, UVM_ALL_ON | UVM_HEX)
    `uvm_field_int(write, UVM_ALL_ON | UVM_BIN)
    `uvm_field_int(rdata_observed, UVM_ALL_ON | UVM_HEX | UVM_READONLY) // Readonly as it's observed
    `uvm_field_int(resp_observed, UVM_ALL_ON | UVM_BIN | UVM_READONLY) // Readonly
  `uvm_object_utils_end

  function new(string name = "axi_transaction");
    super.new(name);
  endfunction

  // Override convert2string for better logging
  virtual function string convert2string();
     return $sformatf("Type:%s Addr:%0h %s Data:%0h %s",
                      get_type_name(), addr,
                      (write ? "WR" : "RD"), data,
                      (write ? "" : $sformatf("-> RData:%0h", rdata_observed))
                      );
  endfunction

endclass

//-----------------------------------------------------------------------------
// File: axi_sequence.sv
//-----------------------------------------------------------------------------
class axi_sequence extends uvm_sequence #(axi_transaction);
  `uvm_object_utils(axi_sequence)

  function new(string name = "axi_sequence");
    super.new(name);
  endfunction

  task body();
    axi_transaction tr;
    `uvm_info(get_type_name(), "Starting AXI Sequence", UVM_MEDIUM)
    // Perform some writes then reads
    for (int i = 0; i < 5; i++) begin
      `uvm_do_with(tr, { write == 1; addr == 32'h1000 + i*4; })
    end
     for (int i = 0; i < 5; i++) begin
      `uvm_do_with(tr, { write == 0; addr == 32'h1000 + i*4; })
    end
    `uvm_info(get_type_name(), "Finished AXI Sequence", UVM_MEDIUM)
  endtask
endclass

//-----------------------------------------------------------------------------
// File: axi_driver.sv
//-----------------------------------------------------------------------------
class axi_driver extends uvm_driver #(axi_transaction);
  `uvm_component_utils(axi_driver)

  virtual axi_if vif;
  uvm_analysis_port #(axi_transaction) drv_ap; // Port to send driven items to scoreboard

  function new(string name, uvm_component parent);
    super.new(name, parent);
    drv_ap = new("drv_ap", this);
  endfunction

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(virtual axi_if)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", "Virtual interface not set for axi_driver")
  endfunction

  virtual task run_phase(uvm_phase phase);
    forever begin
      axi_transaction tr;
      seq_item_port.get_next_item(tr);
      `uvm_info(get_type_name(), $sformatf("Driving transaction: %s", tr.convert2string()), UVM_HIGH)

      // Drive based on transaction type (with proper clocking)
      @(posedge vif.ACLK); // Ensure we start on a clock edge
      if (tr.write) begin
        drive_write(tr);
      end else begin
        drive_read(tr);
      end

      // Send the completed transaction to the scoreboard
      drv_ap.write(tr);

      seq_item_port.item_done();
    end
  endtask

  // Task to drive a write transaction
  virtual task drive_write(axi_transaction tr);
    // Address Phase
    vif.AWADDR <= tr.addr;
    vif.AWVALID <= 1;
    do begin
        @(posedge vif.ACLK);
    end while (!vif.AWREADY);
    vif.AWVALID <= 0;

    // Data Phase
    vif.WDATA <= tr.data;
    vif.WSTRB <= 4'hF; // Assuming full data bus write
    vif.WVALID <= 1;
    do begin
        @(posedge vif.ACLK);
    end while (!vif.WREADY);
    vif.WVALID <= 0;

    // Response Phase (Master waits for slave)
    vif.BREADY <= 1; // Assert ready to accept response
    do begin
        @(posedge vif.ACLK);
    end while (!vif.BVALID);
    tr.resp_observed = vif.BRESP; // Capture response
    vif.BREADY <= 0;
    @(posedge vif.ACLK); // Hold BREADY low for at least one cycle after handshake
  endtask

  // Task to drive a read transaction
  virtual task drive_read(axi_transaction tr);
     // Address Phase
    vif.ARADDR <= tr.addr;
    vif.ARVALID <= 1;
     do begin
        @(posedge vif.ACLK);
    end while (!vif.ARREADY);
    vif.ARVALID <= 0;

    // Data Phase (Master waits for slave)
    vif.RREADY <= 1; // Assert ready to accept data
    do begin
        @(posedge vif.ACLK);
    end while (!vif.RVALID);
    tr.rdata_observed = vif.RDATA; // Capture read data
    tr.resp_observed = vif.RRESP;  // Capture response
    vif.RREADY <= 0;
    @(posedge vif.ACLK); // Hold RREADY low for at least one cycle
  endtask

endclass

//-----------------------------------------------------------------------------
// File: axi_monitor.sv
//-----------------------------------------------------------------------------
class axi_monitor extends uvm_monitor;
  `uvm_component_utils(axi_monitor)

  virtual axi_if vif;
  uvm_analysis_port #(axi_transaction) mon_ap;

  // Internal state for multi-cycle operations
  logic [31:0] write_addr_sampled;
  logic [31:0] write_data_sampled;
  logic        write_addr_phase_done;
  logic        write_data_phase_done;

  logic [31:0] read_addr_sampled;
  logic        read_addr_phase_done;


  function new(string name, uvm_component parent);
    super.new(name, parent);
    mon_ap = new("mon_ap", this);
  endfunction

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db #(virtual axi_if)::get(this, "", "vif", vif))
      `uvm_fatal("NOVIF", "Virtual interface not set for axi_monitor")
  endfunction

  virtual task run_phase(uvm_phase phase);
    fork
      monitor_write();
      monitor_read();
    join_none // Let them run concurrently
  endtask

  // Monitor Write Transactions
  virtual task monitor_write();
    forever begin
      @(posedge vif.ACLK);
      if (!vif.ARESETn) begin // Reset logic
         write_addr_phase_done = 0;
         write_data_phase_done = 0;
         continue;
      end

      // Detect Write Address Phase Handshake
      if (vif.AWVALID && vif.AWREADY) begin
        write_addr_sampled = vif.AWADDR;
        write_addr_phase_done = 1;
         `uvm_info(get_type_name(), $sformatf("Monitor saw AW Handshake: Addr=%0h", write_addr_sampled), UVM_HIGH)
      end

       // Detect Write Data Phase Handshake
      if (vif.WVALID && vif.WREADY) begin
        write_data_sampled = vif.WDATA;
        write_data_phase_done = 1;
        `uvm_info(get_type_name(), $sformatf("Monitor saw W Handshake: Data=%0h", write_data_sampled), UVM_HIGH)
      end

      // Detect Write Response Phase Handshake
      if (vif.BVALID && vif.BREADY) begin
        if (write_addr_phase_done && write_data_phase_done) begin // Ensure prior phases happened
            axi_transaction tr = axi_transaction::type_id::create("tr_mon_write");
            tr.addr = write_addr_sampled;
            tr.data = write_data_sampled; // For writes, 'data' is the written data
            tr.write = 1;
            tr.resp_observed = vif.BRESP;
            mon_ap.write(tr);
            `uvm_info(get_type_name(), $sformatf("Monitor SENT WRITE transaction: %s", tr.convert2string()), UVM_MEDIUM)
            // Reset state for next write transaction
            write_addr_phase_done = 0;
            write_data_phase_done = 0;
        end else begin
            `uvm_warning(get_type_name(), "BVALID/BREADY handshake seen without preceding AW/W handshake")
            // Optionally reset state here too, depending on desired strictness
        end
      end

      // Reset phase flags if VALID goes low before READY (protocol violation or aborted transaction)
      // Simplified: Just reset if no response handshake occurs soon after address/data
       if (!vif.AWVALID && write_addr_phase_done && !write_data_phase_done && !vif.BVALID) begin
           // Could add timeout logic here if needed
       end
        if (!vif.WVALID && write_data_phase_done && !vif.BVALID) begin
             // Could add timeout logic here if needed
        end

    end
  endtask

  // Monitor Read Transactions
  virtual task monitor_read();
    forever begin
      @(posedge vif.ACLK);
       if (!vif.ARESETn) begin // Reset logic
         read_addr_phase_done = 0;
         continue;
      end

      // Detect Read Address Phase Handshake
      if (vif.ARVALID && vif.ARREADY) begin
        read_addr_sampled = vif.ARADDR;
        read_addr_phase_done = 1;
         `uvm_info(get_type_name(), $sformatf("Monitor saw AR Handshake: Addr=%0h", read_addr_sampled), UVM_HIGH)
      end

      // Detect Read Data Phase Handshake
      if (vif.RVALID && vif.RREADY) begin
        if (read_addr_phase_done) begin // Ensure address phase happened
            axi_transaction tr = axi_transaction::type_id::create("tr_mon_read");
            tr.addr = read_addr_sampled;
            tr.data = 0; // 'data' field is for write data in this model
            tr.rdata_observed = vif.RDATA; // Store observed read data
            tr.write = 0;
            tr.resp_observed = vif.RRESP;
            mon_ap.write(tr);
             `uvm_info(get_type_name(), $sformatf("Monitor SENT READ transaction: %s", tr.convert2string()), UVM_MEDIUM)
            // Reset state for next read transaction
            read_addr_phase_done = 0;
        end else begin
             `uvm_warning(get_type_name(), "RVALID/RREADY handshake seen without preceding AR handshake")
        end
      end
       // Reset phase flags if VALID goes low before READY
        if (!vif.ARVALID && read_addr_phase_done && !vif.RVALID) begin
           // Could add timeout logic here if needed
       end
    end
  endtask

endclass

//-----------------------------------------------------------------------------
// File: axi_agent.sv
//-----------------------------------------------------------------------------
class axi_agent extends uvm_agent;
  `uvm_component_utils(axi_agent)

  axi_driver drv;
  axi_monitor mon;
  uvm_sequencer #(axi_transaction) seqr;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    drv = axi_driver::type_id::create("drv", this);
    mon = axi_monitor::type_id::create("mon", this);
    seqr = uvm_sequencer #(axi_transaction)::type_id::create("seqr", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    drv.seq_item_port.connect(seqr.seq_item_export);
    // Analysis port connections happen in the environment
  endfunction
endclass

//-----------------------------------------------------------------------------
// File: axi_scoreboard.sv
//-----------------------------------------------------------------------------
`uvm_analysis_imp_decl(_drv)
`uvm_analysis_imp_decl(_mon)

class axi_scoreboard extends uvm_component;
  `uvm_component_utils(axi_scoreboard)

  uvm_analysis_imp_drv #(axi_transaction, axi_scoreboard) drv_ap_imp;
  uvm_analysis_imp_mon #(axi_transaction, axi_scoreboard) mon_ap_imp;

  // Simple memory model for checking
  logic [31:0] expected_mem [bit [31:0]];
  int unsigned write_count = 0;
  int unsigned read_count = 0;
  int unsigned error_count = 0;


  function new(string name, uvm_component parent);
    super.new(name, parent);
    drv_ap_imp = new("drv_ap_imp", this);
    mon_ap_imp = new("mon_ap_imp", this);
  endfunction

  // Called when a transaction arrives from the DRIVER
  virtual function void write_drv(axi_transaction tr);
    `uvm_info(get_type_name(), $sformatf("SB Rcvd from DRIVER: %s", tr.convert2string()), UVM_HIGH)
    if (tr.write) begin
      // Store written data into our expected memory model
      expected_mem[tr.addr] = tr.data;
       `uvm_info(get_type_name(), $sformatf("SB Stored Expected: MEM[%0h] = %0h", tr.addr, tr.data), UVM_MEDIUM)
    end
    // For reads, we don't need to do anything with the driver tr here,
    // as the comparison happens when the monitor transaction arrives.
  endfunction

  // Called when a transaction arrives from the MONITOR
  virtual function void write_mon(axi_transaction tr);
     // FIX: Declare local variables at the start of the function scope
     logic [31:0] expected_data;
     logic mem_exists;

     `uvm_info(get_type_name(), $sformatf("SB Rcvd from MONITOR: %s", tr.convert2string()), UVM_HIGH)
    if (tr.write) begin
      write_count++;
      // Basic check: ensure write response was OKAY
      if (tr.resp_observed != 2'b00) begin
        `uvm_error(get_type_name(), $sformatf("Write Error Response Observed: Addr=%0h Resp=%b", tr.addr, tr.resp_observed))
        error_count++;
      end
    end else begin // It's a Read transaction
      read_count++;
      // logic [31:0] expected_data; // FIX: Moved declaration up
      // logic mem_exists;           // FIX: Moved declaration up

       // Basic check: ensure read response was OKAY
      if (tr.resp_observed != 2'b00) begin
        `uvm_error(get_type_name(), $sformatf("Read Error Response Observed: Addr=%0h Resp=%b", tr.addr, tr.resp_observed))
        error_count++;
      end else begin
          // Check if we ever wrote to this address
          mem_exists = expected_mem.exists(tr.addr);
          if (mem_exists) begin
              expected_data = expected_mem[tr.addr];
          end else begin
              expected_data = 32'h0; // Or define your default memory value
              `uvm_info(get_type_name(), $sformatf("Read from unwritten address %0h, assuming expected data is 0", tr.addr), UVM_LOW)
          end

          // Compare monitored read data with expected data
          if (tr.rdata_observed !== expected_data) begin
              `uvm_error(get_type_name(), $sformatf("Read Data MISMATCH: Addr=%0h Expected=%0h Observed=%0h", tr.addr, expected_data, tr.rdata_observed))
              error_count++;
          end else begin
              `uvm_info(get_type_name(), $sformatf("Read Data MATCH: Addr=%0h Data=%0h", tr.addr, tr.rdata_observed), UVM_MEDIUM)
          end
      end
    end
  endfunction

  function void report_phase(uvm_phase phase);
      `uvm_info(get_type_name(), $sformatf("Scoreboard Summary: Writes=%0d Reads=%0d Errors=%0d", write_count, read_count, error_count), UVM_LOW)
      if (error_count > 0) begin
           `uvm_error(get_type_name(), $sformatf("%0d ERRORS detected by scoreboard!", error_count))
      end else begin
           `uvm_info(get_type_name(), "Scoreboard detected NO errors.", UVM_LOW)
      end
  endfunction

endclass

//-----------------------------------------------------------------------------
// File: axi_env.sv
//-----------------------------------------------------------------------------
class axi_env extends uvm_env;
  `uvm_component_utils(axi_env)

  axi_agent agt;
  axi_scoreboard sb;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    agt = axi_agent::type_id::create("agt", this);
    sb = axi_scoreboard::type_id::create("sb", this);
  endfunction

  function void connect_phase(uvm_phase phase);
    // Connect agent analysis ports to scoreboard
    agt.mon.mon_ap.connect(sb.mon_ap_imp);
    agt.drv.drv_ap.connect(sb.drv_ap_imp);
  endfunction
endclass

//-----------------------------------------------------------------------------
// File: axi_test.sv
//-----------------------------------------------------------------------------
class axi_test extends uvm_test;
  `uvm_component_utils(axi_test)

  axi_env env;

  function new(string name, uvm_component parent);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = axi_env::type_id::create("env", this);
  endfunction

  task run_phase(uvm_phase phase);
    axi_sequence seq;
    phase.raise_objection(this, "Starting AXI Test Sequence");
    seq = axi_sequence::type_id::create("seq");
    // Add some delay after reset before starting sequence
    #100ns;
    seq.start(env.agt.seqr);
    #100ns; // Allow time for last transaction to complete
    phase.drop_objection(this, "Finished AXI Test Sequence");
  endtask
endclass


//-----------------------------------------------------------------------------
// File: dummy_axi_slave.sv (Simple AXI Slave Memory Model)
//-----------------------------------------------------------------------------
module dummy_axi_slave (axi_if.slave axi); // Use slave modport

  // Simple memory implementation
  logic [31:0] mem [bit [31:0]];

  // Internal state registers
  logic [31:0] stored_awaddr;
  logic [31:0] stored_araddr;
  logic write_addr_received;
  logic read_addr_received;

  // Default assignments (combinational)
  assign axi.AWREADY = !write_addr_received; // Ready if not already processing a write address
  assign axi.WREADY = write_addr_received;  // Ready for data only after address is received
  assign axi.ARREADY = !read_addr_received;   // Ready if not already processing a read address

  // Write Logic
  always_ff @(posedge axi.ACLK or negedge axi.ARESETn) begin
    if (!axi.ARESETn) begin
      write_addr_received <= 1'b0;
      axi.BVALID <= 1'b0;
      axi.BRESP <= 2'b00; // OKAY
    end else begin
      // Address Phase Latching
      if (axi.AWREADY && axi.AWVALID) begin
        stored_awaddr <= axi.AWADDR;
        write_addr_received <= 1'b1; // Latched address, now wait for data
      end

      // Data & Response Phase
      if (write_addr_received) begin // Only proceed if address was latched
          if(axi.WREADY && axi.WVALID) begin // Data received
            // FIX: Use blocking assignment for associative array write
            mem[stored_awaddr] = axi.WDATA; // Write to memory
            axi.BVALID <= 1'b1; // Signal response ready
            write_addr_received <= 1'b0; // Ready for next addr after this data phase completes
             $display("[%0t] SLAVE: Write %h to Addr %h", $time, axi.WDATA, stored_awaddr);
          end
      end

      // Response handshake
      if (axi.BVALID && axi.BREADY) begin
          axi.BVALID <= 1'b0; // Master accepted response
      end
    end
  end

  // Read Logic
  always_ff @(posedge axi.ACLK or negedge axi.ARESETn) begin
     if (!axi.ARESETn) begin
       read_addr_received <= 1'b0;
       axi.RVALID <= 1'b0;
       axi.RDATA <= 32'b0;
       axi.RRESP <= 2'b00; // OKAY
     end else begin
       // Address Phase Latching
       if (axi.ARREADY && axi.ARVALID) begin
         stored_araddr <= axi.ARADDR;
         read_addr_received <= 1'b1; // Latched address, prepare data
       end

       // Data/Response Phase
       if(read_addr_received) begin
            // Look up data (handle non-existent addresses gracefully)
            if (mem.exists(stored_araddr)) begin
                axi.RDATA <= mem[stored_araddr];
                axi.RRESP <= 2'b00; // OKAY
                 $display("[%0t] SLAVE: Read Addr %h Data %h", $time, stored_araddr, mem[stored_araddr]);
            end else begin
                 axi.RDATA <= 32'hDEADBEEF; // Indicate uninitialized read
                 axi.RRESP <= 2'b10; // SLVERR (or keep OKAY depending on spec)
                 $display("[%0t] SLAVE: Read Addr %h Data DEADBEEF (Uninitialized)", $time, stored_araddr);
            end
            axi.RVALID <= 1'b1; // Signal data is ready
            read_addr_received <= 1'b0; // Ready for next address after this data phase
       end

        // Data handshake
       if (axi.RVALID && axi.RREADY) begin
          axi.RVALID <= 1'b0; // Master accepted data
       end
     end
  end

endmodule


//-----------------------------------------------------------------------------
// File: tb_top.sv (Testbench Top Level)
//-----------------------------------------------------------------------------
module tb_top;
  timeprecision 1ns;
  timeunit 1ns;

  // Clock and Reset Generation
  bit ACLK = 0;
  bit ARESETn = 0;

  localparam CLK_PERIOD = 10; // 10ns clock period

  always #(CLK_PERIOD/2) ACLK = ~ACLK;  // Generate clock

  // Instantiate AXI Interface
  axi_if axi_inst(ACLK, ARESETn);

  // Instantiate the Dummy DUT (AXI Slave)
  dummy_axi_slave dut ( .axi(axi_inst) ); // Connect using interface

  // Reset Generation
  initial begin
    ARESETn = 0;
    $display("[%0t] Asserting Reset", $time);
    repeat(5) @(posedge ACLK); // Assert reset for 5 cycles
    ARESETn = 1;
    $display("[%0t] Deasserting Reset", $time);
    @(posedge ACLK); // One cycle settling time
  end

  // UVM Test Execution
  initial begin
    // Set the virtual interface in the UVM configuration database
    // Using "*" targets all components at any hierarchy level below null (top)
    uvm_config_db #(virtual axi_if)::set(null, "*", "vif", axi_inst);

    // Run the specific UVM test
    run_test("axi_test");
  end

  // Optional: Simulation Timeout / End condition
  initial begin
      #5000ns; // Set a timeout duration (e.g., 5000 ns)
      $display("[%0t] Simulation Timeout!", $time);
      $finish;
  end
  
  initial begin
  	$dumpfile("waveform.vcd");
  	$dumpvars(0, tb_top);
  end


endmodule