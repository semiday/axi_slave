# UVM AXI4-Lite Verification Environment

This project demonstrates a basic UVM (Universal Verification Methodology) testbench for verifying an AXI4-Lite slave interface. It includes a configurable AXI master agent, a monitor, a scoreboard for data checking, and a simple AXI slave (memory model) as the Design Under Test (DUT).

**Purpose:**
To provide a foundational example of UVM components interacting to drive AXI4-Lite transactions, monitor bus activity, and verify data integrity.

**Key Features:**

*   **AXI4-Lite Protocol:** Targets the AXI4-Lite interface.
*   **UVM Based:** Utilizes standard UVM classes and methodology (sequences, driver, monitor, agent, scoreboard, environment, test).
*   **Configurable Transactions:** `axi_transaction` sequence item allows randomization of address, data, and operation type (read/write).
*   **Reusable AXI Agent:** `axi_agent` encapsulates the driver, monitor, and sequencer for the AXI master.
*   **Data Checking:** `axi_scoreboard` compares data read from the DUT against data previously written, using a simple internal memory model.
*   **Dummy Slave DUT:** A behavioral `dummy_axi_slave` module acts as a simple memory-mapped slave, responding to read and write requests. This can be replaced with an actual RTL DUT.
*   **Clear Logging:** Uses `uvm_info`, `uvm_error` for reporting test progress and issues.

**How to Run (General EDA Playground / Simulator Steps):**

1.  Ensure you have a SystemVerilog simulator that supports UVM (e.g., VCS, Xcelium, Questa, Riviera-PRO).
2.  Load the UVM library (e.g., UVM 1.2).
3.  Compile all `.sv` files.
4.  Run the simulation, typically by invoking `run_test("axi_test")` from the `tb_top` module.

**Project Structure (Conceptual Files):**

*   `axi_if.sv`: AXI4-Lite interface definition.
*   `axi_transaction.sv`: Defines the AXI transaction (sequence item).
*   `axi_sequence.sv`: Example sequence to generate AXI transactions.
*   `axi_driver.sv`: Drives AXI transactions onto the interface.
*   `axi_monitor.sv`: Observes AXI bus activity and reports transactions.
*   `axi_agent.sv`: Encapsulates driver, monitor, and sequencer.
*   `axi_scoreboard.sv`: Verifies data integrity by comparing driven vs. monitored transactions.
*   `axi_env.sv`: Integrates the agent and scoreboard.
*   `axi_test.sv`: Base test to configure the environment and start sequences.
*   `dummy_axi_slave.sv`: A simple behavioral AXI slave (DUT).
*   `tb_top.sv`: Top-level testbench module, instantiates the DUT, interface, and starts the UVM test.

---

## Testbench Architecture

The UVM testbench is structured hierarchically to verify an AXI4-Lite slave DUT:

1.  **`tb_top` (Top-Level Module):**
    *   Instantiates the AXI4-Lite **interface** (`axi_if`).
    *   Instantiates the **DUT** (`dummy_axi_slave` or your RTL).
    *   Connects the DUT to the `axi_if`.
    *   Provides **clock and reset** generation.
    *   Uses `uvm_config_db` to pass the virtual interface to the UVM environment.
    *   Calls `run_test()` to start the UVM verification.

2.  **`axi_test` (UVM Test):**
    *   The highest level UVM component.
    *   Instantiates the **UVM environment** (`axi_env`).
    *   In its `run_phase`:
        *   Creates and starts one or more **sequences** (`axi_sequence`) on the agent's sequencer.
        *   Controls test duration using UVM objections.

3.  **`axi_env` (UVM Environment):**
    *   Instantiates the AXI **agent** (`axi_agent`).
    *   Instantiates the **scoreboard** (`axi_scoreboard`).
    *   In its `connect_phase`:
        *   Connects the agent's monitor analysis port (`mon_ap`) to the scoreboard's analysis import.
        *   Connects the agent's driver analysis port (`drv_ap`) to the scoreboard's analysis import (for tracking stimulus).

4.  **`axi_agent` (UVM Agent - AXI Master):**
    *   Configured as an active AXI master.
    *   Instantiates:
        *   **`uvm_sequencer #(axi_transaction)` (`seqr`):** Arbitrates and forwards sequence items (`axi_transaction`) from sequences.
        *   **`axi_driver` (`drv`):** Receives transactions from the sequencer and drives them onto the `axi_if` according to AXI4-Lite protocol. It also sends driven transactions to an analysis port (`drv_ap`).
        *   **`axi_monitor` (`mon`):** Passively observes `axi_if` signals, reconstructs AXI transactions, and broadcasts them via an analysis port (`mon_ap`).
    *   In its `connect_phase`: Connects the driver's `seq_item_port` to the sequencer's `seq_item_export`.

5.  **`axi_scoreboard` (UVM Scoreboard):**
    *   Receives transactions from the `axi_driver` (via `drv_ap_imp`) representing the stimulus sent to the DUT.
    *   Receives transactions from the `axi_monitor` (via `mon_ap_imp`) representing the observed behavior of the DUT.
    *   Maintains an **internal predictive model** (e.g., a simple memory model `expected_mem`).
    *   **Compares** observed read data from the monitor against expected data (based on previous writes recorded from the driver).
    *   Reports **matches or mismatches** using UVM reporting mechanisms (`uvm_info`, `uvm_error`).

6.  **Sequences and Sequence Items:**
    *   **`axi_transaction` (Sequence Item):** Represents a single AXI read or write operation. Contains fields for `addr`, `data` (for writes), `write` (direction), and fields to store observed results like `rdata_observed`. It is randomized.
    *   **`axi_sequence`:** Generates a series of `axi_transaction` items and sends them to the driver via the sequencer. This example sequence performs a series of writes followed by reads to the same addresses.

### Data Flow for Verification:

1.  `axi_test` starts `axi_sequence`.
2.  `axi_sequence` creates randomized `axi_transaction` items.
3.  Transactions flow: `axi_sequence` -> `sequencer` -> `axi_driver`.
4.  `axi_driver` drives AXI signals on `axi_if` to the `DUT`.
5.  `axi_driver` also sends a copy of the driven transaction to the `axi_scoreboard`.
6.  `DUT` responds to the AXI transactions.
7.  `axi_monitor` observes `axi_if` and reconstructs the AXI transaction performed by the DUT.
8.  `axi_monitor` sends the observed transaction to the `axi_scoreboard`.
9.  `axi_scoreboard`:
    *   For writes from the driver: Updates its internal `expected_mem`.
    *   For reads from the monitor: Compares `rdata_observed` with data from `expected_mem` at `tr.addr`. Reports pass/fail.

This architecture provides a modular and scalable approach to verifying the AXI4-Lite slave.
