library verilog;
use verilog.vl_types.all;
entity drone_vlg_check_tst is
    port(
        debug           : in     vl_logic;
        \out\           : in     vl_logic;
        sampler_rx      : in     vl_logic
    );
end drone_vlg_check_tst;
