library verilog;
use verilog.vl_types.all;
entity drone_vlg_sample_tst is
    port(
        clock           : in     vl_logic;
        dataIn          : in     vl_logic_vector(10 downto 0);
        load            : in     vl_logic;
        sampler_tx      : out    vl_logic
    );
end drone_vlg_sample_tst;
