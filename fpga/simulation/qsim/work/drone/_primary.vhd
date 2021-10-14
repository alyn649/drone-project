library verilog;
use verilog.vl_types.all;
entity drone is
    port(
        dataIn          : in     vl_logic_vector(10 downto 0);
        \out\           : out    vl_logic;
        clock           : in     vl_logic;
        load            : in     vl_logic;
        debug           : out    vl_logic
    );
end drone;
