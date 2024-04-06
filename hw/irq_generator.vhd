library IEEE;
    use IEEE.STD_LOGIC_1164.ALL;
    use ieee.std_logic_unsigned.all;
    use ieee.std_logic_arith.all;

entity irq_generator is
    port(
        CLK             :   in      std_logic                                   ;
        RESET           :   in      std_Logic                                   ;

        USER_EVENT_IN   :   in      std_logic                                   ;
        RETRY           :   in      std_logic                                   ;
        USER_EVENT_OUT  :   out     std_logic                                   ;
        DURATION        :   in      std_logic_Vector ( 31 downto 0 )            
    );
end irq_generator;



architecture irq_generator_arch of irq_generator is

    signal  duration_reg        :       std_logic_Vector ( 31 downto 0 )    := (others => '0') ;
    signal  d_user_event_in     :       std_logic                           := '0';
    signal  has_user_event      :       std_Logic                           := '0';

    type fsm is (
        IDLE_ST         ,
        EVENT_GEN_ST     
    );

    signal  current_state : fsm := IDLE_ST;

    signal  user_event_out_reg : std_Logic := '0';

begin


    USER_EVENT_OUT <= user_event_out_reg;

    user_event_out_reg_processing : process(CLK)
    begin 
        if CLK'event AND CLK = '1' then 
            case current_state is 
                when EVENT_GEN_ST => 
                    user_event_out_reg <= '1';

                when others => 
                    user_event_out_reg <= '0';
            end case;
        end if;
    end process;

    d_event_processing : process(CLK)
    begin
        if CLK'event AND CLK = '1' then 
            d_user_event_in <= USER_EVENT_IN;
        end if;
    end process;

    has_user_event_processing : process(CLK)
    begin
        if CLK'event AND CLK = '1' then 
            if USER_EVENT_IN = '1' and d_user_event_in = '0' then 
                has_user_event <= '1';
            else
                has_user_event <= '0';
            end if;
        end if;
    end process;

    duration_reg_processing : process(CLK)
    begin
        if CLK'event AND CLK = '1' then 
            if RESET = '1' then 
                duration_reg <= (others => '0');
            else

                case current_state is 
                    when EVENT_GEN_ST => 
                        if (duration_reg < DURATION-1) then 
                            duration_reg <= duration_reg + 1;
                        else
                            duration_reg <= duration_reg;
                        end if; 

                    when others => 
                        duration_reg <= (others => '0');

                end case;
            end if;
        end if;
    end process;

    current_state_processing : process(CLK)
    begin
        if CLK'event AND CLK = '1' then 
            if RESET = '1' then 
                current_state <= IDLE_ST;
            else
                case current_state is

                    when IDLE_ST => 
                        if has_user_event = '1' or RETRY = '1' then 
                            current_state <= EVENT_GEN_ST;
                        else
                            current_state <= current_state;
                        end if;
                    
                    when EVENT_GEN_ST => 
                        if duration_reg = (DURATION-1) then 
                            current_state <= IDLE_ST;
                        else
                            current_state <= current_state;
                        end if;

                    when others => 
                        current_state <= current_state;

                end case;
            end if;
        end if;
    end process;


end irq_generator_arch;
