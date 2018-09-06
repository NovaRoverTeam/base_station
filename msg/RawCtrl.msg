# Message for raw user controller input.

# Is the controller connected?
bool connected

# Have the start or back buttons been depressed since last call?
bool start_trg
bool back_trg

# Are the triggers currently down?
bool trig_l_dwn  # Left trigger down
bool trig_r_dwn  # Right trigger down

# Normalised trigger value (0 to 1)
float64 trig_l_val
float64 trig_r_val

# Are the bumpers currently down?
bool bump_l_dwn
bool bump_r_dwn 

# Have the A, B, X or Y buttons been depressed since last call?
bool but_x_trg  
bool but_y_trg
bool but_a_trg  
bool but_b_trg   

# Normalised stick axis values (-1 to 1)
float64 axis_lx_val
float64 axis_ly_val
float64 axis_rx_val  
float64 axis_ry_val 

int8 axis_dx_dwn  # D-pad x value, -1 left, 0 none/both or 1 right
int8 axis_dy_dwn  # D-pad y value, -1 down, 0 none/both or 1 up
