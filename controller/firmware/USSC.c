#define F_CPU					14745600

#define BAUD_USARTD0			115200
#define USARTD0_RX_BUFFER_SIZE	32
#define COMMAND_SIZE			6

#define BAUDRATE_TWIC 100000
#define TWI_BAUD(F_SYS, F_TWI) ((F_SYS / (2 * F_TWI)) - 5)
#define TWI_BAUDSETTING TWI_BAUD(F_CPU, BAUDRATE_TWIC)

#define STOP_UNLOCK 0x5F3759DF // 1597463007

// chip data
// abstract type: "well" can be crystal or compartment
#define MAX_NUM_WELL 60
#define MAX_NUM_LINE 60
#define STEP_STEP_DELAY 2		
#define STEP_STEP_DELAY_HALF 2
#define DIRECTION_STEP_DELAY 2

/////////////////// I2C addresses
#define PCA9557_FRONT_PUSHBUTTONS  0x32 // = 0x30 (fixed offset, see datasheet) | (0x01 (PCB configuration) << 1)
#define PCA9557_FRONT_LEDS         0x34 // = 0x30 (fixed offset, see datasheet) | (0x02 (PCB configuration) << 1)
#define PCA9557_ENA_ONOFF          0x36 // = 0x30 (fixed offset, see datasheet) | (0x03 (PCB configuration) << 1)
#define PCA9557_CALIB              0x38 // = 0x30 (fixed offset, see datasheet) | (0x04 (PCB configuration) << 1)
#define PCA9557_LIM                0x3A // = 0x30 (fixed offset, see datasheet) | (0x05 (PCB configuration) << 1)
#define PCA9557_BUSY               0x3C // = 0x30 (fixed offset, see datasheet) | (0x06 (PCB configuration) << 1)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

typedef enum {read, write} command_dir_t;
typedef enum {off, simple_trigger_enabled, simple_trigger_disabled, sync_trigger_enabled, sync_trigger_disabled} timing_mode_t;
typedef enum {no_axis, axis_1, axis_2, axis_3} axis_t;
typedef enum {stop, idle, single_pos, single_crystal, scan_crystals, homing} state_t;

typedef struct {
	uint32_t nominal_crys_pitch_well;
	uint32_t nominal_crys_pitch_line;
	uint32_t nominal_comp_pitch_well;
	uint32_t nominal_comp_pitch_line;
} chip_data_t;

typedef struct {
	int32_t desired;
	int32_t controller;
	int32_t encoder;
} axis_position_t;

typedef struct {
	chip_data_t chip_data;
	int32_t upper_limit;
	int32_t lower_limit;
} axis_data_t;

typedef struct {
	int32_t axis_1;
	int32_t axis_2;
	int32_t axis_3;
} position_t;

typedef struct {
	
	// scan data
	uint8_t first_scan_well;
	uint8_t first_scan_line;
	uint8_t last_scan_well;
	uint8_t last_scan_line;
		
	// indices
	uint8_t i_well;
	uint8_t i_line;
	
	// position array, calculated from pos_A, pos_B and pos_C
	//position_t position_offset;
	position_t position_line[MAX_NUM_WELL];
	position_t position_well[MAX_NUM_LINE];
			
} scan_data_t;

typedef struct {
	// A and B: same line index
	// B and C: same well index
	position_t position;
	//position_t position_line;
	uint8_t crys_well_index;
	uint8_t crys_line_index;
	uint8_t comp_well_index;
	uint8_t comp_line_index;
} feature_t;
/*
typedef struct {
	
} comp;*/

typedef struct {
	uint32_t t_post_pos;
	uint32_t t_exposure;
	uint32_t t_post_exp;
	uint32_t t_min_rep;
} timing_data_t;

typedef struct {
	double v1;
	double v2;
	double v3;
} vector_t;

/*
typedef struct {
	uint8_t homing_dir;
	uint8_t start_dir;
} homing_t;*/

// ************ global variables ********************

uint32_t	sdc2_status;

state_t state = stop;
uint8_t dir = 0;

axis_position_t axis_1_pos = {0, 0, 0};
axis_position_t axis_2_pos = {0, 0, 0};
axis_position_t axis_3_pos = {0, 0, 0};
axis_data_t axis_1_data;
axis_data_t axis_2_data;
axis_data_t axis_3_data;

position_t position_offset;
scan_data_t sd_crys;
scan_data_t sd_comp;
feature_t feat_alpha;
feature_t feat_beta;
feature_t feat_gamma;
feature_t feat_custom;


timing_mode_t timing_mode = simple_trigger_enabled;
timing_data_t td_us;
timing_data_t td_cnt;

uint8_t		USARTD0_Rx_buffer_index = 0;
uint8_t		USARTD0_Rx_buffer_offset = 0;
uint8_t		USARTD0_Rx_buffer[USARTD0_RX_BUFFER_SIZE];

// **************************************************

void set16MhzExternalOsc()
{
	//12-16MHz external crystal
	OSC_XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc;	  
	//Enable external oscillator
	OSC_CTRL |= OSC_XOSCEN_bm;	  
	//Wait for clock stabilization
	while(!(OSC_STATUS & OSC_XOSCRDY_bm));	  
	// Selects clock system as external clock through change protection mechanism
	CCP = CCP_IOREG_gc;
	CLK_CTRL = CLK_SCLKSEL_XOSC_gc;	
	return;
}

void send_USARTD0(uint8_t data)
{
	while (!( USARTD0.STATUS & USART_DREIF_bm)); //wait
	USARTD0.DATA = data;
	return;
}

void configurePorts() //according to scheme
{
	//Bit set HIGH -> Output
	PORTA.DIRSET = 0xF0;//Detector(s) signals
	PORTB.DIRSET = 0x08;//REACHED signals, PB3: Debug Pin
	PORTC.DIRSET = 0x1C;//SPI signals
	PORTD.DIRSET = 0xF3;//STEP and DIR signals
	PORTE.DIRSET = 0x08;//PE4: Debug Pin, rest HOME signals
	return;
}

void configureTwiCMaster(void)
{
	TWIC.MASTER.CTRLA  = TWI_MASTER_INTLVL_OFF_gc | TWI_MASTER_RIEN_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_ENABLE_bm;
	TWIC.MASTER.BAUD   = TWI_BAUDSETTING;
	TWIC.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;	
}

void configurePCA9557s()
{

	TWIC.MASTER.ADDR = PCA9557_FRONT_PUSHBUTTONS;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x03; // Configuration register
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0xFF; // 0xFF : all inputs
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;

	TWIC.MASTER.ADDR = PCA9557_FRONT_LEDS;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x03; // Configuration register
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x00; // 0x00 : all outputs
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	
	TWIC.MASTER.ADDR = PCA9557_ENA_ONOFF;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x03; // Configuration register
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x00; // 0x00 : all outputs
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	
	TWIC.MASTER.ADDR = PCA9557_CALIB;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x03; // Configuration register
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x00; // 0x00 : all outputs
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;

	TWIC.MASTER.ADDR = PCA9557_LIM;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x03; // Configuration register
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0xFF; // 0xFF : all inputs
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	
	TWIC.MASTER.ADDR = PCA9557_BUSY;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x03; // Configuration register
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0xFF; // 0xFF : all inputs
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;

}

void configureUSART()
{
	// set speed
	USARTD0.BAUDCTRLB = (uint8_t) ((F_CPU / (16UL * BAUD_USARTD0)) - 1) >> 8;
	USARTD0.BAUDCTRLA = (uint8_t) (F_CPU / (16UL * BAUD_USARTD0)) - 1;
	// 8 bit
	USARTD0.CTRLC = USART_CHSIZE_8BIT_gc;
	// set in an as out
	PORTD.DIRCLR = PIN2_bm;
	PORTD.OUTCLR = PIN2_bm;
	PORTD.DIRSET = PIN3_bm;
	PORTD.OUTSET = PIN3_bm;
	// Rx interrupt med en, Tx int dis, DRE dis
	USARTD0.CTRLA = USART_RXCINTLVL_HI_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	// enable Tx, enable Rx
	USARTD0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
	return;
}

// TCNT0 is used for status request
void configureTCNT0(void) {
	TCC0.CTRLA |= TC_CLKSEL_DIV1024_gc;
	TCC0.PER = 240; // F_CPU / 1024 / 240 = 60 Hz
	TCC0.CNT = 0x00;
	TCC0.INTCTRLA |= TC_OVFINTLVL_LO_gc; // low level interrupt
	return;
}

// TCNT1 is used for detector timing
void configureTCNT1(void) {
	TCC1.CTRLA |= TC_CLKSEL_DIV1024_gc;
	TCC1.CNT = 0x000;
	return;
}

void read_twi(uint8_t address, uint8_t *DataRead){
	
	TWIC.MASTER.ADDR = address;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x00; // send register address
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	address |= 0x01;
	TWIC.MASTER.ADDR = address; // send read command
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_RIF_bm));
	TWIC.MASTER.CTRLC = 0x06;

	address = address & 0xFE;//delete read bit
	if(address ==  PCA9557_FRONT_PUSHBUTTONS || address ==  PCA9557_LIM || address ==  PCA9557_BUSY) {
	//iron PCA9557 readout (output has nibble-wise different polarity)
		*DataRead = TWIC.MASTER.DATA  ^ 0xF0;
	}
	else
		*DataRead = TWIC.MASTER.DATA;
	//	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
	_delay_us(10);
	return;
}

void write_twi(uint8_t address, uint8_t DataToWrite){
	
	TWIC.MASTER.ADDR = address;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = 0x01;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.DATA = DataToWrite;
	while(!(TWIC.MASTER.STATUS&TWI_MASTER_WIF_bm));
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
}

void enableInterrupts()
{
	// enable all level interrupts
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	
	// enable global interrupt
	sei();
	return;
}

void transmit_data(uint8_t command, uint32_t data)
{
	uint8_t * ptr;
	send_USARTD0(command);
	send_USARTD0(~command);
	ptr = (uint8_t*)&data;
	send_USARTD0(ptr[0]);
	send_USARTD0(ptr[1]);
	send_USARTD0(ptr[2]);
	send_USARTD0(ptr[3]);
	return;
}

inline uint8_t all_axes_reached(void)
{
	return (PORTB.IN & 0x07) ? 0 : 1;
}

void axis_set_desried_position(axis_t axis, int32_t position)
{
	if (axis == axis_1) axis_1_pos.desired = position;
	if (axis == axis_2) axis_2_pos.desired = position;
	if (axis == axis_3) axis_3_pos.desired = position;
	return;
}

void axis_get_desired_position(axis_t axis, int32_t* position)
{
	if (axis == axis_1) *position = axis_1_pos.desired;
	if (axis == axis_2) *position = axis_2_pos.desired;
	if (axis == axis_3) *position = axis_3_pos.desired;
	return;
}

void axis_set_controller_position(axis_t axis, int32_t position)
{
	if (axis == axis_1) axis_1_pos.controller = position;
	if (axis == axis_2) axis_2_pos.controller = position;
	if (axis == axis_3) axis_3_pos.controller = position;
	return;
}

void axis_get_controller_position(axis_t axis, int32_t* position)
{
	if (axis == axis_1) *position = axis_1_pos.controller;
	if (axis == axis_2) *position = axis_2_pos.controller;
	if (axis == axis_3) *position = axis_3_pos.controller;
	return;
}

void axis_get_encoder_position(axis_t axis, int32_t* position)
{
	if (axis == axis_1) *position = axis_1_pos.encoder;
	if (axis == axis_2) *position = axis_2_pos.encoder;
	if (axis == axis_3) *position = axis_3_pos.encoder;
	return;
}

void axis_set_chip_data(axis_t axis, uint32_t data)
{
	chip_data_t* cd;
	uint8_t sub_cmd = (((uint8_t*)(&data))[3]) >> 4;
	data &= 0x0FFFFFFF;
	
	if (axis == no_axis) return;
	if (axis == axis_1) cd = &axis_1_data.chip_data;
	if (axis == axis_2) cd = &axis_2_data.chip_data;
	if (axis == axis_3) cd = &axis_3_data.chip_data;
	
	if (sub_cmd == 0x00) cd->nominal_crys_pitch_well = data;
	if (sub_cmd == 0x01) cd->nominal_crys_pitch_line = data;
	if (sub_cmd == 0x02) cd->nominal_comp_pitch_well = data;
	if (sub_cmd == 0x03) cd->nominal_comp_pitch_line = data;
	return;
}

void axis_set_upper_limit(axis_t axis, int32_t limit)
{
	if (axis == axis_1) axis_1_data.upper_limit = limit;
	if (axis == axis_2) axis_2_data.upper_limit = limit;
	if (axis == axis_3) axis_3_data.upper_limit = limit;
	return;
}

void axis_set_lower_limit(axis_t axis, int32_t limit)
{
	if (axis == axis_1) axis_1_data.lower_limit = limit;
	if (axis == axis_2) axis_2_data.lower_limit = limit;
	if (axis == axis_3) axis_3_data.lower_limit = limit;
	return;
}

void set_timing_data(uint32_t data)
{
	uint8_t sub_cmd = (((uint8_t*)(&data))[3]); // MSB
	data &= 0x00FFFFFF;
	if (sub_cmd == 0x00) td_us.t_post_pos = data;
	if (sub_cmd == 0x01) td_us.t_exposure = data;
	if (sub_cmd == 0x02) td_us.t_post_exp = data;
	if (sub_cmd == 0x03) td_us.t_min_rep  = data;
	
	return;
}

void convert_timing_data(void)
{
	double buf = F_CPU / 1024 / 1e6;
	
	td_cnt.t_post_pos = buf * td_us.t_post_pos;
	td_cnt.t_exposure = buf * td_us.t_exposure;
	td_cnt.t_post_exp = buf * td_us.t_post_exp;
	td_cnt.t_min_rep = buf * td_us.t_min_rep;
	
	return;
}

vector_t normalize_vector(vector_t vec)
{
	vector_t ret_vec;
	double norm = sqrt((vec.v1 * vec.v1) + (vec.v2 * vec.v2) + (vec.v3 * vec.v3));
	
	ret_vec.v1 = vec.v1 / norm;
	ret_vec.v2 = vec.v2 / norm;
	ret_vec.v3 = vec.v3 / norm;
		
	return ret_vec;
}

void calculate_crystal_positions(void)
{
	int32_t delta_feature_comp_well	   = feat_beta.comp_well_index - feat_alpha.comp_well_index;
	int32_t delta_feature_comp_line    = feat_gamma.comp_line_index - feat_beta.comp_line_index;
		
	int32_t delta_position_axis_1_well = feat_beta.position.axis_1 - feat_alpha.position.axis_1;
	int32_t delta_position_axis_2_well = feat_beta.position.axis_2 - feat_alpha.position.axis_2;
	int32_t delta_position_axis_3_well = feat_beta.position.axis_3 - feat_alpha.position.axis_3;
	
	int32_t delta_position_axis_1_line = feat_gamma.position.axis_1 - feat_beta.position.axis_1;
	int32_t delta_position_axis_2_line = feat_gamma.position.axis_2 - feat_beta.position.axis_2;
	int32_t delta_position_axis_3_line = feat_gamma.position.axis_3 - feat_beta.position.axis_3;
	
	if (delta_feature_comp_well == 0) return;
	if (delta_feature_comp_line == 0) return;
	
	position_offset.axis_1 = feat_alpha.position.axis_1;
	position_offset.axis_2 = feat_alpha.position.axis_2;
	position_offset.axis_3 = feat_alpha.position.axis_3;
	
	for (sd_comp.i_well = 0; sd_comp.i_well < MAX_NUM_LINE; sd_comp.i_well++) {
		sd_comp.position_well[sd_comp.i_well].axis_1 = (delta_position_axis_1_well * (sd_comp.i_well - feat_alpha.comp_well_index)) / delta_feature_comp_well;
		sd_comp.position_well[sd_comp.i_well].axis_2 = (delta_position_axis_2_well * (sd_comp.i_well - feat_alpha.comp_well_index)) / delta_feature_comp_well;
		sd_comp.position_well[sd_comp.i_well].axis_3 = (delta_position_axis_3_well * (sd_comp.i_well - feat_alpha.comp_well_index)) / delta_feature_comp_well;
			
	}
	for (sd_comp.i_line = 0; sd_comp.i_line < MAX_NUM_WELL; sd_comp.i_line++) {
		sd_comp.position_line[sd_comp.i_line].axis_1 = (delta_position_axis_1_line * (sd_comp.i_line - feat_alpha.comp_well_index)) / delta_feature_comp_line;
		sd_comp.position_line[sd_comp.i_line].axis_2 = (delta_position_axis_2_line * (sd_comp.i_line - feat_alpha.comp_well_index)) / delta_feature_comp_line;
		sd_comp.position_line[sd_comp.i_line].axis_3 = (delta_position_axis_3_line * (sd_comp.i_line - feat_alpha.comp_well_index)) / delta_feature_comp_line;
	}
	
	vector_t well;
	vector_t line;
	
	well.v1 = feat_beta.position.axis_1 - feat_alpha.position.axis_1;
	well.v2 = feat_beta.position.axis_2 - feat_alpha.position.axis_2;
	well.v3 = feat_beta.position.axis_3 - feat_alpha.position.axis_3;
	well = normalize_vector(well);
	well.v1 *= axis_1_data.chip_data.nominal_crys_pitch_well;
	well.v2 *= axis_2_data.chip_data.nominal_crys_pitch_well;
	well.v3 *= axis_3_data.chip_data.nominal_crys_pitch_well;
	
	line.v1 = feat_gamma.position.axis_1 - feat_beta.position.axis_1;
	line.v2 = feat_gamma.position.axis_2 - feat_beta.position.axis_2;
	line.v3 = feat_gamma.position.axis_3 - feat_beta.position.axis_3;
	line = normalize_vector(line);
	line.v1 *= axis_1_data.chip_data.nominal_crys_pitch_line;
	line.v2 *= axis_2_data.chip_data.nominal_crys_pitch_line;
	line.v3 *= axis_3_data.chip_data.nominal_crys_pitch_line;
	
	for (sd_crys.i_well = 0; sd_crys.i_well < MAX_NUM_LINE; sd_crys.i_well++) {
		sd_crys.position_well[sd_crys.i_well].axis_1 = (sd_crys.i_well - feat_alpha.crys_well_index) * well.v1;
		sd_crys.position_well[sd_crys.i_well].axis_2 = (sd_crys.i_well - feat_alpha.crys_well_index) * well.v2;
		sd_crys.position_well[sd_crys.i_well].axis_3 = (sd_crys.i_well - feat_alpha.crys_well_index) * well.v3;
		
			
	}
	for (sd_crys.i_line = 0; sd_crys.i_line < MAX_NUM_WELL; sd_crys.i_line++) {
		sd_crys.position_line[sd_crys.i_line].axis_1 = (sd_crys.i_line - feat_alpha.crys_line_index) * line.v1;
		sd_crys.position_line[sd_crys.i_line].axis_2 = (sd_crys.i_line - feat_alpha.crys_line_index) * line.v2;
		sd_crys.position_line[sd_crys.i_line].axis_3 = (sd_crys.i_line - feat_alpha.crys_line_index) * line.v3;
	}
	
	
	return;
}

// sets direction bit for step/dir output
void axes_set_directions(void)
{
	/*
	if (axis_1_pos.controller < axis_1_pos.desired) PORTD.OUTSET = PIN5_bm;
	if (axis_1_pos.controller > axis_1_pos.desired) PORTD.OUTCLR = PIN5_bm;
	
	if (axis_2_pos.controller < axis_2_pos.desired) PORTD.OUTSET = PIN6_bm;
	if (axis_2_pos.controller > axis_2_pos.desired) PORTD.OUTCLR = PIN6_bm;
	
	if (axis_3_pos.controller < axis_3_pos.desired) PORTD.OUTSET = PIN7_bm;
	if (axis_3_pos.controller > axis_3_pos.desired) PORTD.OUTCLR = PIN7_bm;
	*/

	return;
}

void single_step_back_forth(void)
{
	PORTD.OUTSET = PIN5_bm | PIN6_bm | PIN7_bm; // set dir pos;
	PORTD.OUTSET = PIN0_bm | PIN1_bm | PIN4_bm; // single step
	PORTD.OUTCLR = PIN0_bm | PIN1_bm | PIN4_bm; // single step
	//PORTD.OUTSET = PIN0_bm | PIN1_bm | PIN4_bm; // single step

	PORTD.OUTCLR = PIN5_bm | PIN6_bm | PIN7_bm; // set dir neg;
	PORTD.OUTSET = PIN0_bm | PIN1_bm | PIN4_bm; // single step back
	PORTD.OUTCLR = PIN0_bm | PIN1_bm | PIN4_bm; // single step back
	//PORTD.OUTSET = PIN0_bm | PIN1_bm | PIN4_bm; // single step back
	
	return;
}

void poll_step_dir()
{
	if ((axis_1_pos.desired > axis_1_data.upper_limit) || (axis_1_pos.desired < axis_1_data.lower_limit) ||
	(axis_2_pos.desired > axis_2_data.upper_limit) || (axis_2_pos.desired < axis_2_data.lower_limit) ||
	(axis_3_pos.desired > axis_3_data.upper_limit) || (axis_3_pos.desired < axis_3_data.lower_limit)) {
		state = stop;
	}
		
	if (state == stop) return;
		
	// ************ axis 1 *************
	
	
	//_delay_us(STEP_STEP_DELAY);
	_delay_us(DIRECTION_STEP_DELAY);
	while (axis_1_pos.controller != axis_1_pos.desired) {
		if (state == stop) return;
		
		/*
		PORTD.OUTCLR = PIN0_bm;
		_delay_us(STEP_STEP_DELAY_HALF);
		PORTD.OUTSET = PIN0_bm;
		_delay_us(STEP_STEP_DELAY_HALF);
		*/
		
		PORTD.OUTSET = PIN0_bm;
		if (axis_1_pos.controller < axis_1_pos.desired) PORTD.OUTSET = PIN5_bm;
		else PORTD.OUTCLR = PIN5_bm;
		_delay_us(STEP_STEP_DELAY_HALF);
		PORTD.OUTCLR = PIN0_bm;
		if (axis_1_pos.controller < axis_1_pos.desired) axis_1_pos.controller++;
		else axis_1_pos.controller--;
		
	}
	//PORTD.OUTCLR = PIN5_bm;
	
	// ************ axis 2 *************

	//_delay_us(STEP_STEP_DELAY);
	_delay_us(DIRECTION_STEP_DELAY);
	while (axis_2_pos.controller != axis_2_pos.desired) {
		if (state == stop) return;
		
		/*
		PORTD.OUTCLR = PIN1_bm;
		_delay_us(STEP_STEP_DELAY_HALF);
		PORTD.OUTSET = PIN1_bm;
		_delay_us(STEP_STEP_DELAY_HALF);
		*/
		
		PORTD.OUTSET = PIN1_bm;
		if (axis_2_pos.controller < axis_2_pos.desired) PORTD.OUTSET = PIN6_bm;
		else PORTD.OUTCLR = PIN6_bm;
		_delay_us(STEP_STEP_DELAY_HALF);
		PORTD.OUTCLR = PIN1_bm;
		if (axis_2_pos.controller < axis_2_pos.desired) axis_2_pos.controller++;
		else axis_2_pos.controller--;
	}
	
	// ************ axis 3 *************
	
	//_delay_us(STEP_STEP_DELAY);
	_delay_us(DIRECTION_STEP_DELAY);
	while (axis_3_pos.controller != axis_3_pos.desired) {
		if (state == stop) return;
		
		/*
		PORTD.OUTCLR = PIN4_bm;
		_delay_us(STEP_STEP_DELAY_HALF);
		PORTD.OUTSET = PIN4_bm;
		_delay_us(STEP_STEP_DELAY_HALF);
		*/
		
		
		PORTD.OUTSET = PIN4_bm;
		if (axis_3_pos.controller < axis_3_pos.desired) PORTD.OUTSET = PIN7_bm;
		else PORTD.OUTCLR = PIN7_bm;
		_delay_us(STEP_STEP_DELAY_HALF);
		PORTD.OUTCLR = PIN4_bm;
		if (axis_3_pos.controller < axis_3_pos.desired) axis_3_pos.controller++;
		else axis_3_pos.controller--;
	}

	_delay_us(STEP_STEP_DELAY);
	return;
}

inline void set_trigger_out(void)
{
	PORTA.OUTSET = PIN4_bm;
	PORTA.OUTCLR = PIN5_bm;
	return;
}

inline void clear_trigger_out(void)
{
	PORTA.OUTCLR = PIN4_bm;
	PORTA.OUTSET = PIN5_bm;
	return;
}

void start_simple_timing_sequence(uint8_t enable_trigger)
{
	uint16_t time_1 = td_cnt.t_post_pos;
	uint16_t time_2 = td_cnt.t_post_pos + td_cnt.t_exposure;
	uint16_t time_3 = td_cnt.t_post_pos + td_cnt.t_exposure + td_cnt.t_post_exp;
	
		
	while (TCC1.CNT < td_cnt.t_min_rep) ; // wait
		
	TCC1.CNT = 0x0000; // reset timer
		
	while (TCC1.CNT < time_1) ; // wait
	if (enable_trigger) set_trigger_out();
	while (TCC1.CNT < time_2) ; // wait
	clear_trigger_out();
	while (TCC1.CNT < time_3) ; // wait
	
	return;
}

void start_synchronized_timing_sequence(uint8_t enable_trigger)
{
	uint16_t time_1 = td_cnt.t_post_pos;
		
	while (TCC1.CNT < td_cnt.t_min_rep) ; // wait
	
	TCC1.CNT = 0x0000; // reset timer
	
	while (TCC1.CNT < time_1) ; // wait
	if (enable_trigger) set_trigger_out();
	while ((PORTA.IN & PIN0_bm) == 0) ; // wait until IN1 gets HI
	clear_trigger_out();
	while ((PORTA.IN & PIN0_bm) != 0) ; // wait until IN1 gets LO
	
	return;
}

void start_timing_sequence(void)
{
	if (timing_mode == simple_trigger_enabled)
		start_simple_timing_sequence(1);
	if (timing_mode == simple_trigger_disabled)
		start_simple_timing_sequence(0);
	if (timing_mode == sync_trigger_enabled)
		start_synchronized_timing_sequence(1);
	if (timing_mode == sync_trigger_disabled)
		start_synchronized_timing_sequence(0);
	return;
}

void goto_single_position(void)
{
	poll_step_dir();
	
	if (state != stop) state = idle;
	
	return;
}

void goto_single_crystal(void)
{
	axis_1_pos.desired = position_offset.axis_1
		+ sd_comp.position_well[feat_custom.comp_well_index].axis_1 + sd_comp.position_line[feat_custom.comp_line_index].axis_1
		+ sd_crys.position_well[feat_custom.crys_well_index].axis_1 + sd_crys.position_line[feat_custom.crys_line_index].axis_1;
		
	axis_2_pos.desired = position_offset.axis_2
		+ sd_comp.position_well[feat_custom.comp_well_index].axis_2 + sd_comp.position_line[feat_custom.comp_line_index].axis_2
		+ sd_crys.position_well[feat_custom.crys_well_index].axis_2 + sd_crys.position_line[feat_custom.crys_line_index].axis_2;
		
	axis_3_pos.desired = position_offset.axis_3
		+ sd_comp.position_well[feat_custom.comp_well_index].axis_3 + sd_comp.position_line[feat_custom.comp_line_index].axis_3
		+ sd_crys.position_well[feat_custom.crys_well_index].axis_3 + sd_crys.position_line[feat_custom.crys_line_index].axis_3;
	
	poll_step_dir();
	while (!all_axes_reached())
	{
		//if (state != single_crystal) return;
		if (state == stop) return;
	}
	if (state != stop) state = idle;				
	
	return;
}

void poll_scan_crystals(void)
{
	calculate_crystal_positions();
	
	for (sd_comp.i_line = sd_comp.first_scan_line; sd_comp.i_line <= sd_comp.last_scan_line; sd_comp.i_line++) {
		for (sd_comp.i_well = sd_comp.first_scan_well; sd_comp.i_well <= sd_comp.last_scan_well; sd_comp.i_well++) {
			for (sd_crys.i_line = sd_crys.first_scan_line; sd_crys.i_line <= sd_crys.last_scan_line; sd_crys.i_line++) {
				for (sd_crys.i_well = sd_crys.first_scan_well; sd_crys.i_well <= sd_crys.last_scan_well; sd_crys.i_well++) {
					if (!(sd_crys.i_line & 0x01)) {
						axis_1_pos.desired = position_offset.axis_1 
							+ sd_comp.position_well[sd_comp.i_well].axis_1 + sd_comp.position_line[sd_comp.i_line].axis_1 
							+ sd_crys.position_well[sd_crys.i_well].axis_1 + sd_crys.position_line[sd_crys.i_line].axis_1;
						
						axis_2_pos.desired = position_offset.axis_2
							+ sd_comp.position_well[sd_comp.i_well].axis_2 + sd_comp.position_line[sd_comp.i_line].axis_2
							+ sd_crys.position_well[sd_crys.i_well].axis_2 + sd_crys.position_line[sd_crys.i_line].axis_2;
						
						axis_3_pos.desired = position_offset.axis_3
							+ sd_comp.position_well[sd_comp.i_well].axis_3 + sd_comp.position_line[sd_comp.i_line].axis_3
							+ sd_crys.position_well[sd_crys.i_well].axis_3 + sd_crys.position_line[sd_crys.i_line].axis_3;
					} else {
						axis_1_pos.desired = position_offset.axis_1
						+ sd_comp.position_well[sd_comp.i_well].axis_1 + sd_comp.position_line[sd_comp.i_line].axis_1
						+ sd_crys.position_well[sd_crys.last_scan_well - sd_crys.i_well].axis_1 + sd_crys.position_line[sd_crys.i_line].axis_1;
						
						axis_2_pos.desired = position_offset.axis_2
						+ sd_comp.position_well[sd_comp.i_well].axis_2 + sd_comp.position_line[sd_comp.i_line].axis_2
						+ sd_crys.position_well[sd_crys.last_scan_well - sd_crys.i_well].axis_2 + sd_crys.position_line[sd_crys.i_line].axis_2;
						
						axis_3_pos.desired = position_offset.axis_3
						+ sd_comp.position_well[sd_comp.i_well].axis_3 + sd_comp.position_line[sd_comp.i_line].axis_3
						+ sd_crys.position_well[sd_crys.last_scan_well - sd_crys.i_well].axis_3 + sd_crys.position_line[sd_crys.i_line].axis_3;
					}
								
					poll_step_dir();
					// wait
					while (!all_axes_reached()) {
						if (state != scan_crystals) return;
					}
					start_timing_sequence();
				}
			}
		}
	}
	if (state != stop) state = idle;
	return;
}

void poll_axes(void)
{
	if (state == stop) return;
	if (state == idle) return;
	
	if (state == single_pos) {
		goto_single_position();
		return;
	}
	
	if (state == single_crystal) {
		calculate_crystal_positions();
		goto_single_crystal();
		return;
	}
	
	if (state == scan_crystals) {
		poll_scan_crystals();
		return;	
	}
	return;	
}

void set_scan_parameter(scan_data_t *sd, uint32_t* data)
{
	if (state == scan_crystals) return; // must not change parameters during scan
	sd->first_scan_well = ((uint8_t*)data)[0];
	sd->first_scan_line = ((uint8_t*)data)[1];
	sd->last_scan_well = ((uint8_t*)data)[2];
	sd->last_scan_line = ((uint8_t*)data)[3];
	return;
}

void set_single_crystal_parameter(uint32_t *data)
{
	feat_custom.crys_well_index = ((uint8_t*)data)[0];
	feat_custom.crys_line_index    = ((uint8_t*)data)[1];
	feat_custom.comp_well_index = ((uint8_t*)data)[2];
	feat_custom.comp_line_index    = ((uint8_t*)data)[3];
	
	return;
}

void make_controller_position_feature(feature_t* feature, uint32_t* data)
{
	feature->crys_well_index	= ((uint8_t*)data)[0];
	feature->crys_line_index	= ((uint8_t*)data)[1];
	feature->comp_well_index = ((uint8_t*)data)[2];
	feature->comp_line_index	= ((uint8_t*)data)[3];
	feature->position.axis_1 = axis_1_pos.controller;
	feature->position.axis_2 = axis_2_pos.controller;
	feature->position.axis_3 = axis_3_pos.controller;
	return;
}

void poll_command(uint8_t command, uint32_t data)
{
	uint32_t tx_value;
	command_dir_t cmd_dir;
	axis_t axis = no_axis;
	
	tx_value = 0;
	cmd_dir = (command & 0x80) ? read : write;
	switch (command & 0x60) {
		case 0x20: axis = axis_1;  break;
		case 0x40: axis = axis_2;  break;
		case 0x60: axis = axis_3;  break;
	}
		
	switch (command & 0x1F) {
		
		// status
		case 0x00: 
		if (cmd_dir == read) tx_value = sdc2_status;
		break;
		
		// emergency stop or unlock
		case 0x01:
		state = stop;
		if ((cmd_dir == write) && (data == STOP_UNLOCK)) state = idle;
		tx_value = (uint32_t)state;
		break;
		
		// set controller state
		case 0x02:
		if ((cmd_dir == write) && (state != stop)) state = (state_t)data;
		tx_value = (uint32_t)state;
		break;
				
		// read/write desired position
		case 0x03:
		if (cmd_dir == write) axis_set_desried_position(axis, *((int32_t*)&data));
		axis_get_desired_position(axis, (int32_t*)&tx_value);
		break;
		
		// read/write controller position
		case 0x04:
		if (cmd_dir == write) axis_set_controller_position(axis, *((int32_t*)&data));
		axis_get_controller_position(axis, (int32_t*)&tx_value);
		break;
		
		// read encoder position
		case 0x05:
		axis_get_encoder_position(axis, (int32_t*)&tx_value);
		break;
		
		// set to zero
		case 0x06:
		if (cmd_dir == write) {
			axis_1_pos.controller = axis_1_pos.desired = 0;
			axis_2_pos.controller = axis_2_pos.desired = 0;
			axis_3_pos.controller = axis_3_pos.desired = 0;
		}
		break;
		
		// single step for and back all directions
		case 0x07:
		if ((cmd_dir == write) && (state == idle)) {
			single_step_back_forth();
		}
		break;
		
		// set crystal scan parameter
		case 0x08:
		if (cmd_dir == write) set_scan_parameter(&sd_crys, &data);
		break;
		
		// set compartment scan parameter
		case 0x09:
		if (cmd_dir == write) set_scan_parameter(&sd_comp, &data);
		break;
		
		// make current position posABC
		case 0x0A:
		if (cmd_dir == write) make_controller_position_feature(&feat_alpha, &data);
		break;
		
		case 0x0B:
		if (cmd_dir == write) make_controller_position_feature(&feat_beta , &data);
		break;
		
		case 0x0C:
		if (cmd_dir == write) make_controller_position_feature(&feat_gamma, &data);
		break;
		
		/*// set nominal chip data --> OLD??
		case 0x0D:
		if (cmd_dir == write) axis_set_chip_data(axis, data);
		break;*/
		
		case 0x0D:
		if (cmd_dir == write)
			timing_mode= (timing_mode_t)data;
		clear_trigger_out();
		break;
								
		// set timing data
		case 0x0E:
		if (cmd_dir == write) {
			set_timing_data(data);
			convert_timing_data();
		}
		break;
		
		// trigger manually
		case 0x0F:
		start_timing_sequence();
		break;
		
		// calculate crystal positions
		case 0x10:
		if (cmd_dir == write) calculate_crystal_positions();
		break;
		
		// set goto position for single crystal
		case 0x1A:
		if (cmd_dir == write) set_single_crystal_parameter(&data);
		break;
		
		// set single position, goto, wait
		case 0x1B:
		if ((cmd_dir == write) && (state == idle)) {
			set_single_crystal_parameter(&data);
			goto_single_crystal();
			//start_timing_sequence();
		}
		break;
		
		
		// set single position, goto, wait and trigger (and wait)
		case 0x1C:
		if ((cmd_dir == write) && (state == idle)) {
			set_single_crystal_parameter(&data);
			goto_single_crystal();
			start_timing_sequence();
		}
		break;
		
		// set chip data
		case 0x1D:
		if (cmd_dir == write)
			axis_set_chip_data(axis, data);
		break;
						
		// set upper limits
		case 0x1E:
		if (cmd_dir == write) axis_set_upper_limit(axis, *((int32_t*)&data));
		break;
			
		// set lower limits
		case 0x1F:
		if (cmd_dir == write) axis_set_lower_limit(axis, *((int32_t*)&data));
		break;
	}
		
	transmit_data(command, tx_value);
	
	return;
}

ISR(USARTD0_RXC_vect)
{
	uint8_t  USART_Rx_command;
	uint32_t USART_RX_data;
	
	USARTD0_Rx_buffer[USARTD0_Rx_buffer_index] = USARTD0.DATA;
	USARTD0_Rx_buffer_index++;
	USARTD0_Rx_buffer_index %= USARTD0_RX_BUFFER_SIZE;
	
	if (USARTD0_Rx_buffer_offset > USARTD0_Rx_buffer_index)
	USARTD0_Rx_buffer_offset = USARTD0_Rx_buffer_index;
	
	while (USARTD0_Rx_buffer_index - USARTD0_Rx_buffer_offset >= COMMAND_SIZE)
	{
		if ((USARTD0_Rx_buffer[USARTD0_Rx_buffer_offset] + USARTD0_Rx_buffer[USARTD0_Rx_buffer_offset + 1]) == 0xFF)
		{
			USART_Rx_command = USARTD0_Rx_buffer[USARTD0_Rx_buffer_offset];
			USART_RX_data = *((uint32_t*)(&USARTD0_Rx_buffer[USARTD0_Rx_buffer_offset + 2]));
			
			USARTD0_Rx_buffer_index = 0;
			USARTD0_Rx_buffer_offset = 0;
			
			poll_command(USART_Rx_command, USART_RX_data);
		}
		else
		{
			USARTD0_Rx_buffer_offset++;
		}
	}
	return;
}

void poll_status(void)
{
	uint8_t u01;
	uint32_t  status_axis_1, status_axis_2, status_axis_3, controller_state;
	
	u01 = 0;
	
	//retrieve SDC2 LIM+ and LIM- signals
	read_twi(PCA9557_LIM,  &u01);
	status_axis_1 = (u01 & 0x03) << 3;
	status_axis_2 = (u01 & 0x0C) << 1;
	status_axis_3 = (u01 & 0x30) >> 1;
	//retrieve SDC2 BUSY signals
	read_twi(PCA9557_BUSY, &u01);
	status_axis_1 = status_axis_1 | (u01 & 0x01) << 2;
	status_axis_2 = status_axis_2 | (u01 & 0x02) << 1;
	status_axis_3 = status_axis_3 | (u01 & 0x04);
	//retrieve SDC2 HOME signals
	u01 = PORTE.IN;
	status_axis_1 = status_axis_1 | (u01 & 0x01) << 1;
	status_axis_2 = status_axis_2 | (u01 & 0x02);
	status_axis_3 = status_axis_3 | (u01 & 0x04) >> 1;
	//retrieve SDC2 REACHED signals
	u01 = PORTB.IN;
	status_axis_1 = status_axis_1 | (u01 & 0x01);
	status_axis_2 = status_axis_2 | (u01 & 0x02) >> 1;
	status_axis_3 = status_axis_3 | (u01 & 0x04) >> 2;
	
	// bit 0..2 have to be inverted
	status_axis_1 ^= 0x07;
	status_axis_2 ^= 0x07;
	status_axis_3 ^= 0x07;
	
	status_axis_2 = status_axis_2 <<  8;
	status_axis_3 = status_axis_3 << 16;
	
	controller_state = ((uint32_t)state) << 24;
	
	sdc2_status = status_axis_1 | status_axis_2 | status_axis_3 | controller_state;

}

ISR(TCC0_OVF_vect)
{
	poll_status();
//	check_limits();
	return;
}

/*
ISR(TCC1_CCA_vect)
{
	PORTA.OUTSET = PIN4_bm;
	PORTA.OUTCLR = PIN4_bm;
	PORTA.OUTSET = PIN4_bm;
	return;
}

ISR(TCC1_CCB_vect)
{
	PORTA.OUTCLR = PIN4_bm;
	PORTA.OUTSET = PIN4_bm;
	PORTA.OUTCLR = PIN4_bm;
	TCC1.INTCTRLB = 0; // disable TCC1 interrupt
	return;
}*/

void initializeSequence(void)
{
	_delay_ms(500);
	set16MhzExternalOsc();
	configurePorts();
	configureUSART();
	configureTCNT0();
	configureTCNT1();
	configureTwiCMaster();
	configurePCA9557s();
	write_twi(PCA9557_ENA_ONOFF, 0xFF);// clock input enabled, high voltage enabled
	poll_status();
	_delay_ms(100);
	enableInterrupts();
	_delay_ms(100);
	return;
}

int main(void)
{

    initializeSequence();
	
	//Initiate global variables
	sdc2_status = 0;
	
	
	//Directions
	PORTD.OUTCLR = PIN5_bm;
	PORTD.OUTCLR = PIN6_bm;
	PORTD.OUTCLR = PIN7_bm;
	
	// test line scan 
		
	while (1)
    {
		poll_axes();
	}
	
}


