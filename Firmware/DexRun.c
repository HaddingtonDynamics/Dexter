//#define NO_BOOT_DANCE
//#define DEBUG_API
//#define DEBUG_XL320_UART //Printf coms with servoes
#define DEBUG_MONITOR 
//
#define DEG_ARCSEC 3600
#define MONITOR_MAX_VELOCITY (100*DEG_ARCSEC)
//actual max angular velocity should never be more than 50 degress per second. JW
#define MONITOR_MAX_ERROR 180
//multiplier for the number of degrees of error the raw encoder can be off. 
//180 basically disables it

// this is the SpeedsUpdate code 


#include <stddef.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <termios.h>
#include <inttypes.h>
#include <ctype.h>
//#include "test.h" //TODO: Make that work

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdbool.h>


//prototypes
int getNormalizedInput(int);
int RestoreCalTables(char *filename);
void ProcessServerReceiveData(char *recBuff);
bool ProcessServerReceiveDataDDE(char *recBuff);
bool ProcessServerSendData(char *recBuff);
bool ProcessServerSendDataDDE(char *sendBuff,char *recBuff);
int ParseInput(char *iString);
int MoveRobot(int a1,int a2,int a3,int a4,int a5, int mode);
int ReadDMA(int p1,int p2,char *p3);
int CheckBoundry(int* j1, int* j2, int* j3, int* j4, int* j5);
void SendPacket(unsigned char *TxPkt, int length, int TxRxTimeDelay, unsigned char *RxPkt, int ReadLength);
void KeyholeSend(int *DataArray, int controlOffset, int size, int entryOffset );
int CalcUartTimeout(int size);

#define SIGNEX(v, sb) ((v) | (((v) & (1 << (sb))) ? ~((1 << (sb))-1) : 0))

#define MAX_CONTENT_CHARS 62
//maybe could be 112, 4 * 32 bit integers and then 128 bytes in the socket

#define NUM_JOINTS 5
//how many standard joints are there? e.g. with encoders / drives / etc.. not including tool interface.

#define L1_TABLE 0x600000
#define L1_TABLE_LENGTH 0xa8000

#define L2_TABLE 0xa00000
#define L2_TABLE_LENGTH 0x52be0

#define L3_TABLE 0x800000
#define L3_TABLE_LENGTH 0x93f00

#define L5_TABLE 0xe00000
#define L5_TABLE_LENGTH 0xc4b60

#define L4_TABLE 0xc00000
#define L4_TABLE_LENGTH 0x6b0a0



#define INPUT_OFFSET 15

   // Motor position index
#define CMD_POSITION_KEYHOLE 0
#define CMD_POSITION_KEYHOLE_CMD 19 // s19 datatype
#define CMD_POSITION_KEYHOLE_SIZE 5

#define ACCELERATION_MAXSPEED 1  // made acceleration 12 bits

#define ADC_CENTERS_KEYHOLE 2
#define ADC_CENTERS_KEYHOLE_CMD 29 // fixedpoint 13.16 datatype
#define ADC_CENTERS_KEYHOLE_SIZE 10  // sin/cos 5 axis


#define PID_P 3
#define PID_ADDRESS 4


//FORCE PARAMATERS
#define BOUNDRY_KEYHOLE 5
#define BOUNDRY_KEYHOLE_CMD 19 // S19 DATATYPE
#define BOUNDRY_KEYHOLE_SIZE 10 


#define SPEED_FACTORA 6


#define BETA_XYZ 7  // FIXED 16

#define FRICTION_KEYHOLE 8
#define FRICTION_KEYHOLE_CMD 16 // FIXED 16 DATATYPE
#define FRICTION_KEYHOLE_SIZE 5


#define MOVE_TRHESHOLD 9
#define F_FACTOR 10
#define MAX_ERROR 11

#define FORCE_BIAS_KEYHOLE 12
#define FORCE_BIAS_KEYHOLE_CMD 19 // S19 DATATYPE
#define FORCE_BIAS_KEYHOLE_SIZE 5


// control state register
#define COMMAND_REG 13
#define CMD_CAPCAL_BASE 1
#define CMD_CAPCAL_END 2
#define CMD_CAPCAL_PIVOT 4
#define CMD_MOVEEN 8
#define CMD_MOVEGO 16
#define CMD_ENABLE_LOOP 32
#define CMD_CLEAR_LOOP 64
#define CMD_CALIBRATE_RUN 128
#define CMD_RESET_POSITION 256
#define CMD_RESET_FORCE 512
#define CMD_CAPCAL_ANGLE 1024
#define CMD_CAPCAL_ROT 2048
#define CMD_ANGLE_ENABLE 4096
#define CMD_ROT_ENABLE 8196

#define ISTRING_LEN 255 //should be less or equal to socket buffer size. 
char iString[ISTRING_LEN]; //make global so we can re-use (main, getInput, etc...)





//DMA 
#define DMA_CONTROL 14
#define DMA_WRITE_DATA 15
#define DMA_WRITE_PARAMS 16
#define DMA_WRITE_ADDRESS 17
#define DMA_READ_PARAMS 18
#define DMA_READ_ADDRESS 19

// DMA control bits breakdown

#define DMA_WRITE_ENQUEUE 1
#define DMA_WRITE_INITIATE 2
#define DMA_READ_DEQUEUE 4
#define DMA_READ_BLOCK 8
#define DMA_RESET_ALL 16


// RECORD AND PLAYPACK
#define REC_PLAY_CMD 20

#define CMD_RESET_RECORD 1
#define CMD_RECORD 2
#define CMD_RESET_PLAY 4
#define CMD_PLAYBACK 8
#define CMD_RESET_PLAY_POSITION 16

#define REC_PLAY_TIMEBASE 21


#define MAXSPEED_XYZ 22

#define DIFF_FORCE_BETA 23
#define DIFF_FORCE_MOVE_THRESHOLD 24
#define DIFF_FORCE_MAX_SPEED 25
#define DIFF_FORCE_SPEED_FACTOR_ANGLE 26
#define DIFF_FORCE_SPEED_FACTOR_ROT 27
#define EXTRUDER_CONTROL 28 

#define FINE_ADJUST_KEYHOLE 29  // PID MOVE OFFSET
#define FINE_ADJUST_KEYHOLE_CMD 19 //  S19 DATATYPE
#define FINE_ADJUST_KEYHOLE_SIZE 5


#define RECORD_LENGTH 30



#define END_EFFECTOR_IO 31
// END_EFFECTOR_IO bits
#define IO_BLUE_IN 1 //otherwise it's an output
#define IO_BLUE_HIGH 2 //otherwise low
#define IO_GREEN_IN 4
#define IO_GREEN_HIGH 8
#define IO_GREEN_PWM 16 //assuming it's an output
#define IO_GREEN_RC 32 //if output, PWM at servo frequency and range
#define IO_BLUE_PWM 64
#define IO_BLUE_RC 128
#define IO_BLUE_DYNAMIXEL 128+64 //special serial output mode for Dynamixel servos
#define IO_DYNAMIXEL (IO_BLUE_DYNAMIXEL + IO_GREEN_IN)

#define SERVO_SETPOINT_A 32
#define SERVO_SETPOINT_B 33

#define UART1_XMIT_CNT 34 // SELT,LOADQ,FORCE TRANSMIT,RESET 
#define UART1_XMIT_DATA 35 // 8BIT
#define UART1_XMIT_TIMEBASE 36

#define BASE_FORCE_DECAY 37
#define END_FORCE_DECAY 38
#define PIVOT_FORCE_DECAY 39
#define ANGLE_FORCE_DECAY 40
#define ROTATE_FORCE_DECAY 41

#define PID_SCHEDULE_INDEX 42

#define GRIPPER_MOTOR_CONTROL 43
#define GRIPPER_MOTOR_ON_WIDTH 44
#define GRIPPER_MOTOR_OFF_WIDTH 45

#define START_SPEED 46
#define ANGLE_END_RATIO 47

#define RESET_PID_AND_FLUSH_QUEUE 48 // BIT ZERO = RESET PID ACCUMULATOR, BIT 1 = FLUSH QUEUE


#define XYZ_FORCE_TIMEBASE 49
#define DIFFERENTIAL_FORCE_TIMEBASE 50
#define PID_TIMEBASE 51
 
#define RAW_ECONDER_ANGLE_KEYHOLE 52
#define RAW_ECONDER_ANGLE_KEYHOLE_CMD 10 // s19 datatype
#define RAW_ECONDER_ANGLE_KEYHOLE_SIZE 5





// DMA DATA IN

#define DMA_READ_DATA 30 + INPUT_OFFSET






// READ REGISTER DEFINITIONS

// POSITION REPORT
#define BASE_POSITION_AT 0 + INPUT_OFFSET
#define END_POSITION_AT 1 + INPUT_OFFSET
#define PIVOT_POSITION_AT 2 + INPUT_OFFSET
#define ANGLE_POSITION_AT 3 + INPUT_OFFSET
#define ROT_POSITION_AT 4 + INPUT_OFFSET

//TABLE CALCULATED DELTA

#define BASE_POSITION_DELTA 5 + INPUT_OFFSET
#define END_POSITION_DELTA 6 + INPUT_OFFSET
#define PIVOT_POSITION_DELTA 7 + INPUT_OFFSET
#define ANGLE_POSITION_DELTA 8 + INPUT_OFFSET
#define ROT_POSITION_DELTA 9 + INPUT_OFFSET


//PID CALCULATED DELTA

#define BASE_POSITION_PID_DELTA 10 + INPUT_OFFSET
#define END_POSITION_PID_DELTA 11 + INPUT_OFFSET
#define PIVOT_POSITION_PID_DELTA 12 + INPUT_OFFSET
#define ANGLE_POSITION_PID_DELTA 13 + INPUT_OFFSET
#define ROT_POSITION_PID_DELTA 14 + INPUT_OFFSET


// FORCE CALCULATED POSITION MODIFICATION

#define BASE_POSITION_FORCE_DELTA 15 + INPUT_OFFSET
#define END_POSITION_FORCE_DELTA 16 + INPUT_OFFSET
#define PIVOT_POSITION_FORCE_DELTA 17 + INPUT_OFFSET
#define ANGLE_POSITION_FORCE_DELTA 18 + INPUT_OFFSET
#define ROT_POSITION_FORCE_DELTA 19 + INPUT_OFFSET


// RAW ANALOG TO DIGITAL VALUES

#define BASE_SIN 20 + INPUT_OFFSET
#define BASE_COS 21 + INPUT_OFFSET
#define END_SIN 22 + INPUT_OFFSET
#define END_COS 23 + INPUT_OFFSET
#define PIVOT_SIN 24 + INPUT_OFFSET
#define PIVOT_COS 25 + INPUT_OFFSET
#define ANGLE_SIN 26 + INPUT_OFFSET
#define ANGLE_COS 27 + INPUT_OFFSET
#define ROT_SIN 28 + INPUT_OFFSET
#define ROT_COS 29 + INPUT_OFFSET

// RECORD AND PLAYBACK 

#define RECORD_BLOCK_SIZE 31 + INPUT_OFFSET
#define READ_BLOCK_COUNT 32 + INPUT_OFFSET
#define PLAYBACK_BASE_POSITION 33 + INPUT_OFFSET
#define PLAYBACK_END_POSITION 34 + INPUT_OFFSET
#define PLAYBACK_PIVOT_POSITION 35 + INPUT_OFFSET
#define PLAYBACK_ANGLE_POSITION 36 + INPUT_OFFSET
#define PLAYBACK_ROT_POSITION 37 + INPUT_OFFSET


#define END_EFFECTOR_IO_IN 38 + INPUT_OFFSET

#define SENT_BASE_POSITION 39 + INPUT_OFFSET
#define SENT_END_POSITION 40 + INPUT_OFFSET
#define SENT_PIVOT_POSITION 41 + INPUT_OFFSET
#define SENT_ANGLE_POSITION 42 + INPUT_OFFSET
#define SENT_ROT_POSITION 43 + INPUT_OFFSET

#define SLOPE_BASE_POSITION 44 + INPUT_OFFSET
#define SLOPE_END_POSITION 45 + INPUT_OFFSET
#define SLOPE_PIVOT_POSITION 46 + INPUT_OFFSET
#define SLOPE_ANGLE_POSITION 47 + INPUT_OFFSET
#define SLOPE_ROT_POSITION 48 + INPUT_OFFSET
#define CMD_FIFO_STATE 49 + INPUT_OFFSET
#define UART_DATA_IN 50 + INPUT_OFFSET

//Wiggles code:
#define BASE_MEASURED_ANGLE 51 + INPUT_OFFSET
#define END_MEASURED_ANGLE 52 + INPUT_OFFSET
#define PIVOT_MEASURED_ANGLE 53 + INPUT_OFFSET
#define ANGLE_MEASURED_ANGLE 54 + INPUT_OFFSET
#define ROT_MEASURED_ANGLE 55 + INPUT_OFFSET

// Encoder Angles (Integer portion only??)
#define BASE_EYE_NUMBER 56 + INPUT_OFFSET // 
#define PIVOT_EYE_NUMBER 58 + INPUT_OFFSET// (not 57)
#define END_EYE_NUMBER 57 + INPUT_OFFSET  // (not 58)
#define ANGLE_EYE_NUMBER 59 + INPUT_OFFSET// 
#define ROT_EYE_NUMBER 60 + INPUT_OFFSET  // 

#define BASE_RAW_ENCODER_ANGLE_FXP 61 + INPUT_OFFSET // was 64 //was 61
#define PIVOT_RAW_ENCODER_ANGLE_FXP 63 + INPUT_OFFSET// was 61 //was 63 (not 62)
#define END_RAW_ENCODER_ANGLE_FXP 62 + INPUT_OFFSET  // was 65 //was 62 (not 63)
#define ANGLE_RAW_ENCODER_ANGLE_FXP 64 + INPUT_OFFSET// was 62 //was 64
#define ROT_RAW_ENCODER_ANGLE_FXP 65 + INPUT_OFFSET  // was 63 //was 65

// Step Angles
#define BASE_STEPS 66 + INPUT_OFFSET
#define PIVOT_STEPS 68 + INPUT_OFFSET // was 67
#define END_STEPS 67 + INPUT_OFFSET  // was 68
#define ANGLE_STEPS 69 + INPUT_OFFSET
#define ROT_STEPS 70 + INPUT_OFFSET

#define MAPPED_SIZE 71 
//must be updated if more mapped addresses are added
//TODO: Compute from type data at start of mapped memory from FPGA

int OldMemMapInderection[]={0,0,0,0,0,ACCELERATION_MAXSPEED,0,0,0,0,0,0,0,0,0,0,0,0,0,0,PID_P,PID_ADDRESS,
	0,0,0,0,0,SPEED_FACTORA,BETA_XYZ,0,0,0,0,0,MOVE_TRHESHOLD,F_FACTOR,MAX_ERROR,0,0,0,0,0,COMMAND_REG,
	DMA_CONTROL,DMA_WRITE_DATA,DMA_WRITE_PARAMS,DMA_WRITE_ADDRESS,DMA_READ_PARAMS,DMA_READ_ADDRESS,
	REC_PLAY_CMD,REC_PLAY_TIMEBASE,MAXSPEED_XYZ,DIFF_FORCE_BETA,DIFF_FORCE_MOVE_THRESHOLD,
	DIFF_FORCE_MAX_SPEED,DIFF_FORCE_SPEED_FACTOR_ANGLE,DIFF_FORCE_SPEED_FACTOR_ROT, 
	EXTRUDER_CONTROL,0,0,0,0,0,0,END_EFFECTOR_IO,SERVO_SETPOINT_A,SERVO_SETPOINT_B,
	BASE_FORCE_DECAY,END_FORCE_DECAY,PIVOT_FORCE_DECAY,ANGLE_FORCE_DECAY,ROTATE_FORCE_DECAY,
	0,GRIPPER_MOTOR_CONTROL,GRIPPER_MOTOR_OFF_WIDTH,GRIPPER_MOTOR_ON_WIDTH,0,0,
	RESET_PID_AND_FLUSH_QUEUE,XYZ_FORCE_TIMEBASE,DIFFERENTIAL_FORCE_TIMEBASE,PID_TIMEBASE,0,0,0,0};


int ADLookUp[5] = {BASE_SIN,END_SIN,PIVOT_SIN,ANGLE_SIN,ROT_SIN};

//commands

#define MOVE_CMD 1
#define READ_CMD 2
#define WRITE_CMD 3
#define EXIT_CMD 4
#define SLOWMOVE_CMD 5
#define MOVEALL_CMD 6
#define DMAREAD_CMD 7
#define DMAWRITE_CMD 8
#define CAPTURE_AD_CMD 9
#define FIND_HOME_CMD 10
#define FIND_HOME_REP_CMD 11
#define LOAD_TABLES 12
#define CAPTURE_POINTS_CMD 14
#define FIND_INDEX_CMD 15
#define SLEEP_CMD 16
#define RECORD_MOVEMENT 17
#define REPLAY_MOVEMENT 18
#define MOVEALL_RELATIVE 19
#define SET_PARAM 20
#define SEND_HEARTBEAT 21
#define SET_FORCE_MOVE_POINT 22
#define HEART_BEAT 23
#define PID_FINEMOVE 24
#define SET_ALL_BOUNDRY 25

//////////////////////////////////////////////////////////////////////////
/* Start Wigglesworth Code*/
//////////////////////////////////////////////////////////////////////////
#define MOVETO_CMD 26
#define MOVETOSTRAIGHT_CMD 27
#define WRITE_TO_ROBOT 28

#define DEFAULT_MAXSPEED = 232642; // 30 (deg/s)
#define DEFAULT_STARTSPEED = 512; // .066 (deg/s) This is the smallest number allowed
//////////////////////////////////////////////////////////////////////////
/* End Wigglesworth Code*/
//////////////////////////////////////////////////////////////////////////

// modes
#define BLOCKING_MOVE 1
#define NON_BLOCKING_MOVE 2
#define DEFAULT_MOVE_TIMEOUT 0
#define TRUE 1
#define FALSE 0


// defaults
#define DEFAULT_PID_SETTING_XYZ 0x3dcCCCCC
//#define DEFAULT_PID_SETTING_XYZ 0x3f000000
//#define DEFAULT_PID_SETTING_XYZ 0x3f800000
//#define DEFAULT_PID_SETTING_PY 0x3f000000
//#define DEFAULT_PID_SETTING_PY 0x3D4CCCCC
//#define DEFAULT_PID_SETTING_PY 0x3dcCCCCC
#define DEFAULT_PID_SETTING_PY 0x3cf5c28f


#define DEF_SPEED_FACTOR_A 30
#define DEF_SPEED_FACTOR_DIFF 8

#define ACCELERATION_MAXSPEED_DEF 250000+(3<<20)

#define BASE_SIN_LOW 0X760
#define BASE_COS_LOW 0X1
#define BASE_SIN_HIGH 0x83A
#define BASE_COS_HIGH 0X40

#define END_SIN_LOW 0Xc01
#define END_COS_LOW 0X0
#define END_SIN_HIGH 0xd5c
#define END_COS_HIGH 0X70

#define PIVOT_SIN_LOW 0X50
#define PIVOT_COS_LOW 0X880
#define PIVOT_SIN_HIGH 0x100
#define PIVOT_COS_HIGH 0X8d1

#define ANGLE_SIN_LOW 0X900
#define ANGLE_COS_LOW 0X1
#define ANGLE_SIN_HIGH 0x9A0
#define ANGLE_COS_HIGH 0X50

#define ROT_SIN_LOW 0XA90
#define ROT_COS_LOW 0X1
#define ROT_SIN_HIGH 0xcc2
#define ROT_COS_HIGH 0X50


#define DEFAULT_ANGLE_BOUNDRY_HI 3000
#define DEFAULT_ANGLE_BOUNDRY_LOW -3000

#define SERVO_LOW_BOUND 1142000
#define SERVO_HI_BOUND 1355000

#define MONITOR_ERROR_CODE 666
#define BOUNDARY_ERROR_CODE 665
#define ERROR_INPUT_OFFSET 1024













//////////////////////////////////////////////////////////////////////////
/* Start Wigglesworth Code*/


//double micro_step_per_s_to_MaxSpeed = 8.388535999997595;
double arcsec_per_nbits = 0.4642324678861586;
double bit_sec_per_microstep_clockcycle = 8.192;
int startSpeed_arcsec_per_sec = 3600;
int maxSpeed_arcsec_per_sec = 30*3600;


//int AngularSpeed = 30*3600; 				// (arcsec/s)
//int AngularSpeedStartAndEnd = 360; 			// (arcsec/s)
int AngularAcceleration = 1;				// (arcsec/s^2)

int CartesianSpeed = 300000; 				// (micron/s)
int CartesianSpeedStart = 0; 				// (micron/s)
int CartesianSpeedEnd = 0; 					// (micron/s)
int CartesianAcceleration = 1000000; 		// (micron/s^2)
int CartesianStepSize = 10; 				// (micron)
int CartesianPivotSpeed = 108000; 			// (arcsec/s)
int CartesianPivotSpeedStart = 0; 			// (arcsec/s)
int CartesianPivotSpeedEnd = 0; 			// (arcsec/s)
int CartesianPivotAcceleration = 10800000; 	// (arcsec/s^2)
int CartesianPivotStepSize = 36; 			// (arcsec)
int LastGoal[5]={0,0,0,0,0};


//#include <cstdlib> //C++ header
//#include <math.h> // for trig and floor functions
//#include <iostream> //doesn't exist on Dexter
//#include <limits> // for intentionaly returning nan 
//#include <time.h> /* c//lock_t, clock, CLOCKS_PER_SEC */
//#include <stdlib.h> //defined above
//#include <stdbool.h>




// Vector Library:
#include <math.h>
#define PI (3.141592653589793)
//LinkLengths
//double L[5] = { 0.1651, 0.320675, 0.3302, 0.0508, 0.08255 }; // (meters)
//double L[5] = { 165100, 320675, 330200, 50800, 82550 }; // (microns)
//double L[5] = { 165100, 340320, 321032, 50800, 140000 }; // (microns) JW 20190312 for HD version with gripper
double L[5] = { 228600, 320676, 330201, 50801, 82551 }; // JW / JF 20190523 better for HD from Monty
//double SP[5] = { 0, 0, 0, 0, 0 }; // (arcseconds)

int SP_CommandedAngles[5] = { 0, 0, 0, 0, 0 }; // Starting Position Commanded (arcseconds)
int SP_EyeNumbers[5] = { 0, 0, 0, 0, 0 }; // Starting Position EyeNumber (arcseconds)

struct Vector {
	double x, y, z;
};

double max(double a, double b){
	if(a > b){
		return a;
	}else{
		return b;
	}
}

double min(double a, double b){
	if(a < b){
		return a;
	}else{
		return b;
	}
}


struct Vector new_vector(double x, double y, double z) {
	struct Vector a;
	a.x = x;
	a.y = y;
	a.z = z;
	return a;
}

void print_vector(struct Vector a) {
	printf("[%f, %f, %f]", a.x, a.y, a.z);
}

struct Vector add(struct Vector v1, struct Vector v2) {
	struct Vector result;
	result.x = v1.x + v2.x;
	result.y = v1.y + v2.y;
	result.z = v1.z + v2.z;
	return result;
};


struct Vector subtract(struct Vector v1, struct Vector v2) {
	struct Vector result;
	result.x = v1.x - v2.x;
	result.y = v1.y - v2.y;
	result.z = v1.z - v2.z;
	return result;
}


struct Vector mult(struct Vector v1, struct Vector v2) {
	struct Vector result;
	result.x = v1.x * v2.x;
	result.y = v1.y * v2.y;
	result.z = v1.z * v2.z;
	return result;
}

struct Vector vector_div(struct Vector v1, struct Vector v2) {
	struct Vector result;
	result.x = v1.x / v2.x;
	result.y = v1.y / v2.y;
	result.z = v1.z / v2.z;
	return result;
}

struct Vector scalar_mult(double a, struct Vector v) {
	struct Vector result;
	result.x = a * v.x;
	result.y = a * v.y;
	result.z = a * v.z;
	return result;
}

struct Vector scalar_div(struct Vector v, double a) {
	struct Vector result;
	result.x = v.x / a;
	result.y = v.y / a;
	result.z = v.z / a;
	return result;
}

struct Vector negate(struct Vector v) {
	struct Vector result;
	result.x = -v.x;
	result.y = -v.y;
	result.z = -v.z;
	return result;
}

double magnitude(struct Vector v) {
	return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}


struct Vector normalize(struct Vector v) {
	double mag = magnitude(v);
	return scalar_div(v, mag);
}


struct Vector cross(struct Vector v1, struct Vector v2) {
	struct Vector result = new_vector(v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x);
	return result;
}

double dot(struct Vector v1, struct Vector v2) {
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

struct Vector v_abs(struct Vector v1){
	struct Vector v_result = new_vector(fabs(v1.x), fabs(v1.y), fabs(v1.z));
	return v_result;
}


double v_max(struct Vector v1){
	double cur_max = 0;
	if(v1.x > cur_max)
		cur_max = v1.x;
	if(v1.y > cur_max)
		cur_max = v1.y;
	if(v1.z > cur_max)
		cur_max = v1.z;
	return cur_max;
}

struct Vector v_round(struct Vector v1, int decimals) {
	return new_vector(floor(v1.x * pow(10, decimals)) / pow(10, decimals), floor(v1.y * pow(10, decimals)) / pow(10, decimals), floor(v1.z * pow(10, decimals)) / pow(10, decimals));
}

bool is_equal(struct Vector v1, struct Vector v2, int decimals) {
	bool result =  pow(10, -decimals-1) > v_max(v_abs(subtract(v2, v1)));
	
	
	/*
	struct Vector r_v1 = v_round(v1, decimals);
	struct Vector r_v2 = v_round(v2, decimals);
	*/
	/*
	printf("\n\n is_equal:");
	printf("\nv1: ");
	print_vector(v1);
	printf("\nv2: ");
	print_vector(v2);
	printf("\ndecimals: %d", decimals);
	*/
	
	/*
	printf("\nr_v1: ");
	print_vector(r_v1);
	printf("\nr_v1: ");
	print_vector(r_v1);
	*/
	
	//bool result = (r_v1.x == r_v2.x && r_v1.y == r_v2.y && r_v1.z == r_v2.z);
	//printf("\nresult: %d", result);
	return result;
	
}

struct Vector project_onto_plane(struct Vector vector, struct Vector plane) {
	//double term1 = dot(vector, plane);
	double plane_mag = magnitude(plane);
	return subtract(vector, scalar_mult(dot(vector, plane) / (plane_mag*plane_mag), plane));
}



#define arcsec_to_rad PI/(180*3600); // Floating point error may be a problem
#define rad_to_arcsec (180*3600)/PI;
double sin_arcsec(double theta) {
	//return sin(theta * arcsec_to_rad); //this was erroring for some reason
	return sin(theta * PI / (180 * 3600));
}

double cos_arcsec(double theta) {
	return cos(theta * PI / (180 * 3600));
}

double tan_arcsec(double theta) {
	return tan(theta * PI / (180 * 3600));
}

double asin_arcsec(double ratio) {
	return asin(ratio) * rad_to_arcsec;
}

double acos_arcsec(double ratio) {
	return acos(ratio) * rad_to_arcsec;
}

double atan_arcsec(double ratio) {
	return atan(ratio) * rad_to_arcsec;
}

double atan2_arcsec(double num1, double num2) {
	return atan2(num1, num2) * rad_to_arcsec;
}






struct Vector rotate(struct Vector vector, struct Vector plane, double theta) {
	if (is_equal(vector, plane, 14)) {
		return vector;
	}
	return scalar_mult(magnitude(vector), normalize(add(scalar_mult(cos_arcsec(theta), vector), scalar_mult(sin_arcsec(theta), cross(normalize(plane), vector)))));
}

struct Vector three_points_to_plane(struct Vector u1, struct Vector u2, struct Vector u3) {
	return normalize(cross(subtract(u2, u1), subtract(u3, u1)));
}

double angle(struct Vector v1, struct Vector v2) {
	if (is_equal(v1, v2, 14)) {
		return 0.0;
	}
	if (magnitude(add(v1, v2)) == 0) {
		return 180*3600;
	}
	else {
		return atan2_arcsec(magnitude(cross(v1, v2)), dot(v1, v2));
	}
}

double signed_angle(struct Vector v1, struct Vector v2, struct Vector plane) {
	double guess_angle = angle(v1, v2);
	long double epsilon = 0.0000000001; // 1e-10
	if (abs(guess_angle) <  epsilon || abs(abs(guess_angle) - 648000) < epsilon) {
		return round(guess_angle); // Will rounding increase or decrease error? 
	}

	struct Vector c_prod = normalize(cross(v1, v2));
	if (is_equal(c_prod, normalize(plane), 10)) {
		return guess_angle;
	}
	else if (is_equal(negate(c_prod), normalize(plane), 10)){
		return -guess_angle;
	}
	else {
		printf("\nError: signed_angle wants to return NaN:");
		printf("\nguessangle: %f", guess_angle);
		printf("\nc_prod: ");
		print_vector(c_prod);
		printf("\nnegate(c_prod): ");
		print_vector(negate(c_prod));
		printf("\nv1: ");
		print_vector(v1);
		printf("\nv2: ");
		print_vector(v2);
		printf("\nplane: ");
		print_vector(plane);
		printf("\n\n");
	}
	return 0;
}

double dist_point_to_point(struct Vector point_a, struct Vector point_b) {
	/*
	printf("point_a: ");
	print_vector(point_a);
	printf(", point_b: ");
	print_vector(point_b);
	printf("\n");
	*/
	return magnitude(subtract(point_a, point_b));
}

double dist_point_to_line(struct Vector point, struct Vector line_point_a, struct Vector line_point_b) {
	struct Vector term1 = subtract(point, line_point_a);
	struct Vector term2 = subtract(point, line_point_b);
	struct Vector term3 = subtract(line_point_b, line_point_a);
	return magnitude(cross(term1, term2)) / magnitude(term3);
}

bool is_NaN(struct Vector v) {
	return isnan(v.x) || isnan(v.y) || isnan(v.z);
}


//////////////////////////////////////////////////////////////////
// Kinematics:



//Config structure
struct Config{
	bool right_arm, elbow_up, wrist_out;
};

struct Config new_config(bool right_arm, bool elbow_up, bool wrist_out) {
	struct Config a;
	a.right_arm = right_arm;
	a.elbow_up = elbow_up;
	a.wrist_out = wrist_out;
	return a;
}

void print_config(struct Config a) {
	printf("[");
	if (a.right_arm){
		printf("Right, ");
	}else{
		printf("Left, ");
	}
	if (a.elbow_up){
		printf("Up, ");
	}else {
		printf("Down, ");
	}
	if (a.wrist_out) {
		printf("Out]");
	}
	else {
		printf("In]");
	}
	//return
}


//J_angles structure
struct J_angles{
	double J1, J2, J3, J4, J5;
};

struct J_angles new_J_angles(double J1, double J2, double J3, double J4, double J5) {
	struct J_angles a;
	a.J1 = J1;
	a.J2 = J2;
	a.J3 = J3;
	a.J4 = J4;
	a.J5 = J5;
	return a;
}

void print_J_angles(struct J_angles a) {
	printf("[%f, %f, %f, %f, %f]", a.J1, a.J2, a.J3, a.J4, a.J5);
}

double J_angles_max_diff(struct J_angles J_angles_1, struct J_angles J_angles_2){
	double cur_max = 0.0;
	double delta;
	delta = abs(J_angles_1.J1 - J_angles_2.J1);
	if(delta > cur_max)
		cur_max = delta;
	delta = abs(J_angles_1.J2 - J_angles_2.J2);
	if(delta > cur_max)
		cur_max = delta;
	delta = abs(J_angles_1.J3 - J_angles_2.J3);
	if(delta > cur_max)
		cur_max = delta;
	delta = abs(J_angles_1.J4 - J_angles_2.J4);
	if(delta > cur_max)
		cur_max = delta;
	delta = abs(J_angles_1.J5 - J_angles_2.J5);
	if(delta > cur_max)
		cur_max = delta;

	return cur_max;
}

//XYZ structure
struct XYZ{
	struct Vector position;
	struct Vector direction;
	struct Config config;
};


struct XYZ new_XYZ(struct Vector position, struct Vector direction, struct Config config) {
	struct XYZ a;
	a.position = position;
	a.direction = direction;
	a.config = config;
	return a;
}

void print_XYZ(struct XYZ a) {
	printf("{");
	print_vector(a.position);
	printf(", ");
	print_vector(a.direction);
	printf(", ");
	print_config(a.config);
	printf("}");
}


// Forward Kinematics:
struct XYZ J_angles_to_XYZ(struct J_angles angles) {
	
	//Pre-allocation:
	struct Vector U0 = {0, 0, 0};
	struct Vector U1 = {0, 0, 0};
	struct Vector U2 = {0, 0, 0};
	struct Vector U3 = {0, 0, 0};
	struct Vector U4 = {0, 0, 0};
	struct Vector U5 = {0, 0, 0};

	struct Vector V0 = {0, 0, 1};
	struct Vector V1 = {0, 0, 0};
	struct Vector V2 = {0, 0, 0};
	struct Vector V3 = {0, 0, 0};
	struct Vector V4 = {0, 0, 0};

	struct Vector P0 = {1, 0, 0};
	struct Vector P1 = {0, 0, 0};
	struct Vector P2 = {0, 0, 0};
	
	//FK:
	P1 = rotate(P0, V0, -(angles.J1 - 180*3600)); 	// Links 2, 3 and 4 lie in P1
	V1 = rotate(V0, P1, angles.J2);		   			// Vector for Link 2
	V2 = rotate(V1, P1, angles.J3);		   			// Vector for Link 3
	V3 = rotate(V2, P1, angles.J4);		  			// Vector for Link 4
	P2 = rotate(P1, V3, -(angles.J5 - 180*3600));	// Link 4 and 5 lie in P2
	V4 = rotate(V3, P2, -90*3600);				   	// Vector for Link 5 (90 degree bend)
	
	U1 = add(U0, scalar_mult(L[0], V0));
	U2 = add(U1, scalar_mult(L[1], V1));
	U3 = add(U2, scalar_mult(L[2], V2));
	U4 = add(U3, scalar_mult(L[3], V3));
	U5 = add(U4, scalar_mult(L[4], V4));

	// Forward Kinematic solved, now to determine configuration:

	// Inverse Kinematics for just U3 to determine if wrist is in or out
	struct Vector U54_proj = project_onto_plane(V4, P1);
	struct Vector U3_a = add(U4, scalar_mult(L[3], rotate(normalize(U54_proj), P1, 90)));
	struct Vector U3_b = add(U4, scalar_mult(L[3], rotate(normalize(U54_proj), P1, -90)));
	double dist_a = dist_point_to_line(U3_a, U1, U0);
	double dist_b = dist_point_to_line(U3_b, U1, U0);

	struct Config my_config = new_config(1, 1, 1); // Initialize my_config

							   // Determine wrist state, in or out
	if (is_equal(U3, U3_a, 10)) {  // Negative or positive rotation of Link 4?
		if (dist_a > dist_b) {
			my_config.wrist_out = 1; // Wrist out
		}
		else {
			my_config.wrist_out = 0; // Wrist in
		}
	}
	else {
		if (dist_a > dist_b) {
			my_config.wrist_out = 0; // Wrist in
		}
		else {
			my_config.wrist_out = 1; // Wrist out
		}
	}



	// Determine arm state, right or left 
	struct Vector U50 = subtract(U5, U0);
	if (dot(cross(U50, P1), V0) < 0) {			// Is end effector point on right or left half of plane 1?
		my_config.right_arm = 0;					// Left arm
		my_config.wrist_out = !my_config.wrist_out; // switches wrist state if left arm
	}
	else {
		my_config.right_arm = 1;					// Right arm
	}
 	// Determine elbow state, up or down
	if (my_config.right_arm) {						// If right arm
		if (dot(cross(V1, V2), P1) > 0) {		// Is Joint 3 angle positive or negative?
			my_config.elbow_up = 1;					// Elbow down
		}
		else {
			my_config.elbow_up = 0;					// Elbow up
		}
	}
	else {											// If left arm
		if (dot(cross(V1, V2), P1) > 0) {		// Is Joint 3 angle positive or negative?
			my_config.elbow_up = 0;					// Elbow down
		}
		else {
			my_config.elbow_up = 1;					// Elbow down
		}
	}




	// Calculating the end effector direction
	struct Vector my_dir = normalize(subtract(U5, U4));
	struct XYZ solution = new_XYZ(U5, my_dir, my_config);
	return solution;
}

// Inverse Kinematics:
struct J_angles xyz_to_J_angles(struct XYZ xyz) {

	// Pre-allocation
	struct J_angles J = new_J_angles(0, 0, 0, 0, 0);
	/*
	struct Vector U[6] = { { 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 } };
	struct Vector V[5] = { { 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 } };
	struct Vector P[3] = { { 0, 0, 0 },{ 0, 0, 0 },{ 0, 0, 0 } };
	
	
	// Knowns:
	//U[0] = { 0, 0, 0 };
	P[0][0] = 1;
	V[0][2] = 1;
	V[4] = normalize(xyz.direction);
	U[1] = scalar_mult(L[0], V[0]);
	U[4] = add(xyz.EE_point, scalar_mult(L[4], negate(V[4])));
	U[5] = xyz.EE_point;
	*/
	
	struct Vector V0 = {0, 0, 1};
	struct Vector V1 = {0, 0, 0};
	struct Vector V2 = {0, 0, 0};
	struct Vector V3 = {0, 0, 0};
	struct Vector V4 = normalize(xyz.direction);
	
	struct Vector U0 = {0, 0, 0};
	struct Vector U1 = scalar_mult(L[0], V0);
	struct Vector U2 = {0, 0, 0};
	struct Vector U3 = {0, 0, 0};
	struct Vector U4 = add(xyz.position, scalar_mult(L[4], negate(V4)));
	struct Vector U5 = xyz.position;
	
	
	
	struct Vector P0 = {1, 0, 0};
	struct Vector P1 = {0, 0, 0};
	struct Vector P2 = {0, 0, 0};



	// Solving for Plane 1:
	P1 = three_points_to_plane(U1, U0, U4);
	/*
	!!!!!!!!!!!!!! Come back to this and do it without NaNs !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if (is_NaN(P[1])) {
		P[1] = three_points_to_plane(U[1], U[0], U[3]);
		if (is_NaN(P[1])) {
			// Error message: "Singularity: Toolpoint XYZ is on Base axis."
			//std::cout << "Error: Singularity: Toolpoint XYZ is on Base axis.";
		}
	}
	*/
	

	// Solving for U3:
	struct Vector U54_proj = project_onto_plane(negate(V4), P1);
	struct Vector U3_a = add(U4, scalar_mult(L[3], rotate(normalize(U54_proj), P1, 90*3600)));
	struct Vector U3_b = add(U4, scalar_mult(L[3], rotate(normalize(U54_proj), P1, -90*3600)));

	
	//New wrist code
	//double dist_a = dist_point_to_point(U3_a, { 0, 0, 0 });
	//double dist_b = dist_point_to_point(U3_b, { 0, 0, 0 });
	double dist_a = magnitude(U3_a);
	double dist_b = magnitude(U3_b);
	if (xyz.config.wrist_out) {
		if (dist_a < dist_b) {
			U3 = U3_a;
		}
		else {
			U3 = U3_b;
		}
	}
	else {
		if (dist_a > dist_b) {
			U3 = U3_a;
		}
		else {
			U3 = U3_b;
		}
	}


	// Solving for Plane 2:
	P2 = three_points_to_plane(U5, U4, U3);
	if (is_NaN(P1)) {
		// Shouldnt message: "Unkown Singularity found at " + xyz.print() + ", Please report this message as a bug."
		//std::cout << "Shouldnt: Unkown Singularity";
		printf("\n\nUnkown Singularity found at: ");
		print_XYZ(xyz);
		printf("\nPlease report this message as a bug.\n\n");
	}

	// Checking if in range
	double D3 = magnitude(subtract(U3, U1));
	if (D3 > L[1] + L[2]) {
		// Error message: "Point out of reach"
		//std::cout << "Point out of reach";
		printf("\nOut of Reach Error at position: ");
		print_XYZ(xyz);
		printf("\nLastGoal: [%d, %d, %d, %d, %d]\n", LastGoal[0], LastGoal[1], LastGoal[2], LastGoal[3], LastGoal[4]);

		J.J1 = LastGoal[0];
		J.J2 = LastGoal[1];
		J.J3 = LastGoal[2];
		J.J4 = LastGoal[3];
		J.J5 = LastGoal[4];
		return J;
	}


	//Solving for U2:
	double Beta = acos_arcsec((-(L[2] * L[2]) + L[1] * L[1] + D3*D3) / (2 * D3 * L[1])); // Law of Cosines
	struct Vector V31 = normalize(subtract(U3, U1));
	struct Vector U2_a = add(U1, scalar_mult(L[1], rotate(V31, P1, Beta)));
	struct Vector U2_b = add(U1, scalar_mult(L[1], rotate(V31, P1, -Beta)));
	struct Vector V2a1 = subtract(U2_a, U1);
	struct Vector V32a = subtract(U3, U2_a);
	if (xyz.config.elbow_up) {
		if (dot(cross(V2a1, V32a), P1) < 0) {
			U2 = U2_a;
		}
		else {
			U2 = U2_b;
		}
	}
	else {
		if (dot(cross(V2a1, V32a), P1) > 0) {
			U2 = U2_a;
		}
		else {
			U2 = U2_b;
		}
	}

	
	

	// Now that all joint points are solved, joint angles can be calculated:
	V1 = normalize(subtract(U2, U1));
	V2 = normalize(subtract(U3, U2));
	V3 = normalize(subtract(U4, U3));

	if (xyz.config.right_arm) {
		J.J1 = signed_angle(P1, P0, V0);
		J.J2 = signed_angle(V1, V0, P1);
		J.J3 = signed_angle(V2, V1, P1);
		J.J4 = signed_angle(V3, V2, P1);
		J.J5 = signed_angle(P2, P1, V3);
	}
	else {
		J.J1 = signed_angle(P1, P0, V0) + 180*3600;
		J.J2 = -signed_angle(V1, V0, P1);
		J.J3 = -signed_angle(V2, V1, P1);
		J.J4 = -signed_angle(V3, V2, P1);
		J.J5 = -signed_angle(P2, P1, V3);
	}

	
	//Debugging code
	/*
	printf("\n\nxyz_to_J_angles started:");
	printf("\nInput: ");
	print_XYZ(xyz);
	*/
	
	/*
	printf("\nU2_a: ");
	print_vector(U2_a);
	printf("\n");
	printf("\nU2_b: ");
	print_vector(U2_b);
	printf("\n");

	printf("\nU2: ");
	print_vector(U2);
	printf("\n");
	*/
	
	/*
	printf("\nU1: ");
	print_vector(U1);
	printf("\nU2: ");
	print_vector(U2);
	printf("\nU3: ");
	print_vector(U3);
	printf("\nU4: ");
	print_vector(U4);
	printf("\nU5: ");
	print_vector(U5);
	printf("J_angles results: ");
	print_J_angles(J);
	printf("\n");
	*/
	

	return J;
}

int k_tip_speed_to_angle_speed(struct J_angles J_angles_old, struct J_angles J_angles_new, double cart_speed){
	struct Vector EE_point_1 = J_angles_to_XYZ(J_angles_old).position;
	struct Vector EE_point_2 = J_angles_to_XYZ(J_angles_new).position;
	double dist_EE = dist_point_to_point(EE_point_1, EE_point_2);
	if(dist_EE == 0){
		//printf("SKIP\n");
		return 3877.0; //30 (deg/s)
	}
	double max_theta = J_angles_max_diff(J_angles_old, J_angles_new);
	
	int result = (int)(max_theta*cart_speed/dist_EE);
	//printf("cartspeed: %f, maxtheta: %f, distEE: %f, result: %d\n", cart_speed, max_theta, dist_EE, result);

	return result;
}
		



/*
void print_pos_ori_mat(struct pos_ori_mat a) {
	printf("\n[\n    [%f, %f, %f, %f],\n    [%f, %f, %f, %f],\n    [%f, %f, %f, %f],\n    [%f, %f, %f, %f]\n]\n", a[0][0], a[0][1], a[0][2], a[0][3], a[1][0], a[1][1], a[1][2], a[1][3], a[2][0], a[2][1], a[2][2], a[2][3], a[3][0], a[3][1], a[3][2], a[3][3]);
}
*/

/*
char* pos_ori_mat_to_string(struct pos_ori_mat a) {
	return prints("\n[\n    [%f, %f, %f, %f],\n    [%f, %f, %f, %f],\n    [%f, %f, %f, %f],\n    [%f, %f, %f, %f]\n]\n", a[0][0], a[0][1], a[0][2], a[0][3], a[1][0], a[1][1], a[1][2], a[1][3], a[2][0], a[2][1], a[2][2], a[2][3], a[3][0], a[3][1], a[3][2], a[3][3]);
}
*/

//New Code:
struct row_pos_ori_mat{
	double c0, c1, c2, c3;
};

struct pos_ori_mat{
	struct row_pos_ori_mat r0, r1, r2, r3;
};

void pos_ori_mat_to_string(struct pos_ori_mat A, char *result){
	printf("pos_ori_mat_to_string started\n");
	printf("\n[\n    [%f, %f, %f, %f],\n    [%f, %f, %f, %f],\n    [%f, %f, %f, %f],\n    [%f, %f, %f, %f]\n]\n", A.r0.c0, A.r0.c1, A.r0.c2, A.r0.c3, A.r1.c0, A.r1.c1, A.r1.c2, A.r1.c3, A.r2.c0, A.r2.c1, A.r2.c2, A.r2.c3, A.r3.c0, A.r3.c1, A.r3.c2, A.r3.c3);
	
	sprintf(result, "[[%f, %f, %f, %f],[%f, %f, %f, %f],[%f, %f, %f, %f],[%f, %f, %f, %f]]", A.r0.c0, A.r0.c1, A.r0.c2, A.r0.c3, A.r1.c0, A.r1.c1, A.r1.c2, A.r1.c3, A.r2.c0, A.r2.c1, A.r2.c2, A.r2.c3, A.r3.c0, A.r3.c1, A.r3.c2, A.r3.c3);
	printf("done with pos_ori_mat_to_string\n");
};

struct pos_ori_mat J_angles_to_pos_ori_mat(struct J_angles angles) {
	
	
	
	//Code from Forward Kinematics:
	
	
	//struct Vector U[6] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	//struct Vector V[5] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },{ 0, 0, 0 }};
	//struct Vector P[3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }};

	//P[0][0] = 1; // Datam Plane = [1 0 0]
	//V[0][2] = 1; // Vector for Link 1 (base)

	// P[1] = rotate(P[0], V[0], -(angles.J1 - 180*3600)); // Links 2, 3 and 4 lie in P1
	// V[1] = rotate(V[0], P[1], angles.J2);		   // Vector for Link 2
	// V[2] = rotate(V[1], P[1], angles.J3);		   // Vector for Link 3
	// V[3] = rotate(V[2], P[1], angles.J4);		   // Vector for Link 4
	// P[2] = rotate(P[1], V[3], -(angles.J5 - 180*3600));	   // Link 4 and 5 lie in P2
	// V[4] = rotate(V[3], P[2], -90*3600);				   // Vector for Link 5 (90 degree bend)


												   // Next point = current point + Link length * vector direction
	// int i;
	// for (i = 0; i < 5; i++) {
		// U[i + 1] = add(U[i], scalar_mult(L[i], V[i]));
	// }
	
	//printf("\nStarting J_angles_to_pos_ori_mat()\n");
	
	//Pre-allocation:
	struct Vector U0 = {0, 0, 0};
	struct Vector U1 = {0, 0, 0};
	struct Vector U2 = {0, 0, 0};
	struct Vector U3 = {0, 0, 0};
	struct Vector U4 = {0, 0, 0};
	struct Vector U5 = {0, 0, 0};

	struct Vector V0 = {0, 0, 1};
	struct Vector V1 = {0, 0, 0};
	struct Vector V2 = {0, 0, 0};
	struct Vector V3 = {0, 0, 0};
	struct Vector V4 = {0, 0, 0};

	struct Vector P0 = {1, 0, 0};
	struct Vector P1 = {0, 0, 0};
	struct Vector P2 = {0, 0, 0};
	
	//printf("Pre-allocation complete\n");
	
	//FK:
	P1 = rotate(P0, V0, -(angles.J1 - 180*3600)); 	// Links 2, 3 and 4 lie in P1
	V1 = rotate(V0, P1, angles.J2);		   			// Vector for Link 2
	V2 = rotate(V1, P1, angles.J3);		   			// Vector for Link 3
	V3 = rotate(V2, P1, angles.J4);		  			// Vector for Link 4
	P2 = rotate(P1, V3, -(angles.J5 - 180*3600));	// Link 4 and 5 lie in P2
	V4 = rotate(V3, P2, -90*3600);				   	// Vector for Link 5 (90 degree bend)
	
	//printf("Vector rotations complete\n");
	
	U1 = add(U0, scalar_mult(L[0], V0));
	U2 = add(U1, scalar_mult(L[1], V1));
	U3 = add(U2, scalar_mult(L[2], V2));
	U4 = add(U3, scalar_mult(L[3], V3));
	U5 = add(U4, scalar_mult(L[4], V4));
	
	//printf("Vector adds complete\n");
	
	//Calc pos_ori_mat:
	struct Vector Vz = V3;
	struct Vector Vy = V4;
	struct Vector Vx = cross(Vy, Vz);
	//printf("\nVector cross complete\n");
	struct pos_ori_mat result;
	//printf("\npos_ori_mat Struct def complete\n");
	
	
	result.r0.c0 = Vx.x;
	result.r1.c0 = Vx.y;
	result.r2.c0 = Vx.z;
	
	//printf("\nResult 0 complete\n");
	
	result.r0.c1 = Vy.x;
	result.r1.c1 = Vy.y;
	result.r2.c1 = Vy.z;
	
	//printf("\nResult 1 complete\n");
	
	result.r0.c2 = Vz.x;
	result.r1.c2 = Vz.y;
	result.r2.c2 = Vz.z;
	
	//printf("\nResult 2 complete\n");
	
	result.r0.c3 = U4.x;
	result.r1.c3 = U4.y;
	result.r2.c3 = U4.z;
	
	//printf("\nResult 3 complete\n");
	
	result.r3.c0 = 0;
	result.r3.c1 = 0;
	result.r3.c2 = 0;
	result.r3.c3 = 1;
	
	//printf("\nResult 4 complete\n");
	
	//= {{Vx.x, Vy.x, Vz.x, U4.x}, {Vx.y, Vy.y, Vz.y, U4.y}, {Vx.z, Vy.z, Vz.z, U4.z}, {0, 0, 0, 1}};
	
	
	
	
	// result[0][3] = U4[0];
	// result[1][3] = U4[1];
	// result[2][3] = U4[2];
	
	// result[0][0] = Vx[0];
	// result[1][0] = Vx[1];
	// result[2][0] = Vx[2];
	// result[0][1] = Vy[0];
	// result[1][1] = Vy[1];
	// result[2][1] = Vy[2];
	// result[0][2] = Vz[0];
	// result[1][2] = Vz[1];
	// result[2][2] = Vz[2];
	// result[3][0] = 0;
	// result[3][1] = 0;
	// result[3][2] = 0;
	// result[3][3] = 1;
	
	
	
	//result.orientation = {{Vx[0], Vy[0], Vz[0]}, {Vx[1], Vy[1], Vz[1]}, {Vx[2], Vy[2], Vz[2]}}
	
	return result;
}






//Safe verision of strncpy found by James Newton
size_t strlcpy(char *dst, const char *src, size_t dstsize)
{
  size_t bl = 0;
  size_t len = strlen(src);
  if(dstsize) {
    bl = (len < dstsize-1 ? len : dstsize-1);
    ((char*)memcpy(dst, src, bl))[bl] = 0;
  }
  return bl;
}


struct ellipse{
	double a, b, c, d, e, f;
	double center_x, center_y;
	double eccentricity;
	double major_radius, minor_radius;
	double quad_points_major_Ax, quad_points_major_Ay;
	double quad_points_major_Bx, quad_points_major_By;
	double quad_points_minor_Ax, quad_points_minor_Ay;
	double quad_points_minor_Bx, quad_points_minor_By;
	double rotation_angle;
	double error_state;
};

struct eye_data{
	double xs[128];
	double ys[128];
	double ts[128];
};


double matrix_determinant(int rows, int cols, double matrix[rows][cols]){
	double result = 0;
	if(2 == rows && 2 == cols){
		result = matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0];
	}else if(3 == rows && 3 == cols){
		//Source: https://en.wikipedia.org/wiki/Determinant#n_.C3.97_n_matrices
		double a, b, c, d, e, f, g, h, i;
		a = matrix[0][0];
		b = matrix[0][1];
		c = matrix[0][2];
		d = matrix[1][0];
		e = matrix[1][1];
		f = matrix[1][2];
		g = matrix[2][0];
		h = matrix[2][1];
		i = matrix[2][2];
		result = a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g);
	}else if(rows == 4 && cols == 4){
		// Source: http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche23.html
		double a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34, a41, a42, a43, a44;
		a11 = matrix[0][0];
		a12 = matrix[0][1];
		a13 = matrix[0][2];
		a14 = matrix[0][3];
		a21 = matrix[1][0];
		a22 = matrix[1][1];
		a23 = matrix[1][2];
		a24 = matrix[1][3];
		a31 = matrix[2][0];
		a32 = matrix[2][1];
		a33 = matrix[2][2];
		a34 = matrix[2][3];
		a41 = matrix[3][0];
		a42 = matrix[3][1];
		a43 = matrix[3][2];
		a44 = matrix[3][3];
		
		result = a11*a22*a33*a44 + a11*a23*a34*a42 + a11*a24*a32*a43
				+a12*a21*a34*a43 + a12*a23*a31*a44 + a12*a24*a33*a41
				+a13*a21*a32*a44 + a13*a22*a34*a41 + a13*a24*a31*a42
				+a14*a21*a33*a42 + a14*a22*a31*a43 + a14*a23*a32*a41
				-a11*a22*a34*a43 - a11*a23*a32*a44 - a11*a24*a33*a42
				-a12*a21*a33*a44 - a12*a23*a34*a41 - a12*a24*a31*a43
				-a13*a21*a34*a42 - a13*a22*a31*a44 - a13*a24*a32*a41
				-a14*a21*a32*a43 - a14*a22*a33*a41 - a14*a23*a31*a42;
	}else{
		printf("Error: matrix_determinant() was passed invalid rows and cols, [%d][%d]", rows, cols);
	}
	return result;
}


void matrix_inverse(int rows, int cols, double mat_in[rows][cols], double mat_out[rows][cols]){
	if(2 == rows && 2 == cols){
		//mat_out = {{mat_in[1][1], -mat_in[1][0]}, {-mat_in[0][1], mat_in[0][0]}};
		mat_out[0][0] = mat_in[1][1];
		mat_out[0][1] = -mat_in[1][0];
		mat_out[1][0] = -mat_in[0][1];
		mat_out[1][1] = mat_in[0][0];
		
	}else if(3 == rows && 3 == cols){
		double a, b, c, d, e, f, g, h, i, A, B, C, D, E, F, G, H, I;
		a = mat_in[0][0];
		b = mat_in[0][1];
		c = mat_in[0][2];
		d = mat_in[1][0];
		e = mat_in[1][1];
		f = mat_in[1][2];
		g = mat_in[2][0];
		h = mat_in[2][1];
		i = mat_in[2][2];
		
		double inv_det = 1.0/matrix_determinant(rows,cols,mat_in);
		double temp[2][2];
		
		temp[0][0] = e;
		temp[0][1] = f;
		temp[1][0] = h;
		temp[1][1] = i;
		A = inv_det * matrix_determinant(2,2,temp);
		
		temp[0][0] = d;
		temp[0][1] = f;
		temp[1][0] = g;
		temp[1][1] = i;
		B = inv_det * matrix_determinant(2,2,temp);
		
		temp[0][0] = d;
		temp[0][1] = e;
		temp[1][0] = g;
		temp[1][1] = h;
		C = inv_det * matrix_determinant(2,2,temp);
		
		temp[0][0] = b;
		temp[0][1] = c;
		temp[1][0] = h;
		temp[1][1] = i;
		D = inv_det * matrix_determinant(2,2,temp);
		
		temp[0][0] = a;
		temp[0][1] = c;
		temp[1][0] = g;
		temp[1][1] = i;
		E = inv_det * matrix_determinant(2,2,temp);
		
		temp[0][0] = a;
		temp[0][1] = b;
		temp[1][0] = g;
		temp[1][1] = h;
		F = inv_det * matrix_determinant(2,2,temp);
		
		temp[0][0] = b;
		temp[0][1] = c;
		temp[1][0] = e;
		temp[1][1] = f;
		G = inv_det * matrix_determinant(2,2,temp);
		
		temp[0][0] = a;
		temp[0][1] = c;
		temp[1][0] = d;
		temp[1][1] = f;
		H = inv_det * matrix_determinant(2,2,temp);
		
		temp[0][0] = a;
		temp[0][1] = b;
		temp[1][0] = d;
		temp[1][1] = e;
		I = inv_det * matrix_determinant(2,2,temp);
		
		
		// A =  inv_det * matrix_determinant(rows,cols,(double [rows]){(double[]){e, f}, (double[]){h, i}});
		// B = -inv_det * matrix_determinant(rows,cols,(double [rows]){(double[cols]){d, f}, (double[]){g, i}});
		// C =  inv_det * matrix_determinant(rows,cols,(double *[]){(double[]){d, e}, (double[]){g, h}});
		// D = -inv_det * matrix_determinant(rows,cols,(double *[]){(double[]){b, c}, (double[]){h, i}});
		// E =  inv_det * matrix_determinant(rows,cols,(double *[]){(double[]){a, c}, (double[]){g, i}});
		// F = -inv_det * matrix_determinant(rows,cols,(double *[]){(double[]){a, b}, (double[]){g, h}});
		// G =  inv_det * matrix_determinant(rows,cols,(double *[]){(double[]){b, c}, (double[]){e, f}});
		// H = -inv_det * matrix_determinant(rows,cols,(double *[]){(double[]){a, c}, (double[]){d, f}});
		// I =  inv_det * matrix_determinant(rows,cols,(double *[]){(double[]){a, b}, (double[]){d, e}});
		
		//mat_out = {{A, B, C}, {D, E, F}, {G, H, I}};
		mat_out[0][0] = A;
		mat_out[0][1] = B;
		mat_out[0][2] = C;
		mat_out[1][0] = D;
		mat_out[1][1] = E;
		mat_out[1][2] = F;
		mat_out[2][0] = G;
		mat_out[2][1] = H;
		mat_out[2][2] = I;
		
		
	}else if(4 == rows && 4 == cols){
		// Source: http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche23.html
		double a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34, a41, a42, a43, a44;
		//double b11, b12, b13, b14, b21, b22, b23, b24, b31, b32, b33, b34, b41, b42, b43, b44;
		a11 = mat_in[0][0];
		a12 = mat_in[0][1];
		a13 = mat_in[0][2];
		a14 = mat_in[0][3];
		a21 = mat_in[1][0];
		a22 = mat_in[1][1];
		a23 = mat_in[1][2];
		a24 = mat_in[1][3];
		a31 = mat_in[2][0];
		a32 = mat_in[2][1];
		a33 = mat_in[2][2];
		a34 = mat_in[2][3];
		a41 = mat_in[3][0];
		a42 = mat_in[3][1];
		a43 = mat_in[3][2];
		a44 = mat_in[3][3];
		
		double inv_det = 1.0/matrix_determinant(rows,cols,mat_in);
		mat_out[0][0] = inv_det * (a22*a33*a44 + a23*a34*a42 + a24*a32*a43 - a22*a34*a43 - a23*a32*a44 - a24*a33*a42);
		mat_out[0][1] = inv_det * (a12*a34*a43 + a13*a32*a44 + a14*a33*a42 - a12*a33*a44 - a13*a34*a42 - a14*a32*a43);
		mat_out[0][2] = inv_det * (a12*a23*a44 + a13*a24*a42 + a14*a22*a43 - a12*a24*a43 - a13*a22*a44 - a14*a23*a42);
		mat_out[0][3] = inv_det * (a12*a24*a33 + a13*a22*a34 + a14*a23*a32 - a12*a23*a34 - a13*a24*a32 - a14*a22*a33);
		
		mat_out[1][0] = inv_det * (a21*a34*a43 + a23*a31*a44 + a24*a33*a41 - a21*a33*a44 - a23*a34*a41 - a24*a31*a43);
		mat_out[1][1] = inv_det * (a11*a33*a44 + a13*a34*a41 + a14*a31*a43 - a11*a34*a43 - a13*a31*a44 - a14*a33*a41);
		mat_out[1][2] = inv_det * (a11*a24*a43 + a13*a21*a44 + a14*a23*a41 - a11*a23*a44 - a13*a24*a41 - a14*a21*a43);
		mat_out[1][3] = inv_det * (a11*a23*a34 + a13*a24*a31 + a14*a21*a33 - a11*a24*a33 - a13*a21*a34 - a14*a23*a31);
		
		mat_out[2][0] = inv_det * (a21*a32*a44 + a22*a34*a41 + a24*a31*a42 - a21*a34*a42 - a22*a31*a44 - a24*a32*a41);
		mat_out[2][1] = inv_det * (a11*a34*a42 + a12*a31*a44 + a14*a32*a41 - a11*a32*a44 - a12*a34*a41 - a14*a31*a42);
		mat_out[2][2] = inv_det * (a11*a22*a44 + a12*a24*a41 + a14*a21*a42 - a11*a24*a42 - a12*a21*a44 - a14*a22*a41);
		mat_out[2][3] = inv_det * (a11*a24*a32 + a12*a21*a34 + a14*a22*a31 - a11*a22*a34 - a12*a24*a31 - a14*a21*a32);
		
		mat_out[3][0] = inv_det * (a21*a33*a42 + a22*a31*a43 + a23*a32*a41 - a21*a32*a43 - a22*a33*a41 - a23*a31*a42);
		mat_out[3][1] = inv_det * (a11*a32*a43 + a12*a33*a41 + a13*a31*a42 - a11*a33*a42 - a12*a31*a43 - a13*a32*a41);
		mat_out[3][2] = inv_det * (a11*a23*a42 + a12*a21*a43 + a13*a22*a41 - a11*a22*a43 - a12*a23*a41 - a13*a21*a42);
		mat_out[3][3] = inv_det * (a11*a22*a33 + a12*a23*a31 + a13*a21*a32 - a11*a23*a32 - a12*a21*a33 - a13*a22*a31);
		
		//mat_out = {{b11, b12, b13, b14}, {b21, b22, b23, b24}, {b31, b32, b33, b34}, {b41, b42, b43, b44}};
	}else if(rows == cols){
		//Source: http://blog.acipo.com/matrix-inversion-in-javascript/
		
		//16 Nov 2013 by Andrew Ippoliti
		// I use Guassian Elimination to calculate the inverse:
    	// (1) 'augment' the matrix (left) by the identity (on the right)
    	// (2) Turn the matrix on the left into the identity by elemetry row ops
    	// (3) The matrix on the right is the inverse (was the identity matrix)
    	// There are 3 elemtary row ops: (I combine b and c in my code)
    	// (a) Swap 2 rows
    	// (b) Multiply a row by a scalar
    	// (c) Add 2 rows
    
    	//if the matrix isn't square: exit (error)
    	// if(M.length !== M[0].length){return;}
    
    	//create the identity matrix (I), and a copy (C) of the original
    	int i = 0;
		int ii = 0;
		int j = 0;
		double e = 0;
		//double t = 0;
    	double mat_out[rows][cols];
		double C[rows][cols];
    	for(i = 0; i < rows; i++){
        	for(j = 0; j < rows; j++){
            	//if we're on the diagonal, put a 1 (for identity)
            	if(i == j){ mat_out[i][j] = 1; }
            	else{ mat_out[i][j] = 0; }
            	// Also, make the copy of the original
            	C[i][j] = mat_in[i][j];
        	}
    	}
    
    	// Perform elementary row operations
    	for(i=0; i<rows; i+=1){
        	// get the element e on the diagonal
        	e = C[i][i];
        
        	// if we have a 0 on the diagonal (we'll need to swap with a lower row)
        	if(e==0){
            	//look through every row below the i'th row
            	for(ii = i+1; ii < rows; ii++){
                	//if the ii'th row has a non-0 in the i'th col
                	if(C[ii][i] != 0){
                    	//it would make the diagonal have a non-0 so swap it
                    	for(j=0; j < rows; j++){
                        	e = C[i][j];       //temp store i'th row
                        	C[i][j] = C[ii][j];//replace i'th row by ii'th
                        	C[ii][j] = e;      //repace ii'th by temp
                        	e = mat_out[i][j];       //temp store i'th row
                        	mat_out[i][j] = mat_out[ii][j];//replace i'th row by ii'th
                        	mat_out[ii][j] = e;      //repace ii'th by temp
                    	}
                    	//don't bother checking other rows since we've swapped
                    	break;
                	}
            	}
            	//get the new diagonal
            	e = C[i][i];
            	//if it's still 0, not invertable (error)
            	if(e==0){
					return;
				}
        	}
        
        	// Scale this row down by e (so we have a 1 on the diagonal)
        	for(j=0; j<rows; j++){
            	C[i][j] = C[i][j]/e; //apply to original matrix
            	mat_out[i][j] = mat_out[i][j]/e; //apply to identity
        	}
        
        	// Subtract this row (scaled appropriately for each row) from ALL of
        	// the other rows so that there will be 0's in this column in the
        	// rows above and below this one
        	for(ii=0; ii<rows; ii++){
            	// Only apply to other rows (we want a 1 on the diagonal)
            	if(ii != i){
				
					// We want to change this element to 0
					e = C[ii][i];
				
					// Subtract (the row above(or below) scaled by e) from (the
					// current row) but start at the i'th column and assume all the
					// stuff left of diagonal is 0 (which it should be if we made this
					// algorithm correctly)
					for(j=0; j<rows; j++){
						C[ii][j] -= e*C[i][j]; //apply to original matrix
						mat_out[ii][j] -= e*mat_out[i][j]; //apply to identity
					}
				}
        	}
    	}
    
    	//we've done all operations, C should be the identity
    	//matrix mat_out should be the inverse:
	}else{
		printf("Error: Dimensions of [%d}[%d] invalid. cmatrix_inverse() requires square matrix", rows, cols);
	}
}

struct ellipse v_ellipse_fit(struct eye_data eye, int start_idx, int end_idx) {
	struct ellipse res;
	double orientation_tolerance = 1e-3;
	double sum_x = 0;
	double sum_y = 0;
	int n = end_idx - start_idx;
	double xs[n];
	double ys[n];
	int i, j, k;
	for(i = 0; i < n; i++){
		xs[i] = eye.xs[i+start_idx];
		sum_x += xs[i];
		sum_y += ys[i];
	}
	double mean_x = sum_x / n;
	double mean_y = sum_y / n;
	
	for(i = 0; i < n; i++){
		xs[i] -= mean_x;
		ys[i] -= mean_y;
	}
	
	double X[n][5];
	double X_prime[5][n];
	double X_square[5][5];
	for(i = 0; i < n; i++){
		X_prime[0][i] = xs[i] * xs[i];
		X_prime[1][i] = xs[i] * ys[i];
		X_prime[2][i] = ys[i] * ys[i];
		X_prime[3][i] = xs[i];
		X_prime[4][i] = ys[i];
		
		X[i][0] = X_prime[0][i];
		X[i][1] = X_prime[1][i];
		X[i][2] = X_prime[2][i];
		X[i][3] = X_prime[3][i];
		X[i][4] = X_prime[4][i];
	}
	
	//matrix multiply X_prime*X
	double dot_sum;
	for(i = 0; i < 5; i++){
		for(j = 0; j < 5; j++){
			dot_sum = 0;
			for(k = 0; k < n; k++){
				dot_sum += X_prime[i][k] * X[k][j];
			}
			X_square[i][j] = dot_sum;
		}
	}
	
	double X_inv[5][5];
	matrix_inverse(5, 5, X_square, X_inv);

	double row_sum[5] = {0, 0, 0, 0, 0};
	for(i = 0; i < n; i++){
		for(j = 0; j < 5; j++){
			row_sum[j] += X[i][j];
		}
    }
	
	double coeffs[5];
	//matrix multiply row_sum*X_inv
	for(j = 0; j < 5; j++){ //TODO: This loop ends at index 4 so coeffs[5] is not addressed
		dot_sum = 0;
		for(k = 0; k < n; k++){
			dot_sum += row_sum[k] * X_inv[k][j];
		}
		coeffs[j] = dot_sum;
	}
	
	res.a = coeffs[0];
	res.b = coeffs[1];
	res.c = coeffs[2];
	res.d = coeffs[3];
	res.e = coeffs[4];
	//res.f = coeffs[5]; //TODO: This element isn't initialized 
	
	double cos_phi, sin_phi, orientation_rad;
	if(min(abs(res.b/res.a), abs(res.b/res.c)) > orientation_tolerance){
		orientation_rad = 0.5 * atan(res.b/(res.c-res.a));
		cos_phi = cos(orientation_rad);
		sin_phi = sin(orientation_rad);
		res.a = res.a*cos_phi*cos_phi - res.b*cos_phi*sin_phi + res.c*sin_phi*sin_phi;
		res.b = 0;
		res.c = res.a*sin_phi*sin_phi + res.b*cos_phi*sin_phi + res.c*cos_phi*cos_phi;
		res.d = res.d*cos_phi - res.e*sin_phi;
		res.e = res.d*sin_phi + res.e*cos_phi;
		mean_x = cos_phi*mean_x - sin_phi*mean_y;
		mean_y = sin_phi*mean_x + cos_phi*mean_y;
	}else{
		orientation_rad = 0;
		cos_phi = cos(orientation_rad);
		sin_phi = sin(orientation_rad);
	}
	
	double test = res.a * res.c;
	if(test > 0){
		res.error_state = false;
	}else{
		res.error_state = true;
	}
	
	if(res.a < 0){
		res.a = -res.a;
		res.c = -res.c;
		res.d = -res.d;
		res.e = -res.e;
	}
	
	res.center_x = mean_x - 0.5*res.d/res.a;
	res.center_y = mean_x - 0.5*res.e/res.c;
	double F = 1.0 + (res.d*res.d) / (4.0*res.a) + (res.e*res.e) / (4*res.c);
	double radius_a = sqrt(F/res.a);
	double radius_b = sqrt(F/res.c);
	res.major_radius = max(radius_a, radius_b);
	res.minor_radius = min(radius_a, radius_b);
	
	
	// double R[2][2];
	// R[0][0] = cos_phi;
	// R[1][0] = -sin_phi;
	// R[0][1] = sin_phi;
	// R[1][1] = cos_phi;
	// double P_in
	
	res.eccentricity = res.minor_radius / res.major_radius;
	res.rotation_angle = orientation_rad * 180.0 / PI;
	
	return res;
}







/* End Wigglesworth Code*/
//////////////////////////////////////////////////////////////////////////







FILE *wfp = 0; //File handle to write data into via socket 'W' command and read via 'r'
FILE *wpp = 0; //Handle for process open for 'r' command when shelling out commands to bash
unsigned int shadow_map[MAPPED_SIZE];
//a shadow array of the mapped memory for the FPGA. This should get a copy of everything written
//to the FPGA so that we can read back what the last value written is.
bool reloadDefaults = false;
int XLowBound[5]={BASE_COS_LOW,END_COS_LOW,PIVOT_COS_LOW,ANGLE_COS_LOW,ROT_COS_LOW};
int XHighBound[5]={BASE_COS_HIGH,END_COS_HIGH,PIVOT_COS_HIGH,ANGLE_COS_HIGH,ROT_COS_HIGH};
int YLowBound[5]={BASE_SIN_LOW,END_SIN_LOW,PIVOT_SIN_LOW,ANGLE_SIN_LOW,ROT_SIN_LOW};
int YHighBound[5]={BASE_SIN_HIGH,END_SIN_HIGH,PIVOT_SIN_HIGH,ANGLE_SIN_HIGH,ROT_SIN_HIGH};
int ForcePossition[5]={0,0,0,0,0};
int ForceDestination[5]={0,0,0,0,0};
int ThreadsExit=1;


int StatusReportIndirection[60]={
	DMA_READ_DATA,DMA_READ_DATA,RECORD_BLOCK_SIZE,END_EFFECTOR_IO_IN,
	BASE_POSITION_AT,  BASE_POSITION_DELTA,  BASE_POSITION_PID_DELTA,  BASE_POSITION_FORCE_DELTA,  BASE_SIN,  BASE_COS,  BASE_MEASURED_ANGLE,  SENT_BASE_POSITION,  SLOPE_BASE_POSITION,0,
	PIVOT_POSITION_AT, PIVOT_POSITION_DELTA, PIVOT_POSITION_PID_DELTA, PIVOT_POSITION_FORCE_DELTA, PIVOT_SIN, PIVOT_COS, PIVOT_MEASURED_ANGLE, SENT_PIVOT_POSITION, SLOPE_PIVOT_POSITION,0,
	END_POSITION_AT,   END_POSITION_DELTA,   END_POSITION_PID_DELTA,   END_POSITION_FORCE_DELTA,   END_SIN,   END_COS,   END_MEASURED_ANGLE,   SENT_END_POSITION,   SLOPE_END_POSITION,0,
	ANGLE_POSITION_AT, ANGLE_POSITION_DELTA, ANGLE_POSITION_PID_DELTA, ANGLE_POSITION_FORCE_DELTA, ANGLE_SIN, ANGLE_COS, ANGLE_MEASURED_ANGLE, SENT_ANGLE_POSITION, SLOPE_ANGLE_POSITION,0,
	ROT_POSITION_AT,   ROT_POSITION_DELTA,   ROT_POSITION_PID_DELTA,   ROT_POSITION_FORCE_DELTA,   ROT_SIN,   ROT_COS,   ROT_MEASURED_ANGLE,   SENT_ROT_POSITION,   SLOPE_ROT_POSITION,0
};

void *map_addr,*map_addrCt;
volatile unsigned int *mapped;
volatile unsigned int *CalTables;
volatile unsigned int *RecordTable;
int CmdVal=0,maxSpeed=0,startSpeed=0,coupledAcceleration=0,forceMode=0,fa0=0,fa1=0,fa2=0,fa3=0,fa4=0,RunMode=0,ServerMode=3;
int BaseBoundryHigh,BaseBoundryLow,EndBoundryHigh,EndBoundryLow,PivotBoundryHigh,PivotBoundryLow,AngleBoundryHigh,AngleBoundryLow,RotateBoundryHigh,RotateBoundryLow;
int Cycle = 0;
struct BotPossition {
	int base;
	int end;
	int pivot;
	int angle;
	int rotate;
	int baseForce;
	int endForce;
	int pivotForce;
	int angleForce;
	int rotateForce;
	int Controlled;
	int ForceMoveState;
};
struct BotSetHomePossition {
	int base;
	int end;
	int pivot;
	int angle;
	int rotate;
};
char RemoteRobotAdd[255];
struct MasterSlaveMoveRatio{
	int base;
	int end;
	int pivot;
	int angle;
	int rotate;	
};
struct MasterSlaveMoveDelta{
	int base;
	int end;
	int pivot;
	int angle;
	int rotate;	
};
struct SlaveBotPossiton{
	int base;
	int end;
	int pivot;
	int angle;
	int rotate;	
};
int AngleHIBoundry;
int AngleLOWBoundry;
int controlled=0;

int DexError=0; //Temporary error code for each command / response
int OverError=0; //overriding error which is always returned with DexError
float fLastTime=0;

int cmditer=0;

struct MasterSlaveMoveRatio MSMoveRatio = {1,1,1,1,1};
struct MasterSlaveMoveDelta MSMoveDelta = {0,0,0,0,0};
struct BotSetHomePossition SetHome = {0,0,0,0,0};
struct SlaveBotPossiton SlaveBotPos = {0,0,0,0,0};
pthread_t tid[10];
struct CaptureArgs {
   char *FileName;
   FILE *fp;
};
struct CaptureArgs CptA,CptMove;

//#define PARAM_LENGTH 20
//char Params[MAX_PARAMS][PARAM_LENGTH+1] = {
//NEVER ADD A NAME STARTING WITH A NUMERICAL DIGIT!
const char* Params[] = {
	"MaxSpeed", 	// 0
	"Acceleration", // 1
	"J1Force",		// 2
	"J3Force",		// 3
	"J2Force",		// 4
	"J4Force",		// 5
	"J5Force",		// 6
	"J1Friction",	// 7
	"J3Friction",	// 8
	"J2Friction",	// 9
	"J4Friction",	//10
	"J5Friction",	//11
	"J1BoundryHigh",//12
	"J1BoundryLow",	//13
	"J3BoundryHigh",//14
	"J3BoundryLow",	//15
	"J2BoundryHigh",//16
	"J2BoundryLow",	//17
	"J4BoundryHigh",//18
	"J4BoundryLow",	//19
	"J5BoundryHigh",//20
	"J5BoundryLow",	//21
	"GripperMotor",	//22
	"EERoll",   	//23
	"EESpan",   	//24
	"StartSpeed",	//25
	"EndSpeed", 	//26
	"ServoSet2X",	//27
	"ServoSet", 	//28
	"RebootServo",	//29
	"RunFile",     	//30 Not actually detected from this list. (was "Ctrl" - Depreciated)
	"J1_PID_P", 	//31
	"J2_PID_P", 	//32
	"J3_PID_P", 	//33
	"J4_PID_P", 	//34
	"J5_PID_P", 	//35
	"AngularSpeed",	//36
	"AngularSpeedStartAndEnd",	//37
	"AngularAcceleration",  	//38
	"CartesianSpeed",        	//39
	"CartesianSpeedStart",  	//40
	"CartesianSpeedEnd",    	//41
	"CartesianAcceleration",	//42
	"CartesianStepSize",    	//43
	"CartesianPivotSpeed",  	//44
	"CartesianPivotSpeedStart",	//45
	"CartesianPivotSpeedEnd",	//46
	"CartesianPivotAcceleration",//47
	"CartesianPivotStepSize",	//48
    "EyeNumbers",           	//49
    "CommandedAngles",      	//50
	"LinkLengths",          	//51 Not actually detected from this list.
	"RawEncoderErrorLimits",   	//52
	"RawVelocityLimits",       	//53
	"MaxTorque",				//54 Maximum Torque before each joint stops trying
	"End"};
#define MAX_PARAMS sizeof(Params) / sizeof(Params[0])

static struct termios old, new;

int PositionAdjust[5]={0,0,0,0,0};

int ForceLimit[5]={4400,4400,4400,800,800};

int MyBotForce[5]={0,0,0,0,0};

float ForceAdjustPossition[5]={-.01,-.01,-.01,-.01,-.01};
int FroceMoveMode=0;

//int LastGoal[5]={0,0,0,0,0}; //moved to above kinematics code 

struct ServoRealTimeData{
	unsigned char ServoAddress;
	int PresentPossition;
	int PresentSpeed;
	int PresentLoad;
	int error;
};

#define NUM_SERVOS 2
struct ServoRealTimeData ServoData[NUM_SERVOS];




float JointsCal[5];
//int SoftBoundries[5];

struct UARTTransactionPacket{
	unsigned short ServoNum;
	int PacketLength;
	int ActionAddress;
	int ActionSent;
	int ActionLength;
	unsigned char *ActionData;
	int ResponseLength;	
	unsigned char *ResponseData;
	int ResponseReceived;
	int error;
};




unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
	unsigned short i, j;
	unsigned short crc_table[256] = {0x0000,
	                                0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	                                0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
	                                0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
	                                0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
	                                0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
	                                0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
	                                0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
	                                0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
	                                0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	                                0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
	                                0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
	                                0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
	                                0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
	                                0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
	                                0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
	                                0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
	                                0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	                                0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
	                                0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
	                                0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
	                                0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
	                                0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
	                                0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
	                                0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
	                                0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	                                0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
	                                0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
	                                0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
	                                0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
	                                0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
	                                0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
	                                0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
	                                0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	                                0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
	                                0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
	                                0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
	                                0x820D, 0x8207, 0x0202 };

	for(j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

void UnloadUART(unsigned char* RxBuffer,int length)
{
	int i;
	unsigned char RecData;
	for(i = 0;i < length + 11; i++)
	{
		mapped[UART1_XMIT_CNT]=shadow_map[UART1_XMIT_CNT]= 16; // generate next data pull
		RecData = mapped[UART_DATA_IN];
		RxBuffer[i] = RecData;
		#ifdef DEBUG_XL320_UART
		printf(" %x ", RecData);
		#endif
		mapped[UART1_XMIT_CNT]=shadow_map[UART1_XMIT_CNT]= 0; // generate next data pull		 
   	}
}

void SendGoalSetPacket(int newPos, unsigned char servo)
{
 	//int i;
  	unsigned char RxBuf[27];
  	unsigned char TxPacket[] =  {0xff, 0xff, 0xfd, 0x00, servo, 0x07, 0x00, 0x03, 30, 0, newPos & 0x00ff, (newPos >> 8) & 0x00ff, 0, 0};
  	unsigned short crcVal;
	//printf("Calculate CRC\n");
  	crcVal = update_crc(0, TxPacket, 12);
  	TxPacket[12]=crcVal & 0x00ff;
  	TxPacket[13]=(crcVal >> 8) & 0x00ff;
 
	//printf("Packet ready\n");
	SendPacket(TxPacket, 14, CalcUartTimeout(14 + 14),RxBuf, 16);  // send time plus receive time in bytes transacted
  	//UnloadUART(RxBuf,16); // TODO refine actual size
}
void SendWrite2Packet(int WData, unsigned char servo, int WAddres)
{
 	int i;
  	unsigned char RxBuf[27];
  	unsigned char TxPacket[] =  {0xff, 0xff, 0xfd, 0x00, servo, 0x07, 0x00, 0x03, WAddres & 0x00ff, (WAddres >> 8) & 0x00ff, WData & 0x00ff, (WData >> 8) & 0x00ff, 0, 0};
  	unsigned short crcVal;
  	crcVal = update_crc(0, TxPacket, 12);
  	TxPacket[12]=crcVal & 0x00ff;
  	TxPacket[13]=(crcVal >> 8) & 0x00ff;

	SendPacket(TxPacket, 14, CalcUartTimeout(14 + 14),RxBuf,16);  // send time plus receive time in bytes transacted
  	//UnloadUART(RxBuf,16); // TODO refine actual size
}
void SendWrite1Packet(unsigned char WData, unsigned char servo, int WAddres)
{
 	int i;
  	unsigned char RxBuf[27];
  	unsigned char TxPacket[] =  {0xff, 0xff, 0xfd, 0x00, servo, 0x06, 0x00, 0x03, WAddres & 0x00ff, (WAddres >> 8) & 0x00ff, WData, 0, 0};
  	unsigned short crcVal;
  	crcVal = update_crc(0, TxPacket, 11);
  	TxPacket[11]=crcVal & 0x00ff;
  	TxPacket[12]=(crcVal >> 8) & 0x00ff;

	SendPacket(TxPacket, 13, CalcUartTimeout(14 + 14),RxBuf,16);  // send time plus receive time in bytes transacted
  	//UnloadUART(RxBuf,16); // TODO refine actual size
}
void RebootServo(unsigned char servo)
{
 	int i;
  	unsigned char RxBuf[27];
  	unsigned char TxPacket[] =  {0xff, 0xff, 0xfd, 0x00, servo, 0x03, 0x00, 0x08, 0, 0};
  	unsigned short crcVal;
  	crcVal = update_crc(0, TxPacket, 8);
  	TxPacket[8]=crcVal & 0x00ff;
  	TxPacket[9]=(crcVal >> 8) & 0x00ff;

	SendPacket(TxPacket, 10, CalcUartTimeout(14 + 14),RxBuf,16);  // send time plus receive time in bytes transacted
  	//UnloadUART(RxBuf,16); // TODO refine actual size
}

int Fcritical = 0;
int UARTBaudRate = 20; // .0000086 seconds 9 microseconds  1/115200
int ResponseDelay = 100;

int CalcUartTimeout(int size)
{
	return UARTBaudRate * size * 10 + ResponseDelay;   // 1 start bit 8 data 1 stop + response delay
}

void SendPacket(unsigned char *TxPkt, int length, int TxRxTimeDelay, unsigned char *RxPkt, int ReadLength)
{
	int i;
	while(Fcritical != 0){usleep(1000);} // wait until previous call is complete
	
  	mapped[END_EFFECTOR_IO]=shadow_map[END_EFFECTOR_IO]=IO_DYNAMIXEL;
  	mapped[UART1_XMIT_TIMEBASE]=shadow_map[UART1_XMIT_TIMEBASE]= 868;
 	mapped[UART1_XMIT_CNT]=shadow_map[UART1_XMIT_CNT]= 1;  // reset send queue

	Fcritical = 1;
  	for(i = 0;i < length;i++)
  	{
  
#ifdef DEBUG_XL320_UART

  		printf("Sending UART Data %x \n", TxPkt[i]);
#endif
 		mapped[UART1_XMIT_DATA]=shadow_map[UART1_XMIT_DATA]= TxPkt[i];
		mapped[UART1_XMIT_CNT]=shadow_map[UART1_XMIT_CNT]= 4;
  		mapped[UART1_XMIT_CNT]=shadow_map[UART1_XMIT_CNT]= 0;
    
  	}
  	mapped[UART1_XMIT_CNT]=shadow_map[UART1_XMIT_CNT]= 2;
  	mapped[UART1_XMIT_CNT]=shadow_map[UART1_XMIT_CNT]= 0;
	usleep(TxRxTimeDelay);
	UnloadUART(RxPkt,ReadLength);
	Fcritical = 0;
}
int SendReadPacket(unsigned char* RxBuffer, unsigned char servo,int start, int length) 
{ 
  int i;
  unsigned char TxPacket[] =  {0xff, 0xff, 0xfd, 0x00, servo, 0x07, 0x00, 0x02, start & 0x00ff, (start >> 8) & 0x00ff, length & 0x00ff, (length >> 8) & 0x00ff, 0, 0};
  unsigned short crcVal;
  crcVal = update_crc(0, TxPacket, 12);
  TxPacket[12]=crcVal & 0x00ff;
  TxPacket[13]=(crcVal >> 8) & 0x00ff;


#ifdef DEBUG_XL320_UART

  printf("Sending UART Data Read start %d length %d \n", start, length);

#endif

	SendPacket(TxPacket, 14, CalcUartTimeout(14 + length + 5),RxBuffer, length+7);  // send time plus receive time in bytes transacted
  	//UnloadUART(RxBuf,Length + 7); // TODO refine actual size
}

struct monitorbot {
	int enc_position[NUM_JOINTS]; //last known joint position. Maybe raw or calibrated.
	int enc_velocity[NUM_JOINTS]; //last known joint velocity (e.g. last position - current position)
	int enc_velocity_limit[NUM_JOINTS]; //maximum allowed raw encoder velocity
	int enc_velocity_count; //number of velocity errors seen
	int enc_velocity_count_limit; 
	int enc_error_limit[NUM_JOINTS]; //maximum allowed error between raw encoder and commanded position
	bool start; //is this the first read?
	int joints_off[NUM_JOINTS]; //offset for raw joint positions
	} bot_state;

void printPosition()
{
	int a1,a2,a3,a4,a5;
	a1=getNormalizedInput(BASE_POSITION_AT)+getNormalizedInput(BASE_POSITION_FORCE_DELTA);
	a2=getNormalizedInput(PIVOT_POSITION_AT)+getNormalizedInput(PIVOT_POSITION_FORCE_DELTA);
	a3=getNormalizedInput(END_POSITION_AT)+getNormalizedInput(END_POSITION_FORCE_DELTA);
	a4=getNormalizedInput(ANGLE_POSITION_AT)+getNormalizedInput(ANGLE_POSITION_FORCE_DELTA);
	a5=getNormalizedInput(ROT_POSITION_AT)+getNormalizedInput(ROT_POSITION_FORCE_DELTA);
	//Commanded angles
	printf(" %d,%d,%d,%d,%d                     \r",a1,a2,a3,a4,a5); // \r moves to start of same line
	fflush(stdout); //actually print the data despite no \n end of string.
}

void setOpenLoop() {
	mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=shadow_map[DIFF_FORCE_SPEED_FACTOR_ANGLE]= 0;
	mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=shadow_map[DIFF_FORCE_SPEED_FACTOR_ANGLE]= 0;
	mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=shadow_map[DIFF_FORCE_SPEED_FACTOR_ROT]= 0;
	mapped[SPEED_FACTORA]=shadow_map[SPEED_FACTORA]= 0;
	mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]= 0;
	mapped[PID_P]=shadow_map[PID_P]= 0;
	mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]= 1;
	mapped[PID_P]=shadow_map[PID_P]= 0;
	mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]= 2;
	mapped[PID_P]=shadow_map[PID_P]= 0;
	mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]= 3;
	mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]= 4;
	mapped[COMMAND_REG]=shadow_map[COMMAND_REG]= 12960;
	printf("\nOpen Loop mode set\n");
	}

void monitorTorque() {
	char iString[ISTRING_LEN];
	int err_arc = 0; //amount of error in arc seconds
	int err_at_arc = 0; //actual position where error occured
	const int pos_at[NUM_JOINTS] = {BASE_POSITION_AT,PIVOT_POSITION_AT,END_POSITION_AT,ANGLE_POSITION_AT,ROT_POSITION_AT};
	const int pos_force_delta[NUM_JOINTS] = {BASE_POSITION_FORCE_DELTA, PIVOT_POSITION_FORCE_DELTA, 
			END_POSITION_FORCE_DELTA, ANGLE_POSITION_FORCE_DELTA, ROT_POSITION_FORCE_DELTA};
	const int pos_raw_enc[NUM_JOINTS] = {BASE_RAW_ENCODER_ANGLE_FXP, PIVOT_RAW_ENCODER_ANGLE_FXP,
			END_RAW_ENCODER_ANGLE_FXP, ANGLE_RAW_ENCODER_ANGLE_FXP, ROT_RAW_ENCODER_ANGLE_FXP};
	const int joints_slots[NUM_JOINTS] = {200, 184, 157, 115, 100};
 	//const int joints_corr[NUM_JOINTS] = {1938, 1909, 1946, 1657, 1783};
	const int joints_corr[] = {2063, 2095, 2055, 2141, 2242};
	//const int joints_corr[] = {2000, 2000, 2000, 2100, 2100};
	// printf("pos_at is %i long\n", sizeof(pos_at)/sizeof(pos_at[0]));
	// printf("%i = %i\n", BASE_POSITION_AT, pos_at[0]);
	// printf("%i = %i\n", BASE_POSITION_FORCE_DELTA, pos_force_delta[0]);
	FILE *err_file;
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	for (int i = 0; i < NUM_JOINTS; i++) {
		//printf("J%i, ",i+1);
		//Commanded angles
		int pos_cmd = getNormalizedInput(pos_at[i])+getNormalizedInput(pos_force_delta[i]);
		//TODO: 
		//raw encoder angles. Format is 18.14 bits fixed point. 
		int pos_raw = ( (float)((int)mapped[ pos_raw_enc[i] ])  /  ( pow(2,14) * joints_slots[i])  ) * joints_corr[i];
		if (bot_state.start) bot_state.joints_off[i] = pos_raw; //if this is the first time, record this offset
		pos_raw -= bot_state.joints_off[i]; //apply the inital offset from zero. 
		//int pos_raw = ( (float)((int)mapped[ pos_raw_enc[i] ])  /  ( pow(2,14) * 40 * (360/joints_slots[i]*3600) ) );
		//TODO: Why 40? And is the slots array correct?
		//int pos_raw = ( (int)mapped[ pos_raw_enc[i] ]  / pow(2,14) ) * 2000/joints_slots[i];
		int pos_vel = bot_state.enc_position[i] - pos_raw;
		//printf("%d ", pos_vel);
		// TODO: Compare to maxSpeed_arcsec_per_sec once timebase is better known.
		if (abs(pos_vel) > bot_state.enc_velocity_limit[i]) { 
			bot_state.enc_velocity_count += NUM_JOINTS;
			err_file = fopen("errors.log", "a");
			if (err_file) {
				tm = *localtime(&t);
				fprintf(err_file, "%04d/%02d/%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
				fprintf(err_file, ", Joint %i velocity %d. \n", i+1, pos_vel);
				fclose(err_file);
				printf("Joint %i velocity %d. \n", i+1, pos_vel);
				}
			DexError = MONITOR_ERROR_CODE;
			if (bot_state.enc_velocity_count > bot_state.enc_velocity_count_limit) {
				// strlcpy(iString, "S RunFile ErrorRawVelocity.make_ins ;\0", ISTRING_LEN);
				// printf("\nJoint %i velocity %d. Starting %s returned %d\n",i+1, pos_vel, iString, ParseInput(iString));
				// the above crashes if any other commands are sent at the same time. e.g. from DDE. 
				err_file = fopen("errors.log", "a");
				if (err_file) {
					tm = *localtime(&t);
					fprintf(err_file, "%04d/%02d/%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
					fprintf(err_file, ", Joint %i velocity errors: Set open loop mode \n", i+1);
					fclose(err_file);
					printf("Joint %i velocity errors: Set open loop mode \n", i+1);
					}
				setOpenLoop();
				}
			}
		else {
			if (bot_state.enc_velocity_count > 0) bot_state.enc_velocity_count--;
		}
		bot_state.enc_velocity[i] = pos_vel;
		bot_state.enc_position[i] = pos_raw;
		//JointCal is arcseconds per microstep. 
		//6th line in AxisCal is the ratio between joint 4 and joint 3
		err_arc = pos_cmd - pos_raw; //compute the error
		//printf("%d, ", pos_cmd);
		//printf("%d, \t", pos_raw);
		//printf(" %d, ", err_arc);
		if (abs(err_arc) > bot_state.enc_error_limit[i]) {
			err_file = fopen("errors.log", "a");
			if (err_file) {
				tm = *localtime(&t);
				fprintf(err_file, "%04d/%02d/%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
				fprintf(err_file, ", Joint %i off by %i at %i vs %d\n",i+1, err_arc, pos_raw, pos_cmd);
				fclose(err_file);
				printf("Joint %i off by %i at %i vs %d\n",i+1, err_arc, pos_raw, pos_cmd);
				}
			DexError = MONITOR_ERROR_CODE;
			//strlcpy(iString, "S RunFile ErrorRawPosition.make_ins ;\0", ISTRING_LEN);
			//printf("\nStarting %s returned %d\n", iString, ParseInput(iString));
			// the above crashes if any other commands are sent at the same time. e.g. from DDE. 
			}
		}
	// printf("J2 AT:%8d, m:%8d, d:%8d, pd:%8d, fd:%8d", //\r moves to start of same line
	// 	getNormalizedInput(PIVOT_POSITION_AT), // 
	// 	getNormalizedInput(PIVOT_MEASURED_ANGLE), // 
	// 	getNormalizedInput(PIVOT_POSITION_DELTA), // 
	// 	getNormalizedInput(PIVOT_POSITION_PID_DELTA), // 
	// 	getNormalizedInput(PIVOT_POSITION_FORCE_DELTA) // 
	// 	);
	//printf("        \r" );// \r moves to start of same line, needs fflush.
	fflush(stdout); //actually print the data despite no \n end of string.
	bot_state.start = false;

	
	//Eye numbers e.g. which slot is the disk at
	// const int joints_slots[5] = {200, 184, 157, 115, 100};
	// printf(" %i,%i,%i,%i,%i ",
	// 	(int) (a1 * joints_slots[0] / (360 * 3600)),
	// 	(int) (a2 * joints_slots[1] / (360 * 3600)),
	// 	(int) (a3 * joints_slots[2] / (360 * 3600)),
	// 	(int) (a4 * joints_slots[3] / (360 * 3600)),
	// 	(int) (a5 * joints_slots[4] / (360 * 3600))
	// 	); 
	// printf("- %i,%i,%i,%i,%i              \r", // \r moves to start of same line
	// 	mapped[BASE_EYE_NUMBER]-255, 
	// 	mapped[PIVOT_EYE_NUMBER]-255, 
	// 	mapped[END_EYE_NUMBER]-255, 
	// 	mapped[ANGLE_EYE_NUMBER]-255, 
	// 	mapped[ROT_EYE_NUMBER]-255
	// 	);

}


int sign(int i)
{
	if(i<0)
		return -1;
	if(i>0)
		return 1;
	return 0;
}

void *RealtimeMonitor(void *arg)
{
	int* ExitState = arg;
	int i,j,ForceDelta,disTime=0;
	FILE *err_file;
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	unsigned char ServoRx[64];
	unsigned char err;
	while(*ExitState) {

	if (IO_DYNAMIXEL == shadow_map[END_EFFECTOR_IO]) { //only if the Dynamixels are setup
		ServoData[0].PresentPossition = ServoRx[16] + (ServoRx[17]<<8);
		ServoData[0].PresentSpeed = ServoRx[18] + (ServoRx[19]<<8);
		ServoData[0].PresentLoad = ServoRx[20] + (ServoRx[21]<<8);
		ServoData[0].error = ServoRx[29];
		//printf("\nRaw 16:%x %d ",ServoRx[16],ServoRx[16]);
		//printf(" 17:%x %d ",ServoRx[17],ServoRx[17]);
		//printf(" Possition %d Speed %d Load %d \n", ServoData[0].PresentPossition,ServoData[0].PresentSpeed,ServoData[0].PresentLoad);


		ServoData[1].PresentPossition = ServoRx[16] + (ServoRx[17]<<8);
		ServoData[1].PresentSpeed = ServoRx[18] + (ServoRx[19]<<8);
		ServoData[1].PresentLoad = ServoRx[20] + (ServoRx[21]<<8);
		ServoData[1].error = ServoRx[29];

		} //done with Dynamixel servos
		// if(FroceMoveMode==1) {
		// 	// do force based movement
		// 	for(i=0;i<5;i++) {
		// 		ForceDelta=ForcePossition[i]-getNormalizedInput(BASE_POSITION_FORCE_DELTA+i);
		// 		if(abs(ForceDelta)>500) {
		// 			ForceDelta=(abs(ForceDelta)-500)*sign(ForceDelta);
		// 		}
		// 		//ForceDelta=ForceDelta;//+MyBotForce[i];
		// 		if(disTime==90) {
		// 			//for(j=0;j<5;j++)
		// 			//printf(" Force %d ",ForceDelta);
		// 		}

		// 		if(abs(ForceDelta)<ForceLimit[i]) {
		// 			mapped[FORCE_BIAS_BASE+i]=shadow_map[FORCE_BIAS_BASE+i]=ForceDelta;
		// 		}
		// 		else {
		// 			mapped[FORCE_BIAS_BASE+i]=shadow_map[FORCE_BIAS_BASE+i]=sign(ForceDelta)*ForceLimit[i];
		// 		}

		// 		//ForcePossition[i]=ForcePossition[i]+(int)((float)ForceDelta*ForceAdjustPossition[i]);
		// 		if(ForceDestination[i]>ForcePossition[i]) {
		// 			ForcePossition[i]=ForcePossition[i]+10;
		// 		}
		// 		else 				{
		// 			ForcePossition[i]=ForcePossition[i]-10;
		// 		}
		// 	}
		// }
#ifdef DEBUG_MONITOR
		disTime++;
		if(disTime>20) {
			//printPosition();
			monitorTorque();
			disTime = 0;
		}
#endif
		usleep(30000);
	}
	printf("\nMonitor Thread Exiting\n");
    return NULL;
}

void SetGripperRoll(int Possition)
{
	SendGoalSetPacket(Possition, 3);
   	//printf("Moving Servo 3 to %u\n",Possition);

	/*int ServoSpan=(SERVO_HI_BOUND-SERVO_LOW_BOUND)/360;
	mapped[END_EFFECTOR_IO]=shadow_map[END_EFFECTOR_IO]=80;

	mapped[SERVO_SETPOINT_B]=shadow_map[SERVO_SETPOINT_B]=(ServoSpan*Possition)+SERVO_LOW_BOUND;
//	mapped[SERVO_SETPOINT_B]=shadow_map[SERVO_SETPOINT_B]=Possition;
*/	
}
void SetGripperSpan(int Possition)
{
	//SendReadPacket(3,30,21);       
	SendGoalSetPacket(Possition, 1);
	//printf("Moving Servo 1 to %u\n",Possition);

/*	int ServoSpan=(SERVO_HI_BOUND-SERVO_LOW_BOUND)/360;
	mapped[END_EFFECTOR_IO]=shadow_map[END_EFFECTOR_IO]=80;

	mapped[SERVO_SETPOINT_A]=shadow_map[SERVO_SETPOINT_A]=(ServoSpan*Possition)+SERVO_LOW_BOUND;
//	mapped[SERVO_SETPOINT_B]=shadow_map[SERVO_SETPOINT_B]=Possition;
*/	
}
void SetGripperMotor(int state, int on_width, int off_width)
{ // pin 7 of connector J19 on bottom of PCB. TODO make ON and OFF widths parameters. 
	mapped[GRIPPER_MOTOR_ON_WIDTH]=shadow_map[GRIPPER_MOTOR_ON_WIDTH]=on_width; //12000;
	mapped[GRIPPER_MOTOR_OFF_WIDTH]=shadow_map[GRIPPER_MOTOR_OFF_WIDTH]=off_width; //0;
	mapped[GRIPPER_MOTOR_CONTROL]=shadow_map[GRIPPER_MOTOR_CONTROL]=state; //Pulse Width
}
void *StartServerSocketDDE(void *arg)
{
	struct timeval tv;
	tv.tv_sec = 30;  /* 30 Secs Timeout */
	tv.tv_usec = 0;  // Not init'ing this can cause strange errors

	int* ExitState = arg;
	int listenfd = 0, connfd = 0,RLength = 0,SLength = 0;
    struct sockaddr_in serv_addr; 

    char sendBuff[256];
    char recBuff[256];
    time_t ticks; 
	bool SocketLive=0;
	//printf("\n Start DDE Socket Server \n");

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(50000); 

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)); 

    listen(listenfd, 10); 
	
	setsockopt(listenfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
    while(*ExitState)
    {
        // connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
        while ( (connfd = accept(listenfd, (struct sockaddr*)NULL, NULL)) < 0)
		{
			//printf("error on accept()! %s\n", strerror(errno));
			////printf("\n Timeout no connect \n");
		} 
		//printf("\n new connect \n");
		
		
		//ticks = time(NULL);
		//strcpy(sendBuff,"Dexter Connect Service Connected\n");
		//sn//printf(sendBuff, sizeof(sendBuff), "%.24s\r\n", ctime(&ticks));
		//write(connfd, sendBuff, strlen(sendBuff)); 
		SocketLive=TRUE;
//		while(SocketLive==TRUE)
		{
			while((errno = 0, (RLength = recv(connfd, recBuff, sizeof(recBuff), 0))>0) || 
			errno == EINTR)
			{
				if(RLength>0)
				{
					////printf("Receive size %i ",RLength);
					//output.append(recBuff, RLength);
				}
					
			 

				if(RLength < 0)
				{
					//printf("%s \n",strerror(errno));
					/* handle error - for example throw an exception*/
					SocketLive = FALSE;
				}
				

//			while ( (RLength = recv (connfd,recBuff,sizeof(recBuff),0 )) > 0)
				{
					//recBuff[RLength]=0;
					if (ProcessServerReceiveDataDDE(recBuff)) {
						(void)ProcessServerSendDataDDE(sendBuff,recBuff);/*==TRUE)*/
						write (connfd,sendBuff,60*4/*sizeof(sendBuff)*/); 
						}
				}
			}
			//printf("error code %s \n",strerror(errno));			
		}
		//printf("\n Socket Read Zero closing socket\n");
        close(connfd);
        //sleep(1);
     }
    return NULL;
}

void *StartServerSocket(void *arg)
{
	int* ExitState = arg;
	int listenfd = 0, connfd = 0,RLength = 0,SLength = 0;
    struct sockaddr_in serv_addr; 

    char sendBuff[64];
    char recBuff[64];
    time_t ticks; 
	bool SocketLive=0;

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(sendBuff, '0', sizeof(sendBuff)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(40000); 

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)); 

    listen(listenfd, 10); 

    while(*ExitState)
    {
        connfd = accept(listenfd, (struct sockaddr*)NULL, NULL); 
		//printf("\n new connect \n");
		
		//ticks = time(NULL);
		strcpy(sendBuff,"Dexter Connect Service Connected\n");
		//sn//printf(sendBuff, sizeof(sendBuff), "%.24s\r\n", ctime(&ticks));
		write(connfd, sendBuff, strlen(sendBuff)); 
		SocketLive=TRUE;
		
		while(SocketLive==TRUE)
		{
			while ( (RLength = recv (connfd,recBuff,sizeof(recBuff),0 )) > 0)
			{
				//recBuff[RLength]=0;
				ProcessServerReceiveData(recBuff);
				SocketLive = ProcessServerSendData(sendBuff);
				write (connfd,sendBuff,sizeof(sendBuff)); 
			}
		}
		//printf("\n Socket Read Zero closing socket\n");
        close(connfd);
       
    }
    return NULL;
}

int MaxForce(int Max,int Val)
{
	if(abs(Max) > abs(Val))
	{
		return Val;
	}
	else
	{
		return abs(Max)*sign(Val);
	}
}
void ProcessServerReceiveData(char *recBuff)
{
	struct BotPossition MyBot;
	int MxForce=9800;
	float fScale=1;
	memcpy(&MyBot,recBuff,sizeof(MyBot));
	////printf("Server %d %d %d %d %d \n",MyBot.baseForce,MyBot.endForce,MyBot.pivotForce,MyBot.angleForce,MyBot.rotateForce);
	if(MyBot.Controlled!=0)
	{
		/*MyBotForce[0]=MaxForce(MxForce,MyBot.baseForce);
		MyBotForce[1]=MaxForce(MxForce,MyBot.endForce);
		MyBotForce[2]=MaxForce(MxForce,MyBot.pivotForce);
		MyBotForce[3]=MaxForce(MxForce,MyBot.angleForce);
		MyBotForce[4]=MaxForce(MxForce,MyBot.rotateForce);*/
		if(FroceMoveMode==0)
		{/*
			mapped[FORCE_BIAS_BASE]=shadow_map[FORCE_BIAS_BASE]=MaxForce(MxForce,(int)((float)MyBot.baseForce)*fScale);
			mapped[FORCE_BIAS_END]=shadow_map[FORCE_BIAS_END]=MaxForce(MxForce,(int)((float)MyBot.endForce)*fScale);
			mapped[FORCE_BIAS_PIVOT]=shadow_map[FORCE_BIAS_PIVOT]=MaxForce(MxForce,(int)((float)MyBot.pivotForce)*fScale);
			mapped[FORCE_BIAS_ANGLE]=shadow_map[FORCE_BIAS_ANGLE]=MaxForce(MxForce,(int)((float)MyBot.angleForce)*fScale);
			mapped[FORCE_BIAS_ROT]=shadow_map[FORCE_BIAS_ROT]=MaxForce(MxForce,(int)((float)MyBot.rotateForce)*fScale);
*/		
		}
		else
		{
			ForcePossition[0]=MyBot.base;
			ForcePossition[1]=MyBot.end;
			ForcePossition[2]=MyBot.pivot;
			ForcePossition[3]=MyBot.angle;
			ForcePossition[4]=MyBot.rotate;
		}
		FroceMoveMode=MyBot.ForceMoveState;
		
	}
	
	//MoveRobot(MyBot.base,MyBot.end,MyBot.pivot,MyBot.angle,MyBot.rotate,BLOCKING_MOVE);
/*	Cycle++;
	if(Cycle>100)
		return FALSE;
	else	
		return TRUE;*/
}
bool ProcessServerSendData(char *sendBuff)
{
	
	struct BotPossition MyBot={0,0,0,0,0,0,0,0,0,0,0,0};
	static int Cycle = 0;
	MyBot.baseForce=getNormalizedInput(BASE_POSITION_DELTA);
	MyBot.endForce=getNormalizedInput(END_POSITION_DELTA);
	MyBot.pivotForce=getNormalizedInput(PIVOT_POSITION_DELTA);
	MyBot.angleForce=getNormalizedInput(ANGLE_POSITION_DELTA);
	MyBot.rotateForce=getNormalizedInput(ROT_POSITION_DELTA);
	MyBot.Controlled=controlled;
	MyBot.base=getNormalizedInput(BASE_POSITION_FORCE_DELTA);
	MyBot.end=getNormalizedInput(END_POSITION_FORCE_DELTA);
	MyBot.pivot=getNormalizedInput(PIVOT_POSITION_FORCE_DELTA);
	MyBot.angle=getNormalizedInput(ANGLE_POSITION_FORCE_DELTA);
	MyBot.rotate=getNormalizedInput(ROT_POSITION_FORCE_DELTA);
	MyBot.ForceMoveState=FroceMoveMode;
	memcpy(sendBuff,&MyBot,sizeof(MyBot));
//	
//	s//printf(sendBuff, "%d \n", Cycle);
//	Cycle++;
	return TRUE;
}





bool ProcessServerSendDataDDE(char *sendBuff,char *recBuff)
{
	int i,addr,last;
	int *sendBuffReTyped;
	const char delimiters[] = " ,";
	char *token;
	char oplet;
	float timeStart=0;
    //float timeNormal = 1494640000000;
	//long iTimeNormal = 1494628400;
	//int iTime=0;
	//int iElTime=0;
	long            ms; // Millisecondski
    time_t          s;  // Seconds
	long lTime;
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);

    s  = spec.tv_sec;

    ms = spec.tv_nsec / 1.0e3; // Convert nanoseconds to milliseconds
	lTime = (long)s;
	//printf("\nAT %ld local (lTime)\n",lTime);
	//timeStart = (float)(lTime - iTimeNormal); //no longer used
	//timeStart = timeStart + (float)ms/1000000.0;
	//iElTime=timeStart*1000; //no longer used
	//printf("\n at %d normalized (iElTime)\n",iElTime);
    //printf("\n %li %li %li",spec.tv_nsec ,spec.tv_sec, iElTime);
	//fLastTime = timeStart;
	
/*	for(i=0;i<25;i++)
	{
		//printf("%c",recBuff[i]);
	}*/
//	if(recBuff[0]=="g")

	token = strtok (recBuff, delimiters); //Job ID?
	////printf("returning heartbeat\n");
	sendBuffReTyped=(int *)sendBuff;
	sendBuffReTyped[0]=atoi(token);
	token = strtok (NULL, delimiters); //Instruction ID?
	//printf("\n %s \n",token);
	sendBuffReTyped[1]=atoi(token);
	token = strtok (NULL, delimiters); //Start Time?
	//printf("\n %s \n",token);
	timeStart = atof(token);
	lTime = timeStart / 1000; //convert ms to s
	//printf("\n at %ld raw DDE (lTime)\n",lTime);
	if (lTime - (long)s > 60*60*24*7*52) { //our clock is more than a year behind the PC
		printf("\nUpdating sys time from %ld to %ld (from DDE)\n",s,lTime);
		spec.tv_sec = (time_t)lTime; //set our clock to the PC clock
		if (-1 == clock_settime(CLOCK_REALTIME, &spec)) { //try to set
			printf("failed to set time\n");
		}
	}
	//iTime = timeStart - timeNormal; //no longer used
	//printf("\n at %d normalized (iTime)\n",iTime);

	sendBuffReTyped[2] = spec.tv_sec;//iTime;//   this shoud be start_time_internal broken into 2 words atoi(token);
	token = strtok (NULL, delimiters); //End time?
	//printf("\n %s \n",token);
	sendBuffReTyped[3]=(int)(spec.tv_nsec  / 1.0e3);//iElTime;//   this shoud be start_time_internal broken into 2 words atoi(token);
	token = strtok (NULL, delimiters); //Oplet?
	oplet = token[0]; //printf("\n Oplet:%c.",oplet);
	sendBuffReTyped[4]=token[0];
	sendBuffReTyped[5]=DexError | OverError;
	if('r'==oplet) { //printfs included for debugging in this block only since speed isn't critical at this point.
		//printf("\n r:read_from_robot\n");
		token = strtok (NULL, delimiters); //length
		i = atoi(token); //number of block to read
		//printf("read block %d \n",i);
		token=strtok(NULL, delimiters);//filename
		//printf("opening file:%s.\n ",token);
		//if(wfp>0) {fclose(wfp);} //not needed?
		
		static int mat_string_length = 0;
		static char mat_string[256];
		static char* ptr_mat_string;
		switch(token[0]) { //what's the first character of the path?
		case '`': //shell cmd
			//printf("shell %s\n", token);
			if(0==i){ //first request
				printf("- popen\n");
				if (wpp) { //we can't start a new one, 'case the old one isn't done.
					sendBuffReTyped[5] = EBUSY; //the system is busy
					sendBuffReTyped[6] = 0; //no bytes returned.
					break; //Host should request block 1 or higher with "`" and toss the data until EOF
				}
				wpp = popen(token+1, "r"); //to get both stdout and stderr, the command should end with "2>&1"
				if (errno) {
					sendBuffReTyped[5] = errno; //there was an error
					sendBuffReTyped[6] = 0; //no bytes returned
				}
			}
			if(wpp){
				sendBuffReTyped[6] = fread(sendBuff + sizeof(sendBuffReTyped[0])*7, 1, MAX_CONTENT_CHARS, wpp);
				//possible issue: If child process is working to produce more output, you might get less than
				//MAX_CONTENT_CHARS /before/ the child is done. Need to KEEP reading until you get EOF!
				if(feof(wpp)) { //printf("EOF\n");//stream will be set to eof when process is finished
					errno = pclose(wpp); //can this lock up? Shouldn't if the stream is eof
					sendBuffReTyped[5] = errno; //might be zero (no error) or the error returned by the command
					wpp = 0; //must zero out wpp so we know it's closed.
				}
			}else { //printf("no wpp");
				sendBuffReTyped[5] = ECHILD; //we are done
				sendBuffReTyped[6] = 0; //no bytes returned
			}
			break;
		case '#': //special keyword "files"
			if(0 == i){ //if it's the first time, make the content.
				//Switch case for keywords in read from robot call
				if(strcmp(token, "#POM") == 0 || strcmp(token, "#XYZ") == 0){
					struct J_angles measured_angles = new_J_angles(
						(float)(getNormalizedInput(BASE_MEASURED_ANGLE)),
						(float)(getNormalizedInput(PIVOT_MEASURED_ANGLE)),
						(float)(getNormalizedInput(END_MEASURED_ANGLE)),
						(float)(getNormalizedInput(ANGLE_MEASURED_ANGLE)),
						(float)(getNormalizedInput(ROT_MEASURED_ANGLE))
					);
					struct pos_ori_mat A = J_angles_to_pos_ori_mat(measured_angles);

					//pos_ori_mat_to_string(measured_pos_ori_mat, mat_string);
					
					mat_string_length = sprintf(mat_string, "[[%f, %f, %f, %f],[%f, %f, %f, %f],[%f, %f, %f, %f],[%f, %f, %f, %f]]", A.r0.c0, A.r0.c1, A.r0.c2, A.r0.c3, A.r1.c0, A.r1.c1, A.r1.c2, A.r1.c3, A.r2.c0, A.r2.c1, A.r2.c2, A.r2.c3, A.r3.c0, A.r3.c1, A.r3.c2, A.r3.c3);
					//printf("mat_string: %s length: %d", mat_string, mat_string_length);
				}else if(strcmp(token, "#EyeNumbers") == 0){
                    mat_string_length = sprintf(mat_string, "[%i, %i, %i, %i, %i]", mapped[BASE_EYE_NUMBER], mapped[PIVOT_EYE_NUMBER], mapped[END_EYE_NUMBER], mapped[ANGLE_EYE_NUMBER], mapped[ROT_EYE_NUMBER]);
                }else if(strcmp(token, "#RawEncoders") == 0){
					mat_string_length = sprintf(mat_string, "[%i, %i, %i, %i, %i]"
					, mapped[BASE_RAW_ENCODER_ANGLE_FXP]
					, mapped[PIVOT_RAW_ENCODER_ANGLE_FXP]
					, mapped[END_RAW_ENCODER_ANGLE_FXP]
					, mapped[ANGLE_RAW_ENCODER_ANGLE_FXP]
					, mapped[ROT_RAW_ENCODER_ANGLE_FXP]
					);
				}else if (strcmp(token, "#measured_angles") == 0) {
					mat_string_length = sprintf(mat_string
					, "[%d, %d, %d, %d, %d]"
					, mapped[BASE_MEASURED_ANGLE]
					, mapped[PIVOT_MEASURED_ANGLE]
					, mapped[END_MEASURED_ANGLE]
					, mapped[ANGLE_MEASURED_ANGLE]
					, mapped[ROT_MEASURED_ANGLE]
					  );
					//printf("#measured_angles: %s length: %d", mat_string, mat_string_length);
				}else if(strcmp(token, "#Steps") == 0){
					mat_string_length = sprintf(mat_string
					, "[%i, %i, %i, %i, %i]"
					, SIGNEX(mapped[BASE_STEPS],18)
					, SIGNEX(mapped[PIVOT_STEPS],18)
					, SIGNEX(mapped[END_STEPS],18)
					, SIGNEX(mapped[ANGLE_STEPS],18)
					, SIGNEX(mapped[ROT_STEPS],18)
					  );
					//printf("#Steps: %s length: %d", mat_string, mat_string_length);
				}else if(strcmp(token, "#StepAngles") == 0){
					mat_string_length = sprintf(mat_string
					, "[%i, %i, %i, %i, %i]"
					, getNormalizedInput(BASE_STEPS)
					, getNormalizedInput(PIVOT_STEPS)
					, getNormalizedInput(END_STEPS)
					, getNormalizedInput(ANGLE_STEPS)
					, getNormalizedInput(ROT_STEPS)
					  );
					//printf("#StepAngles: %s length: %d", mat_string, mat_string_length);
				}else if (strcmp(token, "xxx") == 0) {
				  // do something else
				}else{
					printf("Error: %s is not a valid string\n", token);
				}
				ptr_mat_string = mat_string;
			}
		
			if(mat_string_length > 0){ //if there is content to send
				//MAX_CONTENT_CHARS + 1 because strlcpy leaves room for EOS and DDE needs to see MAX_CONTENT_CHARS before EOS. 
				sendBuffReTyped[6] = strlcpy(sendBuff + sizeof(sendBuffReTyped[0])*7, ptr_mat_string, MAX_CONTENT_CHARS + 1);
				ptr_mat_string += MAX_CONTENT_CHARS;
				mat_string_length -= MAX_CONTENT_CHARS;
				//printf("mat_string_length: %d\n", mat_string_length);
				//printf("sendBuffReTyped[6]: %d\n", sendBuffReTyped[6]);
				//printf("mat_string: %s\n", mat_string);
				//printf("ptr_mat_string: %s\n", ptr_mat_string);
				
			}
			break;
		default:
			wfp = fopen(token, "r");
			if (wfp) {
				//printf("Opened as handle %d.\n ",fileno(wfp));
				i *= MAX_CONTENT_CHARS; //starting byte in the file
				//printf("read from byte %d\n",i);
				fseek(wfp, i, SEEK_SET);
				sendBuffReTyped[6] = fread ( sendBuff + sizeof(sendBuffReTyped[0])*7, 1, MAX_CONTENT_CHARS, wfp );
				//printf("Read %d bytes\n",sendBuffReTyped[6]);
				//printf("\n%s",sendBuff + sizeof(sendBuffReTyped[0])*6);
				fclose(wfp);
				wfp = 0;
			}
			else {
				printf("Error %d\n", errno);
				sendBuffReTyped[5] = errno; //there was an error
				sendBuffReTyped[6] = 0; //no bytes returned
				//strerror_s( sendBuff + sizeof(sendBuffReTyped[0])*7, MAX_CONTENT_CHARS, errno );
				addr = i; // first number might actually be the starting address
				last = atoi(token); //ending address maybe?
				if (last > addr && (last-addr) < 20) { //is it a valid range?
					sendBuffReTyped[5] = 0; //actually no error
					sendBuffReTyped[6] = (last-addr)*4; 
					//printf("\n %d %d \n",Addr,Length);
					for(i=7; addr < last; i++, addr++) { // i is the starting address
						sendBuffReTyped[i] = mapped[addr]; // cram it into the return data.
						printf("\n %2d: %x ", addr, mapped[addr]);
						}
					}


			}
		} //end of select(token[0])
		return TRUE;
	}
	else { //if('r'==oplet)
		// Response with status.
		//sendBuffReTyped[1]=token[0];
		//sendBuffReTyped[2]=DexError;
		
		for(i=0;i<59;i++)
		{
			sendBuffReTyped[i+6]=getNormalizedInput(StatusReportIndirection[i]);
		}
		return TRUE;
	} //if('r'==oplet)
	return FALSE;
}
bool ProcessServerReceiveDataDDE(char *recBuff)
{
	struct BotPossition MyBot;
	char CmdString[255];
	int i,FoundStart=0,j=0;
	bool GotDelim=FALSE;
	int *sendBuffReTyped;
	const char delimiters[] = " ,";
	char *token;
	FoundStart=0;
	for(i=0;i<sizeof(CmdString);i++)
	{
		if(FoundStart >= 4)
		{
			CmdString[j]=recBuff[i];
			j++;
		}
		if(recBuff[i]==' ')
		{
			FoundStart++;
		}
		if(recBuff[i]==0x3b)
		{
			recBuff[i]=0;
			GotDelim=TRUE;
			break;
		}
	}
	if(GotDelim==FALSE)
	{
		DexError=2;
		printf("\n No delim:%s\n",recBuff);
		return FALSE;
	}
	CmdString[j-1]=0;
	if(CmdString[0] != 'g')
	{
		//printf("\n%s \n",recBuff);
		//printf("\n%s \n",CmdString);
		
	}
	////printf("\n%s \n",recBuff);
	#ifdef DEBUG_API
	if(CmdString[0]!='g')
	printf("\n%d : %s ",cmditer++,CmdString);
	fflush(stdout);
	#endif
	DexError=ParseInput(CmdString); 
	#ifdef DEBUG_API
	if(CmdString[0]!='g' && DexError > 0)
	printf(" Error %d",DexError);
	fflush(stdout);
	#endif
	return TRUE;
}

void *StartClientSocket()
{
    int sockfd = 0, n = 0,j;
    char recvBuff[64];
    char sendBuff[64];
    struct sockaddr_in serv_addr; 
	int Cycle = 100;
	struct BotPossition MyBot;
	float DiffDelta;
	float ForceScale=0;

    memset(recvBuff, '0',sizeof(recvBuff));
    if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        //printf("\n Error : Could not create socket \n");
        return &"Socket create failed";
    } 

    memset(&serv_addr, '0', sizeof(serv_addr)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(40000); 
	RemoteRobotAdd[strcspn(RemoteRobotAdd,"\n")]= 0;

    if(inet_pton(AF_INET, RemoteRobotAdd/*"192.168.1.145"*/, &serv_addr.sin_addr)<=0)
    {
        return &"inet_pton failed";
    } 
    if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       //printf("\n Error : Connect Failed \n");
       return &"connect failed";
    } 
	{
		while( (n = recv(sockfd, recvBuff, sizeof(recvBuff),0 ) ) > 0)
		{

			if(n < 0)
			{
			} 
#ifdef FOLLOW_POSSITION
			memcpy(&MyBot,recvBuff,sizeof(MyBot));
			DiffDelta=1.5;
			MyBot.base=SlaveBotPos.base + ((GetAxisCurrent(0) - SetHome.base)/MSMoveRatio.base);
			MyBot.end=SlaveBotPos.end + ((GetAxisCurrent(1) - SetHome.end)/MSMoveRatio.end);
			MyBot.pivot=SlaveBotPos.pivot + ((GetAxisCurrent(2) - SetHome.pivot)/MSMoveRatio.pivot);
			MyBot.angle=SlaveBotPos.angle + (((GetAxisCurrent(3) - SetHome.angle)/MSMoveRatio.angle)*DiffDelta);
			MyBot.rotate=SlaveBotPos.rotate + (((GetAxisCurrent(4) - SetHome.rotate)/MSMoveRatio.rotate)*DiffDelta);
			memcpy(sendBuff,&MyBot,sizeof(MyBot));
			Cycle++;

#else
			ProcessServerReceiveData(recvBuff);
			(void)ProcessServerSendData(sendBuff);
			
#endif			
			write(sockfd,sendBuff,sizeof(sendBuff));
		}
	}
    return NULL;
}

int GetAxisCurrent(int Axis)
{
	return getNormalizedInput(BASE_POSITION_AT+Axis)+getNormalizedInput(BASE_POSITION_FORCE_DELTA+Axis);
}
/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  new = old; /* make new settings same as old settings */
  new.c_lflag &= ~ICANON; /* disable buffered i/o */
  new.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void) 
{
  return getch_(1);
}

int FindIndex(int Axis,int Start,int Length,int Delay)
{
	int i,j,k,ADVal,AvgCOS=0,AvgSIN=0;
	switch(Axis)
	{
	    case 0  :
		MoveRobot(Start,0,0,0,0,BLOCKING_MOVE);
		break; 
	    case 1  :
		MoveRobot(0,Start,0,0,0,BLOCKING_MOVE);
		break; 
	    case 2  :
		MoveRobot(0,0,Start,0,0,BLOCKING_MOVE);
		break; 
	    case 3  :
		MoveRobot(0,0,0,Start,0,BLOCKING_MOVE);
		break; 
	    case 4  :
		MoveRobot(0,0,0,0,Start,BLOCKING_MOVE);
		break; 
	}
	for(k=0;k<abs(Length);k++) 
	{	
		if(Length>0)
			mapped[Axis]=shadow_map[Axis]=Start+k;
		else	
			mapped[Axis]=shadow_map[Axis]=Start-k;
		for(j=0;j<Delay;j++)
		{	
			ADVal=mapped[ADLookUp[Axis]];
			AvgSIN=AvgSIN+ADVal;
			AvgCOS=AvgCOS+mapped[ADLookUp[Axis]+1];
		}	
		AvgSIN=AvgSIN/Delay;
		AvgCOS=AvgCOS/Delay;
		
		if((AvgCOS>XLowBound[Axis]) && (AvgCOS<XHighBound[Axis]) && (AvgSIN>YLowBound[Axis]) && (AvgSIN<YHighBound[Axis]))
		{
			// we found the index 
			return 0;
		}
	}
	return 1; // did not find index
}
void SetNewBotRef()
{
	SlaveBotPos.base=SlaveBotPos.base + ((GetAxisCurrent(0) - SetHome.base)/MSMoveRatio.base);
	SlaveBotPos.end=SlaveBotPos.end + ((GetAxisCurrent(1) - SetHome.end)/MSMoveRatio.end);
	SlaveBotPos.pivot=SlaveBotPos.pivot + ((GetAxisCurrent(2) - SetHome.pivot)/MSMoveRatio.pivot);
	SlaveBotPos.angle=SlaveBotPos.angle + ((GetAxisCurrent(3) - SetHome.angle)/MSMoveRatio.angle);
	SlaveBotPos.rotate=SlaveBotPos.rotate + ((GetAxisCurrent(4) - SetHome.rotate)/MSMoveRatio.rotate);
	SetHome.base=GetAxisCurrent(0);
	SetHome.end=GetAxisCurrent(1);
	SetHome.pivot=GetAxisCurrent(2);
	SetHome.angle=GetAxisCurrent(3);
	SetHome.rotate=GetAxisCurrent(4);
	
}
void CapturePoints(void *arg)
{

}
int InitCapturePoints(char *FileName)
{
    int i = 0;
    int err;
	CptA.FileName=FileName;
	CptA.fp=fopen(FileName, "w");
	if(CptA.fp!=0)
	{
		CapturePoints((void*)&CptA);
	}

	return 0;
}


int InitCaptureMovement(char *FileName)
{
    int i = 0,Length=0;
    int err;
	char c;
	//initTermios(0);
//	CptMove.FileName=FileName;
//	CptMove.fp=fopen(FileName, "w");
//	if(CptMove.fp!=0)
	{
		//CaptureMovement((void*)&CptMove);
		mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=CMD_RESET_RECORD;
		mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=CMD_RECORD; // start recording
		c=' ';
		while(c!='k') // kill
		{
			c=getchar();
			
			
		}
		mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=0; // stop recording
		
		Length=mapped[RECORD_BLOCK_SIZE];
		ReadDMA(0x3f000000,Length,FileName);

		
	}
	//resetTermios();
	return 0;
}


int fixedPointCV(float Val,int whole,int fract)
{
	return 0;
}

/* Never used
void CvrtBoundary_CenterMag_to_HILOW(int Center, int Magnitude, int *ResultHi, int *ResultLow)
{
  *ResultHi = Center + Magnitude;
  *ResultLow = Center - Magnitude;  
}
*/


//unsigned int CvrtBoundary_HILOW_to_CenterMag(int High, int Low){}

// Boundary is low first, then high, for joints 1 thru 5
int Boundary[10] = {-670010, 670010, -350010, 350010, -570010, 570010, -390010, 390010, -684010, 684010};
int forceBias[5];
int Friction[5];
int FineAdjust[5];

void readAdcCenters() {
	int KeyHoleData[10];
	int HexValue;
    FILE *CentersFile;
    CentersFile = fopen("AdcCenters.txt", "rs");
    //read file into array
	if(CentersFile!=NULL) {
		fscanf(CentersFile, "%x", &HexValue);
		//printf("%x \n",HexValue);
		KeyHoleData[0] = HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		KeyHoleData[1] = HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		KeyHoleData[2] = HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		KeyHoleData[3] = HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		KeyHoleData[4] = HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		KeyHoleData[5] = HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		KeyHoleData[6] = HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		KeyHoleData[7] = HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		KeyHoleData[8] = HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		KeyHoleData[9] = HexValue;
		fclose(CentersFile);
		KeyholeSend(KeyHoleData, ADC_CENTERS_KEYHOLE_CMD, ADC_CENTERS_KEYHOLE_SIZE, ADC_CENTERS_KEYHOLE );
		printf("Centers Keyhle set\n");
	} else {
		printf("Cant read AdcCenters.txt file! Centers Keyhle NOT set\n");
	}
}

void setDefaults(int State)
{

	printf("Start of setDefaults()\n");
	int i,ForceFelt,j,HiBoundry,LowBoundry,BoundryACC;
	int KeyHoleData[10];
	char c;
    FILE *AxisFile, *RemoteRobotAddress,*DiffFile;
	int HexValue;

	int IntFloat;
	float *fConvert=(float *)(&IntFloat);
	mapped[COMMAND_REG]=shadow_map[COMMAND_REG]=64;  //shut off the servo system
	mapped[COMMAND_REG]=shadow_map[COMMAND_REG]=0;  //shut off the servo system
	CmdVal=0;
	bot_state.start = true;

	for (i=0; i<NUM_JOINTS; i++) {
		//bot_state.enc_error_limit[i] = MONITOR_MAX_ERROR;
		bot_state.enc_velocity_limit[i] = MONITOR_MAX_VELOCITY;
	}
	//increasing error tolerance as the joints become less stiff 
	bot_state.enc_error_limit[1-1] = 5.0 * MONITOR_MAX_ERROR * DEG_ARCSEC;
	bot_state.enc_error_limit[2-1] = 5.0 * MONITOR_MAX_ERROR * DEG_ARCSEC;
	bot_state.enc_error_limit[3-1] = 6.0 * MONITOR_MAX_ERROR * DEG_ARCSEC;
	bot_state.enc_error_limit[4-1] = 7.4 * MONITOR_MAX_ERROR * DEG_ARCSEC;
	bot_state.enc_error_limit[5-1] = 7.4 * MONITOR_MAX_ERROR * DEG_ARCSEC;
	bot_state.enc_velocity_count = 0;
	bot_state.enc_velocity_count_limit = 10; //we can jerk 100 times before we error.
	
/* Load AxisCal.txt file. This is calculated from from Gear Ratio and Microstepping as follows (in DDE) :
//Input:
var diff_pulley_small_num_teeth = 16
var diff_pulley_large_num_teeth = 90
var micro_step = 16
var motor_steps = 400
var gear_ratios = [
	52,
    52,
    52,
    diff_pulley_large_num_teeth / diff_pulley_small_num_teeth,
    diff_pulley_large_num_teeth / diff_pulley_small_num_teeth
]

//Output:
var AxisCal_string = ""
for(let i = 0; i < 5; i++){
	AxisCal_string += -(gear_ratios[i]*micro_step*motor_steps) / (3600*360) + "\n"
}
AxisCal_string += -Math.round(gear_ratios[3] / gear_ratios[1] * Math.pow(2, 24))
*/

	AxisFile = fopen("AxisCal.txt", "rs");
	if(AxisFile!=NULL) {
		fscanf(AxisFile, "%f", &JointsCal[0]);
		fscanf(AxisFile, "%f", &JointsCal[1]);
		fscanf(AxisFile, "%f", &JointsCal[2]);
		fscanf(AxisFile, "%f", &JointsCal[3]);
		fscanf(AxisFile, "%f", &JointsCal[4]);
		fscanf(AxisFile, "%i", &HexValue);
        //printf("Reading AxisCal.txt. HexValue = %d\n", HexValue);
		mapped[ANGLE_END_RATIO]=HexValue;//((LG_RADIUS/SM_RADIUS * MOTOR_STEPS * MICRO_STEP)/(MOTOR_STEPS*GEAR_RATIO*MICRO_STEP))*2^24
		fclose(AxisFile);
	}
	
	RemoteRobotAddress = fopen("RemoteRobotAddress.txt", "rs");
	if(RemoteRobotAddress!=NULL)
	{
		fgets(RemoteRobotAdd, sizeof(RemoteRobotAdd), RemoteRobotAddress);
		//fscanf(RemoteRobotAddress,"%[^\n]",RemoteRobotAdd);
		fclose(RemoteRobotAddress);
	}

	forceBias[0]=0;
	forceBias[1]=0;
	forceBias[2]=0;
	forceBias[3]=0;
	forceBias[4]=0;
	
	KeyholeSend(forceBias, FORCE_BIAS_KEYHOLE_CMD, FORCE_BIAS_KEYHOLE_SIZE, FORCE_BIAS_KEYHOLE );
	printf("ForceBias Keyhle set\n");

	Friction[0]=0;
	Friction[1]=0;
	Friction[2]=0;
	Friction[3]=0;
	Friction[4]=0;
	
	KeyholeSend(Friction, FRICTION_KEYHOLE_CMD, FRICTION_KEYHOLE_SIZE, FRICTION_KEYHOLE );
	printf("Friction Keyhle set\n");

	FineAdjust[0]=0;
	FineAdjust[1]=0;
	FineAdjust[2]=0;
	FineAdjust[3]=0;
	FineAdjust[4]=0;
	
	KeyholeSend(FineAdjust, FINE_ADJUST_KEYHOLE_CMD, FINE_ADJUST_KEYHOLE_SIZE, FINE_ADJUST_KEYHOLE );

	printf("FineAdjust Keyhle set\n");


	mapped[ACCELERATION_MAXSPEED]=shadow_map[ACCELERATION_MAXSPEED]=ACCELERATION_MAXSPEED_DEF;
	maxSpeed=(ACCELERATION_MAXSPEED_DEF) & 0b00000000000011111111111111111111;
	coupledAcceleration=((ACCELERATION_MAXSPEED_DEF) & 0b11111111111100000000000000000000) >> 20;

	if(State==1)
	{


		mapped[BASE_FORCE_DECAY]=shadow_map[BASE_FORCE_DECAY]=000000;
		mapped[END_FORCE_DECAY]=shadow_map[END_FORCE_DECAY]=000000;
		mapped[PIVOT_FORCE_DECAY]=shadow_map[PIVOT_FORCE_DECAY]=000000;
		mapped[ANGLE_FORCE_DECAY]=shadow_map[ANGLE_FORCE_DECAY]=000000;
		mapped[ROTATE_FORCE_DECAY]=shadow_map[ROTATE_FORCE_DECAY]=000000;

		mapped[ACCELERATION_MAXSPEED]=shadow_map[ACCELERATION_MAXSPEED]=ACCELERATION_MAXSPEED_DEF;
		maxSpeed=(ACCELERATION_MAXSPEED_DEF) & 0b00000000000011111111111111111111;
		coupledAcceleration=((ACCELERATION_MAXSPEED_DEF) & 0b11111111111100000000000000000000) >> 20;
	

		mapped[REC_PLAY_TIMEBASE]=shadow_map[REC_PLAY_TIMEBASE]=5;
		
		readAdcCenters();    

		mapped[DIFF_FORCE_BETA ]=shadow_map[DIFF_FORCE_BETA ]=0x0102;
		mapped[BETA_XYZ ]=shadow_map[BETA_XYZ ]=0x0002;


	// set up PID defaults
	
		mapped[PID_P]=shadow_map[PID_P]=DEFAULT_PID_SETTING_XYZ;
		mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]=0;
		mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]=1;
		mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]=2;
		mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]=3;
		mapped[PID_P]=shadow_map[PID_P]=DEFAULT_PID_SETTING_PY;
		mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]=4;
		
		
		
		

		mapped[SPEED_FACTORA]=shadow_map[SPEED_FACTORA]=0;
		mapped[MAX_ERROR]=shadow_map[MAX_ERROR]=(2000 ^ 6000);
		

		mapped[MAXSPEED_XYZ]=shadow_map[MAXSPEED_XYZ]=140000; 
		
		mapped[DIFF_FORCE_MAX_SPEED]=shadow_map[DIFF_FORCE_MAX_SPEED]=90000;

		//mapped[ANGLE_END_RATIO]=645278;//((LG_RADIUS/SM_RADIUS * MOTOR_STEPS * MICRO_STEP)/(MOTOR_STEPS*GEAR_RATIO*MICRO_STEP))*2^24
		

		

	
		if(RestoreCalTables("/srv/samba/share/HiMem.dta")==0)
		{
			/*
			#ifndef NO_BOOT_DANCE
			MoveRobot(20000,20000,20000,20000,20000,BLOCKING_MOVE);
			MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
			#endif
			*/
			#ifndef NO_BOOT_DANCE
			strlcpy(iString, "S RunFile BootDance_setDefaults.make_ins ;\0", ISTRING_LEN);
			printf("Starting %s returned %d\n",iString, ParseInput(iString));
			#endif
		}
	printf("CalTables set\n");

		
	}
	printf("End of setDefaults\n");
}

void wait_fifo_flush(void)
{
	while((mapped[CMD_FIFO_STATE] & 0x2) != 2)
	{
		
	}
}
int HashInputCMD(char *s)
{

	if(s[0]=='W') 
            return WRITE_TO_ROBOT;
	if(s[0]=='r')
		return READ_CMD;
	if(s[0]=='B')
		return SET_ALL_BOUNDRY;
	if(s[0]=='w')
	{
		wait_fifo_flush();
		return WRITE_CMD;
	}
	if(s[0]=='x')
		return EXIT_CMD;
	if(s[0]=='s')
		return SLOWMOVE_CMD;
	if(s[0]=='a')
		return MOVEALL_CMD;
	
	//////////////////////////////////////////////////////////////////////////
	/* Start Wigglesworth Code*/
	//////////////////////////////////////////////////////////////////////////
	//printf("HashInputCMD: %c", s[0]);
	if (s[0] == 'M')
		return MOVETO_CMD;
	if (s[0] == 'T')
		return MOVETOSTRAIGHT_CMD;
	
	//////////////////////////////////////////////////////////////////////////
	/* End Wigglesworth Code*/
	//////////////////////////////////////////////////////////////////////////
	
	
	if(s[0]=='d')
		return DMAREAD_CMD;
	if(s[0]=='t')
		return DMAWRITE_CMD;
	if(s[0]=='c')
		return CAPTURE_AD_CMD;
	if(s[0]=='f')
		return FIND_HOME_CMD;
	if(s[0]=='n')
		return FIND_INDEX_CMD;
	if(s[0]=='p')
		return FIND_HOME_REP_CMD;
	if(s[0]=='l')
		return LOAD_TABLES;
	if(s[0]=='i')
		return CAPTURE_POINTS_CMD;
	if(s[0]=='z')
		return SLEEP_CMD;
	if(s[0]=='m')
		return RECORD_MOVEMENT;
	if(s[0]=='o')
		return REPLAY_MOVEMENT;
	if(s[0]=='R')
		return MOVEALL_RELATIVE;
	if(s[0]=='S')
		return SET_PARAM;
	if(s[0]=='g')
		return SEND_HEARTBEAT;
	/*if(s[0]=='F')
		return SET_FORCE_MOVE_POINT;*/
	if(s[0]=='G')
		return HEART_BEAT;
	if(s[0]=='h')
		return HEART_BEAT;
	if(s[0]=='P')
		return PID_FINEMOVE;
	if(s[0]=='F')
	{
		wait_fifo_flush();
		return HEART_BEAT;
	}
	return 0;
}



int isNear(int a,int b,int range)
{
	if(abs(a-b)<=range)
		return TRUE;
	else
		return FALSE;
}
int getNormalizedInput(int param) 
{
	int val;
	float corrF=1;
	if(param == SLOPE_BASE_POSITION){return ServoData[1].PresentPossition;}
	if(param == SLOPE_PIVOT_POSITION){return ServoData[1].PresentLoad;}
	if(param == SLOPE_END_POSITION){return ServoData[0].PresentPossition;}
	if(param == SLOPE_ANGLE_POSITION){return ServoData[0].PresentLoad;}
	if(param == SLOPE_ROT_POSITION){return (ServoData[0].error & 0x00ff) + ((ServoData[1].error & 0x0ff)<<8);}
	
	val = mapped[param];
	if(param <= ROT_POSITION_FORCE_DELTA)
	{
		corrF = JointsCal[(param-INPUT_OFFSET) % 5];
	}
	if((val & 0x40000)!=0)
	{
		val = (val | 0xfff80000);
	}
	
	if(param == BASE_MEASURED_ANGLE){corrF = JointsCal[0];}
	if(param == PIVOT_MEASURED_ANGLE){corrF = JointsCal[1];}
	if(param == END_MEASURED_ANGLE){corrF = JointsCal[2];}
	if(param == ANGLE_MEASURED_ANGLE){corrF = JointsCal[3] ;}
	if(param == ROT_MEASURED_ANGLE){corrF = JointsCal[4] ;}
	
	if(param == BASE_STEPS){corrF = JointsCal[0];}
	if(param == PIVOT_STEPS){corrF = JointsCal[1];}
	if(param == END_STEPS){corrF = JointsCal[2];}
	if(param == ANGLE_STEPS){corrF = JointsCal[3] ;}
	if(param == ROT_STEPS){corrF = JointsCal[4] ;}

	return (int)((float)val / corrF);
}
int getNormalInput(int param) 
{
	int val;
	float corrF=1;
	val = mapped[param];
	if((val & 0x40000)!=0)
	{
		val = (val | 0xfff80000);
	}
	return (int)((float)val / corrF);
}



int WaitMoveGoal(int a1,int a2,int a3,int a4,int a5,int timeout)
{
	int b1,b2,b3,b4,b5;
    long            ms; // Milliseconds
    time_t          oldTime,newTime;  // Seconds
    struct timespec spec;

	b1=getNormalizedInput(BASE_POSITION_AT);
	b3=getNormalizedInput(END_POSITION_AT);
	b2=getNormalizedInput(PIVOT_POSITION_AT);
	b4=getNormalizedInput(ANGLE_POSITION_AT);
	b5=getNormalizedInput(ROT_POSITION_AT);

    clock_gettime(CLOCK_REALTIME, &spec);
	oldTime  = spec.tv_sec;
	while(!isNear(a1,b1,500) || !isNear(a2,b2,500) || !isNear(a3,b3,500) || !isNear(a4,b4,500) || !isNear(a5,b5,500))
	{
		b1=getNormalizedInput(BASE_POSITION_AT);
		b3=getNormalizedInput(END_POSITION_AT);
		b2=getNormalizedInput(PIVOT_POSITION_AT);
		b4=getNormalizedInput(ANGLE_POSITION_AT);
		b5=getNormalizedInput(ROT_POSITION_AT);
		clock_gettime(CLOCK_REALTIME, &spec);
		newTime  = spec.tv_sec;
		if((newTime-oldTime)>20)
			return 0;
		
	}
	return 0;
		
}

void moverobotPID(int a1,int a2,int a3,int a4,int a5)
{
	DexError = CheckBoundry(&a1,&a2,&a3,&a4,&a5);
	
	a1=(int)((double)a1 * JointsCal[0]);
	a2=(int)((double)a2 * JointsCal[1]);
	a3=(int)((double)a3 * JointsCal[2]);
	a4=(int)((double)a4 * JointsCal[3]);
	a5=(int)((double)a5 * JointsCal[4]);

	//printf("PID move %d %d %d %d %d %d \n",a1,a3,a2,a4,a5);

	FineAdjust[0]=a1;
	FineAdjust[1]=a3;
	FineAdjust[2]=a2;
	FineAdjust[3]=a4;
	FineAdjust[4]=a5;
	
	KeyholeSend(FineAdjust, FINE_ADJUST_KEYHOLE_CMD, FINE_ADJUST_KEYHOLE_SIZE, FINE_ADJUST_KEYHOLE );


}

int MoveRobot(int a1,int a2,int a3,int a4,int a5, int mode)
{
	int KeyHoleArray[5];

	DexError = CheckBoundry(&a1,&a2,&a3,&a4,&a5);


/*	int b1,b2,b3,b4,b5;
	b1=getNormalizedInput(PLAYBACK_BASE_POSITION);
	b2=getNormalizedInput(PLAYBACK_END_POSITION);
	b3=getNormalizedInput(PLAYBACK_PIVOT_POSITION);
	b4=getNormalizedInput(PLAYBACK_ANGLE_POSITION);
	b5=getNormalizedInput(PLAYBACK_ROT_POSITION);
	a1=a1-b1; // subtract out the playback position 
	a2=a2-b2;
	a3=a3-b3;
	a4=a4-b4;
	a5=a5-b5;
	//printf("\nPlayback position %d %d %d %d %d",b1,b2,b3,b4,b5);
*/
	//printf("Start wait Goal\n");
//	332800
//	166400
//	0.25679012345679012345679012345679
//	0.00987654320987654320987654320988

	//if(mode==BLOCKING_MOVE)
		//WaitMoveGoal(LastGoal[0],LastGoal[1],LastGoal[2],LastGoal[3],LastGoal[4],DEFAULT_MOVE_TIMEOUT);
	
	
	
	//Select joint with largest angular displacement (BAD CODE PLEASE RE-WRITE)
	int j_step = 0;
    int j_deg = 0;
    double test_angle;
    double cur_max_step = 0.0;
    double cur_max_deg = 0.0;
    double test_angle_step [] = {0, 0, 0, 0, 0};
    double test_angle_deg [] = {0, 0, 0, 0, 0};
    test_angle_step[0] = abs((double)(a1 - LastGoal[0]) * JointsCal[0]);
    if(test_angle_step[0] > cur_max_step){
        cur_max_step = test_angle_step[0];
        j_step = 0;
    };
    test_angle_step[1] = abs((double)(a2 - LastGoal[1]) * JointsCal[1]);
    if(test_angle_step[1] > cur_max_step){
        cur_max_step = test_angle_step[1];
        j_step = 1;
    };
    test_angle_step[2] = abs((double)(a3 - LastGoal[2]) * JointsCal[2]);
    if(test_angle_step[2] > cur_max_step){
        cur_max_step = test_angle_step[2];
        j_step = 2;
    };
    test_angle_step[3] = abs((double)(a4 - LastGoal[3]) * JointsCal[3]);
    if(test_angle_step[3] > cur_max_step){
        cur_max_step = test_angle_step[3];
        j_step = 3;
    };
    test_angle_step[4] = abs((double)(a5 - LastGoal[4]) * JointsCal[4]);
    if(test_angle_step[4] > cur_max_step){
        cur_max_step = test_angle_step[4];
        j_step = 4;
    };

	if(0 == cur_max_step){
		return 0;
	}

    //printf("major delta step: J%d delta: %f (abs steps)  delta: %f (abs deg)\n", j_step+1, cur_max_step, -cur_max_step / JointsCal[j_step] / 3600);



    test_angle_deg[0] = abs((double)(a1 - LastGoal[0]));
    if(test_angle_deg[0] > cur_max_deg){
        cur_max_deg = test_angle_deg[0];
        j_deg = 0;
    };
    test_angle_deg[1] = abs((double)(a2 - LastGoal[1]));
    if(test_angle_deg[1] > cur_max_deg){
        cur_max_deg = test_angle_deg[1];
        j_deg = 1;
    };
    test_angle_deg[2] = abs((double)(a3 - LastGoal[2]));
    if(test_angle_deg[2] > cur_max_deg){
        cur_max_deg = test_angle_deg[2];
        j_deg = 2;
    };
    test_angle_deg[3] = abs((double)(a4 - LastGoal[3]));
    if(test_angle_deg[3] > cur_max_deg){
        cur_max_deg = test_angle_deg[3];
        j_deg = 3;
    };
    test_angle_deg[4] = abs((double)(a5 - LastGoal[4]));
    if(test_angle_deg[4] > cur_max_deg){
        cur_max_deg = test_angle_deg[4];
        j_deg = 4;
    };

    //printf("major delta deg:  J%d delta: %f (abs steps)  delta: %f (abs deg)\n", j_deg+1, -cur_max_deg * JointsCal[j_deg], cur_max_deg / 3600);

	LastGoal[0]=a1;
	LastGoal[1]=a2;
	LastGoal[2]=a3;
	LastGoal[3]=a4;
	LastGoal[4]=a5;

	//printf("LastGoal set: [%d, %d, %d, %d, %d]\n", LastGoal[0], LastGoal[1], LastGoal[2], LastGoal[3], LastGoal[4]);

	a1=(int)((double)a1 * JointsCal[0]);
	a2=(int)((double)a2 * JointsCal[1]);
	a3=(int)((double)a3 * JointsCal[2]);
	a4=(int)((double)a4 * JointsCal[3]);
	a5=(int)((double)a5 * JointsCal[4]);
	//printf("angles result %d %d %d %d %d\n",a1,a2,a3,a4,a5);

	KeyHoleArray[0] = a1;
	KeyHoleArray[1] = a3;
	KeyHoleArray[2] = a2;
	KeyHoleArray[3] = a4;
	KeyHoleArray[4] = a5;


	while((mapped[CMD_FIFO_STATE] & 0x01) != 0); //This was commented out for some reason in commit: https://github.com/HaddingtonDynamics/Dexter/commit/1ca9251b47468d9841713ec89b62e91050125188


	
	//printf("Largest displacement: J%d, %f\n", j, cur_max);
    //Actually adding the speed change to the queue:
    int new_StartSpeed;
    int new_MaxSpeed;

    /*
    new_StartSpeed = (int)(abs(startSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle * (JointsCal[j_deg] / JointsCal[j_step])));
    new_MaxSpeed = (int)(abs(maxSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle * (JointsCal[j_deg] / JointsCal[j_step])));
    */
	
    /*
    new_StartSpeed = (int)(abs(startSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle * (JointsCal[j_step] / JointsCal[j_deg])));
    new_MaxSpeed = (int)(abs(maxSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle * (JointsCal[j_step] / JointsCal[j_deg])));
    */

    new_StartSpeed = (int)(abs(startSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle) * test_angle_deg[j_step] / test_angle_deg[j_deg]);
    new_MaxSpeed = (int)(abs(maxSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle) * test_angle_deg[j_step] / test_angle_deg[j_deg]);

    /*
    if(j_step < 3 && j_deg > 2){
        
        //new_StartSpeed = (int)(abs(startSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle * (JointsCal[j_deg] / JointsCal[j_step])));
        //new_MaxSpeed = (int)(abs(maxSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle * (JointsCal[j_deg] / JointsCal[j_step])));
        
        
        //new_StartSpeed = (int)(abs(startSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle) * test_angle_step[j_deg] / test_angle_step[j_step]);
        //new_MaxSpeed = (int)(abs(maxSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle) * test_angle_step[j_deg] / test_angle_step[j_step]);
        
        new_StartSpeed = (int)(abs(startSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle) * test_angle_deg[j_step] / test_angle_deg[j_deg]);
        new_MaxSpeed = (int)(abs(maxSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle) * test_angle_deg[j_step] / test_angle_deg[j_deg]);
        printf("j_step < 3 && j_deg > 2 is true\n");
    }else{
        new_StartSpeed = (int)(abs(startSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle));
        new_MaxSpeed = (int)(abs(maxSpeed_arcsec_per_sec * JointsCal[j_step] * bit_sec_per_microstep_clockcycle));
        printf("j_step < 3 && j_deg > 2 is false\n");
    }
    */
	

    if(1 > new_StartSpeed){new_StartSpeed = 1;DexError = 1;} //Anything less than 1 will cause infinite loop. Consider error code for this.
    mapped[START_SPEED]=shadow_map[START_SPEED]= 500 ^ new_StartSpeed; // Start speed is defaulted in the FPGA to 500   
    if(2 > new_MaxSpeed){new_MaxSpeed = 2;DexError = 1;} //Must be higher than minimum StartSpeed. Consider error code for this.
	maxSpeed=(new_MaxSpeed) & 0b00000000000011111111111111111111;
	mapped[ACCELERATION_MAXSPEED]=shadow_map[ACCELERATION_MAXSPEED]= maxSpeed + (coupledAcceleration << 20);
    //printf("new_StartSpeed: %d\n", new_StartSpeed);
    //printf("new_MaxSpeed: %d\n", new_MaxSpeed);

	//printf("MaxSpeed: %d(bit/clockcycle)    MaxSpeed: %d(arcsec/s)    J%d: %f (deg)\n", new_MaxSpeed, maxSpeed_arcsec_per_sec, j, cur_max/3600.0);

	mapped[COMMAND_REG]=shadow_map[COMMAND_REG]=CMD_MOVEEN | CmdVal;
	KeyholeSend(KeyHoleArray, CMD_POSITION_KEYHOLE_CMD, CMD_POSITION_KEYHOLE_SIZE, CMD_POSITION_KEYHOLE );
	mapped[COMMAND_REG]=shadow_map[COMMAND_REG]=CMD_MOVEEN | CMD_MOVEGO | CmdVal;
	mapped[COMMAND_REG]=shadow_map[COMMAND_REG]=CmdVal;
	/*if(mode==BLOCKING_MOVE)
		WaitMoveGoal(LastGoal[0],LastGoal[1],LastGoal[2],LastGoal[3],LastGoal[4],DEFAULT_MOVE_TIMEOUT);*/
	return 0;
}

void KeyholeSend(int *DataArray, int controlOffset, int size, int entryOffset )
{
	int i=0;
	int ControlMask = (1 << controlOffset) - 1;
	mapped[entryOffset]=shadow_map[entryOffset]= 1 << (controlOffset + 1); // reset keyhole
	//printf(" keyhole reset %x \n", 1 << (controlOffset + 1));
	mapped[entryOffset]=shadow_map[entryOffset]= 0;
	for(i=0;i < size; i++)
	{
		mapped[entryOffset]=shadow_map[entryOffset]= DataArray[i] & ControlMask;
		//printf(" keyhole Write Data  %d \n", DataArray[i] & ControlMask);

		mapped[entryOffset]=shadow_map[entryOffset]= (DataArray[i] & ControlMask) + (1 << controlOffset); // toggle the data write
//		printf(" keyhole Write Data toggle  %d \n", DataArray[i] + (1 << controlOffset));

		mapped[entryOffset]=shadow_map[entryOffset]= DataArray[i] & ControlMask;
	}
}

int MoveRobotStraight(struct XYZ xyz_2)
{

	//File IO

	
	int cart_speed = CartesianSpeed;
	/*
	double cart_accel = CartesianAcceleration;
	double cart_step_size = CartesianStepSize;
	double rot_step_size = CartesianPivotStepSize;
	*/


	/*
	wfp = fopen("/srv/samba/share/Cartesian_Settings/Speed.txt", "r");
	if (wfp) {
		fscanf(wfp, "%lf", &cart_speed);
		fclose(wfp);
		wfp = 0;
	}else {
		printf("Failed to load /Cartesian_Settings/Speed.txt Error # %d\n", errno);
		cart_speed = 300000.0;
	}
	wfp = fopen("/srv/samba/share/Cartesian_Settings/Acceleration.txt", "r");
	if (wfp) {
		fscanf(wfp, "%lf", &cart_accel);
		fclose(wfp);
		wfp = 0;
	}else {
		printf("Failed to load /Cartesian_Settings/Acceleration.txt Error # %d\n", errno);
		cart_accel = 1000000.0;
	}
	wfp = fopen("/srv/samba/share/Cartesian_Settings/Step_Size.txt", "r");
	if (wfp) {
		fscanf(wfp, "%lf", &cart_step_size);
		fclose(wfp);
		wfp = 0;
	}else {
		printf("Failed to load /Cartesian_Settings/Step_Size.txt Error # %d\n", errno);
		cart_step_size = 10.0;
	}
	wfp = fopen("/srv/samba/share/Cartesian_Settings/Rotational_Step_Size.txt", "r");
	if (wfp) {
		fscanf(wfp, "%lf", &rot_step_size);
		fclose(wfp);
		wfp = 0;
	}else {
		printf("Failed to load /Cartesian_Settings/Rotational_Step_Size.txt Error # %d\n", errno);
		cart_step_size = 50.0;
	}
	*/


	//Reading Last Commaned Joint Angles
	struct J_angles LastGoal_J_angles = new_J_angles(
		(double)LastGoal[0],
		(double)LastGoal[1],
		(double)LastGoal[2],
		(double)LastGoal[3],
		(double)LastGoal[4]
	);
	
	//Converting Last Commaned Joint Angles to XYZ. Special Case Home.
	struct XYZ xyz_1 = new_XYZ(xyz_2.position, xyz_2.direction, xyz_2.config);
	if(LastGoal_J_angles.J1 == 0.0 && LastGoal_J_angles.J2 == 0.0 && LastGoal_J_angles.J3 == 0.0 && LastGoal_J_angles.J4 == 0.0 && LastGoal_J_angles.J5 == 0.0){
		xyz_1.position = new_vector(0, L[4], L[0]+L[1]+L[2]+L[3]);
		xyz_1.direction = new_vector(0, 1, 0);
		xyz_1.config = new_config(xyz_2.config.right_arm, xyz_2.config.elbow_up,  xyz_2.config.wrist_out);
		printf("Correcting for singularity in MoveRobotStraight\n");
		/*
		struct Vector home_position = new_vector(0, L[4], L[0]+L[1]+L[2]+L[3]);
		struct Vector home_dir = new_vector(0, 1, 0);
		struct Config home_config = new_config(xyz_2.config.right_arm, xyz_2.config.elbow_up,  xyz_2.config.wrist_out);
		 = new_XYZ(home_position, home_dir, home_config);
		 */

	}else{
		xyz_1 = J_angles_to_XYZ(LastGoal_J_angles);
	}

	
	printf("\nLastGoal_J_angles: ");
	print_J_angles(LastGoal_J_angles);
	
	printf("\nxyz_1: ");
	print_XYZ(xyz_1);
	printf("\nxyz_2: ");
	print_XYZ(xyz_2);

	//Prevent Config changes during straight line move
	if(xyz_1.config.right_arm != xyz_2.config.right_arm || xyz_1.config.elbow_up != xyz_2.config.elbow_up || xyz_1.config.wrist_out != xyz_2.config.wrist_out){
		printf("\nError: Configurations cannot change during straight line move.");
		printf("\nCurrent Configuration:\n");
		print_config(xyz_1.config);
		printf("\nDestination Configuration:\n");
		print_config(xyz_2.config);

		return 0;
	}

	

	printf("CartesianSpeed: %i\n", CartesianSpeed);

	int num_div_cart = 1;
	int num_div_pivot = 1;
	struct Vector U1 = xyz_1.position;
	struct Vector U2 = xyz_2.position;
	struct Vector U21 = subtract(U2, U1);
	struct Vector v21 = normalize(U21);
	double U21_mag = magnitude(U21);
	bool diff_xyz = true;
	if(U21_mag > 100.0){
		num_div_cart = (int)ceil(U21_mag/CartesianStepSize);
	}else{
		diff_xyz = false;
		U21_mag = 0;
		v21.x = 0.0;
		v21.y = 0.0;
		v21.z = 0.0;
	}

	/*
	int max_num_div =  50000;
	if(num_div > max_num_div){
		num_div = max_num_div;
	}
	*/
	
	//double step = U21_mag / num_div;
    double step_cart = U21_mag / num_div_cart;
    double step_pivot;

	//int num_div = 10;
	
	//printf("\nnum_div: %i", num_div);
	
	
	//Smooth Acceleration Math:
	double dx = (double)(cart_speed*cart_speed) / (2*CartesianAcceleration);
	if(2*dx >= U21_mag){
		printf("\nAcceleration too low.\ndx = %f\nU21_mag = %f", dx, U21_mag);
		dx = floor(U21_mag/2);
		cart_speed = (int)round(sqrt(2*(double)(CartesianAcceleration)*dx)); //2*a*dx == 2*a*U21_mag/2
	}

	
	
	struct Vector cur_pos = xyz_1.position;
	struct Vector cur_dir = xyz_1.direction;
	struct Config cur_config = xyz_1.config;
	struct XYZ cur_xyz = new_XYZ(cur_pos, cur_dir, cur_config);
	
	struct Vector rot_cross_p = cross(xyz_1.direction, xyz_2.direction);
	double dir_angle = 0.0;
	bool diff_normal = false;
	
	if(magnitude(rot_cross_p) > 0.005){
		printf("\nNormals are different.");
		
		dir_angle = signed_angle(xyz_1.direction, xyz_2.direction, rot_cross_p);
		printf("\ndir_angle: %f", dir_angle);
		diff_normal = true;
		if(dir_angle / ((float)num_div_cart) > CartesianPivotStepSize){
			num_div_pivot = (int)ceil(dir_angle/CartesianPivotStepSize);
			printf("\num_div: %i", num_div_pivot);
			
		}
		step_pivot = dir_angle / ((float)num_div_pivot);
		printf("\nCartesianPivotStepSize: %i", CartesianPivotStepSize);
	}
	
	
	
	
	
	//struct J_angles J_angles_list[num_div];
	//double speeds_list[num_div];
	struct J_angles J_angles_new;
	struct J_angles J_angles_old = xyz_to_J_angles(cur_xyz);
	
	
	
	
	printf("\nU21:");
	print_vector(U21);
	printf("\nv21:");
	print_vector(v21);
	printf("\nU21_mag: %f", U21_mag);
	//printf("\nnum_div: %i", num_div);
	//printf("\nstep: %f", step);
	printf("\ndx: %f\n", dx);
	
	
	int i;
	int cal_max_angular_velocity = 0;
	int angular_velocity;
	struct Vector Ui;
	double cur_speed;
	double dist = 0.0;
	if(diff_xyz && !diff_normal){
		for(i=1;i<=num_div_cart;i++){
			//Cartesian Interpolation
			dist = ((float)i)*step_cart;
			Ui = add(U1, scalar_mult(dist, v21));
			cur_xyz.position = Ui;
			J_angles_new = xyz_to_J_angles(cur_xyz);
			//Smooth Acceleration Speed Calc:
			if(dist <= dx){
				//cur_speed = 0.5*CartesianSpeed*(-cos(dist*PI/dx) + 1); //S-curve
				cur_speed = dist*(CartesianSpeed-CartesianSpeedStart)/dx + CartesianSpeedStart;
			}else if(dist >= U21_mag - dx){
				//cur_speed = 0.5*CartesianSpeed*(cos((dist - U21_mag + dx)*PI/dx) + 1); //S-curve
				cur_speed = CartesianSpeed - (dist-U21_mag + dx)*(CartesianSpeed-CartesianSpeedEnd)/dx;
			}else{
				cur_speed = CartesianSpeed;
			}
			maxSpeed_arcsec_per_sec = k_tip_speed_to_angle_speed(J_angles_old, J_angles_new, cur_speed);
			startSpeed_arcsec_per_sec = maxSpeed_arcsec_per_sec;
			MoveRobot(J_angles_new.J1, J_angles_new.J2, J_angles_new.J3, J_angles_new.J4, J_angles_new.J5, BLOCKING_MOVE);
		}
	}else if(!diff_xyz && diff_normal){
		for(i=1;i<=num_div_pivot;i++){
			//Cartesian Interpolation
			dist = ((float)i)*step_pivot;
			Ui = add(U1, scalar_mult(dist, v21));
			cur_xyz.position = Ui;
			J_angles_new = xyz_to_J_angles(cur_xyz);
			//Smooth Acceleration Speed Calc:
			if(dist <= dx){
				//cur_speed = 0.5*CartesianSpeed*(-cos(dist*PI/dx) + 1); //S-curve
				cur_speed = dist*(CartesianSpeed-CartesianSpeedStart)/dx + CartesianSpeedStart;
			}else if(dist >= U21_mag - dx){
				//cur_speed = 0.5*CartesianSpeed*(cos((dist - U21_mag + dx)*PI/dx) + 1); //S-curve
				cur_speed = CartesianSpeed - (dist-U21_mag + dx)*(CartesianSpeed-CartesianSpeedEnd)/dx;
			}else{
				cur_speed = CartesianSpeed;
			}
			maxSpeed_arcsec_per_sec = k_tip_speed_to_angle_speed(J_angles_old, J_angles_new, cur_speed);
			startSpeed_arcsec_per_sec = maxSpeed_arcsec_per_sec;
			MoveRobot(J_angles_new.J1, J_angles_new.J2, J_angles_new.J3, J_angles_new.J4, J_angles_new.J5, BLOCKING_MOVE);
		}
	}else{
        printf("diff_xyz: %i, diff_normal: %i", diff_xyz, diff_normal);
    }


	//printf("cal_max_angular_velocity = %i", cal_max_angular_velocity);
	printf("\nMoveRobotStraight movement complete\n");
	
	
	
	/*
	for(i=0;i<=num_div;i++){
		cur_angular_velocity = speeds_list[i];
		
		
		//Startspeed
		//printf("cur_angular_velocity: %i\n", cur_angular_velocity);
		mapped[START_SPEED]=shadow_map[START_SPEED]=1 ^ cur_angular_velocity;
		
		
		//Maxspeed
		maxSpeed=cur_angular_velocity & 0b00000000000011111111111111111111;
		//printf("maxSpeed: %i", cur_angular_velocity & 0b00000000000011111111111111111111);
		mapped[ACCELERATION_MAXSPEED]=shadow_map[ACCELERATION_MAXSPEED]=maxSpeed + (coupledAcceleration << 20);
		//printf("mapped startspeed: %i", maxSpeed + (coupledAcceleration << 20));
		
		//printf("i = %i, J1 = %f", i, J_angles_list[i].J1);
		MoveRobot(J_angles_list[i].J1, J_angles_list[i].J2, J_angles_list[i].J3, J_angles_list[i].J4, J_angles_list[i].J5, BLOCKING_MOVE);
	}
	*/
	
	
	return 0;
}

int JointAngleBoundErr(char ejoint, int eangle, int eboundry) {
	FILE *err_file;
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	err_file = fopen("errors.log", "a");
	if (err_file) {
		tm = *localtime(&t);
		fprintf(err_file, "%04d/%02d/%02d %02d:%02d:%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		fprintf(err_file, ", Joint %i angle %i exceeded boundary %i. \n", ejoint, eangle, eboundry);
		fclose(err_file);
		printf ("Joint %i angle %i exceeded boundary %i. \n", ejoint, eangle, eboundry);
		}
	return BOUNDARY_ERROR_CODE;
	}

int CheckBoundry(int* j1, int* j2, int* j3, int* j4, int* j5) {
	int err = 0; // Error number to return, default to zero
	int h1,h2,h3,h4,h5,l1,l2,l3,l4,l5;
	l1=(int)((float)Boundary[0] / fabs(JointsCal[0]));
	l3=(int)((float)Boundary[2] / fabs(JointsCal[1]));
	l2=(int)((float)Boundary[4] / fabs(JointsCal[2]));
	l4=(int)((float)Boundary[6] / fabs(JointsCal[3]));
	l5=(int)((float)Boundary[8] / fabs(JointsCal[4]));
	h1=(int)((float)Boundary[1] / fabs(JointsCal[0]));
	h3=(int)((float)Boundary[3] / fabs(JointsCal[1]));
	h2=(int)((float)Boundary[5] / fabs(JointsCal[2]));
	h4=(int)((float)Boundary[7] / fabs(JointsCal[3]));
	h5=(int)((float)Boundary[9] / fabs(JointsCal[4]));
	
	if(*(j1) > h1) {
		err = JointAngleBoundErr(1, *(j1), h1);
		*(j1) = h1;
		}
	
	if(*(j1) < l1) {
		err = JointAngleBoundErr(1, *(j1), l1);
		*(j1) = l1;
		}
	if(*(j2) > h2) {
		err = JointAngleBoundErr(2, *(j2), h2);
		*(j2) = h2;
		}
	if(*(j2) < l2) {
		err = JointAngleBoundErr(2, *(j2), l2);
		*(j2) = l2;
		}
	if(*(j3) > h3) {
		err = JointAngleBoundErr(3, *(j3), h3);
		*(j3) = h3;
		}
	if(*(j3) < l3) {
		err = JointAngleBoundErr(3, *(j3), l3);
		*(j3) = l3;
		}
	if(*(j4) > h4) {
		err = JointAngleBoundErr(4, *(j4), h4);
		*(j4) = h4;
		}
	if(*(j4) < l4) {
		err = JointAngleBoundErr(4, *(j4), l4);
		*(j4) = l4;
		}
	if(*(j5) > h5) {
		err = JointAngleBoundErr(5, *(j5), h5);
		*(j5) = h5;
		}
	if(*(j5) < l5) {
		err = JointAngleBoundErr(5, *(j5), l5);
		*(j5) = l5;
		}
	// if(*(j1)==0) *(j1)=1;  //why was this here?
 	return err;
}

int SetParam(char *a1,float fa2,int a3,int a4,int a5, int a6)
{
	int i,BDH,BDL,Axis;
	int a2=(int)fa2;
	int fxa2=(a2<<8)+(fa2-a2)*256;
	unsigned int *uia2 = *(unsigned int*)&fa2;
    int angles_temp[5];

	//printf("SetParam: %s %f \n",a1,fa2);
	//Use isdigit to test if a1 is a number, and if so, atoi and set i to that value. 
	if (isdigit(a1[0])) {
		i = atoi(a1);
#ifdef DEBUG_API
		printf("parm no: %s = %d \n",a1,i);
		fflush(stdout);
#endif
		}
	else {
		for(i=0;i<MAX_PARAMS;i++) {
			//printf("%s %s %d %d \n",Params[i],a1,a2,i);
			if(strcmp(a1,Params[i])==0) break;
			}
		}
#ifdef DEBUG_API
	printf("%s = %s %f (%d) %d %d %d %d %d\n",Params[i],a1,fa2,a2,i,a3,a4,a5,a6); 
	//will segfault if i=MAX_PARAMS (not found)
	fflush(stdout);
#endif
	switch(i)
	{
		case 0:
		////printf("Set Speed\n");
			//set Max Speed
			/*
			maxSpeed=a2 & 0b00000000000011111111111111111111;
			mapped[ACCELERATION_MAXSPEED]=shadow_map[ACCELERATION_MAXSPEED]=maxSpeed + (coupledAcceleration << 20);
			*/
			maxSpeed_arcsec_per_sec = a2 * arcsec_per_nbits; // Depricated. For backward compatibility with _nbits_cf from DDE.
			return 0;
		break;
		case 1:
			//set Acceleration
			coupledAcceleration=a2 & 0b111111;
			mapped[ACCELERATION_MAXSPEED]=shadow_map[ACCELERATION_MAXSPEED]=maxSpeed + (coupledAcceleration << 20);
			return 0;
		break;
		case 2:
			forceBias[0]=a2;
			KeyholeSend(forceBias, FORCE_BIAS_KEYHOLE_CMD, FORCE_BIAS_KEYHOLE_SIZE, FORCE_BIAS_KEYHOLE );

			return 0;
		case 3:
			forceBias[1]=a2;
			KeyholeSend(forceBias, FORCE_BIAS_KEYHOLE_CMD, FORCE_BIAS_KEYHOLE_SIZE, FORCE_BIAS_KEYHOLE );
			return 0;
		break;
		case 4:
			forceBias[2]=a2;
			KeyholeSend(forceBias, FORCE_BIAS_KEYHOLE_CMD, FORCE_BIAS_KEYHOLE_SIZE, FORCE_BIAS_KEYHOLE );
			return 0;
		break;
		case 5:
			forceBias[3]=a2;
			KeyholeSend(forceBias, FORCE_BIAS_KEYHOLE_CMD, FORCE_BIAS_KEYHOLE_SIZE, FORCE_BIAS_KEYHOLE );
			return 0;
		break;
		case 6:
			forceBias[4]=a2;
			KeyholeSend(forceBias, FORCE_BIAS_KEYHOLE_CMD, FORCE_BIAS_KEYHOLE_SIZE, FORCE_BIAS_KEYHOLE );
			return 0;
		break;
		case 7:
			Friction[0]=fxa2;
			KeyholeSend(Friction, FRICTION_KEYHOLE_CMD, FRICTION_KEYHOLE_SIZE, FRICTION_KEYHOLE );
			return 0;
		break;
		case 8:
			Friction[1]=fxa2;
			KeyholeSend(Friction, FRICTION_KEYHOLE_CMD, FRICTION_KEYHOLE_SIZE, FRICTION_KEYHOLE );
			return 0;
		break;
		case 9:
			Friction[2]=fxa2;
			KeyholeSend(Friction, FRICTION_KEYHOLE_CMD, FRICTION_KEYHOLE_SIZE, FRICTION_KEYHOLE );
			return 0;
		break;
		case 10:
			Friction[3]=fxa2;
			KeyholeSend(Friction, FRICTION_KEYHOLE_CMD, FRICTION_KEYHOLE_SIZE, FRICTION_KEYHOLE );
			return 0;
		break;
		case 11:
			Friction[4]=fxa2;
			KeyholeSend(Friction, FRICTION_KEYHOLE_CMD, FRICTION_KEYHOLE_SIZE, FRICTION_KEYHOLE );
			return 0;
		break;
		case 12:
			Boundary[1]=(int)((float)a2 * fabs(JointsCal[0]));
			KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );

			return 0;
		break;
		case 13:
			Boundary[0]=(int)((float)a2 * fabs(JointsCal[0]));
			KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );
			return 0;
		break;
		case 14:
			Boundary[3]=(int)((float)a2 * fabs(JointsCal[2]));
			KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );
			return 0;
		break;
		case 15:
			Boundary[2]=(int)((float)a2 * fabs(JointsCal[2]));
			KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );
			return 0;
		break;
		case 16:
			Boundary[5]=(int)((float)a2 * fabs(JointsCal[1]));
			KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );
			return 0;
		break;
		case 17:
			Boundary[4]=(int)((float)a2 * fabs(JointsCal[1]));
			KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );
			return 0;
		break;
		case 18:
			Boundary[7]=(int)((float)a2 * fabs(JointsCal[3]));
			KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );
			return 0;
		break;
		case 19:
			Boundary[6]=(int)((float)a2 * fabs(JointsCal[3]));
			KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );
			return 0;
		break;
		case 20:
			Boundary[9]=(int)((float)a2 * fabs(JointsCal[4]));
			KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );
			return 0;
		break;
		case 21:
			Boundary[8]=(int)((float)a2 * fabs(JointsCal[4]));
			KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );
			return 0;
		break;
		case 22:
			SetGripperMotor(a2, a3, a4);
			printf("Gripper Motor Set state: %d, on: %d, off: %d\n", a2, a3, a4);
			return 0;
		break;
		case 23:
			SetGripperRoll(a2);
			//printf("Gripper Roll Set\n");
			return 0;
		break;
		case 24:
			SetGripperSpan(a2);
			//printf("Gripper Span Set\n");
			return 0;
		break;
		case 25:
			//mapped[START_SPEED]=shadow_map[START_SPEED]=1 ^ a2; //Replaced
			startSpeed_arcsec_per_sec = a2 * arcsec_per_nbits; // Depricated. For backward compatibility with _nbits_cf from DDE.
			return 0;
		break;
		case 26:     // end speed todo not implemented
			return 0;
		break;
		case 27:     // ServoSet2X
			//printf("Write Packet %d %d %d \n", a2,a3,a4);
			SendWrite2Packet(a4, a2, a3);
			return 0;
		break;
		case 28:     // ServoSet
			SendWrite1Packet((unsigned char)a4, a2, a3);
			return 0;
		break;
		case 29:     // ServoReset
			//printf("Servo Reboot %d",a2);
			RebootServo(a2); 
			return 0;
		break;
		case 30:     // Ctrl
			//This is implimented at a higher level because of the atof on the second argument 
			return 0;
		break;
		case 31:     // J1_PID_P							
			mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]=0;
			mapped[PID_P]=shadow_map[PID_P]=(int)uia2;
			// printf("\nSetting J1_PID_P to:\n");
			// printf("  Float: %f\n", fa2);
			// printf("  Hex: %x\n", uia2);
			return 0;
		break;
		case 32:     // J2_PID_P
			mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]=2;
			mapped[PID_P]=shadow_map[PID_P]=(int)uia2;
			// printf("\nSetting J2_PID_P to:\n");
			// printf("  Float: %f\n", fa2);
			// printf("  Hex: %x\n", uia2);
			return 0;
		break;
		case 33:     // J3_PID_P
			mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]=1;
			mapped[PID_P]=shadow_map[PID_P]=(int)uia2;
			// printf("\nSetting J3_PID_P to:\n");
			// printf("  Float: %f\n", fa2);
			// printf("  Hex: %x\n", uia2);
			return 0;
		break;
		case 34:     // J4_PID_P
			mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]=3;
			mapped[PID_P]=shadow_map[PID_P]=(int)uia2;
			// printf("\nSetting J4_PID_P to:\n");
			// printf("  Float: %f\n", fa2);
			// printf("  Hex: %x\n", uia2);
			return 0;
		break;
		case 35:     // J5_PID_P
			mapped[PID_ADDRESS]=shadow_map[PID_ADDRESS]=4;
			mapped[PID_P]=shadow_map[PID_P]=(int)uia2;
			// printf("\nSetting J5_PID_P to:\n");
			// printf("  Float: %f\n", fa2);
			// printf("  Hex: %x\n", uia2);
			return 0;
		break;
		case 36:
			//AngularSpeed = a2;
			maxSpeed_arcsec_per_sec = a2;
			return 0;
		break;
		case 37:
			//AngularSpeedStartAndEnd = a2;
			startSpeed_arcsec_per_sec = a2;
			return 0;
		break;
		case 38:
			AngularAcceleration = a2;
			return 0;
		break;
		case 39:
			CartesianSpeed = a2;
			return 0;
		break;
		case 40:
			CartesianSpeedStart = a2;
			return 0;
		break;
		case 41:
			CartesianSpeedEnd = a2;
			return 0;
		break;
		case 42:
			CartesianAcceleration = a2;
			return 0;
		break;
		case 43:
			CartesianStepSize = a2;
			return 0;
		break;
		case 44:
			CartesianPivotSpeed = a2;
			return 0;
		break;
		case 45:
			CartesianPivotSpeedStart = a2;
			return 0;
		break;
		case 46:
			CartesianPivotSpeedEnd = a2;
			return 0;
		break;
		case 47:
			CartesianPivotAcceleration = a2;
			return 0;
		break;
		case 48:
			CartesianPivotStepSize = a2;
			return 0;
		break;
		case 49:
			//EyeNumbers
			angles_temp[0]=a2^255; 
			angles_temp[2]=a3^255; 
			angles_temp[1]=a4^255; 
			angles_temp[3]=a5^255; 
			angles_temp[4]=a6^255; 
			printf("EyeNumbers (after xor): %d %d %d %d %d\n",angles_temp[0],angles_temp[1], angles_temp[2],angles_temp[3], angles_temp[4]);
			KeyholeSend(angles_temp, RAW_ECONDER_ANGLE_KEYHOLE_CMD, RAW_ECONDER_ANGLE_KEYHOLE_SIZE, RAW_ECONDER_ANGLE_KEYHOLE );
			return 0;
		break;
		case 50:
			//CommandedAngles
			angles_temp[0]=a2; //atoi(p1); //a2
			angles_temp[1]=a3; //atoi(p2); //a3
			angles_temp[2]=a4; //atoi(p3); //a4
			angles_temp[3]=a5; //atoi(p4); //a5
			angles_temp[4]=a6; //atoi(p5); //a6
			LastGoal[0] = angles_temp[0];
			LastGoal[1] = angles_temp[1];
			LastGoal[2] = angles_temp[2];
			LastGoal[3] = angles_temp[3];
			LastGoal[4] = angles_temp[4];
			printf("CommandedAngles: %i %i %i %i %i\n", angles_temp[0],angles_temp[1], angles_temp[2],angles_temp[3], angles_temp[4]);

			//getNormalizedInput(mapped[BASE_POSITION_AT]) / JointsCal[0];

			/*
			angles_temp[0] = (int)((double)a2 * JointsCal[0]) + getNormalizedInput(BASE_POSITION_AT);
			angles_temp[1] = (int)((double)a4 * JointsCal[2]) + getNormalizedInput(END_POSITION_AT);
			angles_temp[2] = (int)((double)a3 * JointsCal[1]) + getNormalizedInput(PIVOT_POSITION_AT);
			angles_temp[3] = (int)((double)a5 * JointsCal[3]) + getNormalizedInput(ANGLE_POSITION_AT);
			angles_temp[4] = (int)((double)a6 * JointsCal[4]) + getNormalizedInput(ROT_POSITION_AT);
			*/

			angles_temp[0] = ((int)((double)a2 - getNormalizedInput(BASE_POSITION_AT)) * JointsCal[0]);
			angles_temp[1] = ((int)((double)a4 - getNormalizedInput(END_POSITION_AT)) * JointsCal[2]);
			angles_temp[2] = ((int)((double)a3 - getNormalizedInput(PIVOT_POSITION_AT)) * JointsCal[1]);
			angles_temp[3] = ((int)((double)a5 - getNormalizedInput(ANGLE_POSITION_AT)) * JointsCal[3]);
			angles_temp[4] = ((int)((double)a6 - getNormalizedInput(ROT_POSITION_AT)) * JointsCal[4]);

			/*
			angles_temp[0] = -(int)((double)a2 * JointsCal[0]);
			angles_temp[1] = -(int)((double)a4 * JointsCal[2]);
			angles_temp[2] = -(int)((double)a3 * JointsCal[1]);
			angles_temp[3] = -(int)((double)a5 * JointsCal[3]);
			angles_temp[4] = -(int)((double)a6 * JointsCal[4]);
			*/

			printf("CommandedAngles: %i %i %i %i %i (steps)\n",angles_temp[0],angles_temp[1], angles_temp[2],angles_temp[3], angles_temp[4]);
			mapped[COMMAND_REG]=shadow_map[COMMAND_REG]= CMD_MOVEEN | CmdVal;
			KeyholeSend(angles_temp, CMD_POSITION_KEYHOLE_CMD, CMD_POSITION_KEYHOLE_SIZE, CMD_POSITION_KEYHOLE );
			mapped[COMMAND_REG]=shadow_map[COMMAND_REG]= CMD_MOVEEN | CmdVal | 0x80000000; //sets mux to read from commanded angles keyhole
			mapped[COMMAND_REG]=shadow_map[COMMAND_REG]= CMD_MOVEEN | CmdVal | 0xC0000000; //triggers the add
			mapped[COMMAND_REG]=shadow_map[COMMAND_REG]= CMD_MOVEEN | CmdVal;
			return 0;
		break;
		case 51:
			//LinkLengths (implemented in SET_PARAM)
			return 0;
			break;
		case 52:
			//RawEncoderErrorLimits
			bot_state.enc_error_limit[0] = a2;
			bot_state.enc_error_limit[1] = a3;
			bot_state.enc_error_limit[2] = a4;
			bot_state.enc_error_limit[3] = a5;
			bot_state.enc_error_limit[4] = a6;
			return 0;
			break;
		case 53:
			//RawVelocityLimits
			bot_state.enc_velocity_limit[0] = a2;
			bot_state.enc_velocity_limit[1] = a3;
			bot_state.enc_velocity_limit[2] = a4;
			bot_state.enc_velocity_limit[3] = a5;
			bot_state.enc_velocity_limit[4] = a6;
			return 0;
			break;

		default:
			return 1;
			break;
	} //switch
		
	return 1;
}

int MoveRobotRelative(int a1,int a2,int a3,int a4,int a5, int mode)
{
	int b1,b2,b3,b4,b5;
	b1=getNormalizedInput(BASE_POSITION_AT);
	b3=getNormalizedInput(END_POSITION_AT);
	b2=getNormalizedInput(PIVOT_POSITION_AT);
	b4=getNormalizedInput(ANGLE_POSITION_AT);
	b5=getNormalizedInput(ROT_POSITION_AT);
	
	printf("Relative move relative to: %d %d %d %d %d\n",b1,b2,b3,b4,b5);
	return MoveRobot(a1+b1,a2+b2,a3+b3,a4+b4,a5+b5,mode);
}


int ReadDMA(int p1,int p2,char *p3)
{
	FILE *fp;
	int i,j,blocks,writeSize;
	int dataarray[256];
	fp=fopen(p3, "wb");
	if(fp!=0)
	{
		mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=DMA_RESET_ALL;
		mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=0;
		blocks=p2/256;
		for(j=0;j<blocks;j++) // only do full blocks inside loop
		{	
			mapped[DMA_READ_ADDRESS]=shadow_map[DMA_READ_ADDRESS]=p1+(j*1024);
			mapped[DMA_READ_PARAMS]=shadow_map[DMA_READ_PARAMS]=(2<<8) | 127;
			mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=DMA_READ_BLOCK;
			mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=0;
			for(i=0;i<256;i++)
			{
				mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=DMA_READ_DEQUEUE;
				dataarray[i]=mapped[DMA_READ_DATA];
				////printf("\n %d",dataarray[i]);
				mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=0;
			}
			writeSize=fwrite((const void *)dataarray,sizeof(int),256,fp);
			//printf("\n write %d iteration %d",writeSize,j);
			mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=0;
		}
		fclose(fp);
	}
	else
	{
		//printf("\n %s",p3);
	}
	return 0;
}
void SaveTables(int Address,char *FileName)
{
	FILE *fp;
	int writeSize=0,i;
	// fill in unwritten tables
	
	//for(i=0;i<)
	fp=fopen(FileName, "w");
	
	if(fp!=0)
	{
		fseek(fp, 0, SEEK_SET);    /* file pointer at the beginning of file */
		writeSize=fwrite((const void *)CalTables,sizeof(int),4*1024*1024,fp);
		fclose(fp);
	}
	fp=fopen("memText.txt", "w");
	
	if(fp!=0)
	{
		fseek(fp, 0, SEEK_SET);    /* file pointer at the beginning of file */
		fprintf(fp,"var fileArray = [");
		for(i = 0;i < 4*1024*1024;i ++)
		{
			fprintf(fp,"%d, ",CalTables[i]);
		}
		fprintf(fp,"0 ]");
		fclose(fp);
	}
}
void InterpTables(int start,int length)
{
	int i;
	int avgUP=0;
	int avgDown=0,StartUP=0,StartDown=0;
	StartUP = CalTables[(start + length)];
	StartDown = CalTables[(start + 0x200000/4) - (length)];
	for(i=0;i<1000;i++)
	{
		avgUP=avgUP+abs(abs(CalTables[(start + length)-i]) - abs(CalTables[(start + length)-i-1]));
		avgDown=avgDown+abs(abs(CalTables[(start + 0x200000/4) - (length) + i]) - abs(CalTables[(start + 0x200000/4) - (length) + i + 1])) ;
		
	}
	 //printf("\n %x %x ",avgDown,avgUP);
	avgDown=avgDown/1000;
	avgUP=avgUP/1000;
	//printf("\n %x %x ",avgDown,avgUP);
	for(i=0;i< (0x200000/4) - (length);i++)
	{
		CalTables[start + length+i]=StartUP - (avgUP * i);
		CalTables[(start + 0x200000/4) - length-i] = StartDown + (avgDown * i);
	}

}
int RestoreCalTables(char *FileName)
{
	FILE *fp;
	int readSize,Length,i,avgUP,avgDown,StartUP,StartDown;
	fp=fopen(FileName, "rb");
	for(i=0;i<4*1024*1024;i++)
	{
		CalTables[i]=0;		
	}
	if(fp!=0)
	{
		fseek(fp, 0, SEEK_END);    /* file pointer at the end of file */
		Length = ftell(fp);   /* take a position of file pointer size variable */
		fseek(fp, 0, SEEK_SET);    /* file pointer at the beginning of file */
		if((readSize=fread((void *)CalTables,sizeof(int),4*1024*1024,fp))==(Length/4))
		{

			//printf(" size good %d %d",readSize ,Length);
			#ifndef NO_BOOT_DANCE

			strlcpy(iString, "S RunFile BootDance_RestoredCalTable.make_ins ;\0", ISTRING_LEN);
			printf("Starting %s returned %d\n",iString, ParseInput(iString));

			#endif
		}
		else
		{
			//printf(" size not good %d %d ",readSize ,Length);
		}
		fclose(fp);
		//InterpTables(L1_TABLE/4,L1_TABLE_LENGTH/4);
		//InterpTables(L2_TABLE/4,L2_TABLE_LENGTH/4);
		//InterpTables(L3_TABLE/4,L3_TABLE_LENGTH/4);
		//InterpTables(L4_TABLE/4,L4_TABLE_LENGTH/4);
		//InterpTables(L5_TABLE/4,L5_TABLE_LENGTH/4);
		return 0;
	}
	return 1;
}
int WriteDMA(int Address,char *FileName)
{
	FILE *fp;
	int i,j,blocks,readSize,Length=0;
	int dataarray[256];
	fp=fopen(FileName, "rb");
	if(fp!=0)
	{
		
		fseek(fp, 0, SEEK_END);    /* file pointer at the end of file */
		Length = ftell(fp);   /* take a position of file pointer size variable */
		fseek(fp, 0, SEEK_SET);    /* file pointer at the beginning of file */
		mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=DMA_RESET_ALL;
		mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=0;
		blocks=Length/256;
		for(j=0;j<blocks;j++) // only do full blocks inside loop
		{	
			if( ( readSize=fread((void *)dataarray,sizeof(int),256,fp) )==256)
			{
				mapped[DMA_WRITE_ADDRESS]=shadow_map[DMA_WRITE_ADDRESS]=Address+(j*1024);
				mapped[DMA_WRITE_PARAMS]=shadow_map[DMA_WRITE_PARAMS]=(2<<8) | 127;
				mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=0;
				for(i=0;i<256;i++)
				{
					mapped[DMA_WRITE_DATA]=shadow_map[DMA_WRITE_DATA]=dataarray[i];
					mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=DMA_WRITE_ENQUEUE;
					mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=0;
				}
				mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=DMA_WRITE_INITIATE;
				mapped[DMA_CONTROL]=shadow_map[DMA_CONTROL]=0;
				////printf("\n write %d iteration %d",readSize,j);
			}
		}
		fclose(fp);
		//printf("\n Table Loaded: Length %d",Length);
	}
	else
	{
	    //printf("errno = %d.\n", errno);
		//perror(errno);
		//printf("\n failed to open %s",FileName);
	}
	return Length;
}

int CaptureADtoFile(int Axis,int Start,int Length,int Delay,char *FileName)
{
	FILE *fp;
	int i,j,k,blocks,writeSize,AvgSIN,AvgCOS,ADVal;
	int dataarray[512];
	switch(Axis)
	{
	    case 0  :
		MoveRobot(Start,0,0,0,0,BLOCKING_MOVE);
		break; 
	    case 1  :
		MoveRobot(0,Start,0,0,0,BLOCKING_MOVE);
		break; 
	    case 2  :
		MoveRobot(0,0,Start,0,0,BLOCKING_MOVE);
		break; 
	    case 3  :
		MoveRobot(0,0,0,Start,0,BLOCKING_MOVE);
		break; 
	    case 4  :
		MoveRobot(0,0,0,0,Start,BLOCKING_MOVE);
		break; 
	}
	fp=fopen(FileName, "w");
	if(fp!=0)
	{
		blocks=Length/256;
		for(k=0;k<blocks;k++) // only do full blocks inside loop
		{	
			for(i=0;i<256;i++)
			{
				mapped[Axis]=shadow_map[Axis]=Start+i+(k*256);
				AvgSIN=0;
				AvgCOS=0;
				for(j=0;j<Delay;j++)
				{	
					ADVal=mapped[ADLookUp[Axis]];
					AvgSIN=AvgSIN+ADVal;
					AvgCOS=AvgCOS+mapped[ADLookUp[Axis]+1];
				}			
				dataarray[i*2]=AvgSIN/Delay;
				dataarray[(i*2)+1]=AvgCOS/Delay;
//				//printf("\n SIN %d COS %d",dataarray[i*2],dataarray[(i*2)+1]);
			}
			writeSize=fwrite((const void *)dataarray,sizeof(int),512,fp);
			//printf("\n write %d iteration %d",writeSize,k);
		}
		fclose(fp);
	}
	else
	{
		//printf("\n %s",FileName);
	}
	return 0;
}
int FindHome(int Axis,int Start,int Length,int Delay,char *FileName)
{
	FILE *fp;
	int i,j,k,blocks,AvgSIN,AvgCOS,ADVal,fileLen,ReadSize,MinSIN,MinCOS,MinSINIdx=0,MinCOSIdx=0;
	int RefrenceArray[1000000];
	int CaptureArray[Length*2];
	switch(Axis)
	{
	    case 0  :
		MoveRobot(Start,0,0,0,0,BLOCKING_MOVE);
		break; 
	    case 1  :
		MoveRobot(0,Start,0,0,0,BLOCKING_MOVE);
		break; 
	    case 2  :
		MoveRobot(0,0,Start,0,0,BLOCKING_MOVE);
		break; 
	    case 3  :
		MoveRobot(0,0,0,Start,0,BLOCKING_MOVE);
		break; 
	    case 4  :
		MoveRobot(0,0,0,0,Start,BLOCKING_MOVE);
		break; 
	}
	fp=fopen(FileName, "rb");
	if(fp!=0)
	{
		fseek(fp, 0, SEEK_END);
		fileLen=ftell(fp);
		fseek(fp, 0, SEEK_SET);
		ReadSize=fread((char *)RefrenceArray, fileLen, 1, fp);
		fclose(fp);
	}
	else
	{
		//printf("\n %s",FileName);
		return 1;
	}
	for(k=0;k<Length;k++) 
	{	
		mapped[Axis]=shadow_map[Axis]=Start+k;
		AvgSIN=0;
		AvgCOS=0;
		for(j=0;j<Delay;j++)
		{	
			ADVal=mapped[ADLookUp[Axis]];
			AvgSIN=AvgSIN+ADVal;
			AvgCOS=AvgCOS+mapped[ADLookUp[Axis]+1];
		}			
		CaptureArray[k*2]=AvgSIN/Delay;
		CaptureArray[(k*2)+1]=AvgCOS/Delay;
//		//printf("\n Query %d Refrence %d",CaptureArray[k*2],RefrenceArray[k*2]);
	}
	MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
	blocks=fileLen/8;
	MinSIN=0x7fffffff;
	MinCOS=0x7fffffff;
	for(i=0;i<blocks;i++)
	{
		AvgSIN=0;
		AvgCOS=0;
		for(k=0;k<Length;k++)
		{
			AvgSIN=AvgSIN+abs(CaptureArray[k*2]-RefrenceArray[(k+i)*2]);
			AvgSIN=AvgSIN+abs(CaptureArray[(k*2)+1]-RefrenceArray[(((k+i)*2)+1)]);
//			AvgCOS=AvgCOS+abs(CaptureArray[(k*2)+1]-RefrenceArray[(((k+i)*2)+1)]);
		}
		if(MinSIN>AvgSIN)
		{
			MinSIN=AvgSIN;
			MinSINIdx=i;
		}
/*		if(MinCOS>AvgCOS)
		{
			MinCOS=AvgCOS;
			MinCOSIdx=i;
		}*/
	}
	//printf("\n SAD %d SINidx %d\n",MinSIN,MinSINIdx);
	switch(Axis)
	{
	    case 0  :
		MoveRobot(30000-MinSINIdx,0,0,0,0,BLOCKING_MOVE);
		break; 
	    case 1  :
		MoveRobot(0,30000-MinSINIdx,0,0,0,BLOCKING_MOVE);
		break; 
	    case 2  :
		MoveRobot(0,0,30000-MinSINIdx,0,0,BLOCKING_MOVE);
		break; 
	    case 3  :
		MoveRobot(0,0,0,30000-MinSINIdx,0,BLOCKING_MOVE);
		break; 
	    case 4  :
		MoveRobot(0,0,0,0,30000-MinSINIdx,BLOCKING_MOVE);
		break; 
	}
	mapped[COMMAND_REG]=shadow_map[COMMAND_REG]=256;  //reset home
	mapped[COMMAND_REG]=shadow_map[COMMAND_REG]=0;
	return 30000-MinSINIdx;
}


void showPosAt(void)
{
		int b1,b2,b3,b4,b5;
		b1=getNormalizedInput(BASE_POSITION_AT);
		b3=getNormalizedInput(END_POSITION_AT);
		b2=getNormalizedInput(PIVOT_POSITION_AT);
		b4=getNormalizedInput(ANGLE_POSITION_AT);
		b5=getNormalizedInput(ROT_POSITION_AT);
		//printf("\nPos %d %d %d %d %d  ",b1,b2,b3,b4,b5);	
}
void ReplayMovement(char *FileName)
{
	int Length,rbc;
	showPosAt();
	Length=WriteDMA(0x3f000000,FileName);
	mapped[RECORD_LENGTH]=shadow_map[RECORD_LENGTH]=Length/4;
	mapped[REC_PLAY_TIMEBASE]=shadow_map[REC_PLAY_TIMEBASE]=1;
	mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=CMD_RESET_RECORD;
	mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=CMD_RESET_PLAY;
	mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=0;
	mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=CMD_PLAYBACK;
	rbc=mapped[READ_BLOCK_COUNT];
	showPosAt();
	
	//sleep(1);
	while((mapped[READ_BLOCK_COUNT] & 0x003fffff) != 0 )
	{
		if(rbc != mapped[READ_BLOCK_COUNT])
		{
				
			//printf("%d \n",mapped[READ_BLOCK_COUNT] & 0x003fffff);
			rbc=mapped[READ_BLOCK_COUNT];
			showPosAt();
		}
		////printf("%d \n",mapped[READ_BLOCK_COUNT]);
	}
	while((mapped[READ_BLOCK_COUNT] & 0x00400000) != 0 )
		//printf("%d \n",mapped[READ_BLOCK_COUNT]);
	showPosAt();
	mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=CMD_RESET_RECORD;
	mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=CMD_RESET_PLAY;
	mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=0;

	/*fa0=fa0+mapped[PLAYBACK_BASE_POSITION];
	fa1=fa1+mapped[PLAYBACK_END_POSITION];
	fa2=fa2+mapped[PLAYBACK_PIVOT_POSITION];
	fa3=fa3+mapped[PLAYBACK_ANGLE_POSITION];
	fa4=fa4+mapped[PLAYBACK_ROT_POSITION];
	mapped[FINE_ADJUST_BASE]=shadow_map[FINE_ADJUST_BASE]=fa0;
	mapped[FINE_ADJUST_END]=shadow_map[FINE_ADJUST_END]=fa1;
	mapped[FINE_ADJUST_PIVOT]=shadow_map[FINE_ADJUST_PIVOT]=fa2;
	mapped[FINE_ADJUST_ANGLE]=shadow_map[FINE_ADJUST_ANGLE]=fa3;
	mapped[FINE_ADJUST_ROT]=shadow_map[FINE_ADJUST_ROT]=fa4;*/
	mapped[REC_PLAY_CMD]=shadow_map[REC_PLAY_CMD]=CMD_RESET_RECORD;
}


int getInput(void)
{
	if(fgets(iString,sizeof(iString),stdin)!=NULL)
	{
		return ParseInput(iString);
	}
    return 0;
}

unsigned char h2int(char c) {
  if (c >= '0' && c <='9') { return((unsigned char)c - '0');      }
  if (c >= 'a' && c <='f') { return((unsigned char)c - 'a' + 10); }
  if (c >= 'A' && c <='F') { return((unsigned char)c - 'A' + 10); }
  return(0);
  }

int unescape(char *buf, int len) {
// (originally based on https://code.google.com/p/avr-netino/)
  char c;
  int olen=0;
  char *out;
  out = buf;
  for(;len>0;len--) {
    c = *buf++;
    if (c == '%') {
      c = *buf++;len--;
      c = (h2int(c) << 4) | h2int(*buf++);len--;
      }
    *out++=c;olen++;
    }
  return olen;
  }

int ParseInput(char *iString)
{
	//char iString[255];
	const char delimiters[] = " ,";
	const char ctrldelims[] = ":[] ,";
	FILE *fp;
	char *token,*p1,*p2,*p3,*p4,*p5,*p6,*p7,*p8,*p9,*p10,*p11,*p12,*p13,*p14,*p15,*p16,*p17,*p18,*p19;
	int BDH,BDL;

	int i,j,Add,Start,Length,Delay,Axis,tokenVal;
	float f1;
	int d2,d3,d4,d5;
	////printf("\nStart wait Goal");
#ifdef DEBUG_API
	printf("ParseInput: %s\n", iString);
#endif
	if(iString !=NULL)
	{
		token = strtok (iString, delimiters);
		if (token !=NULL)
		{
			tokenVal=HashInputCMD(token);
#ifdef DEBUG_API
			printf("Token %s TokenVal %i",token,tokenVal);
			fflush(stdout);
#endif
		}
		else
			return 1;
		if (tokenVal != 0 ){
			switch(tokenVal)
			{
				case WRITE_TO_ROBOT:
					p1=strtok (NULL, delimiters);
                              Add=(int)p1[0];
                              //printf("write %s %d: \n",p1,Add);
					switch(Add) {
					   case 'f': //filename
						p2=strtok(NULL, delimiters);//always zero, toss it.
						p2=strtok(NULL, delimiters);//filename
						if(wfp>0) fclose(wfp);
						wfp = fopen(p2, "w");
						reloadDefaults = false;
						if (!strcmp(p2,"/srv/samba/share/AdcCenters.txt")) reloadDefaults = true; 
                        printf("\nWriting file to %s as handle %d...\n",p2,fileno(wfp));
						break;
					    case 's': //start
					    case 'm': //middle
					    case 'e': //end
						p2=strtok(NULL, delimiters);//bytes
							Length=atoi(p2);
							//printf("Length: %d bytes. \n", Length);
							if (0<Length) {
								p3=strtok(NULL, "");//remaining data
								//printf("Found: %d bytes. ", strlen(p3));
								Length = unescape(p3, Length);
								//printf(" %d bytes after unescape",Length);
								i=fwrite(p3, 1, Length, wfp);
								//printf("Wrote %d bytes. ",i);
								}
						if('e'==Add && (wfp>0)) {
							fclose(wfp);
							wfp = 0;
							printf("...Finished writing.\n");
							if (reloadDefaults) readAdcCenters();
							}
						break;
				          default : 
		              		printf("\nunrecognized subcommand");
            				break;
						}
				break;
				case PID_FINEMOVE :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);

                                        p6=strtok (NULL, delimiters);
                                        if (p6 && 'N'!=p6[0]) SetGripperRoll(atoi(p6));
                                        p7=strtok (NULL, delimiters);
                                        if (p7 && 'N'!=p7[0]) SetGripperSpan(atoi(p7));

					moverobotPID(atoi(p1),atoi(p2),atoi(p3),atoi(p4),atoi(p5));
				break; 
				case HEART_BEAT :
				break; 
			
				case SET_FORCE_MOVE_POINT :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					ForceDestination[0]=atoi(p1);
					ForceDestination[1]=atoi(p2);
					ForceDestination[2]=atoi(p3);
					ForceDestination[3]=atoi(p4);
					ForceDestination[4]=atoi(p5);
				break; 
	
				case SEND_HEARTBEAT :
					////printf("heartbeat dispatched\n");
				break; 
				case SET_PARAM :
					p1=strtok (NULL, delimiters);

					//printf("SET_PARAM: %s\n", p1);
					if (!strcmp("RunFile",p1) || !strcmp("30",p1)) {
						p2 = strtok (NULL, delimiters);
						fp = fopen(p2, "r");
						if (fp) {
							printf("Opened %s as handle %d\n", p2, fileno(fp));
							do {
								p3 = fgets(iString, ISTRING_LEN, fp);
								if (p3) { //not EOF
									p4 = strchr(iString,';');//,ISTRING_LEN); //Check that there is a ';' in there.
									if (p4) { //found a ;
										*p4=0; //terminate at the ;
										//printf(" read: \"%s\"\n", iString);
										ParseInput(iString); //recursivly execute
										} 
									}
								} while (p3);
							fclose(fp);
							printf("Done\n");
							}
						else { 
							printf("Failed to load %s Error # %d %s\n", p2, errno, strerror(errno)); 
							return errno;
							}
                    
					}else if(!strcmp("LinkLengths",p1)){
						p2=strtok (NULL, delimiters);
						p3=strtok (NULL, delimiters);
						p4=strtok (NULL, delimiters);
						p5=strtok (NULL, delimiters);
						p6=strtok (NULL, delimiters);
						if (p2!=NULL) L[4]=atof(p2); //Link 5 FIRST!
						if (p3!=NULL) L[3]=atof(p3); // then Link 4
						if (p4!=NULL) L[2]=atof(p4); // so that Link 3
						if (p5!=NULL) L[1]=atof(p5); // and on become
						if (p6!=NULL) L[0]=atof(p6); // optional. 
						printf("LinkLengths: %lf, %lf, %lf, %lf, %lf \n", L[0], L[1], L[2], L[3], L[4]);

					}else {
						//printf("generic %s\n",p1);
						p2=strtok (NULL, delimiters);
						p3=strtok (NULL, delimiters);
						p4=strtok (NULL, delimiters);
						p5=strtok (NULL, delimiters);
						p6=strtok (NULL, delimiters);
						f1=0.0;
						d2=d3=d4=d5=0;
						if(p2!=NULL) f1=atof(p2);
						if(p3!=NULL) d2=atoi(p3);
						if(p4!=NULL) d3=atoi(p4);
						if(p5!=NULL) d4=atoi(p5);
						if(p6!=NULL) d5=atoi(p6);
						return SetParam(p1,f1,d2,d3,d4,d5);
					}
				break; 
				case MOVEALL_RELATIVE :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					MoveRobotRelative(atoi(p1),atoi(p2),atoi(p3),atoi(p4),atoi(p5),BLOCKING_MOVE);
				break; 
				case REPLAY_MOVEMENT  :
					p1=strtok (NULL, delimiters);
					ReplayMovement(p1);
				break; 
				case SLEEP_CMD  :
					p1=strtok (NULL, delimiters);
					usleep(atoi(p1));
				break; 
				case MOVE_CMD  :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
				break; 
				case LOAD_TABLES  :
					SaveTables(0x3f800000,"/srv/samba/share/HiMem.dta");
					//WriteDMA(0x3f900000,"/srv/samba/share/a1tbl.dta");
					//WriteDMA(0x3fa00000,"/srv/samba/share/a2tbl.dta");
				break; 
				case CAPTURE_POINTS_CMD :
					p1=strtok (NULL, delimiters);
					InitCapturePoints(p1);
				break;
				case RECORD_MOVEMENT :
					p1=strtok (NULL, delimiters);
					InitCaptureMovement(p1);
				break;
				case DMAREAD_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					Add=(int)strtol(p1,0,16);
					Length=(int)strtol(p2,0,16);
					ReadDMA(Add,Length,p3);
				break;
				case DMAWRITE_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					Add=(int)strtol(p1,0,16);
					WriteDMA(Add,p2);
				break;
				case CAPTURE_AD_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					Axis=atoi(p1);
					Start=atoi(p2);
					Length=atoi(p3);
					Delay=atoi(p4);
					CaptureADtoFile(Axis,Start,Length,Delay,p5);
				break;
				case FIND_HOME_REP_CMD :
					p1=strtok (NULL, delimiters);
					Axis=atoi(p1);
					Start=0;
					Length=5000;
					Delay=10000;
					switch(Axis)
					{
						case 0  :
							while(abs(FindHome(Axis,Start,Length,Delay,"/srv/samba/share/d0.dta"))>300);
						break; 
						case 1  :
							while(abs(FindHome(Axis,Start,Length,Delay,"/srv/samba/share/d1.dta"))>300);
						break; 
						case 2  :
							while(abs(FindHome(Axis,Start,Length,Delay,"/srv/samba/share/d2.dta"))>300);
						break; 
						case 3  :
							while(abs(FindHome(Axis,Start,Length,Delay,"/srv/samba/share/d3.dta"))>300);
						break; 
						case 4  :
							while(abs(FindHome(Axis,Start,Length,Delay,"/srv/samba/share/d4.dta"))>300);
						break; 
					}
				break; 
					case FIND_HOME_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					Axis=atoi(p1);
					Start=atoi(p2);
					Length=atoi(p3);
					Delay=atoi(p4);
					FindHome(Axis,Start,Length,Delay,p5);
				break; 
				case FIND_INDEX_CMD :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					Axis=atoi(p1);
					Start=atoi(p2);
					Length=atoi(p3);
					Delay=atoi(p4);
					if(FindIndex(Axis,Start,Length,Delay)==0)
					{
						//printf("/nIndex reached");
						break;
					}
					else
					{
						//printf("/nIndex not found");
					}
				break;
				case MOVEALL_CMD :
					//printf("\nMOVEALL_CMD\n");
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);

                                        p6=strtok (NULL, delimiters);
                                        if (p6 && 'N'!=p6[0]) SetGripperRoll(atoi(p6));
                                        //if(p6 != NULL){ printf("p6 %s\n",p6); }
                                        //else{ printf("p6 doesn't exist\n"); }
                                        p7=strtok (NULL, delimiters);
                                        if (p7 && 'N'!=p7[0]) SetGripperSpan(atoi(p7));
                                        //if(p7 != NULL){ printf("p7 %s\n",p7); }
                                        //else{ printf("p7 doesn't exist\n"); }

					
					if(p1!=NULL && p2!=NULL && p3!=NULL && p4!=NULL && p5!=NULL)	
						MoveRobot(atoi(p1),atoi(p2),atoi(p3),atoi(p4),atoi(p5),BLOCKING_MOVE);
				break; 
				
				
				//////////////////////////////////////////////////////////////////////////
				/* Start Wigglesworth Code*/
				
				case MOVETO_CMD:
					//printf("\nMOVETO_CMD\n");
					//MoveRobot(36000, 36000, 36000, 36000, 36000, BLOCKING_MOVE);
					
					p1 = strtok(NULL, delimiters);
					p2 = strtok(NULL, delimiters);
					p3 = strtok(NULL, delimiters);
					p4 = strtok(NULL, delimiters);
					p5 = strtok(NULL, delimiters);
					p6 = strtok(NULL, delimiters);
					p7 = strtok(NULL, delimiters);
					p8 = strtok(NULL, delimiters);
					p9 = strtok(NULL, delimiters);
					
					//printf("xyz: [%d, %d, %d] dir: [%d, %d, %d] config: [%d, %d, %d]\n", p1f, p2f, p3f, p4f, p5f, p6f, p7f, p8f, p9f);
					
					struct Vector my_point = new_vector((float)atoi(p1), (float)atoi(p2), (float)atoi(p3));
					struct Vector my_dir = new_vector((float)atoi(p4), (float)atoi(p5), (float)atoi(p6));
					struct Config my_config = new_config((bool)atoi(p7), (bool)atoi(p8), (bool)atoi(p9));
					struct XYZ xyz_1 = new_XYZ(my_point, my_dir, my_config);
					struct J_angles result_J_angles = xyz_to_J_angles(xyz_1);
					
					int J1 = (int)round(result_J_angles.J1);
					int J2 = (int)round(result_J_angles.J2);
					int J3 = (int)round(result_J_angles.J3);
					int J4 = (int)round(result_J_angles.J4);
					int J5 = (int)round(result_J_angles.J5);
					
					//printf("\nJangles: \n");
					//printf("[%d, %d, %d, %d, %d]", J1, J2, J3, J4, J5);
					//printf("\n");
					

					if (p1 != NULL && p2 != NULL && p3 != NULL && p4 != NULL && p5 != NULL)
						MoveRobot(J1, J2, J3, J4, J5, BLOCKING_MOVE);
						//MoveRobot(36000, 36000, 36000, 36000, 36000, BLOCKING_MOVE);
				break;
				
				case MOVETOSTRAIGHT_CMD:
					//printf("\n\nStarting MoveToStraight3\n");

					p1 = strtok(NULL, delimiters);
					p2 = strtok(NULL, delimiters);
					p3 = strtok(NULL, delimiters);
					p4 = strtok(NULL, delimiters);
					p5 = strtok(NULL, delimiters);
					p6 = strtok(NULL, delimiters);
					p7 = strtok(NULL, delimiters);
					p8 = strtok(NULL, delimiters);
					p9 = strtok(NULL, delimiters);
					
					printf("\np1: %f", (float)atoi(p1));
					printf("\np2: %f", (float)atoi(p2));
					printf("\np3: %f", (float)atoi(p3));
					printf("\np4: %f", (float)atoi(p4));
					printf("\np5: %f", (float)atoi(p5));
					printf("\np6: %f", (float)atoi(p6));
					printf("\np7: %f", (float)atoi(p7));
					printf("\np8: %f", (float)atoi(p8));
					printf("\np9: %f", (float)atoi(p9));
					
					/*
					struct J_angles measured_angles = new_J_angles(
							(float)(getNormalizedInput(BASE_MEASURED_ANGLE)),
							(float)(getNormalizedInput(PIVOT_MEASURED_ANGLE)),
							(float)(getNormalizedInput(END_MEASURED_ANGLE)),
							(float)(getNormalizedInput(ANGLE_MEASURED_ANGLE)),
							(float)(getNormalizedInput(ROT_MEASURED_ANGLE))
						);
					
					
					struct XYZ xyz_start = J_angles_to_XYZ(measured_angles);
					printf("\nxyz_start:\n");
					print_XYZ(xyz_start);
					*/
					
					// struct Vector my_point_start = new_vector(-100000.0, 500000.0, 100000.0);
					// struct Vector my_dir_start = new_vector(0.0, 0.0, -1.0);
					// struct Config my_config_start = new_config(1, 1, 1);
					// struct XYZ xyz_start = new_XYZ(my_point_start, my_dir_start, my_config_start);


					struct Vector my_point_end = new_vector((float)atoi(p1), (float)atoi(p2), (float)atoi(p3));
					struct Vector my_dir_end = new_vector((float)atoi(p4), (float)atoi(p5), (float)atoi(p6));
					struct Config my_config_end = new_config((bool)atoi(p7), (bool)atoi(p8), (bool)atoi(p9));
					struct XYZ xyz_end = new_XYZ(my_point_end, my_dir_end, my_config_end);
					printf("\nxyz_end:\n");
					print_XYZ(xyz_end);
					
					if (p1 != NULL && p2 != NULL && p3 != NULL && p4 != NULL && p5 != NULL){
						//printf("\n\nStarting MoveRobotStraight:\n");
						
						MoveRobotStraight(xyz_end);
						//MoveRobot(J1, J2, J3, J4, J5, BLOCKING_MOVE);
					}
					
				break;
				
					
				/* End Wigglesworth Code*/
				//////////////////////////////////////////////////////////////////////////
				
				case SLOWMOVE_CMD  :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					Add=atoi(p1);
					Length=atoi(p3);
					Start=atoi(p2);
					Delay=atoi(p4);
					mapped[Add]=shadow_map[Add]=Start;
					//printf("\n %d %d \n",Add,Length);
					for(i=0;i<Length;i++)
					{
						for(j=0;j<Delay;j++)			
							mapped[Add]=shadow_map[Add]=Start+i;
					}
				break;
				//case READ_CMD  : //now this gets managed during the 'r' reply
				//	p1=strtok (NULL, delimiters);
				//	p2=strtok (NULL, delimiters);
				//	if((p1==NULL) | (p2==NULL)) {
				//		//printf("\n %d %d need addres and length",atoi(p1),atoi(p2));
				//		break;
				//	}
				//	Add=atoi(p1);
				//	Length=atoi(p2); 
				//	//printf("\n %d %d \n",Add,Length);
				//	for(i=0;i<Length;i++) {
				//		//printf("\n  %x %d",CalTables[Add+i],CalTables[Add+i]);
				//		printf("\n %x ",mapped[Add+i]);
				//	}
				//break; 
				case WRITE_CMD  :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					if((p1==NULL) | (p2==NULL))
					{
						//printf("\n %d %d need addres and data",atoi(p1),atoi(p2));
					}
					i=atoi(p1);
					j=atoi(p2);
					//printf("\n %d %d %d \n",OldMemMapInderection[i],i,j);
					if(OldMemMapInderection[i]==COMMAND_REG)
						CmdVal=j;
					//printf("\n  mapped[%d] = %d;\n",OldMemMapInderection[i],j);
					//to build up a set of FPGA commands, just use the above printf and copy the result into your code.
					mapped[OldMemMapInderection[i]]=j;
					if(i==ACCELERATION_MAXSPEED)
					{
						maxSpeed=j & 0b00000000000011111111111111111111;
						coupledAcceleration=(j & 0b111111111111100000000000000000000) >> 20;
						//startSpeed=0,
						
					}
				break; 
				case SET_ALL_BOUNDRY :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					p6=strtok (NULL, delimiters);
					p7=strtok (NULL, delimiters);
					p8=strtok (NULL, delimiters);
					p9=strtok (NULL, delimiters);
					p10=strtok (NULL, delimiters);
					Boundary[1]=(int)((float)atoi(p1) * fabs(JointsCal[0])); //high
					Boundary[0]=(int)((float)atoi(p2) * fabs(JointsCal[0])); //low
					Boundary[3]=(int)((float)atoi(p5) * fabs(JointsCal[2]));
					Boundary[2]=(int)((float)atoi(p6) * fabs(JointsCal[2]));
					Boundary[5]=(int)((float)atoi(p3) * fabs(JointsCal[1]));
					Boundary[4]=(int)((float)atoi(p4) * fabs(JointsCal[1]));
					Boundary[7]=(int)((float)atoi(p7) * fabs(JointsCal[3]));
					Boundary[6]=(int)((float)atoi(p8) * fabs(JointsCal[3]));
					Boundary[9]=(int)((float)atoi(p9) * fabs(JointsCal[4]));
					Boundary[8]=(int)((float)atoi(p10) * fabs(JointsCal[4]));
					printf(" Boundary set %d %d %d %d %d %d %d %d %d %d \n", Boundary[0],Boundary[1],Boundary[2],Boundary[3],Boundary[4],Boundary[5],Boundary[6],Boundary[7],Boundary[8],Boundary[9]);
					KeyholeSend(Boundary, BOUNDRY_KEYHOLE_CMD, BOUNDRY_KEYHOLE_SIZE, BOUNDRY_KEYHOLE );
					
				break;
				
				
				case EXIT_CMD  :
/*
					mapped[FINE_ADJUST_BASE]=0;
					mapped[FINE_ADJUST_END]=0;
					mapped[FINE_ADJUST_PIVOT]=0;
					mapped[FINE_ADJUST_ANGLE]=0;
					mapped[FINE_ADJUST_ROT]=0;
*/
					return 1;
				break; 
				
		
				
				default : 
				  //printf("\nunrecognized command");
				  break;
			}
			return 0;
		}
		else{
			return 1;
		}
	}
	return 1;
}

int main(int argc, char *argv[]) {
  printf("Start of main()\n");
  //int err_code = 0;
  //printf("hello world %s", err_code);
  
  
  int fd,mfd,err;
  
  int size;
  int DefaultMode;
  int CalTblSize = 32*1024*1024; 

  if (argc != 4) {
    fprintf(stderr, "Usage: %s Needs init mode, Master/Slave and RunMode\n", argv[0]);
    exit(1);
  }

  DefaultMode = atoi(argv[1]);
  size = 4095;
  RunMode=atoi(argv[3]);
  ServerMode = atoi(argv[2]);
  if (size <= 0) {
    fprintf(stderr, "Bad size: %d\n", size);
    exit(1);
  }

  fd = open("/dev/uio1", O_RDWR);
  if (fd < 0) {
    perror("Failed to open devfile");
    return 1;
  }

  map_addr = mmap( NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  if (map_addr == MAP_FAILED) {
    perror("Failed to mmap");
    return 1;
  }

  mapped = map_addr;
  err = ((mapped[0]>>16)&0xFF)+1;
  printf("mapped, first address offset = %d\n", err);
  if (INPUT_OFFSET!=err) {
	  printf("DexRun.c - XILLYDEMO.BIT mismatch error! INPUT_OFFSET:%d\n",INPUT_OFFSET);
	  OverError = ERROR_INPUT_OFFSET;
	  mapped = shadow_map;
	 mapped[CMD_FIFO_STATE]=shadow_map[CMD_FIFO_STATE]= 0x2; //required so checking the FIFO doesn't hang us
	  //return ERROR_INPUT_OFFSET; //don't need to stop, we've made a fake memory map.
  }
  
  mfd = open("/dev/mem", O_RDWR);
  map_addrCt = mmap(NULL, CalTblSize, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, 0x3f000000);


   if (map_addrCt == MAP_FAILED) {
		//printf("\nCant Map Callibration Tables \n");
    perror("Failed to mmap High memory");
    return 1;
  }
  CalTables = map_addrCt;

// Load AxisCal data into JointsCal array
	int HexValue;
	FILE *AxisFile;
	AxisFile = fopen("AxisCal.txt", "rs");
	if(AxisFile!=NULL)
	{
		fscanf(AxisFile, "%f", &JointsCal[0]);
		fscanf(AxisFile, "%f", &JointsCal[1]);
		fscanf(AxisFile, "%f", &JointsCal[2]);
		fscanf(AxisFile, "%f", &JointsCal[3]);
		fscanf(AxisFile, "%f", &JointsCal[4]);
		fscanf(AxisFile, "%i", &HexValue);
        //printf("Reading AxisCal.txt. HexValue = %d\n", HexValue);
		mapped[ANGLE_END_RATIO]=HexValue;//((LG_RADIUS/SM_RADIUS * MOTOR_STEPS * MICRO_STEP)/(MOTOR_STEPS*GEAR_RATIO*MICRO_STEP))*2^24
		fclose(AxisFile);
	}


    //Start Position code:
    int reset_StartPosition = 1; // 1 = reset, 0 = do not reset

	wfp = fopen("/srv/samba/share/StartPosition_CommandedAngles.txt", "r");

	if (wfp) {
		printf("CommandedAngles Start Positions: Loaded %d. Values: ", fscanf(wfp, "[%i, %i, %i, %i, %i]", &SP_CommandedAngles[0], &SP_CommandedAngles[1], &SP_CommandedAngles[2], &SP_CommandedAngles[3], &SP_CommandedAngles[4]));
		printf("[%i, %i, %i, %i, %i]\n", SP_CommandedAngles[0], SP_CommandedAngles[1], SP_CommandedAngles[2], SP_CommandedAngles[3], SP_CommandedAngles[4]);
		fclose(wfp);
		wfp = 0;
    }else {
        printf("Failed to load StartPosition_CommandedAngles.txt Error # %d\n", errno);
        reset_StartPosition = 0;
    }

	
    wfp = fopen("/srv/samba/share/StartPosition_EyeNumbers.txt", "r");
	if (wfp) {
		printf("EyeNumbers Start Positions: Loaded %d. Values: ", fscanf(wfp, "[%i, %i, %i, %i, %i]", &SP_EyeNumbers[0], &SP_EyeNumbers[1], &SP_EyeNumbers[2], &SP_EyeNumbers[3], &SP_EyeNumbers[4]));
		printf("[%i, %i, %i, %i, %i]\n", SP_EyeNumbers[0], SP_EyeNumbers[1], SP_EyeNumbers[2], SP_EyeNumbers[3], SP_EyeNumbers[4]);
		fclose(wfp);
		wfp = 0;
	}else {
        printf("Failed to load StartPosition_EyeNumbers.txt Error # %d\n", errno);
        reset_StartPosition = 0;
    }

    if(reset_StartPosition){
        printf("Resetting StartPosition\n");
        sprintf(iString, "S, EyeNumbers, %d, %d, %d, %d, %d;", SP_EyeNumbers[0], SP_EyeNumbers[1], SP_EyeNumbers[2], SP_EyeNumbers[3], SP_EyeNumbers[4]);
        ParseInput(iString);
        //sprintf(iString, "F;");
        //ParseInput(iString);
        sprintf(iString, "w, 42, 256;");
        ParseInput(iString);
        sprintf(iString, "w, 42, 0;");
        ParseInput(iString);
        sprintf(iString, "S, CommandedAngles, %d, %d, %d, %d, %d;", SP_CommandedAngles[0], SP_CommandedAngles[1], SP_CommandedAngles[2], SP_CommandedAngles[3], SP_CommandedAngles[4]);
        ParseInput(iString);
    }else{
        printf("Not resetting StartPosition: Could not find files.\n");
    }

//  Addr= = atoi(argv[3]);
//  Dta= = atoi(argv[4]);
//mapped[Addr]=shadow_map[Addr]= Dta;

	setDefaults(DefaultMode);
	strlcpy(iString, "S RunFile autoexec.make_ins ;\0", ISTRING_LEN); //start running default instructions
	printf("Starting %s returned %d\n",iString, ParseInput(iString));
	
//	if(DefaultMode ==2 )
	if(ServerMode==1)
	{
		printf("\nStarting raw socket");	
		err = pthread_create(&(tid[0]), NULL, &StartServerSocket,  (void*)&ThreadsExit);
		if (err != 0)
		{
			printf("\ncan't create thread :[%s]", strerror(err));
			return 1;
		}
		else
		{
			printf("\n Begin Socket Server Thread\n");
		}
	}
	else if(ServerMode==2)
	{
		printf("\nStarting client socket");
		err = pthread_create(&(tid[1]), NULL, &StartClientSocket, NULL);
		if (err != 0)
		{
			printf("\ncan't create thread :[%s]", strerror(err));
			return 1;
		}
		else
		{
			printf("\n Begin Socket Client Thread\n");
		}
	}
	else if(ServerMode==3)
	{
		printf("Starting DDE socket\n");
		if(mapped[SENT_BASE_POSITION]!=0)
		{
	   		munmap(map_addr, size);
			munmap(map_addrCt, CalTblSize);
 			close(fd);
			close(mfd);
			printf("Sent base position NOT zero\n");
			return 0;   
		}
		//mapped[BASE_POSITION]=shadow_map[BASE_POSITION]=1;
    	#ifndef NO_BOOT_DANCE
		
		printf("Boot Dance\n");
		/*
		//Wigglesworth Code Start
		int DEFAULT_MAXSPEED = 232642; // 30 (deg/s)
		int DEFAULT_STARTSPEED = 512; // .066 (deg/s) This is the smallest number allowed
		
		//Maxspeed
		mapped[ACCELERATION_MAXSPEED]=shadow_map[ACCELERATION_MAXSPEED]=DEFAULT_MAXSPEED;
		maxSpeed=(DEFAULT_MAXSPEED) & 0b00000000000011111111111111111111;
		coupledAcceleration=((DEFAULT_MAXSPEED) & 0b00000011111100000000000000000000) >> 20;
		
		//Startspeed
		mapped[START_SPEED]=shadow_map[START_SPEED]=1 ^ DEFAULT_STARTSPEED;
		//Wigglesworth Code End
		*/

		/*
		MoveRobot(0,0,0,50000,50000,BLOCKING_MOVE);
	    MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
	    MoveRobot(0,0,0,50000,-50000,BLOCKING_MOVE);
	    MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
	    MoveRobot(0,0,0,50000,50000,BLOCKING_MOVE);
	    MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
	    MoveRobot(0,0,0,50000,-50000,BLOCKING_MOVE);
	    MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
		*/

		strlcpy(iString, "S RunFile BootDance_ServerMode3.make_ins ;\0", ISTRING_LEN);
		printf("Starting %s returned %d\n",iString, ParseInput(iString));
		char *token;
		int i = 0;
		int ip_last = 0;
		float ip_sleep = 1.0;
		int ip_a = 0;
		int ip_b = 0;
		int ip_c = 0;
		const char delimiters[] = " .\t";

		printf("IP Dance\n");
		wfp = fopen("/etc/network/interfaces", "r");
		while(fgets(iString, ISTRING_LEN, wfp) != NULL && i < 20) {
			if((strstr(iString, "address 192.168.")) != NULL) {
				token = strtok(iString, delimiters);
				token = strtok(NULL, delimiters);
				token = strtok(NULL, delimiters);
				token = strtok(NULL, delimiters);
				token = strtok(NULL, delimiters);

				ip_last = atoi(token);
				printf("Last feild found: %i\n", ip_last);

				ip_a = (int)floorf((float)ip_last/100.0);
				printf("ip_a: %i\n", ip_a);
				ip_b = (int)floorf((float)(ip_last-ip_a*100)/10.0);
				printf("ip_a: %i\n", ip_b);
				ip_c = ip_last-ip_b*10-ip_a*100;
				printf("ip_a: %i\n", ip_c);


				strlcpy(iString, "S RunFile BootDance_IP_0.make_ins ;\0", ISTRING_LEN);
    			printf("Starting %s returned %d\n",iString, ParseInput(iString));
				for(i = 0; i < ip_a; i++){
					strlcpy(iString, "S RunFile BootDance_IP_a.make_ins ;\0", ISTRING_LEN);
    				printf("Starting %s returned %d\n",iString, ParseInput(iString));
				}
				sleep(0.5*ip_sleep);
				strlcpy(iString, "S RunFile BootDance_IP_0.make_ins ;\0", ISTRING_LEN);
    			printf("Starting %s returned %d\n",iString, ParseInput(iString));
				sleep(ip_sleep);
				for(i = 0; i < ip_b; i++){
					strlcpy(iString, "S RunFile BootDance_IP_b.make_ins ;\0", ISTRING_LEN);
    				printf("Starting %s returned %d\n",iString, ParseInput(iString));
				}
				sleep(0.5*ip_sleep);
				strlcpy(iString, "S RunFile BootDance_IP_0.make_ins ;\0", ISTRING_LEN);
    			printf("Starting %s returned %d\n",iString, ParseInput(iString));
				sleep(ip_sleep);
				for(i = 0; i < ip_c; i++){
					strlcpy(iString, "S RunFile BootDance_IP_c.make_ins ;\0", ISTRING_LEN);
    				printf("Starting %s returned %d\n",iString, ParseInput(iString));
				}
				sleep(0.5*ip_sleep);
				strlcpy(iString, "S RunFile BootDance_IP_0.make_ins ;\0", ISTRING_LEN);
    			printf("Starting %s returned %d\n",iString, ParseInput(iString));


				//printf("First: %i, Second: %i, Third: %i\n", ip_a, ip_b, ip_c);

				
				//for(i = 0; i < 5; i++){
				//	token = strtok(NULL, delimiters);
				//	printf("%s\n", token);
				//}

				break;

				//fscanf(iString, "address 192.168.1.%i\n", ip_last);
				//printf("last feild of IP: %i\n", ip_last);

			}
			i++;
		}

		if (wfp>0) {fclose(wfp); wfp = 0;}
		/*
		wfp = fopen("/etc/network/interfaces", "r");
		token = strtok ((char *)wfp, delimiters); 
		printf("First Token: %s\n", token);
		for(i = 0; i < 30; i++){
			token = strtok (NULL, delimiters);
			printf("%s\n", token);
		}
		*/
		/*
		while(strcmp(token, "address") != 0){
			token = strtok (NULL, delimiters);
			printf("%s\n", token);
		}
		token = strtok (NULL, delimiters); 

		printf("\nip_address found: %s\n", token);
		*/

	    #endif	
		printf("Start thread\n");
		err = pthread_create(&(tid[0]), NULL, &StartServerSocketDDE,  (void*)&ThreadsExit);
		if (err != 0)
		{
			printf("can't create thread :[%s]\n", strerror(err));
			return 1;
		}

	}
	if(RunMode==1 || RunMode==2) //3rd argument. E.g. DexRun 1 3 1 or 1 3 2
	{
		printf("Start realtime monitor thread\n");
		err = pthread_create(&(tid[2]), NULL, &RealtimeMonitor, (void*)&ThreadsExit );
		if (err != 0)
		{
			printf("\ncan't create thread :[%s]\n", strerror(err));
			return 1;
		}	
	}

	
	
    if(ServerMode==3)
	{
		printf("Going to sleep\n");
		while(1){sleep(1);} //loop forever TODO: Add a sleep in this loop
	}
	while(getInput()==0);
	ThreadsExit=0;
	sleep(1);


	printf("\nExiting \n");

    munmap(map_addr, size);
    munmap(map_addrCt, CalTblSize);

    close(fd);
    close(mfd);
    printf("End of main()\n");
	return 0;
}
