//#define NO_BOOT_DANCE


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


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


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




#define INPUT_OFFSET 11

   // Motor position index
#define BASE_POSITION 0
#define END_POSITON 1
#define PIVOT_POSITON 2
#define ANGLE_POSITON 3
#define ROT_POSITON 4

#define ACCELERATION_MAXSPEED 5

// DC bias paramaters
#define BASE_SIN_CENTER 6
#define BASE_COS_CENTER 7
#define END_SIN_CENTER 8
#define END_COS_CENTER 9
#define PIVOT_SIN_CENTER 10
#define PIVOT_COS_CENTER 11
#define ANGLE_SIN_CENTER 12
#define ANGLE_COS_CENTER 13
#define ROT_SIN_CENTER 14
#define ROT_COS_CENTER 15

//PID VALUES
#define PID_DELTATNOT 16
#define PID_DELTAT 17
#define PID_D 18
#define PID_I 19
#define PID_P 20
#define PID_ADDRESS 21

//FORCE PARAMATERS
#define BOUNDRY_BASE 22
#define BOUNDRY_END 23
#define BOUNDRY_PIVOT 24

#define BOUNDRY_ANGLE 25
#define BOUNDRY_ROT 26

#define SPEED_FACTORA 27
#define SPEED_FACTORB 28

#define FRICTION_BASE 29
#define FRICTION_END 30
#define FRICTION_PIVOT 31
#define FRICTION_ANGLE 32
#define FRICTION_ROT 33

#define MOVE_TRHESHOLD 34
#define F_FACTOR 35
#define MAX_ERROR 36

#define FORCE_BIAS_BASE 37
#define FORCE_BIAS_END 38
#define FORCE_BIAS_PIVOT 39
#define FORCE_BIAS_ANGLE 40
#define FORCE_BIAS_ROT 41

// control state register
#define COMMAND_REG 42
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







//DMA 
#define DMA_READ_ADDRESS 48
#define DMA_READ_PARAMS 47
#define DMA_WRITE_ADDRESS 46
#define DMA_WRITE_PARAMS 45
#define DMA_WRITE_DATA 44
#define DMA_CONTROL 43

// DMA control bits breakdown

#define DMA_WRITE_ENQUEUE 1
#define DMA_WRITE_INITIATE 2
#define DMA_READ_DEQUEUE 4
#define DMA_READ_BLOCK 8
#define DMA_RESET_ALL 16

#define REC_PLAY_CMD 49
#define CMD_RESET_RECORD 1
#define CMD_RECORD 2
#define CMD_RESET_PLAY 4
#define CMD_PLAYBACK 8
#define CMD_RESET_PLAY_POSITION 16



#define REC_PLAY_TIMEBASE 50
#define DIFF_FORCE_TIMEBASE 51
#define DIFF_FORCE_BETA 52
#define DIFF_FORCE_MOVE_THRESHOLD 53
#define DIFF_FORCE_MAX_SPEED 54
#define DIFF_FORCE_SPEED_FACTOR_ANGLE 55
#define DIFF_FORCE_SPEED_FACTOR_ROT 56
#define DIFF_FORCE_ANGLE_COMPENSATE 57

#define FINE_ADJUST_BASE 58
#define FINE_ADJUST_END 59
#define FINE_ADJUST_PIVOT 60
#define FINE_ADJUST_ANGLE 61
#define FINE_ADJUST_ROT 62

#define RECORD_LENGTH 63



#define END_EFFECTOR_IO 64
#define SERVO_SETPOINT_A 65
#define SERVO_SETPOINT_B 66

#define BASE_FORCE_DECAY 67
#define END_FORCE_DECAY 68
#define PIVOT_FORCE_DECAY 69
#define ANGLE_FORCE_DECAY 70
#define ROTATE_FORCE_DECAY 71

#define PID_SCHEDULE_INDEX 72

#define GRIPPER_MOTOR_CONTROL 73
#define GRIPPER_MOTOR_ON_WIDTH 75
#define GRIPPER_MOTOR_OFF_WIDTH 74
#define START_SPEED 76
#define ANGLE_END_RATIO 77





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


#define bool int
#define TRUE 1
#define FALSE 0















//////////////////////////////////////////////////////////////////////////
/* Start Wigglesworth Code*/



//#include <cstdlib> //C++ header
#include <math.h> // for trig and floor functions
//#include <iostream> //doesn't exist on Dexter
//#include <limits> // for intentionaly returning nan 
//#include <time.h> /* c//lock_t, clock, CLOCKS_PER_SEC */
//#include <stdlib.h> //defined above
//#include <stdbool.h>


// Vector Library:
#define PI (3.141592653589793)
//double L[5] = { 0.1651, 0.320675, 0.3302, 0.0508, 0.08255 }; // (meters)
double L[5] = { 165100, 320675, 330200, 50800, 82550 }; // (microns)
struct Vector {
	double x, y, z;
};


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
		return 180;
	}
	else {
		return atan2_arcsec(magnitude(cross(v1, v2)), dot(v1, v2));
	}
}

double signed_angle(struct Vector v1, struct Vector v2, struct Vector plane) {
	double guess_angle = angle(v1, v2);
	long double epsilon = 0.0000000001; // 1e-10
	if (abs(guess_angle) <  epsilon || abs(abs(guess_angle) - 180) < epsilon) {
		return round(guess_angle); // Will rounding increase or decrease error? 
	}

	struct Vector c_prod = normalize(cross(v1, v2));
	if (is_equal(c_prod, plane, 10)) {
		return guess_angle;
	}
	else if (is_equal(negate(c_prod), plane, 10)){
		return -guess_angle;
	}
	else {
		printf("\nError: singled_angle wants to return NaN:");
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
		printf("Int]");
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
	printf("{ ");
	print_vector(a.position);
	printf(", ");
	print_vector(a.direction);
	printf(", ");
	print_config(a.config);
	printf(" }");
}


// Forward Kinematics:
struct XYZ J_angles_to_xyz(struct J_angles angles) {

	/*
	// Pre allocation:
	struct Vector U[6] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	struct Vector V[5] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },{ 0, 0, 0 }};
	struct Vector P[3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }};

	P[0][0] = 1; // Datam Plane = [1 0 0]
	V[0][2] = 1; // Vector for Link 1 (base)

	P[1] = rotate(P[0], V[0], -(angles.J1 - 180*3600)); // Links 2, 3 and 4 lie in P1
	V[1] = rotate(V[0], P[1], angles.J2);		   // Vector for Link 2
	V[2] = rotate(V[1], P[1], angles.J3);		   // Vector for Link 3
	V[3] = rotate(V[2], P[1], angles.J4);		   // Vector for Link 4
	P[2] = rotate(P[1], V[3], -(angles.J5 - 180*3600));	   // Link 4 and 5 lie in P2
	V[4] = rotate(V[3], P[2], -90*3600);				   // Vector for Link 5 (90 degree bend)


												   // Next point = current point + Link length * vector direction
	int i;
	for (i = 0; i < 5; i++) {
		U[i + 1] = add(U[i], scalar_mult(L[i], V[i]));
	}
	*/
	
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
	//struct Vector P2 = {0, 0, 0};
	

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
		if (dist_a < dist_b) {
			my_config.wrist_out = 1; // Wrist out
		}
		else {
			my_config.wrist_out = 0; // Wrist in
		}
	}
	else {
		if (dist_a < dist_b) {
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


	
	//debugging:
	/*
	printf("\nU:\n");
	struct Vector temp_vector;
	for (int i = 0; i < sizeof(U) / 24; i++) {
		printf("i:%d = ", i);
		temp_vector = U[i];
		print_vector(temp_vector);
		printf("\n");
	}
	printf("\nV:\n");
	for (int i = 0; i < sizeof(V) / 24; i++) {
		printf("i:%d = ", i);
		temp_vector = V[i];
		print_vector(temp_vector);
		printf("\n");
	}
	printf("\nP:\n");
	for (int i = 0; i < sizeof(P) / 24; i++) {
		printf("i:%d = ", i);
		temp_vector = P[i];
		print_vector(temp_vector);
		printf("\n");
	}
	*/
	


	struct XYZ result = new_XYZ(U5, my_dir, my_config);
	return result;
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
		printf("\n\nOut of Reach Error at position: ");
		print_XYZ(xyz);
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
		J.J1 = signed_angle(P1, P0, V0) + 180;
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
	struct Vector EE_point_1 = J_angles_to_xyz(J_angles_old).position;
	struct Vector EE_point_2 = J_angles_to_xyz(J_angles_old).position;
	double dist_EE = dist_point_to_point(EE_point_1, EE_point_2);
	if(dist_EE == 0){
		return 3877; //30 (deg/s)
	}
	double max_theta = J_angles_max_diff(J_angles_old, J_angles_new);
	
	return (int)(max_theta*cart_speed/dist_EE);
}
		









/* End Wigglesworth Code*/
//////////////////////////////////////////////////////////////////////////







FILE *wfp; //File handle to write data into via socket 'W' command
int XLowBound[5]={BASE_COS_LOW,END_COS_LOW,PIVOT_COS_LOW,ANGLE_COS_LOW,ROT_COS_LOW};
int XHighBound[5]={BASE_COS_HIGH,END_COS_HIGH,PIVOT_COS_HIGH,ANGLE_COS_HIGH,ROT_COS_HIGH};
int YLowBound[5]={BASE_SIN_LOW,END_SIN_LOW,PIVOT_SIN_LOW,ANGLE_SIN_LOW,ROT_SIN_LOW};
int YHighBound[5]={BASE_SIN_HIGH,END_SIN_HIGH,PIVOT_SIN_HIGH,ANGLE_SIN_HIGH,ROT_SIN_HIGH};
int ForcePossition[5]={0,0,0,0,0};
int ForceDestination[5]={0,0,0,0,0};
int ThreadsExit=1;
int StatusReportIndirection[60]={DMA_READ_DATA,DMA_READ_DATA,RECORD_BLOCK_SIZE,END_EFFECTOR_IO_IN,BASE_POSITION_AT,BASE_POSITION_DELTA,
								BASE_POSITION_PID_DELTA,BASE_POSITION_FORCE_DELTA,BASE_SIN,BASE_COS,PLAYBACK_BASE_POSITION,SENT_BASE_POSITION,
								SLOPE_BASE_POSITION,0,PIVOT_POSITION_AT,PIVOT_POSITION_DELTA,PIVOT_POSITION_PID_DELTA,PIVOT_POSITION_FORCE_DELTA,
								PIVOT_SIN,PIVOT_COS,PLAYBACK_PIVOT_POSITION,SENT_PIVOT_POSITION,SLOPE_PIVOT_POSITION,0,END_POSITION_AT,
								END_POSITION_DELTA,END_POSITION_PID_DELTA,END_POSITION_FORCE_DELTA,END_SIN,END_COS,PLAYBACK_END_POSITION,
								SENT_END_POSITION,SLOPE_END_POSITION,0,ANGLE_POSITION_AT,ANGLE_POSITION_DELTA,ANGLE_POSITION_PID_DELTA,
								ANGLE_POSITION_FORCE_DELTA,ANGLE_SIN,ANGLE_COS,PLAYBACK_ANGLE_POSITION,SENT_ANGLE_POSITION,SLOPE_ANGLE_POSITION,
								0,ROT_POSITION_AT,ROT_POSITION_DELTA,ROT_POSITION_PID_DELTA,ROT_POSITION_FORCE_DELTA,ROT_SIN,ROT_COS,
								PLAYBACK_ROT_POSITION,SENT_ROT_POSITION,SLOPE_ROT_POSITION,0};

void *map_addr,*map_addrCt;
volatile unsigned int *mapped;
volatile unsigned int *CalTables;
volatile unsigned int *RecordTable;
int CmdVal=0,maxSpeed=0,startSpeed=0,coupledAcceleration=0,forceMode=0,fa0=0,fa1=0,fa2=0,fa3=0,fa4=0,RunMode=0,ServerMode=3;
int BaseBoundryHigh,BaseBoundryLow,EndBoundryHigh,EndBoundryLow,PivotBoundryHigh,PivotBoundryLow,AngleBoundryHigh=DEFAULT_ANGLE_BOUNDRY_HI,AngleBoundryLow=DEFAULT_ANGLE_BOUNDRY_LOW,RotateBoundryHigh,RotateBoundryLow;
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
float DiffCorrectionFactor;
int AngleHIBoundry;
int AngleLOWBoundry;
int controlled=0;

int DexError=0;
float fLastTime=0;

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
#define MAX_PARAMS 28
#define PARAM_LENGTH 20
char Params[MAX_PARAMS][PARAM_LENGTH+1] = {"MaxSpeed", "Acceleration", "J1Force","J3Force","J2Force","J4Force","J5Force","J1Friction","J3Friction","J2Friction","J4Friction","J5Friction","J1BoundryHigh","J1BoundryLow","J3BoundryHigh","J3BoundryLow","J2BoundryHigh","J2BoundryLow","J4BoundryHigh","J4BoundryLow","J5BoundryHigh","J5BoundryLow","GripperMotor","EERoll","EESpan","StartSpeed","EndSpeed","End"};


static struct termios old, new;

int PositionAdjust[5]={0,0,0,0,0};

int ForceLimit[5]={4400,4400,4400,800,800};

int MyBotForce[5]={0,0,0,0,0};

float ForceAdjustPossition[5]={-.01,-.01,-.01,-.01,-.01};
int FroceMoveMode=0;

int LastGoal[5]={0,0,0,0,0};


float JointsCal[5];
int SoftBoundries[5];

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

void printPosition()
{
	int a1,a2,a3,a4,a5;
	a1=getNormalizedInput(BASE_POSITION_AT)+getNormalizedInput(BASE_POSITION_FORCE_DELTA);
	a2=getNormalizedInput(END_POSITION_AT)+getNormalizedInput(END_POSITION_FORCE_DELTA);
	a3=getNormalizedInput(PIVOT_POSITION_AT)+getNormalizedInput(PIVOT_POSITION_FORCE_DELTA);
	a4=getNormalizedInput(ANGLE_POSITION_AT)+getNormalizedInput(ANGLE_POSITION_FORCE_DELTA);
	a5=getNormalizedInput(ROT_POSITION_AT)+getNormalizedInput(ROT_POSITION_FORCE_DELTA);
	//printf(" %d,%d,%d,%d,%d\n",a1,a2,a3,a4,a5);

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
	while(*ExitState)
	{
		if(FroceMoveMode==1)
		{
	
	
			
			// do force based movement
			for(i=0;i<5;i++)
			{
				ForceDelta=ForcePossition[i]-getNormalizedInput(BASE_POSITION_FORCE_DELTA+i);
				/*if(abs(ForceDelta)>500)
				{
					ForceDelta=(abs(ForceDelta)-500)*sign(ForceDelta);
				}*/
				//ForceDelta=ForceDelta;//+MyBotForce[i];
				if(disTime==90)
				{
					//for(j=0;j<5;j++)
					//printf(" Force %d ",ForceDelta);
				}
				if(abs(ForceDelta)<ForceLimit[i])
				{
					mapped[FORCE_BIAS_BASE+i]=ForceDelta;
				}
				else
				{
					mapped[FORCE_BIAS_BASE+i]=sign(ForceDelta)*ForceLimit[i];
				}
				//ForcePossition[i]=ForcePossition[i]+(int)((float)ForceDelta*ForceAdjustPossition[i]);
				/*if(ForceDestination[i]>ForcePossition[i])
				{
					ForcePossition[i]=ForcePossition[i]+10;
				}
				else
				{
					ForcePossition[i]=ForcePossition[i]-10;
				}*/
			}
		}
		disTime++;
		if(disTime>100)
		{
			//printPosition();
			disTime = 0;
		}

		usleep(1000);
	}
	//printf("\nMonitor Thread Exiting\n");
    return NULL;
}

void SetGripperRoll(int Possition)
{
	int ServoSpan=(SERVO_HI_BOUND-SERVO_LOW_BOUND)/360;
	mapped[END_EFFECTOR_IO]=80;

	mapped[SERVO_SETPOINT_B]=(ServoSpan*Possition)+SERVO_LOW_BOUND;
//	mapped[SERVO_SETPOINT_B]=Possition;
	
}void SetGripperSpan(int Possition)
{
	int ServoSpan=(SERVO_HI_BOUND-SERVO_LOW_BOUND)/360;
	mapped[END_EFFECTOR_IO]=80;

	mapped[SERVO_SETPOINT_A]=(ServoSpan*Possition)+SERVO_LOW_BOUND;
//	mapped[SERVO_SETPOINT_B]=Possition;
	
}
void SetGripperMotor(int state)
{
	mapped[GRIPPER_MOTOR_ON_WIDTH]=12000;
	mapped[GRIPPER_MOTOR_OFF_WIDTH]=0;
	mapped[GRIPPER_MOTOR_CONTROL]=state;
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
					(void)ProcessServerReceiveDataDDE(recBuff);
				
					(void)ProcessServerSendDataDDE(sendBuff,recBuff);/*==TRUE)*/
						write (connfd,sendBuff,60*4/*sizeof(sendBuff)*/); 
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
		{
			mapped[FORCE_BIAS_BASE]=MaxForce(MxForce,(int)((float)MyBot.baseForce)*fScale);
			mapped[FORCE_BIAS_END]=MaxForce(MxForce,(int)((float)MyBot.endForce)*fScale);
			mapped[FORCE_BIAS_PIVOT]=MaxForce(MxForce,(int)((float)MyBot.pivotForce)*fScale);
			mapped[FORCE_BIAS_ANGLE]=MaxForce(MxForce,(int)((float)MyBot.angleForce)*fScale);
			mapped[FORCE_BIAS_ROT]=MaxForce(MxForce,(int)((float)MyBot.rotateForce)*fScale);
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
	int i;
	int *sendBuffReTyped;
	const char delimiters[] = " ,";
	char *token;
	float timeStart=0;
    float timeNormal = 1494640000000;
	long iTimeNormal = 1494628400;
	int iTime=0;
	int iElTime=0;
	long            ms; // Millisecondski
    time_t          s;  // Seconds
	long lTime;
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);

    s  = spec.tv_sec;

    ms = spec.tv_nsec / 1.0e3; // Convert nanoseconds to milliseconds
	lTime = (long)s;
	timeStart = (float)(lTime - iTimeNormal);
	timeStart = timeStart + (float)ms/1000000.0;
	iElTime=timeStart*1000;
	
	
	
    //printf("\n %li %li %li",spec.tv_nsec ,spec.tv_sec, iElTime);
	//fLastTime = timeStart;
	
/*	for(i=0;i<25;i++)
	{
		//printf("%c",recBuff[i]);
	}*/
//	if(recBuff[0]=="g")
	

	token = strtok (recBuff, delimiters);

	{
		////printf("returning heartbeat\n");
		sendBuffReTyped=(int *)sendBuff;
		sendBuffReTyped[0]=atoi(token);
		token = strtok (NULL, delimiters);
		//printf("\n %s \n",token);
		sendBuffReTyped[1]=atoi(token);
		token = strtok (NULL, delimiters);
		//printf("\n %s \n",token);
		timeStart = atof(token) - timeNormal;
		iTime = timeStart;

		sendBuffReTyped[2] = spec.tv_sec;//iTime;//   this shoud be start_time_internal broken into 2 words atoi(token);
		token = strtok (NULL, delimiters);
		//printf("\n %s \n",token);
		token = strtok (NULL, delimiters);
		
		sendBuffReTyped[3]=(int)(spec.tv_nsec  / 1.0e3);//iElTime;//   this shoud be start_time_internal broken into 2 words atoi(token);
		sendBuffReTyped[4]=token[0];
		sendBuffReTyped[5]=DexError;



		//sendBuffReTyped[1]=token[0];
		//sendBuffReTyped[2]=DexError;
		
		for(i=0;i<59;i++)
		{
			sendBuffReTyped[i+6]=getNormalizedInput(StatusReportIndirection[i]);
			/*if(StatusReportIndirection[i] == BASE_POSITION_PID_DELTA)
			{
				//printf(" PID Delta for L1 = %d", sendBuffReTyped[i+6]);
			}
			if(StatusReportIndirection[i] == BASE_POSITION_DELTA)
			{
				//printf(" TABLE OFFSET Delta for L1 = %d", sendBuffReTyped[i+6]);
			}*/
		}
		return TRUE;
	}
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
	
	//memcpy(&MyBot,recBuff,sizeof(MyBot));
	////printf("Server %d %d %d %d %d \n",MyBot.base,MyBot.end,MyBot.pivot,MyBot.angle,MyBot.rotate);
	//mapped[FINE_ADJUST_BASE]=MyBot.base;
	//mapped[FINE_ADJUST_END]=MyBot.end;
	//mapped[FINE_ADJUST_PIVOT]=MyBot.pivot;
	//mapped[FINE_ADJUST_ANGLE]=MyBot.angle;
	//mapped[FINE_ADJUST_ROT]=MyBot.rotate;
	
	//MoveRobot(MyBot.base,MyBot.end,MyBot.pivot,MyBot.angle,MyBot.rotate,BLOCKING_MOVE);
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
	//printf("\n%s \n",CmdString);
	DexError=ParseInput(CmdString); 
////printf("\nsent parse\n"); 
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
        //printf("\n inet_pton error occured\n");
        return &"inet_pton failed";
    } 
    if( connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
       //printf("\n Error : Connect Failed \n");
       return &"connect failed";
    } 
//	//printf("\n Socket connect\n");
//	while(1)//for(j=0;j<1000;j++)
	{
		////printf("\n next\n");
		while( (n = recv(sockfd, recvBuff, sizeof(recvBuff),0 ) ) > 0)
		{
			//recvBuff[n] = 0;
			////printf("Socket Read\n");
			////printf("%s ",recvBuff);

			if(n < 0)
			{
				//printf("\n Read error \n");
			} 
			/*mapped[FORCE_BIAS_BASE]=(int)(((float)MyBot.base)*ForceScale);
			mapped[FORCE_BIAS_END]=(int)(((float)MyBot.end)*ForceScale);
			mapped[FORCE_BIAS_PIVOT]=(int)(((float)MyBot.pivot)*ForceScale);
			mapped[FORCE_BIAS_ANGLE]=(int)(((float)MyBot.angle)*ForceScale);
			mapped[FORCE_BIAS_ROT]=(int)(((float)MyBot.rotate)*ForceScale);*/
			////printf("%d %d %d %d %d \n",MyBot.base,MyBot.end,MyBot.pivot,MyBot.angle,MyBot.rotate);
#ifdef FOLLOW_POSSITION
			memcpy(&MyBot,recvBuff,sizeof(MyBot));
			DiffDelta=1.5;
			MyBot.base=SlaveBotPos.base + ((GetAxisCurrent(0) - SetHome.base)/MSMoveRatio.base);
			MyBot.end=SlaveBotPos.end + ((GetAxisCurrent(1) - SetHome.end)/MSMoveRatio.end);
			MyBot.pivot=SlaveBotPos.pivot + ((GetAxisCurrent(2) - SetHome.pivot)/MSMoveRatio.pivot);
			MyBot.angle=SlaveBotPos.angle + (((GetAxisCurrent(3) - SetHome.angle)/MSMoveRatio.angle)*DiffDelta);
			MyBot.rotate=SlaveBotPos.rotate + (((GetAxisCurrent(4) - SetHome.rotate)/MSMoveRatio.rotate)*DiffDelta);
/*			MyBot.base=0;
			MyBot.end=0;
			MyBot.pivot=0;
			MyBot.angle=0;
			MyBot.rotate=0;*/

			/*
			SlaveBotPos.base=MyBot.base;
			SlaveBotPos.end=MyBot.base;
			SlaveBotPos.pivot=MyBot.base;
			SlaveBotPos.angle=MyBot.base;
			SlaveBotPos.rotate=MyBot.base;
			*/
			
			//usleep(25000);

			/*MyBot.base=BASE_POSITION_FORCE_DELTA;
			MyBot.end=12300;
			MyBot.pivot=5123;
			MyBot.angle=10;
			MyBot.rotate=15;*/
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
			mapped[Axis]=Start+k;
		else	
			mapped[Axis]=Start-k;
		for(j=0;j<Delay;j++)
		{	
			ADVal=mapped[ADLookUp[Axis]];
			AvgSIN=AvgSIN+ADVal;
			AvgCOS=AvgCOS+mapped[ADLookUp[Axis]+1];
		}	
		AvgSIN=AvgSIN/Delay;
		AvgCOS=AvgCOS/Delay;
		////printf("\n SIN %x COS %x",AvgSIN,AvgCOS);
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
void* CapturePoints(void *arg)
{
    unsigned long i = 0;
	int a1,a2,a3,a4,a5,f1,f2,f3,f4,f5,BGM=10,EmbedRec=1,RecState=0,Length=0,GripperToggle=0,ServoSet=180;
	const char delimiters[] = ".";

	char c,*FileNameBase,*FileNameExt,RecordFilename[256],iString[256];
      char tmpSt[(8 * sizeof(EmbedRec) / 3) + 3]; //safe bet for the number of decimal digits
//    pthread_t id = pthread_self();
	initTermios(0);
	f1=0;
	mapped[FINE_ADJUST_BASE]=f1;
	f3 =0;
	mapped[FINE_ADJUST_END]=f3;
	f2=0;
	mapped[FINE_ADJUST_PIVOT]=f2;
	f4=0;
	mapped[FINE_ADJUST_ANGLE]=f4;
	f5=0;
	mapped[FINE_ADJUST_ROT]=f5;
	ForcePossition[0]=0;
	ForcePossition[1]=0;
	ForcePossition[2]=0;
	ForcePossition[3]=0;
	ForcePossition[4]=0;


	mapped[COMMAND_REG]=12448;
	//printf("/n Thread running");
	if(RunMode==1)
		forceMode=3;
	c=' ';
	//mapped[MAX_ERROR]=(2000 ^ 6000);
	while(c!='k') // kill
	{
		mapped[FINE_ADJUST_BASE]=f1;
		mapped[FINE_ADJUST_END]=f3;
		mapped[FINE_ADJUST_PIVOT]=f2;
		mapped[FINE_ADJUST_ANGLE]=f4;
		mapped[FINE_ADJUST_ROT]=f5;
		c=getchar();
		if(c=='l')  // learn movement
		{
			if(RecState==0)
			{
				RecState=1;
				strcpy(iString,CptA.FileName);
				FileNameBase = strtok (iString, delimiters);
				//FileNameExt=strtok (NULL, delimiters);
				strcpy(RecordFilename,FileNameBase);
				strcat(RecordFilename,"ERM");
				snprintf(tmpSt, sizeof(tmpSt), "%d", EmbedRec++);
				strcat(RecordFilename,tmpSt);
				//free(tmpSt);
				strcat(RecordFilename,".dta");
				a1=getNormalizedInput(BASE_POSITION_AT)+getNormalizedInput(BASE_POSITION_FORCE_DELTA)+f1;
				a3=getNormalizedInput(END_POSITION_AT)+getNormalizedInput(END_POSITION_FORCE_DELTA)+f3;
				a2=getNormalizedInput(PIVOT_POSITION_AT)+getNormalizedInput(PIVOT_POSITION_FORCE_DELTA)+f2;
				a4=getNormalizedInput(ANGLE_POSITION_AT)+getNormalizedInput(ANGLE_POSITION_FORCE_DELTA)+f4;
				a5=getNormalizedInput(ROT_POSITION_AT)+getNormalizedInput(ROT_POSITION_FORCE_DELTA)+f5;
				fprintf(CptA.fp,"a %d,%d,%d,%d,%d\n",a1,a2,a3,a4,a5);
				//printf("\na %d,%d,%d,%d,%d",a1,a2,a3,a4,a5);
				fprintf(CptA.fp,"o %s\n",RecordFilename);
				mapped[REC_PLAY_TIMEBASE]=5;
				mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
				mapped[REC_PLAY_CMD]=0;				
				mapped[REC_PLAY_CMD]=CMD_RECORD; // start recording

				//InitCaptureMovement(RecordFilename);
			}
			else
			{
				RecState=0;
				a1=getNormalizedInput(BASE_POSITION_AT)+getNormalizedInput(BASE_POSITION_FORCE_DELTA)+f1;
				a3=getNormalizedInput(END_POSITION_AT)+getNormalizedInput(END_POSITION_FORCE_DELTA)+f3;
				a2=getNormalizedInput(PIVOT_POSITION_AT)+getNormalizedInput(PIVOT_POSITION_FORCE_DELTA)+f2;
				a4=getNormalizedInput(ANGLE_POSITION_AT)+getNormalizedInput(ANGLE_POSITION_FORCE_DELTA)+f4;
				a5=getNormalizedInput(ROT_POSITION_AT)+getNormalizedInput(ROT_POSITION_FORCE_DELTA)+f5;
				//f//printf(CptA.fp,"a %d,%d,%d,%d,%d\n",a1,a2,a3,a4,a5);
				//printf("\na %d,%d,%d,%d,%d",a1,a2,a3,a4,a5);
				//f//printf(CptA.fp,"o %s\n",RecordFilename);

				mapped[REC_PLAY_CMD]=0; // stop recording
				Length=mapped[RECORD_BLOCK_SIZE];
				ReadDMA(0x3f000000,Length,RecordFilename);
				mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
				mapped[REC_PLAY_CMD]=0;
			}
			
		}
		if(c=='h')
		{
			if(FroceMoveMode==0)
			{
				FroceMoveMode=1;
				//printf("/n force mode %d",FroceMoveMode);
			}
			else
			{
				FroceMoveMode=0;
				//printf("/n force mode %d",FroceMoveMode);
			}
		}
		if(c=='+')
		{
			BGM=BGM+10;
			SetNewBotRef();
			/*SetHome.base = SlaveBotPos.base;
			SetHome.end = SlaveBotPos.end;
			SetHome.pivot = SlaveBotPos.pivot;
			SetHome.angle = SlaveBotPos.angle;
			SetHome.rotate = SlaveBotPos.rotate;*/
			
			MSMoveRatio.base++;
			MSMoveRatio.end++;
			MSMoveRatio.pivot++;
			MSMoveRatio.angle++;
			MSMoveRatio.rotate++;
			//printf("\n big move %d MasterSlave Move ratio %d\n",BGM,MSMoveRatio.base);
		}
		if(c=='-')
		{
			BGM=BGM-10;
			if(BGM < 10)
			{
				BGM=10;
			}
			if(MSMoveRatio.base != 1)
			{
				/*SetHome.base = SlaveBotPos.base;
				SetHome.end = SlaveBotPos.end;
				SetHome.pivot = SlaveBotPos.pivot;
				SetHome.angle = SlaveBotPos.angle;
				SetHome.rotate = SlaveBotPos.rotate;*/
				SetNewBotRef();
				MSMoveRatio.base--;
				MSMoveRatio.end--;
				MSMoveRatio.pivot--;
				MSMoveRatio.angle--;
				MSMoveRatio.rotate--;
			}
			//printf("\n big move %d MasterSlave Move ratio %d\n",BGM,MSMoveRatio.base);
		}
		if(c=='h')
		{
			SetNewBotRef();
			/*
			SetHome.base = getNormalizedInput(BASE_POSITION_AT)+getNormalizedInput(BASE_POSITION_FORCE_DELTA);
			SetHome.end = getNormalizedInput(END_POSITION_AT)+getNormalizedInput(END_POSITION_FORCE_DELTA);
			SetHome.pivot = getNormalizedInput(PIVOT_POSITION_AT)+getNormalizedInput(PIVOT_POSITION_FORCE_DELTA);
			SetHome.angle = getNormalizedInput(ANGLE_POSITION_AT)+getNormalizedInput(ANGLE_POSITION_FORCE_DELTA);
			SetHome.rotate = getNormalizedInput(ROT_POSITION_AT)+getNormalizedInput(ROT_POSITION_FORCE_DELTA);
			*/
		}	
		if(c=='p')
		{
			a1=getNormalizedInput(BASE_POSITION_AT)+getNormalizedInput(BASE_POSITION_FORCE_DELTA)+f1;
			a3=getNormalizedInput(END_POSITION_AT)+getNormalizedInput(END_POSITION_FORCE_DELTA)+f3;
			a2=getNormalizedInput(PIVOT_POSITION_AT)+getNormalizedInput(PIVOT_POSITION_FORCE_DELTA)+f2;
			a4=getNormalizedInput(ANGLE_POSITION_AT)+getNormalizedInput(ANGLE_POSITION_FORCE_DELTA)+f4;
			a5=getNormalizedInput(ROT_POSITION_AT)+getNormalizedInput(ROT_POSITION_FORCE_DELTA)+f5;
			fprintf(CptA.fp,"a %d,%d,%d,%d,%d\n",a1,a2,a3,a4,a5);
			fprintf(CptA.fp,"S EERoll,%d\n",ServoSet);
			//printf("\na %d,%d,%d,%d,%d",a1,a2,a3,a4,a5);
		}
		if(c==' ')
		{ 
			fprintf(CptA.fp,"z 500000\n");
			//printf("\n insert dwell .5 seconds\n");
		}
		if(c=='g')
		{ 
			if(GripperToggle==0)
			{
				fprintf(CptA.fp,"S GripperMotor,1\n");
				//printf("\n GripperMotor On\n");
				GripperToggle=1;
				SetGripperMotor(GripperToggle);
			}
			else
			{
				fprintf(CptA.fp,"S GripperMotor,0\n");
				//printf("\n GripperMotor Off\n");
				GripperToggle=0;
				SetGripperMotor(GripperToggle);
			}
		}
		if(c=='t')
		{
			if(forceMode==1)
			{
				mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=0;
				mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=0;
				mapped[PID_ADDRESS]=3;
				mapped[PID_P]=DEFAULT_PID_SETTING_PY;
				mapped[PID_ADDRESS]=4;
				mapped[PID_ADDRESS]=0;
				mapped[PID_P]=0;
				mapped[PID_ADDRESS]=1;
				mapped[PID_ADDRESS]=2;
				mapped[SPEED_FACTORA]=DEF_SPEED_FACTOR_A ;
				forceMode=2;
				//printf("\n Move XYZ yo \n");
				controlled=1;
				
			}else if(forceMode==0)
			{

				mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=DEF_SPEED_FACTOR_DIFF;
				mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=DEF_SPEED_FACTOR_DIFF;
				mapped[PID_ADDRESS]=3;
				mapped[PID_P]=0;
				mapped[PID_ADDRESS]=4;
				mapped[PID_ADDRESS]=0;
				mapped[PID_P]=DEFAULT_PID_SETTING_XYZ;
				mapped[PID_ADDRESS]=1;
				mapped[PID_ADDRESS]=2;
				mapped[SPEED_FACTORA]=0;
				forceMode=1;
				controlled=0;
				
				//printf("\n Move Pitch and Yaw \n");
				
			}else if(forceMode==2)
			{
				mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=0;
				mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=0;
				mapped[PID_ADDRESS]=0;
				mapped[PID_P]=DEFAULT_PID_SETTING_XYZ;
				mapped[PID_ADDRESS]=1;
				mapped[PID_ADDRESS]=2;
				mapped[PID_ADDRESS]=3;
				
//				mapped[PID_P]=0; //DEFAULT_PID_SETTING_PY;
				mapped[PID_P]=DEFAULT_PID_SETTING_PY;
				mapped[PID_ADDRESS]=4;
				mapped[SPEED_FACTORA]=0;
				if(RunMode==1)
					forceMode=3;
				else
					forceMode=0;
				//printf("\n Move fine adjust all \n");
				controlled=0;
			}else if(forceMode==3)
			{
				////printf("\n Move all \n");
				mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=DEF_SPEED_FACTOR_DIFF;
				mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=DEF_SPEED_FACTOR_DIFF;
				mapped[PID_P]=0;
				mapped[PID_ADDRESS]=3;
				mapped[PID_ADDRESS]=4;
				mapped[PID_ADDRESS]=0;
				mapped[PID_ADDRESS]=1;
				mapped[PID_ADDRESS]=2;
				mapped[SPEED_FACTORA]=DEF_SPEED_FACTOR_A ;
				forceMode=2;
				controlled=1;
				//printf("\n Move All \n");
			}
		}
		if(c=='q')
		{
			f1--;
		}
		if(c=='w')
		{
			f1++;
		}
		if(c=='a')
		{
			f2--;
		}
		if(c=='s')
		{
			f2++;
		}
		if(c=='z')
		{
			f3--;
		}
		if(c=='x')
		{
			f3++;
		}
		if(c=='e')
		{
			f4--;
		}
		if(c=='r')
		{
			f4++;
		}
		if(c=='d')
		{
			f5--;
		}
		if(c=='f')
		{
			f5++;
		}
		if(c=='Q')
		{
			f1=f1-BGM;
		}
		if(c=='W')
		{
			f1=f1+BGM;
		}
		if(c=='A')
		{
			f2=f2-BGM;
		}
		if(c=='S')
		{
			f2=f2+BGM;
		}
		if(c=='Z')
		{
			f3=f3-BGM;
		}
		if(c=='X')
		{
			f3=f3+BGM;
		}
		if(c=='E')
		{
			f4=f4-BGM;
		}
		if(c=='R')
		{
			f4=f4+BGM;
		}
		if(c=='D')
		{
			f5=f5-BGM;
		}
		if(c=='F')
		{
			f5=f5+BGM;
		}
		if(c==',')
		{
			ServoSet=ServoSet+1;
			SetGripperRoll(ServoSet);
		}
		if(c=='.')
		{
			ServoSet=ServoSet-1;
			SetGripperRoll(ServoSet);
		}
	}
	mapped[MAX_ERROR]=(2000 ^ 4000);
	mapped[DIFF_FORCE_SPEED_FACTOR_ANGLE]=0;
	mapped[DIFF_FORCE_SPEED_FACTOR_ROT]=0;
	mapped[PID_ADDRESS]=0;
	mapped[PID_P]=DEFAULT_PID_SETTING_XYZ;
	mapped[PID_ADDRESS]=1;
	mapped[PID_ADDRESS]=2;
	mapped[PID_ADDRESS]=3;
	mapped[PID_P]=DEFAULT_PID_SETTING_PY;
	mapped[PID_ADDRESS]=4;
	mapped[SPEED_FACTORA]=0;
	
	
	mapped[FINE_ADJUST_BASE]=0;
	mapped[FINE_ADJUST_END]=0;
	mapped[FINE_ADJUST_PIVOT]=0;
	mapped[FINE_ADJUST_ANGLE]=0;
	mapped[FINE_ADJUST_ROT]=0;
        mapped[COMMAND_REG]=3158688;	
	//printf("\nAll done\n");
	controlled=0;
	fprintf(CptA.fp,"x\n");
			
	fclose(CptA.fp);
	resetTermios();
	
    return NULL;
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

/*		err = pthread_create(&(tid[i]), NULL, &CapturePoints, (void*)&CptA);
		if (err != 0)
		{
			//printf("\ncan't create thread :[%s]", strerror(err));
			return 1;
		}
		else
		{
			//printf("\n Begin Program\n");
		}
    }*/
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
		mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
		mapped[REC_PLAY_CMD]=CMD_RECORD; // start recording
		c=' ';
		while(c!='k') // kill
		{
			c=getchar();
			
			
		}
		mapped[REC_PLAY_CMD]=0; // stop recording
		
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


void setDefaults(int State)
{

	int i,ForceFelt,j,HiBoundry,LowBoundry,BoundryACC;
	char c;
    FILE *CentersFile,*RemoteRobotAddress,*DiffFile;
	int HexValue;
	int BaseBoundryHighL,BaseBoundryLowL,EndBoundryHighL,EndBoundryLowL,PivotBoundryHighL,PivotBoundryLowL;
	int AngleBoundryHighL,AngleBoundryLowL,RotateBoundryHighL,RotateBoundryLowL;

	int IntFloat;
	float *fConvert=(float *)(&IntFloat);
	
	
	
	DiffFile = fopen("DiffCorrectionFactor.txt", "rs");
	if(DiffFile!=NULL)
	{
		fscanf (DiffFile, "%f", &DiffCorrectionFactor);
		//fgets(DiffFile, sizeof(RemoteRobotAdd)+1, RemoteRobotAddress);
		//fscanf(RemoteRobotAddress,"%[^\n]",RemoteRobotAdd);
		//printf("%f \n",DiffCorrectionFactor);
		fclose(DiffFile);
	}
	DiffFile = fopen("Boundaries.txt", "rs");
	if(DiffFile!=NULL)
	{
// BaseBoundryHigh,BaseBoundryLow,EndBoundryHigh,EndBoundryLow,PivotBoundryHigh,PivotBoundryLow,AngleBoundryHigh=DEFAULT_ANGLE_BOUNDRY_HI,AngleBoundryLow=DEFAULT_ANGLE_BOUNDRY_LOW,RotateBoundryHigh,RotateBoundryLow;
		
		
		fscanf (DiffFile, "%i %i", &BaseBoundryHighL,&BaseBoundryLowL);
		fscanf (DiffFile, "%i %i", &PivotBoundryHighL,&PivotBoundryLowL);
		fscanf (DiffFile, "%i %i", &EndBoundryHighL,&EndBoundryLowL);
		fscanf (DiffFile, "%i %i", &AngleBoundryHighL,&AngleBoundryLowL);
		fscanf (DiffFile, "%i %i", &RotateBoundryHighL,&RotateBoundryLowL);
		//fgets(DiffFile, sizeof(RemoteRobotAdd)+1, RemoteRobotAddress);
		//fscanf(RemoteRobotAddress,"%[^\n]",RemoteRobotAdd);
		//printf("Angle High boundry%i    Angle Low Boundry %i \n",AngleHIBoundry,AngleLOWBoundry);
		fclose(DiffFile);
	}

	
	RemoteRobotAddress = fopen("RemoteRobotAddress.txt", "rs");
	if(RemoteRobotAddress!=NULL)
	{
		fgets(RemoteRobotAdd, sizeof(RemoteRobotAdd), RemoteRobotAddress);
		//fscanf(RemoteRobotAddress,"%[^\n]",RemoteRobotAdd);
		//printf("%s \n",RemoteRobotAdd);
		fclose(RemoteRobotAddress);
	}
	mapped[FORCE_BIAS_BASE]=0;
	mapped[FORCE_BIAS_END]=0;
	mapped[FORCE_BIAS_PIVOT]=0;
	mapped[FORCE_BIAS_ANGLE]=0;
	mapped[FORCE_BIAS_ROT]=0;

	mapped[ACCELERATION_MAXSPEED]=ACCELERATION_MAXSPEED_DEF;
	maxSpeed=(ACCELERATION_MAXSPEED_DEF) & 0b00000000000011111111111111111111;
	coupledAcceleration=((ACCELERATION_MAXSPEED_DEF) & 0b00000011111100000000000000000000) >> 20;

	if(State==1)
	{


		mapped[BASE_FORCE_DECAY]=000000;
		mapped[END_FORCE_DECAY]=000000;
		mapped[PIVOT_FORCE_DECAY]=000000;
		mapped[ANGLE_FORCE_DECAY]=000000;
		mapped[ROTATE_FORCE_DECAY]=000000;

		mapped[ACCELERATION_MAXSPEED]=ACCELERATION_MAXSPEED_DEF;
		maxSpeed=(ACCELERATION_MAXSPEED_DEF) & 0b00000000000011111111111111111111;
		coupledAcceleration=((ACCELERATION_MAXSPEED_DEF) & 0b00000011111100000000000000000000) >> 20;
	

		mapped[REC_PLAY_TIMEBASE]=5;
		
    CentersFile = fopen("AdcCenters.txt", "rs");

    //read file into array
	if(CentersFile!=NULL)
	{
		fscanf(CentersFile, "%x", &HexValue);
		//printf("%x \n",HexValue);
		mapped[BASE_SIN_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[BASE_COS_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[END_SIN_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[END_COS_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[PIVOT_SIN_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[PIVOT_COS_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[ANGLE_SIN_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[ANGLE_COS_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[ROT_SIN_CENTER]=HexValue;
		fscanf(CentersFile, "%x", &HexValue);
		mapped[ROT_COS_CENTER]=HexValue;
		fclose(CentersFile);
	}
	
	CentersFile = fopen("AxisCal.txt", "rs");

    //read file into array
	if(CentersFile!=NULL)
	{
		fscanf(CentersFile, "%f", &JointsCal[0]);
		//printf("%f \n",JointsCal[0]);
		fscanf(CentersFile, "%f", &JointsCal[1]);
		fscanf(CentersFile, "%f", &JointsCal[2]);
		fscanf(CentersFile, "%f", &JointsCal[3]);
		fscanf(CentersFile, "%f", &JointsCal[4]);
		fscanf(CentersFile, "%i", &HexValue);
		mapped[ANGLE_END_RATIO]=HexValue;//((LG_RADIUS/SM_RADIUS * MOTOR_STEPS * MICRO_STEP)/(MOTOR_STEPS*GEAR_RATIO*MICRO_STEP))*2^24
		fclose(CentersFile);
	}
 
    

/*		mapped[BASE_SIN_CENTER]=0x7f80000;
		mapped[BASE_COS_CENTER]=0x7f80000;
		mapped[END_SIN_CENTER]=0x7f80000;
		mapped[END_COS_CENTER]=0x07f80000;
		mapped[PIVOT_SIN_CENTER]=0x4a00000;
		mapped[PIVOT_COS_CENTER]=0x6500000;
		mapped[ANGLE_SIN_CENTER]=0x7890000;
		mapped[ANGLE_COS_CENTER]=0x5080000;
		mapped[ROT_SIN_CENTER]=0x5200000;
		mapped[ROT_COS_CENTER]=0x6f00000;*/

//		mapped[BASE_SIN_CENTER]=0x7f80000;
//		mapped[BASE_COS_CENTER]=0x7f80000;
//		mapped[END_SIN_CENTER]=0x7f80000;
//		mapped[END_COS_CENTER]=0x07f80000;
//		mapped[PIVOT_SIN_CENTER]=0x6500000;
//		mapped[PIVOT_COS_CENTER]=0x4a00000;
//		mapped[ANGLE_SIN_CENTER]=0x7f80000;
//		mapped[ANGLE_COS_CENTER]=0x4900000;
//		mapped[ROT_SIN_CENTER]=0x6f00000;
//		mapped[ROT_COS_CENTER]=0x5200000;


		mapped[DIFF_FORCE_BETA ]=0x0102;

	// set up PID defaults
	
		mapped[PID_P]=DEFAULT_PID_SETTING_XYZ;
		mapped[PID_I]=0;
		mapped[PID_D]=0;
		mapped[PID_DELTATNOT]=0;
		mapped[PID_DELTAT]=0;
		mapped[PID_ADDRESS]=0;
		mapped[PID_ADDRESS]=1;
		mapped[PID_ADDRESS]=2;
		mapped[PID_ADDRESS]=3;
		mapped[PID_P]=DEFAULT_PID_SETTING_PY;
		mapped[PID_ADDRESS]=4;
		
		// set boundries  
/*		BaseBoundryHigh=90000;
		BaseBoundryLow=-90000;
		EndBoundryHigh=130000;
		EndBoundryLow=-130000;
		PivotBoundryHigh=70000;
		PivotBoundryLow=-70000;
		AngleBoundryHigh=DEFAULT_ANGLE_BOUNDRY_HI;
		AngleBoundryLow=DEFAULT_ANGLE_BOUNDRY_LOW;
		RotateBoundryHigh=3000;
		RotateBoundryLow=-3000;
		*/
		
		
		mapped[BOUNDRY_BASE]=BaseBoundryHigh + BaseBoundryLow;
		//0x2bf2d40e; //
		mapped[BOUNDRY_END]= EndBoundryHigh + EndBoundryLow;
		//0x3f7ac086;  //
		mapped[BOUNDRY_PIVOT]=PivotBoundryHigh + PivotBoundryLow;
		//0x222eddd2; //1
		//HiBoundry=AngelBoundryHigh + AngleBoundryLow;
		//AngleHIBoundry/8;
		//LowBoundry=AngleLOWBoundry/8;
		//BoundryACC=(HiBoundry << 16) | (0x0000ffff & LowBoundry);
		mapped[BOUNDRY_ANGLE]=AngleBoundryHigh + AngleBoundryLow;
		//BoundryACC;// 0x01f4fe0c; //
		//printf("Angle boundry pack %x ",BoundryACC);
		
		mapped[BOUNDRY_ROT]=RotateBoundryHigh + RotateBoundryLow;
		//((RotateBoundryHigh/8) << 16) | (0X0000FFFF & (RotateBoundryLow/8));

		
		
		
		

		mapped[SPEED_FACTORA]=0;
		mapped[SPEED_FACTORB]=0x130;
		mapped[MAX_ERROR]=(2000 ^ 6000);
		

		mapped[DIFF_FORCE_TIMEBASE]=140000; // this is currently remapped to MaxSpeed XYZ Force
		
		mapped[DIFF_FORCE_MAX_SPEED]=90000;
//		IntFloat)
		*fConvert=DiffCorrectionFactor;
		
		//printf(" float value %x \n",IntFloat);
		//if(RunMode==1)
			mapped[DIFF_FORCE_ANGLE_COMPENSATE]=0;//IntFloat;
		//else
		//	mapped[DIFF_FORCE_ANGLE_COMPENSATE]=0;//IntFloat;
		
		//mapped[ANGLE_END_RATIO]=645278;//((LG_RADIUS/SM_RADIUS * MOTOR_STEPS * MICRO_STEP)/(MOTOR_STEPS*GEAR_RATIO*MICRO_STEP))*2^24
		




	
		mapped[FRICTION_BASE]=0x130; // fixed point 16 bit == .1
		mapped[FRICTION_END]=0x130; // fixed point 16 bit == .1
		mapped[FRICTION_PIVOT]=0x130; // fixed point 16 bit == .1
		mapped[FRICTION_ANGLE]=0x50; // fixed point 16 bit == .1
		mapped[FRICTION_ROT]=0x50; // fixed point 16 bit == .1
	
		if(RestoreCalTables("/srv/samba/share/HiMem.dta")==0)
		{
			MoveRobot(20000,20000,20000,20000,20000,BLOCKING_MOVE);
			MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
		}

		
	}
	if(State==2)
	{
		CmdVal=3158688;
		mapped[FINE_ADJUST_BASE]=0;
		mapped[FINE_ADJUST_END]=0;
		mapped[FINE_ADJUST_PIVOT]=0;
		mapped[FINE_ADJUST_ANGLE]=0;
		mapped[FINE_ADJUST_ROT]=0;
	}
	if(State==3)
	{
		
		//printf("\nMonitor Force Started");
		while(1)
		{
			//printf("\nTable Delta ");
			for(i=0;i<5;i++)
			{
				//printf("%6d ",getNormalInput(BASE_POSITION_DELTA+i));
			}
			//printf(" PID_DELTA ");
			for(i=0;i<5;i++)
			{
				//printf("%5d ",getNormalizedInput(BASE_POSITION_PID_DELTA+i));
			}
			////printf("\nForce_DELTA ");
			//for(i=0;i<5;i++)
			//{
			//	//printf("%5d ",getNormalizedInput(BASE_POSITION_FORCE_DELTA+i));
			//}
		}
		
	}
	
	if(State==4)
	{
		//initTermios(0);
		mapped[PID_ADDRESS]=3;
		mapped[PID_P]=0;
		mapped[PID_I]=0;
		mapped[PID_D]=0;
		mapped[PID_DELTATNOT]=0;
		mapped[PID_DELTAT]=0;
		mapped[PID_ADDRESS]=4;
		c=' ';
		mapped[FINE_ADJUST_BASE]=0;
		mapped[FINE_ADJUST_END]=0;
		mapped[FINE_ADJUST_PIVOT]=0;
		mapped[FINE_ADJUST_ANGLE]=0;
		mapped[FINE_ADJUST_ROT]=0;
		
		//printf("\nForce Protect Started");
		while(1) 
//		while(c!='k') // kill
		{
			//c=getchar();
			for(i=0;i<5;i++)
			{
				ForceFelt=getNormalizedInput(BASE_POSITION_PID_DELTA+i)-getNormalizedInput(BASE_POSITION_DELTA+i);
				//printf("%5d ",ForceFelt);
				if(abs(ForceFelt-PositionAdjust[i]) >= ForceLimit[i])
				{
					if((ForceFelt-PositionAdjust[i]) < 0)
					{
						//PositionAdjust[i]=2*(abs((ForceFelt-PositionAdjust[i]))-ForceLimit[i]);						
					}
					else
					{
						//PositionAdjust[i]=2*-(abs((ForceFelt-PositionAdjust[i]))-ForceLimit[i]);
					}
				}
			}
			for(i=0;i<3;i++)
			{
				//mapped[FINE_ADJUST_BASE+i]=PositionAdjust[i];
				//printf("%5d ",PositionAdjust[i]);
			}
			//printf("\n");
			
			usleep(10000);
		}	
	//resetTermios();		
	}
	if(State==5)
	{
		
		//printf("\nMonitor Accelerateion Force Started");
		while(1)
		{
			//printf("\nSpeed Delta ");
			for(i=0;i<5;i++)
			{
				//printf("%5d ",mapped[SLOPE_BASE_POSITION+i]);
			}
		}
		
	}
	if(State==6)
	{
		
		//printf("\nMonitor Force offset Started");
		while(1)
		{
			////printf("\nSpeed Delta ");
			for(i=0;i<5;i++)
			{
				//printf("%6d  ",GetAxisCurrent(i));
			}
			//printf("\n");
		}
		
	}
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
	val = mapped[param];
	if(param <= ROT_POSITION_FORCE_DELTA)
	{
		corrF = JointsCal[(param-INPUT_OFFSET) % 5];
	}
	if((val & 0x40000)!=0)
	{
		val = (val | 0xfff80000);
	}
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
	////printf("\n%d %d %d %d %d\n",b1,b2,b3,b4,b5);	
	////printf("\n%d %d %d %d %d\n",a1,a2,a3,a4,a5);	
//    //printf("\nStart wait Goal");
	while(!isNear(a1,b1,500) || !isNear(a2,b2,500) || !isNear(a3,b3,500) || !isNear(a4,b4,500) || !isNear(a5,b5,500))
	{
		b1=getNormalizedInput(BASE_POSITION_AT);
		b3=getNormalizedInput(END_POSITION_AT);
		b2=getNormalizedInput(PIVOT_POSITION_AT);
		b4=getNormalizedInput(ANGLE_POSITION_AT);
		b5=getNormalizedInput(ROT_POSITION_AT);
/*		c1=getNormalizedInput(BASE_POSITION_DELTA);
		c3=getNormalizedInput(END_POSITION_DELTA);
		c2=getNormalizedInput(PIVOT_POSITION_DELTA);
		c4=getNormalizedInput(ANGLE_POSITION_DELTA);
		c5=getNormalizedInput(ROT_POSITION_DELTA);*/
		clock_gettime(CLOCK_REALTIME, &spec);
		newTime  = spec.tv_sec;
		if((newTime-oldTime)>20)
			return 0;
		
		////printf("\n%d %d %d %d %d Waiting for %d %d %d %d %d ",b1,b2,b3,b4,b5,a1,a2,a3,a4,a5);	
	}
    ////printf("\nEnd wait Goal");
	return 0;
		
}

void moverobotPID(int a1,int a2,int a3,int a4,int a5)
{
	CheckBoundry(&a1,&a2,&a3,&a4,&a5);	
	a1=(int)((double)a1 * JointsCal[0]);
	a2=(int)((double)a2 * JointsCal[1]);
	a3=(int)((double)a3 * JointsCal[2]);
	a4=(int)((double)a4 * JointsCal[3]);
	a5=(int)((double)a5 * JointsCal[4]);
	mapped[FINE_ADJUST_BASE]=a1;
	mapped[FINE_ADJUST_END]=a3;
	mapped[FINE_ADJUST_PIVOT]=a2;
	mapped[FINE_ADJUST_ANGLE]=a4;
	mapped[FINE_ADJUST_ROT]=a5;
}

int MoveRobot(int a1,int a2,int a3,int a4,int a5, int mode)
{

	//CheckBoundry(&a1,&a2,&a3,&a4,&a5);
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
	////printf("\nStart wait Goal");
//	332800
//	166400
//	0.25679012345679012345679012345679
//	0.00987654320987654320987654320988

	//if(mode==BLOCKING_MOVE)
		//WaitMoveGoal(LastGoal[0],LastGoal[1],LastGoal[2],LastGoal[3],LastGoal[4],DEFAULT_MOVE_TIMEOUT);

	

	LastGoal[0]=a1;
	LastGoal[1]=a2;
	LastGoal[2]=a3;
	LastGoal[3]=a4;
	LastGoal[4]=a5;

	a1=(int)((double)a1 * JointsCal[0]);
	a2=(int)((double)a2 * JointsCal[1]);
	a3=(int)((double)a3 * JointsCal[2]);
	a4=(int)((double)a4 * JointsCal[3]);
	a5=(int)((double)a5 * JointsCal[4]);
	//printf("angles result %d %d %d %d %d",a1,a2,a3,a4,a5);
	while((mapped[CMD_FIFO_STATE] & 0x01) != 0);
	
	mapped[COMMAND_REG]=CMD_MOVEEN | CmdVal;
	mapped[BASE_POSITION]=a1;
	mapped[END_POSITON]=a3;
	mapped[PIVOT_POSITON]=a2;
	mapped[ANGLE_POSITON]=a4;
	mapped[ROT_POSITON]=a5;
	mapped[COMMAND_REG]=CMD_MOVEEN | CMD_MOVEGO | CmdVal;
	mapped[COMMAND_REG]=CmdVal;
	/*if(mode==BLOCKING_MOVE)
		WaitMoveGoal(LastGoal[0],LastGoal[1],LastGoal[2],LastGoal[3],LastGoal[4],DEFAULT_MOVE_TIMEOUT);*/
	return 0;
}

int MoveRobotStraight(struct XYZ xyz_1, struct XYZ xyz_2, double cart_speed)
{
	
	double max_step_size = 1; //microns
	struct Vector U1 = xyz_1.position;
	struct Vector U2 = xyz_2.position;
	struct Vector U21 = subtract(U2, U1);
	struct Vector v21 = normalize(U21);
	double U21_mag = magnitude(U21);
	int num_div = (int)ceil(U21_mag/max_step_size);
	double step = U21_mag / num_div;
	
	int angular_velocity;
	struct Vector Ui;
	
	
	struct Vector cur_pos = xyz_1.position;
	struct Vector cur_dir = xyz_1.direction;
	struct Config cur_config = xyz_1.config;
	struct XYZ cur_xyz = new_XYZ(cur_pos, cur_dir, cur_config);
	
	struct J_angles J_angles_list[num_div];
	double speeds_list[num_div];
	struct J_angles J_angles_new;
	struct J_angles J_angles_old = xyz_to_J_angles(cur_xyz);
	
	
	printf("\n\nStarting pre-calculation for MoveRobotStraight:");
	
	printf("\nxyz_1: ");
	print_XYZ(xyz_1);
	printf("\nxyz_2: ");
	print_XYZ(xyz_2);
	printf("\ncart_speed: %f", cart_speed);
	
	printf("\nU21:");
	print_vector(U21);
	printf("\nv21:");
	print_vector(v21);
	printf("\nU21_mag: %f", U21_mag);
	printf("\nnum_div: %i", num_div);
	printf("\nstep: %f", step);
	
	
	int i;
	int cal_max_angular_velocity = 0;
	for(i=0;i<=num_div;i++){
		Ui = add(U1, scalar_mult(((float)i)*step, v21));
		cur_xyz.position = Ui;
		J_angles_new = xyz_to_J_angles(cur_xyz);
		angular_velocity = k_tip_speed_to_angle_speed(J_angles_old, J_angles_new, cart_speed);
		
		J_angles_list[i] = J_angles_new;
		speeds_list[i] = angular_velocity;
		
		
		//printf("\nang_vel: %d  J: ");
		//print_J_angles(J_angles_new);
		if(angular_velocity > cal_max_angular_velocity){
			cal_max_angular_velocity = angular_velocity;
		}
		//printf("%i: %i", i, angular_velocity);
		
	}
	
	printf("\nDone pre-calculating MoveRobotStraight");
	printf("cal_max_angular_velocity = %i", cal_max_angular_velocity);
	printf("\nStarting MoveRobotStraight movement");
	printf("\n");
	
	
	/*
	int cur_angular_velocity = 38773; //5 (deg/s)
	for(i=0;i<=num_div;i++){
		//cur_angular_velocity = speeds_list[i];
		
		
		//Startspeed
		printf("mapped startspeed: %i", 1 ^ cur_angular_velocity);
		mapped[START_SPEED]=1 ^ cur_angular_velocity;
		
		
		//Maxspeed
		maxSpeed=cur_angular_velocity & 0b00000000000011111111111111111111;
		printf("maxSpeed: %i", cur_angular_velocity & 0b00000000000011111111111111111111);
		mapped[ACCELERATION_MAXSPEED]=maxSpeed + (coupledAcceleration << 20);
		printf("mapped startspeed: %i", maxSpeed + (coupledAcceleration << 20));
		
		printf("i = %i, J1 = %f", i, J_angles_list[i].J1);
		//MoveRobot(J_angles_list[i].J1, J_angles_list[i].J2, J_angles_list[i].J3, J_angles_list[i].J4, J_angles_list[i].J5, BLOCKING_MOVE);
	}
	*/
	printf("\n MoveRobotStraight movement");
	
	return 0;
}

int CheckBoundry(int* j1, int* j2, int* j3, int* j4, int* j5)
{
	int h1,h2,h3,h4,h5,l1,l2,l3,l4,l5;
	h1=(int)((float)BaseBoundryHigh / fabs(JointsCal[0]));
	h2=(int)((float)PivotBoundryHigh / fabs(JointsCal[1]));
	h3=(int)((float)EndBoundryHigh / fabs(JointsCal[2]));
	h4=(int)((float)AngleBoundryHigh / fabs(JointsCal[3]));
	h5=(int)((float)RotateBoundryHigh / fabs(JointsCal[4]));
	l1=(int)((float)BaseBoundryLow / fabs(JointsCal[0]));
	l2=(int)((float)PivotBoundryLow / fabs(JointsCal[1]));
	l3=(int)((float)EndBoundryLow / fabs(JointsCal[2]));
	l4=(int)((float)AngleBoundryLow / fabs(JointsCal[3]));
	l5=(int)((float)RotateBoundryLow / fabs(JointsCal[4]));
	
	if(*(j1) >= h1)
	{
		*(j1) = h1;
	}
	
	if(*(j1) <= l1)
	{
		*(j1) = l1;
	}
	if(*(j2) >= h2)
	{
		*(j2) = h2;
	}
	if(*(j2) <= l2)
	{
		*(j2) = l2;
	}
	if(*(j3) >= h3)
	{
		*(j3) = h3;
	}
	if(*(j3) <= l3)
	{
		*(j3) = l3;
	}
	if(*(j4) >= h4)
	{
		*(j4) = h4;
	}
	if(*(j4) <= l4)
	{
		*(j4) = l4;
	}
	if(*(j5) >= h5)
	{
		*(j5) = h5;
	}
	if(*(j5) <= l5)
	{
		*(j5) = l5;
	}
	////printf(" boundry calc %d %d %f %d",*(j1),BaseBoundryHigh,fabs(JointsCal[0]),(int)((float)BaseBoundryHigh / fabs(JointsCal[0])));
/*
EndBoundryHigh
EndBoundryLow
PivotBoundryHigh
PivotBoundryLow
AngleBoundryHigh
AngleBoundryLow
RotateBoundryHigh
RotateBoundryLow
*/
 if(*(j1)==0)
	 *(j1)=1;
 return 0;
}

int SetParam(char *a1,float fa2,int a3,int a4,int a5)
{
	int i,BDH,BDL,Axis;
	int a2=(int)fa2;
	int fxa2=(a2<<8)+(fa2-a2)*256;
	
	////printf("%s %s %d %d \n",Params[i],a1,a2,i);

	for(i=0;i<MAX_PARAMS;i++)
	{
		////printf("%s %s %d %d \n",Params[i],a1,a2,i);
		if(strcmp(a1,Params[i])==0)
		{
				switch(i)
				{
						case 0:
						////printf("Set Speed\n");
							//set Max Speed
							maxSpeed=a2 & 0b00000000000011111111111111111111;
							mapped[ACCELERATION_MAXSPEED]=maxSpeed + (coupledAcceleration << 20);
							return 0;
						break;
						case 1:
							//set Acceleration
							coupledAcceleration=a2 & 0b111111;
							mapped[ACCELERATION_MAXSPEED]=maxSpeed + (coupledAcceleration << 20);
							return 0;
						break;
						case 2:
							mapped[FORCE_BIAS_BASE]=a2;
							return 0;
						case 3:
							mapped[FORCE_BIAS_END]=a2;
							return 0;
						break;
						case 4:
							mapped[FORCE_BIAS_PIVOT]=a2;
							return 0;
						break;
						case 5:
							mapped[FORCE_BIAS_ANGLE]=a2;
							return 0;
						break;
						case 6:
							mapped[FORCE_BIAS_ROT]=a2;
							return 0;
						break;
						case 7:
							mapped[FRICTION_BASE]=fxa2;
							return 0;
						break;
						case 8:
							mapped[FRICTION_END]=fxa2;
							return 0;
						break;
						case 9:
							mapped[FRICTION_PIVOT]=fxa2;
							return 0;
						break;
						case 10:
							mapped[FRICTION_ANGLE]=fxa2;
							return 0;
						break;
						case 11:
							mapped[FRICTION_ROT]=fxa2;
							return 0;
						break;
						case 12:
							BaseBoundryHigh=(int)((float)a2 * fabs(JointsCal[0]));
							////printf(" boundry set %d %d %f %f ", a2, BaseBoundryHigh, JointsCal[0], fabs(JointsCal[0]));
							BDH=BaseBoundryHigh;
							BDL=BaseBoundryLow;
							Axis=BOUNDRY_BASE;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 13:
							BaseBoundryLow=(int)((float)a2 * fabs(JointsCal[0]));
							BDH=BaseBoundryHigh;
							BDL=BaseBoundryLow;
							Axis=BOUNDRY_BASE;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 14:
							EndBoundryHigh=(int)((float)a2 * fabs(JointsCal[2]));
							BDH=EndBoundryHigh;
							BDL=EndBoundryLow;
							Axis=BOUNDRY_END;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 15:
							EndBoundryLow=(int)((float)a2 * fabs(JointsCal[2]));
							BDH=EndBoundryHigh;
							BDL=EndBoundryLow;
							Axis=BOUNDRY_END;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 16:
							PivotBoundryHigh=(int)((float)a2 * fabs(JointsCal[1]));
							BDH=PivotBoundryHigh;
							BDL=PivotBoundryLow;
							Axis=BOUNDRY_PIVOT;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 17:
							PivotBoundryLow=(int)((float)a2 * fabs(JointsCal[1]));
							BDH=PivotBoundryHigh;
							BDL=PivotBoundryLow;
							Axis=BOUNDRY_PIVOT;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 18:
							AngleBoundryHigh=(int)((float)a2 * fabs(JointsCal[3]));
							BDH=AngleBoundryHigh;
							BDL=AngleBoundryLow;
							Axis=BOUNDRY_ANGLE;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 19:
							AngleBoundryLow=(int)((float)a2 * fabs(JointsCal[3]));
							BDH=AngleBoundryHigh;
							BDL=AngleBoundryLow;
							Axis=BOUNDRY_ANGLE;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 20:
							RotateBoundryHigh=(int)((float)a2 * fabs(JointsCal[4]));
							BDH=RotateBoundryHigh;
							BDL=RotateBoundryLow;
							Axis=BOUNDRY_ROT;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 21:
							RotateBoundryLow=(int)((float)a2 * fabs(JointsCal[4]));
							BDH=RotateBoundryHigh;
							BDL=RotateBoundryLow;
							Axis=BOUNDRY_ROT;
							mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
							return 0;
						break;
						case 22:
							SetGripperMotor(a2);
							//printf("Gripper Motor Set\n");
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
							mapped[START_SPEED]=1 ^ a2;
						break;
						case 26:
						break;
						default:
						break;
				}
		
		}
	}
	return 0;

}
int MoveRobotRelative(int a1,int a2,int a3,int a4,int a5, int mode)
{
	int b1,b2,b3,b4,b5;
	b1=getNormalizedInput(BASE_POSITION_AT);
	b3=getNormalizedInput(END_POSITION_AT);
	b2=getNormalizedInput(PIVOT_POSITION_AT);
	b4=getNormalizedInput(ANGLE_POSITION_AT);
	b5=getNormalizedInput(ROT_POSITION_AT);
	
	////printf("\nRelative move %d %d %d %d %d",b1,b2,b3,b4,b5);
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
		mapped[DMA_CONTROL]=DMA_RESET_ALL;
		mapped[DMA_CONTROL]=0;
		blocks=p2/256;
		for(j=0;j<blocks;j++) // only do full blocks inside loop
		{	
			mapped[DMA_READ_ADDRESS]=p1+(j*1024);
			mapped[DMA_READ_PARAMS]=(2<<8) | 127;
			mapped[DMA_CONTROL]=DMA_READ_BLOCK;
			mapped[DMA_CONTROL]=0;
			for(i=0;i<256;i++)
			{
				mapped[DMA_CONTROL]=DMA_READ_DEQUEUE;
				dataarray[i]=mapped[DMA_READ_DATA];
				////printf("\n %d",dataarray[i]);
				mapped[DMA_CONTROL]=0;
			}
			writeSize=fwrite((const void *)dataarray,sizeof(int),256,fp);
			//printf("\n write %d iteration %d",writeSize,j);
			mapped[DMA_CONTROL]=0;
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
		if((readSize=fread((const void *)CalTables,sizeof(int),4*1024*1024,fp))==(Length/4))
		{

			//printf(" size good %d %d",readSize ,Length);
			#ifndef NO_BOOT_DANCE
			MoveRobot(50000,50000,50000,5000,5000,BLOCKING_MOVE);
			MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
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
		mapped[DMA_CONTROL]=DMA_RESET_ALL;
		mapped[DMA_CONTROL]=0;
		blocks=Length/256;
		for(j=0;j<blocks;j++) // only do full blocks inside loop
		{	
			if( ( readSize=fread((const void *)dataarray,sizeof(int),256,fp) )==256)
			{
				mapped[DMA_WRITE_ADDRESS]=Address+(j*1024);
				mapped[DMA_WRITE_PARAMS]=(2<<8) | 127;
				mapped[DMA_CONTROL]=0;
				for(i=0;i<256;i++)
				{
					mapped[DMA_WRITE_DATA]=dataarray[i];
					mapped[DMA_CONTROL]=DMA_WRITE_ENQUEUE;
					mapped[DMA_CONTROL]=0;
				}
				mapped[DMA_CONTROL]=DMA_WRITE_INITIATE;
				mapped[DMA_CONTROL]=0;
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
				mapped[Axis]=Start+i+(k*256);
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
		mapped[Axis]=Start+k;
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
	mapped[COMMAND_REG]=256;  //reset home
	mapped[COMMAND_REG]=0;
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
	mapped[RECORD_LENGTH]=Length/4;
	mapped[REC_PLAY_TIMEBASE]=1;
	mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
	mapped[REC_PLAY_CMD]=CMD_RESET_PLAY;
	mapped[REC_PLAY_CMD]=0;
	mapped[REC_PLAY_CMD]=CMD_PLAYBACK;
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
	mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
	mapped[REC_PLAY_CMD]=CMD_RESET_PLAY;
	mapped[REC_PLAY_CMD]=0;

	/*fa0=fa0+mapped[PLAYBACK_BASE_POSITION];
	fa1=fa1+mapped[PLAYBACK_END_POSITION];
	fa2=fa2+mapped[PLAYBACK_PIVOT_POSITION];
	fa3=fa3+mapped[PLAYBACK_ANGLE_POSITION];
	fa4=fa4+mapped[PLAYBACK_ROT_POSITION];
	mapped[FINE_ADJUST_BASE]=fa0;
	mapped[FINE_ADJUST_END]=fa1;
	mapped[FINE_ADJUST_PIVOT]=fa2;
	mapped[FINE_ADJUST_ANGLE]=fa3;
	mapped[FINE_ADJUST_ROT]=fa4;*/
	mapped[REC_PLAY_CMD]=CMD_RESET_RECORD;
}
int getInput(void)
{
	char iString[510];
	if(gets(iString)!=NULL)
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
	char *token,*p1,*p2,*p3,*p4,*p5,*p6,*p7,*p8,*p9,*p10,*p11,*p12,*p13,*p14,*p15,*p16,*p17,*p18,*p19;
	int BDH,BDL;


	int i,j,Add,Start,Length,Delay,Axis,tokenVal;
	////printf("\nStart wait Goal");
	if(iString !=NULL)
	{
		token = strtok (iString, delimiters);
		if (token !=NULL)
		{
			tokenVal=HashInputCMD(token);
			////printf("Token %s TokenVal %i",token,tokenVal);
		}
		else
			return 1;
		if (tokenVal != 0 ){
			switch(tokenVal)
			{
				case WRITE_TO_ROBOT:
					p1=strtok (NULL, delimiters);
                              Add=(int)p1[0];
                              printf("\nwrite %s %d: ",p1,Add);
					switch(Add) {
					   case 'f': //filename
						p2=strtok(NULL, delimiters);//always zero, toss it.
						p2=strtok(NULL, delimiters);//filename
						if(wfp) fclose(wfp);
						wfp = fopen(p2, "w");
                                    printf("Opened %s as handle %d. ",p2,fileno(wfp));
						break;
					   case 's': //start
					   case 'm': //middle
					   case 'e': //end
						p2=strtok(NULL, delimiters);//bytes
                                    Length=atoi(p2);
                                    printf("Length: %d bytes. \n", Length);
                                    if (0<Length) {
                                        p3=strtok(NULL, "");//remaining data
                                        printf("Found: %d bytes. ", strlen(p3));
                                        Length = unescape(p3, Length);
                                        printf(" %d bytes after unescape",Length);
                                        i=fwrite(p3, 1, Length, wfp);
                                        printf("Wrote %d bytes. ",i);
                                        }
						if('e'==Add && wfp) {
                                        fclose(wfp);
                                        wfp = 0;
                                        printf(" Finished write.\n");
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
					p2=strtok (NULL, delimiters);
	//				p3=strtok (NULL, delimiters);
	//				p4=strtok (NULL, delimiters);
	//				p5=strtok (NULL, delimiters);
					SetParam(p1,atof(p2),0,0,0);
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
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					p3=strtok (NULL, delimiters);
					p4=strtok (NULL, delimiters);
					p5=strtok (NULL, delimiters);
					if(p1!=NULL && p2!=NULL && p3!=NULL && p4!=NULL && p5!=NULL)						
						MoveRobot(atoi(p1),atoi(p2),atoi(p3),atoi(p4),atoi(p5),BLOCKING_MOVE);
				break; 
				
				
				//////////////////////////////////////////////////////////////////////////
				/* Start Wigglesworth Code*/
				
				case MOVETO_CMD:
					printf("\nMOVETO_CMD\n");
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
					printf("\n\nStarting MoveToStraight\n");
					
					//printf("\niString: \n%s", iString);
					
				
					p1 = strtok(NULL, delimiters);
					p2 = strtok(NULL, delimiters);
					p3 = strtok(NULL, delimiters);
					p4 = strtok(NULL, delimiters);
					p5 = strtok(NULL, delimiters);
					p6 = strtok(NULL, delimiters);
					p7 = strtok(NULL, delimiters);
					p8 = strtok(NULL, delimiters);
					p9 = strtok(NULL, delimiters);
					p10 = strtok(NULL, delimiters);
					
					/*
					if(!p10){
						printf("\nNot p10");
					}else{
						printf("\n%u", p9);
						printf("\n%u", p10);
						printf("\n%s", p10);
						
						printf("\n%u", p11);
						printf("\n%s", p11);
					}
					*/
					
					p11 = strtok(NULL, delimiters);
					p12 = strtok(NULL, delimiters);
					p13 = strtok(NULL, delimiters);
					p14 = strtok(NULL, delimiters);
					p15 = strtok(NULL, delimiters);
					p16 = strtok(NULL, delimiters);
					p17 = strtok(NULL, delimiters);
					p18 = strtok(NULL, delimiters);
					p19 = strtok(NULL, delimiters);
					
					//printf("\nParsing Complete\n");
					//printf("\n1: %d, \n2: %d, \n3: %d, \n4: %d, \n5: %d, \n6: %d, \n7: %d, \n8: %d, \n9: %d, \n10: %d, \n11: %d, \n12: %d, \n13: %d, \n14: %d, \n15: %d, \n16: %d, \n17: %d, \n18: %d, \n19: %d", (float)atoi(p1), (float)atoi(p2), (float)atoi(p3), (float)atoi(p4), (float)atoi(p5), (float)atoi(p6), (float)atoi(p7), (float)atoi(p8), (float)atoi(p9), (float)atoi(p10), (float)atoi(p11), (float)atoi(p12), (float)atoi(p13), (float)atoi(p14), (float)atoi(p15), (float)atoi(p16), (float)atoi(p17), (float)atoi(p18), (float)atoi(p19));
					//printf("\nSpeed: %d", )
					//printf("xyz: [%d, %d, %d] dir: [%d, %d, %d] config: [%d, %d, %d]\n", p2f, p3f, p4f, p5f, p6f, p7f, p8f, p9f, p10f);
					//printf("xyz: [%d, %d, %d] dir: [%d, %d, %d] config: [%d, %d, %d]\n", p11f, p12f, p13f, p14f, p15f, p16f, p17f, p18f, p19f);
					
					/*
					printf("\np1: %f", (float)atoi(p1));
					printf("\np2: %f", (float)atoi(p2));
					printf("\np3: %f", (float)atoi(p3));
					printf("\np4: %f", (float)atoi(p4));
					printf("\np5: %f", (float)atoi(p5));
					printf("\np6: %f", (float)atoi(p6));
					printf("\np7: %f", (float)atoi(p7));
					printf("\np8: %f", (float)atoi(p8));
					printf("\np9: %f", (float)atoi(p9));
					
					
					//printf("\np10: %f", (float)atoi(p10));
					printf("\np11: %f", (float)atoi(p11));
					printf("\np12: %f", (float)atoi(p12));
					printf("\np13: %f", (float)atoi(p13));
					printf("\np14: %f", (float)atoi(p14));
					printf("\np15: %f", (float)atoi(p15));
					printf("\np16: %f", (float)atoi(p16));
					printf("\np17: %f", (float)atoi(p7));
					printf("\np18: %f", (float)atoi(p18));
					printf("\np19: %f", (float)atoi(p19));
					
					*/
					
					
					double cart_speed = (float)atoi(p1);
					//printf("\n1");
					
					struct Vector my_point_start = new_vector((float)atoi(p2), (float)atoi(p3), (float)atoi(p4));
					//printf("\n2");
					struct Vector my_dir_start = new_vector((float)atoi(p5), (float)atoi(p6), (float)atoi(p7));
					//printf("\n3");
					struct Config my_config_start = new_config((bool)atoi(p8), (bool)atoi(p9), (bool)atoi(p10));
					//printf("\n4");
					
					/*
					printf("\ncart_speed: %f (microns/sec)\n", cart_speed);
					printf("\nmy_point_start: ");
					print_vector(my_point_start);
					printf("\nmy_dir_start: ");
					print_vector(my_dir_start);
					printf("\nBool input: %d, %d, %d\n", atoi(p17), atoi(p18), atoi(p19));
					printf("\nmy_config_start:" );
					print_config(my_config_start);
					*/
					struct XYZ xyz_start = new_XYZ(my_point_start, my_dir_start, my_config_start);
					
					
					//printf("\n5");
					struct Vector my_point_end = new_vector((float)atoi(p11), (float)atoi(p12), (float)atoi(p13));
					//printf("\n6");
					struct Vector my_dir_end = new_vector((float)atoi(p14), (float)atoi(p15), (float)atoi(p16));
					//printf("\n7");
					struct Config my_config_end = new_config((bool)atoi(p17), (bool)atoi(p18), (bool)atoi(p19));
					//printf("\n8");
					struct XYZ xyz_end = new_XYZ(my_point_end, my_dir_end, my_config_end);
					//printf("\n9");
					
					//printf("\nJangles: \n");
					//printf("[%d, %d, %d, %d, %d]", J1, J2, J3, J4, J5);
					//printf("\n");
					
					//printf("\n\nStarting MOVETOSTRAIGHT:");
					
					//print_config(my_config_start);
					/*
					printf("\nxyz_start: ");
					print_XYZ(xyz_start);
					
					printf("\n\nmy_point_end: ");
					print_vector(my_point_end);
					printf("\nmy_dir_end: ");
					print_vector(my_dir_end);
					printf("\nmy_config_end");
					print_config(my_config_end);
					printf("\nxyz_end: ");
					print_XYZ(xyz_end);
					printf("\n");
					*/					

					if (p1 != NULL && p2 != NULL && p3 != NULL && p4 != NULL && p5 != NULL){
						
						//printf("\n\nStarting MoveRobotStraight:\n");
						MoveRobotStraight(xyz_start, xyz_end, cart_speed);
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
					mapped[Add]=Start;
					//printf("\n %d %d \n",Add,Length);
					for(i=0;i<Length;i++)
					{
						for(j=0;j<Delay;j++)			
							mapped[Add]=Start+i;
					}
				break;
				case READ_CMD  :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					if((p1==NULL) | (p2==NULL))
					{
						//printf("\n %d %d need addres and length",atoi(p1),atoi(p2));
						break;
					}
					Add=atoi(p1);
					Length=atoi(p2); 
					//printf("\n %d %d \n",Add,Length);
					for(i=0;i<Length;i++)
					{
						

						//printf("\n  %x %d",CalTables[Add+i],CalTables[Add+i]);
						//printf("\n %x ",mapped[Add+i]);
					}
				break; 
				case WRITE_CMD  :
					p1=strtok (NULL, delimiters);
					p2=strtok (NULL, delimiters);
					if((p1==NULL) | (p2==NULL))
					{
						//printf("\n %d %d need addres and data",atoi(p1),atoi(p2));
					}
					//printf("\n %d %d \n",atoi(p1),atoi(p2));
					i=atoi(p1);
					j=atoi(p2);
					if(i==COMMAND_REG)
						CmdVal=j;
					mapped[i]=j;
					if(i==ACCELERATION_MAXSPEED)
					{
						maxSpeed=j & 0b00000000000011111111111111111111;
						coupledAcceleration=(j & 0b00000011111100000000000000000000) >> 20;
						//startSpeed=0,
						
					}
				break; 
				case SET_ALL_BOUNDRY  :
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
					BaseBoundryHigh=(int)((float)atoi(p1) * fabs(JointsCal[0]));
					BaseBoundryLow=(int)((float)atoi(p2) * fabs(JointsCal[0]));
					BDH=BaseBoundryHigh;
					BDL=BaseBoundryLow;
					Axis=BOUNDRY_BASE;
					mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
					PivotBoundryHigh=(int)((float)atoi(p3) * fabs(JointsCal[1]));
					PivotBoundryLow=(int)((float)atoi(p4) * fabs(JointsCal[1]));
					BDH=PivotBoundryHigh;
					BDL=PivotBoundryLow;
					Axis=BOUNDRY_PIVOT;
					mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
					EndBoundryHigh=(int)((float)atoi(p5) * fabs(JointsCal[2]));
					EndBoundryLow=(int)((float)atoi(p6) * fabs(JointsCal[2]));
					BDH=EndBoundryHigh;
					BDL=EndBoundryLow;
					Axis=BOUNDRY_END;
					mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
					AngleBoundryHigh=(int)((float)atoi(p7) * fabs(JointsCal[3]));
					AngleBoundryLow=(int)((float)atoi(p8) * fabs(JointsCal[3]));
					BDH=AngleBoundryHigh;
					BDL=AngleBoundryLow;
					Axis=BOUNDRY_ANGLE;
					mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
					RotateBoundryHigh=(int)((float)atoi(p9) * fabs(JointsCal[4]));
					RotateBoundryLow=(int)((float)atoi(p10) * fabs(JointsCal[4]));
					BDH=RotateBoundryHigh;
					BDL=RotateBoundryLow;
					Axis=BOUNDRY_ROT;
					mapped[Axis]=((BDH/8) << 16) | (0X0000FFFF & (BDL/8));
					
				break;
				
				
				case EXIT_CMD  :
					mapped[FINE_ADJUST_BASE]=0;
					mapped[FINE_ADJUST_END]=0;
					mapped[FINE_ADJUST_PIVOT]=0;
					mapped[FINE_ADJUST_ANGLE]=0;
					mapped[FINE_ADJUST_ROT]=0;
					return 1;
				break; 
				
		
				
				default : 
				  //printf("\nunrecognized command");
				  break;
			}
			return 0;
		}
		else
			return 1;
	}
	return 1;
}

int main(int argc, char *argv[]) {

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
  
  
  mfd = open("/dev/mem", O_RDWR);
  map_addrCt = mmap(NULL, CalTblSize, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, 0x3f000000);


   if (map_addrCt == MAP_FAILED) {
		//printf("\nCant Map Callibration Tables \n");
    perror("Failed to mmap High memory");
    return 1;
  }
  CalTables = map_addrCt;


//  Addr= = atoi(argv[3]);
//  Dta= = atoi(argv[4]);
//  mapped[Addr] = Dta;
	setDefaults(DefaultMode);
//	if(DefaultMode ==2 )
	if(ServerMode==1)
	{
	
		err = pthread_create(&(tid[0]), NULL, &StartServerSocket,  (void*)&ThreadsExit);
		if (err != 0)
		{
			//printf("\ncan't create thread :[%s]", strerror(err));
			return 1;
		}
		else
		{
			//printf("\n Begin Socket Server Thread\n");
		}
	}
	else if(ServerMode==2)
	{
	
		err = pthread_create(&(tid[1]), NULL, &StartClientSocket, NULL);
		if (err != 0)
		{
			//printf("\ncan't create thread :[%s]", strerror(err));
			return 1;
		}
		else
		{
			//printf("\n Begin Socket Client Thread\n");
		}
	}
	else if(ServerMode==3)
	{
		if(mapped[SENT_BASE_POSITION]!=0)
		{
	   		munmap(map_addr, size);
			munmap(map_addrCt, CalTblSize);
 			close(fd);
			close(mfd);
			return 0;   
		}
		mapped[BASE_POSITION]=1;
    	#ifndef NO_BOOT_DANCE
		
		
		/*
		//Wigglesworth Code Start
		int DEFAULT_MAXSPEED = 232642; // 30 (deg/s)
		int DEFAULT_STARTSPEED = 512; // .066 (deg/s) This is the smallest number allowed
		
		//Maxspeed
		mapped[ACCELERATION_MAXSPEED]=DEFAULT_MAXSPEED;
		maxSpeed=(DEFAULT_MAXSPEED) & 0b00000000000011111111111111111111;
		coupledAcceleration=((DEFAULT_MAXSPEED) & 0b00000011111100000000000000000000) >> 20;
		
		//Startspeed
		mapped[START_SPEED]=1 ^ DEFAULT_STARTSPEED;
		//Wigglesworth Code End
		*/

		MoveRobot(0,0,0,50000,50000,BLOCKING_MOVE);
	    MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
	    MoveRobot(0,0,0,50000,-50000,BLOCKING_MOVE);
	    MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
	    MoveRobot(0,0,0,50000,50000,BLOCKING_MOVE);
	    MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
	    MoveRobot(0,0,0,50000,-50000,BLOCKING_MOVE);
	    MoveRobot(0,0,0,0,0,BLOCKING_MOVE);
	    #endif	
		err = pthread_create(&(tid[0]), NULL, &StartServerSocketDDE,  (void*)&ThreadsExit);
		if (err != 0)
		{
			//printf("\ncan't create thread :[%s]", strerror(err));
			return 1;
		}
		else
		{
			//printf("\n Begin Socket Server Thread For DDE\n");
		}
	}
	if(RunMode==1 || RunMode==2)
	{
		err = pthread_create(&(tid[2]), NULL, &RealtimeMonitor, (void*)&ThreadsExit );
		if (err != 0)
		{
			//printf("\ncan't create thread :[%s]", strerror(err));
			return 1;
		}
		else
		{
			//printf("\n Begin Realtime Monitor Thread\n");
		}		
	}

	
	
    if(ServerMode==3)
	{
		while(1){} //loop forever
	}
	while(getInput()==0);
	ThreadsExit=0;
	sleep(1);


	//printf("\nExiting \n");

  munmap(map_addr, size);
  munmap(map_addrCt, CalTblSize);
  

  close(fd);
   close(mfd);
  return 0;
}
