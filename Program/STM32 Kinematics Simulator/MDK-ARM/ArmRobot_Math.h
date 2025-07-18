/*
	6-DOF Arm Manipulator Formula Library
	Author	: Alfonsus Giovanni Mahendra Putra
	Date		: Februari 2025
	Licensed Program
*/

#ifndef ARMROBOT_MATH_H
#define ARMROBOT_MATH_H

#include "main.h"
#include "math.h"
#include "string.h"
#include "stdbool.h"
 
#define M_PI			3.14159265358979323846
#define DEG2RAD 	(M_PI/180)

#define J1	0X00
#define J2	0x01
#define J3	0x02
#define J4	0x03
#define J5	0x04
#define J6	0x05

#define RX	0x00
#define RY	0x01
#define RZ	0x02

#define OUTPUT_MAIN		0x01	// Memilih variabel utama sebagai variabel hasil kalulasi
#define OUTPUT_SEC		0x02	// Memilih variabel sekunder sebagai variabel hasil kalkulasi

typedef struct{
	bool
	singularity;
	
	float
	dh_theta[6],	// Sudut rotasi Zi-1 dengan Z1 searah rotasi Zi
	dh_d[6],			// Jarak translasi sepanjang sumbu Z yang menhubungkan link satu dengan link yang lain
	dh_a[6],			// Jarak translasi sepanjang sumbu X antara sumubu Zi+1 dengan Zi
	dh_alpha[6],	// Sudut rotasi antara sumbu Zi-1 dengan Z1 searah rotasi Xi
	theta_cal[6],	// Sudut kalibrasi theta agar berada pada sudut 0 derajat
	
	tool_x,		// Koordinat toolframe pada sumbu x 
	tool_y,		// Koordinat toolframe pada sumbu y 
	tool_z,		// Koordinat toolframe pada sumbu z 
	tool_rx,	// Sudut rotasi toolframe pada sumbu x 
	tool_ry,	// Sudut rotasi toolframe pada sumbu y
	tool_rz,	// Sudut rotasi toolframe pada sumbu z 
	
	joint_ang_in[6],	// Masukan sudut setiap sendi
	
	bs_mat_trans[4][4],	// Matriks transformasi pada base
	j1_mat_trans[4][4],	// Matriks transformasi joint 1
	j2_mat_trans[4][4],	// Matriks transformasi joint 2
	j3_mat_trans[4][4],	// Matriks transformasi joint 3
	j4_mat_trans[4][4],	// Matriks transformasi joint 4
	j5_mat_trans[4][4],	// Matriks transformasi joint 5
	j6_mat_trans[4][4],	// Matriks transformasi joint 6
	tf_mat_trans[4][4], // Matriks transformasi pada toolframe
	
	trans_mat_0_1[4][4], 	// Matriks transformasi base frame ke frame 1
	trans_mat_0_2[4][4],	// Matriks transformasi base frame ke frame 2
	trans_mat_0_3[4][4],	// Matriks transformasi base frame ke frame 3
	trans_mat_0_4[4][4],	// Matriks transformasi base frame ke frame 4
	trans_mat_0_5[4][4],	// Matriks transformasi base frame ke frame 5
	trans_mat_0_6[4][4],	// Matriks transformasi base frame ke frame 6
	trans_mat_0_t[4][4],	// Matriks transformasi base frame ke toolframe
	
	axis_pos_out[3],	// Output posisi pada setiap sumbu
	axis_rot_rad[3],	// Output rotasi pada setiap sumbu
	axis_rot_out[3];	// Output rotasi pada setiap sumbu
	
	float
	trans_mat_1_t_rev[4][4], // Reverse matriks transformasi toolframe ke frame 1
	tf_mat_trans_inv[4][4],	 // Inverse matriks transformasi pada toolframe
	
	trans_mat_1_6_rev[4][4], // Reverse matriks transformasi frame 1 ke frame 6
	trans_mat_1_6_neg[4][4], // Negated matriks transformasi frame 1 ke frame 6 
	trans_mat_1_5_rev[4][4], // Reverse matriks transformasi frame 1 ke frame 5 (titik tengah lengan putar)
	rot_mat_1_6_rev[3][3],	 // Reverse matriks rotasi frame 1 ke frame 6
	
	j1_mat_trans_inv[4][4],	// Matriks transformasi setiap joint 1 pada inverse kinematic
	j2_mat_trans_inv[4][4],	// Matriks transformasi setiap joint 2 pada inverse kinematic
	j3_mat_trans_inv[4][4],	// Matriks transformasi setiap joint 3 pada inverse kinematic
	
	trans_mat_1_2_inv[4][4],	// Matriks transformasi frame 1 ke frame 2
	trans_mat_1_3_inv[4][4],	// Matriks transformasi frame 1 ke frame 3
	
	rot_mat_1_3_inv_T[3][3],	// Matriks rotasi dari matriks transformasi frame 1 ke frame 3
	rot_mat_3_6_inv[3][3],		// Matriks rotasi frame 3 ke frame 6
	
	x_j1_zero,	// Koordinat sumbu X saat joint 1 pada posisi zero
	y_j1_zero,	// Koordinat sumbu Z saat joint 1 pada posisi zero
	
	L1, L2, L3, 
	theta_b, theta_c, theta_d,
	
	j5_enc_angle,	// Sudut encoder pada Joint 5 
	joint_ang_out_rad[6],	// Keluaran sudut setiap sendi dalam radian
	joint_ang_out[6]; // Keluaran sudut setiap sendi dalam derajat
	
	float	
	j1_jcb_delta_pos[3],	// Delta posisi joint	1
	j2_jcb_delta_pos[3],	// Delta posisi joint	2
	j3_jcb_delta_pos[3],	// Delta posisi joint	3
	j4_jcb_delta_pos[3],	// Delta posisi joint	4
	j5_jcb_delta_pos[3],	// Delta posisi joint	5
	j6_jcb_delta_pos[3],	// Delta posisi joint	6
	
	j1_jcb_lin_vel[3], 	// Linear velocity joint 1
	j2_jcb_lin_vel[3],	// Linear velocity joint 2
	j3_jcb_lin_vel[3],	// Linear velocity joint 3
	j4_jcb_lin_vel[3],	// Linear velocity joint 4
	j5_jcb_lin_vel[3],	// Linear velocity joint 5
	j6_jcb_lin_vel[3],	// Linear velocity joint 6
		
	j1_jcb_ang_vel[3],	// Angular velocity joint 1
	j2_jcb_ang_vel[3],	// Angular velocity joint 2
	j3_jcb_ang_vel[3],	// Angular velocity joint 3
	j4_jcb_ang_vel[3],	// Angular velocity joint 4
	j5_jcb_ang_vel[3],	// Angular velocity joint 5
	j6_jcb_ang_vel[3],	// Angular velocity joint 6
	
	jacobian_matrix[6][6],	// Matrix jacobian
	jacobian_det; // Determinan dari matrix jacobian
	
	float
	calc_axis_pos[6],		// Variable untuk menyimpan hasil kalkulasi posisi tanpa mengubah variable utama
	calc_joint_ang[6];	// Variable untuk menyimpan hasil kalkulasi sudut tanpa mengubah variable utama
}Kinematics_t;

/* MATRIX FUNCTION ---------------------------------------------------------------------------------------------------------------------------------*/

/* Multiply Matrix 4x4 */
void multiply_4x4(float a[4][4], float b[4][4], float c[4][4]);
/* Multiply Matrix 3x3 */
void multiply_3x3(float a[3][3], float b[3][3], float c[3][3]);
/* Matrix 6x6 Determinant*/
float determinant_6x6(float A[6][6]);

/* -------------------------------------------------------------------------------------------------------------------------------------------------*/


/* FORWARD KINEMATICS ------------------------------------------------------------------------------------------------------------------------------*/

/* DH Parameter Variable Init */
void DHparam_init(Kinematics_t *param, float input_link_offset[6], float input_link_length[6], float input_link_twist[6]);
/* Toolframe Variable Init */
void toolframe_init(Kinematics_t *param, float x, float y, float z, float rx, float ry, float rz);
/* Matrix Transformation Calculation */
void forward_transform_matrix(Kinematics_t *param);
/* Calculate All Link Transformation*/
void calculate_all_link(Kinematics_t *param);
/* Forward Kinematics Calculation */
void run_forward_kinematic(Kinematics_t *param, volatile float joint_angle[6]);

/* -------------------------------------------------------------------------------------------------------------------------------------------------*/


/* INVERSE KINEMATICS ------------------------------------------------------------------------------------------------------------------------------*/

/* Invert Toolframe Axis Rotation Calculation */
float calculate_tf_rot_axis(Kinematics_t *param, uint8_t sel_axis);
/* Inverse Kinematic Parameter Variable Init */
void run_inverse_kinematic(Kinematics_t *param, float tool_axes[6]);

/* ------------------------------------------------------------------------------------------------------------------------------------------------*/


/* JACOBIAN MATRIX --------------------------------------------------------------------------------------------------------------------------------*/

// Find Jacobian Matrix Variables
void find_jacobian_variable(Kinematics_t *param);
// Check Sinuglarity
void check_singularity(Kinematics_t *param);

/* ------------------------------------------------------------------------------------------------------------------------------------------------*/


/* MINUS ZERO PREVENTION --------------------------------------------------------------------------------------------------------------------------*/
float sanitize_zero(float value);
/* ------------------------------------------------------------------------------------------------------------------------------------------------*/

#endif
