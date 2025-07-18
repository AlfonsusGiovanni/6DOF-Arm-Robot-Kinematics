/*
	6-DOF Arm Manipulator Formula Library
	Author	: Alfonsus Giovanni Mahendra Putra
	Date		: Februari 2025
	Licensed Program
*/

#include "ArmRobot_Math.h"

/* MATRIX MULTIPLY ---------------------------------------------------------------------------------------------------------------------------------*/

/* Multiply Matrix 4x4 */
void multiply_4x4(float a[4][4], float b[4][4], float c[4][4]){
	for(int i=0; i<4; i++){
		for (int j=0; j<4; j++){
			c[i][j] = 0;
			for (int k=0; k<4; k++){
				c[i][j] += a[i][k]*b[k][j];
			}
		}
	}
}

/* Multiply Matrix 3x3 */
void multiply_3x3(float a[3][3], float b[3][3], float c[3][3]){
	for(int i=0; i<3; i++){
		for (int j=0; j<3; j++){
			c[i][j] = 0;
			for (int k=0; k<3; k++){
				c[i][j] += a[i][k]*b[k][j];
			}
		}
	}
}

float determinant_6x6(float A[6][6]){
	float 
	U[6][6],
	L[6][6],
	det = 1.0;
	
	// Initialize L as identity and copy A to U
	for(int i=0; i<6; i++){
		for(int j=0; j<6; j++){
			if(i==j){
				L[i][j] = 1;
			}
			else{
				L[i][j] = 0;
			}
			U[i][j] = A[i][j];
		}
	}

	// Perform LU decomposition
	for(int i=0; i<6; i++) {
		for(int j=i+1; j<6; j++) {
			if(U[i][i] == 0) return 0;  // Singular matrix
			float factor = U[j][i] / U[i][i];
			L[j][i] = factor;
			for(int k=i; k<6; k++){
				U[j][k] -= factor * U[i][k];
			}
		}
	}

	// Determinant is product of diagonal elements of U
	for(int i=0; i<6; i++) {
		det *= U[i][i];
	}
	return det;
}

/* -------------------------------------------------------------------------------------------------------------------------------------------------*/


/* FORWARD KINEMATICS ------------------------------------------------------------------------------------------------------------------------------*/

/* DH Parameter Variable Init */
void DHparam_init(Kinematics_t *param, float input_link_offset[6], float input_link_length[6], float input_link_twist[6]){	
	// Set Calibration Value
	param->theta_cal[0] = 0;
	param->theta_cal[1] = -90;
	param->theta_cal[2] = 180;
	param->theta_cal[3] = 0;
	param->theta_cal[4] = 0;
	param->theta_cal[5] = 0;
	
	for(int i=0; i<6; i++){	
		param->dh_d[i] = input_link_offset[i];
		param->dh_a[i] = input_link_length[i];
		param->dh_alpha[i] = input_link_twist[i] * DEG2RAD;
	}
}

/* Toolframe Variable Init */
void toolframe_init(Kinematics_t *param, float x, float y, float z, float rx, float ry, float rz){
	param->tool_x = x;
	param->tool_y = y;
	param->tool_z = z;
	
	param->tool_rx = rx * DEG2RAD;
	param->tool_ry = ry * DEG2RAD;
	param->tool_rz = rz * DEG2RAD;
}

/* Matrix Transformation Calculation */
void forward_transform_matrix(Kinematics_t *param){
	// Base Frame Transformation Matrix
	param->bs_mat_trans[0][0] = 1;
	param->bs_mat_trans[0][1] = 0;
	param->bs_mat_trans[0][2] = 0;
	param->bs_mat_trans[0][3] = 0;
	
	param->bs_mat_trans[1][0] = 0;
	param->bs_mat_trans[1][1] = 1;
	param->bs_mat_trans[1][2] = 0;
	param->bs_mat_trans[1][3] = 0;
	
	param->bs_mat_trans[2][0] = 0;
	param->bs_mat_trans[2][1] = 0;
	param->bs_mat_trans[2][2] = 1;
	param->bs_mat_trans[2][3] = 0;
	
	param->bs_mat_trans[3][0] = 0;
	param->bs_mat_trans[3][1] = 0;
	param->bs_mat_trans[3][2] = 0;
	param->bs_mat_trans[3][3] = 1;
	
	// Joint 1 Transformation Matrix 
	param->j1_mat_trans[0][0] = sanitize_zero(cos(param->dh_theta[0]));												
	param->j1_mat_trans[0][1] = sanitize_zero(-sin(param->dh_theta[0])*cos(param->dh_alpha[0]));	
	param->j1_mat_trans[0][2] = sanitize_zero(sin(param->dh_theta[0])*sin(param->dh_alpha[0]));
	param->j1_mat_trans[0][3] = sanitize_zero(param->dh_a[0]*cos(param->dh_theta[0]));
	
	param->j1_mat_trans[1][0] = sanitize_zero(sin(param->dh_theta[0]));												
	param->j1_mat_trans[1][1] = sanitize_zero(cos(param->dh_theta[0])*cos(param->dh_alpha[0]));	
	param->j1_mat_trans[1][2] = sanitize_zero(-cos(param->dh_theta[0])*sin(param->dh_alpha[0]));
	param->j1_mat_trans[1][3] = sanitize_zero(param->dh_a[0]*sin(param->dh_theta[0]));
	
	param->j1_mat_trans[2][0] = 0.0; 																																				
	param->j1_mat_trans[2][1] = sanitize_zero(sin(param->dh_alpha[0]));
	param->j1_mat_trans[2][2] = sanitize_zero(cos(param->dh_alpha[0]));
	param->j1_mat_trans[2][3] = sanitize_zero(param->dh_d[0]);
	
	param->j1_mat_trans[3][0] = 0.0; 																																													
	param->j1_mat_trans[3][1] = 0.0;			
	param->j1_mat_trans[3][2] = 0.0;
	param->j1_mat_trans[3][3] = 1.0;	

	// Joint 2 Transformation Matrix 
	param->j2_mat_trans[0][0] = sanitize_zero(cos(param->dh_theta[1]));												
	param->j2_mat_trans[0][1] = sanitize_zero(-sin(param->dh_theta[1])*cos(param->dh_alpha[1]));	
	param->j2_mat_trans[0][2] = sanitize_zero(sin(param->dh_theta[1])*sin(param->dh_alpha[1]));
	param->j2_mat_trans[0][3] = sanitize_zero(param->dh_a[1]*cos(param->dh_theta[1]));
	
	param->j2_mat_trans[1][0] = sanitize_zero(sin(param->dh_theta[1]));												
	param->j2_mat_trans[1][1] = sanitize_zero(cos(param->dh_theta[1])*cos(param->dh_alpha[1]));	
	param->j2_mat_trans[1][2] = sanitize_zero(-cos(param->dh_theta[1])*sin(param->dh_alpha[1]));
	param->j2_mat_trans[1][3] = sanitize_zero(param->dh_a[1]*sin(param->dh_theta[1]));

	param->j2_mat_trans[2][0] = 0.0; 																																				
	param->j2_mat_trans[2][1] = sanitize_zero(sin(param->dh_alpha[1]));
	param->j2_mat_trans[2][2] = sanitize_zero(cos(param->dh_alpha[1]));
	param->j2_mat_trans[2][3] = sanitize_zero(param->dh_d[1]);
	
	param->j2_mat_trans[3][0] = 0.0; 																																													
	param->j2_mat_trans[3][1] = 0.0;			
	param->j2_mat_trans[3][2] = 0.0;
	param->j2_mat_trans[3][3] = 1.0;	
	
	// Joint 3 Transformation Matrix 
	param->j3_mat_trans[0][0] = sanitize_zero(cos(param->dh_theta[2]));												
	param->j3_mat_trans[0][1] = sanitize_zero(-sin(param->dh_theta[2])*cos(param->dh_alpha[2]));	
	param->j3_mat_trans[0][2] = sanitize_zero(sin(param->dh_theta[2])*sin(param->dh_alpha[2]));
	param->j3_mat_trans[0][3] = sanitize_zero(param->dh_a[2]*cos(param->dh_theta[2]));
	
	param->j3_mat_trans[1][0] = sanitize_zero(sin(param->dh_theta[2]));												
	param->j3_mat_trans[1][1] = sanitize_zero(cos(param->dh_theta[2])*cos(param->dh_alpha[2]));	
	param->j3_mat_trans[1][2] = sanitize_zero(-cos(param->dh_theta[2])*sin(param->dh_alpha[2]));
	param->j3_mat_trans[1][3] = sanitize_zero(param->dh_a[2]*sin(param->dh_theta[2]));

	param->j3_mat_trans[2][0] = 0.0; 																																				
	param->j3_mat_trans[2][1] = sanitize_zero(sin(param->dh_alpha[2]));
	param->j3_mat_trans[2][2] = sanitize_zero(cos(param->dh_alpha[2]));
	param->j3_mat_trans[2][3] = sanitize_zero(param->dh_d[2]);
	
	param->j3_mat_trans[3][0] = 0.0; 																																													
	param->j3_mat_trans[3][1] = 0.0;			
	param->j3_mat_trans[3][2] = 0.0;
	param->j3_mat_trans[3][3] = 1.0;

	// Joint 4 Transformation Matrix 
	param->j4_mat_trans[0][0] = sanitize_zero(cos(param->dh_theta[3]));												
	param->j4_mat_trans[0][1] = sanitize_zero(-sin(param->dh_theta[3])*cos(param->dh_alpha[3]));	
	param->j4_mat_trans[0][2] = sanitize_zero(sin(param->dh_theta[3])*sin(param->dh_alpha[3]));
	param->j4_mat_trans[0][3] = sanitize_zero(param->dh_a[3]*cos(param->dh_theta[3]));

	param->j4_mat_trans[1][0] = sanitize_zero(sin(param->dh_theta[3]));												
	param->j4_mat_trans[1][1] = sanitize_zero(cos(param->dh_theta[3])*cos(param->dh_alpha[3]));	
	param->j4_mat_trans[1][2] = sanitize_zero(-cos(param->dh_theta[3])*sin(param->dh_alpha[3]));
	param->j4_mat_trans[1][3] = sanitize_zero(param->dh_a[3]*sin(param->dh_theta[3]));

	param->j4_mat_trans[2][0] = 0.0; 																																				
	param->j4_mat_trans[2][1] = sanitize_zero(sin(param->dh_alpha[3]));
	param->j4_mat_trans[2][2] = sanitize_zero(cos(param->dh_alpha[3]));
	param->j4_mat_trans[2][3] = sanitize_zero(param->dh_d[3]);
	
	param->j4_mat_trans[3][0] = 0.0; 																																													
	param->j4_mat_trans[3][1] = 0.0;		
	param->j4_mat_trans[3][2] = 0.0;
	param->j4_mat_trans[3][3] = 1,0;
	
	// Joint 5 Transformation Matrix 
	param->j5_mat_trans[0][0] = sanitize_zero(cos(param->dh_theta[4]));												
	param->j5_mat_trans[0][1] = sanitize_zero(-sin(param->dh_theta[4])*cos(param->dh_alpha[4]));	
	param->j5_mat_trans[0][2] = sanitize_zero(sin(param->dh_theta[4])*sin(param->dh_alpha[4]));
	param->j5_mat_trans[0][3] = sanitize_zero(param->dh_a[4]*cos(param->dh_theta[4]));

	param->j5_mat_trans[1][0] = sanitize_zero(sin(param->dh_theta[4]));												
	param->j5_mat_trans[1][1] = sanitize_zero(cos(param->dh_theta[4])*cos(param->dh_alpha[4]));	
	param->j5_mat_trans[1][2] = sanitize_zero(-cos(param->dh_theta[4])*sin(param->dh_alpha[4]));
	param->j5_mat_trans[1][3] = sanitize_zero(param->dh_a[4]*sin(param->dh_theta[4]));

	param->j5_mat_trans[2][0] = 0.0; 																																				
	param->j5_mat_trans[2][1] = sanitize_zero(sin(param->dh_alpha[4]));
	param->j5_mat_trans[2][2] = sanitize_zero(cos(param->dh_alpha[4]));
	param->j5_mat_trans[2][3] = sanitize_zero(param->dh_d[4]);
	
	param->j5_mat_trans[3][0] = 0.0; 																																													
	param->j5_mat_trans[3][1] = 0.0;			
	param->j5_mat_trans[3][2] = 0.0;
	param->j5_mat_trans[3][3] = 1.0;
	
	// Joint 6 Transformation Matrix 
	param->j6_mat_trans[0][0] = sanitize_zero(cos(param->dh_theta[5]));												
	param->j6_mat_trans[0][1] = sanitize_zero(-sin(param->dh_theta[5])*cos(param->dh_alpha[5]));	
	param->j6_mat_trans[0][2] = sanitize_zero(sin(param->dh_theta[5])*sin(param->dh_alpha[5]));
	param->j6_mat_trans[0][3] = sanitize_zero(param->dh_a[5]*cos(param->dh_theta[5]));

	param->j6_mat_trans[1][0] = sanitize_zero(sin(param->dh_theta[5]));												
	param->j6_mat_trans[1][1] = sanitize_zero(cos(param->dh_theta[5])*cos(param->dh_alpha[5]));	
	param->j6_mat_trans[1][2] = sanitize_zero(-cos(param->dh_theta[5])*sin(param->dh_alpha[5]));
	param->j6_mat_trans[1][3] = sanitize_zero(param->dh_a[5]*sin(param->dh_theta[5]));

	param->j6_mat_trans[2][0] = 0.0; 																																				
	param->j6_mat_trans[2][1] = sanitize_zero(sin(param->dh_alpha[5]));
	param->j6_mat_trans[2][2] = sanitize_zero(cos(param->dh_alpha[5]));
	param->j6_mat_trans[2][3] = sanitize_zero(param->dh_d[5]);
	
	param->j6_mat_trans[3][0] = 0.0; 																																													
	param->j6_mat_trans[3][1] = 0.0;			
	param->j6_mat_trans[3][2] = 0.0;
	param->j6_mat_trans[3][3] = 1.0;
	
	// Toolframe Transformation Matrix
	param->tf_mat_trans[0][0] = sanitize_zero(cos(param->tool_rz)*cos(param->tool_ry));
	param->tf_mat_trans[0][1] = sanitize_zero(cos(param->tool_rz)*sin(param->tool_ry)*sin(param->tool_rx) - sin(param->tool_rz)*cos(param->tool_rx));
	param->tf_mat_trans[0][2] = sanitize_zero(cos(param->tool_rz)*sin(param->tool_ry)*cos(param->tool_rx) + sin(param->tool_rz)*sin(param->tool_rx));
	param->tf_mat_trans[0][3] = sanitize_zero(param->tool_x);
	
	param->tf_mat_trans[1][0] = sanitize_zero(sin(param->tool_rz)*cos(param->tool_ry));
	param->tf_mat_trans[1][1] = sanitize_zero(sin(param->tool_rz)*sin(param->tool_ry)*sin(param->tool_rx) + cos(param->tool_rz)*cos(param->tool_rx));
	param->tf_mat_trans[1][2] = sanitize_zero(sin(param->tool_rz)*sin(param->tool_ry)*cos(param->tool_rx) - cos(param->tool_rz)*sin(param->tool_rx));
	param->tf_mat_trans[1][3] = sanitize_zero(param->tool_y);
	
	param->tf_mat_trans[2][0] = sanitize_zero(-sin(param->tool_ry));
	param->tf_mat_trans[2][1] = sanitize_zero(cos(param->tool_ry)*sin(param->tool_rx));
	param->tf_mat_trans[2][2] = sanitize_zero(cos(param->tool_ry)*cos(param->tool_rx));
	param->tf_mat_trans[2][3] = sanitize_zero(param->tool_z);
	
	param->tf_mat_trans[3][0] = 0.0;
	param->tf_mat_trans[3][1] = 0.0;
	param->tf_mat_trans[3][2] = 0.0;
	param->tf_mat_trans[3][3] = 1.0;
}

/* Calculate All Link Transformation*/
void calculate_all_link(Kinematics_t *param){
	// Find Transformation Matrix From Base Frame to Joint 2
	multiply_4x4(param->bs_mat_trans, param->j1_mat_trans, param->trans_mat_0_1);
	
	// Find Transformation Matrix From Joint 1 to Joint 2
	multiply_4x4(param->trans_mat_0_1, param->j2_mat_trans, param->trans_mat_0_2);
	
	// Find Transformation Matrix From Joint 1 to Joint 3
	multiply_4x4(param->trans_mat_0_2, param->j3_mat_trans, param->trans_mat_0_3);
	
	// Find Transformation Matrix From Joint 1 to Joint 4
	multiply_4x4(param->trans_mat_0_3, param->j4_mat_trans, param->trans_mat_0_4);
	
	// Find Transformation Matrix From Joint 1 to Joint 
	multiply_4x4(param->trans_mat_0_4, param->j5_mat_trans, param->trans_mat_0_5);
	
	// Find Transformation Matrix From Joint 1 to Joint 6
	multiply_4x4(param->trans_mat_0_5, param->j6_mat_trans, param->trans_mat_0_6);
	
	// Find Transformation Matrix From Joint 1 to Toolframe
	multiply_4x4(param->trans_mat_0_6, param->tf_mat_trans, param->trans_mat_0_t);
	
	// Get Toolframe Cartesian Position
	param->axis_pos_out[0] = param->trans_mat_0_t[0][3]; 	// X axis
	param->axis_pos_out[1] = param->trans_mat_0_t[1][3];	// Y axis
	param->axis_pos_out[2] = param->trans_mat_0_t[2][3];	// Z axis
	
	// Get Toolframe Axis Rotation
	param->axis_rot_rad[1] = atan2(-param->trans_mat_0_t[2][0], sqrt(pow(param->trans_mat_0_t[0][0], 2) + pow(param->trans_mat_0_t[1][0], 2))); 			// Y axis
	param->axis_rot_rad[0] = atan2(param->trans_mat_0_t[2][1]/cos(param->axis_rot_rad[1]), param->trans_mat_0_t[2][2]/cos(param->axis_rot_rad[1]));		// X axis
	param->axis_rot_rad[2] = atan2(param->trans_mat_0_t[1][0]/cos(param->axis_rot_rad[1]), param->trans_mat_0_t[0][0]/cos(param->axis_rot_rad[1]));		// Z axis
	
	param->axis_rot_out[0] = param->axis_rot_rad[0] / DEG2RAD;
	param->axis_rot_out[1] = param->axis_rot_rad[1] / DEG2RAD;
	param->axis_rot_out[2] = param->axis_rot_rad[2] / DEG2RAD;
}

/* Forward Kinematics Calculation */
void run_forward_kinematic(Kinematics_t *param, volatile float joint_angle[6]){
	for(int i=0; i<6; i++){	
		param->joint_ang_in[i] = joint_angle[i];
		param->dh_theta[i] = (joint_angle[i] + param->theta_cal[i]) * DEG2RAD;
	}
	forward_transform_matrix(param);
	calculate_all_link(param);
}

/* -------------------------------------------------------------------------------------------------------------------------------------------------*/


/* INVERSE KINEMATICS ------------------------------------------------------------------------------------------------------------------------------*/

/* Invert Toolframe Axis Rotation Calculation */
float calculate_tf_rot_axis(Kinematics_t *param, uint8_t sel_axis){
	float 
	rx_val[3],
	ry_val[3],
	rz_val[3],
	value;
	
	if(sel_axis == RX){
		rx_val[0] = param->tf_mat_trans_inv[0][0]*param->tf_mat_trans[0][3];
		rx_val[1] = param->tf_mat_trans_inv[0][1]*param->tf_mat_trans[1][3];
		rx_val[2] =	param->tf_mat_trans_inv[0][2]*param->tf_mat_trans[2][3];
		value = rx_val[0] + rx_val[1] + rx_val[2];
	}
		
	if(sel_axis == RY){
		ry_val[0] = param->tf_mat_trans_inv[1][0]*param->tf_mat_trans[0][3];
		ry_val[1] = param->tf_mat_trans_inv[1][1]*param->tf_mat_trans[1][3];
		ry_val[2] =	param->tf_mat_trans_inv[1][2]*param->tf_mat_trans[2][3];
		value = ry_val[0] + ry_val[1] + ry_val[2];
	}
		
	if(sel_axis == RZ){
		rz_val[0] = param->tf_mat_trans_inv[2][0]*param->tf_mat_trans[0][3];
		rz_val[1] = param->tf_mat_trans_inv[2][1]*param->tf_mat_trans[1][3];
		rz_val[2] =	param->tf_mat_trans_inv[2][2]*param->tf_mat_trans[2][3];
		value = rz_val[0] + rz_val[1] + rz_val[2];
	}
	
	return value;
}

/* Inverse Kinematics Calculation */
void run_inverse_kinematic(Kinematics_t *param, float tool_axes[6]){
	// Spherical Wirst Transformation Matrix
	param->trans_mat_1_t_rev[0][0] = cos(tool_axes[5]*DEG2RAD)*cos(tool_axes[4]*DEG2RAD);
	param->trans_mat_1_t_rev[0][1] = cos(tool_axes[5]*DEG2RAD)*sin(tool_axes[4]*DEG2RAD)*sin(tool_axes[3]*DEG2RAD) - sin(tool_axes[5]*DEG2RAD)*cos(tool_axes[3]*DEG2RAD);
	param->trans_mat_1_t_rev[0][2] = cos(tool_axes[5]*DEG2RAD)*sin(tool_axes[4]*DEG2RAD)*cos(tool_axes[3]*DEG2RAD) + sin(tool_axes[5]*DEG2RAD)*sin(tool_axes[3]*DEG2RAD);
	param->trans_mat_1_t_rev[0][3] = tool_axes[0];
	
	param->trans_mat_1_t_rev[1][0] = sin(tool_axes[5]*DEG2RAD)*cos(tool_axes[4]*DEG2RAD);
	param->trans_mat_1_t_rev[1][1] = sin(tool_axes[5]*DEG2RAD)*sin(tool_axes[4]*DEG2RAD)*sin(tool_axes[3]*DEG2RAD) + cos(tool_axes[5]*DEG2RAD)*cos(tool_axes[3]*DEG2RAD);
	param->trans_mat_1_t_rev[1][2] = sin(tool_axes[5]*DEG2RAD)*sin(tool_axes[4]*DEG2RAD)*cos(tool_axes[3]*DEG2RAD) - cos(tool_axes[5]*DEG2RAD)*sin(tool_axes[3]*DEG2RAD);
	param->trans_mat_1_t_rev[1][3] = tool_axes[1];
	
	param->trans_mat_1_t_rev[2][0] = -sin(tool_axes[4]*DEG2RAD);
	param->trans_mat_1_t_rev[2][1] = cos(tool_axes[4]*DEG2RAD)*sin(tool_axes[3]*DEG2RAD);
	param->trans_mat_1_t_rev[2][2] = cos(tool_axes[4]*DEG2RAD)*cos(tool_axes[3]*DEG2RAD);
	param->trans_mat_1_t_rev[2][3] = tool_axes[2];
	
	param->trans_mat_1_t_rev[3][0] = 0;
	param->trans_mat_1_t_rev[3][1] = 0;
	param->trans_mat_1_t_rev[3][2] = 0;
	param->trans_mat_1_t_rev[3][3] = 1;
	
	// Inverted Toolframe Transformation Matrix
	param->tf_mat_trans_inv[0][0] = param->tf_mat_trans[0][0];
	param->tf_mat_trans_inv[0][1] = param->tf_mat_trans[1][0];
	param->tf_mat_trans_inv[0][2] = param->tf_mat_trans[2][0];
	param->tf_mat_trans_inv[0][3] = calculate_tf_rot_axis(param, RX);
	
	param->tf_mat_trans_inv[1][0] = param->tf_mat_trans[0][1];
	param->tf_mat_trans_inv[1][1] = param->tf_mat_trans[1][1];
	param->tf_mat_trans_inv[1][2] = param->tf_mat_trans[1][2];
	param->tf_mat_trans_inv[1][3] =	calculate_tf_rot_axis(param, RY);
	
	param->tf_mat_trans_inv[2][0] =	param->tf_mat_trans[0][2];
	param->tf_mat_trans_inv[2][1] = param->tf_mat_trans[1][2];
	param->tf_mat_trans_inv[2][2] = param->tf_mat_trans[2][2];
	param->tf_mat_trans_inv[2][3] = calculate_tf_rot_axis(param, RZ);
	
	param->tf_mat_trans_inv[3][0] = 0;
	param->tf_mat_trans_inv[3][1] = 0;
	param->tf_mat_trans_inv[3][2] =	0;
	param->tf_mat_trans_inv[3][3] = 1;
	
	// Find Inverted Transformation Matrix From Joint 1 to Joint 6
	multiply_4x4(param->trans_mat_1_t_rev, param->tf_mat_trans_inv, param->trans_mat_1_6_rev);
	
	// Negated Transformation Matrix From Joint 1 to Joint 6
	param->trans_mat_1_6_neg[0][0] = 1;
	param->trans_mat_1_6_neg[0][1] = 0;
	param->trans_mat_1_6_neg[0][2] = 0;
	param->trans_mat_1_6_neg[0][3] = 0;
	
	param->trans_mat_1_6_neg[1][0] = 0;
	param->trans_mat_1_6_neg[1][1] = 1;
	param->trans_mat_1_6_neg[1][2] = 0;
	param->trans_mat_1_6_neg[1][3] = 0;
	
	param->trans_mat_1_6_neg[2][0] = 0;
	param->trans_mat_1_6_neg[2][1] = 0;
	param->trans_mat_1_6_neg[2][2] = 1;
	param->trans_mat_1_6_neg[2][3] = -param->dh_d[5];
	
	param->trans_mat_1_6_neg[3][0] = 0;
	param->trans_mat_1_6_neg[3][1] = 0;
	param->trans_mat_1_6_neg[3][2] = 0;
	param->trans_mat_1_6_neg[3][3] = 1;
	
	// Find Inverted Rotational Matrix From Joint 1 to Joint 5 (Wirst Position)
	multiply_4x4(param->trans_mat_1_6_rev, param->trans_mat_1_6_neg, param->trans_mat_1_5_rev);
	
	// Calculate J1 Angle	
	param->joint_ang_out_rad[0] = atan2(param->trans_mat_1_5_rev[1][3], param->trans_mat_1_5_rev[0][3]);
	param->joint_ang_out[0] = param->joint_ang_out_rad[0] / DEG2RAD;
	
	// Calculate X and Y pos From Joint 1 to Joint 5 at J1 zero
	param->x_j1_zero = param->trans_mat_1_5_rev[0][3]*cos(-param->joint_ang_out_rad[0]) - param->trans_mat_1_5_rev[1][3]*sin(-param->joint_ang_out_rad[0]);
	param->y_j1_zero = param->trans_mat_1_5_rev[1][3]*cos(-param->joint_ang_out_rad[0]) - param->trans_mat_1_5_rev[0][3]*sin(-param->joint_ang_out_rad[0]);
	
	// Find L1, L2, and L3
	param->L1 = param->x_j1_zero - param->dh_a[0];
	param->L3 = param->trans_mat_1_5_rev[2][3] - param->dh_d[0];
	param->L2 = sqrt(pow(param->L1,2) + pow(param->L3,2));
	
	// Find Theta B, C, and D
	param->theta_b = atan2(param->L3, param->L1)/DEG2RAD;
	param->theta_c = acos((pow(param->dh_a[1],2) + pow(param->L2,2) - pow(param->dh_d[3],2))/(2*param->dh_a[1]*param->L2))/DEG2RAD;
	param->theta_d = acos((pow(param->dh_a[1],2) + pow(param->dh_d[3],2) - pow(param->L2,2))/(2*param->dh_a[1]*param->dh_d[3]))/DEG2RAD;
	
	// Calculate J2 Angle
	param->joint_ang_out[1] = 90 - (param->theta_b + param->theta_c);
	param->joint_ang_out_rad[1] = param->joint_ang_out[1] * DEG2RAD;
	
	// Calculate J3 Angle
	param->joint_ang_out[2] = -param->theta_d + 90;
	param->joint_ang_out_rad[2] = param->joint_ang_out[2] * DEG2RAD;
	
	// Joint 1 Transformation Matrix 
	param->j1_mat_trans_inv[0][0] = cos(param->joint_ang_out_rad[0]);												
	param->j1_mat_trans_inv[0][1] = -sin(param->joint_ang_out_rad[0])*cos(param->dh_alpha[0]);	
	param->j1_mat_trans_inv[0][2] = sin(param->joint_ang_out_rad[0])*sin(param->dh_alpha[0]);
	param->j1_mat_trans_inv[0][3] = param->dh_a[0]*cos(param->joint_ang_out_rad[0]);
	
	param->j1_mat_trans_inv[1][0] = sin(param->joint_ang_out_rad[0]);												
	param->j1_mat_trans_inv[1][1] = cos(param->joint_ang_out_rad[0])*cos(param->dh_alpha[0]);	
	param->j1_mat_trans_inv[1][2] = -cos(param->joint_ang_out_rad[0])*sin(param->dh_alpha[0]);
	param->j1_mat_trans_inv[1][3] = param->dh_a[0]*sin(param->joint_ang_out_rad[0]);
	
	param->j1_mat_trans_inv[2][0] = 0; 																																				
	param->j1_mat_trans_inv[2][1] = sin(param->dh_alpha[0]);
	param->j1_mat_trans_inv[2][2] = cos(param->dh_alpha[0]);
	param->j1_mat_trans_inv[2][3] = param->dh_d[0];
	
	param->j1_mat_trans_inv[3][0] = 0; 																																													
	param->j1_mat_trans_inv[3][1] = 0;			
	param->j1_mat_trans_inv[3][2] = 0;
	param->j1_mat_trans_inv[3][3] = 1;
	
	// Joint 2 Transformation Matrix 
	param->j2_mat_trans_inv[0][0] = cos(param->joint_ang_out_rad[1] - 90*DEG2RAD);												
	param->j2_mat_trans_inv[0][1] = -sin(param->joint_ang_out_rad[1] - 90*DEG2RAD)*cos(param->dh_alpha[1]);	
	param->j2_mat_trans_inv[0][2] = sin(param->joint_ang_out_rad[1] - 90*DEG2RAD)*sin(param->dh_alpha[1]);
	param->j2_mat_trans_inv[0][3] = param->dh_a[1]*cos(param->joint_ang_out_rad[1] - 90*DEG2RAD);
	
	param->j2_mat_trans_inv[1][0] = sin(param->joint_ang_out_rad[1] - 90*DEG2RAD);												
	param->j2_mat_trans_inv[1][1] = cos(param->joint_ang_out_rad[1] - 90*DEG2RAD)*cos(param->dh_alpha[1]);	
	param->j2_mat_trans_inv[1][2] = -cos(param->joint_ang_out_rad[1] - 90*DEG2RAD)*sin(param->dh_alpha[1]);
	param->j2_mat_trans_inv[1][3] = param->dh_a[1]*sin(param->joint_ang_out_rad[1] - 90*DEG2RAD);
	
	param->j2_mat_trans_inv[2][0] = 0; 																																				
	param->j2_mat_trans_inv[2][1] = sin(param->dh_alpha[1]);
	param->j2_mat_trans_inv[2][2] = cos(param->dh_alpha[1]);
	param->j2_mat_trans_inv[2][3] = param->dh_d[1];
	
	param->j2_mat_trans_inv[3][0] = 0; 																																													
	param->j2_mat_trans_inv[3][1] = 0;			
	param->j2_mat_trans_inv[3][2] = 0;
	param->j2_mat_trans_inv[3][3] = 1;
	
	// Joint 3 Transformation Matrix 
	param->j3_mat_trans_inv[0][0] = cos(param->joint_ang_out_rad[2] + 180*DEG2RAD);												
	param->j3_mat_trans_inv[0][1] = -sin(param->joint_ang_out_rad[2] + 180*DEG2RAD)*cos(param->dh_alpha[2]);	
	param->j3_mat_trans_inv[0][2] = sin(param->joint_ang_out_rad[2] + 180*DEG2RAD)*sin(param->dh_alpha[2]);
	param->j3_mat_trans_inv[0][3] = param->dh_a[2]*cos(param->joint_ang_out_rad[2] + 180*DEG2RAD);
	
	param->j3_mat_trans_inv[1][0] = sin(param->joint_ang_out_rad[2] + 180*DEG2RAD);												
	param->j3_mat_trans_inv[1][1] = cos(param->joint_ang_out_rad[2] + 180*DEG2RAD)*cos(param->dh_alpha[2]);	
	param->j3_mat_trans_inv[1][2] = -cos(param->joint_ang_out_rad[2] + 180*DEG2RAD)*sin(param->dh_alpha[2]);
	param->j3_mat_trans_inv[1][3] = param->dh_a[2]*sin(param->joint_ang_out_rad[2] + 180*DEG2RAD);
	
	param->j3_mat_trans_inv[2][0] = 0; 																																				
	param->j3_mat_trans_inv[2][1] = sin(param->dh_alpha[2]);
	param->j3_mat_trans_inv[2][2] = cos(param->dh_alpha[2]);
	param->j3_mat_trans_inv[2][3] = param->dh_d[2];
	
	param->j3_mat_trans_inv[3][0] = 0; 																																													
	param->j3_mat_trans_inv[3][1] = 0;			
	param->j3_mat_trans_inv[3][2] = 0;
	param->j3_mat_trans_inv[3][3] = 1;
	
	// Find Transformation Matrix From Joint 1 to Joint 2
	multiply_4x4(param->j1_mat_trans_inv, param->j2_mat_trans_inv, param->trans_mat_1_2_inv);
	
	// Find Transformation Matrix From Joint 1 to Joint 3
	multiply_4x4(param->trans_mat_1_2_inv, param->j3_mat_trans_inv, param->trans_mat_1_3_inv);
	
	// Rotational Matrix From Transformation Matrix Joint 1 to Joint 6
	param->rot_mat_1_6_rev[0][0] = param->trans_mat_1_6_rev[0][0];
	param->rot_mat_1_6_rev[0][1] = param->trans_mat_1_6_rev[0][1];
	param->rot_mat_1_6_rev[0][2] = param->trans_mat_1_6_rev[0][2];
	
	param->rot_mat_1_6_rev[1][0] = param->trans_mat_1_6_rev[1][0];
	param->rot_mat_1_6_rev[1][1] = param->trans_mat_1_6_rev[1][1];
	param->rot_mat_1_6_rev[1][2] = param->trans_mat_1_6_rev[1][2];
	
	param->rot_mat_1_6_rev[2][0] = param->trans_mat_1_6_rev[2][0];
	param->rot_mat_1_6_rev[2][1] = param->trans_mat_1_6_rev[2][1];
	param->rot_mat_1_6_rev[2][2] = param->trans_mat_1_6_rev[2][2];
	
	// Transposed Rotational Matrix From Transformation Matrix Joint 1 to Joint 3
	param->rot_mat_1_3_inv_T[0][0] = param->trans_mat_1_3_inv[0][0];
	param->rot_mat_1_3_inv_T[0][1] = param->trans_mat_1_3_inv[1][0];
	param->rot_mat_1_3_inv_T[0][2] = param->trans_mat_1_3_inv[2][0];
	
	param->rot_mat_1_3_inv_T[1][0] = param->trans_mat_1_3_inv[0][1];
	param->rot_mat_1_3_inv_T[1][1] = param->trans_mat_1_3_inv[1][1];
	param->rot_mat_1_3_inv_T[1][2] = param->trans_mat_1_3_inv[2][1];
	
	param->rot_mat_1_3_inv_T[2][0] = param->trans_mat_1_3_inv[0][2];
	param->rot_mat_1_3_inv_T[2][1] = param->trans_mat_1_3_inv[1][2];
	param->rot_mat_1_3_inv_T[2][2] = param->trans_mat_1_3_inv[2][2];
	
	// Rotational Matrix From Joint 3 to Joint 6
	multiply_3x3(param->rot_mat_1_3_inv_T, param->rot_mat_1_6_rev, param->rot_mat_3_6_inv);
	
	// Calculate Joint 5 Angle
	if(param->j5_enc_angle > 0){
		param->joint_ang_out[4] = atan2(sqrt(1-pow(param->rot_mat_3_6_inv[2][2],2)), param->rot_mat_3_6_inv[2][2])/DEG2RAD;
	}
	else{
		param->joint_ang_out[4] = atan2(-sqrt(1-pow(param->rot_mat_3_6_inv[2][2],2)), param->rot_mat_3_6_inv[2][2])/DEG2RAD;
	}
	
	// Calculate Joint 4 & Joint 6 Angle
	if(param->joint_ang_out[4] > 0){
		param->joint_ang_out[3] = atan2(param->rot_mat_3_6_inv[1][2], param->rot_mat_3_6_inv[0][2])/DEG2RAD;
		param->joint_ang_out[5] = atan2(param->rot_mat_3_6_inv[2][1], -param->rot_mat_3_6_inv[2][0])/DEG2RAD;
	}
	else{
		param->joint_ang_out[3] = atan2(-param->rot_mat_3_6_inv[1][2], -param->rot_mat_3_6_inv[0][2])/DEG2RAD;
		param->joint_ang_out[5] = atan2(-param->rot_mat_3_6_inv[2][1], param->rot_mat_3_6_inv[2][0])/DEG2RAD;
	}
}
/*  -------------------------------------------------------------------------------------------------------------------------------------------------*/


/* JACOBIAN MATRIX ----------------------------------------------------------------------------------------------------------------------------------*/

// Find Jacobian Matrix Variables
void find_jacobian_variable(Kinematics_t *param){
	// Joint 1 Jacobian Delta Origin Position
	param->j1_jcb_delta_pos[0] = param->trans_mat_0_t[0][3] - param->trans_mat_0_1[0][3];
	param->j1_jcb_delta_pos[1] = param->trans_mat_0_t[1][3] - param->trans_mat_0_1[1][3];
	param->j1_jcb_delta_pos[2] = param->trans_mat_0_t[2][3] - param->trans_mat_0_1[2][3];
	
	// Joint 2 Jacobian Delta Origin Position
	param->j2_jcb_delta_pos[0] = param->trans_mat_0_t[0][3] - param->trans_mat_0_2[0][3];
	param->j2_jcb_delta_pos[1] = param->trans_mat_0_t[1][3] - param->trans_mat_0_2[1][3];
	param->j2_jcb_delta_pos[2] = param->trans_mat_0_t[2][3] - param->trans_mat_0_2[2][3];
	
	// Joint 3 Jacobian Delta Origin Position
	param->j3_jcb_delta_pos[0] = param->trans_mat_0_t[0][3] - param->trans_mat_0_3[0][3];
	param->j3_jcb_delta_pos[1] = param->trans_mat_0_t[1][3] - param->trans_mat_0_3[1][3];
	param->j3_jcb_delta_pos[2] = param->trans_mat_0_t[2][3] - param->trans_mat_0_3[2][3];
	
	// Joint 4 Jacobian Delta Origin Position
	param->j4_jcb_delta_pos[0] = param->trans_mat_0_t[0][3] - param->trans_mat_0_4[0][3];
	param->j4_jcb_delta_pos[1] = param->trans_mat_0_t[1][3] - param->trans_mat_0_4[1][3];
	param->j4_jcb_delta_pos[2] = param->trans_mat_0_t[2][3] - param->trans_mat_0_4[2][3];
	
	// Joint 5 Jacobian Delta Origin Position
	param->j5_jcb_delta_pos[0] = param->trans_mat_0_t[0][3] - param->trans_mat_0_5[0][3];
	param->j5_jcb_delta_pos[1] = param->trans_mat_0_t[1][3] - param->trans_mat_0_5[1][3];
	param->j5_jcb_delta_pos[2] = param->trans_mat_0_t[2][3] - param->trans_mat_0_5[2][3];
	
	// Joint 6 Jacobian Delta Origin Position
	param->j6_jcb_delta_pos[0] = param->trans_mat_0_t[0][3] - param->trans_mat_0_6[0][3];
	param->j6_jcb_delta_pos[1] = param->trans_mat_0_t[1][3] - param->trans_mat_0_6[1][3];
	param->j6_jcb_delta_pos[2] = param->trans_mat_0_t[2][3] - param->trans_mat_0_6[2][3];
	
	// Joint 1 Jacobian Linear Velocity
	param->j1_jcb_lin_vel[0] = param->trans_mat_0_1[1][2] * param->j1_jcb_delta_pos[2] - param->trans_mat_0_1[2][2] * param->j1_jcb_delta_pos[1];
	param->j1_jcb_lin_vel[1] = param->trans_mat_0_1[2][2] * param->j1_jcb_delta_pos[0] - param->trans_mat_0_1[0][2] * param->j1_jcb_delta_pos[2];
	param->j1_jcb_lin_vel[2] = param->trans_mat_0_1[0][2] * param->j1_jcb_delta_pos[1] - param->trans_mat_0_1[1][2] * param->j1_jcb_delta_pos[0];
	
	// Joint 2 Jacobian Linear Velocity
	param->j2_jcb_lin_vel[0] = param->trans_mat_0_2[1][2] * param->j2_jcb_delta_pos[2] - param->trans_mat_0_2[2][2] * param->j2_jcb_delta_pos[1];
	param->j2_jcb_lin_vel[1] = param->trans_mat_0_2[2][2] * param->j2_jcb_delta_pos[0] - param->trans_mat_0_2[0][2] * param->j2_jcb_delta_pos[2];
	param->j2_jcb_lin_vel[2] = param->trans_mat_0_2[0][2] * param->j2_jcb_delta_pos[1] - param->trans_mat_0_2[1][2] * param->j2_jcb_delta_pos[0];
	
	// Joint 3 Jacobian Linear Velocity
	param->j3_jcb_lin_vel[0] = param->trans_mat_0_3[1][2] * param->j3_jcb_delta_pos[2] - param->trans_mat_0_3[2][2] * param->j3_jcb_delta_pos[1];
	param->j3_jcb_lin_vel[1] = param->trans_mat_0_3[2][2] * param->j3_jcb_delta_pos[0] - param->trans_mat_0_3[0][2] * param->j3_jcb_delta_pos[2];
	param->j3_jcb_lin_vel[2] = param->trans_mat_0_3[0][2] * param->j3_jcb_delta_pos[1] - param->trans_mat_0_3[1][2] * param->j3_jcb_delta_pos[0];
	
	// Joint 4 Jacobian Linear Velocity
	param->j4_jcb_lin_vel[0] = param->trans_mat_0_4[1][2] * param->j4_jcb_delta_pos[2] - param->trans_mat_0_4[2][2] * param->j4_jcb_delta_pos[1];
	param->j4_jcb_lin_vel[1] = param->trans_mat_0_4[2][2] * param->j4_jcb_delta_pos[0] - param->trans_mat_0_4[0][2] * param->j4_jcb_delta_pos[2];
	param->j4_jcb_lin_vel[2] = param->trans_mat_0_4[0][2] * param->j4_jcb_delta_pos[1] - param->trans_mat_0_4[1][2] * param->j4_jcb_delta_pos[0];
	
	// Joint 5 Jacobian Linear Velocity
	param->j5_jcb_lin_vel[0] = param->trans_mat_0_5[1][2] * param->j5_jcb_delta_pos[2] - param->trans_mat_0_5[2][2] * param->j5_jcb_delta_pos[1];
	param->j5_jcb_lin_vel[1] = param->trans_mat_0_5[2][2] * param->j5_jcb_delta_pos[0] - param->trans_mat_0_5[0][2] * param->j5_jcb_delta_pos[2];
	param->j5_jcb_lin_vel[2] = param->trans_mat_0_5[0][2] * param->j5_jcb_delta_pos[1] - param->trans_mat_0_5[1][2] * param->j5_jcb_delta_pos[0];
	
	// Joint 6 Jacobian Linear Velocity
	param->j6_jcb_lin_vel[0] = param->trans_mat_0_6[1][2] * param->j6_jcb_delta_pos[2] - param->trans_mat_0_6[2][2] * param->j6_jcb_delta_pos[1];
	param->j6_jcb_lin_vel[1] = param->trans_mat_0_6[2][2] * param->j6_jcb_delta_pos[0] - param->trans_mat_0_6[0][2] * param->j6_jcb_delta_pos[2];
	param->j6_jcb_lin_vel[2] = param->trans_mat_0_6[0][2] * param->j6_jcb_delta_pos[1] - param->trans_mat_0_6[1][2] * param->j6_jcb_delta_pos[0];

	// Insert Jacobian Value To Jaobian Matrix
	for(int i=0; i<6; i++){
		if(i<3){
			param->jacobian_matrix[i][0] = param->j1_jcb_lin_vel[i];
			param->jacobian_matrix[i][1] = param->j2_jcb_lin_vel[i];
			param->jacobian_matrix[i][2] = param->j3_jcb_lin_vel[i];
			param->jacobian_matrix[i][3] = param->j4_jcb_lin_vel[i];
			param->jacobian_matrix[i][4] = param->j5_jcb_lin_vel[i];
			param->jacobian_matrix[i][5] = param->j6_jcb_lin_vel[i];
		}
		else{
			param->jacobian_matrix[i][0] = param->trans_mat_0_1[i-3][2];
			param->jacobian_matrix[i][1] = param->trans_mat_0_2[i-3][2];
			param->jacobian_matrix[i][2] = param->trans_mat_0_3[i-3][2];
			param->jacobian_matrix[i][3] = param->trans_mat_0_4[i-3][2];
			param->jacobian_matrix[i][4] = param->trans_mat_0_5[i-3][2];
			param->jacobian_matrix[i][5] = param->trans_mat_0_6[i-3][2];
		}
	}
}

// Check Sinuglarity
void check_singularity(Kinematics_t *param){
	param->jacobian_det = determinant_6x6(param->jacobian_matrix);
	
	if(param->jacobian_det == 0.0000000f){
		param->singularity = true;
	}
	else param->singularity =  false;
}

/* --------------------------------------------------------------------------------------------------------------------------------------------------*/


/* MINUS ZERO PREVENTION ----------------------------------------------------------------------------------------------------------------------------*/
float sanitize_zero(float value){
	return (value == 0.0f) ? 0.0f : value;
}
/* --------------------------------------------------------------------------------------------------------------------------------------------------*/
