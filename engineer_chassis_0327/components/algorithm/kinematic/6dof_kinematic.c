/**
  ****************************(C) COPYRIGHT 2023 TJU****************************
  * @file       6dof_kinematic.c/h
  * @brief      Kinematics solution of robotic arm,
  *             ��е���˶�ѧ����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-20-2023     Wang Yiwen       1. done
  *  V1.0.1     Dec-1-2023      Wang Yiwen       1. add quaterniont transform
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 TJU****************************
  */
#include "6dof_kinematic.h"

float cosf(float x)
{
    return arm_cos_f32(x);
}

float sinf(float x)
{
    return arm_sin_f32(x);
}

//��Ԫ���˷�
void QuaternionMultiply(const float* q1, const float* q2, float* result)
{
    // ��ȡ��Ԫ������
    float w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
    float w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
    
    // ����˷����
    result[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;  // ʵ��
    result[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;  // i����
    result[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;  // j����
    result[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;  // k����
}

//ŷ����->��ת����
static void EulerAngleToRotMat(const float* _eulerAngles, float* _rotationM)//Z-Y-X
{
    float ca, cb, cc, sa, sb, sc;

    cc = cosf(_eulerAngles[0]);
    cb = cosf(_eulerAngles[1]);
    ca = cosf(_eulerAngles[2]);
    sc = sinf(_eulerAngles[0]);
    sb = sinf(_eulerAngles[1]);
    sa = sinf(_eulerAngles[2]);

    _rotationM[0] = ca * cb;
    _rotationM[1] = ca * sb * sc - sa * cc;
    _rotationM[2] = ca * sb * cc + sa * sc;
    _rotationM[3] = sa * cb;
    _rotationM[4] = sa * sb * sc + ca * cc;
    _rotationM[5] = sa * sb * cc - ca * sc;
    _rotationM[6] = -sb;
    _rotationM[7] = cb * sc;
    _rotationM[8] = cb * cc;
}


//��ת����->ŷ����
static void RotMatToEulerAngle(const float* _rotationM, float* _eulerAngles)//Z-Y-X
{
    float A, B, C, cb;

    if (fabs(_rotationM[6]) >= 1.0 - 0.0001)
    {
        if (_rotationM[6] < 0)
        {
            A = 0.0f;
            B = (float) PI/2;
            C = atan2f(_rotationM[1], _rotationM[4]);
        } else
        {
            A = 0.0f;
            B = -(float) PI/2;
            C = -atan2f(_rotationM[1], _rotationM[4]);
        }
    } else
    {
        B = atan2f(-_rotationM[6], sqrtf(_rotationM[0] * _rotationM[0] + _rotationM[3] * _rotationM[3]));
        cb = cosf(B);
        A = atan2f(_rotationM[3] / cb, _rotationM[0] / cb);
        C = atan2f(_rotationM[7] / cb, _rotationM[8] / cb);
    }

    _eulerAngles[0] = C;
    _eulerAngles[1] = B;
    _eulerAngles[2] = A;
}


//��Ԫ��->��ת����
static void QuaToRotMat(const float *_Q, float* _R)
{
  _R[0] = 1 - 2 * (_Q[2] * _Q[2]) - 2 * (_Q[3] * _Q[3]);
  _R[1] = 2 * _Q[1] * _Q[2] - 2 * _Q[0] * _Q[3];
  _R[2] = 2 * _Q[1] * _Q[3] + 2 * _Q[0] * _Q[2];
  _R[3] = 2 * _Q[1] * _Q[2] + 2 * _Q[0] * _Q[3];
  _R[4] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[3] * _Q[3]);
  _R[5] = 2 * _Q[2] * _Q[3] - 2 * _Q[0] * _Q[1];
  _R[6] = 2 * _Q[1] * _Q[3] - 2 * _Q[0] * _Q[2];
  _R[7] = 2 * _Q[2] * _Q[3] + 2 * _Q[0] * _Q[1];
  _R[8] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[2] * _Q[2]);
}


//  _R[0] = (0,0);
//  _R[1] = (0,1);
//  _R[2] = (0,2);
//  _R[3] = (1,0);
//  _R[4] = (1,1);
//  _R[5] = (1,2);
//  _R[6] = (2,0);
//  _R[7] = (2,1);
//  _R[8] = (2,2);

//��ת����ת��Ԫ��
void RotMatToQua(const float* _R, float *_Q)
{
		float trace = _R[0] + _R[4] + _R[8];
	
    if (trace > 0.0f) 
    {
        float s = sqrt(trace + 1.0f);
        _Q[0] = (s * 0.5f);
        s = 0.5f / s;
        _Q[1] = ((_R[7] - _R[5]) * s);
        _Q[2] = ((_R[2] - _R[6]) * s);
        _Q[3] = ((_R[3] - _R[1]) * s);
    } 
    
    else 
    {
        int i = _R[0] < _R[4] ? (_R[4] < _R[8] ? 2 : 1) : (_R[0] < _R[8] ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        float s = sqrt(_R[i * 3 + i] - _R[j * 3 + j]  - _R[k * 3 + k] + 1.0f);
        _Q[i + 1] = s * 0.5f;
        s = 0.5f / s;

        _Q[0] 		= (_R[k * 3 + j] - _R[j * 3 + k]) * s;
        _Q[j + 1] = (_R[j * 3 + i] + _R[i * 3 + j]) * s;
        _Q[k + 1] = (_R[k * 3 + i] + _R[i * 3 + k]) * s;
    }
}

//��Ԫ��->Z-Y-Xŷ����
static void QuaToEulerAngle(const float *_Q, float* _eulerAngles)
{
	float row_R[9];
  row_R[0] = 1 - 2 * (_Q[2] * _Q[2]) - 2 * (_Q[3] * _Q[3]);
  row_R[1] = 2 * _Q[1] * _Q[2] - 2 * _Q[0] * _Q[3];
  row_R[2] = 2 * _Q[1] * _Q[3] + 2 * _Q[0] * _Q[2];
  row_R[3] = 2 * _Q[1] * _Q[2] + 2 * _Q[0] * _Q[3];
  row_R[4] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[3] * _Q[3]);
  row_R[5] = 2 * _Q[2] * _Q[3] - 2 * _Q[0] * _Q[1];
  row_R[6] = 2 * _Q[1] * _Q[3] - 2 * _Q[0] * _Q[2];
  row_R[7] = 2 * _Q[2] * _Q[3] + 2 * _Q[0] * _Q[1];
  row_R[8] = 1 - 2 * (_Q[1] * _Q[1]) - 2 * (_Q[2] * _Q[2]);
	
//	column_R[0] = row_R[0];
//	column_R[3] = row_R[1];
//	column_R[6] = row_R[2];
//	column_R[1] = row_R[3];
//	column_R[4] = row_R[4];
//	column_R[7] = row_R[5];
//	column_R[2] = row_R[6];
//	column_R[5] = row_R[7];
//	column_R[8] = row_R[8];
	
	float Y, P, R;
	Y = atan2f(row_R[3], row_R[0]);
	P = atan2f(-row_R[6], sqrtf(row_R[7] * row_R[7] + row_R[8] * row_R[8]));
	R = atan2f(row_R[7], row_R[8]);
	
	_eulerAngles[0] = Y;
	_eulerAngles[1] = P;
	_eulerAngles[2] = R;
}

//Z-Y-Xŷ����->��Ԫ��
static void EulerAngleToQua(const float* _eulerAngles, float *_Q)
{
	float cx, cy, cz, sx, sy, sz;
	float row_R[9];
	cz = cosf(_eulerAngles[0]);
	cy = cosf(_eulerAngles[1]);
	cx = cosf(_eulerAngles[2]);
	sz = sinf(_eulerAngles[0]);
	sy = sinf(_eulerAngles[1]);
	sx = sinf(_eulerAngles[2]);

	row_R[0] = cy * cz;
	row_R[1] = -cx * sz + sx * sy * cz;
	row_R[2] =  sx * sz + cx * sy * cz;
	row_R[3] = cy * sz;
	row_R[4] = sx * sy * sz + cx * cz;
	row_R[5] = cx * sy * sz - sx * cz;
	row_R[6] = -sy;
	row_R[7] = cy * sx;
	row_R[8] = cx * cy;

//	column_R[0] = row_R[0];
//	column_R[3] = row_R[1];
//	column_R[6] = row_R[2];
//	column_R[1] = row_R[3];
//	column_R[4] = row_R[4];
//	column_R[7] = row_R[5];
//	column_R[2] = row_R[6];
//	column_R[5] = row_R[7];
//	column_R[8] = row_R[8];
	
	float trace = row_R[0] + row_R[4] + row_R[8];
	if (trace > 0.0f) 
	{
			float s = sqrt(trace + 1.0f);
			_Q[0] = (s * 0.5f);
			s = 0.5f / s;
			_Q[1] = ((row_R[7] - row_R[5]) * s);
			_Q[2] = ((row_R[2] - row_R[6]) * s);
			_Q[3] = ((row_R[3] - row_R[1]) * s);
	} 
	
	else 
	{
			int i = row_R[0] < row_R[4] ? (row_R[4] < row_R[8] ? 2 : 1) : (row_R[0] < row_R[8] ? 2 : 0); 
			int j = (i + 1) % 3;  
			int k = (i + 2) % 3;

			float s = sqrt(row_R[i * 3 + i] - row_R[j * 3 + j] - row_R[k * 3 + k] + 1.0f);
			_Q[i + 1] = s * 0.5f;
			s = 0.5f / s;

			_Q[0] 		= (row_R[k * 3 + j] - row_R[j * 3 + k]) * s;
			_Q[j + 1] = (row_R[j * 3 + i] + row_R[i * 3 + j]) * s;
			_Q[k + 1] = (row_R[k * 3 + i] + row_R[i * 3 + k]) * s;
	}
}

//��е������ʼ��
void Joint6D_Init(Robotic_6DOF * Robotic_6D)
{
		//alpha,a,d,theta
    float DH_Matrix[6][4] = {
        {0.0f,          0.0f,      332.0f,       0.0f},
        {-PI/2,         0.0f,        0.0f,       0.0f},
        {0.0f,        300.0f,        0.0f,       0.0f},
        {PI/2,         	65.0f,     	430.0f,       0.0f},
        {PI/2,          0.0f,        0.0f,       0.0f},
        {-PI/2,         0.0f,     40.953f,       0.0f}};
		
		//����DH����->Robotic_6D�ṹ��
		for(int i = 0; i < 6; i++)
		{
				Robotic_6D->_alpha[i]=	DH_Matrix[i][0];
				Robotic_6D->_a[i]=	DH_Matrix[i][1];
				Robotic_6D->_d[i]=	DH_Matrix[i][2];
		}

    //��ʼ��λ�˱任����T
		for(int i = 0; i < 6; i++)
		{
		    Robotic_6D->_T_data[i] = (float *)user_malloc(sizeof(float) * 4 * 4);
				memset(Robotic_6D->_T_data[i], 0, sizeof(float) * 4 * 4);
			  Matrix_Init(&Robotic_6D->_T[i], 4, 4, (float *)Robotic_6D->_T_data[i]);
		}
		
		//��ʼ��������temp����
		for(int i = 0; i < 2; i++)
		{
				Robotic_6D->fk_temp_matrix_data[i] = (float *)user_malloc(sizeof(float) * 4 * 4);
				memset(Robotic_6D->fk_temp_matrix_data[i], 0, sizeof(float) * 4 * 4);
				Matrix_Init(&Robotic_6D->fk_temp_matrix[i], 4, 4, (float *)Robotic_6D->fk_temp_matrix_data[i]);		
		}
				
		//��ʼ��R02��ת����
		Robotic_6D->_R02_inv_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R02_inv_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->R02_inv, 3, 3, (float *)Robotic_6D->_R02_inv_data);

		//��ʼ��R23��ת����������
		Robotic_6D->_R23_inv_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R23_inv_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->R23_inv, 3, 3, (float *)Robotic_6D->_R23_inv_data);

		//��ʼ��R36��ת����
		Robotic_6D->_R36_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R36_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_R36, 3, 3, (float *)Robotic_6D->_R36_data);
		
		//��ʼ��R06��ת����
		Robotic_6D->_R06_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_R06_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_R06, 3, 3, (float *)Robotic_6D->_R06_data);
		
		//��ʼ�������temp����		
		Robotic_6D->_ik_temp_matrix_data = (float *)user_malloc(sizeof(float) * 3 * 3);
		memset(Robotic_6D->_ik_temp_matrix_data, 0, sizeof(float) * 3 * 3);
		Matrix_Init(&Robotic_6D->_ik_temp_matrix, 3, 3, (float *)Robotic_6D->_ik_temp_matrix_data);	

		//��ʼ��P56��temp����
		Robotic_6D->_P56_data = (float *)user_malloc(sizeof(float) * 3 * 1);
		memset(Robotic_6D->_P56_data, 0, sizeof(float) * 3 * 1);
		Matrix_Init(&Robotic_6D->_P56, 3, 1, (float *)Robotic_6D->_P56_data);	
		Robotic_6D->_P56_temp_data = (float *)user_malloc(sizeof(float) * 3 * 1);
		memset(Robotic_6D->_P56_temp_data, 0, sizeof(float) * 3 * 1);
		Matrix_Init(&Robotic_6D->_P56_temp, 3, 1, (float *)Robotic_6D->_P56_temp_data);		
}


//��е�������˶�ѧ���
void SolveFK(Robotic_6DOF * Robotic_6D, const Joint6D_t *_Joint6D, Pose6D_t *_Pose6D)
{
    float cosq, sinq;
    float cosa, sina;	
		float R06[9];
		float EulerAngles[3];
		float Quaterniont[4];
	
		//����ֵ
    for (int i = 0; i < 6; i++)
    {
        cosa = cosf(Robotic_6D->_alpha[i]);
        sina = sinf(Robotic_6D->_alpha[i]);
        cosq = cosf(_Joint6D->theta[i]);
        sinq = sinf(_Joint6D->theta[i]);
			
				Robotic_6D->_T_data[i][0] = cosq;
        Robotic_6D->_T_data[i][1] = -sinq;
        Robotic_6D->_T_data[i][2] = 0.0f;
        Robotic_6D->_T_data[i][3] = Robotic_6D->_a[i];
        Robotic_6D->_T_data[i][4] =  cosa * sinq;
        Robotic_6D->_T_data[i][5] =  cosa * cosq;
        Robotic_6D->_T_data[i][6] = -sina;
        Robotic_6D->_T_data[i][7] = -sina * Robotic_6D->_d[i];
        Robotic_6D->_T_data[i][8] =  sina * sinq;
			  Robotic_6D->_T_data[i][9] =  sina * cosq;
			  Robotic_6D->_T_data[i][10] = cosa;
			  Robotic_6D->_T_data[i][11] = cosa * Robotic_6D->_d[i];
			  Robotic_6D->_T_data[i][12] = 0.0f;
			  Robotic_6D->_T_data[i][13] = 0.0f;
			  Robotic_6D->_T_data[i][14] = 0.0f;
			  Robotic_6D->_T_data[i][15] = 1.0f;
    }	
			
		//����任�������
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->_T[0], &Robotic_6D->_T[1], &Robotic_6D->fk_temp_matrix[0]);		
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->fk_temp_matrix[0], &Robotic_6D->_T[2], &Robotic_6D->fk_temp_matrix[1]);
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->fk_temp_matrix[1], &Robotic_6D->_T[3], &Robotic_6D->fk_temp_matrix[0]);
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->fk_temp_matrix[0], &Robotic_6D->_T[4], &Robotic_6D->fk_temp_matrix[1]);
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->fk_temp_matrix[1], &Robotic_6D->_T[5], &Robotic_6D->fk_temp_matrix[0]);

		
		//ȡ����任�����е���ת����
		for(int i = 0; i < 3; i++)
		{
				for(int j = 0; j < 3; j++)
				{
						R06[i * 3 + j] = Robotic_6D->fk_temp_matrix_data[0][i * 4 + j];
				}
		}

		//��ת����->ĩ��ŷ����Z-Y-X
		RotMatToEulerAngle(R06, &(EulerAngles[0]));
		//��ת����->��Ԫ��
		RotMatToQua(R06, &(Quaterniont[0]));
		
		_Pose6D->X = Robotic_6D->fk_temp_matrix_data[0][3];
		_Pose6D->Y = Robotic_6D->fk_temp_matrix_data[0][7];
		_Pose6D->Z = Robotic_6D->fk_temp_matrix_data[0][11];
		_Pose6D->A = EulerAngles[0] * RAD_TO_DEG;//yaw
		_Pose6D->B = EulerAngles[1] * RAD_TO_DEG;//pitch
		_Pose6D->C = EulerAngles[2] * RAD_TO_DEG;//roll	
		_Pose6D->Q[0] = Quaterniont[0];
		_Pose6D->Q[1] = Quaterniont[1];
		_Pose6D->Q[2] = Quaterniont[2];
		_Pose6D->Q[3] = Quaterniont[3];
}


//��е�������˶�ѧ���
bool SolveIK(Robotic_6DOF * Robotic_6D, Pose6D_t *_inputPose6D, Solver6D_t *_Out_Solver6D, const Joint6D_t *_lastJoint6D, uint8_t _Quaterniont_mode)
{
    float q3[2];   //theta3
    float q12[2][2];//theta1,theta2
    float q456[2][3];//theta4,theta5,theta6
		float r,x,y,z;
		float a,b,c;
		float f1,f2,g1,k1,k2;
		float tmp;//�м����
		float R06[9];
		float Euler[3];
		float Quaterniont[4];
    int ind_theta3=2;//��������
		int ind_theta2=2;//��������
		int ind_theta5=2;//��������
		
		//����ĩ��λ�������ת����
		if(_Quaterniont_mode)//��Ԫ��ģʽ���Ӿ�ģʽ��
		{
				//��Ԫ��->��ת����
				QuaToRotMat(&(_inputPose6D->Q[0]), Robotic_6D->_R06_data);
				for(int i = 0; i < 3; i++)
				//ȡ��ת����
				{
						for(int j = 0; j < 3; j++)
						{
								R06[i * 3 + j] = Robotic_6D->_R06_data[i * 3 + j];
						}
				}
				//��ת����->ĩ��ŷ����Z-Y-X
				RotMatToEulerAngle(R06, &(Euler[0]));
				//Ŀ��ĩ����̬��ŷ���Ǹ���
				_inputPose6D->A = Euler[0] * RAD_TO_DEG;//yaw
				_inputPose6D->B = Euler[1] * RAD_TO_DEG;//pitch
				_inputPose6D->C = Euler[2] * RAD_TO_DEG;//roll					
		}
		else//ŷ����ģʽ
		{
				Euler[0]=_inputPose6D->A/RAD_TO_DEG;
				Euler[1]=_inputPose6D->B/RAD_TO_DEG;
				Euler[2]=_inputPose6D->C/RAD_TO_DEG;
				//ŷ����->��ת����
				EulerAngleToRotMat(&(Euler[0]), Robotic_6D->_R06_data);

				//ȡ��ת����
				for(int i = 0; i < 3; i++)
				{
						for(int j = 0; j < 3; j++)
						{
								R06[i * 3 + j] = Robotic_6D->_R06_data[i * 3 + j];
						}
				}
				//��ת����->��Ԫ��
				RotMatToQua(R06, &(Quaterniont[0]));
				//Ŀ��ĩ����̬����Ԫ������
				_inputPose6D->Q[0] = Quaterniont[0];
				_inputPose6D->Q[1] = Quaterniont[1];
				_inputPose6D->Q[2] = Quaterniont[2];
				_inputPose6D->Q[3] = Quaterniont[3];			
		}
	
		//�ϴν������֪�Ƕȸ�ֵ������ʵû���ã�
		for (int i = 0; i < 2; i++)
    {
        q3[i]      = _lastJoint6D->theta[2];
        q12[i][0]  = _lastJoint6D->theta[0];
        q12[i][1]  = _lastJoint6D->theta[1];
        q456[i][0] = _lastJoint6D->theta[3];
        q456[i][1] = _lastJoint6D->theta[4];
        q456[i][2] = _lastJoint6D->theta[5];
    }
		
		//����0�����ؽ�4��5��ԭ������
		Robotic_6D->_P56_data[0] = 0.0f;
		Robotic_6D->_P56_data[1] = 0.0f;		
		Robotic_6D->_P56_data[2] = 40.953f;
		Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->_R06, &Robotic_6D->_P56, &Robotic_6D->_P56_temp);
		
		//����1�����theta3
		//��ֵx,y,z,r
		x =	_inputPose6D->X - Robotic_6D->_P56_temp_data[0];
		y = _inputPose6D->Y - Robotic_6D->_P56_temp_data[1];
		z = _inputPose6D->Z - Robotic_6D->_P56_temp_data[2] - Robotic_6D->_d[0];
		r = x * x + y * y + z * z;
		//a=2*a2*a3
		a = 2 * Robotic_6D->_a[2] * Robotic_6D->_a[3];
		//b=2*a2*d4
		b = 2 * Robotic_6D->_a[2] * Robotic_6D->_d[3];
		//c=r-(a2^2+a3^2+d4^2+d3^2+a1^2+d4^2+2d2d3
		c = r - (Robotic_6D->_a[1] * Robotic_6D->_a[1] + Robotic_6D->_a[2] * Robotic_6D->_a[2] + Robotic_6D->_a[3] * Robotic_6D->_a[3]
					  + Robotic_6D->_d[1] * Robotic_6D->_d[1] + Robotic_6D->_d[2] * Robotic_6D->_d[2] + Robotic_6D->_d[3] * Robotic_6D->_d[3]
				    + 2 * Robotic_6D->_d[1] * Robotic_6D->_d[2]);
		
		if(a + c == 0)
		{
				if(b == 0)
				{		
						q3[0] = -PI;
						q3[1] =  PI;
						//˫��
						for (int i = 0; i < 4; i++)
						{
								Robotic_6D->_IK_Flags[0 + i][0] = 2;
								Robotic_6D->_IK_Flags[4 + i][0] = 2;
						}
						return false;
				}
				else
				{
						//����
					  q3[0] = 2*atan2f(c - a, 2 * b); 
						q3[1] = q3[0];
						for (int i = 0; i < 4; i++)
						{
								Robotic_6D->_IK_Flags[0 + i][0] = 1;
								Robotic_6D->_IK_Flags[4 + i][0] = 1;
						}			
				}
		}
		else
		{
				if(b * b + a * a - c * c > 0)
				{
						//˫��
						arm_sqrt_f32(a * a + b * b - c * c, &tmp);
						q3[0] = 2*atan2f(b + tmp, a + c);
						q3[1] = 2*atan2f(b - tmp, a + c);
						for (int i = 0; i < 4; i++)
						{
								Robotic_6D->_IK_Flags[0 + i][0] = 2;
								Robotic_6D->_IK_Flags[4 + i][0] = 2;
						}
				}
				else if(b * b + a * a - c * c == 0)
				{
						//����
						q3[0] = 2*atan2f(b, a + c);
						q3[1] = q3[0];
						for (int i = 0; i < 4; i++)
						{
								Robotic_6D->_IK_Flags[0 + i][0] = 1;
								Robotic_6D->_IK_Flags[4 + i][0] = 1;
						}			
				}
				else
				{
						//�޽�
						for (int i = 0; i < 4; i++)
						{
								Robotic_6D->_IK_Flags[0 + i][0] = 0;
								Robotic_6D->_IK_Flags[4 + i][0] = 0;
						}			
					  return false;
				}
		}


		for(int i=0;i<ind_theta3;i++)
		{		
				//����2�����theta2
				float cosq3 = cosf(q3[i]);
				float sinq3 = sinf(q3[i]);
			  //���R23�������				
				Robotic_6D->_R23_inv_data[0] = cosq3;
				Robotic_6D->_R23_inv_data[1] = sinq3;
				Robotic_6D->_R23_inv_data[2] = 0;
				Robotic_6D->_R23_inv_data[3] = -sinq3;
				Robotic_6D->_R23_inv_data[4] = cosq3;
				Robotic_6D->_R23_inv_data[5] = 0;
				Robotic_6D->_R23_inv_data[6] = 0;
				Robotic_6D->_R23_inv_data[7] = 0;
				Robotic_6D->_R23_inv_data[8] = 1;
				
				//k1=a3*c3+d4*s3 a2=f1
				k1 = Robotic_6D->_a[3] * cosq3 + Robotic_6D->_d[3] *sinq3 + Robotic_6D->_a[2];
				//k2=d4*c2-a3*s3=-f2
				k2 = Robotic_6D->_d[3] * cosq3 - Robotic_6D->_a[3] * sinq3;
				//a=k2,b=-k1,c=z
				a = k2; b = -k1; c = z;
			
				if(a + c == 0)
				{
						if(b == 0)
						{
								//˫��
								q12[0][1] = -PI; 
								q12[1][1] =  PI;
                for (int j = 0; j < 2; j++)
                {
                    Robotic_6D->_IK_Flags[4 * i + 0 + j][1] = 2;
                    Robotic_6D->_IK_Flags[4 * i + 2 + j][1] = 2;
                }
								return false;
						}
						else
						{
								//����
								q12[0][1] = 2*atan2f(c-a,2*b); 
								q12[1][1] = q12[0][1];
							
                for (int j = 0; j < 2; j++)
                {
                    Robotic_6D->_IK_Flags[4 * i + 0 + j][1] = 1;
                    Robotic_6D->_IK_Flags[4 * i + 2 + j][1] = 1;
                }
						}
				}
				else
				{
						if(b * b + a * a - c * c > 0)
						{
								//˫��
								arm_sqrt_f32( a * a + b * b - c * c,&tmp);
								q12[0][1] = 2*atan2f(b + tmp, a + c);
								q12[1][1] = 2*atan2f(b - tmp, a + c);
							
                for (int j = 0; j < 2; j++)
                {
                    Robotic_6D->_IK_Flags[4 * i + 0 + j][1] = 2;
                    Robotic_6D->_IK_Flags[4 * i + 2 + j][1] = 2;
                }
						}
						else if(b * b + a * a - c * c == 0)
						{
								//����
								q12[0][1] = 2*atan2f(b, a + c);
								q12[1][1] = q12[0][1];
							
                for (int j = 0; j < 2; j++)
                {
                    Robotic_6D->_IK_Flags[4 * i + 0 + j][1] = 1;
                    Robotic_6D->_IK_Flags[4 * i + 2 + j][1] = 1;
                }
						}
						else
						{
								//�޽�
                for (int j = 0; j < 2; j++)
                {
                    Robotic_6D->_IK_Flags[4 * i + 0 + j][1] = 0;
                    Robotic_6D->_IK_Flags[4 * i + 2 + j][1] = 0;
                }
								return false;
						}
				}

				//����3�����theta1 
				for(int j = 0; j < ind_theta2; j++)
				{
						//k1=a3*c3+d4*s3 +a2=f1
						f1 = k1;
						//k2=d4*c2-a3*s3=-f2
						f2 = -k2;
						//g1=c2*f1-s2*f2+a1
					  g1 = cosf(q12[j][1]) * f1 - sinf(q12[j][1]) * f2 + Robotic_6D->_a[1];
						
						if(g1 == 0)
						{		
								//�޽�
								Robotic_6D->_IK_Flags[4 * i + 0 + j][2] = 0;
								Robotic_6D->_IK_Flags[4 * i + 2 + j][2] = 0;
								return false;
						}
						else
						{
								//����
								q12[j][0] = atan2f(y / g1, x / g1);
								Robotic_6D->_IK_Flags[4 * i + 0 + j][2] = 1;
								Robotic_6D->_IK_Flags[4 * i + 2 + j][2] = 1;
						}
				}
				
				//����4�����theta4,theta5,theta6 
				for(int j = 0; j < ind_theta2; j++)
				{
						//����R12_inv*R01_inv����R02_inv
						float cos1 = cosf(q12[j][0]);//cos(theta1)
						float sin1 = sinf(q12[j][0]);//sin(theta1)
						float cos2 = cosf(q12[j][1]);//cos(theta2)
						float sin2 = sinf(q12[j][1]);//sin(theta2)			
						//R02�������ֵ
						Robotic_6D->_R02_inv_data[0] =  cos1 * cos2;
						Robotic_6D->_R02_inv_data[1] =  cos2 * sin1;
						Robotic_6D->_R02_inv_data[2] =  -sin2;
						Robotic_6D->_R02_inv_data[3] =  -cos1 * sin2;
						Robotic_6D->_R02_inv_data[4] =  -sin1 * sin2;
						Robotic_6D->_R02_inv_data[5] =  -cos2;
						Robotic_6D->_R02_inv_data[6] =  -sin1;
						Robotic_6D->_R02_inv_data[7] =   cos1;
						Robotic_6D->_R02_inv_data[8] =  0;

						//������ת����R36
						Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->R02_inv, &Robotic_6D->_R06, &Robotic_6D->_ik_temp_matrix);
						Robotic_6D->MatStatus = Matrix_Multiply(&Robotic_6D->R23_inv, &Robotic_6D->_ik_temp_matrix, &Robotic_6D->_R36);
						
						//���theta5��theta4,theta6�����������
						arm_sqrt_f32(Robotic_6D->_R36_data[2] * Robotic_6D->_R36_data[2] + Robotic_6D->_R36_data[8] * Robotic_6D->_R36_data[8], &tmp);											
						
						//theta5=0
						if(tmp < 0.00001f)
						{
								//����⣺theta5=0/theta5=180������һ�鲻����Լ����Χ�����ڱ���������Ϊ�ǵ���
								for(int k = 0; k < ind_theta5; k++)
								{
										float s4,s6;
									  float c4,c6;									
										//theta5
										q456[k][1] = k * PI;									
										//theta4,��theta5�ӽ�0��ʱֻ�����theta��theta6�ĺͻ�����theta4����һ�εĽǶȣ�������0��
										q456[k][0] = _lastJoint6D->theta[3];
										//theta6
										c4 = cosf(_lastJoint6D->theta[3]);
										s4 = sinf(_lastJoint6D->theta[3]);												
										c6 = -(2 * k - 1) * c4 * Robotic_6D->_R36_data[0]  - s4 * Robotic_6D->_R36_data[1];
										s6 = - s4 * Robotic_6D->_R36_data[0] + (2 * k - 1) * c4 * Robotic_6D->_R36_data[1];										
										q456[k][2] = atan2f(s6,c6);
										//����
										Robotic_6D->_IK_Flags[4 * i + 2 * j + k][3] = 1;
								}
						}
						else
						{		
								//˫��			
								for(int k = 0; k < ind_theta5; k++)
								{
										//theta5
										q456[k][1] = atan2f((2 * k - 1) * tmp, -Robotic_6D->_R36_data[5]);
										//theta4,theta6
										float sinq5 = sinf(q456[k][1]);
										q456[k][0] = atan2f(-Robotic_6D->_R36_data[8] / sinq5, -Robotic_6D->_R36_data[2] / sinq5);
										q456[k][2] = atan2f( Robotic_6D->_R36_data[4] / sinq5, -Robotic_6D->_R36_data[3] / sinq5);
										//˫��
										Robotic_6D->_IK_Flags[4 * i + 2 * j + k][3] = 2;
								}
						}								
						
						
						//��ֵ
						for(int k = 0; k < ind_theta5; k++)
						{
								//theta3��ֵ
								if(q3[i] > PI)
								{
										_Out_Solver6D->theta[4 * i + 2 * j + k][2] = q3[i] - 2 * PI;
								}	
								else if(q3[i] < -PI)
								{
										_Out_Solver6D->theta[4 * i + 2 * j + k][2] = q3[i] + 2 * PI;
								}
								else
								{
										_Out_Solver6D->theta[4 * i + 2 * j + k][2] = q3[i];
								}
								
								//theta1,theta2��ֵ
								for(int m = 0; m < 2; m++)
								{										
										if(q12[j][m] > PI)
										{
												_Out_Solver6D->theta[4 * i + 2 * j + k][m] = q12[j][m] - 2 * PI;
										}	
										else if(q12[j][m] < -PI)
										{
												_Out_Solver6D->theta[4 * i + 2 * j + k][m] = q12[j][m] + 2 * PI;
										}
										else
										{
												_Out_Solver6D->theta[4 * i + 2 * j + k][m] = q12[j][m];
										}	
								}
								//theta4,theta5,theta6��ֵ
								for(int n = 0; n < 3; n++)
								{
										if(q456[k][n] > PI)
										{
												_Out_Solver6D->theta[4 * i + 2 * j + k][3 + n] = q456[k][n] - 2 * PI;
										}	
										else if(q456[k][n] < -PI)
										{
												_Out_Solver6D->theta[4 * i + 2 * j + k][3 + n] = q456[k][n] + 2 * PI;
										}
										else
										{
												_Out_Solver6D->theta[4 * i + 2 * j + k][3 + n] = q456[k][n];
										}		
								}				
						}
				}
		}
		
		return true;
}

