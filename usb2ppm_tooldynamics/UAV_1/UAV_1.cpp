////////////////////////////////////////////////////////////
// Extract trajectory works                             //
// BLUE                                                 //
////////////////////////////////////////////////////////////

#include "stdafx.h"
#include <iostream>
#include <stdio.h>
#include <Windows.h>
#include <string>
#include <fstream>
#include <vector>
#include <conio.h>

#include "Client.h"
#include "Tracker.h"
#include "DataStructure.h"
#include "Vector3.h"
#include "Desired.h"
#include "Matrix33.h"
#include "usb2PPM.h"

using namespace std;


/////////Global DATA////////////

CTRL_Input g_XPCommand;


XConfig g_XConfig;
XStatus g_XStatus;

TState g_TState;

//All in Client.h
//Mutex...
HANDLE g_XCommandMutex = NULL;
HANDLE g_XStateMutex = NULL;

HANDLE g_TStateMutex;

//motion scale
long long initialTick;
double eta = 3.0+2.0+2.0-1.0; //v = haptic_x*eta
Vector3 e_3(0.0, 0.0, 1.0);
int index = 1;// hovering
int mode = 2;// 1: teleoperation, 2: trajectory tracking, 3: velocity field control
bool PTAM_FLY = false;
double time0=0.0;
Vector3 Force;
Vector3 Force_env;
Vector3 acc_l;
double TDelay=0.0;
unsigned long long C_loss=0;
unsigned long long C_data=0;

double PI = 3.1415926535897932384626433832795028841971;
double m = 0.8+0.1, m2 = 0.8+0.1;//in kg, measured by balance 19oz*28.35
double g = 9.80665; // 1350 is the thrust that the flyer take off;
static double lambda = m*g, lambda2 = m2*g;
static double w1=0.0, w2=0.0, w21=0.0, w22=0.0;

ofstream outFile1("E:\\DHLee/data/STATE.txt");//path should be modified
ofstream outFile2("E:\\DHLee/data/ERROR.txt");//path should be modified
ofstream outFile3("E:\\DHLee/data/FORCE.txt");//path should be modified
ofstream outFile4("E:\\DHLee/data/SPEED.txt");//path should be modified
ofstream outFile5("E:\\DHLee/data/RAW.txt");//path should be modified

SOCKET   ServerSocket;
SOCKADDR_IN  server_addr;
SOCKADDR_IN  client_addr;


////////MainThreadDeclear////
int HXControl(void);
int HXTest(void);

int _tmain(int argc, _TCHAR* argv[]) //This Control the All the states' Communication
{
	int ret = 0;
	// About the global Datas
	SECURITY_ATTRIBUTES    sec_attr;
	
	sec_attr.nLength              = sizeof( SECURITY_ATTRIBUTES );
	sec_attr.lpSecurityDescriptor = NULL;
	sec_attr.bInheritHandle       = TRUE;
	// MutexCreated
	g_XCommandMutex = CreateMutex(&sec_attr, NULL, _T("mutex for Xcommand"));
	g_XStateMutex = CreateMutex(&sec_attr, NULL, _T("mutex for XState"));
	g_TStateMutex = CreateMutex(&sec_attr, NULL, _T("mutex for TState"));
	
	// Serial Port Side Established
	Serial ppmserial;
	ret += ppmserial.Ini("\\\\.\\COM24");
	
	// Tracker Polling Established
	Tracker vicon; //updateRate 100
	ret += vicon.Ini();


	// Ini the rotation matrix
	g_TState.RotationMatrix[0] =1.0;
	g_TState.RotationMatrix[4] =1.0; 
	g_TState.RotationMatrix[8] =1.0; 

	Sleep(1000);
///////////////////////////////////////////////Here is CONTROLLER////////////////////////////////////////////////////
	
	while(true)
	{
		HXControl();
		Sleep(1); // Synchronizing : '1' for haptic, '10' for vicon
	}
	
	vicon.Close();
	//usbSerial.Close();
	getchar();
	outFile1.close();
	outFile2.close();
	outFile3.close();
	outFile4.close();
	outFile5.close();
	
	return ret;
}

int HXControl(void)
{
	int iResult = 0;
	
	
	TState cur_TState;
	CTRL_Input cur_XPCommand;
	

////////////////////////////////Main Loop T/////////////////////////// //////
	static LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	long long cpuFreq = freq.QuadPart;

	static LARGE_INTEGER l;
	static LARGE_INTEGER llast = {0};
	static bool firstRun = true;
	QueryPerformanceCounter(&l);
	if(firstRun == true) // Ini Last time
	{
		initialTick = l.QuadPart;
		llast.QuadPart = l.QuadPart;
		firstRun = false;
		return iResult;
	}
	double T = (double)((l.QuadPart)-(llast.QuadPart))/cpuFreq;
	llast.QuadPart = l.QuadPart;
	double time = (double)(l.QuadPart - initialTick)/cpuFreq;
	


////////////////////////////////////////Here begin the state polling from Haptic(virtual mass and object), Flyer, Tracker
	//Get all the States

	// get parameters from Vicon
	WaitForSingleObject(g_TStateMutex,INFINITE);
	memcpy(&cur_TState, &g_TState, sizeof(TState));
	ReleaseMutex(g_TStateMutex);

	// Get position, velocity,Rotation matrix from Tracker
	//printf("%d\n", cur_TState.Translation[0]);
	Vector3 x(cur_TState.Translation[0]/1000.0, cur_TState.Translation[1]/1000.0, cur_TState.Translation[2]/1000.0); //in m
	Vector3 dx(cur_TState.Velocity[0]/1000.0, cur_TState.Velocity[1]/1000.0, cur_TState.Velocity[2]/1000.0); //in m/s
	Vector3 ddx;

	Matrix33 R(cur_TState.RotationMatrix);// R is from Body frame to Global Frame
	Matrix33 dR= (R-R_last)/dt;
	Matrix33 w_cof= R.Trans()*dR;
	Vector3 w(w_cof.a11, );
	memcpy(R_last, R);
	//Calculate omega
	w=R;//Calculate body rotational velocity omega
	//Low pass filter?

	/////Tool dynamics controller implementation
	// Controller input to the quadrotor is angle input
	//by Jaeyoung Lim
	//Parameters
	Vector3 d(0.2, 0.0, -0.2); //Tool form
	static double alpha = sqrt(d.x*d.x + d.z*d.z);
	Matrix33 sigma(d.z*-1, 0, d.x, 0, alpha, 0, d.x, 0, d.z);
	static double m_hat = 0.55+0.25; // initial estimated mass value

	// Calculate the postion of tool
	Vector3 y = x+R*d;//Position of tool
	Vector3 dy = dx+R*d;//Velocity of tool
	Vector3 ddy;
	
	//Get Acceleration from Dynamics
	//Vector3 Dyn=R*e_3*lambda*(-1.0)/m_hat+e_3*g; // dynamics\

	//Desired position
	Vector3 yd;//Desired tool postion
	Vector3 dyd;
	Vector3 ddyd;
	Vector3 dddyd;	
	
	//Desired trajectories are in Desired.cpp
	Desired traj(mode, index, time, time0);
	yd = traj.x;	dyd = traj.v;	ddyd = traj.a;	dddyd = traj.da;

	// Controller
	static double k=1.0; //Position controller constants
	static double b=0.4;

	Vector3 e=y-yd;
	Vector3 de=dy-dyd;
	Vector3 ud = de*-1*b-e*k;//Postion control input
	Vector3 ud_hat = R.Trans()*(de*-1*b-e*k)*(1/m_hat);//Postion control input
	
	//nu dot = 
	Vector3 nu=sigma.Trans()*w;
	Vector3 dnu;
	dnu.x = d1*;
	dnu.y = ;
	dnu.z = ;

	//w dot =
	Vector3 dw = sigma*nu;
	Vector3 w = sigma*nu;
	
	Cur_Tstate.Euler


	//Calculate command euler angle
	//w1 = -Rtv.y/lambda;
	//w2 = Rtv.x/lambda;

	//keyboard setting!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	if( _kbhit() )
		{
			char input = _getch();
			if (input == 'q')
			{//Exit all modes
				cout << "\nExit";
				Sleep(50);
				exit(9);
			}

			else if( input == 'a')
			{//Hover on a fixed point
				mode=2;	index=1;
			}
			else if( input == 'b')
			{//Move in a desired trajectory
				mode=2;	index=2;
				time0 = time;
			}
			else if( input == 'c')
			{//Land mode
				mode=2;	index=3;
			}
			else if( input == 'd')
			{
				mode=2;	index=4;
				time0=time;
			}	
	
	}
	
	double yawspeed = 1023; //yawspeed = 2048 + compen_w3;// + 3*yawangle;
	
//*************************************************Translate all to USB2PPM Command*********************************************************

		cur_XPCommand.thrust = max(min(thrustValue,1023),0);
		cur_XPCommand.pitch = min(max(2*w2+512,0),1023); // minus is positive direction   
		cur_XPCommand.roll = min(max(2*w1+512,0),1023); // minus is positive direction   
		cur_XPCommand.yaw= (yawspeed-1023);//min(max(2048 + compen_w3 -11.4*compen_w3,0),4096);//yawspeed; //  // positve
		
//*************************************** Here POST the command to haptic device and flyerzzz
	
	WaitForSingleObject(g_XCommandMutex,INFINITE);
	memcpy(&g_XPCommand, &cur_XPCommand, sizeof(CTRL_Input));
	ReleaseMutex(g_XCommandMutex);
	
	//**********************************************CONFIRM ZONE**********************************************************//
	//logging

	return iResult;
}