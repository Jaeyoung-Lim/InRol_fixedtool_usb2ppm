////////////////////////////////////////////////////////////
/////Tool dynamics controller implementation
// Controller input to the quadrotor is angle input
//by Jaeyoung Lim                     //
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
#include <math.h>

#include "Client.h"
#include "Tracker.h"
#include "DataStructure.h"
#include "Vector3.h"
#include "Matrix33.h"
#include "Desired.h"

#include "usb2PPM.h"
#include "Constants.h";

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
//double eta = 3.0+2.0+2.0-1.0; //v = haptic_x*eta
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
int loop_count;

double PI = 3.1415926535897932384626433832795028841971;
double m = 1.0;//in kg, measured by balance 19oz*28.35
double g = 9.80665; // 1350 is the thrust that the flyer take off;
static double lambda = m*g;
static double w1=0.0, w2=0.0, w21=0.0, w22=0.0;
double dt = 0.17; //0.017 seconds
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
	//Vector3 ddx;

	Matrix33 R(cur_TState.RotationMatrix);// R is from Body frame to Global Frame
	//Calculate Euler Angles (What rotation)
	//Vector3 Euler(cur_TState.RotationEuler[0], cur_TState.RotationEuler[1], cur_TState.RotationEuler[2]); //Current Euler Angles
	double phi, theta, psi;
	theta = asin(-R.r3.x);
	phi = asin(R.r3.y/cos(theta));
	psi = asin(R.r2.x/cos(theta));
	Vector3 Euler(phi, theta, psi);

	loop_count++;
	if(loop_count>100){
		
		printf("Roll :%f\tPitch :%f\tYaw :%f\n", Euler.x*180/3.14, Euler.y*180/3.14, Euler.z*180/3.14);
		loop_count=0;
	}
	Vector3 w(cur_TState.AngularVelocity[0], cur_TState.AngularVelocity[1], cur_TState.AngularVelocity[2]);
	double Sw_array[9]={0, -w.z, w.y,
						w.z, 0 -w.x,
						-w.y, w.x, 0};
	Matrix33 Sw(Sw_array);
	Vector3 wd;
	//Vector3 dw; // angular acceleration
	// Calculate the postion of tool
	Vector3 y = x+R*d;//Position of tool
	Vector3 dy = dx+R*Sw*d;//Velocity of tool
	Vector3 ddy;

	//Desired position
	Vector3 yd;//Desired tool postion
	Vector3 dyd;
	Vector3 ddyd;
	Vector3 dddyd;		
	//Desired trajectories are in Desired.cpp
	Desired traj(mode, index, time, time0);
	yd = traj.x;	dyd = traj.v;	ddyd = traj.a;	dddyd = traj.da;

	if(Ctrl_Mode == Tool_Ctrl){//Tool Position Control
		Vector3 e=y-yd; //Tool tip position error [m]
		Vector3 de=dy-dyd; // Tool tip position velocity error [m/s]
		Vector3 g_hat = R.Trans()*e_3*g;
		double sigma_array[9] = {-d.z/alpha, 0, d.x/alpha,
								0, 1, 0,
								d.x/alpha, 0, d.z/alpha};
		Matrix33 sigma(sigma_array);
		//Calculate desired Input
		Vector3 ud = ddyd*(-m)+de*(-1)*b-e*k;//Postion control input
		Vector3 ud_hat = R.Trans()*(de*b*(-1)-e*k)*(1/m_hat);//Postion control input

		//nu dot
		Vector3 nu=sigma.Trans()*w;
		Vector3 dnu;
		dnu.y = (1/d.z)*(ud_hat.x+g_hat.x+(d.x*nu.x*nu.x+d.x*nu.y*nu.y+d.z*nu.x*nu.z));
		dnu.x = (1/alpha)*(ud_hat.y-g_hat.y-alpha*nu.y*nu.z);
		lambda = m*(-ud_hat.z+g_hat.z-d.x*dnu.y-(d.z*nu.x*nu.x+d.z*nu.y*nu.y-d.x*nu.x*nu.z));
	
		Vector3 nud = nu+dnu*dt;//nu desired
	
		//Calculate command euler angle
		double gamma_array[9]={1, sin(phi)*sin(theta)/cos(theta), cos(phi)*sin(theta)/cos(theta),
						0, cos(phi), -sin(phi),
						0, sin(phi)/cos(theta), cos(phi)/cos(theta)};
		Matrix33 gamma(gamma_array);
	
		Vector3 dEulerd=gamma*sigma*nud;//dEuler desired
		Vector3 Eulerd=Euler+dEulerd*dt; //Desired Eueler angles on next step

		 
		phid= Eulerd.x; //rad
		thetad = Eulerd.y;
		psid= Eulerd.z;
	}
	if(Ctrl_Mode == Cg_Ctrl){//Center of Mass Control
		Vector3 e=y-yd; //Tool tip position error [m]
		Vector3 de=dy-dyd; // Tool tip position velocity error [m/s]
		//Calculate desired Input
		Vector3 ud = ddyd*(-m)+de*(-1)*b-e*k;//Postion control input
		Vector3 ud_hat = R.Trans()*(de*b*(-1)-e*k)*(1/m_hat);//Postion control input
	}
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
	
//*************************************************Translate all to USB2PPM Command*********************************************************
		
		cur_XPCommand.thrust = max(min((lambda+200),1023),0);
		cur_XPCommand.roll = min(max(phid*652.23+512,0),1023);// thetad 0rad=512pwm 0.785rad=1023pwm   
		cur_XPCommand.pitch = min(max(thetad*652.23+512,0),1023);
		cur_XPCommand.yaw= min((psid*652.23+512), 1023);
		
//*************************************** Here POST the command to haptic device and flyerzzz
	
	WaitForSingleObject(g_XCommandMutex,INFINITE);
	memcpy(&g_XPCommand, &cur_XPCommand, sizeof(CTRL_Input));
	ReleaseMutex(g_XCommandMutex);
	
	//**********************************************CONFIRM ZONE**********************************************************//
	//logging

	return iResult;
}