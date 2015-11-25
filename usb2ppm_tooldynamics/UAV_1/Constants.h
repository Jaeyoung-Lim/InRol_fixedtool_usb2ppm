#define Tool_Ctrl 1 //Tool Tip Control Mode
#define Cg_Ctrl 0 //CG Control Mode

//Quadrotor parameters
static double m_hat = 1.07; // initial estimated mass value 1069g
Vector3 d(0.2, 0.0, -0.2); //Tool form
static double alpha = sqrt(d.x*d.x + d.z*d.z);
	

// Controller Parameters
static double k=5.0; //Position controller constants
static double b=1.4;

double thetad;
double phid;
double dpsid;


int Ctrl_Mode=1;