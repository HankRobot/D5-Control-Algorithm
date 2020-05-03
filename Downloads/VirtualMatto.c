#include "VirtualMatto.h"

int triggerFlag = 0;


//*************************************************************
// STUDENT VARIABLES GO HERE
//*************************************************************
//Variables for storing sensor values
double busbarv = 0;
double busbari = 0;
double mainsp = 3;
double windp = 0;
double solarp = 0;
double supply = 0;
double batteryp = 0;

double controlbattery = 0;

double on1 = 1;
double on2 = 1;
double on3 = 1;
//Variables for storing load power (100kw unit)
double load1p = 0.8;
double load2p = 1.8;
double load3p = 1.4;

//pid control variables
double set_target = 230;
double integral = 0;
double last_error = 0;
double kp = 1.5;  //2 0.1 0.05 //2 0.01 0.05 //2 0 0.05
double ki = 0;
double kd = 0.1;  

void pid_control(double kp, double ki, double kd){
    //calculate the current error
    double error = set_target - busbarv;

    //calculate the integral
	integral += error;
	
    //calculate the derivative
    double derivative = error - last_error;

    //calculate the control variable
    mainsp = (kp*error) + (ki*integral) + (kd*derivative);

    //tune the mainsp
    if(mainsp>3) mainsp = 3;
    else if(mainsp < 0) mainsp = 0;
    last_error = error;
}

//*************************************************************
/*
Input Signals
	Ai1 busbar input voltage
	Ai2 busbar input current
	Ai3 wind input voltage
	Ai4 solar input voltage

	Di1 call for 1st load
	Di2 call for 2nd load
	Di3 call for 3rd load

Output Signals
	Do1 turn on 1st load
	Do2 turn on 2nd load
	Do3 turn on 3rd load
	Ao1 busbar power, 0-3, a multiply of 100kW
	Ao2 battery power, -1 to 1, multiply of 100kW, positive means supply to grid
*/
void getsensorvalues(double voltage, double current, double wind, double solar, double load1, double load2, double load3){
	busbarv = voltage;
	busbari = current;
	windp = wind;
	solarp = solar;
	pid_control(kp,ki,kd);
	supply = (busbarv * busbari)/100000 + windp + solarp + mainsp - (load1*load1p + load2*load2p + load3*load3p) + controlbattery;
	
	if (supply>0) //over supply
	{
		if (supply >= 1)
		{
			controlbattery = -1; //begin charging at max capacity
			batteryp += 1; //batteryp increases per oversupply, which is 100kw

			/*
			if (mainsp - (supply-1) > 0 ) //considering the battery as a load, if its still excess
			{
				mainsp -= supply - 1; //lower mains supply
			}
			else
			{
				mainsp = 0; //too much powaaaa lets just let the wind and solar do everything
			}
			*/
		}
		else
		{
			controlbattery = -1* supply; //begin charging based on how many excess power is, negative value means charging
			batteryp += supply;
		}
	}
	else //under supply
	{

		if (batteryp >= 0) //begin considering battery options
		{
			if (supply <= -1 && batteryp >= 1) //load exceeds what battery can supply
			{
				controlbattery = 1; //begin discharging at max capacity
				batteryp -= 1; //batteryp decreases per undersupply, which is -100kw
			}
			else if(supply <= 0 && batteryp >= -1*(supply))
			{
				controlbattery = -1*(supply); //begin discharging based on how many power required by supply
				batteryp += supply; //supply here is negative
			}
			else
			{
				controlbattery = -batteryp; //use up all the battery power left
				batteryp = 0; 
			}
		}	
		else
		{
			controlbattery = 0;
		}

		/*
		if (mainsp + -1*(supply) <= 3) //if main supply can still provide more power
		{
			mainsp += -1*supply; //increase mains call			
		}
		else  //if main supply cant provide anymore power
		{
			mainsp = 3; //turn on maximum power
		}
		*/
	}
	
}

void update(inputVector* input, double clk, outputVector* output)
{

	if (clk == 1 && triggerFlag == 0) //Only trigger at edge
	{
		triggerFlag = 1;
		//*************************************************************
		// STUDENT CODES GO HERE
		getsensorvalues(input->Ai1,input->Ai2,input->Ai3,input->Ai4,input->Di1,input->Di2,input->Di3);

		//----------------------
		// example
        temp = 0;

		//Setting Output
        output->Do1 = input->Di1;
		output->Do2 = input->Di2;
		output->Do3 = input->Di3;
		
        output->Ao1 = mainsp;
        output->Ao2 = controlbattery;

		// end example
		// ---------------------- 

		// STUDENT CODE END
		//*************************************************************
	}
	else if (clk == 0)
	{
		triggerFlag = 0;
	}
	return output;

}