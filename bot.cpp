#include <iostream>
#include <string>
using namespace std;

const int8_t fin_height(100); //desired height
const int8_t fin_roll(0); // desired roll
const int8_t fin_pitch(0); // desired pitch

int8_t tot_intp = 0; //integration sum for pitch
int8_t tot_intr = 0; //integration sum for roll
int8_t prev_error_p = 0; // previous error for pitch
int8_t prev_error_r = 0; // previous error for roll

const int dt(1); // timestep
const int kp(1.5); //setting up variable for testing 
const int ki(.05); //setting up variable for testing 
const int kd(.7); //setting up variable for testing 
const int8_t constant = 1; //application of rotors to pitch and roll

static int8_t fr = 0; //strenth of fr rotor
static int8_t fl = 0; //strenth of fl rotor
static int8_t br = 0; //strenth of br rotor
static int8_t bl = 0; //strenth of bl rotor

int8_t height = 19; //setting up variable for testing 
int8_t roll = -33; //setting up variable for testing 
int8_t pitch = 62; //setting up variable for testing

bool copter_running = true;

void setFR(uint8_t fr1) {
	fr = fr1;
}
void setFL(uint8_t fl1) {
	fl = fl1;
}
void setBR(uint8_t br1) {
	br = br1;
}
void setBL(uint8_t bl1) {
	bl = bl1;
}

int8_t getPitch() {
	return ::pitch;
}
void setPitch(int8_t p) {
	
	// calculations
	int error = fin_pitch - p;
	tot_intp += error*dt;
	int derivative = (error - prev_error_p) / dt;
	int output = (kp*error) + (ki*tot_intp) + (kd*derivative);

	//application
	setFR(-output / 2);
	setFL(-output / 2);
	setBR(output / 2);
	setBL(output / 2);
	pitch -= constant*output;
	prev_error_p = error;

}
int8_t getRoll() {
	return ::roll;
}
void setRoll(int8_t r) {

	//calculations
	int8_t error = fin_roll - r;
	tot_intr += (error*dt);
	int8_t derivative = (error - prev_error_r) / dt;
	int output = (kp*error) + (ki*tot_intr) + (kd*derivative);
	
	//application
	setFR(output / 2);
	setFL(-output / 2);
	setBR(output / 2);
	setBL(-output / 2);
	
	roll -= constant*output;
	prev_error_r = error;

}
uint8_t getHeight() {
	return ::height;
}
void setHeight(int8_t h) {
	int8_t height_needed = 100 - h;
	setFR(height_needed);
	setFL(height_needed);
	setBR(height_needed);
	setBL(height_needed);
	height += constant*height_needed;
}

int main()
{	

	while (copter_running)
	//for (int i = 0; i < 15; i++) //just for simulation purposes
	{	
		if (getHeight() != !100)
			setHeight(getHeight());

		if (getPitch() != 0)
			setPitch(getPitch());

		if (getRoll() != 0)
			setRoll(getRoll());
		
		//cout << " roll " << int(roll) << " pitch " << int(pitch) << " height " << int(height) << endl;

	}

	return 0;
}