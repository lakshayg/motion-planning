#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#define size(x) (sizeof(x)/sizeof(*x))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):(-(x)))
#define DOF 2
#define MAX_PRIM 500

typedef struct {
	double disp[DOF];   // displacement caused by the primitive
	double vel[DOF][2]; // start and end velocities for each DoF
	double cmd[DOF][3]; // quadratic representing velocity of joints 1 and 2
	double time;        // duration of motion primitive
	double length;      // path length
} Mprim;

int n_primitives = 0;
Mprim primitives[MAX_PRIM];

void primitive(double disp[], double v_start[], double v_end[], double v_max[]);
double bisection_search(double ds, double td0, double td1, double tdm);
void print(Mprim *m);

int main() {
	double vel[9][DOF];                 // allowed joint velocities
	double disp[DOF];                   // array for storing displacements
	double v_max[DOF] = {2.5, 2.5};
	int i, j, x, y;

	// all possible velocity pairs
	for (i = 0; i < 9; i++) {
		vel[i][0] = (i/3) - 1.;
		vel[i][1] = (i%3) - 1.;
	}

	for (i = 0; i < size(vel); ++i) {      // ] all possible combinations
		for (j = 0; j < size(vel); ++j) {  // ] of joint velocities
			for (x = -1; x <= 1; ++x) {
				for (y = -1; y <= 1; ++y) {
					disp[0] = 1. * x; disp[1] = 1. * y;
					primitive(disp, vel[i], vel[j], v_max);
				}
			}
		}
	}

	// print the motion primivives
	for(i = 0; i < n_primitives; ++i) {
		print(&primitives[i]);
	}

	return 0;
}

void print(Mprim *m) {
	int i;
	printf("Motion Primitive:\n");
	printf("Duration: %8.3f\n", m->time);
	printf("Displacement: [%8.3f  %8.3f]\n", m->disp[0], m->disp[1]);
	printf("Start/End Velocities: [%8.3f  %8.3f]  [%8.3f  %8.3f]\n", \
			m->vel[0][0], m->vel[0][1], m->vel[1][0], m->vel[1][1]);
	printf("Velocity command:\n");
	for (i = 0; i < DOF; ++i)
		printf("  %8.3f  %8.3f  %8.3f\n", m->cmd[i][0], m->cmd[i][1], m->cmd[i][2]);
	printf("\n");
}

// generate a motion primitive from the given conditions
// INPUTS:
//     disp    - displacement caused by the primitive
//     v_start - starting velocity
//     v_end   - ending velocity
//     v_max   - maximum permitted velocity
void primitive(double disp[], double v_start[], double v_end[], double v_max[]) {
	int i;
	const bool log = false;
	double time;
	const double eps = 1e-3;
	const double teps = 1e-1; // no motion primitive < 0.1 sec

	// check if the input is consistent
	for (i = 0; i < DOF; ++i) {
		assert(abs(v_start[i]) <= v_max[i]);
		assert(abs(v_end[i]) <= v_max[i]);
	}

	// determine the duration of motion primitive
	time = -1.;
	for (i = 0; i < DOF; ++i) {
		// TODO: the case when displacement is ZERO has not been handled properly
		if (abs(disp[i]) < eps) {
			if (log) printf("Displacement required for joint is ZERO\n");
			if (v_start[i] == v_end[i]) {
				time = max(time, 0);
			}
			else {
				if (log) printf("Cannot generate a motion primitive with INFINITE acceleration\n");
				return;
			}
		}
		else {
			time = max(time, bisection_search(disp[i], v_start[i], v_end[i], v_max[i]));
		}
	}
	if (time < teps) {
		// this case may occur if bisection_search fails to find a solution
		// or if no motion is required for meeting the provided conditions
		if (log) printf("Cannot generate motion primitive, time duration too small\n");
		return;
	}

	// fit a cubic equation to the path
	double p[DOF][3]; // quadratic for the velocity
	for (i = 0; i < DOF; ++i) {
		p[i][0] = v_start[i];
		p[i][1] = 2*(3*disp[i]/(time*time) - (2*v_start[i] + v_end[i])/time);
		p[i][2] = 3*(-2*disp[i]/(time*time*time) + (v_start[i]+v_end[i])/(time*time));
	}

	// store the results
	Mprim *m = &primitives[n_primitives];
	m->time = time;
	m->disp[0] = disp[0]; m->disp[1] = disp[1];
	for (i = 0; i < DOF; ++i) {
		m->vel[i][0] = v_start[i]; m->vel[i][1] = v_end[i];
		m->cmd[i][0] = p[i][0]; m->cmd[i][1] = p[i][1]; m->cmd[i][2] = p[i][2];
	}
	n_primitives++;

}

// function used in bisection search for minimizing time
double f(double disp, double td0, double td1, double tdm, double t) {
	double a = 3*disp - (2*td0 + td1)*t;
	double b = -2*disp*t + (td0 + td1)*t*t;
	double c = td0 - (a*a)/(3*b);
	return abs(c) - tdm;
}

double bisection_search(double ds, double td0, double td1, double tdm) {
	const bool log = false;
	const double eps = 1e-3;
	const int max_iter = 50;
	double t_lo, t_hi, t;
	double y[3];
	int i;

	// assuming that the solution always lies in the interval [0.1,10]
	t_lo = 0.1; t_hi = 10;
	for (i = 0; i < max_iter; ++i) {
		t = 0.5 * (t_lo + t_hi);
		y[0] = f(ds, td0, td1, tdm, t_lo);
		y[1] = f(ds, td0, td1, tdm, t);
		y[2] = f(ds, td0, td1, tdm, t_hi);

		if (y[0]*y[2] > 0) {
			if (log) printf("Solution bracket lost, returning with error value\n");
			return -1.;
		}

		if (y[0]*y[1] >= 0) t_lo = t;
		if (y[1]*y[2] >= 0) t_hi = t;
		if (abs(t_hi - t_lo) < eps) {
			if (log) printf("Bisection converged after %d iterations\n", i);
			return t_hi;
		}
	}

	if (log) printf("Iteration limit reached before convergence\n");
	return t_hi;
}

