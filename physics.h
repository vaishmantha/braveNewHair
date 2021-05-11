/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

void computeAcceleration(struct world * jello, struct point a[numPoints]);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);

void stretchSpringForce(struct world *jello);
void smoothing(struct world *jello);
void getFrames(struct world *jello, double frames[numPoints][3][3]);
void transpose(double m[3][3], double mT[3][3]);
void frameTimesVector(double frame[3][3], struct point& vec, struct point& reference);
void getInitialReferenceVectors(struct world *jello);
void getReferenceVectors(struct world *jello);
void bendSpringForce(struct world *jello);
void gravity(struct world *jello);
void wind(struct world *jello); 
void dampingForces(struct world *jello, struct point a[numPoints]);
void stretchDampingForce(struct world *jello); 
void bendDampingForce(struct world *jello); 
void integrateForces(struct world* jello);
#endif
