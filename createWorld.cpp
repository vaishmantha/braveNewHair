/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  createWorld utility to create your own world files

  Note: this utility uses its own copy of writeWorld routine, which is identical to the one
  found in input.cpp . If you need to change that routine, or even the definition of the
  world structure (you don't have to do this unless you decide to do some fancy
  extra credit), you have to update both copies.

*/

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define numPoints 10
#define pi 3.141592653589793238462643383279 

struct point 
{
   double x;
   double y;
   double z;
};

struct world
{
  char integrator[10]; // "RK4" or "Euler"
  double dt; // timestep, e.g.. 0.001
  double dt_damp; //damping loop timestep
  int n; // display only every nth timestep
  double kElastic; // Hook's elasticity coefficient for all springs except collision springs
  double dElastic; // Damping coefficient for all springs except collision springs
  double kStretch; // Spring  constant for stretch spring
  double dStretch; // Damping constant for stretch spring
  double kCollision; // Hook's elasticity coefficient for collision springs
  double dCollision; // Damping coefficient collision springs
  double mass; // mass of each of the 512 control points, mass assumed to be equal for every control point
  int incPlanePresent; // Is the inclined plane present? 1 = YES, 0 = NO
  double a,b,c,d; // inclined plane has equation a * x + b * y + c * z + d = 0; if no inclined plane, these four fields are not used
  int resolution; // resolution for the 3d grid specifying the external force field; value of 0 means that there is no force field
  struct point * forceField; // pointer to the array of values of the force field
  struct point p[numPoints]; // position of the 512 control points
  struct point v[numPoints]; // velocities of the 512 control points
};


/* writes the world parameters to a world file on disk*/
/* fileName = string containing the name of the output world file, ex: jello1.w */
/* function creates the output world file and then fills it corresponding to the contents
   of structure 'jello' */
/* function aborts the program if can't access the file */

/* writes the world parameters to a world file on disk*/
/* fileName = string containing the name of the output world file, ex: jello1.w */
/* function creates the output world file and then fills it corresponding to the contents
   of structure 'jello' */
/* function aborts the program if can't access the file */
void writeWorld(const char * fileName, struct world * jello)
{
  int i,j,k;
  FILE * file;
  
  file = fopen(fileName, "w");
  if (file == NULL) {
    printf ("can't open file\n");
    exit(1);
  }

  /* write integrator algorithm */ 
  fprintf(file,"%s\n",jello->integrator);

  /* write timestep */
  fprintf(file,"%lf %d\n",jello->dt,jello->n);
  fprintf(file,"%lf %d\n",jello->dt_damp,jello->n);

  /* write physical parameters */
  fprintf(file, "%lf %lf %lf %lf %lf %lf\n", 
    jello->kElastic, jello->dElastic, jello->dStretch, jello->kStretch, jello->kCollision, jello->dCollision);

  /* write mass */
  fprintf(file, "%lf\n", jello->mass);

  /* write info about the plane */
  fprintf(file, "%d\n", jello->incPlanePresent);
  if (jello->incPlanePresent == 1)
    fprintf(file, "%lf %lf %lf %lf\n", jello->a, jello->b, jello->c, jello->d);

  /* write info about the force field */
  fprintf(file, "%d\n", jello->resolution);
  if (jello->resolution != 0)
    for (i=0; i<= jello->resolution-1; i++)
      for (j=0; j<= jello->resolution-1; j++)
        for (k=0; k<= jello->resolution-1; k++)
          fprintf(file, "%lf %lf %lf\n", 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].x, 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].y, 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].z);
  

  /* write initial point positions */
  for (i = 0; i < numPoints; i++)
  {
    fprintf(file, "%lf %lf %lf\n", 
      jello->p[i].x, jello->p[i].y, jello->p[i].z);
  }
      
  /* write initial point velocities */
  for (i = 0; i < numPoints; i++)
  {
    fprintf(file, "%lf %lf %lf\n", 
      jello->v[i].x, jello->v[i].y, jello->v[i].z);
  }

  fclose(file);
  
  return;
}

/* modify main to create your own world */
int main()
{
  struct world jello;
  int i,j,k;
  double x,y,z;

  // set the integrator and the physical parameters
  // the values below are EXAMPLES, to be modified by you as needed
  strcpy(jello.integrator,"Euler");
  jello.dt=0.00005000;
  jello.dt_damp = 0.00006000; 
  jello.n=1;
  jello.kElastic=800;
  jello.dElastic=40;
  jello.kStretch = 4000;
  jello.dStretch = 450; 
  jello.kCollision = 1400.0;
  jello.dCollision=0.25;
  jello.mass= 5; // / (double)512;

  // set the inclined plane (not used in this assignment; ignore)
  jello.incPlanePresent=1;
  jello.a=-1;
  jello.b=1;
  jello.c=1;
  jello.d=2;

  // set the external force field
  jello.resolution=3;   // FIX: CHANGED FROM 30 
  jello.forceField = 
    (struct point *)malloc(jello.resolution*jello.resolution*jello.resolution*sizeof(struct point));
  for (i=0; i<= jello.resolution-1; i++)
    for (j=0; j<= jello.resolution-1; j++)
      for (k=0; k<= jello.resolution-1; k++)
      {
        // set the force at node i,j,k
        // actual space location = x,y,z
        x = -2 + 4*(1.0 * i / (jello.resolution-1));
        y = -2 + 4*(1.0 * j / (jello.resolution-1));
        z = -2 + 4*(1.0 * k / (jello.resolution-1));

        jello.forceField[i * jello.resolution * jello.resolution 
          + j * jello.resolution + k].x = 0; 
        jello.forceField[i * jello.resolution * jello.resolution 
          + j * jello.resolution + k].y = 0;
        jello.forceField[i * jello.resolution * jello.resolution 
          + j * jello.resolution + k].z = 0;
      }

  double radius = 0.25;
  double c = 0.05;
  double radians = 0.0;

  // set the positions of control points
  for (i=0; i< numPoints; i++)
  {
    radians = -((i / 5.0) * 360.0 * pi/180.0);
    jello.p[i].x = radius * cos(radians);
    jello.p[i].y = radius * sin(radians);
    jello.p[i].z = c * radians + 1.5;
  }

  // set the velocities of control points
  for (i=0; i< numPoints; i++)
  {
    jello.v[i].x=0.0;
    jello.v[i].y=0.0;
    jello.v[i].z=0.0;
  }

  // write the jello variable out to file on disk
  // change jello.w to whatever you need
  writeWorld("world/temp.w",&jello);

  return 0;
}


