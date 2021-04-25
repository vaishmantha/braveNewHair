 /*

   USC/Viterbi/Computer Science
   "Jello Cube" Assignment 1 starter code

 */

 #include "jello.h"
 #include "physics.h"

 /* Computes acceleration to every control point of the jello cube, 
    which is in state given by 'jello'.
    Returns result in array 'a'. */
 void computeAcceleration(struct world * jello, struct point a[numPoints])
 {
   /* for you to implement ... */

   // Map keys to up, down, left, and right forces to interact with hair? (Would
   // apply the forces here; mapping keys would be in input.cpp)

   // EMPTY force accumulator jello->f[numPoints]
   int i;
   for (i = 0; i < numPoints; i++) {
    //    pMAKE(0.0, 0.0, 0.0, jello->f[i]);
    jello->f[i].x = 0;
    jello->f[i].y = 0;
    jello->f[i].z = 0;
   }

   stretchSpringForce(jello); // Equivalent of computeStructuralSpringForces

   // smoothing(); // Calculates jello->p_smoothed[numPoints] (smoothed positions)
   // getFrames(); // Frames (jello->local_frames) are computed along smoothed
                   // representation of hair curve (to reduce sensitivity of frame
                   // to changes in hair positions)
   // getReferenceVectors(); // Uses local frames to calculate reference vectors jello->t
   // bendSpringForce(jello, forceAccumulator); // Uses reference vectors to calculate
                                                // bending spring force and apply to
                                                // each point and their neighbor

   gravity(jello); // Equivalent of computeExternalForces()

   // Compute acceleration for each point
   for (i = 0; i < numPoints; i++) {
       a[i].x = jello->f[i].x / jello->mass;
       a[i].y = jello->f[i].y / jello->mass;
       a[i].z = jello->f[i].z / jello->mass;
   }

   // collisions();
 }

 void stretchSpringForce(struct world *jello) {
    // For each point mass, add stretch spring force to its force accumulator and
    // to that of the next point (equal and opposite)
    for (int i = 0; i < (numPoints - 1); i++) {
        // Damped stretch spring equation from paper
        // TODO: add kStretch (spring coeff) and dStretch (damping coeff) to jello
        //       struct AND initialize them in createWorld/input.cpp
        struct point edge;
        struct point rest_edge;
        double length; // length of edge (will be assigned value by pNORMALIZE)
        double curr_length;
        double rest_length;
        struct point delta_v;
        double dot; // dot product of delta_v and edge unit vector
        struct point spring_term;
        struct point damp_term;
        struct point stretch_force;
        struct point neighbor_force; // equal and opposite of stretch_force, to be
                                     // applied to neighbor

        pDIFFERENCE(jello->p[i+1], jello->p[i], edge);
        pDIFFERENCE(jello->p0[i+1], jello->p0[i], rest_edge);

        pNORMALIZE(edge); // edge now holds edge unit vector
        curr_length = length;
        pNORMALIZE(rest_edge);
        rest_length = length;

        pDIFFERENCE(jello->v[i + 1], jello->v[i], delta_v);
        pDOT(delta_v, edge, dot);
        pMULTIPLY(edge, (jello->dStretch * dot), damp_term);

        pMULTIPLY(edge, (jello->kStretch * (rest_length - curr_length)), spring_term);

        pSUM(spring_term, damp_term, stretch_force);
        pMULTIPLY(stretch_force, -1.0, neighbor_force);

        pSUM(stretch_force, jello->f[i], jello->f[i]);

        pSUM(neighbor_force, jello->f[i + 1], jello->f[i + 1]);
    }
 }

 void gravity(struct world *jello) {
     for (int i = 0; i < numPoints; i++) {
         jello->f[i].z -= 9.81;
     }
 }

  /* performs one step of SYMPLECTIC Euler Integration */
  /* as a result, updates the jello structure */
  void Euler(struct world * jello)
  {

    // TODO: change CreateWorld to user Euler instead of RK4

    int i,j,k;
    struct point a[numPoints];

    computeAcceleration(jello, a);
    
    // Updating position/velocity for all points except first point (pinned)
    for (i = 1; i < numPoints; i++) {
        jello->v[i].x += jello->dt * a[i].x;
        jello->v[i].y += jello->dt * a[i].y;
        jello->v[i].z += jello->dt * a[i].z;

        jello->p[i].x += jello->dt * jello->v[i].x;
        jello->p[i].y += jello->dt * jello->v[i].y;
        jello->p[i].z += jello->dt * jello->v[i].z;
    }
  }

  /* performs one step of RK4 Integration */
  /* as a result, updates the jello structure */
  void RK4(struct world * jello)
  {
    // point F1p[8][8][8], F1v[8][8][8], 
    //       F2p[8][8][8], F2v[8][8][8],
    //       F3p[8][8][8], F3v[8][8][8],
    //       F4p[8][8][8], F4v[8][8][8];

    // point a[8][8][8];


    // struct world buffer;

    // int i,j,k;

    // buffer = *jello; // make a copy of jello

    // computeAcceleration(jello, a);

    // for (i=0; i<=7; i++)
    //   for (j=0; j<=7; j++)
    //     for (k=0; k<=7; k++)
    //     {
    //        pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
    //        pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
    //        pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
    //        pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
    //        pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
    //        pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
    //     }

    // computeAcceleration(&buffer, a);

    // for (i=0; i<=7; i++)
    //   for (j=0; j<=7; j++)
    //     for (k=0; k<=7; k++)
    //     {
    //        // F2p = dt * buffer.v;
    //        pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
    //        // F2v = dt * a(buffer.p,buffer.v);     
    //        pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
    //        pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
    //        pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
    //        pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
    //        pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
    //     }

    // computeAcceleration(&buffer, a);

    // for (i=0; i<=7; i++)
    //   for (j=0; j<=7; j++)
    //     for (k=0; k<=7; k++)
    //     {
    //        // F3p = dt * buffer.v;
    //        pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
    //        // F3v = dt * a(buffer.p,buffer.v);     
    //        pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
    //        pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
    //        pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
    //        pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
    //        pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
    //     }
         
    // computeAcceleration(&buffer, a);


    // for (i=0; i<=7; i++)
    //   for (j=0; j<=7; j++)
    //     for (k=0; k<=7; k++)
    //     {
    //        // F3p = dt * buffer.v;
    //        pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
    //        // F3v = dt * a(buffer.p,buffer.v);     
    //        pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

    //        pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
    //        pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
    //        pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
    //        pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
    //        pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
    //        pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
    //        pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

    //        pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
    //        pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
    //        pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
    //        pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
    //        pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
    //        pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
    //        pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
    //     }

    // return;  
  }
