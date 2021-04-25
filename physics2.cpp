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

   smoothing(jello); // Calculates jello->p_smoothed[numPoints] (smoothed positions)
   // getFrames(); // Frames (jello->local_frames) are computed along smoothed
                   // representation of hair curve (to reduce sensitivity of frame
                   // to changes in hair positions)
   // getReferenceVectors(); // Uses local frames to calculate reference vectors jello->t
   // bendSpringForce(jello, forceAccumulator); // Uses reference vectors to calculate
                                                // bending spring force and apply to
                                                // each point and their neighbor

   gravity(jello); // Equivalent of computeExternalForces()
   
  
   printf("After accumulating forces: %lf, %lf, %lf", jello->f[50].x, jello->f[50].y, jello->f[50].z);
   printf("p[i]: %f, %f, %f\n", jello->p[50].x, jello->p[50].y, jello->p[50].z);
    // printf("p[i+1]: %f, %f, %f\n", jello->p[50+1].x, jello->p[i+1].y, jello->p[i+1].z);   

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


        //if(i==50){
        //    printf("p[i]: %f, %f, %f\n", jello->p[i].x, jello->p[i].y, jello->p[i].z);
        //    printf("p[i+1]: %f, %f, %f\n", jello->p[i+1].x, jello->p[i+1].y, jello->p[i+1].z);

        //    printf("edge: %f, %f, %f\n", edge.x, edge.y, edge.z);
        //    printf("rest_edge: %f, %f, %f\n", rest_edge.x, rest_edge.y, rest_edge.z);
        //}
        

        pNORMALIZE(edge); // edge now holds edge unit vector
        curr_length = length;
        if (i == 50) {
            //printf("curr_length: %f", curr_length);
        }
        pNORMALIZE(rest_edge);
        rest_length = length;
        if (i == 1) {
            //printf("rest_length: %f", rest_length);
        }


        pDIFFERENCE(jello->v[i + 1], jello->v[i], delta_v);
        //if (i == 50) {
        //    printf("delta_v: %f, %f, %f\n", delta_v.x, delta_v.y, delta_v.z);
        //}
        pDOT(delta_v, edge, dot);
        //if (i == 50) {
        //    printf("dot: %f\n", dot);
        //}
        pMULTIPLY(edge, (jello->dStretch * dot), damp_term);

        //if (i == 50) {
            // printf("edge: %f, %f, %f\n", edge.x, edge.y, edge.z);
            // printf("rest_edge: %f, %f, %f\n", rest_edge.x, rest_edge.y, rest_edge.z);

        //    printf("rest_length: %f\n", rest_length);
        //    printf("curr_length: %f\n", curr_length);
        //}    
        
        pMULTIPLY(edge, (jello->kStretch * (rest_length - curr_length)), spring_term);

        //if (i == 50) {
        //    printf("spring_term: %f, %f, %f\n", spring_term.x, spring_term.y, spring_term.z);
        //    printf("damp_term: %f, %f, %f\n", damp_term.x, damp_term.y, damp_term.z);
        //}        
        pSUM(spring_term, damp_term, stretch_force);
        pMULTIPLY(stretch_force, -1.0, neighbor_force);

        pSUM(stretch_force, jello->f[i], jello->f[i]);
        //if (i == 50) {
        //    printf("stretch_force: %f, %f, %f\n", stretch_force.x, stretch_force.y, stretch_force.z);
        //    printf("neighbor_force: %f, %f, %f\n", neighbor_force.x, neighbor_force.y, neighbor_force.z);
        //}        
        pSUM(neighbor_force, jello->f[i + 1], jello->f[i + 1]);
    }
 }

 void smoothing(struct world *jello) {
     // TODO: add p_smooth to jello struct (jello.h; no need to init in world file)
     // TODO: add pLENGTH macro to jello.h
     // TODO: move avg_rest_len calculation to input and store inside of jello struct
     //       to save computation expense

     int i;
     struct point len;
     double avg_rest_len, total_len;
     for (i = 0; i < (numPoints - 1); i++)
     {
         pDIFFERENCE(jello->p0[i + 1], jello->p0[i], len);
         total_len += pLENGTH(len);
     }

     avg_rest_len = total_len / (numPoints - 1); // TODO: why did Carter divide by 7?
                                                 // Maybe a "segment" is a larger
                                                 // region than just the distance
                                                 // between two points

     double alpha = 10.0; // smoothing amount
     double beta = std::fmin(1.0, 1.0 - exp(-1.0 * avg_rest_len / alpha));

     struct point d[numPoints];
     struct point d_prior, d_2prior, to_next;
     struct point d_init;
     pDIFFERENCE(jello->p[1], jello->p[0], d_init);
     for (i = 0; i < (numPoints - 1); i++)
     {
         d_prior  = ((i - 1) < 0) ? d_init : d[i - 1];
         d_2prior = ((i - 2) < 0) ? d_init : d[i - 2];

         pDIFFERENCE(jello->p[i + 1], jello->p[i], to_next);

         // d_prior now holds first term
         pMULTIPLY(d_prior, 2.0 * (1 - beta), d_prior);
         // d_2prior now holds second term
         pMULTIPLY(d_2prior, -1.0 * (1.0 - beta) * (1.0 - beta), d_2prior);
         // to_next now holds third term
         pMULTIPLY(to_next, beta * beta, to_next);

         pSUM(d_prior, d_2prior, d[i]);
         pSUM(d[i], to_next, d[i]);
     }

     for (i = 0; i < numPoints; i++) {
          if (i == 0) {
              jello->p_smooth[i] = jello->p0[i];
          } else {
              pSUM(jello->p_smooth[i - 1], d[i - 1], jello->p_smooth[i]);
          }
     }
 }

 void gravity(struct world *jello) {
     struct point g;
     g.x = 0.0;
     g.y = 0.0;
     g.z = -9.81;
     for (int i = 0; i < numPoints; i++)
     {
         pMULTIPLY(g, jello->mass, jello->f[i]);
     }
 }

  /* performs one step of SYMPLECTIC Euler Integration */
  /* as a result, updates the jello structure */
  void Euler(struct world * jello)
  {
    // printf("Called Euler\n");
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

        // printf("%f, %f, %f\n", jello->p[i].x, jello->p[i].y, jello->p[i].z); 
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
