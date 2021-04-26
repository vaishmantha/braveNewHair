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
   
  
//    printf("After accumulating forces: %lf, %lf, %lf", jello->f[50].x, jello->f[50].y, jello->f[50].z);
//    printf("p[i]: %f, %f, %f\n", jello->p[50].x, jello->p[50].y, jello->p[50].z);
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


        if(i==50){
            printf("p[i]: %f, %f, %f\n", jello->p[i].x, jello->p[i].y, jello->p[i].z);
            printf("p[i+1]: %f, %f, %f\n", jello->p[i+1].x, jello->p[i+1].y, jello->p[i+1].z);

            printf("edge: %f, %f, %f\n", edge.x, edge.y, edge.z);
            printf("rest_edge: %f, %f, %f\n", rest_edge.x, rest_edge.y, rest_edge.z);
        }
        

        pNORMALIZE(edge); // edge now holds edge unit vector
        curr_length = length;
        // if (i == 1) {
        //     //printf("curr_length: %f", curr_length);
        // }
        pNORMALIZE(rest_edge);
        rest_length = length;
        // if (i == 1) {
        //     //printf("rest_length: %f", rest_length);
        // }


        pDIFFERENCE(jello->v[i + 1], jello->v[i], delta_v);
        if (i == 1) {
            printf("delta_v: %f, %f, %f\n", delta_v.x, delta_v.y, delta_v.z);
        }
        pDOT(delta_v, edge, dot);
        if (i == 1) {
            printf("dot: %f\n", dot);
        }
        pMULTIPLY(edge, (jello->dStretch * dot), damp_term);

        if (i == 1) {
            // printf("edge: %f, %f, %f\n", edge.x, edge.y, edge.z);
            // printf("rest_edge: %f, %f, %f\n", rest_edge.x, rest_edge.y, rest_edge.z);

            printf("rest_length: %f\n", rest_length);
            printf("curr_length: %f\n", curr_length);
        }    
        
        pMULTIPLY(edge, (jello->kStretch * (rest_length - curr_length)), spring_term);

        if (i == 1) {
            printf("spring_term: %f, %f, %f\n", spring_term.x, spring_term.y, spring_term.z);
            printf("damp_term: %f, %f, %f\n", damp_term.x, damp_term.y, damp_term.z);
        }        
        pSUM(spring_term, damp_term, stretch_force);
        pMULTIPLY(stretch_force, -1.0, neighbor_force);

        pSUM(stretch_force, jello->f[i], jello->f[i]);
        if (i == 1) {
            printf("stretch_force: %f, %f, %f\n", stretch_force.x, stretch_force.y, stretch_force.z);
            printf("neighbor_force: %f, %f, %f\n", neighbor_force.x, neighbor_force.y, neighbor_force.z);
        }        
        pSUM(neighbor_force, jello->f[i + 1], jello->f[i + 1]);
    }
 }

 void gravity(struct world *jello) {
     struct point g;
     g.x = 0.0;
     g.y = 0.0;
     g.z = -9.81;

     struct point w; 
     for (int i = 0; i < numPoints; i++)
     {
        pMULTIPLY(g, jello->mass, w);
        pSUM(jello->f[i], w, jello->f[i]); 
        // jello->f[i].z -=9.810;
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
