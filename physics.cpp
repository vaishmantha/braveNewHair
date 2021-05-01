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

//    smoothing(jello); // Calculates jello->p_smoothed[numPoints] (smoothed positions)
    printf("BEFORE: %f %f %f | %f %f %f | %f %f %f\n", jello->F[5][0][0], jello->F[5][0][1], jello->F[5][0][2],jello->F[5][1][0], jello->F[5][1][1], jello->F[5][1][2],jello->F[5][2][0], jello->F[5][2][1], jello->F[5][2][2]); 

   getFrames(jello, jello->F); // Frames (jello->local_frames) are computed along
                               // smoothed representation of hair curve (to reduce
                               // sensitivity of frame to changes in hair positions)
    printf("AFTER: %f %f %f | %f %f %f | %f %f %f\n", jello->F[5][0][0], jello->F[5][0][1], jello->F[5][0][2],jello->F[5][1][0], jello->F[5][1][1], jello->F[5][1][2],jello->F[5][2][0], jello->F[5][2][1], jello->F[5][2][2]); 

   getReferenceVectors(jello); // Uses local frames to calculate reference vectors jello->t
   bendSpringForce(jello); // Uses reference vectors to calculate
                                                // bending spring force and apply to
                                                // each point and their neighbor

   gravity(jello); // Equivalent of computeExternalForces()
  
    // printf("After accumulating forces: %lf, %lf, %lf", jello->f[50].x, jello->f[50].y, jello->f[50].z);
    // printf("p[i]: %f, %f, %f\n", jello->p[50].x, jello->p[50].y, jello->p[50].z);
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

        pNORMALIZE(edge); // edge now holds edge unit vector
        curr_length = length;
        pNORMALIZE(rest_edge);
        rest_length = length;
        pDIFFERENCE(jello->v[i + 1], jello->v[i], delta_v);
        // if (i == 1) {
        //     printf("delta_v: %f, %f, %f\n", delta_v.x, delta_v.y, delta_v.z);
        // }
        pDOT(delta_v, edge, dot);
        // if (i == 1) {
        //     printf("dot: %f\n", dot);
        // }
        pMULTIPLY(edge, (jello->dStretch * dot), damp_term);

        // if (i == 1) {
        //     printf("rest_length: %f\n", rest_length);
        //     printf("curr_length: %f\n", curr_length);
        // }    
        
        pMULTIPLY(edge, (jello->kStretch * (rest_length - curr_length)), spring_term);

        // if (i == 1) {
        //     printf("spring_term: %f, %f, %f\n", spring_term.x, spring_term.y, spring_term.z);
        //     printf("damp_term: %f, %f, %f\n", damp_term.x, damp_term.y, damp_term.z);
        // }        
        pSUM(spring_term, damp_term, stretch_force);
        pMULTIPLY(stretch_force, -1.0, neighbor_force);

        pSUM(stretch_force, jello->f[i], jello->f[i]);
        // if (i == 1) {
        //     printf("stretch_force: %f, %f, %f\n", stretch_force.x, stretch_force.y, stretch_force.z);
        //     printf("neighbor_force: %f, %f, %f\n", neighbor_force.x, neighbor_force.y, neighbor_force.z);
        // }        
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
     }
 }

 void smoothing(struct world *jello) {
     // TODO: any changes besides adding to jello.h for p_smooth?
     // TODO: move avg_rest_len calculation to input and store inside of jello struct
     //       to save computation expense

     int i;
     struct point displacement;
     double len, avg_rest_len, total_len;
     total_len = 0.0; 
     for (i = 0; i < (numPoints - 1); i++)
     {
         pDIFFERENCE(jello->p0[i + 1], jello->p0[i], displacement);
         pLENGTH(displacement, len);
         total_len += len;
     }

     avg_rest_len = total_len / (numPoints - 1); // FIXME: divide by 7?
                                                 // Maybe a "segment" is a larger
                                                 // region than just the distance
                                                 // between two points

     double alpha = 5.0; // smoothing amount
     double beta = fmin(1.0, 1.0 - exp(-1.0 * avg_rest_len / alpha));

     struct point d[numPoints - 1];
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
              //jello->p_smooth[i] = jello->p0[i];  // FIXME
               jello->p[i] = jello->p0[i];
          } else {
              //pSUM(jello->p_smooth[i - 1], d[i - 1], jello->p_smooth[i]); // FIXME 
               pSUM(jello->p[i - 1], d[i - 1], jello->p[i]);
          }
     }
 }

 // Referenced from https://giordi91.github.io/post/2018-31-07-parallel-transport/
 void getFrames(struct world *jello, double frames[numPoints][3][3]) {
     // TODO: any changes besides adding to jello.h for local_frames?
     // TODO: am I passing in (references to) arrays correctly lol

     struct point up, start, end, aim, cross;
     up.x = 0.0;
     up.y = 0.0;
     up.z = 1.0;
     for (int i = 0; i < (numPoints - 1); i++) {
         pCPY(jello->p[i],     start); // FIXME: p_smooth
         pCPY(jello->p[i + 1], end);   // FIXME: p_smooth

         pDIFFERENCE(end, start, aim); // FIXME: correct order?
         double length; 
         pNORMALIZE(aim);

         CROSSPRODUCTp(aim, up, cross);

         pNORMALIZE(cross);

         CROSSPRODUCTp(cross, aim, up);
         pNORMALIZE(up);

         // jello->F[i] is the frame for point i, stored as a 3x3 matrix
         // 1st column is aim; 2nd up; 3rd cross
         frames[i][0][0] = aim.x;
         frames[i][1][0] = aim.y;
         frames[i][2][0] = aim.z;

         frames[i][0][1] = up.x;
         frames[i][1][1] = up.y;
         frames[i][2][1] = up.z;

         frames[i][0][2] = cross.x;
         frames[i][1][2] = cross.y;
         frames[i][2][2] = cross.z;
     }
 }

  // Takes in matrix m, transposes it, and stores in mT
 void transpose(double m[3][3], double mT[3][3]) { // TODO: reference syntax ??

     for (int i = 0; i < 3; i++)
     {
         for (int j = 0; j < 3; j++) {
             mT[j][i] = m[i][j];
         }
     }
 }

 void frameTimesVector(double frame[3][3], struct point& vec, struct point& reference) {
     double ref[3];
     ref[0] = 0.0;
     ref[1] = 0.0; 
     ref[2] = 0.0; 
     
     double v[3];
     v[0] = vec.x;
     v[1] = vec.y;
     v[2] = vec.z;
    //printf("Frame: \n[%lf, %lf, %lf] \n[%lf, %lf, %lf] \n[%lf, %lf, %lf]\n",
    //          frame[0][0], frame[0][1], frame[0][2], frame[1][0], frame[1][1], frame[1][2], frame[2][0], frame[2][1], frame[2][2]);
    //printf("Vector: \n[%lf, %lf, %lf]\n" , v[0], v[1], v[2]);

    for (int i = 0; i < 3; i++)
     {
         for (int j = 0; j < 3; j++)
         {
            //  printf("frame: %f, v: %f", frame[i][j], v[j]);
             ref[i] += frame[i][j] * v[j];
         }
     }

     reference.x = ref[0];
     reference.y = ref[1];
     reference.z = ref[2];
     //printf("Reference: \n[%lf, %lf, %lf]\n", ref[0], ref[1], ref[2]);
 }

 void getInitialReferenceVectors(struct world *jello){
     // Transpose frame before calling frameTimesEdge
     double frameT[3][3];
     struct point edge;
     struct point zero; 
     zero.x = 0.0;
     zero.y = 0.0;
     zero.z = 0.0;
     jello->t0[0] = zero; 
     for (int i = 1; i < (numPoints - 1); i++) { // TODO: i = 0 ??
        // printf("F0: %lf\n", jello->F0[i]);
        transpose(jello->F0[i - 1], frameT);
         pDIFFERENCE(jello->p0[i + 1], jello->p0[i], edge); // FIXME: correct order?
        printf("edge: %f, %f, %f\n", edge.x, edge.y, edge.z);
        printf("frameT: %f %f %f | %f %f %f | %f %f %f\n", frameT[0][0], frameT[0][1], frameT[0][2],frameT[1][0], frameT[1][1], frameT[1][2],frameT[2][0], frameT[2][1], frameT[2][2]); 
        frameTimesVector(frameT, edge, jello->t0[i]);
        printf("t0NEXT: %f, %f, %f\n", jello->t0[i].x, jello->t0[i].y, jello->t0[i].z);
     }
 }

 void getReferenceVectors(struct world *jello) {
     struct point zero; 
     zero.x = 0.0;
     zero.y = 0.0;
     zero.z = 0.0;
     pCPY(jello->t[0], zero);
     pCPY(jello->t[numPoints - 1], zero);
     for (int i = 1; i < (numPoints - 1); i++)
     {
         printf("t0: %f, %f, %f\n", jello->t0[i].x, jello->t0[i].y, jello->t0[i].z);
         //frameTimesVector(jello->F[i - 1], jello->t0[i], jello->t[i]); FIXME
         jello->t[i] = jello->t0[i];
        printf("t: %f, %f, %f\n", jello->t[i].x, jello->t[i].y, jello->t[i].z);
     }
 }

 void bendSpringForce(struct world *jello) {
  // Control bending similar to elastic rods model 
    for (int i = 0; i < (numPoints - 1); i++) {
        // Damped stretch spring equation from paper
        struct point og_edge; 
        struct point edge;
        double length; // length of edge (will be assigned value by pNORMALIZE)
        struct point delta_v;
        double dot; // dot product of delta_v and edge unit vector
        struct point spring_term;
        struct point damp_term;
        struct point bending_force;
        
        struct point neighbor_force; // equal and opposite of stretch_force, to be
                                     // applied to neighbor

        pDIFFERENCE(jello->p[i+1], jello->p[i], og_edge);
        edge = og_edge; 
        pNORMALIZE(edge); // edge now holds edge unit vector
        if (i == 1) {
            printf("og_edge: %f, %f, %f\n", og_edge.x, og_edge.y, og_edge.z);
            printf("edge: %f, %f, %f\n", edge.x, edge.y, edge.z);
        }    
        pDIFFERENCE(jello->v[i + 1], jello->v[i], delta_v);
        pDOT(delta_v, edge, dot);
        if (i == 1) {
            printf("deltav: %f, %f, %f\n", delta_v.x, delta_v.y, delta_v.z);
            printf("dot: %f\n", dot);
        }  

        pMULTIPLY(edge, dot, damp_term);   
        pDIFFERENCE(delta_v, damp_term, damp_term);
        pMULTIPLY(damp_term, jello->dElastic, damp_term); 

        pDIFFERENCE(edge, jello->t[i], spring_term)

        pMULTIPLY(spring_term, jello->kElastic, spring_term);
        if (i == 1) {
            printf("damp_term: %f, %f, %f\n", damp_term.x, damp_term.y, damp_term.z);
            printf("spring_term: %f, %f, %f\n", spring_term.x, spring_term.y, spring_term.z);
        }  
        pSUM(spring_term, damp_term, bending_force);

        pSUM(bending_force, jello->f[i], jello->f[i]);

        pMULTIPLY(bending_force, -1.0, neighbor_force);
        pSUM(neighbor_force, jello->f[i+1], jello->f[i+1]);
    }
 }

  /* performs one step of SYMPLECTIC Euler Integration */
  /* as a result, updates the jello structure */
  void Euler(struct world * jello)
  {
    // printf("Called Euler\n");
    // TODO: change CreateWorld to user Euler instead of RK4 [DONE]
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
