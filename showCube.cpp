/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "showCube.h"

// int pointMap(int side, int i, int j)
// {
//   int r;

//   switch (side)
//   {
//   case 1: //[i][j][0] bottom face
//     r = 64 * i + 8 * j;
//     break;
//   case 6: //[i][j][7] top face
//     r = 64 * i + 8 * j + 7;
//     break;
//   case 2: //[i][0][j] front face
//     r = 64 * i + j;
//     break;
//   case 5: //[i][7][j] back face
//     r = 64 * i + 56 + j;
//     break;
//   case 3: //[0][i][j] left face
//     r = 8 * i + j;
//     break;
//   case 4: //[7][i][j] right face
//     r = 448 + 8 * i + j;
//     break;
//   }

//   return r;
// }

void showCube(struct world * jello)
{
  int i, ip; 
  // int i,j,k,ip,jp,kp;
  // point r1,r2,r3; // aux variables
  
  // /* normals buffer and counter for Gourad shading*/
  // struct point normal[8][8];
  // int counter[8][8];

  // int face;
  // double faceFactor, length;

  // if (fabs(jello->p[0][0][0].x) > 10)
  // {
  //   printf ("Your cube somehow escaped way out of the box.\n");
  //   exit(0);
  // }

  
//   #define NODE(face,i,j) (*((struct point * )(jello->p) + pointMap((face),(i),(j))))

  
  // #define PROCESS_NEIGHBOUR(di,dj,dk) \
  //   ip=i+(di);\
  //   jp=j+(dj);\
  //   kp=k+(dk);\
  //   if\
  //   (!( (ip>7) || (ip<0) ||\
  //     (jp>7) || (jp<0) ||\
  //   (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
  //      && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
  //   {\
  //     glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
  //     glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
  //   }\

  #define PROCESS_NEIGHBOUR(di) \
    ip=i+(di);\
    if\
    (!( (ip>=numPoints) || (ip<0) ) && !((i>=numPoints) || (i<0))) \
    {\
      glVertex3f(jello->p[i].x,jello->p[i].y,jello->p[i].z);\
      glVertex3f(jello->p[ip].x,jello->p[ip].y,jello->p[ip].z);\
    }\

  // render wireframe as hair 
  glLineWidth(1);
	glPointSize(5);
	glDisable(GL_LIGHTING);

	for (int i=0; i<numPoints; i++)
	{
		glBegin(GL_POINTS); // draw point
    // printf("%f\n", jello->forceField->y );
		glColor4f(jello->forceField->x / 10,jello->forceField->y/ 10 ,jello->forceField->z /10,0);  
		glVertex3f(jello->p[i].x,jello->p[i].y,jello->p[i].z);        
		glEnd();

		glBegin(GL_LINES);      
		// structural
		if (structural == 1)
		{
		  glColor4f(0,0,1,1);
		  PROCESS_NEIGHBOUR(1);
		}
                     
		glEnd();
    
		// glEnable(GL_LIGHTING);
    }
  glFrontFace(GL_CCW);
}

void showBoundingBox()
{
  int i,j;

  glColor4f(0.6,0.6,0.6,0);

  glBegin(GL_LINES);

  // front face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,-2,-2);
    glVertex3f(i,-2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(2,-2,j);
  }

  // back face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(i,2,-2);
    glVertex3f(i,2,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,2,j);
    glVertex3f(2,2,j);
  }

  // left face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(-2,i,-2);
    glVertex3f(-2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(-2,-2,j);
    glVertex3f(-2,2,j);
  }

  // right face
  for(i=-2; i<=2; i++)
  {
    glVertex3f(2,i,-2);
    glVertex3f(2,i,2);
  }
  for(j=-2; j<=2; j++)
  {
    glVertex3f(2,-2,j);
    glVertex3f(2,2,j);
  }
  
  glEnd();

  return;
}
