#include "cpplapack_plus.h"
// landmarks.dat にかならず dgematrix形式でlandmarkの正確な位置を保存．


#define MCL_M 100 // number of particles
#define MCL_L 4 //number of landmarks
	
#define MCL_F_N 0.1 // forward move - noise
#define MCL_THETA_N 0.1 // turn round - noise
#define MCL_VISION_N 0.1 // estimated landmark position - noise

class ALMcl{
public:
  dgematrix X; // particles  one row = one particle (x,y,theta, phi)
  dcovector w; // weight for each particle
  
  dgematrix landmarks; // data of landmarks L*2 (x-coordinate, y-coordinate) for each landmarkd 

  double forward_noise; // noise s.d. happened when a robot move forward
  double theta_noise; // noise s.d. happend when a robot turn
  double vision_noise; // noise s.d. of estimated posiiton for x y coordinate

  void update(double d_forward,double d_theta,double phi, dgematrix view);
  //rotating d_tehta, and then move d_forward. after that the robot observed "view" with phi head direction.

  drovector get_pos();// method returning current position in 4 dimensional drovector

  void move(double d_forward, double d_theta, double phi); // move every particle

  drovector vision_simulator(drovector pos,drovector mark);// get ideal landmark observation for a landmark withoug noise


  double prob_vision(drovector pos,drovector mark, drovector vision);
  // calculate a probability (without normalization) for a landmark and a estimated position data from vision observation
  double prob_vision(drovector pos,drovector mark, dgematrix view);
  double prob_vision(drovector pos, dgematrix view);
  void prob_vision(dgematrix view);// update weight by calculateing max prob_vision for every pair of view and landmarks

  void init(int M, const char *filename);

  //private:
  drovector move(drovector pos,double d_forward, double d_theta,double phi); // move with noise term

};

