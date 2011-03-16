#include "ALMcl.h"



void ALMcl::move(double d_forward, double d_theta, double phi){
	for(int i=0;i<X.m;i++){
		vec_set(X,i,move(rovec_read(X,i),d_forward,d_theta,phi)) ;
	}
}

drovector ALMcl::get_pos(){
	drovector ave(4);
	ave.zero();
	for(int i=0;i<X.m;i++){
		ave += rovec_read(X,i);
	}
	return (1.0/ (double)X.m)*ave ;
}


double ALMcl::prob_vision(drovector pos,dgematrix view){
	double max = 0;
	for(int i=0; i<landmarks.m ; i++){
	  double p = prob_vision(pos,rovec_read(landmarks,i),view);
		if(p > max){max=p;}
	}
	return max;
}

void ALMcl::prob_vision(dgematrix view){
	for(int i=0;i<X.m;i++){
		w(i) = prob_vision(rovec_read(X,i),view);
	}
}


drovector ALMcl::vision_simulator(drovector pos, drovector mark){
  dcovector x(2);
  x(0) = mark(0)-pos(0);
  x(1) = mark(1)-pos(1);
 
  double angle = pos(2)+pos(3);

  dgematrix rot(2,2);
  rot(0,0) = cos(angle);
  rot(0,1) = sin(angle);
  rot(1,0) = -sin(angle);
  rot(1,1) = cos(angle);

  dcovector x_ = rot*x;

  drovector v_(2);
  v_(1) = atan2(x_(1),x_(0));
  v_(0) = nrm2(x_);

  return v_;

}


double ALMcl::prob_vision(drovector pos,drovector mark, dgematrix view){
  int argmax_ = 0;
  if(view.m == 0){return 1;}
  //  cout << rovec_read(view,0) <<endl;
  //  cout <<  prob_vision(pos,mark,rovec_read(view,0)) << endl;

  double max_ = prob_vision(pos,mark,rovec_read(view,0));

//  cout << "max" << max_ << endl;
  for(int i=1;i<view.m;i++){
    double p =  prob_vision(pos,mark,rovec_read(view,i));
    if(max_ < p){
      argmax_ = i;
      max_ = p;
    }
  }
//  cout << "argmax" << argmax_ << endl;

  return max_;

}





double ALMcl::prob_vision(drovector pos,drovector mark, drovector vision){

  drovector vision_ = vision_simulator(pos,mark);
  drovector v_(2); 
  v_(0) = vision_(0)*cos(vision_(1));
  v_(1) = vision_(0)*sin(vision_(1));

  drovector v(2);

  v(0) = vision(0)*cos(vision(1));
  v(1) = vision(0)*sin(vision(1));

  double  dist  = nrm2(v-v_);
  //cout << "dist " << dist <<endl;
  // now I'm using Gaussian distribution as a vision error
  return exp(-dist*dist/(2*vision_noise*vision_noise));

}



drovector ALMcl::move(drovector pos,double d_forward, double d_theta, double phi){
  pos(2) += d_theta + theta_noise * gauss_rand();

  cout << "theta noise sample "<< theta_noise * gauss_rand() <<endl;

  double d_forward_n = d_forward + forward_noise*gauss_rand();

  cout << "forward noise sample " <<  forward_noise*gauss_rand() << endl;

  pos(0) += d_forward_n*cos(pos(2));
  pos(1) += d_forward_n*sin(pos(2));
  pos(3) = phi;
  
  return pos;
}
 


void ALMcl::update(double d_forward,double d_theta,double phi, dgematrix view){
 
	move(d_forward,d_theta,phi);
 	prob_vision(view);

	cout << "moved particles " << endl << X << endl;
	dgematrix Y = X;
	//draw samples from multi nomial dist.
	for(int i = 0; i<X.m ;i++){
		vec_set(Y,i,rovec_read(X,ProbSelect(w)));		
	}
	
	X=Y;
	//all_set(w,1);

}

void ALMcl::init(int M, const char *filename){
  X.resize(M,4);
  X.zero();
  w.resize(M);
  
  landmarks.read(filename);
  
  forward_noise = MCL_F_N;
  theta_noise = MCL_THETA_N;
  vision_noise = MCL_VISION_N;

  cout << "MCL_F_N " << MCL_F_N  << endl;
  cout << "forward_noise  " << forward_noise  << endl;


}


 
