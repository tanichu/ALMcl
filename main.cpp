#include"ALMcl.h"

#define T 30

int main(){

  scatter();

  cout << "Gaussian " << gauss_rand() << endl;

  ALMcl core;
  core.init(10,"landmarks.dat");
  
  drovector pos_(4);
  pos_.zero();
  
  cout << "landmark 0 " <<  core.vision_simulator(pos_,rovec_read(core.landmarks,0)) <<endl;
  cout << "landmark 1 " <<  core.vision_simulator(pos_,rovec_read(core.landmarks,1)) <<endl;
  cout << "landmark 2 " <<  core.vision_simulator(pos_,rovec_read(core.landmarks,2)) <<endl;
  cout << "landmark 3 " <<  core.vision_simulator(pos_,rovec_read(core.landmarks,3)) <<endl;


  //ここまでdebug

  	ALMcl engine, sim, pure;//engine = localization engine, sim = real world simulator
	//pure is for simulation excluding noise term
	//	pure.forward_noise = 0;
  	//pure.theta_noise = 0;

	drovector pos(4),purepos(4);
	pos.zero();purepos.zero();

	engine.init(100,"landmarks.dat");

	dgematrix senario;
	senario.read("senario.txt");

	dgematrix logPos(T,4),logPure(T,4),logEst(T,4);
	sim.forward_noise = 1.0;
	sim.theta_noise = 0.1;
	sim.vision_noise = 1.0;

	pure.forward_noise = 0;
	pure.theta_noise = 0;
	pure.vision_noise = 1.0;

	engine.forward_noise = 1.0;
	engine.theta_noise = 0.1;
	engine.vision_noise = 1.0;

       


	cout <<"sim fn " <<  sim.forward_noise << endl;
	cout <<"sim tn " << sim.theta_noise << endl; 

	cout <<"pure fn " <<  pure.forward_noise << endl;
	cout <<"pure tn " << pure.theta_noise << endl; 


	


       
	for(int t=0;t<T;t++){
	  //command and move
	  pos = sim.move(pos,senario(t,0),senario(t,1),1);
	  cout << "pos " << pos << endl; 
	  //pure movement 
	  purepos = pure.move(purepos,senario(t,0),senario(t,1),1);
	  cout << "purepos " << purepos << endl;
	  //	getchar();
	  
	  //obtaining view
	  drovector vision = engine.vision_simulator(pos,rovec_read(engine.landmarks,t%4)); //each time t%4 landmark is observed
	  cout << "landmark " << vision << endl;
	  dgematrix view(0,2);// vec_set(view,0,vision);
	  //updating localization engine
	  engine.update(senario(t,0),senario(t,1),1,view);
	  cout << "updated particles "<< endl <<  engine.X << endl;
	  cout << "updated weight " << endl << engine.w << endl;
	  getchar();//estimated self position
		engine.get_pos();
		
		//loginmg
		vec_set(logPos,t,pos);
		vec_set(logPure,t,purepos);
		vec_set(logEst,t,engine.get_pos());
		
	}
	
  logPos.write("logPos.txt");
  logPure.write("logPure.txt");
  logEst.write("logEst.txt");
  
  return 0;

}


