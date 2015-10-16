
import java.time.Duration;
import java.util.ArrayList;

import javafx.application.Platform;

import org.reactfx.util.FxTimer;

import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics;
import com.neuronrobotics.sdk.addons.kinematics.MobileBase;
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import com.neuronrobotics.sdk.util.ThreadUtil;
import com.neuronrobotics.sdk.addons.kinematics.IDriveEngine;


	
return new com.neuronrobotics.sdk.addons.kinematics.IDriveEngine (){
	boolean resetting=false;
	double stepOverHeight=5;
	long stepOverTime=75;
	boolean takingStep = false;
	private Double zLock=-70;
	
	TransformNR previousGLobalState;
	TransformNR target;
	RotationNR rot;
	int resettingindex=0;
	private long reset = System.currentTimeMillis();
	MobileBase source;
	
	Thread stepResetter=null;
	
	public void resetStepTimer(){
		reset = System.currentTimeMillis();
	}
	
	@Override
	public void DriveArc(MobileBase source, TransformNR newPose, double seconds) {
		if(stepResetter==null){
			stepResetter = new Thread(){
				public void run(){
					println "Starting step reset thread"
					int numlegs = source.getLegs().size();
					ArrayList<DHParameterKinematics> legs = source.getLegs();
					while(source.isAvailable()){
						ThreadUtil.wait(10);
						if(reset+5000 < System.currentTimeMillis()){
							println "FIRING reset from reset thread"
							resetting=true;
							long tmp= reset;
							TransformNR [] home = new TransformNR[numlegs];
							for(int i=0;i<numlegs;i++){
								//home[i] = legs.get(i).forwardOffset(new TransformNR());
								home[i] =legs.get(i).calcHome();
								TransformNR up = home[i].copy()
								up.setZ(stepOverHeight + zLock )
								TransformNR down = home[i].copy()
								down.setZ( zLock )
								try {
									// lift leg above home
									println "lift leg "+i
									legs.get(i).setDesiredTaskSpaceTransform(up, 0);
								} catch (Exception e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}
									ThreadUtil.wait((int)stepOverTime);
								try {
									//step to new target
									println "step leg "+i
									legs.get(i).setDesiredTaskSpaceTransform(down, 0);
									//set new target for the coordinated motion step at the end
								} catch (Exception e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}
							}
							resetting=false;
							println "Legs all reset legs"
							while(source.isAvailable() && tmp==reset){
								ThreadUtil.wait(10);
							}
						}
					}
				}
				
				
			};
			stepResetter.start();
		}
		resetStepTimer();
	
		if(takingStep)
			return;
		takingStep = true;
		try{
				int numlegs = source.getLegs().size();
				TransformNR [] feetLocations = new TransformNR[numlegs];
				TransformNR [] newFeetLocations = new TransformNR[numlegs];
				TransformNR [] home = new TransformNR[numlegs];
				ArrayList<DHParameterKinematics> legs = source.getLegs();
				
				// Load in the locations of the tips of each of the feet.
				for(int i=0;i<numlegs;i++){
					feetLocations[i]=legs.get(i).getCurrentPoseTarget();
					if(zLock==null){
						//sets a standard plane at the z location of the first leg.
						zLock=feetLocations[i].getZ();
						println "ZLock level set to "+zLock
					}
					// this sets up the location under the legs shoulder as the base
					//home[i] = legs.get(i).forwardOffset(new TransformNR());
					home[i] =legs.get(i).calcHome();
					feetLocations[i].setZ(home[i].getZ());
				}
				//zLock =zLock+newPose.getZ();
				previousGLobalState = source.getFiducialToGlobalTransform().copy();
				newPose.setY(0);
				target= newPose.copy();
				//Apply transform to each dimention of current pose
				
				double el = newPose.getRotation().getRotationElevation() ;
				double ti = newPose.getRotation().getRotationTilt() ;
				TransformNR global= source.getFiducialToGlobalTransform().times(newPose);
				// New target calculated appliaed to global offset
				source.setGlobalToFiducialTransform(global);
				for(int i=0;i<numlegs;i++){
					//legs.get(i).setGlobalToFiducialTransform(global);
					newFeetLocations[i]=legs.get(i).getCurrentPoseTarget();
					
				}
				//Set it back to where it was to use the interpolator for global move at the end


				for(int i=0;i<numlegs;i++){
					double footx,footy;
					TransformNR startLocation = newFeetLocations[i];
					// start by storing where the feet are
				
					if(!legs.get(i).checkTaskSpaceTransform(feetLocations[i]) && (!(resetting && resettingindex==i) ) ){
						//perform the step over
						//home[i].setZ(stepOverHeight+zLock+newPose.getZ());
						//println "Leg "+i+" setep over to x="+feetLocations[i].getX()+" y="+feetLocations[i].getY()
						if(resetting)
						System.out.println("\r\nWaiting for "+resettingindex+"  reset to finish...")
						while(resetting && source.isAvailable()){
							
							ThreadUtil.wait(100);
						}
						
						println i+" foot reset needed "+feetLocations[i].getX()
						feetLocations[i].setZ(zLock+newPose.getZ());
						feetLocations[i].setX(home[i].getX());
						feetLocations[i].setY(home[i].getY());
						int j=0;
						println i+" Step over location"+feetLocations[i].getX()
						
						double xunit;
						double yunit ;
						TransformNR lastGood= feetLocations[i].copy();
						TransformNR stepup = feetLocations[i].copy();
						stepup.setZ(stepOverHeight + zLock );
						while(legs.get(i).checkTaskSpaceTransform(feetLocations[i]) &&
							 legs.get(i).checkTaskSpaceTransform(stepup) && 
							 j<1000){
							feetLocations[i].setZ(zLock );
							lastGood=feetLocations[i].copy();
							
							TransformNR incr =newPose.inverse().times(feetLocations[i]);
//							println i+" NewPose x:"+newPose.getX()+" y:"+newPose.getY()
//							println i+" NewPoseInverse x:"+newPose.inverse().getX()+" y:"+newPose.inverse().getY()
//							println i+" inverse x:"+incr.getX()+" y:"+incr.getY()
//							println i+" from    x:"+feetLocations[i].getX()+" y:"+feetLocations[i].getY()
							double xinc=(feetLocations[i].getX()-incr.getX())/10;
							double yinc=(feetLocations[i].getY()-incr.getY())/10;
							//println i+" increment x:"+xinc+" y:"+yinc
							feetLocations[i].translateX(xinc);
							feetLocations[i].translateY(yinc);
							j++;
//							stepup = feetLocations[i].copy();
//							stepup.setZ(stepOverHeight + zLock );
						}
//						stepup = lastGood.copy();
//						stepup.setZ(stepOverHeight + zLock );
						feetLocations[i]=lastGood.times(newPose);
						
						println i+" furthest availible "+feetLocations[i].getX()
						//step back one unit vector to get to acheivable location
						//feetLocations[i]=lastGood.times(newPose);
						
						DHParameterKinematics leg = legs.get(i);
						resettingindex=i;
						resetting=true;
//						new Thread(){
//							public void run(){
								try {
									// lift leg above home
									println "lift leg "+resettingindex
									leg.setDesiredTaskSpaceTransform(stepup, 0);
								} catch (Exception e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}
									ThreadUtil.wait((int)stepOverTime);
								try {
									//step to new target
									println "step leg "+resettingindex
									leg.setDesiredTaskSpaceTransform(lastGood, 0);
									//set new target for the coordinated motion step at the end
								} catch (Exception e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}
								ThreadUtil.wait((int)stepOverTime/2);
								resetting=false;
								System.out.println(" Reset "+resettingindex+" Done!\r\n")
//							}
//						}.start();
						
					}
					resetStepTimer();
				}
				
				
				//all legs have a valid target set, perform coordinated motion
				for(int i=0;i<numlegs;i++){
					feetLocations[i].setZ(zLock.doubleValue()+newPose.getZ());
					try {
						if(!(resetting && resettingindex==i) )
							legs.get(i).setDesiredTaskSpaceTransform(feetLocations[i], seconds);
					} catch (Exception e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
				
				// while(resetting && source.isAvailable()){
				// 	//System.out.println("Waiting...")
				// 	ThreadUtil.wait(100);
				// }
				
		}catch (Exception ex){
			ex.printStackTrace();
		}
		takingStep = false;
		
	}

	@Override
	public void DriveVelocityStraight(MobileBase source, double cmPerSecond) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void DriveVelocityArc(MobileBase source, double degreesPerSecond,
			double cmRadius) {
		// TODO Auto-generated method stub
		
	}



}
