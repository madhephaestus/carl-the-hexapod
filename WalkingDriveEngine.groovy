
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
	double stepOverHeight=20;
	long stepOverTime=100;
	boolean takingStep = false;
	private Double zLock=-75
	
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
								home[i] = legs.get(i).forwardOffset(new TransformNR());
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
					home[i] = legs.get(i).forwardOffset(new TransformNR());
					//feetLocations[i].setZ(home[i].getZ());
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
						TransformNR lastGood;
						TransformNR stepup = feetLocations[i].copy();
						while(legs.get(i).checkTaskSpaceTransform(feetLocations[i]) &&
							 legs.get(i).checkTaskSpaceTransform(stepup) &&
							 j<1000){
							lastGood=feetLocations[i].copy();
							TransformNR unitvVect = newPose.times(feetLocations[i])
							xunit = (unitvVect.getX()-feetLocations[i].getX() )/10;
							yunit = (unitvVect.getY() -feetLocations[i].getY())/10;
							//increment by the xy unit vectors
							feetLocations[i].translateX(xunit);
							feetLocations[i].translateY(yunit);
							j++;
							stepup = feetLocations[i].copy();
							stepup.setZ(stepOverHeight + zLock );
						}
						println "furthest availible "+feetLocations[i].getX()
						//step back one unit vector to get to acheivable location
						feetLocations[i]=lastGood;
						
						DHParameterKinematics leg = legs.get(i);
						TransformNR up = feetLocations[i].copy();
						up.setZ(stepOverHeight + zLock )
						TransformNR down = feetLocations[i].copy();
						resettingindex=i;
						resetting=true;
//						new Thread(){
//							public void run(){
								try {
									// lift leg above home
									println "lift leg "+resettingindex
									leg.setDesiredTaskSpaceTransform(up, 0);
								} catch (Exception e) {
									// TODO Auto-generated catch block
									e.printStackTrace();
								}
									ThreadUtil.wait((int)stepOverTime);
								try {
									//step to new target
									println "step leg "+resettingindex
									leg.setDesiredTaskSpaceTransform(down, 0);
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
				resetStepTimer();
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

IDriveEngine engine =  new IDriveEngine (){
	double stepOverHeight=20;
	boolean takingStep = false;
	private Double zLock=-75;
	TransformNR previousGLobalState;
	TransformNR target;
	RotationNR rot;
	@Override
	public void DriveArc(MobileBase source, TransformNR newPose, double seconds) {
		
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
					home[i] = legs.get(i).calcHome();
					//feetLocations[i].setZ(home[i].getZ());
				}
				//zLock =zLock+newPose.getZ();
				previousGLobalState = source.getFiducialToGlobalTransform().copy();
				target= newPose.copy();
				//Apply transform to each dimention of current pose
				double el = newPose.getRotation().getRotationElevation() ;
				double ti = newPose.getRotation().getRotationTilt() ;
				TransformNR global= source.getFiducialToGlobalTransform().times(newPose);

//				println "Target rotation = "+Math.toDegrees(newPose.getRotation().getRotationAzimuth())
//				println "Current rotation = "+Math.toDegrees(previousGLobalState.getRotation().getRotationAzimuth())
////				RotationNR neRot = new RotationNR(	Math.toDegrees(
////														previousGLobalState.getRotation().getRotationAzimuth()-
////														newPose.getRotation().getRotationAzimuth()
////														),
////													Math.toDegrees(ti),
////													Math.toDegrees(el)
////													);
////
////				global.setRotation(neRot );
//				println "FInal rotation = "+Math.toDegrees(global.getRotation().getRotationAzimuth())
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
					footx = startLocation.getX() - feetLocations[i].getX() ;
					footy = startLocation.getY() - feetLocations[i].getY() ;
					if(!legs.get(i).checkTaskSpaceTransform(feetLocations[i])){
						
						feetLocations[i].setX(home[i].getX()-footx);
						feetLocations[i].setY(home[i].getY()-footy);
						int j=0;
						while(legs.get(i).checkTaskSpaceTransform(feetLocations[i]) && j<20){
							//increment by the xy unit vectors
							feetLocations[i].translateX(footx/2);
							feetLocations[i].translateY(footy/2);
							j++;
							
						}
						//step back one unit vector
						feetLocations[i].translateX(-footx);
						feetLocations[i].translateY(-footy);
						
						//perform the step over
						home[i].setZ(stepOverHeight+zLock+newPose.getZ());
						//println "Leg "+i+" setep over to x="+feetLocations[i].getX()+" y="+feetLocations[i].getY()
						try {
							double time = 0.2
							// lift leg above home
							legs.get(i).setDesiredTaskSpaceTransform(home[i], 0);
							ThreadUtil.wait(75);
							//step to new target
							legs.get(i).setDesiredTaskSpaceTransform(feetLocations[i], 0);
							ThreadUtil.wait(75);
							//set new target for the coordinated motion step at the end
							feetLocations[i].translateX(newPose.getX());
							feetLocations[i].translateY(newPose.getY());
						} catch (Exception e) {
							// TODO Auto-generated catch block
							//e.printStackTrace();
						}
						
					}
		
				}
				
				//all legs have a valid target set, perform coordinated motion
				for(int i=0;i<numlegs;i++){
					feetLocations[i].setZ(zLock.doubleValue()+newPose.getZ());
					try {
						legs.get(i).setDesiredTaskSpaceTransform(feetLocations[i], seconds);
					} catch (Exception e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
				//source.setGlobalToFiducialTransform(previousGLobalState);
				//interopolate(1,5,seconds/4,source);
				
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

return engine;
