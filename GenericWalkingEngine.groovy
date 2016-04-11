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
import com.neuronrobotics.sdk.common.Log;
if(args==null){
	double stepOverHeight=5;
	long stepOverTime=80;
	Double zLock=-70;
	Closure calcHome = { DHParameterKinematics leg -> 
			TransformNR h=leg.calcHome() 
	
			TransformNR tr = leg.forwardOffset(new TransformNR())
			tr.setZ(zLock)
			
			return h;
	
	}
	boolean usePhysicsToMove = true;

	args= [stepOverHeight,stepOverTime,zLock,calcHome,usePhysicsToMove]
}

return new com.neuronrobotics.sdk.addons.kinematics.IDriveEngine (){
	boolean resetting=false;
	double stepOverHeight=(double)args.get(0);
	long stepOverTime=(long)args.get(1);
	private Double zLock=(Double)args.get(2);
	Closure calcHome =(Closure)args.get(3);
	boolean usePhysics=(args.size()>4?((boolean)args.get(4)):false);

	TransformNR [] home=null;
	
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
							//println "FIRING reset from reset thread"
							resetting=true;
							long tmp= reset;
							if(home==null){
								home = new TransformNR[numlegs];
								for(int i=0;i<numlegs;i++){
									//home[i] = legs.get(i).forwardOffset(new TransformNR());
									home[i] =calcHome(legs.get(i))
								}
							}
							for(int i=0;i<numlegs;i++){
								TransformNR up = home[i].copy()
								up.setZ(stepOverHeight + zLock )
								TransformNR down = home[i].copy()
								down.setZ( zLock )
								try {
									// lift leg above home
									//println "lift leg "+i
									legs.get(i).setDesiredTaskSpaceTransform(up, 0);
								} catch (Exception e) {
									//println "Failed to reach "+up
									e.printStackTrace();
								}
									ThreadUtil.wait((int)stepOverTime);
								try {
									//step to new target
									//println "step leg "+i
									
									legs.get(i).setDesiredTaskSpaceTransform(down, 0);
									//set new target for the coordinated motion step at the end
								} catch (Exception e) {
									//println "Failed to reach "+down
									e.printStackTrace();
								}
							}
							resettingindex=numlegs;
							resetting=false;
							//println "Legs all reset legs"
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
	
		try{
				int numlegs = source.getLegs().size();
				TransformNR [] feetLocations = new TransformNR[numlegs];
				TransformNR [] newFeetLocations = new TransformNR[numlegs];
				ArrayList<DHParameterKinematics> legs = source.getLegs();
				
				if(home==null){
					Log.enableSystemPrint(true)
					home = new TransformNR[numlegs];
					for(int i=0;i<numlegs;i++){
						//home[i] = legs.get(i).forwardOffset(new TransformNR());
						home[i] =calcHome(legs.get(i))
						//println "Home for link "+i+" is "+home[i]
					}
				}
				
				// Load in the locations of the tips of each of the feet.
				for(int i=0;i<numlegs;i++){
					//get the orientation of the base and invert it
					TransformNR inverseRot =new TransformNR(0,0,0,source.getFiducialToGlobalTransform().getRotation()).inverse()
					//TransformNR inverseRot =new TransformNR()
														
					//transform the feet by the inverse orientation
					TransformNR rotPose=inverseRot.times(legs.get(i).getCurrentPoseTarget());
					//invert the target pose
					TransformNR rotPoseinv = newPose.inverse();
					//apply the inverted target
					TransformNR newTar = rotPoseinv.times(rotPose);
					
					//un-do the orientation inversion to get final location
					TransformNR incr =inverseRot.inverse().times(newTar);
					feetLocations[i]=incr
					
					if(zLock==null){
						//sets a standard plane at the z location of the first leg.
						zLock=feetLocations[i].getZ();
						println "ZLock level set to "+zLock
					}
					home[i] =calcHome(legs.get(i))
					feetLocations[i].setZ(home[i].getZ());

				}
				//zLock =zLock+newPose.getZ();
				previousGLobalState = source.getFiducialToGlobalTransform().copy();
				//newPose.setY(0);
				target= newPose.copy();
				//Apply transform to each dimention of current pose
				
				double el = newPose.getRotation().getRotationElevation() ;
				double ti = newPose.getRotation().getRotationTilt() ;
				TransformNR global= source.getFiducialToGlobalTransform().times(newPose);
				// New target calculated appliaed to global offset
				
				//
				//Set it back to where it was to use the interpolator for global move at the end


				for(int i=0;i<numlegs;i++){
					double footx,footy;
					newFeetLocations[i]=legs.get(i).getCurrentPoseTarget();
					// start by storing where the feet are
					
					if(!legs.get(i).checkTaskSpaceTransform(feetLocations[i])){
						//perform the step over
						home[i] =calcHome(legs.get(i))
						//println "Leg "+i+" setep over to x="+feetLocations[i].getX()+" y="+feetLocations[i].getY()
						if(resetting)
						//System.out.println("\r\nWaiting for "+resettingindex+"  reset to finish...")
						while(resetting && resettingindex==i && source.isAvailable()){
							
							ThreadUtil.wait(10);
						}
						//println i+" foot reset needed "+feetLocations[i].getX()+" y:"+feetLocations[i].getY()
						feetLocations[i].setZ(zLock);
						feetLocations[i].setX(home[i].getX());
						feetLocations[i].setY(home[i].getY());
						int j=0;
						//println i+" Step over location"+feetLocations[i].getX()+" y:"+feetLocations[i].getY()
						
						double xunit;
						double yunit ;
						TransformNR lastGood= feetLocations[i].copy();
						TransformNR stepup = feetLocations[i].copy();
						TransformNR stepUnit = feetLocations[i].copy();
						stepup.setZ(stepOverHeight + zLock );
						while(legs.get(i).checkTaskSpaceTransform(feetLocations[i]) &&
							 legs.get(i).checkTaskSpaceTransform(stepup) &&
							 legs.get(i).checkTaskSpaceTransform(stepUnit) &&
							 j<10000){
							feetLocations[i].setZ(zLock );
							stepUnit=lastGood;
							lastGood=feetLocations[i].copy();
							//get the orientation of the base and invert it
							TransformNR inverseRot =new TransformNR(0,0,0,source.getFiducialToGlobalTransform().getRotation()).inverse()
							
							
							//transform the feet by the inverse orientation
							TransformNR rotPose=inverseRot.times(feetLocations[i]);
							//invert the target pose
							TransformNR rotPoseinv = newPose.inverse();
							//apply the inverted target, then un-do the orientation inversion to get final location
							TransformNR incr =inverseRot.inverse().times(rotPoseinv.times(rotPose));
							//now calculate a a unit vector increment
							double xinc=(feetLocations[i].getX()-incr.getX())/1;
							double yinc=(feetLocations[i].getY()-incr.getY())/1;
							//apply the increment to the feet
							feetLocations[i].translateX(xinc);
							feetLocations[i].translateY(yinc);
							//feetLocations[i].translateX(-newPose.getX());
							//feetLocations[i].translateY(-newPose.getX());
							j++;
							stepup = lastGood.copy();
							stepup.setZ(stepOverHeight + zLock );
						}
						//println i+" furthest availible x:"+lastGood.getX()+" y:"+lastGood.getY()
						//step back one unit vector to get to acheivable location
						feetLocations[i]=stepUnit;
						stepup = stepUnit.copy();
						lastGood= stepUnit.copy();
						lastGood.setZ(zLock );
						stepup.setZ(stepOverHeight + zLock );
						
						//println i+" new step y:"+feetLocations[i].getX()+" y:"+feetLocations[i].getY()
						DHParameterKinematics leg = legs.get(i);
						
						resetting=true;
//						new Thread(){
//							public void run(){
								resettingindex=i;
								try {
									// lift leg above home
									//println "lift leg "+resettingindex
									leg.setDesiredTaskSpaceTransform(stepup, 0);
								} catch (Exception e) {
									//println "Failed to reach "+stepup
									e.printStackTrace();
								}
									ThreadUtil.wait((int)stepOverTime);
								try {
									//step to new target
									//println "step leg "+resettingindex
									leg.setDesiredTaskSpaceTransform(lastGood, 0);
									//set new target for the coordinated motion step at the end
								} catch (Exception e) {
									//println "Failed to reach "+lastGood
									e.printStackTrace();
								}
								ThreadUtil.wait((int)stepOverTime/2);
								
								//System.out.println(" Reset "+resettingindex+" Done!\r\n")
								resetting=false;
								resettingindex=numlegs;
//							}
//						}.start();
						
					}
					
					resetStepTimer();
				}
				
				for(int i=0;i<numlegs;i++){
					if(!legs.get(i).checkTaskSpaceTransform(feetLocations[i])){
						throw new RuntimeException(i + " leg foot locatrion is not acheivable "+newPose);
					}
					
				}
				
				
				//all legs have a valid target set, perform coordinated motion
				for(int i=0;i<numlegs;i++){
					legs.get(i).setDesiredTaskSpaceTransform(feetLocations[i], seconds);
		
				}
				if(!usePhysics)
					source.setGlobalToFiducialTransform(global);
				
				// while(resetting && source.isAvailable()){
				// 	//System.out.println("Waiting...")
				// 	ThreadUtil.wait(100);
				// }
				
		}catch (Exception ex){
			ex.printStackTrace();
			println "This step is not possible, undoing "
			// New target calculated appliaed to global offset
			source.setGlobalToFiducialTransform(previousGLobalState);
			//Set it back to where it was to use the interpolator for global move at the end
			return;
			
		}
		
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
