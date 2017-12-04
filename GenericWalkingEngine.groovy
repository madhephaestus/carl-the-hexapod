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
import Jama.Matrix;

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
	
	Thread stepResetter=null;
	
	public void resetStepTimer(){
		reset = System.currentTimeMillis();
	}
	@Override
	public void DriveArc(MobileBase source, TransformNR newPose, double seconds) {
		DriveArcLocal( source,  newPose,  seconds,true);
	}
	/**
	 * Calc Inverse kinematics of a limb .
	 *
	 * @param jointSpaceVect the joint space vect
	 * @return the transform nr
	 */
	public double[] calcForward(DHParameterKinematics leg ,TransformNR transformTarget){
		return leg.inverseKinematics(leg.inverseOffset(transformTarget));
	}
	
	public void DriveArcLocal(MobileBase source, TransformNR newPose, double seconds, boolean retry) {
		TransformNR incomingTarget=newPose.copy()
		newPose = newPose.inverse()
		if(stepResetter==null){
			stepResetter = new Thread(){
				public void run(){
					//println "Starting step reset thread"
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
								ThreadUtil.wait((int)stepOverTime);
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

				//println "Loading feet target locations for "+legs.size()
				// Load in the locations of the tips of each of the feet.
				for(int i=0;i<legs.size();i++){
					//println "Loading Leg "+legs.get(i).getScriptingName()
					TransformNR global= source.getFiducialToGlobalTransform();
					if(global==null){
						global=new TransformNR()
						source.setGlobalToFiducialTransform(global)
					}
					TransformNR footStarting = legs.get(i).getCurrentTaskSpaceTransform();

					if(global==null)
						global=new TransformNR()
					double[] joints = legs.get(i).getCurrentJointSpaceVector()	
					TransformNR armOffset = legs.get(i).forwardKinematics(joints)	
					global=global.times(newPose);// new global pose
					Matrix btt =  legs.get(i).getRobotToFiducialTransform().getMatrixTransform();
					Matrix ftb = global.getMatrixTransform();// our new target
					Matrix current = armOffset.getMatrixTransform();
					Matrix mForward = ftb.times(btt).times(current);
					TransformNR inc =new TransformNR( mForward);
					feetLocations[i]=inc
					
					if(zLock==null){
						//sets a standard plane at the z location of the first leg.
						zLock=feetLocations[i].getZ();
						//println "ZLock level set to "+zLock
					}
					home[i] =calcHome(legs.get(i))
					feetLocations[i].setZ(home[i].getZ());


				}
				boolean resetDetect=false;
				for(int i=0;i<numlegs;i++){
					double footx,footy;
					newFeetLocations[i]=legs.get(i).getCurrentPoseTarget();
					// start by storing where the feet are
					
					if(!legs.get(i).checkTaskSpaceTransform(feetLocations[i])){
						//perform the step over
						home[i] =calcHome(legs.get(i))
						//println "Leg "+i+" setep over to x="+feetLocations[i].getX()+" y="+feetLocations[i].getY()

						//println i+" foot reset needed "+feetLocations[i].getX()+" y:"+feetLocations[i].getY()
						feetLocations[i].setZ(zLock);
						//Force the search for a new foothold to start at the home point
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
							TransformNR g= source.getFiducialToGlobalTransform();
							if(g==null)
								g=new TransformNR()
							g=g.times(newPose);// new global pose
							double[] joints = legs.get(i).inverseKinematics(legs.get(i).inverseOffset(feetLocations[i]))	
							TransformNR armOffset = legs.get(i).forwardKinematics(joints)
							Matrix btt = legs.get(i).getRobotToFiducialTransform().getMatrixTransform();
							Matrix ftb = g.getMatrixTransform();// our new target
							Matrix current = armOffset.getMatrixTransform();
							Matrix mForward = ftb.times(btt).times(current);
							TransformNR incr =new TransformNR( mForward);
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
								resettingindex=i
								int stepIncrement = 20
								// lift leg above home
								stepup = calcHome(leg)
								stepup.setZ(stepOverHeight + zLock )
								
								try {
									
									leg.setDesiredTaskSpaceTransform(stepup, stepOverTime/2000.0);
								} catch (Exception e) {
									//println "Failed to reach "+stepup
									e.printStackTrace();
								}
								ThreadUtil.wait((int)(stepOverTime/2));
								
								double[] joints = calcForward( leg ,lastGood)
								for(int x=0;x<joints.length;x++){
									double ms=(stepOverTime/2)/joints.length
									try {
										//step to new target
										//println "step leg "+resettingindex
										//leg.setDesiredTaskSpaceTransform(lastGood, stepOverTime/4000.0);
										//set new target for the coordinated motion step at the end
										leg.setDesiredJointAxisValue(x,joints[x],ms/1000.0)
									} catch (Exception e) {
										//println "Failed to reach "+lastGood
										e.printStackTrace();
									}
									ThreadUtil.wait((int)ms);
								}
								
								//System.out.println(" Reset "+resettingindex+" Done!\r\n")
								resetting=false;
								resettingindex=numlegs;
								resetDetect=true;
//							}
//						}.start();
						
					}
					
					resetStepTimer();
				}
				if(resetDetect && retry){
					 DriveArcLocal( source,  incomingTarget,  seconds,false) 
					 return;
				}
					
				for(int i=0;i<numlegs;i++){
					if(!legs.get(i).checkTaskSpaceTransform(feetLocations[i])){
						throw new RuntimeException(i + " leg foot location is not acheivable "+newPose);
					}
					
				}
				
				
				//all legs have a valid target set, perform coordinated motion
				for(int i=0;i<numlegs;i++){
					legs.get(i).setDesiredTaskSpaceTransform(feetLocations[i], seconds*1.5);
		
				}
				if(!usePhysics)
					source.setGlobalToFiducialTransform(global);
				
				// while(resetting && source.isAvailable()){
				// 	//System.out.println("Waiting...")
				// 	ThreadUtil.wait(100);
				// }
				
		}catch (Exception ex){
			ex.printStackTrace(System.out);
			println "This step is not possible, undoing "+newPose
			
			//Set it back to where it was to use the interpolator for global move at the end
			throw ex
			
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
