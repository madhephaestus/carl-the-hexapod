import eu.mihosoft.vrl.v3d.Extrude;

import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Paths;
import java.util.ArrayList;
import javafx.scene.paint.Color;
import javax.vecmath.Matrix4d;
import Jama.Matrix;
import com.neuronrobotics.bowlerstudio.BowlerStudioController;
import com.neuronrobotics.bowlerstudio.creature.CreatureLab;
import com.neuronrobotics.bowlerstudio.creature.ICadGenerator;
import com.neuronrobotics.imageprovider.NativeResource;
import com.neuronrobotics.nrconsole.plugin.BowlerCam.RGBSlider.ColorBox;
import com.neuronrobotics.sdk.addons.kinematics.DHLink;
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics;
import com.neuronrobotics.sdk.addons.kinematics.MobileBase;
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import com.neuronrobotics.sdk.common.Log;
import com.neuronrobotics.bowlerstudio.vitamins.IVitamin;
import com.neuronrobotics.bowlerstudio.vitamins.MicroServo;
import com.neuronrobotics.bowlerstudio.vitamins.Vitamins;

import eu.mihosoft.vrl.v3d.CSG;
import eu.mihosoft.vrl.v3d.Cube;
import eu.mihosoft.vrl.v3d.FileUtil;
import eu.mihosoft.vrl.v3d.RoundedCube
import eu.mihosoft.vrl.v3d.STL;
import eu.mihosoft.vrl.v3d.Sphere;
import eu.mihosoft.vrl.v3d.Transform;
import eu.mihosoft.vrl.v3d.Cylinder;
import eu.mihosoft.vrl.v3d.Vector3d;
import javafx.scene.paint.Color;
import com.neuronrobotics.bowlerstudio.physics.*;

return new ICadGenerator(){
	//CSG servoReference= new MicroServo().toCSG();
	CSG servoReference=   (CSG)(ScriptingEngine.inlineGistScriptRun("3f9fef17b23acfadf3f7", "servo.groovy" , null))
	.transformed(new Transform().rotZ(-90))
	CSG dyioReference=   (CSG)(ScriptingEngine.inlineGistScriptRun("fb4cf429372deeb36f52", "dyioCad.groovy" , null))
//	.transformed(new Transform().translateZ(12.0))
//	.transformed(new Transform().translateX(5.4));
	
	//CSG horn=  STL.file(NativeResource.inJarLoad(IVitamin.class,"smallmotorhorn.stl").toPath())
	CSG horn = new Cube(6,5,12).toCSG();
	private double attachmentRodWidth=10;
	private double attachmentBaseWidth=15;
	private double printerTollerence =0.5;
	private double mountScrewKeepawaySize= 7.5;
	private double mountScrewHoleKeepawaySize= 4.1;
	private double mountScrewSeperationDistance=attachmentRodWidth/2+mountScrewHoleKeepawaySize/2+0.5;
	double cylandarRadius = 13.5;
	private double bearingPinRadius=3;
	
	private CSG toZMin(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateZ(-target.getBounds().getMin().z));
	}
	private CSG toZMax(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateZ(-target.getBounds().getMax().z));
	}
	private CSG toXMin(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateX(-target.getBounds().getMin().x));
	}
	private CSG toXMax(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateX(-target.getBounds().getMax().x));
	}
	private CSG toYMin(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateY(-target.getBounds().getMin().y));
	}
	private CSG toYMax(CSG incoming,CSG target){
		return incoming.transformed(new Transform().translateY(-target.getBounds().getMax().y));
	}
	private CSG toZMin(CSG incoming){
		return toZMin(incoming,incoming);
	}
	private CSG toZMax(CSG incoming){
		return toZMax(incoming,incoming);
	}
	private CSG toXMin(CSG incoming){
		return toXMin(incoming,incoming);
	}
	private CSG toXMax(CSG incoming){
		return toXMax(incoming,incoming);
	}
	private CSG toYMin(CSG incoming){
		return toYMin(incoming,incoming);
	}
	private CSG toYMax(CSG incoming){
		return toYMax(incoming,incoming);
	}
	private CSG getAppendageMount(){
		double cylindarKeepawayHeight = 80;
		CSG attachmentbase =new Cylinder(// The first part is the hole to put the screw in
					40,
					cylindarKeepawayHeight,
					 (int)20).toCSG()
					 .toXMin()
			.transformed(new Transform().translateX(-14*1.2))
			.transformed(new Transform().translateZ(-cylindarKeepawayHeight/2))
		
		return attachmentbase;
	}
	
	
	private CSG getMountScrewKeepaway(){
		CSG screw = new Cylinder(// The first part is the hole to put the screw in
					mountScrewKeepawaySize/2,
					200,
					 (int)20).toCSG()
					 screw =screw.union(new Cylinder(// This the the tapper section in the fasening part
						 mountScrewHoleKeepawaySize/2,
						 mountScrewKeepawaySize/2,
						  3,
						  (int)10).toCSG().toZMax()
						  ).toZMin()
		screw =screw.union(new Cylinder(// This the the hole in the fasening part
					mountScrewHoleKeepawaySize/2,
					 attachmentBaseWidth/2+1,
					 (int)10).toCSG().toZMax()
					 ).toZMin()
		screw =screw.union(new Cylinder(// This the the hole in the threaded part
			2.6/2,
			 30,
			 (int)10).toCSG().toZMax()
			 )
			 .movex(mountScrewSeperationDistance)
		screw =screw.union(screw	 .movex(-mountScrewSeperationDistance*2))
		
		return screw;
	}
	
	private CSG getAttachment(){
		CSG attachmentbase = new RoundedCube(attachmentBaseWidth,attachmentBaseWidth,4)
							.cornerRadius(attachmentBaseWidth/10)
							.noCenter()
							.toCSG()
							.movex(-attachmentBaseWidth/2)
							.movey(-(attachmentBaseWidth/2))
		CSG cutOffBottomOfAttachment 	=new Cube(	(attachmentBaseWidth-attachmentRodWidth)/2,
									attachmentBaseWidth,
									10)
									.toCSG()
									
									.movex(-attachmentBaseWidth/2+(attachmentBaseWidth-attachmentRodWidth)/4)
									.rotz(-90)
		attachmentbase=attachmentbase.difference(cutOffBottomOfAttachment)
		
		CSG post = toZMin(new Cube(	attachmentRodWidth,
									attachmentRodWidth,
									Math.abs(servoReference.getBounds().getMax().x)+4
									+attachmentRodWidth/2)
									
									.toCSG());
		attachmentbase = toZMax(attachmentbase.union(post))
		.transformed(new Transform().translateZ( attachmentRodWidth/2));
		
		CSG hornAttach =toZMin(toYMin(	toYMax( toZMax(horn).transformed(new Transform().translateZ( 4))) , 
										post),
									post
									);
		attachmentbase =attachmentbase.difference(hornAttach);
		
		
		double pinMax = bearingPinRadius;
		double pinMin =bearingPinRadius;
		CSG bearingPin =toYMax( new Cylinder(pinMax,pinMin, (int)5 ,(int)50).toCSG()
			.transformed(new Transform().rotX(90)),
										post);
		attachmentbase =attachmentbase.difference(bearingPin);
		return attachmentbase.transformed(new Transform().rot(-90, -90, 0));

	}
	
	private CSG getFoot(){
		CSG attach = getAttachment();
		CSG foot = new Sphere(attachmentRodWidth).toCSG();
		return  toXMax(attach.union(foot));
	}

	private CSG reverseDHValues(CSG incoming,DHLink dh ){
		println "Reversing "+dh
		return incoming
			.rotx(Math.toDegrees(-dh.getAlpha()))
			.movex(dh.getR()/2)		
	}
	
	private CSG moveDHValues(CSG incoming,DHLink dh ){
		return incoming.transformed(new Transform().translateZ(-dh.getD()))
		.transformed(new Transform().rotZ(-Math.toDegrees(dh.getTheta())))
		.transformed(new Transform().rotZ((90+Math.toDegrees(dh.getTheta()))))
		.transformed(new Transform().translateX(-dh.getR()))
		.transformed(new Transform().rotX(Math.toDegrees(dh.getAlpha())));
		
	}

		/**
	 * Gets the all dh chains.
	 *
	 * @return the all dh chains
	 */
	public ArrayList<DHParameterKinematics> getLimbDHChains(MobileBase base) {
		ArrayList<DHParameterKinematics> copy = new ArrayList<DHParameterKinematics>();
		for(DHParameterKinematics l:base.getLegs()){
			copy.add(l);	
		}
		for(DHParameterKinematics l:base.getAppendages() ){
			copy.add(l);	
		}
		return copy;
	}

	ArrayList<CSG> generateBody(MobileBase base){
		ArrayList<CSG> allCad=new ArrayList<>();
		ArrayList<CSG> cutouts=new ArrayList<>();
		ArrayList<CSG> attach=new ArrayList<>();
		CSG attachUnion=null;
		for(DHParameterKinematics l:getLimbDHChains(base)){
			TransformNR position = l.getRobotToFiducialTransform();
			Transform csgTrans = TransformFactory.nrToCSG(position)
			cutouts.add(getAppendageMount()
				.transformed(csgTrans)
				);
			CSG attachment = getAttachment()
				.transformed(csgTrans)
			attach.add(attachment);
			if(attachUnion==null){
				attachUnion=attachment;
			}else{
				attachUnion = 	attachUnion.union(attachment)
			}
			
		}

		CSG upperBody = attachUnion.hull()
		
		
						
		CSG myDyIO=dyioReference
				.movez(upperBody.getMaxZ()+22.0)
				.movex(	upperBody.getMaxX()-
						Math.abs(upperBody.getMinX())+
						dyioReference.getMaxX()-
						dyioReference.getMinX())
				.movey(	upperBody.getMaxY()-
						Math.abs(upperBody.getMinY())+
						dyioReference.getMaxY()-
						Math.abs(dyioReference.getMinY()))
		upperBody=upperBody
		.union(myDyIO)
		.hull()
		.difference(myDyIO)
		
		upperBody= upperBody.difference(cutouts);
		
		upperBody= upperBody.union(attach);
		
	
					
		upperBody.setColor(Color.CYAN);
		upperBody.setManipulator(base.getRootListener());
		upperBody.setManufactuing(new PrepForManufacturing() {
					public CSG prep(CSG arg0) {
						return arg0.toZMin();
					}
				});
		allCad.add(upperBody)
		
		
		return allCad;
	}

	
	public ArrayList<CSG> generateCad(DHParameterKinematics sourceLimb, int linkIndex){
		//printBed=true;
		ArrayList<DHLink> dhLinks=sourceLimb.getChain().getLinks();
		ArrayList<CSG> csg = new ArrayList<CSG>();

		
		CSG servoKeepaway = toXMin(toZMax(	new Cube(Math.abs(servoReference.getBounds().getMin().x) +
			Math.abs(servoReference.getBounds().getMax().x),
			Math.abs(servoReference.getBounds().getMin().y) +
			Math.abs(servoReference.getBounds().getMax().y),
			Math.abs(servoReference.getBounds().getMax().z)).toCSG(),
		
		)
		)
		servoKeepaway = servoKeepaway
			.movex(-Math.abs(servoReference.getBounds().getMin().x))
			.movez(-5)
			
			dh = dhLinks.get(linkIndex);
			CSG nextAttachment=getAttachment();
			
			
			CSG servo=servoReference.transformed(new Transform().translateZ(-12.5))// allign to the horn
			.union(servoKeepaway)
			.transformed(new Transform().rotX(180))// allign to the horn
			.transformed(new Transform().rotZ(-90))// allign to the horn
			
			;
			servo= servo.makeKeepaway(printerTollerence)
			double totalServoExtention =  Math.abs(servo.getMinY())
			
			boolean addNub=false;
			
			println "Servo Keepaway Radius = "+cylandarRadius
			
			double rOffsetForNextLinkStart=dh.getR()+mountScrewKeepawaySize+0.75;
			double rOffsetForNextLink=rOffsetForNextLinkStart;
			
			if(linkIndex==dhLinks.size()-1){
					 rOffsetForNextLink = rOffsetForNextLink-
					Math.abs(foot.toXMax().getBounds().getMin().x) 
				
			}else{
				rOffsetForNextLink = rOffsetForNextLink-
					Math.abs(nextAttachment.toXMax().getBounds().getMin().x)
				
			}
			//println "Link # "+linkIndex+" offset = "+(rOffsetForNextLink)+" rad "+dh.getR() +" offset = "
			
			double linkThickness = dh.getD();
			if(linkThickness<attachmentBaseWidth/2)
				linkThickness=attachmentBaseWidth/2
			linkThickness +=3;
			
			servo = servo
					.movez(-3.30)
					
			servo= moveDHValues(servo,dh);
			
			double yScrewOffset = 2.5
			CSG upperLink = toZMin(new Cylinder(cylandarRadius,linkThickness,(int)20).toCSG())
			/*
			if(addNub){
				totalServoExtention
				upperLink=upperLink.union(new Cylinder(cylandarRadius,linkThickness,(int)20)
				.toCSG()
				.toZMin()
				.movey(-totalServoExtention+cylandarRadius)
				)
			}
			*/
			
			double screwsCloseY = 20;
			double screwsFarY=rOffsetForNextLink+mountScrewKeepawaySize/2
			double screwsZ=upperLink.getBounds().getMax().z - linkThickness
			
			CSG mountHoleAttachment = new Cylinder((mountScrewKeepawaySize+1)/2, // Radius at the top
			  				(mountScrewKeepawaySize+1)/2, // Radius at the bottom
			  				linkThickness, // Height
			  			         (int)10 //resolution
			  			         ).toCSG()
			  			         .toZMin()
											
			mountHoleAttachment = mountHoleAttachment.movex(mountScrewSeperationDistance)
			mountHoleAttachment =mountHoleAttachment.union(mountHoleAttachment.movex(-2*mountScrewSeperationDistance))
			CSG mountHoleAttachmentGroup = mountHoleAttachment
										.movey(screwsFarY)
										
			CSG upperScrews = getMountScrewKeepaway()
				.movey(screwsFarY)
				.movez(screwsZ)
			if(dh.getR()>80){
				upperScrews =upperScrews.union( getMountScrewKeepaway()
					
					.movey(screwsCloseY)
					)
					mountHoleAttachmentGroup=mountHoleAttachmentGroup
					.union(
						mountHoleAttachment
										.movey(screwsCloseY)
						)
			}
			// adding the radius rod
			CSG rod = toYMin(
								toZMin(
									new Cube( 
										attachmentBaseWidth+3,
										rOffsetForNextLink,
										upperLink.getBounds().getMax().z
										).toCSG()
									)
								)
			CSG clip = toYMin(
				toZMax(
					new Cube(
						attachmentBaseWidth+12,
						9+12,
						attachmentBaseWidth+3
						).toCSG()
					)
				)
				.toYMax()
				.movey(rOffsetForNextLink+8)// allign to the NEXT ATTACHMENT
				.movez(linkThickness)// allign to the NEXT ATTACHMENT
			
			double upperLinkZOffset = Math.abs(servoReference.getBounds().getMax().z-3)
			//Build the upper link
			upperLink=upperLink.union(mountHoleAttachmentGroup,rod)
			.hull()//smooth out the shape
			CSG projection = upperLink
							.scalez(10)
							.intersect(clip)
			
			upperLink=upperLink	.union(projection.toZMax())// add the clip in
			
			upperLink= upperLink.difference(upperScrews);
			upperLink=upperLink.movez(upperLinkZOffset)
			
			upperLink= moveDHValues(upperLink,dh).difference(servo);


				// debugging	
			projection=moveDHValues(	projection,dh)		
			projection.setManipulator(dh.getListener());
			//csg.add(projection);// view the clip
			if(linkIndex== dhLinks.size()-1)
				upperLink= upperLink.difference(foot.makeKeepaway(printerTollerence*2));
			else
				upperLink= upperLink.difference(nextAttachment.makeKeepaway(printerTollerence*2));
			
			double LowerLinkThickness = attachmentRodWidth/2-2
			CSG lowerLink = new Cylinder(
				cylandarRadius,
				 LowerLinkThickness +linkThickness+6,
				 (int)20)
				 .toCSG()
				 .toZMin()
				 .movez(-LowerLinkThickness)
			CSG linkSweepCutout= new Cylinder(
				cylandarRadius+1,
				 LowerLinkThickness +linkThickness+6,
				 (int)20)
				 .toCSG()
				 .toZMin()
				 .movez(-attachmentRodWidth/2)
			CSG pinBlank = new Cylinder( bearingPinRadius-printerTollerence,
										 LowerLinkThickness,
										 (int)20)
							.toCSG()
							.toZMin()
							.movez(-attachmentRodWidth/2)
			linkSweepCutout=	linkSweepCutout.difference(	pinBlank)	
							
			//lowerLink=lowerLink.union(pinBlank)
							
			
			lowerLink=lowerLink.transformed(new Transform().translateZ(-attachmentRodWidth/2))
			CSG lowerClip =
					
					new Cube(
						attachmentBaseWidth+3,
						rOffsetForNextLink-2,
						LowerLinkThickness +linkThickness+6
						).toCSG().toZMin().toYMin()
				
				
				.transformed(new Transform().translateY(9))// allign to the NEXT ATTACHMENT
				
				.transformed(new Transform().translateZ(-attachmentRodWidth/2 -LowerLinkThickness ))
			
			lowerLink=lowerLink.union(
				lowerClip,
				mountHoleAttachmentGroup.movez(lowerLink.getMinZ())
				
				).hull();
			//Remove the divit or the bearing
			lowerLink= lowerLink.difference(
					nextAttachment
						.makeKeepaway((double)-0.2)
						.movez(-0.15),
					upperScrews
					.movez(6),
					linkSweepCutout
					)// allign to the NEXT ATTACHMENT);
			
			lowerLink= moveDHValues(lowerLink,dh);
			//remove the next links connector and the upper link for mating surface
			lowerLink= lowerLink.difference(upperLink,servo);
			if(linkIndex== dhLinks.size()-1)
				lowerLink= lowerLink.difference(foot.makeKeepaway(printerTollerence*2));
			else
				lowerLink= lowerLink.difference(nextAttachment.makeKeepaway(printerTollerence*2));
			
			
			
			upperLink.setManufactuing(new PrepForManufacturing() {
				public CSG prep(CSG arg0) {
					return reverseDHValues(arg0,dhLinks.get(linkIndex))
							.roty(180)
							.toZMin()
							.toXMin()
				}
			});
			
			
			lowerLink.setManufactuing(new PrepForManufacturing() {
				public CSG prep(CSG arg0) {
					return 	reverseDHValues(arg0,dhLinks.get(linkIndex))
							.toZMin()
							.toXMin()
				}
			});
			
			nextAttachment.setManufactuing(new PrepForManufacturing() {
				public CSG prep(CSG arg0) {
					return 	arg0
							//.transformed(new Transform().rotY(90))
							.toZMin()
							.toXMin()
				}
			});
				
			
			
			nextAttachment.setManipulator(dh.getListener());
			nextAttachment.setColor(Color.CHOCOLATE);
			servo.setManipulator(dh.getListener());
			upperLink.setColor(Color.GREEN);
			upperLink.setManipulator(dh.getListener());
			
			
			lowerLink.setColor(Color.PURPLE);
			lowerLink.setManipulator(dh.getListener());
			
			//csg.add(servo);// view the servo
			upperScrews= moveDHValues(upperScrews.movez(upperLinkZOffset),dh)
						
			upperScrews.setManipulator(dh.getListener());
			//csg.add(upperScrews);//view the screws
			
			
			if(linkIndex<dhLinks.size()-1){
				csg.add(nextAttachment);//This is the root that attaches to the base
				BowlerStudioController.addCsg(nextAttachment);
			}
			
			csg.add(upperLink);//This is the root that attaches to the base
			csg.add(lowerLink);//White link forming the lower link
			BowlerStudioController.addCsg(upperLink);
			BowlerStudioController.addCsg(lowerLink);
				
			if(linkIndex==	dhLinks.size()-1){
				CSG foot=getFoot();	
				//test for last link
				foot.setManufactuing(new PrepForManufacturing() {
					public CSG prep(CSG arg0) {
						return 	arg0.transformed(new Transform().rotY(90))
								.toZMin()
								.toXMin()
					}
				});
	
			
				foot.setManipulator(dhLinks.get(dhLinks.size()-1).getListener());
					
				
				foot.setColor(Color.GOLD);
				csg.add(foot);//This is the root that attaches to the base
				BowlerStudioController.addCsg(foot);
			}
		
		return csg;
	}
};
