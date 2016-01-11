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

return new ICadGenerator(){
	//CSG servoReference= new MicroServo().toCSG();
	CSG servoReference=   (CSG)(ScriptingEngine.inlineGistScriptRun("3f9fef17b23acfadf3f7", "servo.groovy" , null))
	.transformed(new Transform().rotZ(-90))
//	.transformed(new Transform().translateZ(12.0))
//	.transformed(new Transform().translateX(5.4));
	
	//CSG horn=  STL.file(NativeResource.inJarLoad(IVitamin.class,"smallmotorhorn.stl").toPath())
	CSG horn = new Cube(6,5,19).toCSG();
	private double attachmentRodWidth=10;
	private double attachmentBaseWidth=15;
	private double printerTollerence =0.5;
	private double mountScrewKeepawaySize= 7.5;
	private double mountScrewHoleKeepawaySize= 4.1;
	private double mountScrewSeperationDistance=attachmentRodWidth/2+mountScrewHoleKeepawaySize/2+0.5;
	double cylandarRadius = 13;
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
	
	private CSG makeKeepaway(CSG incoming){
		
		double x = Math.abs(incoming.getBounds().getMax().x )+ Math.abs(incoming.getBounds().getMin().x)
		double y = Math.abs(incoming.getBounds().getMax().y) + Math.abs(incoming.getBounds().getMin().y)
		
		double z = Math.abs(incoming.getBounds().getMax().z )+ Math.abs(incoming.getBounds().getMin().z)
		
		double xtol=(x+printerTollerence)/x
		double ytol= (y+printerTollerence)/y
		double ztol=(z+printerTollerence)/z
		
		double xPer=-(Math.abs(incoming.getBounds().getMax().x)-Math.abs(incoming.getBounds().getMin().x))/x
		double yPer=-(Math.abs(incoming.getBounds().getMax().y)-Math.abs(incoming.getBounds().getMin().y))/y
		double zPer=-(Math.abs(incoming.getBounds().getMax().z)-Math.abs(incoming.getBounds().getMin().z))/z
		
		//println " Keep away x = "+y+" new = "+ytol
		return 	incoming
				.transformed(new Transform().scale(xtol,
													ytol,
													 ztol ))
				.transformed(new Transform().translateX(printerTollerence * xPer))
				.transformed(new Transform().translateY(printerTollerence*yPer))
				.transformed(new Transform().translateZ(printerTollerence*zPer))
				
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
									Math.abs(servoReference.getBounds().getMax().x)+3+attachmentRodWidth/2)
									
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
	
	Transform convertTransform(TransformNR incoming){
		
	}
	
	private CSG reverseDHValues(CSG incoming,DHLink dh ){
		return incoming
		.transformed(new Transform().rotX(-Math.toDegrees(dh.getAlpha())))
		//.transformed(new Transform().rotZ(Math.toDegrees(dh.getTheta())))
	}
	
	private CSG moveDHValues(CSG incoming,DHLink dh ){
		return incoming.transformed(new Transform().translateZ(-dh.getD()))
		.transformed(new Transform().rotZ(-Math.toDegrees(dh.getTheta())))
		.transformed(new Transform().rotZ((90+Math.toDegrees(dh.getTheta()))))
		.transformed(new Transform().translateX(-dh.getR()))
		.transformed(new Transform().rotX(Math.toDegrees(dh.getAlpha())));
		
	}

	ArrayList<CSG> generateBodyParts(MobileBase base ,boolean printing){
		ArrayList<CSG> allCad=new ArrayList<>();
		ArrayList<Vector3d> points=new ArrayList<>();
		ArrayList<CSG> cutouts=new ArrayList<>();
		ArrayList<CSG> attach=new ArrayList<>();
		int minz= 1000000;
		int maxz=-1000000;
		int minx= 1000000;
		int maxx=-1000000;
		int miny= 1000000;
		int maxy=-1000000;
		CSG attachUnion=null;
		for(DHParameterKinematics l:base.getAllDHChains()){
			TransformNR position = l.getRobotToFiducialTransform();
			RotationNR rot = position.getRotation()
			Matrix vals =position.getMatrixTransform();
			double [] elemenents = [ 
				vals.get(0, 0),
				vals.get(0, 1),
				vals.get(0, 2),
				vals.get(0, 3),
				
				vals.get(1, 0),
				vals.get(1, 1),
				vals.get(1, 2),
				vals.get(1, 3),
				
				vals.get(2, 0),
				vals.get(2, 1),
				vals.get(2, 2),
				vals.get(2, 3),
				
				vals.get(3, 0),
				vals.get(3, 1),
				vals.get(3, 2),
				vals.get(3, 3),
				
				 ] as double[];
			
			
			Matrix4d rotation=	new Matrix4d(elemenents);
			double xpos = position.getX();

			cutouts.add(getAppendageMount()
				.transformed(new Transform(rotation))

				);
			CSG attachment = getAttachment()
				.transformed(new Transform(rotation))
			int thisMinz = attachment.getBounds().getMin().z;
			int thisMaxz = attachment.getBounds().getMax().z;
			if(thisMinz<minz)
				minz=thisMinz
			if(thisMaxz>maxz)
				maxz=thisMaxz

			int thisMiny = attachment.getBounds().getMin().y
			int thisMaxy = attachment.getBounds().getMax().y
			if(thisMiny<miny)
				miny=thisMiny
			if(thisMaxy>maxy)
				maxy=thisMaxy	

			int thisMinx = attachment.getBounds().getMin().x;
			int thisMaxx = attachment.getBounds().getMax().x;
			if(thisMinx<minx)
				minx=thisMinx
			if(thisMaxx>maxx)
				maxx=thisMaxx	
			attach.add(attachment);
			if(attachUnion==null){
				attachUnion=attachment;
			}else{
				attachUnion = 	attachUnion.union(attachment)
			}
			points.add(new Vector3d(position.getX(), position.getY()));
			
		}
		int heightOfBody=(maxz-minz);
		int widthOfBody=(maxx-minx);
		int depthOfBody=(maxy-miny);
		println "Height= "+ heightOfBody+ " widthOfBody= "+ widthOfBody+" depthOfBody= "+ depthOfBody
		CSG upperBody = attachUnion.hull()
		
		CSG dyioReference=   (CSG)(ScriptingEngine.inlineGistScriptRun("fb4cf429372deeb36f52", "dyioCad.groovy" , null))
						
		CSG myDyIO=dyioReference.movez(upperBody.getMaxZ()+22.0)
		upperBody=upperBody
		.union(myDyIO)
		.hull()
		.difference(myDyIO)
		for(CSG c:cutouts){
			upperBody= upperBody.difference(c);
			//allCad.add(c)
		}
		for(CSG c:attach){
			upperBody= upperBody.union(c);
			//allCad.add(c)
		}
		
		if(!printing){			
			upperBody.setColor(Color.CYAN);
			upperBody.setManipulator(base.getRootListener());
			//myDyIO.setManipulator(base.getRootListener());
			//allCad.add(myDyIO)
			
		}else{
			upperBody=upperBody
					.transformed(new Transform().rotX(180))
					.toZMin()
		}
		allCad.add(upperBody)
		
		
		return allCad;
	}
	ArrayList<CSG> generateBody(MobileBase base ,boolean toManufacture){
		
		ArrayList<CSG> allCad=new ArrayList<>();
		//Start by generating the legs using the DH link based generator
		/*
		for(DHParameterKinematics l:base.getAllDHChains()){
			for(CSG csg:generateCad(l.getChain().getLinks())){
				allCad.add(csg);
			}
		}
		*/
		try{
			//now we genrate the base pieces
			for(CSG csg:generateBodyParts( base ,toManufacture)){
				allCad.add(csg);
			}
		}catch (Exception ex){
			
		}
		return allCad;
	}
	
	public ArrayList<CSG> generateCad(DHParameterKinematics sourceLimb,boolean printBed ){
		//printBed=true;
		ArrayList<DHLink> dhLinks=sourceLimb.getChain().getLinks();
		ArrayList<CSG> csg = new ArrayList<CSG>();

		DHLink dh = dhLinks.get(0);
		
		CSG rootAttachment=getAttachment();
		//CSG rootAttachment=getAppendageMount();
		if(printBed){
			
			rootAttachment=rootAttachment 
			//.transformed(new Transform().rotY(90))
			.toZMin()
		}else{
			rootAttachment.setManipulator(dh.getRootListener());

		}
		
		csg.add(rootAttachment);//This is the root that attaches to the base
		rootAttachment.setColor(Color.GOLD);
		BowlerStudioController.addCsg(rootAttachment);
		CSG foot=getFoot();

		
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

		if(dhLinks!=null){
			for(int i=0;i<dhLinks.size();i++){
				dh = dhLinks.get(i);
				CSG nextAttachment=getAttachment();
				

				CSG servo=servoReference.transformed(new Transform().translateZ(-12.5))// allign to the horn
				.union(servoKeepaway)
				.transformed(new Transform().rotX(180))// allign to the horn
				.transformed(new Transform().rotZ(-90))// allign to the horn
				
				;
				servo= makeKeepaway(servo)
				double totalServoExtention =  Math.abs(servo.getMinY())
				
				boolean addNub=false;

				println "Servo Keepaway Radius = "+cylandarRadius
				
				double rOffsetForNextLinkStart=dh.getR()+mountScrewKeepawaySize+0.75;
				double rOffsetForNextLink=rOffsetForNextLinkStart;
				
				if(i==dhLinks.size()-1){
						 rOffsetForNextLink = rOffsetForNextLink-
						Math.abs(foot.toXMax().getBounds().getMin().x) 
					
				}else{
					rOffsetForNextLink = rOffsetForNextLink-
						Math.abs(nextAttachment.toXMax().getBounds().getMin().x)
					
				}
				println "Link # "+i+" offset = "+(rOffsetForNextLink)+" rad "+dh.getR() +" offset = "

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
							attachmentBaseWidth+3,
							9,
							attachmentBaseWidth+3
							).toCSG()
						)
					)
					.transformed(new Transform().translateY(rOffsetForNextLink))// allign to the NEXT ATTACHMENT
					.transformed(new Transform().translateZ(linkThickness))// allign to the NEXT ATTACHMENT
				
				double upperLinkZOffset = Math.abs(servoReference.getBounds().getMax().z-3)
				upperLink=upperLink.union(rod,clip,mountHoleAttachmentGroup).hull();
				upperLink= upperLink.difference(upperScrews);
				upperLink=upperLink.movez(upperLinkZOffset)

				upperLink= moveDHValues(upperLink,dh).difference(servo);
				if(i== dhLinks.size()-1)
					upperLink= upperLink.difference(makeKeepaway(foot));
				else
					upperLink= upperLink.difference(makeKeepaway(nextAttachment));
				
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
				CSG pinBlank = new Cylinder( bearingPinRadius+printerTollerence,
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
							rOffsetForNextLink,
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
				if(i== dhLinks.size()-1)
					lowerLink= lowerLink.difference(makeKeepaway(foot));
				else
					lowerLink= lowerLink.difference(makeKeepaway(nextAttachment));

				
				if(printBed){
					upperLink=reverseDHValues(upperLink,dh)
					.transformed(new Transform().rotY(180))
					.toZMin()
					
					lowerLink=reverseDHValues(lowerLink,dh)
					.transformed(new Transform().rotY(0))
					.toZMin()
					
					nextAttachment=nextAttachment
					//.transformed(new Transform().rotY(90))
					.toZMin()
					
				}else{
				
					nextAttachment.setManipulator(dh.getListener());
					nextAttachment.setColor(Color.CHOCOLATE);
					servo.setManipulator(dh.getListener());
					upperLink.setColor(Color.GREEN);
					upperLink.setManipulator(dh.getListener());
					
					
					lowerLink.setColor(Color.WHITE);
					lowerLink.setManipulator(dh.getListener());
					
					//csg.add(servo);// view the servo
					upperScrews= moveDHValues(upperScrews.movez(upperLinkZOffset),dh)
								
					upperScrews.setManipulator(dh.getListener());
					//csg.add(upperScrews);//view the screws
				}
				
				if(i<dhLinks.size()-1){
					csg.add(nextAttachment);//This is the root that attaches to the base
					BowlerStudioController.addCsg(nextAttachment);
				}
				
				csg.add(upperLink);//This is the root that attaches to the base
				csg.add(lowerLink);//White link forming the lower link
				BowlerStudioController.addCsg(upperLink);
				BowlerStudioController.addCsg(lowerLink);
				
					
			}
			if(printBed){
				
				foot=foot
				.transformed(new Transform().rotY(90))
				.toZMin()
			}else{
				foot.setManipulator(dhLinks.get(dhLinks.size()-1).getListener());
				
			}
			foot.setColor(Color.GOLD);
			csg.add(foot);//This is the root that attaches to the base
			BowlerStudioController.addCsg(foot);

		}
		return csg;
	}
};
