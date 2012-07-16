#ifndef MOSAIC_EVENT_HANDLER_H
#define MOSAIC_EVENT_HANDLER_H

#include "PlanarGraspSpec.h"
#include "MosaicManipulator.h"
#include "math.h"

class MosaicEventHandler : public osgGA::GUIEventHandler
{
private:
	PlanarGraspSpec* _spec;
	MainWindow* _window;
	ViewerWidget* _viewer;
	osg::Vec3 point1,point2,point3,point4, point5, point6, center, pointer;
	osg::Vec3 point1_w, point2_w, point3_w, point5_w, point6_w;
	float angle;
	int points;
    

public:
    MosaicEventHandler(PlanarGraspSpec* spec, MainWindow* window, ViewerWidget* viewer):_spec(spec),_window(window),_viewer(viewer)
    {
		points=0;
	}

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
    {
       if ( ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButtonMask() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON && (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) != 0  )
       {
		   
		    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
			float x = ea.getX();
			float y = ea.getY();
			osg::ref_ptr< osgUtil::LineSegmentIntersector> picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, x, y);
			osgUtil::IntersectionVisitor iv(picker.get());
			view->getCamera()->accept(iv);
			if (picker->containsIntersections())
			{
				osgUtil::LineSegmentIntersector::Intersections intersections;
				intersections = picker->getIntersections();
				if (view->computeIntersections(x,y,intersections))
				{

					
				if(points==0){
					MosaicManipulator *manipulator=dynamic_cast<MosaicManipulator*> (view->getCameraManipulator());
					manipulator->disableManipulator(true);
					point1_w=osg::Vec3(x,y,0);
					point1=osg::Vec3(intersections.begin()->getWorldIntersectPoint()[0],intersections.begin()->getWorldIntersectPoint()[1],1.1);
					_spec->addPoint(point1);
				}
				if(points==1){
					point2_w=osg::Vec3(x,y,0);
					point2=osg::Vec3(intersections.begin()->getWorldIntersectPoint()[0],intersections.begin()->getWorldIntersectPoint()[1],1.1);
					_spec->addPoint(point2);
				}
				if(points==2){
					point3_w=osg::Vec3(x,y,0);
					point3=osg::Vec3(intersections.begin()->getWorldIntersectPoint()[0],intersections.begin()->getWorldIntersectPoint()[1],1.1);
					_spec->addPoint(point3);
					 point4=osg::Vec3(point3[0]+point1[0]-point2[0], point3[1]+point1[1]-point2[1],1.1);
					_spec->addLine(point1, point2);
					_spec->addLine(point2, point3);
					_spec->addLine(point3, point4);
					_spec->addLine(point4, point1);
				}
				if(points==3){
					point5_w=osg::Vec3(x,y,0);
					point5=osg::Vec3(intersections.begin()->getWorldIntersectPoint()[0],intersections.begin()->getWorldIntersectPoint()[1],1.1);
					_spec->addPoint(point5);
				}
				if(points==4){
					point6_w=osg::Vec3(x,y,0);
					point6=osg::Vec3(intersections.begin()->getWorldIntersectPoint()[0],intersections.begin()->getWorldIntersectPoint()[1],1.1);
					_spec->addPoint(point6);
					_spec->addLine(point5,point6);
					center=osg::Vec3((point5[0]+point6[0])/2, (point5[1]+point6[1])/2,1.1);
				}
				if(points==5){
					QFileDialog *dialog=new QFileDialog(_window, "Save Target Template", ".", "Image (*.png)");
					dialog->setAcceptMode(QFileDialog::AcceptSave);
					if(dialog->exec()){
						QFileDialog *dialog_txt=new QFileDialog(_window, "Save Points Data", ".", "Text (*.txt)");
							dialog_txt->setAcceptMode(QFileDialog::AcceptSave);
							if(dialog_txt->exec()){
								points=-1;
								_spec->deleteLines();
								dialog->setDefaultSuffix("png");
								QStringList target_file=dialog->selectedFiles();
								dialog_txt->setDefaultSuffix("txt");
								QStringList points_file=dialog_txt->selectedFiles();
								osgViewer::ScreenCaptureHandler::ScreenCaptureHandler::WriteToFile *operation=new osgViewer::ScreenCaptureHandler::ScreenCaptureHandler::WriteToFile("/tmp/target_aux","png");
								osgViewer::ScreenCaptureHandler *capture=new osgViewer::ScreenCaptureHandler(operation);
								capture->captureNextFrame(*view->getViewerBase());
								_viewer->frame();
								
								QString arg="cp /tmp/target_aux_1_0.png ";
								arg.append(target_file[0]);
								QProcess *process;
								process->execute(arg);
								arg="rm /tmp/target_aux_1_0.png";
								process->execute(arg);
								
								QFile file(points_file[0]);
								file.open(QIODevice::WriteOnly | QIODevice::Text);
								QTextStream out(&file);
								out<<point1_w[0]<<","<<point1_w[1]<<endl;
								out<<point2_w[0]<<","<<point2_w[1]<<endl;
								out<<point3_w[0]<<","<<point3_w[1]<<endl;
								out<<point5_w[0]<<","<<point5_w[1]<<endl;
								out<<point6_w[0]<<","<<point6_w[1]<<endl;
								out<<angle<<endl;
								file.close();
								MosaicManipulator *manipulator=dynamic_cast<MosaicManipulator*> (view->getCameraManipulator());
								manipulator->disableManipulator(false);
								
							}
						}
				}
				if(points!=5){
					points++;
				}
				
								
					
				}	

			}
		}	
		if (ea.getKey()==osgGA::GUIEventAdapter::KEY_E ){
			osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
			MosaicManipulator *manipulator=dynamic_cast<MosaicManipulator*> (view->getCameraManipulator());
			manipulator->disableManipulator(false);
			points=0;
			_spec->deleteLines();
		}
			
		if(points==5){	
			osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
			float x = ea.getX();
			float y = ea.getY();
			osg::ref_ptr< osgUtil::LineSegmentIntersector> picker = new osgUtil::LineSegmentIntersector(osgUtil::Intersector::WINDOW, x, y);
			osgUtil::IntersectionVisitor iv(picker.get());
			view->getCamera()->accept(iv);
			if (picker->containsIntersections())
			{
				osgUtil::LineSegmentIntersector::Intersections intersections;
				intersections = picker->getIntersections();
				if (view->computeIntersections(x,y,intersections))
				{
					pointer=osg::Vec3(intersections.begin()->getWorldIntersectPoint()[0],intersections.begin()->getWorldIntersectPoint()[1],1.1);
					_spec->drawAxis(center,pointer);
					float a=sqrt(pow(point6[0]-center[0],2)+pow(point6[1]-center[1],2));
					float b=sqrt(pow(pointer[0]-center[0],2)+pow(pointer[1]-center[1],2));
					if(a==0 || b==0){
						angle=0;
					}
					else{
						angle=acos(((point6[0]-center[0])*(pointer[0]-center[0])+(point6[1]-center[1])*(pointer[1]-center[1]))/(a*b));
						if(angle<M_PI)
							angle=-angle;
						else{
							angle=(2*M_PI)-angle;
						}
					}
				}
			}
		}
					
			
		return false;
	
	}

};


#endif
