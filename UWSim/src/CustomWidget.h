#ifndef CUSTOMWIDGET
#define CUSTOMWIDGET

#include "SimulatorConfig.h"

#include <osgWidget/Util>
#include <osgWidget/WindowManager>
#include <osgWidget/Box>

class CustomWidget {
  public:
    CustomWidget() {}

    virtual osg::ref_ptr<osgWidget::Window> getWidgetWindow()=0;

    ~CustomWidget() {}
};

#endif

