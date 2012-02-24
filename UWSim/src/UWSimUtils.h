#ifndef UWSIMUTILS_H
#define UWSIMUTILS_H

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osg/Timer>

#include <iostream>
#include <vector>

typedef std::vector<osg::Node*> nodeListType; 

class findNodeVisitor : public osg::NodeVisitor { 
public: 

   findNodeVisitor(); 

   findNodeVisitor(const std::string &searchName) ;

   virtual void apply(osg::Node &searchNode);

   void setNameToFind(const std::string &searchName);

   osg::Node* getFirst();

   

   nodeListType& getNodeList() { return foundNodeList; }

private: 

   std::string searchForName; 
   nodeListType foundNodeList; 

}; 

class findRoutedNode {
public:

  findRoutedNode();

  findRoutedNode(const std::string &searchName) ;
  void setNameToFind(const std::string &searchName);
  void find(osg::ref_ptr<osg::Node> searchNode);
  osg::Node* getFirst();

private:

  findNodeVisitor nodeVisitor;	
  std::string searchRoute;
  nodeListType rootList;


};


class ScopedTimer
{
public:
    ScopedTimer(const std::string& description,
                std::ostream& output_stream = std::cout,
                bool endline_after_time = true)
        : _output_stream(output_stream)
        , _start()
        , _endline_after_time(endline_after_time)
    {   
        _output_stream << description << std::flush;
        _start = osg::Timer::instance()->tick();
    }   

    ~ScopedTimer()
    {   
        osg::Timer_t end = osg::Timer::instance()->tick();
        _output_stream << osg::Timer::instance()->delta_s(_start, end) << "s";
        if (_endline_after_time) _output_stream << std::endl;
        else                     _output_stream << std::flush;
    }   

private:
    std::ostream& _output_stream;
    osg::Timer_t _start;
    bool _endline_after_time;
};


#endif

