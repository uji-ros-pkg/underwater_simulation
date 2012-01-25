#ifndef UWSIMUTILS_H
#define UWSIMUTILS_H

#include <osg/NodeVisitor>
#include <osg/Node>

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

#endif

